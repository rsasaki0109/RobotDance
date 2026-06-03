"""SMPL/SMPL-H body skeleton の forward kinematics（skeleton-first, v0）。

AMASS 等は SMPL の axis-angle pose を保持する。SMPL の **body mesh / model file（商用・再配布
制限あり）は使わず**、公開された骨格構造 + 近似 rest offset だけで FK して joint 位置を得る。
retarget は direction-preserving なので rest 長が近似でも下流に影響しない（skeleton-first 方針）。

⚠️ rest offset は近似値。SMPL の正確な shape-conditioned joint regressor は使っていない。
"""

from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation as Rot

from robotdance_core.skeleton import NUM_JOINTS, index_of

# SMPL body 22 joint（SMPL-H/-X も body は先頭 22）。index: name。
SMPL_BODY_JOINTS = [
    "pelvis", "l_hip", "r_hip", "spine1", "l_knee", "r_knee", "spine2",
    "l_ankle", "r_ankle", "spine3", "l_foot", "r_foot", "neck", "l_collar",
    "r_collar", "head", "l_shoulder", "r_shoulder", "l_elbow", "r_elbow",
    "l_wrist", "r_wrist",
]
SMPL_PARENTS = [-1, 0, 0, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 9, 9, 12, 13, 14, 16, 17, 18, 19]

# 近似 rest offset（親→子, SMPL frame: x=左, y=上, z=前, 単位 m）。
_SMPL_OFFSETS = np.array(
    [
        [0.00, 0.00, 0.00],    # pelvis (root)
        [0.08, -0.06, 0.00],   # l_hip
        [-0.08, -0.06, 0.00],  # r_hip
        [0.00, 0.12, 0.00],    # spine1
        [0.00, -0.40, 0.00],   # l_knee
        [0.00, -0.40, 0.00],   # r_knee
        [0.00, 0.13, 0.00],    # spine2
        [0.00, -0.40, 0.00],   # l_ankle
        [0.00, -0.40, 0.00],   # r_ankle
        [0.00, 0.05, 0.00],    # spine3
        [0.00, -0.06, 0.12],   # l_foot
        [0.00, -0.06, 0.12],   # r_foot
        [0.00, 0.21, 0.00],    # neck
        [0.06, 0.10, 0.00],    # l_collar
        [-0.06, 0.10, 0.00],   # r_collar
        [0.00, 0.12, 0.00],    # head
        [0.12, 0.02, 0.00],    # l_shoulder
        [-0.12, 0.02, 0.00],   # r_shoulder
        [0.26, 0.00, 0.00],    # l_elbow
        [-0.26, 0.00, 0.00],   # r_elbow
        [0.24, 0.00, 0.00],    # l_wrist
        [-0.24, 0.00, 0.00],   # r_wrist
    ],
    dtype=np.float64,
)

# SMPL joint → canonical 19-joint の対応。
_SMPL_TO_CANONICAL = {
    "pelvis": "pelvis", "spine1": "spine", "spine3": "chest", "neck": "neck", "head": "head",
    "l_shoulder": "left_shoulder", "r_shoulder": "right_shoulder",
    "l_elbow": "left_elbow", "r_elbow": "right_elbow",
    "l_wrist": "left_wrist", "r_wrist": "right_wrist",
    "l_hip": "left_hip", "r_hip": "right_hip",
    "l_knee": "left_knee", "r_knee": "right_knee",
    "l_ankle": "left_ankle", "r_ankle": "right_ankle",
    "l_foot": "left_foot", "r_foot": "right_foot",
}


def _smpl_to_canonical_frame(pos: np.ndarray) -> np.ndarray:
    """SMPL frame(x=左,y=上,z=前) → canonical(x=前,y=左,z=上): (sx,sy,sz)→(sz,sx,sy)。"""
    return pos[..., [2, 0, 1]]


def fk_smpl_body(poses: np.ndarray, trans: np.ndarray | None = None) -> np.ndarray:
    """SMPL body axis-angle poses [T, 22, 3]（+ root trans [T,3]）→ joint 位置 [T, 22, 3]（SMPL frame）。"""
    n = poses.shape[0]
    out = np.zeros((n, 22, 3))
    for f in range(n):
        world_rot: list[Rot] = [Rot.identity()] * 22
        pos = np.zeros((22, 3))
        for j, parent in enumerate(SMPL_PARENTS):
            local = Rot.from_rotvec(poses[f, j])
            if parent < 0:
                world_rot[j] = local
                pos[j] = trans[f] if trans is not None else np.zeros(3)
            else:
                world_rot[j] = world_rot[parent] * local
                pos[j] = pos[parent] + world_rot[parent].apply(_SMPL_OFFSETS[j])
        out[f] = pos
    return out


def smpl_poses_to_canonical(poses: np.ndarray, trans: np.ndarray | None = None) -> np.ndarray:
    """SMPL body poses [T, 22, 3] → canonical 19-joint keypoints [T, 19, 3]（z-up）。"""
    smpl_pos = _smpl_to_canonical_frame(fk_smpl_body(poses, trans))  # [T,22,3] canonical frame
    return _reindex_smpl_to_canonical(smpl_pos)


def smpl_joints_to_canonical(joints: np.ndarray) -> np.ndarray:
    """SMPL body の **joint 位置** [T, 22, 3]（SMPL frame）→ canonical 19-joint [T, 19, 3]。

    HumanML3D 等は axis-angle pose ではなく FK 済みの joint 位置を配布する。本関数はそれを
    canonical frame に変換し 19-joint へ再マップする（pose→canonical と同じ写像の位置版）。
    """
    pos = _smpl_to_canonical_frame(np.asarray(joints, dtype=np.float64))  # [T,22,3] canonical frame
    return _reindex_smpl_to_canonical(pos)


def _reindex_smpl_to_canonical(smpl_pos: np.ndarray) -> np.ndarray:
    """canonical frame の SMPL 22-joint [T,22,3] → canonical 19-joint [T,19,3] へ再マップ。"""
    name_to_idx = {n: i for i, n in enumerate(SMPL_BODY_JOINTS)}
    out = np.zeros((smpl_pos.shape[0], NUM_JOINTS, 3))
    for smpl_name, canon_name in _SMPL_TO_CANONICAL.items():
        out[:, index_of(canon_name)] = smpl_pos[:, name_to_idx[smpl_name]]
    return out
