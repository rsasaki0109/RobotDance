"""GMR (General Motion Retargeting) backend — RD-MIR → robot motion via external OSS.

GMR expects SMPL-X-style body dicts per frame. We convert canonical RD-MIR keypoints into that
format, run GMR's mink IK, then read MuJoCo body positions back into canonical robot keypoints.

Requires a full GMR install with assets (pip install from the cloned repo, not PyPI wheel only):

    git clone https://github.com/YanjieZe/GMR.git
    pip install -e GMR/

Optional: ``ROBOTDANCE_GMR_ROOT`` points at the clone if the package is on PYTHONPATH without assets.
"""

from __future__ import annotations

import importlib.util
import os
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation as R

from robotdance_core.rd_mir import RdMir, Skeleton
from robotdance_core.rd_motion import RdMotion
from robotdance_core.skeleton import FOOT_JOINTS, JOINT_NAMES, NUM_JOINTS, PARENTS, index_of
from robotdance_retarget.embodiment import RobotMorphology
from robotdance_retarget.kinematic import _joint_flexion_metrics, _retarget_metrics

_EPS = 1e-8

# RobotDance robot name → GMR ``tgt_robot`` id.
ROBOT_TO_GMR: dict[str, str] = {
    "unitree_g1": "unitree_g1",
    "unitree_h1": "unitree_h1",
    "unitree_h2": "unitree_h1_2",
    "booster_t1": "booster_t1",
    "fourier_n1": "fourier_n1",
}

# Canonical joint → GMR smplx IK body name (positions + orientations).
_CANONICAL_TO_GMR_BODY: dict[str, str] = {
    "pelvis": "pelvis",
    "chest": "spine3",
    "left_hip": "left_hip",
    "right_hip": "right_hip",
    "left_knee": "left_knee",
    "right_knee": "right_knee",
    "left_foot": "left_foot",
    "right_foot": "right_foot",
    "left_shoulder": "left_shoulder",
    "right_shoulder": "right_shoulder",
    "left_elbow": "left_elbow",
    "right_elbow": "right_elbow",
    "left_wrist": "left_wrist",
    "right_wrist": "right_wrist",
}

# GMR MuJoCo body → canonical joint (per GMR robot id). Unlisted joints are interpolated.
_GMR_BODY_TO_CANONICAL: dict[str, dict[str, str]] = {
    "unitree_g1": {
        "pelvis": "pelvis",
        "torso_link": "chest",
        "left_hip_roll_link": "left_hip",
        "right_hip_roll_link": "right_hip",
        "left_knee_link": "left_knee",
        "right_knee_link": "right_knee",
        "left_toe_link": "left_foot",
        "right_toe_link": "right_foot",
        "left_shoulder_yaw_link": "left_shoulder",
        "right_shoulder_yaw_link": "right_shoulder",
        "left_elbow_link": "left_elbow",
        "right_elbow_link": "right_elbow",
        "left_wrist_yaw_link": "left_wrist",
        "right_wrist_yaw_link": "right_wrist",
    },
    "unitree_h1": {
        "pelvis": "pelvis",
        "torso_link": "chest",
        "left_hip_yaw_link": "left_hip",
        "right_hip_yaw_link": "right_hip",
        "left_knee_link": "left_knee",
        "right_knee_link": "right_knee",
        "left_ankle_link": "left_ankle",
        "right_ankle_link": "right_ankle",
        "left_shoulder_pitch_link": "left_shoulder",
        "right_shoulder_pitch_link": "right_shoulder",
        "left_elbow_link": "left_elbow",
        "right_elbow_link": "right_elbow",
        "left_wrist_link": "left_wrist",
        "right_wrist_link": "right_wrist",
    },
    "unitree_h1_2": {
        "pelvis": "pelvis",
        "torso_link": "chest",
        "left_hip_roll_link": "left_hip",
        "right_hip_roll_link": "right_hip",
        "left_knee_link": "left_knee",
        "right_knee_link": "right_knee",
        "left_ankle_pitch_link": "left_ankle",
        "right_ankle_pitch_link": "right_ankle",
        "left_shoulder_pitch_link": "left_shoulder",
        "right_shoulder_pitch_link": "right_shoulder",
        "left_elbow_link": "left_elbow",
        "right_elbow_link": "right_elbow",
        "left_wrist_link": "left_wrist",
        "right_wrist_link": "right_wrist",
    },
    "booster_t1": {
        "Waist": "pelvis",
        "Trunk": "chest",
        "Left_Hip": "left_hip",
        "Right_Hip": "right_hip",
        "Left_Knee": "left_knee",
        "Right_Knee": "right_knee",
        "Left_Ankle": "left_ankle",
        "Right_Ankle": "right_ankle",
        "Left_Shoulder": "left_shoulder",
        "Right_Shoulder": "right_shoulder",
        "Left_Elbow": "left_elbow",
        "Right_Elbow": "right_elbow",
        "Left_Wrist": "left_wrist",
        "Right_Wrist": "right_wrist",
    },
    "fourier_n1": {
        "base_link": "pelvis",
        "torso_link": "chest",
        "left_hip_pitch_link": "left_hip",
        "right_hip_pitch_link": "right_hip",
        "left_knee_link": "left_knee",
        "right_knee_link": "right_knee",
        "left_ankle_pitch_link": "left_ankle",
        "right_ankle_pitch_link": "right_ankle",
        "left_shoulder_pitch_link": "left_shoulder",
        "right_shoulder_pitch_link": "right_shoulder",
        "left_elbow_link": "left_elbow",
        "right_elbow_link": "right_elbow",
        "left_wrist_yaw_link": "left_wrist",
        "right_wrist_yaw_link": "right_wrist",
    },
}


def gmr_importable() -> bool:
    return (
        importlib.util.find_spec("general_motion_retargeting") is not None
        and importlib.util.find_spec("mink") is not None
    )


def gmr_assets_available() -> bool:
    """True when GMR robot XML assets are on disk (editable clone, not bare PyPI wheel)."""
    if not gmr_importable():
        return False
    root = os.environ.get("ROBOTDANCE_GMR_ROOT")
    if root:
        probe = Path(root) / "assets" / "unitree_g1" / "g1_mocap_29dof.xml"
        return probe.is_file()
    try:
        from general_motion_retargeting.params import ROBOT_XML_DICT

        return Path(ROBOT_XML_DICT["unitree_g1"]).is_file()
    except Exception:
        return False


def gmr_available() -> bool:
    return gmr_importable() and gmr_assets_available()


def gmr_install_hint() -> str:
    return (
        "GMR が未導入または assets がありません。clone から editable install してください:\n"
        "  git clone https://github.com/YanjieZe/GMR.git\n"
        "  pip install -e GMR/\n"
        "（mink, mujoco, scipy も必要）"
    )


def _height_frame(kps: np.ndarray) -> float:
    return float(kps[:, 2].max() - kps[:, 2].min())


def _quat_from_axes(x: np.ndarray, y: np.ndarray, z: np.ndarray) -> np.ndarray:
    mat = np.column_stack([x, y, z])
    return R.from_matrix(mat).as_quat(scalar_first=True)


def _quat_align_z_to_vec(vec: np.ndarray) -> np.ndarray:
    z = np.array([0.0, 0.0, 1.0])
    v = vec / max(np.linalg.norm(vec), _EPS)
    if np.allclose(v, z):
        return np.array([1.0, 0.0, 0.0, 0.0])
    if np.allclose(v, -z):
        return np.array([0.0, 1.0, 0.0, 0.0])
    axis = np.cross(z, v)
    axis /= max(np.linalg.norm(axis), _EPS)
    angle = float(np.arccos(np.clip(z @ v, -1.0, 1.0)))
    return R.from_rotvec(axis * angle).as_quat(scalar_first=True)


def canonical_frame_to_gmr(kps: np.ndarray) -> dict[str, tuple[np.ndarray, np.ndarray]]:
    """Single frame canonical keypoints [J,3] → GMR smplx-style body dict."""
    idx = {n: index_of(n) for n in JOINT_NAMES}
    pelvis = kps[idx["pelvis"]]
    chest = kps[idx["chest"]]
    lhip, rhip = kps[idx["left_hip"]], kps[idx["right_hip"]]

    y_axis = lhip - rhip
    y_axis /= max(np.linalg.norm(y_axis), _EPS)
    z_axis = chest - pelvis
    z_axis /= max(np.linalg.norm(z_axis), _EPS)
    x_axis = np.cross(y_axis, z_axis)
    x_axis /= max(np.linalg.norm(x_axis), _EPS)
    y_axis = np.cross(z_axis, x_axis)
    pelvis_quat = _quat_from_axes(x_axis, y_axis, z_axis)

    out: dict[str, tuple[np.ndarray, np.ndarray]] = {}
    for canon, gmr_name in _CANONICAL_TO_GMR_BODY.items():
        pos = kps[idx[canon]].copy()
        parent = PARENTS[idx[canon]]
        child = _child_index(idx[canon])
        if parent >= 0 and child is not None:
            bone = kps[child] - kps[idx[canon]]
            quat = _quat_align_z_to_vec(bone)
        elif canon == "pelvis":
            quat = pelvis_quat
        else:
            quat = np.array([1.0, 0.0, 0.0, 0.0])
        out[gmr_name] = (pos, quat)
    return out


def _child_index(j: int) -> int | None:
    children = [c for c, p in enumerate(PARENTS) if p == j]
    if not children:
        return None
    return children[0]


def _fill_missing_joints(kps: np.ndarray) -> np.ndarray:
    """Interpolate spine/neck/head/toe from mapped bodies."""
    out = kps.copy()
    idx = {n: index_of(n) for n in JOINT_NAMES}

    if np.allclose(out[idx["spine"]], 0):
        out[idx["spine"]] = 0.33 * out[idx["pelvis"]] + 0.67 * out[idx["chest"]]
    if np.allclose(out[idx["neck"]], 0):
        out[idx["neck"]] = 0.75 * out[idx["chest"]] + 0.25 * out[idx["head"]]
    if np.allclose(out[idx["head"]], 0):
        head_dir = out[idx["chest"]] - out[idx["pelvis"]]
        head_dir /= max(np.linalg.norm(head_dir), _EPS)
        out[idx["head"]] = out[idx["chest"]] + 0.12 * head_dir

    for side in ("left", "right"):
        ankle_i = idx[f"{side}_ankle"]
        foot_i = idx[f"{side}_foot"]
        knee_i = idx[f"{side}_knee"]
        if np.allclose(out[ankle_i], 0) and not np.allclose(out[foot_i], 0):
            out[ankle_i] = 0.65 * out[knee_i] + 0.35 * out[foot_i]
        if np.allclose(out[foot_i], 0) and not np.allclose(out[ankle_i], 0):
            foot_dir = out[ankle_i] - out[knee_i]
            foot_dir /= max(np.linalg.norm(foot_dir), _EPS)
            out[foot_i] = out[ankle_i] + 0.05 * foot_dir
    return out


def _qpos_to_keypoints(model, qpos: np.ndarray, gmr_robot: str) -> np.ndarray:
    import mujoco

    body_map = _GMR_BODY_TO_CANONICAL.get(gmr_robot, {})
    data = mujoco.MjData(model)
    data.qpos[:] = qpos
    mujoco.mj_forward(model, data)

    kps = np.zeros((NUM_JOINTS, 3), dtype=np.float64)
    for gmr_body, canon in body_map.items():
        try:
            bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, gmr_body)
        except ValueError:
            continue
        kps[index_of(canon)] = data.xpos[bid].copy()
    return _fill_missing_joints(kps)


def gmr_retarget(
    mir: RdMir,
    morphology: RobotMorphology,
    *,
    verbose: bool = False,
    offset_to_ground: bool = True,
) -> RdMotion:
    """RD-MIR → GMR IK → RD-Motion (canonical robot keypoints)."""
    if not gmr_available():
        raise RuntimeError(gmr_install_hint())

    gmr_robot = ROBOT_TO_GMR.get(morphology.name)
    if gmr_robot is None:
        supported = ", ".join(sorted(ROBOT_TO_GMR))
        raise ValueError(
            f"GMR backend は {morphology.name} 未対応。対応: {supported}"
        )

    from general_motion_retargeting import GeneralMotionRetargeting as GMR

    human = mir.keypoints_3d_array()
    if human.shape[1] != NUM_JOINTS:
        raise ValueError(f"想定 joint 数 {NUM_JOINTS} と不一致: {human.shape[1]}")

    human_h = float(np.median([_height_frame(human[f]) for f in range(human.shape[0])]))
    retargeter = GMR(
        src_human="smplx",
        tgt_robot=gmr_robot,
        actual_human_height=human_h,
        verbose=verbose,
    )

    robot_frames = []
    for f in range(human.shape[0]):
        human_data = canonical_frame_to_gmr(human[f])
        qpos = retargeter.retarget(human_data, offset_to_ground=offset_to_ground)
        robot_frames.append(_qpos_to_keypoints(retargeter.model, qpos, gmr_robot))
    robot = np.stack(robot_frames, axis=0)

    # Ground clamp for toe/ankle consistency with builtin metrics.
    ground = 0.03
    foot_indices = [idx for pair in FOOT_JOINTS.values() for idx in pair]
    min_z = robot[:, foot_indices, 2].min(axis=1)
    robot[:, :, 2] += (ground - min_z)[:, None]

    height_scale = morphology.nominal_height / max(human_h, _EPS)
    contacts = mir.contacts or {}
    metrics = _retarget_metrics(human, robot, contacts, height_scale)
    metrics["backend"] = "gmr"
    metrics["gmr_robot"] = gmr_robot
    metrics["method"] = "GMR mink IK (external)"
    flexion = _joint_flexion_metrics(robot, morphology)
    if flexion is not None:
        metrics["joint_flexion"] = flexion

    return RdMotion(
        robot_name=morphology.name,
        fps=mir.fps,
        duration=mir.duration,
        source_motion_id=mir.motion_id,
        skeleton=Skeleton(joint_names=list(JOINT_NAMES), parents=list(PARENTS)),
        control_mode="kinematic_preview",
        keypoints_3d=robot.tolist(),
        base_trajectory={"position": robot[:, 0, :].tolist()},
        contact_schedule={k: list(v) for k, v in contacts.items()},
        retarget_metrics=metrics,
        sim_certificate=None,
        source_provenance={
            "rd_mir_motion_id": mir.motion_id,
            "method": "gmr",
            "gmr_robot": gmr_robot,
        },
    )


__all__ = [
    "ROBOT_TO_GMR",
    "canonical_frame_to_gmr",
    "gmr_assets_available",
    "gmr_available",
    "gmr_importable",
    "gmr_install_hint",
    "gmr_retarget",
]
