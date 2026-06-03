"""Kinematic retargeting（v0）。

人間 canonical motion（RD-MIR）を任意の robot embodiment（RobotMorphology）へ写像する。
手法は direction-preserving + morphology normalization:

  1. 人間 keypoints から各 bone の単位方向を取る
  2. robot の bone 長でルートから FK 再構成（bone 長は robot のもの → morphology normalization）
  3. robot を接地クランプ（足が地面を貫かない / 浮きすぎない）

⚠️ これは運動学のみ。物理 sim（転倒・トルク・滑り）は通していないため実機 feasibility は未保証。
sim 検証は Phase 2 で sim_certificate に記録する。
"""

from __future__ import annotations

import numpy as np

from robotdance_core.rd_mir import RdMir, Skeleton
from robotdance_core.rd_motion import RdMotion
from robotdance_core.skeleton import FOOT_JOINTS, JOINT_NAMES, NUM_JOINTS, PARENTS
from robotdance_retarget.embodiment import RobotMorphology

_EPS = 1e-8


def _bone_directions(kps: np.ndarray) -> np.ndarray:
    """[T, J, 3] keypoints → [T, J, 3] の親→子 単位方向（root は 0）。"""
    parent_idx = np.array([max(p, 0) for p in PARENTS])
    vec = kps - kps[:, parent_idx, :]
    norm = np.linalg.norm(vec, axis=2, keepdims=True)
    dirs = vec / np.maximum(norm, _EPS)
    dirs[:, [j for j, p in enumerate(PARENTS) if p < 0], :] = 0.0
    return dirs


def _height(kps_frame: np.ndarray) -> float:
    return float(kps_frame[:, 2].max() - kps_frame[:, 2].min())


def retarget(mir: RdMir, morphology: RobotMorphology) -> RdMotion:
    """RD-MIR を任意 robot 形態へ kinematic retarget して RD-Motion を返す。"""
    human = mir.keypoints_3d_array()  # [T, J, 3]
    n_frames = human.shape[0]
    if human.shape[1] != NUM_JOINTS:
        raise ValueError(f"想定 joint 数 {NUM_JOINTS} と不一致: {human.shape[1]}")

    dirs = _bone_directions(human)
    bone_len = morphology.bone_lengths

    # 人間 root の水平移動はそのまま、垂直は morphology に合わせて height 比でスケール。
    human_h = float(np.median([_height(human[f]) for f in range(n_frames)]))
    height_scale = morphology.nominal_height / max(human_h, _EPS)

    robot = np.zeros_like(human)
    for f in range(n_frames):
        # root: 水平は人間に追従、垂直はスケール（接地クランプで最終調整）。
        root = human[f, 0].copy()
        root[2] *= height_scale
        robot[f, 0] = root
        # FK: child = parent + dir * robot_bone_len（root から topological 順）。
        for j in range(1, NUM_JOINTS):
            robot[f, j] = robot[f, PARENTS[j]] + dirs[f, j] * bone_len[j]

    # 接地クランプ: 各フレームで最下端の足を地面（z=ground）に合わせる。
    ground = 0.03
    foot_indices = [idx for pair in FOOT_JOINTS.values() for idx in pair]
    min_z = robot[:, foot_indices, 2].min(axis=1)  # [T]
    robot[:, :, 2] += (ground - min_z)[:, None]

    contacts = mir.contacts or {}
    metrics = _retarget_metrics(human, robot, contacts, height_scale)

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
        sim_certificate=None,  # v0 kinematic: 物理検証なし
        source_provenance={"rd_mir_motion_id": mir.motion_id, "method": "direction_preserving_fk"},
    )


def retarget_to_g1(mir: RdMir) -> RdMotion:
    """RD-MIR を Unitree G1（v0 プロキシ）へ retarget する薄いラッパー。"""
    from robotdance_unitree import g1

    return retarget(mir, g1.MORPHOLOGY)


def _retarget_metrics(
    human: np.ndarray, robot: np.ndarray, contacts: dict, height_scale: float
) -> dict:
    """運動学リターゲットの正直な品質指標を計算する。"""
    # bone 方向が保たれているか（人間 vs robot の bone 単位ベクトルの cos 類似）。
    hd, rd = _bone_directions(human), _bone_directions(robot)
    cos = (hd * rd).sum(axis=2)  # [T, J]
    bone_mask = np.array([p >= 0 for p in PARENTS])
    direction_cos = float(cos[:, bone_mask].mean())

    # foot sliding: 接地中の足の水平移動量（小さいほど良い）。
    sliding = []
    for side, (ankle_idx, _toe) in FOOT_JOINTS.items():
        flag = np.asarray(contacts.get(f"{side}_foot", []), dtype=bool)
        if flag.size != robot.shape[0]:
            continue
        xy = robot[:, ankle_idx, :2]
        step = np.linalg.norm(np.diff(xy, axis=0), axis=1)
        in_contact = flag[1:] & flag[:-1]
        if in_contact.any():
            sliding.append(float(step[in_contact].mean()))
    foot_sliding = float(np.mean(sliding)) if sliding else None

    return {
        "method": "direction_preserving_fk + morphology_normalization + ground_clamp",
        "height_scale": round(height_scale, 4),
        "bone_direction_cosine": round(direction_cos, 4),
        "foot_sliding_m_per_frame": round(foot_sliding, 5) if foot_sliding is not None else None,
        "physically_validated": False,
        "note": "kinematic preview only — sim/torque/balance は未検証（Phase 2）",
    }
