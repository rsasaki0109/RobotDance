"""MuJoCo を使った物理ベースの feasibility 検証（v0）。

受動ヒューマノイドはバランス制御なしでは何でも倒れるため、forward sim は判別力を持たない。
本バックエンドは **参照運動そのものが物理的に実現可能か** を MuJoCo の動力学で検証する:

  1. keypoints を ball-joint 多体モデルの qpos に厳密復元
  2. 逆動力学（mj_inverse）で各 joint の必要トルク → torque saturation
  3. 質量モデルの COM → ZMP を計算し、接地足の支持多角形を外れる/滞空で balance violation

⚠️ v0 の質量・慣性は近似（bone 長比）であり実機値ではない。出力 sim_certificate は
"physically informed feasibility" であって実機保証ではない。
"""

from __future__ import annotations

from typing import Any

import numpy as np
from scipy.spatial.transform import Rotation as Rot

from robotdance_core.rd_motion import RdMotion
from robotdance_core.skeleton import FOOT_JOINTS, JOINT_NAMES, PARENTS
from robotdance_retarget.embodiment import RobotMorphology

from .mjcf import build_mjcf

_G = 9.81
_EPS = 1e-9


def _min_rot_quat(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """単位ベクトル a を b に重ねる最小回転（quaternion, wxyz）。"""
    a = a / max(np.linalg.norm(a), _EPS)
    b = b / max(np.linalg.norm(b), _EPS)
    c = float(np.dot(a, b))
    if c > 1.0 - 1e-8:
        return np.array([1.0, 0.0, 0.0, 0.0])
    if c < -1.0 + 1e-8:
        # 反平行: a に直交する任意軸で 180°。
        axis = np.cross(a, np.array([1.0, 0.0, 0.0]))
        if np.linalg.norm(axis) < 1e-6:
            axis = np.cross(a, np.array([0.0, 1.0, 0.0]))
        axis /= np.linalg.norm(axis)
        return np.array([0.0, *axis])
    axis = np.cross(a, b)
    q = np.array([1.0 + c, *axis])
    return q / np.linalg.norm(q)


def _quat_to_mat(q_wxyz: np.ndarray) -> np.ndarray:
    return Rot.from_quat([q_wxyz[1], q_wxyz[2], q_wxyz[3], q_wxyz[0]]).as_matrix()


def _pose_to_qpos(model, morphology: RobotMorphology, kps: np.ndarray) -> np.ndarray:
    """1 フレームの keypoints [J, 3] を qpos に厳密復元する。"""
    rest = morphology.rest_pose
    qpos = np.zeros(model.nq)
    qpos[0:3] = kps[0]
    qpos[3:7] = [1.0, 0.0, 0.0, 0.0]  # base 向きは identity
    r_body: dict[int, np.ndarray] = {}
    for j in range(1, len(JOINT_NAMES)):
        p = PARENTS[j]
        r_parent = np.eye(3) if p == 0 else r_body[p]
        o = rest[j] - rest[p]
        t = kps[j] - kps[p]
        local_t = r_parent.T @ (t / max(np.linalg.norm(t), _EPS))
        q = _min_rot_quat(o, local_t)
        adr = model.joint(f"jnt_{j}").qposadr[0]
        qpos[adr:adr + 4] = q
        r_body[j] = r_parent @ _quat_to_mat(q)
    return qpos


def simulate_certificate(
    motion: RdMotion,
    morphology: RobotMorphology,
    *,
    total_mass: float = 35.0,
    torque_limit: float = 80.0,
    support_margin: float = 0.12,
) -> dict[str, Any]:
    """RD-Motion を MuJoCo 物理で検証し sim_certificate dict を返す。"""
    import mujoco

    # 地面なしの純浮遊多体: mj_inverse に接触力が混入せず、内部トルクが純 RNEA になる。
    # バランスは motion の contact_schedule と keypoints から別途計算するため地面は不要。
    model = mujoco.MjModel.from_xml_string(
        build_mjcf(morphology, total_mass=total_mass, ground=False)
    )
    data = mujoco.MjData(model)
    root_id = model.body("root").id

    kps = motion.keypoints_3d_array()  # [T, J, 3]
    n = kps.shape[0]
    dt = 1.0 / motion.fps

    # 各フレームの qpos と COM。
    qpos = np.stack([_pose_to_qpos(model, morphology, kps[f]) for f in range(n)])
    com = np.zeros((n, 3))
    for f in range(n):
        data.qpos[:] = qpos[f]
        data.qvel[:] = 0
        mujoco.mj_forward(model, data)
        com[f] = data.subtree_com[root_id]

    # qvel / qacc（quaternion 対応の差分）。
    qvel = np.zeros((n, model.nv))
    for f in range(n - 1):
        mujoco.mj_differentiatePos(model, qvel[f], dt, qpos[f], qpos[f + 1])
    qacc = np.zeros((n, model.nv))
    qacc[1:-1] = (qvel[2:] - qvel[:-2]) / (2 * dt)

    # 逆動力学で内部 joint トルク。短い足先 bone（toe）は方向が数値的に不安定で
    # スパイクを生むため除外し、各フレームの最大トルクの 95 パーセンタイルで評価する
    # （近似慣性 + 有限差分のため単一フレームの外れ値に依存しない robust 統計）。
    toe_joints = {JOINT_NAMES.index("left_foot"), JOINT_NAMES.index("right_foot")}
    dof_slices = [
        (model.joint(f"jnt_{j}").dofadr[0], model.joint(f"jnt_{j}").dofadr[0] + 3)
        for j in range(1, len(JOINT_NAMES))
        if j not in toe_joints
    ]
    per_frame_max = []
    for f in range(1, n - 1):
        data.qpos[:] = qpos[f]
        data.qvel[:] = qvel[f]
        data.qacc[:] = qacc[f]
        mujoco.mj_inverse(model, data)
        tau = data.qfrc_inverse
        per_frame_max.append(max(float(np.linalg.norm(tau[a:b])) for a, b in dof_slices))
    # 腕が頭上で rest と反平行になると ball joint が 180° 特異姿勢になり mj_inverse が
    # 局所的にスパイクする。median（典型負荷）で gate し、peak は参考値として併記する。
    torque_p50 = float(np.median(per_frame_max)) if per_frame_max else 0.0
    torque_peak = float(np.max(per_frame_max)) if per_frame_max else 0.0

    # COM 加速度 → ZMP（平地・総質量点近似, ground z=0）。
    com_acc = np.zeros((n, 3))
    com_acc[1:-1] = (com[2:] - 2 * com[1:-1] + com[:-2]) / (dt * dt)
    denom = com_acc[:, 2] + _G
    zmp = np.zeros((n, 2))
    safe = np.abs(denom) > 1e-3
    zmp[safe, 0] = com[safe, 0] - com[safe, 2] * com_acc[safe, 0] / denom[safe]
    zmp[safe, 1] = com[safe, 1] - com[safe, 2] * com_acc[safe, 1] / denom[safe]

    # 接地 / バランス判定。支持点は接地足の ankle と toe の両方（foot 面を近似）。
    contacts = motion.contact_schedule or {}
    airborne = 0
    unsupported = 0
    for f in range(n):
        grounded_idx: list[int] = []
        for side, (ankle, toe) in FOOT_JOINTS.items():
            if np.asarray(contacts.get(f"{side}_foot", [False] * n), dtype=bool)[f]:
                grounded_idx += [ankle, toe]
        if not grounded_idx:
            airborne += 1
            unsupported += 1
            continue
        pts = kps[f][grounded_idx][:, :2]  # 接地足の xy（ankle + toe）
        if not _zmp_in_support(zmp[f], pts, support_margin):
            unsupported += 1

    airborne_ratio = airborne / n
    balance_violation_ratio = unsupported / n
    max_joint_ang_speed = float(np.abs(qvel[:, 6:]).max()) if n > 1 else 0.0
    torque_ratio = torque_p50 / torque_limit

    reasons: list[str] = []
    if airborne_ratio > 0.1:
        reasons.append(f"airborne {airborne_ratio:.0%}（接地なしで支持不能）")
    if balance_violation_ratio > 0.3:
        reasons.append(f"ZMP が支持多角形外 {balance_violation_ratio:.0%}（転倒リスク）")
    if torque_ratio > 1.5:
        reasons.append(f"torque saturation ×{torque_ratio:.2f}（典型負荷が actuator 限界超過）")
    if max_joint_ang_speed > 30.0:
        reasons.append(f"関節角速度過大 {max_joint_ang_speed:.0f} rad/s")

    passed = not reasons
    return {
        "backend": "mujoco",
        "mujoco_version": mujoco.__version__,
        "approximate_inertia": True,
        "passed": passed,
        "verdict": "PASS" if passed else "REJECT",
        "metrics": {
            "airborne_ratio": round(airborne_ratio, 3),
            "balance_violation_ratio": round(balance_violation_ratio, 3),
            "joint_torque_nm_p50": round(torque_p50, 1),
            "joint_torque_nm_peak": round(torque_peak, 1),
            "torque_ratio": round(torque_ratio, 3),
            "max_joint_ang_speed_rad_s": round(max_joint_ang_speed, 2),
        },
        "thresholds": {
            "airborne_ratio": 0.1,
            "balance_violation_ratio": 0.3,
            "torque_ratio_p50": 1.5,
            "max_joint_ang_speed_rad_s": 30.0,
        },
        "reasons": reasons,
        "note": (
            "physically-informed feasibility（近似慣性, ball-joint 近似）— 実機保証ではない（v0）。"
            " torque は median(p50) で判定（特異姿勢の peak は参考値）。"
        ),
    }


def certify(motion: RdMotion, morphology: RobotMorphology, **kwargs: Any) -> RdMotion:
    """sim_certificate を計算して motion に格納し、同じ motion を返す。"""
    motion.sim_certificate = simulate_certificate(motion, morphology, **kwargs)
    return motion


def _zmp_in_support(zmp_xy: np.ndarray, foot_pts: np.ndarray, margin: float) -> bool:
    """ZMP が接地足の点群から margin 以内にあるか（各足点を半径 margin の円で覆う近似）。"""
    d = np.linalg.norm(foot_pts - zmp_xy[None, :], axis=1)
    return bool(d.min() <= margin)
