"""reference qpos 軌道の品質指標（twist 安定化の効果定量化, v0.48）。

RL tracking / PD 追従は keypoints を復元した **reference qpos 列** を追う。その品質は
「フレーム間で reference がどれだけ速く動くよう要求するか」で決まる。極端な屈曲では単フレーム
独立復元（_pose_to_qpos）が bone 軸まわりに**データに無い偽 twist スパイク**を生み、reference
速度が ~80 rad/s に跳ねて PD 追従誤差（mj_differentiatePos）を汚す（v0.43→v0.47 の経緯）。

本モジュールは reference 速度を **MuJoCo 自身の tangent 空間差分**（env の `_err_to` と同じ写像）で
測り、単フレーム復元 vs 時系列復元（_poses_to_qpos）を比較して twist 安定化の効果を数値化する。
sim_certificate の verdict 等とは独立な「reference 軌道の clean さ」のメトリクスである。
"""

from __future__ import annotations

from typing import Any

import numpy as np

from robotdance_core.rd_motion import RdMotion
from robotdance_retarget.embodiment import RobotMorphology

from .mjcf import build_mjcf
from .mujoco_backend import _max_bone_angular_speed, _pose_to_qpos, _poses_to_qpos


def _max_ref_joint_speed(model, qpos: np.ndarray, dt: float) -> float:
    """reference qpos 列の駆動関節 reference 速度の最大値 [rad/s]。

    連続フレーム間を MuJoCo の tangent 空間差分（mj_differentiatePos）で取る。これは env の
    PD が `kp * (target ⊖ current)` で追うベクトルそのもの＝コントローラが要求する reference 速度。
    free joint の base 6-DOF は除外し、駆動関節（ball joint）の最大角速度を返す。
    """
    import mujoco

    n = qpos.shape[0]
    if n < 2:
        return 0.0
    nv = model.nv
    worst = 0.0
    dq = np.zeros(nv)
    for f in range(n - 1):
        mujoco.mj_differentiatePos(model, dq, 1.0, qpos[f], qpos[f + 1])
        # 駆動関節 = base 6-DOF を除く各 ball joint（3-DOF）の角速度ノルム。
        joint_dofs = dq[6:].reshape(-1, 3)
        worst = max(worst, float(np.linalg.norm(joint_dofs, axis=1).max()) / dt)
    return worst


def reference_velocity_report(
    motion: RdMotion, morphology: RobotMorphology
) -> dict[str, Any]:
    """単フレーム復元 vs 時系列復元の reference 速度を比較した品質レポート。

    返り値:
      per_frame_max_rad_s: 単フレーム独立復元（_pose_to_qpos）の reference 最大関節速度。
      temporal_max_rad_s:  時系列復元（_poses_to_qpos, 既定経路）の reference 最大関節速度。
      bone_truth_rad_s:    bone 方向変化率（twist-free, 物理的に真の運動速度）。
      spike_factor:        per_frame / temporal（>1 ほど単フレーム復元の偽スパイクが大きい）。
    bone 方向は両復元で厳密一致するので、temporal は bone_truth に整合し、per_frame との差は
    すべて不可観測な twist アーティファクト。
    """
    import mujoco

    kps = motion.keypoints_3d_array()  # [T, J, 3]
    dt = 1.0 / float(motion.fps)
    model = mujoco.MjModel.from_xml_string(
        build_mjcf(morphology, total_mass=morphology.sim_defaults.total_mass, ground=False)
    )
    n = kps.shape[0]
    per_frame = np.stack([_pose_to_qpos(model, morphology, kps[f]) for f in range(n)])
    temporal = _poses_to_qpos(model, morphology, kps)

    pf = _max_ref_joint_speed(model, per_frame, dt)
    tm = _max_ref_joint_speed(model, temporal, dt)
    truth = _max_bone_angular_speed(kps, dt)
    return {
        "per_frame_max_rad_s": pf,
        "temporal_max_rad_s": tm,
        "bone_truth_rad_s": truth,
        "spike_factor": (pf / tm) if tm > 1e-6 else float("inf"),
    }


def reference_quality_table(
    robots: "list[str] | None" = None, motions: "dict[str, Any] | None" = None
) -> str:
    """default_motion_suite × robots の reference 品質を markdown 表に整形する。"""
    from robotdance_benchmarks.suite import default_motion_suite
    from robotdance_retarget.kinematic import retarget
    from robotdance_unitree import get_morphology

    robots = robots or ["unitree_g1", "unitree_h1"]
    suite = motions or default_motion_suite()

    rows = []
    for rob in robots:
        morph = get_morphology(rob)
        for name, mir in suite.items():
            motion = retarget(mir, morph)
            r = reference_velocity_report(motion, morph)
            rows.append((rob, name, r))

    out = [
        "# Reference qpos 品質（twist 安定化の効果）",
        "",
        "RL tracking / PD 追従が追う reference qpos 列の **reference 関節速度**（連続フレーム間の",
        "tangent 空間差分 = コントローラが要求する速度）を、単フレーム独立復元と時系列復元",
        "（`_poses_to_qpos`, v0.47 既定）で比較する。極端な屈曲では単フレーム復元が bone 軸まわりに",
        "偽 twist スパイクを生み reference 速度が跳ねる。時系列復元はそれを除去し、物理的に真の",
        "bone 方向速度（twist-free）に整合する。**bone 方向は両者で厳密一致するので位置・COM・",
        "verdict は不変**で、差はすべて不可観測な twist アーティファクト。",
        "",
        "| robot | motion | per-frame [rad/s] | temporal [rad/s] | bone-truth [rad/s] | spike factor |",
        "| --- | --- | ---: | ---: | ---: | ---: |",
    ]
    for rob, name, r in rows:
        sf = r["spike_factor"]
        sf_s = "∞" if sf == float("inf") else f"{sf:.1f}×"
        out.append(
            f"| {rob} | {name} | {r['per_frame_max_rad_s']:.1f} | "
            f"{r['temporal_max_rad_s']:.2f} | {r['bone_truth_rad_s']:.2f} | {sf_s} |"
        )
    out.append("")
    out.append(
        "> per-frame と temporal が一致する motion（spike factor ≈ 1）は反平行付近に滞在する bone が"
    )
    out.append(
        "> 無く特異点を踏まないため。overbend のような過屈曲でのみ偽スパイクが顕在化する。"
    )
    out.append("")
    return "\n".join(out)


if __name__ == "__main__":  # python3 -m robotdance_sim.reference_quality で doc 再生成
    hint = (
        "> 生成: `python3 -m robotdance_sim.reference_quality > "
        "docs/sim/REFERENCE_QUALITY.md`（決定的・sim 依存）\n"
    )
    lines = reference_quality_table().split("\n")
    lines.insert(2, hint)
    print("\n".join(lines))
