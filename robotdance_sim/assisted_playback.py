"""Assisted balance playback — TrackingEnv で PD-only / RL 物理追従。

kinematic（倒れないキネマティック）と learned policy の中間段階。
`demo-assisted` / `demo-fight --assisted` が参照 vs 物理追従を使う。
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from robotdance_core.rd_motion import RdMotion
from robotdance_retarget.embodiment import RobotMorphology


@dataclass
class AssistedPlaybackResult:
    survived_frames: int
    total_frames: int
    survival_ratio: float
    mean_pose_rmse: float
    fallen: bool
    keypoints: np.ndarray = field(repr=False)  # [T, J, 3] sim 軌跡


def rollout_pd_only(reference: RdMotion, morphology: RobotMorphology) -> AssistedPlaybackResult:
    """参照 RD-Motion を PD-only（action=0）で物理追従し、キーポイント列を返す。"""
    from robotdance_sim.tracking_env import TrackingEnv

    env = TrackingEnv(reference, morphology)
    env.reset()
    kps: list[np.ndarray] = [env.current_keypoints()]
    rmse_sum = 0.0
    n = 0
    fallen = False
    for _ in range(env.T - 1):
        _o, _r, done, info = env.step(np.zeros(env.action_dim, dtype=np.float64))
        kps.append(env.current_keypoints())
        rmse_sum += float(info["pose_rmse"])
        n += 1
        if done and info.get("fallen"):
            fallen = True
            break
    survived = n
    total = max(env.T - 1, 1)
    return AssistedPlaybackResult(
        survived_frames=survived,
        total_frames=total,
        survival_ratio=round(survived / total, 3),
        mean_pose_rmse=round(rmse_sum / max(n, 1), 4),
        fallen=fallen,
        keypoints=np.stack(kps, axis=0),
    )


def rollout_rl(
    reference: RdMotion,
    morphology: RobotMorphology,
    *,
    iterations: int = 20,
    steps_per_iter: int = 256,
    seed: int = 0,
) -> AssistedPlaybackResult:
    """参照 RD-Motion を PPO で学習し、決定論的 RL ロールアウトのキーポイント列を返す。"""
    from robotdance_models.tracking_policy import train_tracking_policy

    policy, _info = train_tracking_policy(
        reference, morphology, iterations=iterations, steps_per_iter=steps_per_iter, seed=seed,
    )
    motion, metrics = policy.rollout()
    kps_arr = motion.keypoints_3d_array()
    survived = int(metrics["survived_frames"])
    total = int(metrics["reference_frames"])
    return AssistedPlaybackResult(
        survived_frames=survived,
        total_frames=total,
        survival_ratio=round(float(metrics["survival_ratio"]), 3),
        mean_pose_rmse=round(float(metrics["mean_pose_rmse"]), 4),
        fallen=survived < total,
        keypoints=kps_arr,
    )


__all__ = ["AssistedPlaybackResult", "rollout_pd_only", "rollout_rl"]
