"""RL tracking policy baseline（§4.5）の縦スライス。

mujoco / torch 未インストール環境では skip する（CI には含めない）。
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pytest

pytest.importorskip("mujoco")
pytest.importorskip("torch")

import jsonschema  # noqa: E402

from robotdance_core.synthetic import generate_dance  # noqa: E402
from robotdance_models.tracking_policy import train_tracking_policy  # noqa: E402
from robotdance_retarget.kinematic import retarget  # noqa: E402
from robotdance_sim.tracking_env import TrackingEnv  # noqa: E402
from robotdance_unitree import get_morphology  # noqa: E402


def _gentle_reference():
    morph = get_morphology("unitree_g1")
    ref = retarget(generate_dance(duration=1.0, arm_amp=0.6, sway_amp=0.08), morph)
    return ref, morph


def test_env_pd_baseline_is_stable() -> None:
    """残差ゼロ（関節 PD のみ）で gentle 参照を物理上で追従でき、転倒しない。"""
    ref, morph = _gentle_reference()
    env = TrackingEnv(ref, morph)
    # base は非駆動（free joint の 6-DOF を除く）。
    assert env.action_dim == env.nv - 6
    assert env.obs_dim > 0

    o = env.reset()
    assert o.shape == (env.obs_dim,)
    survived = 0
    info = {"upright": 1.0, "pose_rmse": 0.0}
    for _ in range(env.T - 1):
        o, r, d, info = env.step(np.zeros(env.action_dim))
        assert np.isfinite(r)
        survived += 1
        if d:
            break
    # PD だけで全フレーム生存し、直立を保つ（物理が健全である sanity）。
    assert survived == env.T - 1
    assert info["upright"] > 0.3
    assert np.isfinite(info["pose_rmse"])


def test_ppo_trains_and_rolls_out_valid_motion() -> None:
    """PPO が学習でき、物理ロールアウトが schema 適合の RD-Motion になる。"""
    ref, morph = _gentle_reference()
    policy, info = train_tracking_policy(
        ref, morph, iterations=5, steps_per_iter=256, seed=0
    )
    # 学習が走り、return が有限。
    assert len(info["return_history"]) == 5
    assert all(np.isfinite(info["return_history"]))
    assert info["action_dim"] == info["actuated_dofs"]

    motion, metrics = policy.rollout()
    # 物理ロールアウトの結果が pipeline 互換の RD-Motion。
    assert motion.control_mode == "policy"
    assert motion.num_frames > 0
    assert metrics["method"] == "rl_tracking_policy_ppo"
    assert 0.0 <= metrics["survival_ratio"] <= 1.0
    # 学習後の方策は gentle 参照で相当数のフレームを生存する。
    assert metrics["survival_ratio"] > 0.3
    assert np.isfinite(metrics["mean_pose_rmse"])

    # RD-Motion schema に適合する。
    schema = json.loads(
        (Path(__file__).resolve().parent.parent / "specs" / "rd-motion" / "rd-motion.schema.json")
        .read_text(encoding="utf-8")
    )
    jsonschema.Draft202012Validator(schema).validate(motion.to_dict())


def test_checkpoint_saved(tmp_path: Path) -> None:
    """out_path 指定で torch checkpoint が保存される。"""
    import torch

    ref, morph = _gentle_reference()
    ckpt = tmp_path / "policy.pt"
    _, info = train_tracking_policy(
        ref, morph, iterations=2, steps_per_iter=128, seed=0, out_path=ckpt
    )
    assert ckpt.exists()
    blob = torch.load(ckpt, weights_only=True)
    assert blob["obs_dim"] == info["obs_dim"]
    assert blob["action_dim"] == info["action_dim"]
