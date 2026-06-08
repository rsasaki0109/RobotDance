"""Assisted PD-only playback（balance controller の第一歩）の検証。"""

from __future__ import annotations

import numpy as np
import pytest

from robotdance_core.synthetic import generate_dance
from robotdance_retarget.kinematic import retarget
from robotdance_sim.assisted_playback import rollout_pd_only, rollout_rl
from robotdance_unitree import get_morphology


def _gentle_reference():
    morph = get_morphology("unitree_g1")
    ref = retarget(generate_dance(duration=1.0, arm_amp=0.6, sway_amp=0.08), morph)
    return ref, morph


def test_rollout_pd_only_gentle_survives() -> None:
    pytest.importorskip("mujoco")
    ref, morph = _gentle_reference()
    result = rollout_pd_only(ref, morph)
    assert result.survival_ratio == 1.0
    assert not result.fallen
    assert result.keypoints.shape[1:] == (19, 3)
    assert np.isfinite(result.keypoints).all()


def test_rollout_pd_only_matches_env_step_count() -> None:
    pytest.importorskip("mujoco")
    from robotdance_sim.tracking_env import TrackingEnv

    ref, morph = _gentle_reference()
    env = TrackingEnv(ref, morph)
    env.reset()
    result = rollout_pd_only(ref, morph)
    assert len(result.keypoints) == env.T


def test_rollout_rl_karate_runs() -> None:
    pytest.importorskip("mujoco")
    pytest.importorskip("torch")
    from robotdance_sim.fight_tracking import fight_tracking_reference

    ref = fight_tracking_reference("unitree_g1", "karate")
    result = rollout_rl(ref, get_morphology("unitree_g1"), iterations=5)
    assert result.survival_ratio >= 0.0
    assert result.keypoints.shape[1:] == (19, 3)
