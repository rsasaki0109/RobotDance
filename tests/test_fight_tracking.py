"""Fight motion RL tracking ヘルパーの検証。"""

from __future__ import annotations

import pytest

from robotdance_sim.fight_tracking import (
    FIGHT_TRACKING_SUITE,
    fight_tracking_reference,
    fight_tracking_suite,
)
from robotdance_unitree import get_morphology


@pytest.fixture(scope="module")
def mujoco():
    return pytest.importorskip("mujoco")


@pytest.fixture(scope="module")
def torch():
    return pytest.importorskip("torch")


def test_fight_tracking_reference_shape(mujoco) -> None:
    ref = fight_tracking_reference("unitree_g1", "boxing", duration=2.5)
    assert ref.num_frames > 0
    kps = ref.keypoints_3d_array()
    assert kps.shape[1:] == (19, 3)


def test_fight_tracking_suite_has_four_styles(mujoco) -> None:
    morph = get_morphology("unitree_g1")
    suite = fight_tracking_suite(morph, duration=2.5)
    assert len(suite) == len(FIGHT_TRACKING_SUITE)
    assert [name for name, _ in suite] == list(FIGHT_TRACKING_SUITE)


def test_ppo_trains_on_fight_karate(torch, mujoco) -> None:
    from robotdance_models.tracking_policy import train_tracking_policy

    ref = fight_tracking_reference("unitree_g1", "karate")
    policy, info = train_tracking_policy(ref, get_morphology("unitree_g1"),
                                           iterations=5, steps_per_iter=256, seed=0)
    assert len(info["return_history"]) == 5
    metrics = policy.rollout()[1]
    assert metrics["survival_ratio"] > 0.3


def test_ppo_trains_on_fight_kick_with_depth_refine(torch, mujoco) -> None:
    from robotdance_models.tracking_policy import train_tracking_policy

    ref = fight_tracking_reference("unitree_g1", "kick", depth_refine=True, duration=3.0)
    policy, info = train_tracking_policy(ref, get_morphology("unitree_g1"),
                                           iterations=5, steps_per_iter=256, seed=0)
    metrics = policy.rollout()[1]
    assert metrics["survival_ratio"] > 0.3
