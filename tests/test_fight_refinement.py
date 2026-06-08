"""fight 向け depth 精緻化パイプラインの検証。"""

from __future__ import annotations

import numpy as np

from robotdance_core.synthetic import generate_squat
from robotdance_motion.fight_refinement import refine_for_fight
from robotdance_sim.fight_moves import generate_boxing


def test_refine_for_fight_preserves_yz_and_records_metrics() -> None:
    mir = generate_boxing(duration=2.0)
    before = mir.keypoints_3d_array().copy()
    refined = refine_for_fight(mir, balance_strength=0.4)
    after = refined.keypoints_3d_array()
    assert np.allclose(after[:, :, 1], before[:, :, 1], atol=1e-9)
    assert np.allclose(after[:, :, 2], before[:, :, 2], atol=1e-9)
    fr = refined.quality_metrics["fight_refinement"]
    assert fr["stabilize"] is True and fr["balance"] is True


def test_refine_for_fight_stabilize_only() -> None:
    mir = generate_squat(duration=1.5)
    refined = refine_for_fight(mir, balance=False)
    assert refined.quality_metrics["fight_refinement"]["balance"] is False


def test_run_fight_depth_refine_flag() -> None:
    import pytest

    pytest.importorskip("mujoco")
    from robotdance_sim.arena import run_fight
    from robotdance_unitree import get_morphology

    res = run_fight(
        get_morphology("unitree_g1"), get_morphology("unitree_h1"),
        name_a="unitree_g1", name_b="unitree_h1", duration=2.5, render=False,
        style="boxing", depth_refine=True,
    )
    assert res.winner in ("unitree_g1", "unitree_h1", "DRAW")
