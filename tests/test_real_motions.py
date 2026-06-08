"""実動画フィクスチャ（karate / kathak）のロードと HumanoidBattle 統合。"""

from __future__ import annotations

import pytest

from robotdance_benchmarks.battle import DIFFICULTY, MOTIONS, evaluate, run_battle
from robotdance_benchmarks.real_motions import REAL_MOTION_NAMES, load_karate, load_kathak
from robotdance_sim.arena import motion_for_style


def test_real_motion_fixtures_load() -> None:
    for name, loader in (("karate", load_karate), ("kathak", load_kathak)):
        mir = loader()
        k = mir.keypoints_3d_array()
        assert k.shape[1:] == (19, 3) and k.shape[0] >= 30
        assert mir.privacy_flags.get("source_pixels_redistributed") is False
        assert mir.source_ref.get("license") == "CC BY-SA 4.0"
        assert name in REAL_MOTION_NAMES


def test_real_motions_in_battle_registry() -> None:
    for name in REAL_MOTION_NAMES:
        assert name in MOTIONS and name in DIFFICULTY
        mir = MOTIONS[name]()
        assert mir.keypoints_3d_array().shape[1:] == (19, 3)


def test_battle_with_real_karate_move() -> None:
    res = run_battle("unitree_g1:karate", "unitree_h1:karate")
    assert res.winner in (res.p1_name, res.p2_name, "DRAW")
    assert res.p1_kps.shape == res.p2_kps.shape


def test_evaluate_real_kathak() -> None:
    m = evaluate("unitree_g1", "kathak")
    assert 0.0 <= m.score <= 100.0
    assert m.move == "kathak"


def test_motion_for_style_boxing_and_karate() -> None:
    box = motion_for_style("boxing", duration=2.0)
    assert box.semantics.get("action_label") == "boxing"
    kata = motion_for_style("karate")
    assert kata.semantics.get("action_label") == "karate_kata"
    with pytest.raises(ValueError, match="未知"):
        motion_for_style("moonwalk")


def test_run_fight_karate_style_scores() -> None:
    pytest.importorskip("mujoco")
    from robotdance_sim.arena import run_fight
    from robotdance_unitree import get_morphology

    res = run_fight(get_morphology("unitree_g1"), get_morphology("unitree_h1"),
                    name_a="unitree_g1", name_b="unitree_h1", style="karate", render=False)
    assert res.p1_hits >= 0 and res.winner in ("unitree_g1", "DRAW")
