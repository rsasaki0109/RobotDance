"""Physical tournament 決勝 assisted 描画の検証。"""

from __future__ import annotations

import pytest

from robotdance_benchmarks.fight_tournament import (
    play_fight_match,
    resolve_assisted_corner,
    run_fight_tournament,
)
from robotdance_sim.arena import run_fight
from robotdance_unitree import get_morphology


def test_resolve_assisted_corner_champion() -> None:
    assert resolve_assisted_corner("champion", champion="unitree_h1",
                                   p1="unitree_g1", p2="unitree_h1") == "p2"
    assert resolve_assisted_corner("champion", champion="unitree_g1",
                                   p1="unitree_g1", p2="unitree_h1") == "p1"
    assert resolve_assisted_corner("p1", champion="unitree_h1",
                                   p1="unitree_g1", p2="unitree_h1") == "p1"
    assert resolve_assisted_corner(None, champion="x", p1="a", p2="b") is None


def test_bracket_stays_kinematic_without_assisted() -> None:
    pytest.importorskip("mujoco")
    t = run_fight_tournament(["unitree_g1", "unitree_h1"], ["boxing"], duration=2.5)
    assert t.champion in ("unitree_g1", "unitree_h1")
    assert t.final.hi_fight is not None
    assert t.final.hi_fight.assisted_corner is None


def test_final_assisted_rl_fight_runs() -> None:
    pytest.importorskip("torch")
    pytest.importorskip("mujoco")
    m = play_fight_match("unitree_g1", "unitree_h1", ["karate"], duration=2.5)
    fin = run_fight(
        get_morphology(m.p1), get_morphology(m.p2),
        name_a=m.p1, name_b=m.p2, duration=2.5, style=m.hi_style,
        render=False, assisted="p1", assisted_mode="rl", rl_iterations=6,
    )
    assert fin.assisted_corner == "p1"
    assert fin.assisted_mode == "rl"
    assert fin.assisted_survival is not None


def test_final_assisted_champion_corner() -> None:
    pytest.importorskip("mujoco")
    t = run_fight_tournament(["unitree_g1", "unitree_h1"], ["karate"], duration=2.5)
    corner = resolve_assisted_corner(
        "champion", champion=t.champion, p1=t.final.p1, p2=t.final.p2,
    )
    fin = run_fight(
        get_morphology(t.final.p1), get_morphology(t.final.p2),
        name_a=t.final.p1, name_b=t.final.p2, duration=2.5, style=t.final.hi_style,
        render=False, assisted=corner,
    )
    champ_robot = t.final.p1 if corner == "p1" else t.final.p2
    assert champ_robot == t.champion
    assert fin.assisted_corner == corner
