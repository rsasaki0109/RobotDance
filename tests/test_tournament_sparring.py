"""Physical tournament 決勝 sparring 描画の検証。"""

from __future__ import annotations

from pathlib import Path

import pytest

from robotdance_benchmarks.fight_tournament import run_fight_tournament
from robotdance_core.cli import main
from robotdance_sim.arena import run_fight
from robotdance_unitree import get_morphology


def test_bracket_stays_kinematic_without_sparring() -> None:
    pytest.importorskip("mujoco")
    t = run_fight_tournament(["unitree_g1", "unitree_h1"], ["boxing"], duration=2.5)
    assert t.final.hi_fight is not None
    assert not t.final.hi_fight.sparring


def test_final_sparring_fight_runs() -> None:
    pytest.importorskip("mujoco")
    t = run_fight_tournament(["unitree_g1", "unitree_h1"], ["karate"], duration=2.5)
    fin = run_fight(
        get_morphology(t.final.p1), get_morphology(t.final.p2),
        name_a=t.final.p1, name_b=t.final.p2, duration=2.5, style=t.final.hi_style,
        render=False, sparring=True,
    )
    assert fin.sparring is True
    assert fin.p1_survival is not None
    assert fin.p2_survival is not None


def test_cli_tournament_sparring_assisted_conflict(tmp_path: Path) -> None:
    code = main([
        "demo-tournament", "--physical", "--sparring", "--assisted", "champion",
        "--robots", "unitree_g1", "unitree_h1",
        "--moves", "boxing", "-o", str(tmp_path / "x.gif"),
    ])
    assert code == 1


def test_final_sparring_contact_scoring_runs() -> None:
    pytest.importorskip("mujoco")
    t = run_fight_tournament(["unitree_g1", "unitree_h1"], ["boxing"], duration=2.5)
    fin = run_fight(
        get_morphology(t.final.p1), get_morphology(t.final.p2),
        name_a=t.final.p1, name_b=t.final.p2, duration=2.5, style=t.final.hi_style,
        render=False, sparring=True, contact_scoring=True,
    )
    assert fin.scoring_mode == "contact"
    assert fin.p1_geom_hits is not None


def test_cli_tournament_contact_requires_sparring(tmp_path: Path) -> None:
    code = main([
        "demo-tournament", "--physical", "--contact-scoring",
        "--robots", "unitree_g1", "unitree_h1",
        "--moves", "boxing", "-o", str(tmp_path / "x.gif"),
    ])
    assert code == 1
