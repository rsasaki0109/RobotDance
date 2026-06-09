"""2-body contact sparring (shared arena PD physics)."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from robotdance_sim.arena import FightResult, run_fight


def test_sparring_and_assisted_mutually_exclusive():
    pytest.importorskip("mujoco")
    from robotdance_unitree import get_morphology

    with pytest.raises(ValueError, match="併用"):
        run_fight(
            get_morphology("unitree_g1"), get_morphology("unitree_h1"),
            name_a="unitree_g1", name_b="unitree_h1", duration=2.0, render=False,
            sparring=True, assisted="p1",
        )


def test_run_fight_sparring_scores():
    pytest.importorskip("mujoco")
    from robotdance_unitree import get_morphology

    res = run_fight(
        get_morphology("unitree_g1"), get_morphology("unitree_h1"),
        name_a="unitree_g1", name_b="unitree_h1", duration=2.5, render=False,
        style="boxing", sparring=True,
    )
    assert res.sparring is True
    assert res.p1_survival is not None
    assert res.p2_survival is not None
    assert 0.0 <= res.p1_survival <= 1.0
    assert res.winner in ("unitree_g1", "unitree_h1", "DRAW")


def test_fight_hud_sparring_bar_height():
    from robotdance_core.cli import _fight_hud

    frame = np.full((100, 140, 3), 50, np.uint8)
    res = FightResult(
        "unitree_g1", "unitree_h1", 2, 1, "unitree_g1",
        frames=[frame], p1_cum=[1, 2], p2_cum=[0, 1],
        sparring=True, p1_survival=0.87, p2_survival=1.0,
    )
    out = _fight_hud(res)[0]
    assert out.shape[0] == 100 + 50


def test_cli_sparring_assisted_conflict(tmp_path: Path) -> None:
    from robotdance_core.cli import main

    assert main([
        "demo-fight", "--p1", "unitree_g1", "--p2", "unitree_h1",
        "--sparring", "--assisted", "p1", "-o", str(tmp_path / "x.gif"),
    ]) == 1
