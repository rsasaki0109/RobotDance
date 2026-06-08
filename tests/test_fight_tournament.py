"""Physical fight トーナメント（ヒット採点ブラケット）の検証。"""

from __future__ import annotations

import pytest

from robotdance_benchmarks.fight_tournament import (
    play_fight_match,
    run_fight_tournament,
)


def test_play_fight_match_picks_winner_by_hits() -> None:
    pytest.importorskip("mujoco")
    m = play_fight_match("unitree_g1", "unitree_h1", ["boxing"], duration=3.0)
    assert len(m.rounds) == 1
    r = m.rounds[0]
    assert r.style == "boxing"
    assert m.p1_total_hits == r.p1_hits
    assert m.winner in ("unitree_g1", "unitree_h1", "DRAW")
    if m.winner != "DRAW":
        assert m.p1_rounds == 1 or m.p2_rounds == 1


def test_play_fight_match_best_of_styles() -> None:
    pytest.importorskip("mujoco")
    m = play_fight_match("unitree_g1", "unitree_h1", ["boxing", "karate"], duration=2.5)
    assert len(m.rounds) == 2
    assert {r.style for r in m.rounds} == {"boxing", "karate"}
    assert m.p1_rounds + m.p2_rounds <= 2


def test_invalid_fight_style_raises() -> None:
    with pytest.raises(ValueError, match="physical"):
        play_fight_match("unitree_g1", "unitree_h1", ["kata"])


def test_fight_tournament_crowns_champion() -> None:
    pytest.importorskip("mujoco")
    robots = ["unitree_g1", "unitree_h1", "fourier_n1"]
    t = run_fight_tournament(robots, ["boxing"], duration=2.5)
    assert t.champion in robots
    assert len(t.bracket) >= 1
    assert t.final.winner == t.champion or t.final.winner == "DRAW"
    assert t.champion in (t.final.p1, t.final.p2)


def test_fight_tournament_odd_field_bye() -> None:
    pytest.importorskip("mujoco")
    t = run_fight_tournament(["unitree_g1", "unitree_h1", "unitree_h2"], ["boxing"], duration=2.0)
    assert t.byes
    assert t.champion in ("unitree_g1", "unitree_h1", "unitree_h2")
