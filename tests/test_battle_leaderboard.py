"""HumanoidBattle leaderboard（ELO / Hall of Champions）の検証。"""

from __future__ import annotations

import json

import pytest

from robotdance_benchmarks.battle_leaderboard import (
    INITIAL_ELO,
    BattleLeaderboardState,
    expected_score,
    load_state,
    record_fight_tournament,
    render_markdown,
    save_state,
    update_elo,
)
from robotdance_benchmarks.fight_tournament import (
    FightMatchResult,
    FightRoundResult,
    FightTournamentResult,
)


def test_elo_favourite_wins_gains_less_than_upset() -> None:
    na_fav, _ = update_elo(1600, 1400, 1.0)
    na_upset, _ = update_elo(1400, 1600, 1.0)
    assert (na_fav - 1600) < (na_upset - 1400)  # 格上撃破の方が ELO 上昇幅が大きい


def test_elo_draw_moves_toward_center() -> None:
    na, nb = update_elo(1500, 1500, 0.5)
    assert na == pytest.approx(1500) and nb == pytest.approx(1500)


def test_apply_match_updates_elo_and_bout_log() -> None:
    st = BattleLeaderboardState()
    m = FightMatchResult(
        "unitree_g1", "unitree_h1",
        [FightRoundResult("boxing", 8, 5, "unitree_g1")],
        1, 0, 8, 5, "unitree_g1",
    )
    st.apply_match(m, styles=["boxing"], event="test")
    assert st.elo["unitree_g1"] > INITIAL_ELO
    assert st.elo["unitree_h1"] < INITIAL_ELO
    assert len(st.bouts) == 1


def test_record_tournament_persists_state(tmp_path) -> None:
    m = FightMatchResult(
        "unitree_g1", "unitree_h1",
        [FightRoundResult("boxing", 6, 4, "unitree_g1")],
        1, 0, 6, 4, "unitree_g1",
    )
    t = FightTournamentResult([[m]], "unitree_g1", m)
    sp = tmp_path / "state.json"
    st = record_fight_tournament(t, ["boxing"], state_path=sp)
    assert sp.is_file()
    assert st.hall[-1]["champion"] == "unitree_g1"
    reloaded = load_state(sp)
    assert reloaded.elo["unitree_g1"] == st.elo["unitree_g1"]


def test_render_markdown_contains_sections() -> None:
    st = BattleLeaderboardState(elo={"unitree_g1": 1512.0})
    md = render_markdown(st)
    assert "ELO Rankings" in md and "Hall of Champions" in md and "unitree_g1" in md


def test_save_load_roundtrip(tmp_path) -> None:
    st = BattleLeaderboardState(elo={"a": 1505.0}, hall=[{"date": "2026-06-08", "champion": "a"}])
    p = tmp_path / "s.json"
    save_state(st, p)
    data = json.loads(p.read_text())
    assert data["elo"]["a"] == 1505.0
    assert load_state(p).hall[0]["champion"] == "a"
