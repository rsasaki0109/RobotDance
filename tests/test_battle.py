"""HumanoidBattle の採点・対戦ロジックの検証（render 不要・決定的）。"""

from __future__ import annotations

import pytest

from robotdance_benchmarks.battle import (
    DIFFICULTY,
    MOTIONS,
    _parse_fighter,
    evaluate,
    play_match,
    run_battle,
    run_tournament,
    score_fighter,
)


def test_score_is_bounded_and_metric_driven() -> None:
    perfect = {"endeffector_reach_error_m": 0.0, "bone_direction_cosine": 1.0,
               "foot_sliding_m_per_frame": 0.0, "joint_flexion": {"any_violation_ratio": 0.0}}
    bad = {"endeffector_reach_error_m": 0.3, "bone_direction_cosine": 0.5,
           "foot_sliding_m_per_frame": 0.05, "joint_flexion": {"any_violation_ratio": 0.8}}
    sp, sb = score_fighter(perfect), score_fighter(bad)
    assert 0.0 <= sb.overall < sp.overall <= 100.0
    assert sp.overall > 95.0  # 完璧な metrics はほぼ満点
    # breakdown は各 metric 由来で透明。
    assert set(sp.breakdown) >= {"reach", "form", "footwork", "control"}


def test_lower_reach_error_wins() -> None:
    base = {"bone_direction_cosine": 1.0, "foot_sliding_m_per_frame": 0.0,
            "joint_flexion": {"any_violation_ratio": 0.0}}
    good = score_fighter({**base, "endeffector_reach_error_m": 0.05})
    worse = score_fighter({**base, "endeffector_reach_error_m": 0.15})
    assert good.overall > worse.overall


def test_sim_reject_penalizes_balance() -> None:
    rm = {"endeffector_reach_error_m": 0.1, "bone_direction_cosine": 1.0,
          "foot_sliding_m_per_frame": 0.0, "joint_flexion": {"any_violation_ratio": 0.0}}
    passed = score_fighter(rm, {"passed": True,
                                "metrics": {"balance_violation_ratio": 0.0, "torque_ratio": 0.5}})
    rejected = score_fighter(rm, {"passed": False,
                                  "metrics": {"balance_violation_ratio": 0.6, "torque_ratio": 1.5}})
    assert rejected.overall < passed.overall


def test_parse_fighter_defaults_and_validation() -> None:
    assert _parse_fighter("unitree_g1") == ("unitree_g1", "kata")
    assert _parse_fighter("unitree_h1:march") == ("unitree_h1", "march")
    with pytest.raises(ValueError, match="未知の motion"):
        _parse_fighter("unitree_g1:moonwalk")


def test_run_battle_picks_a_winner() -> None:
    res = run_battle("unitree_g1:kata", "unitree_h1:kata")
    assert res.winner in (res.p1_name, res.p2_name, "DRAW")
    assert res.p1_kps.shape == res.p2_kps.shape  # 同一モーション → 同フレーム数
    # 勝者は overall が最大（または DRAW）。
    if res.winner != "DRAW":
        hi = max(res.p1_card.overall, res.p2_card.overall)
        win_card = res.p1_card if res.winner == res.p1_name else res.p2_card
        assert win_card.overall == hi


def test_same_robot_same_motion_is_draw() -> None:
    res = run_battle("unitree_g1:kata", "unitree_g1:kata")
    assert res.winner == "DRAW"  # 同条件は引き分け


def test_motions_registry_nonempty() -> None:
    assert "kata" in MOTIONS and callable(MOTIONS["kata"])


# --- ゲーム層（move / match / tournament）---

def test_evaluate_applies_difficulty_and_is_deterministic() -> None:
    a = evaluate("unitree_g1", "kata")
    b = evaluate("unitree_g1", "kata")
    assert a.score == b.score                     # 決定的
    assert 0.0 <= a.score <= 100.0
    assert a.kps is not None and a.kps.shape[1:] == (19, 3)
    # backflip は難度倍率が最大。
    assert DIFFICULTY["backflip"] == max(DIFFICULTY.values())


def test_play_match_best_of_n_winner_by_rounds() -> None:
    m = play_match("unitree_g1", "unitree_h1", ["kata", "squat", "backflip"])
    assert len(m.rounds) == 3
    assert m.p1_rounds + m.p2_rounds <= 3
    assert m.winner in ("unitree_g1", "unitree_h1", "DRAW")
    if m.winner != "DRAW":
        # 勝者は勝ちラウンド数が多い、または同数なら総得点が多い。
        if m.p1_rounds == m.p2_rounds:
            assert (m.p1_total > m.p2_total) == (m.winner == "unitree_g1")
        else:
            assert (m.p1_rounds > m.p2_rounds) == (m.winner == "unitree_g1")
    assert m.p1_kps is not None and m.p2_kps is not None  # 描画用ハイライト


def test_identical_fighters_draw_match() -> None:
    m = play_match("unitree_g1", "unitree_g1", ["kata", "squat"])
    assert m.winner == "DRAW"
    assert all(r.winner == "TIE" for r in m.rounds)


def test_tournament_crowns_a_valid_champion() -> None:
    robots = ["unitree_g1", "unitree_h1", "unitree_h2",
              "booster_t1", "apptronik_apollo", "fourier_n1"]
    t = run_tournament(robots, ["kata", "squat", "backflip"])
    assert t.champion in robots
    assert len(t.bracket) >= 2                     # 6 体 → 複数ラウンド
    assert t.final.winner == t.champion or t.final.winner == "DRAW"
    # 決勝の対戦者にチャンピオンが含まれる。
    assert t.champion in (t.final.p1, t.final.p2)


def test_tournament_handles_odd_field_with_bye() -> None:
    t = run_tournament(["unitree_g1", "unitree_h1", "fourier_n1"], ["kata"])
    assert t.champion in ("unitree_g1", "unitree_h1", "fourier_n1")
    assert t.byes  # 奇数なので最低 1 回 bye が発生
