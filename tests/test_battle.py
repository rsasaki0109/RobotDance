"""HumanoidBattle の採点・対戦ロジックの検証（render 不要・決定的）。"""

from __future__ import annotations

import pytest

from robotdance_benchmarks.battle import (
    MOTIONS,
    _parse_fighter,
    run_battle,
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
