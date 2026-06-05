"""benchmark ハーネス（suite + report）の検証。

sim なし（mujoco 不要）の経路を常にテストし、sim ありは importorskip。
"""

from __future__ import annotations

import csv

import pytest

from robotdance_benchmarks.report import aggregate_by_robot, write_csv, write_markdown
from robotdance_benchmarks.suite import default_motion_suite, run_benchmark


def test_default_suite_has_variety() -> None:
    suite = default_motion_suite()
    assert {"dance_normal", "dance_fast", "idle", "backflip", "overbend",
            "squat", "march"} <= set(suite)


def test_run_benchmark_no_sim_shape() -> None:
    n = len(default_motion_suite())
    report = run_benchmark(default_motion_suite(), ["unitree_g1", "unitree_h1"], with_sim=False)
    assert len(report["rows"]) == n * 2
    assert report["sim_available"] is False
    # retarget 指標は sim なしでも入る。
    for r in report["rows"]:
        assert r["bone_direction_cosine"] is not None
        assert r["height_scale"] is not None
        assert r["verdict"] is None  # sim off
        # G1/H1 は per_joint_limits 持ち → 屈曲違反率が入る。
        assert r["joint_flexion_violation"] is not None


def test_write_csv_and_markdown(tmp_path) -> None:
    report = run_benchmark(default_motion_suite(), ["unitree_g1"], with_sim=False)
    csv_path = write_csv(report, tmp_path / "b.csv")
    md_path = write_markdown(report, tmp_path / "L.md")
    with csv_path.open() as f:
        rows = list(csv.DictReader(f))
    assert len(rows) == len(default_motion_suite())
    assert "motion_id" in rows[0] and "verdict" in rows[0]
    assert "joint_flexion_violation" in rows[0]  # 新列が CSV に出る
    # 重力保持 vs 動的（重力＋慣性）トルクが CSV に分離して出る（v0.62/v0.63 の可視化）。
    assert "gravity_torque_nm" in rows[0] and "dynamic_torque_nm" in rows[0]
    md = md_path.read_text("utf-8")
    assert "Leaderboard" in md and "unitree_g1" in md
    assert "屈曲違反" in md  # leaderboard / 全 run 表に屈曲列が出る
    assert "重力tq" in md and "動的tq" in md  # トルク分離列が leaderboard に出る


def test_dynamic_torque_propagates_to_benchmark() -> None:
    """sim 経由で gravity/dynamic torque が row に伝播し、動的 ≥ 重力（慣性は非負寄与）。"""
    pytest.importorskip("mujoco")
    report = run_benchmark({"dance_fast": default_motion_suite()["dance_fast"]},
                           ["unitree_h1"], with_sim=True)
    row = report["rows"][0]
    assert row["gravity_torque_nm"] is not None and row["dynamic_torque_nm"] is not None
    # 速い運動なので動的（重力＋慣性）は重力保持を上回る。
    assert row["dynamic_torque_nm"] >= row["gravity_torque_nm"]


def test_overbend_propagates_violation_to_leaderboard() -> None:
    """overbend motion が benchmark 経由で joint_flexion_violation>0 として leaderboard に伝播。"""
    report = run_benchmark(default_motion_suite(), ["unitree_g1"], with_sim=False)
    overbend = next(r for r in report["rows"] if r["motion_id"] == "overbend")
    dance = next(r for r in report["rows"] if r["motion_id"] == "dance_normal")
    assert overbend["joint_flexion_violation"] > 0.0   # 実機可動域超過を検出
    assert dance["joint_flexion_violation"] == 0.0     # 通常ダンスは違反なし


def test_aggregate_pass_rate() -> None:
    report = {
        "robots": ["r"],
        "rows": [
            {"robot": "r", "verdict": "PASS", "bone_direction_cosine": 1.0,
             "foot_sliding": 0.01, "height_scale": 0.8},
            {"robot": "r", "verdict": "REJECT", "bone_direction_cosine": 1.0,
             "foot_sliding": 0.03, "height_scale": 0.8},
        ],
    }
    agg = aggregate_by_robot(report)[0]
    assert agg["pass_rate"] == 0.5
    assert agg["n"] == 2


def test_run_benchmark_with_sim() -> None:
    pytest.importorskip("mujoco")
    report = run_benchmark(default_motion_suite(), ["unitree_g1"], with_sim=True)
    assert report["sim_available"] is True
    verdicts = {r["motion_id"]: r["verdict"] for r in report["rows"]}
    assert verdicts["backflip"] == "REJECT"   # 危険動作は reject
    assert verdicts["dance_normal"] == "PASS"  # 安全動作は pass
