"""Assisted survival benchmark の検証。"""

from __future__ import annotations

import pytest

from robotdance_benchmarks.assisted_survival import (
    evaluate_assisted_survival,
    evaluate_rl_survival,
    render_assisted_survival_markdown,
    run_assisted_survival_benchmark,
    write_assisted_survival_csv,
)


@pytest.fixture(scope="module")
def mujoco():
    return pytest.importorskip("mujoco")


def test_evaluate_boxing_g1_survives(mujoco) -> None:
    row = evaluate_assisted_survival("unitree_g1", "boxing", duration=2.5)
    assert row.survival_ratio == 1.0
    assert not row.fallen
    assert row.mean_pose_rmse > 0


def test_depth_refine_rescues_g1_kick(mujoco) -> None:
    raw = evaluate_assisted_survival("unitree_g1", "kick", duration=3.0, depth_refine=False)
    refined = evaluate_assisted_survival("unitree_g1", "kick", duration=3.0, depth_refine=True)
    assert raw.survival_ratio < 0.5
    assert refined.survival_ratio == 1.0


def test_karate_kathak_survive_on_g1(mujoco) -> None:
    for style in ("karate", "kathak"):
        row = evaluate_assisted_survival("unitree_g1", style, depth_refine=False)
        assert row.survival_ratio == 1.0


def test_benchmark_report_structure(mujoco, tmp_path) -> None:
    report = run_assisted_survival_benchmark(
        robots=["unitree_g1"],
        styles=["boxing", "kick"],
        duration=3.0,
        compare_refine=True,
    )
    assert len(report["rows"]) == 4
    assert report["rescued"]
    assert any(r["style"] == "kick" for r in report["rescued"])
    md = render_assisted_survival_markdown(report)
    assert "Assisted Survival Benchmark" in md
    assert "unitree_g1" in md
    csv_path = write_assisted_survival_csv(report, tmp_path / "assisted.csv")
    assert csv_path.is_file()
    assert "survival_ratio" in csv_path.read_text(encoding="utf-8")
    assert "controller" in csv_path.read_text(encoding="utf-8")


def test_benchmark_with_rl_on_pd_failures(mujoco) -> None:
    pytest.importorskip("torch")
    report = run_assisted_survival_benchmark(
        robots=["unitree_g1"],
        styles=["kick"],
        duration=3.0,
        compare_refine=True,
        with_rl=True,
        rl_iterations=8,
    )
    rl_rows = [r for r in report["rows"] if r["controller"] == "rl"]
    assert len(rl_rows) == 1  # raw kick のみ PD 失敗（refine は 1.0 でスキップ）
    assert rl_rows[0]["pd_survival"] < 1.0
    md = render_assisted_survival_markdown(report)
    assert "PD vs RL" in md


def test_rescued_by_rl_only_when_refine_fixed(mujoco) -> None:
    pytest.importorskip("torch")
    report = run_assisted_survival_benchmark(
        robots=["unitree_g1"],
        styles=["kick"],
        duration=3.0,
        compare_refine=True,
        with_rl=True,
        rl_iterations=6,
    )
    # G1 kick: raw PD 失敗 → refine で救済 → RL raw のみ（refine 失敗行は無し）
    rl_raw = [r for r in report["rows"]
              if r["controller"] == "rl" and not r["depth_refine"]]
    assert len(rl_raw) == 1
    assert report["rescued_by_rl_only"] == []


def test_evaluate_rl_kick_runs(mujoco) -> None:
    pytest.importorskip("torch")
    row = evaluate_rl_survival(
        "unitree_g1", "kick", depth_refine=True, duration=3.0, iterations=5,
    )
    assert row.controller == "rl"
    assert row.survival_ratio >= 0.0
