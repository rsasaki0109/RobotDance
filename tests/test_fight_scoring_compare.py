"""Geometric vs contact fight 採点比較 benchmark の検証。"""

from __future__ import annotations

import pytest

from robotdance_benchmarks.fight_scoring_compare import (
    evaluate_fight_scoring_compare,
    render_fight_scoring_compare_markdown,
    run_fight_scoring_compare_benchmark,
    write_fight_scoring_compare_csv,
)


@pytest.fixture(scope="module")
def mujoco():
    return pytest.importorskip("mujoco")


def test_evaluate_scoring_compare_runs(mujoco) -> None:
    row = evaluate_fight_scoring_compare(
        "unitree_g1", "unitree_h1", "boxing", duration=2.5,
    )
    assert row.geom_p1_hits >= 0
    assert row.contact_p1_hits >= 0
    assert row.geom_winner in ("unitree_g1", "unitree_h1", "DRAW")
    assert row.contact_winner in ("unitree_g1", "unitree_h1", "DRAW")
    assert row.delta_p1_hits == row.contact_p1_hits - row.geom_p1_hits


def test_benchmark_report_structure(mujoco, tmp_path) -> None:
    report = run_fight_scoring_compare_benchmark(
        robots=["unitree_g1"],
        opponent="unitree_h1",
        styles=["boxing"],
        duration=2.5,
        compare_refine=True,
    )
    assert len(report["rows"]) == 2
    assert 0.0 <= report["winner_agreement_rate"] <= 1.0
    md = render_fight_scoring_compare_markdown(report)
    assert "Fight Scoring Compare Benchmark" in md
    assert "winner agreement" in md
    csv_path = write_fight_scoring_compare_csv(report, tmp_path / "scoring.csv")
    assert csv_path.is_file()
    text = csv_path.read_text(encoding="utf-8")
    assert "geom_p1_hits" in text
    assert "contact_p1_hits" in text
