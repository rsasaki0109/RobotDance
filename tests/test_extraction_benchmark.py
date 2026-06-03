"""Extraction benchmark（§4.1）の検証。

純 numpy で torch/mujoco/モデル不要 → CI でも走る。
"""

from __future__ import annotations

import numpy as np

from robotdance_benchmarks.extraction import (
    _umeyama_align,
    compare_extractions,
    extraction_metrics,
    render_extraction_markdown,
    synthetic_extraction_demo,
    write_extraction_csv,
)


def test_identical_extraction_is_zero_error() -> None:
    """GT と同一なら全誤差ゼロ・PCK=1。"""
    gt, _ = synthetic_extraction_demo(seed=0)
    kp = gt.keypoints_3d_array()
    m = extraction_metrics(kp, kp, fps=gt.fps)
    assert m["mpjpe_m"] == 0.0
    assert m["pa_mpjpe_m"] < 1e-6
    assert m["pck@5cm"] == 1.0
    assert m["bone_length_mae_m"] == 0.0


def test_umeyama_recovers_similarity_transform() -> None:
    """相似変換（回転+並進+スケール）された点群を整列で復元できる。"""
    rng = np.random.default_rng(1)
    src = rng.normal(size=(19, 3))
    theta = 0.7
    rot = np.array([[np.cos(theta), -np.sin(theta), 0],
                    [np.sin(theta), np.cos(theta), 0], [0, 0, 1]])
    dst = 1.5 * (rot @ src.T).T + np.array([2.0, -1.0, 0.5])
    aligned = _umeyama_align(src, dst)
    assert np.linalg.norm(aligned - dst, axis=1).mean() < 1e-9


def test_metrics_in_valid_ranges() -> None:
    gt, preds = synthetic_extraction_demo(seed=0)
    rows = compare_extractions(gt, preds)
    assert len(rows) == 2
    for r in rows:
        assert r["mpjpe_m"] >= 0.0
        assert 0.0 <= r["pck@5cm"] <= 1.0
        assert 0.0 <= r["pck@10cm"] <= 1.0
        # PA-MPJPE は相似整列するので MPJPE 以下。
        assert r["pa_mpjpe_m"] <= r["mpjpe_m"] + 1e-6


def test_distinguishes_extractor_profiles() -> None:
    """MediaPipe 風は HMR 風より誤差・jitter が大きい（harness が profile を区別する）。"""
    gt, preds = synthetic_extraction_demo(seed=0)
    rows = {r["extractor"]: r for r in compare_extractions(gt, preds)}
    assert rows["mediapipe_like"]["jitter_pred"] > rows["hmr_like"]["jitter_pred"]
    assert rows["mediapipe_like"]["mpjpe_m"] > rows["hmr_like"]["mpjpe_m"]
    # leaderboard は MPJPE 昇順（良い順）。
    ordered = compare_extractions(gt, preds)
    assert ordered[0]["extractor"] == "hmr_like"


def test_report_outputs(tmp_path) -> None:
    gt, preds = synthetic_extraction_demo(seed=0)
    rows = compare_extractions(gt, preds)
    csv_path = write_extraction_csv(rows, tmp_path / "bx.csv")
    assert csv_path.exists()
    assert "extractor" in csv_path.read_text(encoding="utf-8").splitlines()[0]
    md = render_extraction_markdown(rows, gt_id=gt.motion_id)
    assert "# RobotDance Extraction Benchmark" in md
    assert "MPJPE" in md and "PA-MPJPE" in md
