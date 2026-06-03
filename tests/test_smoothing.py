"""temporal smoothing と jitter 指標の検証。"""

from __future__ import annotations

import numpy as np

from robotdance_core.skeleton import NUM_JOINTS
from robotdance_core.synthetic import generate_dance
from robotdance_motion.smoothing import add_jitter, jitter, savgol_smooth, smooth_rdmir


def test_jitter_zero_for_constant() -> None:
    kps = np.ones((10, NUM_JOINTS, 3))
    assert jitter(kps) == 0.0


def test_savgol_preserves_shape() -> None:
    kps = np.random.default_rng(0).normal(size=(30, NUM_JOINTS, 3))
    out = savgol_smooth(kps, window=7, polyorder=2)
    assert out.shape == kps.shape


def test_savgol_handles_short_clip() -> None:
    kps = np.random.default_rng(0).normal(size=(2, NUM_JOINTS, 3))
    out = savgol_smooth(kps, window=7)
    assert out.shape == kps.shape  # クラッシュせず返る


def test_smoothing_reduces_jitter() -> None:
    noisy = add_jitter(generate_dance(duration=2.0), sigma=0.03, seed=1)
    smoothed = smooth_rdmir(noisy)
    qm = smoothed.quality_metrics
    assert qm["jitter_after"] < qm["jitter_before"]
    # 実際に滑らかになっている（半分以下）。
    assert qm["jitter_after"] < 0.5 * qm["jitter_before"]


def test_smooth_rdmir_roundtrips_schema() -> None:
    import json
    from pathlib import Path

    import jsonschema

    smoothed = smooth_rdmir(add_jitter(generate_dance(duration=1.0)))
    schema = json.loads(
        (Path(__file__).resolve().parent.parent / "specs" / "rd-mir" / "rd-mir.schema.json")
        .read_text(encoding="utf-8")
    )
    jsonschema.Draft202012Validator(schema).validate(smoothed.to_dict())
