"""Contrastive text-motion アライメントの検証。torch 無しは skip。"""

from __future__ import annotations

import numpy as np
import pytest

pytest.importorskip("torch")

from robotdance_core.synthetic import generate_backflip, generate_dance  # noqa: E402
from robotdance_models.text import TEXT_DIM, text_features, tokenize  # noqa: E402


def test_text_features_deterministic_and_normalized() -> None:
    a = text_features("a person doing a backflip")
    b = text_features("a person doing a backflip")
    assert a.shape == (TEXT_DIM,)
    assert np.allclose(a, b)                       # プロセス間で決定的
    assert abs(np.linalg.norm(a) - 1.0) < 1e-5     # L2 正規化
    # 空文字は zero ベクトル。
    assert np.allclose(text_features(""), 0.0)
    # 別の文は別のベクトル。
    assert not np.allclose(a, text_features("standing still"))


def test_tokenize() -> None:
    assert tokenize("Fast, energetic DANCE!") == ["fast", "energetic", "dance"]


def test_corpus_structure() -> None:
    from robotdance_models.contrastive import build_labeled_corpus

    pairs = build_labeled_corpus()
    assert len(pairs) > 10
    groups = {g for _, _, g in pairs}
    assert {"dance_fast", "dance_slow", "idle", "backflip"} <= groups
    for mir, cap, group in pairs:
        assert cap and isinstance(cap, str)
        assert mir.keypoints_3d is not None


def test_train_reduces_loss_and_retrieves(tmp_path) -> None:
    from robotdance_models.contrastive import TextMotionModel, train_text_motion

    res = train_text_motion(out_path=tmp_path / "tm.pt", epochs=200, seed=0)
    h = res["loss_history"]
    assert len(h) == 200
    assert h[-1] < h[0]                 # 学習が進む
    # caption→motion を action 群レベルで概ね正しく引ける（variant は可換なので exact は問わない）。
    assert res["group_top1"] >= 0.9

    model = TextMotionModel(tmp_path / "tm.pt")
    # text / motion とも単位ベクトル（共有球面）。
    v = model.embed_text("a backflip")
    assert abs(np.linalg.norm(v) - 1.0) < 1e-4

    suite = {
        "dance_fast": generate_dance(beats_per_second=1.6),
        "dance_slow": generate_dance(beats_per_second=0.7),
        "idle": generate_dance(beats_per_second=0.5, arm_amp=0.15, sway_amp=0.04),
        "backflip": generate_backflip(duration=1.6),
    }
    # 学習に無い言い回しでも正しい motion を top-1 で引ける。
    assert model.search("flipping backwards through the air", suite, k=1)[0][0] == "backflip"
    assert model.search("a person standing motionless", suite, k=1)[0][0] == "idle"
