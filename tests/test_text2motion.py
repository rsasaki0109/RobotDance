"""Text-conditioned motion generation の検証。torch 無しは skip。"""

from __future__ import annotations

import numpy as np
import pytest

pytest.importorskip("torch")

from robotdance_models.text2motion import (  # noqa: E402
    ConditionalMotionPrior,
    TextToMotion,
    train_text2motion,
)
from robotdance_models.tokenizer import train_tokenizer  # noqa: E402


def test_conditional_prior_shapes() -> None:
    import torch

    from robotdance_models.text import TEXT_DIM

    net = ConditionalMotionPrior(vocab=40, d_model=32, nhead=2, nlayers=1, max_len=16)
    tokens = torch.zeros(4, 16, dtype=torch.long)
    feats = torch.zeros(4, TEXT_DIM)
    logits = net(tokens, feats)
    assert logits.shape == (4, 16, 40)       # cond 位置を除き L 個の予測


def _energy(mir) -> float:
    return float(mir.keypoints_3d_array().std(axis=0).mean())


def test_train_and_text_conditioning(tmp_path) -> None:
    tok = tmp_path / "tok.pt"
    t2m = tmp_path / "t2m.pt"
    train_tokenizer(out_path=tok, epochs=150, num_codes=128, seed=0)
    res = train_text2motion(tokenizer_ckpt=tok, out_path=t2m, seq_len=16, epochs=400, seed=0)
    assert res["loss_history"][-1] < 0.3 * res["loss_history"][0]
    assert res["next_token_acc"] > 0.7

    g = TextToMotion(t2m)
    flip = g.generate("a person doing a backflip", temperature=0.8, seed=0)
    idle = g.generate("standing still", temperature=0.8, seed=0)
    dance = g.generate("fast energetic dancing", temperature=0.8, seed=0)

    # 生成物は妥当な RD-MIR。
    for m in (flip, idle, dance):
        assert m.keypoints_3d is not None
        assert np.isfinite(m.keypoints_3d_array()).all()

    # テキスト条件付けが効く: backflip は idle より明確に高エネルギー。
    assert _energy(flip) > _energy(idle) * 3.0
    assert _energy(flip) > _energy(dance)
