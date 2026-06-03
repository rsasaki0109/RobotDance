"""学習 motion encoder（masked motion modeling）の検証。torch 無しは skip。"""

from __future__ import annotations

import numpy as np
import pytest

pytest.importorskip("torch")

from robotdance_core.skeleton import NUM_JOINTS  # noqa: E402
from robotdance_core.synthetic import generate_backflip, generate_dance  # noqa: E402
from robotdance_models.encoder import INPUT_DIM, MotionEncoderNet, window_motion  # noqa: E402


def test_input_dim() -> None:
    assert INPUT_DIM == NUM_JOINTS * 3


def test_window_motion_shapes() -> None:
    rel = np.zeros((50, NUM_JOINTS, 3))
    w = window_motion(rel, window=32, stride=8)
    assert w.ndim == 3 and w.shape[1] == 32 and w.shape[2] == INPUT_DIM
    # 短いクリップは pad されて 1 ウィンドウ。
    short = window_motion(np.zeros((10, NUM_JOINTS, 3)), window=32, stride=8)
    assert short.shape == (1, 32, INPUT_DIM)


def test_encoder_forward_shapes() -> None:
    import torch

    net = MotionEncoderNet(d_model=32, nhead=2, nlayers=1, emb_dim=16)
    x = torch.zeros(4, 32, INPUT_DIM)
    mask = torch.zeros(4, 32, dtype=torch.bool)
    emb, recon = net(x, mask)
    assert emb.shape == (4, 16)
    assert recon.shape == (4, 32, INPUT_DIM)


def test_training_reduces_loss(tmp_path) -> None:
    from robotdance_models.train import train_encoder

    res = train_encoder(out_path=tmp_path / "enc.pt", epochs=15, seed=0)
    h = res["loss_history"]
    assert len(h) == 15
    assert h[-1] < h[0]            # 学習が進む
    assert h[-1] < 0.5 * h[0]      # masked 再構成 loss が半分以下に


def test_learned_embed_and_retrieval(tmp_path) -> None:
    from robotdance_models.train import LearnedMotionEncoder, train_encoder
    from robotdance_motion.embeddings import MotionIndex

    train_encoder(out_path=tmp_path / "enc.pt", epochs=20, seed=0)
    enc = LearnedMotionEncoder(tmp_path / "enc.pt")

    e = enc.embed(generate_dance(duration=2.0))
    assert e.ndim == 1 and e.shape[0] > 0

    idx = MotionIndex(embed_fn=enc.embed)
    for mid, mir in {"dance_a": generate_dance(beats_per_second=1.0),
                     "dance_b": generate_dance(beats_per_second=1.4),
                     "backflip": generate_backflip()}.items():
        mir.motion_id = mid
        idx.add_mir(mir)
    ranked = idx.query(enc.embed(generate_dance(beats_per_second=1.2)), k=3)
    # dance クエリで backflip は最下位。
    assert [r[0] for r in ranked].index("backflip") == 2
