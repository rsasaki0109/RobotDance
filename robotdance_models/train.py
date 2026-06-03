"""Motion encoder の学習・チェックポイント・推論（v0）。

build_corpus → train_encoder（masked 再構成）→ checkpoint 保存。LearnedMotionEncoder で
読み込み、手作りと同じ `embed(mir) -> np.ndarray` を提供する（MotionIndex に差し込める）。
"""

from __future__ import annotations

from pathlib import Path
from typing import Optional

import numpy as np
import torch
from torch import nn

from robotdance_core.rd_mir import RdMir
from robotdance_motion.embeddings import normalized_keypoints

from .encoder import MotionEncoderNet, make_mask, window_motion


def build_corpus() -> list[RdMir]:
    """学習用の合成モーション集合（多様な dance / idle / backflip）。決定的・権利クリーン。"""
    from robotdance_core.synthetic import generate_backflip, generate_dance

    motions: list[RdMir] = []
    for bps in (0.7, 1.0, 1.3, 1.6):
        motions.append(generate_dance(beats_per_second=bps))
    for arm in (0.1, 0.2, 0.3):
        motions.append(generate_dance(beats_per_second=0.5, arm_amp=arm, sway_amp=0.04))
    for dur in (1.3, 1.6, 1.9):
        motions.append(generate_backflip(duration=dur))
    return motions


def _windows_from(motions: list[RdMir], window: int, stride: int) -> np.ndarray:
    chunks = [window_motion(normalized_keypoints(m), window, stride) for m in motions]
    return np.concatenate(chunks, axis=0).astype(np.float32)  # [N, W, INPUT_DIM]


def train_encoder(
    *,
    out_path: str | Path = "motion_encoder.pt",
    window: int = 32,
    stride: int = 8,
    epochs: int = 40,
    batch_size: int = 32,
    mask_ratio: float = 0.4,
    lr: float = 1e-3,
    seed: int = 0,
    device: Optional[str] = None,
) -> dict:
    """合成 corpus で masked 再構成学習し checkpoint を保存。loss 履歴を返す。"""
    dev = device or ("cuda" if torch.cuda.is_available() else "cpu")
    gen = torch.Generator().manual_seed(seed)
    torch.manual_seed(seed)

    data = torch.from_numpy(_windows_from(build_corpus(), window, stride))
    model = MotionEncoderNet().to(dev)
    opt = torch.optim.Adam(model.parameters(), lr=lr)
    loss_fn = nn.MSELoss()

    n = data.shape[0]
    history: list[float] = []
    model.train()
    for _ in range(epochs):
        perm = torch.randperm(n, generator=gen)
        epoch_loss = 0.0
        for s in range(0, n, batch_size):
            xb = data[perm[s:s + batch_size]].to(dev)
            mask = make_mask(xb.shape[0], window, mask_ratio, gen).to(dev)
            _, recon = model(xb, mask)
            loss = loss_fn(recon[mask], xb[mask])  # マスク位置のみ再構成
            opt.zero_grad()
            loss.backward()
            opt.step()
            epoch_loss += loss.item() * xb.shape[0]
        history.append(epoch_loss / n)

    ckpt = {
        "state_dict": model.state_dict(),
        "config": {"window": window},
        "loss_history": history,
    }
    torch.save(ckpt, str(out_path))
    return {"loss_history": history, "checkpoint": str(out_path), "windows": n, "device": dev}


class LearnedMotionEncoder:
    """学習済み encoder。`embed(mir)` で固定長ベクトルを返す（手作りと同 interface）。"""

    def __init__(self, checkpoint: str | Path = "motion_encoder.pt", device: Optional[str] = None):
        self.device = device or ("cuda" if torch.cuda.is_available() else "cpu")
        ckpt = torch.load(str(checkpoint), map_location=self.device, weights_only=False)
        self.window = ckpt["config"]["window"]
        self.model = MotionEncoderNet().to(self.device)
        self.model.load_state_dict(ckpt["state_dict"])
        self.model.eval()

    @torch.no_grad()
    def embed(self, mir: RdMir) -> np.ndarray:
        rel = normalized_keypoints(mir)
        windows = window_motion(rel, self.window, max(1, self.window // 2))
        x = torch.from_numpy(windows.astype(np.float32)).to(self.device)
        emb, _ = self.model(x)              # [N, emb_dim]
        return emb.mean(dim=0).cpu().numpy()  # ウィンドウ平均
