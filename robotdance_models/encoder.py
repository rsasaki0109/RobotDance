"""Masked Motion Modeling encoder（自己教師あり, v0）。

canonical motion window をマスク再構成で学習する小型 Transformer。学習した表現を mean-pool して
固定長 motion embedding を作る。手作り特徴量（robotdance_motion.embeddings）と同じ前処理
（normalized_keypoints）を共有し、同じ `embed(mir)` interface で MotionIndex に差し込める。

⚠️ v0: 学習基盤の提供が目的。toy 合成データで学習が進む（loss 低下）ことを示すが、
手作り baseline を超えると主張するものではない（要・実データ規模）。
"""

from __future__ import annotations

import numpy as np
import torch
from torch import nn

from robotdance_core.skeleton import NUM_JOINTS

INPUT_DIM = NUM_JOINTS * 3  # 57


def window_motion(rel: np.ndarray, window: int, stride: int) -> np.ndarray:
    """正規化済み [T, J, 3] を [N, window, INPUT_DIM] のウィンドウ群に切る（短ければ pad）。"""
    flat = rel.reshape(rel.shape[0], -1)  # [T, J*3]
    t = flat.shape[0]
    if t < window:
        flat = np.concatenate([flat, np.repeat(flat[-1:], window - t, axis=0)], axis=0)
        return flat[None, :window, :]
    idx = list(range(0, t - window + 1, stride)) or [0]
    return np.stack([flat[i:i + window] for i in idx])


class MotionEncoderNet(nn.Module):
    """[B, W, INPUT_DIM] → embedding [B, emb_dim] + masked 再構成 [B, W, INPUT_DIM]。"""

    def __init__(self, *, d_model: int = 64, nhead: int = 4, nlayers: int = 2,
                 emb_dim: int = 64, max_len: int = 128) -> None:
        super().__init__()
        self.d_model = d_model
        self.in_proj = nn.Linear(INPUT_DIM, d_model)
        self.pos = nn.Parameter(torch.zeros(1, max_len, d_model))
        self.mask_token = nn.Parameter(torch.zeros(d_model))
        layer = nn.TransformerEncoderLayer(d_model, nhead, dim_feedforward=2 * d_model,
                                           batch_first=True, dropout=0.0)
        self.encoder = nn.TransformerEncoder(layer, nlayers)
        self.embed_head = nn.Linear(d_model, emb_dim)
        self.recon_head = nn.Linear(d_model, INPUT_DIM)

    def forward(self, x: torch.Tensor, mask: torch.Tensor | None = None):
        h = self.in_proj(x)
        if mask is not None:
            h = torch.where(mask.unsqueeze(-1), self.mask_token, h)
        h = h + self.pos[:, : h.shape[1]]
        h = self.encoder(h)
        emb = self.embed_head(h.mean(dim=1))
        recon = self.recon_head(h)
        return emb, recon


def make_mask(batch: int, window: int, ratio: float, generator: torch.Generator) -> torch.Tensor:
    """各ウィンドウの frame を ratio だけランダムにマスクする bool [B, W]。"""
    r = torch.rand(batch, window, generator=generator)
    mask = r < ratio
    mask[:, 0] = False  # 全マスクを避けるため最低 1 frame は残す
    return mask
