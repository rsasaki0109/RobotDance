"""Contrastive text-motion アライメント（CLIP 風, v0）。

motion encoder（masked modeling の MotionEncoderNet を再利用）と text MLP を **共有埋め込み空間**
に射影し、対になる (motion, caption) を InfoNCE で引き寄せる。学習後は

    model.embed_text("a person doing a backflip") -> 共有空間の単位ベクトル
    model.embed_motion(rd_mir)                    -> 同じ空間の単位ベクトル

をコサイン類似度で比較でき、テキスト → モーション検索が可能になる。

⚠️ v0: 小さな合成 corpus・ハッシュ n-gram テキスト特徴（事前学習言語モデルなし）。
インタフェースと「toy データで text→motion 検索が成立する」ことの実証が目的で、実キャプション・
データ規模・事前学習エンコーダへの差し替えは将来（§4.2）。
"""

from __future__ import annotations

from pathlib import Path
from typing import Optional

import numpy as np
import torch
from torch import nn
from torch.nn import functional as F

from robotdance_core.rd_mir import RdMir
from robotdance_motion.embeddings import normalized_keypoints

from .encoder import MotionEncoderNet, window_motion
from .text import TEXT_DIM, text_features


def build_labeled_corpus() -> list[tuple[RdMir, str, str]]:
    """(合成モーション, caption, group) の学習ペア集合。決定的・権利クリーン。

    各 action に複数の言い回しを与え、ハッシュ n-gram + MLP が語の同義を吸収できるようにする。
    group は action 系統（dance_fast / dance_slow / idle / backflip）で、retrieval 評価に使う。
    """
    from robotdance_core.synthetic import generate_backflip, generate_dance

    pairs: list[tuple[RdMir, str, str]] = []

    def add(mir: RdMir, group: str, captions: list[str]) -> None:
        for cap in captions:
            pairs.append((mir, cap, group))

    # エネルギッシュなダンス。
    for bps in (1.0, 1.3, 1.6):
        add(generate_dance(beats_per_second=bps), "dance_fast", [
            "a person dancing energetically",
            "fast energetic dance",
            "someone is dancing",
            "an upbeat dance",
        ])
    # ゆったりしたダンス / sway。
    add(generate_dance(beats_per_second=0.7), "dance_slow", [
        "a slow gentle dance",
        "swaying slowly side to side",
        "a calm relaxed dance",
    ])
    # ほぼ静止（idle）。
    for arm, sway in ((0.15, 0.04), (0.20, 0.05)):
        add(generate_dance(beats_per_second=0.5, arm_amp=arm, sway_amp=sway), "idle", [
            "a person standing still",
            "standing almost motionless",
            "barely moving while standing",
            "an idle resting pose",
        ])
    # バックフリップ（アクロバット）。
    for dur in (1.4, 1.6, 1.9):
        add(generate_backflip(duration=dur), "backflip", [
            "a person doing a backflip",
            "flipping backwards in the air",
            "an acrobatic backflip",
            "a backward somersault",
        ])
    return pairs


class TextMotionNet(nn.Module):
    """motion branch（MotionEncoderNet → proj）と text branch（MLP）を共有空間に射影する。"""

    def __init__(self, *, shared_dim: int = 64, motion_emb_dim: int = 64) -> None:
        super().__init__()
        self.motion_net = MotionEncoderNet(emb_dim=motion_emb_dim)
        self.motion_proj = nn.Linear(motion_emb_dim, shared_dim)
        self.text_mlp = nn.Sequential(
            nn.Linear(TEXT_DIM, 128),
            nn.ReLU(),
            nn.Linear(128, shared_dim),
        )
        # 温度（学習可能 logit scale, CLIP と同様）。
        self.logit_scale = nn.Parameter(torch.tensor(float(np.log(1 / 0.07))))

    def encode_motion_windows(self, windows: torch.Tensor) -> torch.Tensor:
        """[N, W, INPUT_DIM] のウィンドウ群 → mean-pool した共有空間ベクトル [shared_dim]。"""
        emb, _ = self.motion_net(windows)          # [N, motion_emb_dim]
        pooled = emb.mean(dim=0, keepdim=True)      # [1, motion_emb_dim]
        z = self.motion_proj(pooled)                # [1, shared_dim]
        return F.normalize(z, dim=-1).squeeze(0)

    def encode_text(self, feats: torch.Tensor) -> torch.Tensor:
        """[B, TEXT_DIM] → 正規化済み共有ベクトル [B, shared_dim]。"""
        return F.normalize(self.text_mlp(feats), dim=-1)


def _motion_windows(mir: RdMir, window: int) -> np.ndarray:
    return window_motion(normalized_keypoints(mir), window, max(1, window // 2)).astype(np.float32)


def train_text_motion(
    *,
    out_path: str | Path = "text_motion.pt",
    window: int = 32,
    epochs: int = 120,
    lr: float = 1e-3,
    seed: int = 0,
    device: Optional[str] = None,
) -> dict:
    """合成 (motion, caption) ペアで CLIP 風 InfoNCE を学習し checkpoint を保存する。

    各 step で全ペアの motion / text を共有空間に符号化し、対角を正解とする対称交差エントロピーで
    motion↔text を整合させる。loss 履歴と最終 top-1 retrieval 精度を返す。
    """
    dev = device or ("cuda" if torch.cuda.is_available() else "cpu")
    torch.manual_seed(seed)

    pairs = build_labeled_corpus()
    # ユニークな motion を index 化し、ペアごとに motion id を持たせる。
    uniq_motions: list[RdMir] = []
    motion_key: dict[int, int] = {}
    pair_motion_idx: list[int] = []
    captions: list[str] = []
    motion_group: list[str] = []  # ユニーク motion ごとの action 群
    for mir, cap, group in pairs:
        key = id(mir)
        if key not in motion_key:
            motion_key[key] = len(uniq_motions)
            uniq_motions.append(mir)
            motion_group.append(group)
        pair_motion_idx.append(motion_key[key])
        captions.append(cap)

    # motion ウィンドウ（可変本数）と text 特徴を事前計算。
    win_tensors = [torch.from_numpy(_motion_windows(m, window)).to(dev) for m in uniq_motions]
    text_feat = torch.from_numpy(
        np.stack([text_features(c) for c in captions]).astype(np.float32)
    ).to(dev)
    midx = torch.tensor(pair_motion_idx, device=dev)
    n_pairs = len(captions)

    model = TextMotionNet().to(dev)
    opt = torch.optim.Adam(model.parameters(), lr=lr)

    n_motions = len(uniq_motions)
    # motion→text 用の multi-positive target（各 motion に属する caption へ均等分配）。
    pos_mask = (midx[None, :] == torch.arange(n_motions, device=dev)[:, None]).float()  # [M, P]
    target_dist = pos_mask / pos_mask.sum(dim=1, keepdim=True)

    history: list[float] = []
    model.train()
    for _ in range(epochs):
        m_uniq = torch.stack([model.encode_motion_windows(w) for w in win_tensors])  # [M, D]
        t_emb = model.encode_text(text_feat)                 # [P, D]
        scale = model.logit_scale.exp().clamp(max=100.0)

        # caption→motion: 各 caption の正解は単一 motion（dance variant 同士は真の負例）。
        logits_t2m = scale * t_emb @ m_uniq.t()              # [P, M]
        loss_t2m = F.cross_entropy(logits_t2m, midx)

        # motion→caption: 各 motion の正例はその全 caption（同義 caption を負例にしない）。
        logits_m2t = scale * m_uniq @ t_emb.t()              # [M, P]
        loss_m2t = -(target_dist * F.log_softmax(logits_m2t, dim=1)).sum(dim=1).mean()

        loss = 0.5 * (loss_t2m + loss_m2t)
        opt.zero_grad()
        loss.backward()
        opt.step()
        history.append(loss.item())

    # 学習後 retrieval: 各 caption から最も近い motion を引く。
    # exact = ちょうどそのペアの motion / group = 同じ action 群（dance variant 等は可換）。
    model.eval()
    with torch.no_grad():
        m_uniq = torch.stack([model.encode_motion_windows(w) for w in win_tensors])
        t_emb = model.encode_text(text_feat)
        pred = (t_emb @ m_uniq.t()).argmax(dim=1)            # [P]
        exact = float((pred == midx).float().mean().item())
        pred_group = [motion_group[i] for i in pred.tolist()]
        true_group = [motion_group[i] for i in pair_motion_idx]
        group_top1 = float(np.mean([p == t for p, t in zip(pred_group, true_group)]))

    ckpt = {
        "state_dict": model.state_dict(),
        "config": {"window": window},
        "loss_history": history,
        "train_top1": exact,
        "group_top1": group_top1,
    }
    torch.save(ckpt, str(out_path))
    return {
        "loss_history": history,
        "train_top1": exact,
        "group_top1": group_top1,
        "checkpoint": str(out_path),
        "pairs": n_pairs,
        "motions": len(uniq_motions),
        "device": dev,
    }


class TextMotionModel:
    """学習済み text-motion モデル。text / motion を同じ単位球面に埋め込む。"""

    def __init__(self, checkpoint: str | Path = "text_motion.pt", device: Optional[str] = None):
        self.device = device or ("cuda" if torch.cuda.is_available() else "cpu")
        ckpt = torch.load(str(checkpoint), map_location=self.device, weights_only=False)
        self.window = ckpt["config"]["window"]
        self.model = TextMotionNet().to(self.device)
        self.model.load_state_dict(ckpt["state_dict"])
        self.model.eval()

    @torch.no_grad()
    def embed_text(self, text: str) -> np.ndarray:
        feats = torch.from_numpy(text_features(text)[None, :]).to(self.device)
        return self.model.encode_text(feats).squeeze(0).cpu().numpy()

    @torch.no_grad()
    def embed_motion(self, mir: RdMir) -> np.ndarray:
        w = torch.from_numpy(_motion_windows(mir, self.window)).to(self.device)
        return self.model.encode_motion_windows(w).cpu().numpy()

    def search(self, query: str, motions: dict[str, RdMir], k: int = 5) -> list[tuple[str, float]]:
        """テキスト query に近い順に (motion_id, cosine) を最大 k 件返す。"""
        q = self.embed_text(query)
        scored = [(mid, float(np.dot(q, self.embed_motion(m)))) for mid, m in motions.items()]
        scored.sort(key=lambda kv: kv[1], reverse=True)
        return scored[:k]
