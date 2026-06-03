"""Text-conditioned motion generation — テキストからモーションを生成する（v0）。

これまでの 3 つを 1 本に繋ぐ:
  - `text.py`      : caption → 決定的ハッシュ n-gram テキスト特徴
  - `tokenizer.py` : motion ⇄ 離散トークン列（VQ-VAE）
  - `prior.py`     : トークン列の autoregressive 生成

本モジュールは token prior を**テキスト特徴で条件付け**する。caption の特徴ベクトルを系列先頭の
conditioning トークン（連続ベクトル）として与え、causal Transformer がそれに沿ったトークン列を
生成する。decode すれば caption に対応するモーションになる（"a backflip" → バックフリップ）。

⚠️ v0: 小さな合成 corpus で学習。caption の **action 群**（dance / idle / backflip）に応じて生成が
変わることを示すが、語彙・多様性・新規 caption 汎化は限定的。**生成物は物理的に妥当とは限らない**
— retarget → sim_certificate（MuJoCo）で必ず検証する。
"""

from __future__ import annotations

from pathlib import Path
from typing import Optional

import numpy as np
import torch
from torch import nn
from torch.nn import functional as F

from robotdance_core.rd_mir import RdMir

from .text import TEXT_DIM, text_features
from .tokenizer import MotionTokenizer


class ConditionalMotionPrior(nn.Module):
    """テキスト特徴で条件付けする causal Transformer。

    系列は [cond(text), emb(BOS), emb(t0), ...] で、cond 以降の各位置で次トークンを予測する。
    """

    def __init__(self, *, vocab: int, d_model: int = 128, nhead: int = 4,
                 nlayers: int = 3, max_len: int = 64) -> None:
        super().__init__()
        self.vocab = vocab
        self.max_len = max_len
        self.token_emb = nn.Embedding(vocab, d_model)
        self.text_proj = nn.Sequential(
            nn.Linear(TEXT_DIM, d_model), nn.ReLU(), nn.Linear(d_model, d_model)
        )
        self.pos = nn.Parameter(torch.zeros(1, max_len + 1, d_model))
        layer = nn.TransformerEncoderLayer(d_model, nhead, dim_feedforward=4 * d_model,
                                           batch_first=True, dropout=0.0)
        self.encoder = nn.TransformerEncoder(layer, nlayers)
        self.head = nn.Linear(d_model, vocab)

    def forward(self, tokens: torch.Tensor, text_feats: torch.Tensor) -> torch.Tensor:
        """tokens [B, L]（先頭 BOS 込み）, text_feats [B, TEXT_DIM] → logits [B, L, vocab]。"""
        b, length = tokens.shape
        cond = self.text_proj(text_feats).unsqueeze(1)        # [B, 1, d]
        h = torch.cat([cond, self.token_emb(tokens)], dim=1)  # [B, 1+L, d]
        h = h + self.pos[:, : length + 1]
        mask = torch.triu(torch.ones(length + 1, length + 1, device=tokens.device,
                                     dtype=torch.bool), diagonal=1)
        h = self.encoder(h, mask=mask)
        return self.head(h[:, 1:])                            # cond 位置の出力は捨てる


def _conditional_sequences(tok: MotionTokenizer, seq_len: int):
    """ラベル付き corpus を tokenize し (token_seqs [N, seq_len], text_feats [N, TEXT_DIM]) を作る。"""
    from .contrastive import build_labeled_corpus

    seqs: list[np.ndarray] = []
    feats: list[np.ndarray] = []
    for mir, caption, _group in build_labeled_corpus():
        ids = tok.encode(mir)
        tf = text_features(caption)
        if len(ids) >= seq_len:
            for s in range(0, len(ids) - seq_len + 1):
                seqs.append(ids[s:s + seq_len])
                feats.append(tf)
        else:
            pad = np.concatenate([ids, np.repeat(ids[-1:], seq_len - len(ids))])
            seqs.append(pad)
            feats.append(tf)
    return np.stack(seqs).astype(np.int64), np.stack(feats).astype(np.float32)


def train_text2motion(
    *,
    tokenizer_ckpt: str | Path = "motion_tokenizer.pt",
    out_path: str | Path = "text2motion.pt",
    seq_len: int = 16,
    epochs: int = 400,
    batch_size: int = 32,
    lr: float = 3e-4,
    seed: int = 0,
    device: Optional[str] = None,
) -> dict:
    """ラベル付き corpus で text-conditioned prior を学習し checkpoint を保存する。"""
    dev = device or ("cuda" if torch.cuda.is_available() else "cpu")
    gen = torch.Generator().manual_seed(seed)
    torch.manual_seed(seed)

    tok = MotionTokenizer(tokenizer_ckpt, device=dev)
    num_codes = tok.num_codes
    bos = num_codes
    vocab = num_codes + 1

    seq_np, feat_np = _conditional_sequences(tok, seq_len)
    seqs = torch.from_numpy(seq_np).to(dev)                  # [N, L]
    feats = torch.from_numpy(feat_np).to(dev)               # [N, TEXT_DIM]
    n = seqs.shape[0]
    bos_col = torch.full((n, 1), bos, dtype=torch.long, device=dev)
    inp = torch.cat([bos_col, seqs[:, :-1]], dim=1)         # [BOS, t0..t_{L-2}]
    tgt = seqs

    model = ConditionalMotionPrior(vocab=vocab, max_len=seq_len).to(dev)
    opt = torch.optim.Adam(model.parameters(), lr=lr)

    history: list[float] = []
    model.train()
    for _ in range(epochs):
        perm = torch.randperm(n, generator=gen)
        epoch_loss = 0.0
        for s in range(0, n, batch_size):
            b = perm[s:s + batch_size]
            logits = model(inp[b], feats[b])
            loss = F.cross_entropy(logits.reshape(-1, vocab), tgt[b].reshape(-1))
            opt.zero_grad()
            loss.backward()
            opt.step()
            epoch_loss += loss.item() * b.shape[0]
        history.append(epoch_loss / n)

    model.eval()
    with torch.no_grad():
        acc = float((model(inp, feats).argmax(-1) == tgt).float().mean().item())

    ckpt = {
        "state_dict": model.state_dict(),
        "config": {"vocab": vocab, "num_codes": num_codes, "bos": bos, "seq_len": seq_len},
        "tokenizer_ckpt": str(tokenizer_ckpt),
        "loss_history": history,
        "next_token_acc": acc,
    }
    torch.save(ckpt, str(out_path))
    return {
        "loss_history": history,
        "next_token_acc": acc,
        "checkpoint": str(out_path),
        "sequences": n,
        "vocab": vocab,
        "device": dev,
    }


class TextToMotion:
    """tokenizer + conditional prior。caption からモーションを生成する。"""

    def __init__(self, checkpoint: str | Path = "text2motion.pt",
                 tokenizer_ckpt: str | Path | None = None,
                 device: Optional[str] = None):
        self.device = device or ("cuda" if torch.cuda.is_available() else "cpu")
        ckpt = torch.load(str(checkpoint), map_location=self.device, weights_only=False)
        cfg = ckpt["config"]
        self.num_codes = cfg["num_codes"]
        self.bos = cfg["bos"]
        self.seq_len = cfg["seq_len"]
        self.model = ConditionalMotionPrior(vocab=cfg["vocab"], max_len=self.seq_len).to(self.device)
        self.model.load_state_dict(ckpt["state_dict"])
        self.model.eval()
        self.tok = MotionTokenizer(tokenizer_ckpt or ckpt["tokenizer_ckpt"], device=self.device)

    @torch.no_grad()
    def generate(self, caption: str, *, length: int | None = None, temperature: float = 1.0,
                 seed: int = 0, fps: float = 30.0) -> RdMir:
        """caption に対応するモーションを生成する。"""
        gen = torch.Generator(device=self.device).manual_seed(seed)
        length = length or self.seq_len
        feats = torch.from_numpy(text_features(caption)[None, :]).to(self.device)
        seq = [self.bos]
        while len(seq) - 1 < length:
            ctx = torch.tensor([seq[-self.seq_len:]], device=self.device)
            logits = self.model(ctx, feats)[0, -1]
            logits[self.bos] = float("-inf")
            probs = F.softmax(logits / max(temperature, 1e-6), dim=-1)
            seq.append(int(torch.multinomial(probs, 1, generator=gen).item()))
        tokens = np.array(seq[1:length + 1], dtype=np.int64)
        return self.tok.decode_to_mir(tokens, fps=fps, motion_id="rdmir-text2motion")
