"""決定的ハッシュ n-gram テキスト特徴（v0, 依存なし）。

caption / action label を固定長の bag-of-features ベクトルに符号化する。事前学習言語モデルを
使わず、`hashlib` による安定ハッシュで unigram + bigram を固定次元へ振り分ける（プロセス間で
決定的: Python 組み込み `hash()` は PYTHONHASHSEED で攪乱されるため使わない）。

⚠️ v0: contrastive text-motion の **テキスト側の足場**。語の同義性は学習側の MLP が吸収する
想定で、ここでは「同じ語 → 同じ次元」の決定的な疎特徴のみを提供する。事前学習エンコーダ
（CLIP/sentence-transformers 等）への差し替えは将来。
"""

from __future__ import annotations

import hashlib
import re

import numpy as np

TEXT_DIM = 256

_TOKEN_RE = re.compile(r"[a-z0-9]+")


def tokenize(text: str) -> list[str]:
    """小文字化して英数字トークンに分割する（記号は区切り）。"""
    return _TOKEN_RE.findall(text.lower())


def _bucket(token: str) -> int:
    """トークンを安定ハッシュで [0, TEXT_DIM) のバケットに写す。"""
    digest = hashlib.blake2b(token.encode("utf-8"), digest_size=8).digest()
    return int.from_bytes(digest, "big") % TEXT_DIM


def text_features(text: str) -> np.ndarray:
    """caption を L2 正規化済みの bag-of-(unigram, bigram) 特徴ベクトル [TEXT_DIM] にする。

    unigram と bigram の両方をハッシュして加算し、語順の手掛かりを少しだけ残す。
    空文字や未知語のみでも zero ベクトルを返す（学習側で扱える）。
    """
    toks = tokenize(text)
    vec = np.zeros(TEXT_DIM, dtype=np.float32)
    if not toks:
        return vec
    for tok in toks:
        vec[_bucket(tok)] += 1.0
    for a, b in zip(toks[:-1], toks[1:]):
        vec[_bucket(f"{a}_{b}")] += 1.0
    norm = float(np.linalg.norm(vec))
    return vec / norm if norm > 0 else vec
