"""実動画から抽出した RD-MIR フィクスチャ（license-safe: 数値 motion のみ、ピクセル非同梱）。

README の karate kata / kathak クリップを MediaPipe で抽出し、HumanoidBattle の選択技として
同梱する。生動画は repo に含めない（出典は source_ref に明記）。
"""

from __future__ import annotations

from functools import lru_cache
from pathlib import Path
from typing import Callable

from robotdance_core.rd_mir import RdMir

_DATA = Path(__file__).resolve().parent / "data"

# style 名 → loader（battle / fight で共有）。
REAL_MOTION_NAMES = ("karate", "kathak")


@lru_cache(maxsize=len(REAL_MOTION_NAMES))
def _load(name: str) -> RdMir:
    path = _DATA / f"{name}.rdmir.json"
    if not path.is_file():
        raise FileNotFoundError(f"実動画フィクスチャがありません: {path}")
    return RdMir.load(path)


def load_karate() -> RdMir:
    """空手型（Wikimedia / Sdcsabac, CC BY-SA 4.0 由来の抽出 motion）。"""
    return _load("karate")


def load_kathak() -> RdMir:
    """カタック舞踊（Wikimedia / Suyash Dwivedi, CC BY-SA 4.0 由来の抽出 motion）。"""
    return _load("kathak")


REAL_MOTIONS: dict[str, Callable[[], RdMir]] = {
    "karate": load_karate,
    "kathak": load_kathak,
}


def get_real_motion(name: str) -> RdMir:
    """実動画由来の motion を返す。未知名は ValueError。"""
    if name not in REAL_MOTIONS:
        raise ValueError(f"未知の実動画 motion '{name}'（利用可能: {sorted(REAL_MOTIONS)}）")
    return REAL_MOTIONS[name]()


__all__ = ["REAL_MOTION_NAMES", "REAL_MOTIONS", "get_real_motion", "load_karate", "load_kathak"]
