"""RD-MIR semantics の構造化（§3, v0）。

`RdMir.semantics` はこれまで自由 dict だった。本モジュールは推奨構造を pydantic で定義し、
build / validate のヘルパーを提供する:

  - action_label: 主たる行動ラベル（例: walk, dance）
  - style_tag:    スタイル/様式タグ（例: ballet, casual）
  - captions:     自然文記述（複数可）
  - segments:     フレーム/時間レベルの連続行動 [{label, start_t, end_t}]
  - source_dataset, その他は extra として保持

後方互換のため `RdMir.semantics` 自体は dict のまま（schema も additionalProperties=true）。
adapter は `build_semantics(...)` で正規化した dict を作る。
"""

from __future__ import annotations

from typing import Any, Optional

from pydantic import BaseModel, ConfigDict, Field


class Segment(BaseModel):
    """連続行動セグメント（時間レンジ + ラベル）。"""

    model_config = ConfigDict(extra="allow")
    label: str
    start_t: Optional[float] = None
    end_t: Optional[float] = None


class Semantics(BaseModel):
    """RD-MIR semantics の推奨構造（extra 許可）。"""

    model_config = ConfigDict(extra="allow")
    action_label: str = "unknown"
    style_tag: Optional[str] = None
    captions: list[str] = Field(default_factory=list)
    segments: list[Segment] = Field(default_factory=list)
    source_dataset: Optional[str] = None


def build_semantics(
    *,
    action_label: Optional[str] = None,
    style_tag: Optional[str] = None,
    captions: Optional[list[str]] = None,
    segments: Optional[list[dict[str, Any]]] = None,
    source_dataset: Optional[str] = None,
    **extra: Any,
) -> dict[str, Any]:
    """正規化した semantics dict を作る（segments は label 必須でバリデートされる）。

    action_label 未指定なら captions の先頭、無ければ "unknown"。
    """
    caps = list(captions or [])
    label = action_label or (caps[0] if caps else "unknown")
    sem = Semantics(
        action_label=label,
        style_tag=style_tag,
        captions=caps,
        segments=[Segment(**s) for s in (segments or [])],
        source_dataset=source_dataset,
        **extra,
    )
    return sem.model_dump(exclude_none=True)


def validate_semantics(sem: dict[str, Any]) -> Semantics:
    """semantics dict を Semantics として検証する（segments の label 必須等）。"""
    return Semantics.model_validate(sem)


def segment_labels(sem: Optional[dict[str, Any]]) -> list[str]:
    """semantics から segment ラベル列を取り出す（無ければ空）。"""
    if not sem:
        return []
    return [str(s.get("label")) for s in sem.get("segments", []) if s.get("label")]
