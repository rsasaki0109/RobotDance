"""BABEL (action labels for AMASS) → canonical RD-MIR ローダ（skeleton-first, v0）。

BABEL は **AMASS の各シーケンスに行動ラベル**（sequence-level / frame-level の action 注釈）を
付けたデータセット。モーション本体は持たず AMASS .npz を指す（`feat_p`）。本ローダは BABEL の
注釈から対応する AMASS を [既存の AMASS ローダ] で読み、**行動ラベルを `semantics` に格納**して
RD-MIR にする。これで「動き + 行動テキスト」の実データが既存パイプラインに乗る。

⚠️ ライセンス: BABEL ラベルは研究用途、モーションは AMASS（research_only）。既定 license_state は
"research_only"。AMASS .npz / SMPL model file は repo に含めない（利用者が各自取得）。
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Callable, Iterator, Optional

from robotdance_core.rd_mir import LicenseState, RdMir

from .amass import load_amass_npz


def _seq_labels(entry: dict[str, Any]) -> list[str]:
    """BABEL entry から sequence-level の行動ラベル群を取り出す（act_cat 優先, なければ raw_label）。"""
    seq = entry.get("seq_ann") or {}
    labels: list[str] = []
    for lab in seq.get("labels", []) or []:
        cats = lab.get("act_cat")
        if cats:
            labels.extend(str(c) for c in cats)
        elif lab.get("raw_label"):
            labels.append(str(lab["raw_label"]))
    return labels


def _frame_segments(entry: dict[str, Any]) -> list[dict[str, Any]]:
    """frame-level の行動セグメント（{label, start_t, end_t}）を取り出す。"""
    fr = entry.get("frame_ann") or {}
    segs: list[dict[str, Any]] = []
    for lab in fr.get("labels", []) or []:
        cats = lab.get("act_cat")
        name = (str(cats[0]) if cats else str(lab.get("raw_label", "unknown")))
        segs.append({"label": name, "start_t": lab.get("start_t"), "end_t": lab.get("end_t")})
    return segs


def babel_entry_to_mir(
    entry: dict[str, Any], amass_root: str | Path, *,
    loader: Callable[..., RdMir] = load_amass_npz,
    license_state: LicenseState = "research_only",
    motion_id: Optional[str] = None,
) -> RdMir:
    """1 つの BABEL entry → RD-MIR（AMASS を読み行動ラベルを semantics に付与）。"""
    feat_p = entry.get("feat_p")
    if not feat_p:
        raise ValueError("BABEL entry に feat_p（AMASS パス）がありません")
    npz = Path(amass_root) / feat_p
    sid = str(entry.get("babel_sid", Path(feat_p).stem))
    mir = loader(npz, license_state=license_state, motion_id=motion_id or f"rdmir-babel-{sid}")

    from robotdance_core.semantics import build_semantics

    labels = _seq_labels(entry)
    mir.semantics = build_semantics(
        action_label=labels[0] if labels else "unknown",
        captions=labels,
        segments=_frame_segments(entry),     # 構造化セグメント [{label, start_t, end_t}]
        source_dataset="babel",
        babel_labels=labels,
        babel_sid=sid,
    )
    src = dict(mir.source_ref or {})
    src["babel_sid"] = sid
    mir.source_ref = src
    return mir


def iter_babel(
    babel_json: str | Path, amass_root: str | Path, *,
    loader: Callable[..., RdMir] = load_amass_npz,
    limit: Optional[int] = None,
) -> Iterator[RdMir]:
    """BABEL の JSON（dict keyed by babel_sid）→ RD-MIR を逐次生成する。

    AMASS .npz が見つからない entry はスキップする（欠損はそのまま進む）。
    """
    data = json.loads(Path(babel_json).read_text(encoding="utf-8"))
    entries = data.values() if isinstance(data, dict) else data
    yielded = 0
    for entry in entries:
        if limit is not None and yielded >= limit:
            return
        feat_p = entry.get("feat_p")
        if not feat_p or not (Path(amass_root) / feat_p).exists():
            continue
        yield babel_entry_to_mir(entry, amass_root, loader=loader)
        yielded += 1


def load_babel(
    babel_json: str | Path, amass_root: str | Path, *, limit: Optional[int] = None
) -> list[RdMir]:
    """BABEL の JSON → RD-MIR リスト（AMASS が見つかるものだけ）。"""
    return list(iter_babel(babel_json, amass_root, limit=limit))
