"""RD-MIR コレクションの near-duplicate 除去（汎用, v0）。

manifest 駆動ビルド（`robotdance_data.dataset`）だけでなく、text-motion adapter
（HumanML3D / BABEL / Motion-X）等で得た **任意の RD-MIR リスト**に対し、motion embedding の
cosine 類似度で near-duplicate をグループ化し、各グループ 1 本（最長フレーム）を代表として残す。
ファイル I/O を持たない純粋関数なので、どの入口の出力にも適用できる。

⚠️ v0: 重複判定は手作り motion embedding（位置/向き/スケール不変）。motion_id は collection 内で
**一意**であることを前提とする。
"""

from __future__ import annotations

from typing import Any

from robotdance_core.rd_mir import RdMir


def dedupe_groups(mirs: list[RdMir], *, threshold: float = 0.98) -> list[list[int]]:
    """RD-MIR リストを near-duplicate でグループ化し、各グループの index リストを返す。"""
    from robotdance_motion.embeddings import MotionIndex

    n = len(mirs)
    if n <= 1:
        return [[i] for i in range(n)]
    index = MotionIndex()
    for m in mirs:
        index.add_mir(m)
    idx_of = {m.motion_id: i for i, m in enumerate(mirs)}

    parent = list(range(n))

    def find(x: int) -> int:
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    for a, b, _ in index.duplicates(threshold):
        if a in idx_of and b in idx_of:
            parent[find(idx_of[a])] = find(idx_of[b])

    groups: dict[int, list[int]] = {}
    for i in range(n):
        groups.setdefault(find(i), []).append(i)
    return list(groups.values())


def dedupe_mirs(mirs: list[RdMir], *, threshold: float = 0.98) -> dict[str, Any]:
    """near-duplicate を除去し、{kept, removed, groups, ...} を返す。

    各グループの代表は **最長フレーム**の clip。kept は代表のみ（順序は入力準拠）。
    """
    groups = dedupe_groups(mirs, threshold=threshold)
    rep_idx: set[int] = set()
    group_info: list[dict[str, Any]] = []
    removed: list[str] = []
    for members in groups:
        rep = max(members, key=lambda i: mirs[i].num_frames)
        rep_idx.add(rep)
        group_info.append({
            "representative": mirs[rep].motion_id,
            "members": [mirs[i].motion_id for i in members],
            "size": len(members),
        })
        removed.extend(mirs[i].motion_id for i in members if i != rep)

    kept = [mirs[i] for i in range(len(mirs)) if i in rep_idx]
    return {
        "kept": kept,
        "removed": removed,
        "groups": group_info,
        "total": len(mirs),
        "kept_count": len(kept),
        "removed_count": len(removed),
        "threshold": threshold,
    }
