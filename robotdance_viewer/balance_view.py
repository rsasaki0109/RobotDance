"""sim_certificate のバランス（ZMP × 支持多角形）を上面図 PNG に描くビューア（v0）。

`balance_violation_ratio` の単一数値では「どこで・なぜ支持外なのか」が見えない。本ビューアは
`simulate_certificate(..., return_trace=True)` の trace（ZMP 軌跡・各フレームの支持多角形・in/out
判定）を上面図に描き、ZMP が支持多角形を出る瞬間を可視化する（トルク可視化 v0.64-66 と対の balance 版）。

trace は certificate と同じ値（single source of truth）。可視化が ZMP/多角形を再計算しないので、
図と verdict が必ず一致する。
"""

from __future__ import annotations

import warnings
from pathlib import Path

import matplotlib

matplotlib.use("Agg")  # ヘッドレス環境でのレンダリング
warnings.filterwarnings("ignore", message="Unable to import Axes3D")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402


def render_balance_plot(
    trace: dict,
    out_path: str | Path,
    *,
    title: str | None = None,
    stride: int = 1,
    dpi: int = 110,
) -> Path:
    """certificate trace を上面図（x-forward 横軸 / y-left 縦軸）の PNG に描く。

    - 薄いグレー多角形: 各フレームの支持多角形（接地足フットプリントの凸包）。
    - 線: ZMP 軌跡。点: 各フレームの ZMP（支持内=緑 / 支持外=赤）。
    - 赤点が出れば「その瞬間に ZMP が支持を外れた＝転倒リスク」を意味する。
    stride で間引いて多角形の重なりを減らせる。
    """
    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    zmp = np.asarray(trace["zmp_xy"], dtype=float)        # [n, 2]
    polys = trace["support_polys"]                         # list[list[[x,y]]]
    in_support = np.asarray(trace["in_support"], dtype=bool)
    n = len(zmp)

    fig, ax = plt.subplots(figsize=(5, 5), dpi=dpi)
    # 支持多角形（凸包順に描く）。
    for f in range(0, n, max(stride, 1)):
        pts = np.asarray(polys[f], dtype=float) if polys[f] else None
        if pts is None or len(pts) < 3:
            continue
        hull = _convex_hull(pts)
        ax.fill(hull[:, 0], hull[:, 1], facecolor="0.85", edgecolor="0.6",
                linewidth=0.5, alpha=0.5, zorder=1)

    # ZMP 軌跡（時系列の線）と in/out 点。
    ax.plot(zmp[:, 0], zmp[:, 1], color="0.4", linewidth=0.8, zorder=2)
    ok = in_support
    ax.scatter(zmp[ok, 0], zmp[ok, 1], s=14, c="#2ca02c", label="ZMP in support", zorder=3)
    if (~ok).any():
        ax.scatter(zmp[~ok, 0], zmp[~ok, 1], s=22, c="#d62728",
                   marker="x", label="ZMP out (fall risk)", zorder=4)

    # 軸範囲は支持多角形（足の位置）から決める。ZMP は準静的式の特性上、鉛直加速度が大きい瞬間に
    # 大きく外れる（転倒リスクの表現として正しい）が、autoscale だと足が潰れて読めないため固定する。
    foot_pts = np.array([pt for f in range(n) for pt in (polys[f] or [])], dtype=float)
    if len(foot_pts):
        cx, cy = foot_pts[:, 0].mean(), foot_pts[:, 1].mean()
        span_x = foot_pts[:, 0].max() - foot_pts[:, 0].min()
        span_y = foot_pts[:, 1].max() - foot_pts[:, 1].min()
        half = max(span_x, span_y) * 0.5 + 0.12
        ax.set_xlim(cx - half, cx + half)
        ax.set_ylim(cy - half, cy + half)

    viol = float((~ok).mean()) if n else 0.0
    ax.set_aspect("equal")
    ax.set_xlabel("x forward [m]")
    ax.set_ylabel("y left [m]")
    base = title or "balance: ZMP vs support polygon"
    ax.set_title(f"{base}  (out of support {viol:.0%})", fontsize=10)
    ax.legend(loc="upper right", fontsize=7, framealpha=0.9)
    ax.grid(True, linewidth=0.3, alpha=0.4)

    fig.tight_layout()
    fig.savefig(out_path)
    plt.close(fig)
    return out_path


def _convex_hull(pts: np.ndarray) -> np.ndarray:
    """2D 凸包（Andrew monotone chain）。描画用に閉路（先頭点を末尾へ）で返す。"""
    p = np.unique(pts, axis=0)
    if len(p) <= 2:
        return np.vstack([p, p[:1]])
    p = p[np.lexsort((p[:, 1], p[:, 0]))]

    def cross2(o: np.ndarray, a: np.ndarray, b: np.ndarray) -> float:
        # 2D 外積（スカラ）。np.cross の 2D 形は NumPy 2.0 で非推奨のため明示計算。
        return float((a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]))

    def half(points: np.ndarray) -> list:
        h: list = []
        for q in points:
            while len(h) >= 2 and cross2(h[-2], h[-1], q) <= 0:
                h.pop()
            h.append(q)
        return h[:-1]

    hull = half(p) + half(p[::-1])
    arr = np.asarray(hull)
    return np.vstack([arr, arr[:1]])
