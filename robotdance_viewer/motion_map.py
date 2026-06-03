"""Motion Map — embedding の 2D 射影を散布図に描く（v0）。

類似動作が近く、異なる動作が離れて配置されることを可視化する（§6.2 Demo 3）。
"""

from __future__ import annotations

import warnings
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
warnings.filterwarnings("ignore", message="Unable to import Axes3D")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

_PALETTE = ["#1f77b4", "#ff7f0e", "#2ca02c", "#d62728", "#9467bd", "#8c564b", "#e377c2"]


def render_motion_map(
    points: np.ndarray,
    labels: list[str],
    out_path: str | Path,
    *,
    groups: list[str] | None = None,
    title: str = "RobotDance Motion Map",
    dpi: int = 110,
) -> Path:
    """2D 点 [N,2] を散布図に描く。groups（ラベルのカテゴリ）で色分けする。"""
    points = np.asarray(points, dtype=np.float64)
    groups = groups or ["motion"] * len(labels)
    uniq = sorted(set(groups))
    color_of = {g: _PALETTE[i % len(_PALETTE)] for i, g in enumerate(uniq)}

    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    fig, ax = plt.subplots(figsize=(6, 5), dpi=dpi)
    for g in uniq:
        mask = [gr == g for gr in groups]
        pts = points[np.array(mask)]
        ax.scatter(pts[:, 0], pts[:, 1], c=color_of[g], label=g, s=90, edgecolors="white", zorder=3)
    for (x, y), lab in zip(points, labels):
        ax.annotate(lab, (x, y), fontsize=7, xytext=(4, 4), textcoords="offset points")
    ax.set_title(title, fontsize=12)
    ax.set_xlabel("PC1")
    ax.set_ylabel("PC2")
    ax.legend(fontsize=8, loc="best")
    ax.grid(True, alpha=0.2)
    fig.tight_layout()
    fig.savefig(out_path)
    plt.close(fig)
    return out_path
