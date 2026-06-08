"""hero GIF 結合ユーティリティ（scripts/make_hero_gif）の smoke テスト。"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))


def _tiny_gif(path: Path, w: int, h: int, n: int = 5) -> None:
    imageio = pytest.importorskip("imageio.v2")
    frames = [np.full((h, w, 3), 40 * i % 255, np.uint8) for i in range(n)]
    imageio.mimsave(str(path), frames, duration=0.1, loop=0)


def test_make_hero_combines_two_gifs_side_by_side(tmp_path: Path) -> None:
    pytest.importorskip("cv2")
    pytest.importorskip("PIL")
    imageio = pytest.importorskip("imageio.v2")
    from scripts.make_hero_gif import make_hero

    # 高さの異なる 2 本（フレーム数も少し違う）。
    left = tmp_path / "l.gif"
    right = tmp_path / "r.gif"
    _tiny_gif(left, 200, 240, n=6)
    _tiny_gif(right, 360, 460, n=5)

    out = make_hero([(left, "left"), (right, "right")], tmp_path / "hero.gif", height=120)
    assert out.exists() and out.stat().st_size > 0

    frames = imageio.mimread(str(out))
    assert len(frames) == 5  # min(6, 5)
    fh, fw = frames[0].shape[:2]
    # 高さ = banner(24) + height(120)。
    assert fh == 120 + 24
    # 幅 = 左右パネルを height=120 にリサイズした幅 + gap(10)。
    lw = round(200 * 120 / 240)   # 100
    rw = round(360 * 120 / 460)   # 94
    assert fw == lw + 10 + rw


def test_make_hero_combines_n_panels(tmp_path: Path) -> None:
    pytest.importorskip("cv2")
    pytest.importorskip("PIL")
    imageio = pytest.importorskip("imageio.v2")
    from scripts.make_hero_gif import make_hero

    # 4 パネル（実動画 + 3 ロボット相当）。幅 = 各パネル幅 + 区切り(gap)×3。
    paths = []
    widths = [200, 360, 360, 360]
    for i, w in enumerate(widths):
        g = tmp_path / f"p{i}.gif"
        _tiny_gif(g, w, 460, n=5)
        paths.append((g, f"panel{i}"))

    out = make_hero(paths, tmp_path / "hero4.gif", height=120)
    frames = imageio.mimread(str(out))
    fh, fw = frames[0].shape[:2]
    assert fh == 120 + 24
    panel_w = [round(w * 120 / 460) for w in widths]
    assert fw == sum(panel_w) + 10 * (len(widths) - 1)
