#!/usr/bin/env python3
"""2 本の GIF を共通の高さに揃え、ラベル付きで横並び結合して 1 本の hero GIF を作る。

README 冒頭の hero（実 karate overlay ｜ 実 G1）を**再現可能**にするためのユーティリティ。
入力 GIF は同一 extract・同一 stride で生成された同期済みのものを想定（フレーム数が揃っている）。
出力は PIL の adaptive palette で減色して軽量化する（実写を含む左パネルは GIF 圧縮が効きにくいため）。

⚠️ 入力 GIF はパイプライン出力（overlay は CC-BY 出典明記で利用可）。生動画は同梱しない。

使い方:
    python scripts/make_hero_gif.py \
        assets/readme/real/karate3_g1_overlay.gif "real video + skeleton overlay" \
        assets/readme/real/karate3_g1_robot.gif   "Unitree G1 reproduces it" \
        -o assets/readme/karate_hero.gif
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

# 左右パネルのラベル色（BGR ではなく RGB。cv2.putText には RGB 画像にそのまま描く）。
_LEFT_COLOR = (120, 200, 255)
_RIGHT_COLOR = (160, 230, 160)


def _panel(img: np.ndarray, label: str, color, height: int, banner: int) -> np.ndarray:
    import cv2

    img = img[:, :, :3]
    h, w = img.shape[:2]
    w2 = int(round(w * height / h))
    resized = cv2.resize(img, (w2, height), interpolation=cv2.INTER_AREA)
    bar = np.full((banner, w2, 3), 28, np.uint8)
    cv2.putText(bar, label, (5, banner - 7), cv2.FONT_HERSHEY_SIMPLEX, 0.42, color, 1, cv2.LINE_AA)
    return np.vstack([bar, resized])


def make_hero(panels: list[tuple[Path, str]], out: Path, *, height: int = 300, gap: int = 10,
              colors: int = 48, banner: int = 24, duration: float = 0.1) -> Path:
    """N 本の GIF を横並び結合して out に書き出す。書き出した Path を返す。

    panels: (gif_path, label) の列。先頭パネル（実動画 overlay 想定）は左色、残りはロボット色。
    全 GIF は同一 extract・同一 stride 想定（フレーム数は最短に揃える）。
    """
    import imageio.v2 as imageio
    from PIL import Image

    # HD 動画由来の overlay GIF は数十 MB になり得る（パネルは後で height へ縮小されるので
    # 最終 GIF は軽量）。imageio の 256MB デコード上限に当たらないよう memtest を外す。
    seqs = [imageio.mimread(str(g), memtest=False) for g, _ in panels]
    n = min(len(s) for s in seqs)
    frames = []
    for i in range(n):
        cells = []
        for j, ((_, label), seq) in enumerate(zip(panels, seqs)):
            color = _LEFT_COLOR if j == 0 else _RIGHT_COLOR
            cell = _panel(seq[i], label, color, height, banner)
            if cells:
                cells.append(np.full((cell.shape[0], gap, 3), 255, np.uint8))  # 区切り
            cells.append(cell)
        frames.append(np.hstack(cells))
    ims = [Image.fromarray(f).convert("P", palette=Image.ADAPTIVE, colors=colors) for f in frames]
    out.parent.mkdir(parents=True, exist_ok=True)
    ims[0].save(out, save_all=True, append_images=ims[1:],
                duration=int(duration * 1000), loop=0, optimize=True)
    return out


def main() -> None:
    ap = argparse.ArgumentParser(
        description="N 本の GIF を横並び結合して hero GIF を作る。--panel GIF LABEL を複数指定。")
    ap.add_argument("--panel", nargs=2, action="append", metavar=("GIF", "LABEL"),
                    required=True, help="パネルの (GIF, ラベル)。先頭が左（実動画）。複数指定可。")
    ap.add_argument("-o", "--out", type=Path, default=Path("assets/readme/karate_hero.gif"))
    ap.add_argument("--height", type=int, default=300)
    ap.add_argument("--colors", type=int, default=48)
    args = ap.parse_args()

    panels = [(Path(g), label) for g, label in args.panel]
    out = make_hero(panels, args.out, height=args.height, colors=args.colors)
    print(f"✓ hero GIF → {out} ({out.stat().st_size // 1024} KB)")


if __name__ == "__main__":
    main()
