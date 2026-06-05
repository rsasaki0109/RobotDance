#!/usr/bin/env python3
"""複数の OSS 2D pose 検出器を同じ実動画で走らせ、横並び overlay GIF で比較する。

RobotDance の抽出は MediaPipe Pose（3D world landmarks）が既定だが、pose 検出器は色々ある。
本スクリプトは **MediaPipe / YOLO11-pose(Ultralytics) / RTMPose(rtmlib)** を同一クリップに当て、
各検出器の骨格を **共通の COCO-17 表現**に揃えて原フレームへ重ね、3 パネルの比較 GIF を書き出す。
検出率・平均 confidence・推論時間も集計して表示する。

比較ロジックは `robotdance_perception.compare.compare_backends`（CLI `pose-compare` と共通）に集約。
検出器・COCO-17 表現・ランナーは `robotdance_perception.backends` レジストリを単一情報源とする。
未導入の検出器は自動でスキップし、何を飛ばしたか表示する。

⚠️ 入力動画は repo に同梱しない（license-safe）。出力は overlay（ソース動画ピクセルを含む派生物 →
CC-BY 等の出典明記で利用）。MediaPipe のみ 3D world landmarks を返し robot retarget に使える。
YOLO/RTMPose は 2D で、3D 化には別途 lifting が要る（本比較は検出品質の確認が目的）。

依存（dev のみ・パッケージ依存ではない）: mediapipe, ultralytics, rtmlib, opencv-python, imageio。

使い方:
    python scripts/compare_pose_backends.py clip.mp4 -o assets/readme/pose_compare.gif
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from robotdance_perception.compare import compare_backends  # noqa: E402


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("video", type=Path)
    ap.add_argument("-o", "--out", type=Path, default=Path("assets/readme/pose_compare.gif"))
    ap.add_argument("--stride", type=int, default=3)
    ap.add_argument("--width", type=int, default=300, help="各パネルのリサイズ幅")
    args = ap.parse_args()

    r = compare_backends(args.video, out_gif=args.out, stride=args.stride, width=args.width)
    if r["skipped"]:
        print(f"⚠️ 未導入のためスキップ: {', '.join(r['skipped'])}")
    if r["out_gif"]:
        kb = Path(r["out_gif"]).stat().st_size // 1024
        print(f"✓ {r['n_frames']} frames → {r['out_gif']} ({kb} KB)\n")
    print(f"{'backend':14s} {'det_rate':>8s} {'mean_conf':>10s} {'ms/frame':>9s}")
    for k, m in r["metrics"].items():
        print(f"{k:14s} {m['det_rate']:8.2f} {m['mean_conf']:10.3f} {m['ms_per_frame']:9.0f}")


if __name__ == "__main__":
    main()
