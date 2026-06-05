#!/usr/bin/env python3
"""複数の OSS 2D pose 検出器を同じ実動画で走らせ、横並び overlay GIF で比較する。

RobotDance の抽出は MediaPipe Pose（3D world landmarks）が既定だが、pose 検出器は色々ある。
本スクリプトは **MediaPipe / YOLO11-pose(Ultralytics) / RTMPose(rtmlib)** を同一クリップに当て、
各検出器の骨格を **共通の COCO-17 表現**に揃えて原フレームへ重ね、3 パネルの比較 GIF を書き出す。
検出率・平均 confidence・推論時間も集計して表示する。

検出器・COCO-17 表現・ランナーは `robotdance_perception.backends` レジストリを単一情報源とする
（`make_runner_2d` / `COCO_EDGES`）。未導入の検出器は自動でスキップし、何を飛ばしたか表示する。

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
import time
import warnings
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
warnings.filterwarnings("ignore")

from robotdance_perception.backends import COCO_EDGES, list_backends, make_runner_2d  # noqa: E402

_COLORS = {
    "mediapipe": (80, 200, 120),
    "yolo11-pose": (255, 170, 0),
    "rtmpose": (180, 120, 255),
}


def _draw(frame: np.ndarray, xy: np.ndarray, conf: np.ndarray, color, thr: float = 0.3) -> None:
    import cv2

    for a, b in COCO_EDGES:
        if conf[a] > thr and conf[b] > thr:
            cv2.line(frame, tuple(xy[a].astype(int)), tuple(xy[b].astype(int)), color, 2,
                     cv2.LINE_AA)
    for i in range(17):
        if conf[i] > thr:
            cv2.circle(frame, tuple(xy[i].astype(int)), 3, (0, 0, 255), -1, cv2.LINE_AA)


def _label(frame: np.ndarray, text: str, color) -> None:
    import cv2

    cv2.rectangle(frame, (0, 0), (frame.shape[1], 24), (32, 32, 32), -1)
    cv2.putText(frame, text, (6, 17), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("video", type=Path)
    ap.add_argument("-o", "--out", type=Path, default=Path("assets/readme/pose_compare.gif"))
    ap.add_argument("--stride", type=int, default=3)
    ap.add_argument("--width", type=int, default=300, help="各パネルのリサイズ幅")
    args = ap.parse_args()

    import cv2
    import imageio.v2 as imageio

    # レジストリから 2D ランナーを生成。未導入の検出器はスキップし、何を飛ばしたか明示する。
    runners, skipped = {}, []
    for b in list_backends():
        if b.available():
            runners[b.name] = make_runner_2d(b.name)
        else:
            skipped.append(b.name)
    if skipped:
        print(f"⚠️ 未導入のためスキップ: {', '.join(skipped)}")
    if not runners:
        sys.exit("利用可能な pose backend がありません（mediapipe 等を入れてください）。")

    stats = {k: {"det": 0, "conf": 0.0, "ms": 0.0} for k in runners}
    n_seen = 0
    cap = cv2.VideoCapture(str(args.video))
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    frames_out, idx = [], 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        if idx % args.stride == 0:
            n_seen += 1
            panels = {}
            for name, run in runners.items():
                t = time.time()
                panels[name] = run(frame, idx, fps)
                stats[name]["ms"] += (time.time() - t) * 1000

            tiles = []
            for name in runners:
                tile = frame.copy()
                res = panels[name]
                if res is not None:
                    _draw(tile, res[0], res[1], _COLORS.get(name, (200, 200, 200)))
                    stats[name]["det"] += 1
                    stats[name]["conf"] += float(res[1].mean())
                _label(tile, name, _COLORS.get(name, (200, 200, 200)))
                scale = args.width / tile.shape[1]
                tiles.append(cv2.resize(tile, (args.width, int(tile.shape[0] * scale))))
            frames_out.append(cv2.cvtColor(np.hstack(tiles), cv2.COLOR_BGR2RGB))
        idx += 1
    cap.release()

    args.out.parent.mkdir(parents=True, exist_ok=True)
    imageio.mimsave(args.out, frames_out, duration=args.stride / fps, loop=0)
    print(f"✓ {len(frames_out)} frames → {args.out} ({args.out.stat().st_size // 1024} KB)\n")
    print(f"{'backend':14s} {'det_rate':>8s} {'mean_conf':>10s} {'ms/frame':>9s}")
    for k, s in stats.items():
        d = s["det"]
        print(f"{k:14s} {d / n_seen:8.2f} {(s['conf'] / d if d else 0):10.3f} {s['ms'] / n_seen:9.0f}")


if __name__ == "__main__":
    main()
