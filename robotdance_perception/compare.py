"""複数 pose 検出バックエンドを同一動画で比較する（overlay GIF + 指標）。

`backends` レジストリの全 available 検出器を回し、各骨格を共通 COCO-17 で原フレームへ重ねた
横並び overlay を作り、検出率・平均 confidence・推論時間を集計する。CLI `pose-compare` と
`scripts/compare_pose_backends.py` の共通実装。

⚠️ ライセンス: overlay は**ソース動画ピクセルを含む派生物**（CC-BY 等の出典明記で利用可）。
入力動画は repo に同梱しない。heavy 依存（cv2/imageio/各検出器）は本関数の呼び出し時のみ遅延 import。
"""

from __future__ import annotations

import time
import warnings
from pathlib import Path
from typing import Optional

import numpy as np

from robotdance_perception.backends import COCO_EDGES, list_backends, make_runner_2d

# overlay のバックエンド別パネル色（BGR）。
PANEL_COLORS = {
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


def compare_backends(
    video: str | Path,
    *,
    out_gif: Optional[str | Path] = None,
    stride: int = 3,
    width: int = 300,
) -> dict:
    """available な全 pose backend を同一動画で比較する。

    返り値: {"metrics": {name: {det_rate, mean_conf, ms_per_frame}}, "skipped": [...],
             "n_frames": int, "out_gif": str|None}。out_gif 指定時は overlay GIF を書き出す。
    """
    import cv2

    warnings.filterwarnings("ignore")

    # 比較は 2D COCO-17 を出す検出器のみ（lift 派生・import 系 backend は 2D ランナーが無いので除外）。
    runners, skipped = {}, []
    for b in list_backends():
        if b.lift_from or b.extract_mode != "video":
            continue
        (runners.__setitem__(b.name, make_runner_2d(b.name)) if b.available()
         else skipped.append(b.name))
    if not runners:
        raise RuntimeError("利用可能な pose backend がありません（mediapipe 等を入れてください）。")

    stats = {k: {"det": 0, "conf": 0.0, "ms": 0.0} for k in runners}
    cap = cv2.VideoCapture(str(video))
    if not cap.isOpened():
        raise FileNotFoundError(f"動画を開けません: {video}")
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    frames_out: list[np.ndarray] = []
    n_seen, idx = 0, 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        if idx % stride == 0:
            n_seen += 1
            panels = {}
            for name, run in runners.items():
                t = time.time()
                panels[name] = run(frame, idx, fps)
                stats[name]["ms"] += (time.time() - t) * 1000
            if out_gif is not None:
                tiles = []
                for name in runners:
                    tile = frame.copy()
                    res = panels[name]
                    color = PANEL_COLORS.get(name, (200, 200, 200))
                    if res is not None:
                        _draw(tile, res[0], res[1], color)
                    _label(tile, name, color)
                    scale = width / tile.shape[1]
                    tiles.append(cv2.resize(tile, (width, int(tile.shape[0] * scale))))
                frames_out.append(cv2.cvtColor(np.hstack(tiles), cv2.COLOR_BGR2RGB))
            for name in runners:
                res = panels[name]
                if res is not None:
                    stats[name]["det"] += 1
                    stats[name]["conf"] += float(res[1].mean())
        idx += 1
    cap.release()

    if n_seen == 0:
        raise RuntimeError(f"フレームを読めませんでした: {video}")

    out_path = None
    if out_gif is not None and frames_out:
        import imageio.v2 as imageio

        out_path = Path(out_gif)
        out_path.parent.mkdir(parents=True, exist_ok=True)
        imageio.mimsave(out_path, frames_out, duration=stride / fps, loop=0)

    metrics = {}
    for k, s in stats.items():
        d = s["det"]
        metrics[k] = {
            "det_rate": round(d / n_seen, 3),
            "mean_conf": round(s["conf"] / d, 3) if d else 0.0,
            "ms_per_frame": round(s["ms"] / n_seen, 1),
        }
    return {
        "metrics": metrics,
        "skipped": skipped,
        "n_frames": n_seen,
        "out_gif": str(out_path) if out_path else None,
    }
