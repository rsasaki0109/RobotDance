"""原動画フレームに canonical skeleton を重ねる overlay ビューア（v0）。

RD-MIR の keypoints_2d（画像正規化座標）を使い、抽出元の動画にスケルトンを描いて
annotated GIF を書き出す。pose 抽出が実ピクセル上で正しいかを目視確認できる。
"""

from __future__ import annotations

from pathlib import Path

import numpy as np

from robotdance_core.rd_mir import RdMir
from robotdance_core.skeleton import BONES


def render_overlay(
    video_path: str | Path,
    mir: RdMir,
    out_path: str | Path,
    *,
    stride: int = 2,
    max_frames: int = 200,
) -> Path:
    """動画フレームに RD-MIR の 2D スケルトンを重ねて GIF 化する。"""
    import cv2
    import imageio.v2 as imageio

    if mir.keypoints_2d is None:
        raise ValueError("mir.keypoints_2d が無い（extract_motion で抽出した RD-MIR を渡す）")
    kp2d = np.asarray(mir.keypoints_2d, dtype=np.float64)  # [T, J, 3] 正規化 x,y,vis

    cap = cv2.VideoCapture(str(video_path))
    if not cap.isOpened():
        raise FileNotFoundError(f"動画を開けません: {video_path}")

    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    frames: list[np.ndarray] = []
    fi = 0
    while fi < kp2d.shape[0] and len(frames) < max_frames:
        ok, frame = cap.read()
        if not ok:
            break
        if fi % stride == 0:
            h, w = frame.shape[:2]
            pts = kp2d[fi]
            px = (pts[:, :2] * np.array([w, h])).astype(int)
            for child, parent in BONES:
                cv2.line(frame, tuple(px[child]), tuple(px[parent]), (0, 200, 255), 3, cv2.LINE_AA)
            for j in range(px.shape[0]):
                cv2.circle(frame, tuple(px[j]), 4, (0, 0, 255), -1, cv2.LINE_AA)
            frames.append(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        fi += 1
    cap.release()

    if not frames:
        raise RuntimeError("overlay 用フレームを生成できなかった")
    out_fps = max(1, round((mir.fps or 30.0) / stride))
    imageio.mimsave(out_path, frames, duration=1.0 / out_fps, loop=0)
    return out_path
