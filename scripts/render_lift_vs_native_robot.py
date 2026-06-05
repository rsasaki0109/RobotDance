#!/usr/bin/env python3
"""native 3D と coarse planar lift の両方を実 G1 メッシュに retarget し、横並び GIF にする。

同じ動画から 2 経路で RD-MIR を作り、それぞれ actuator-IK で実 Unitree の関節角へ落として
実メッシュを踊らせる:
- left  (native): MediaPipe world landmarks（深度あり）
- right (lift):   2D 検出器（YOLO11-pose 等）の COCO-17 → 解析的 planar lift（深度なし・x=0 平面）

狙い: 「2D 検出器だけでもロボットは踊る」を実証しつつ、native との差（特に深度）を並べて見せる。

⚠️ 入力動画は repo に同梱しない。GIF は retarget 関節角の可視化で動画ピクセルを含まない（license-safe）。

使い方:
    python scripts/render_lift_vs_native_robot.py karate.mp4 /path/g1_23dof.urdf \
        --detector yolo11-pose -o assets/readme/pose/lift_vs_native_robot.gif
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from scripts.render_real_video_gif import _render_mesh  # noqa: E402


def _retarget_angles(mir, urdf: Path, robot: str):
    from robotdance_retarget.actuator_ik import actuator_retarget
    from robotdance_unitree.urdf_import import G1_LINK_MAP, H1_LINK_MAP

    link_map = H1_LINK_MAP if robot == "h1" else G1_LINK_MAP
    motion = actuator_retarget(mir, str(urdf), steps=250,
                               link_map=link_map, robot_name=f"unitree_{robot}")
    angles = np.asarray(motion.joint_rotations["angles_rad"])
    names = [str(n) for n in motion.joint_rotations["actuated_joint_names"]]
    return angles, names, motion.fps, motion.retarget_metrics["ik_mean_pos_error_m"]


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("video", type=Path)
    ap.add_argument("urdf", type=Path)
    ap.add_argument("--robot", choices=["g1", "h1"], default="g1")
    ap.add_argument("--detector", default="yolo11-pose", help="lift 元の 2D 検出器")
    ap.add_argument("-o", "--out", type=Path,
                    default=Path("assets/readme/pose/lift_vs_native_robot.gif"))
    ap.add_argument("--stride", type=int, default=3)
    ap.add_argument("--width", type=int, default=300)
    ap.add_argument("--height", type=int, default=420)
    args = ap.parse_args()

    import imageio.v2 as imageio
    import pybullet as p

    from robotdance_perception.lifting import extract_via_lift
    from robotdance_perception.mediapipe_adapter import extract_motion

    print(f"native(MediaPipe) 抽出: {args.video.name} ...")
    native = extract_motion(args.video, smooth=True)
    print(f"lift({args.detector}+planar) 抽出 ...")
    lift = extract_via_lift(args.video, detector=args.detector, smooth=True)

    a_ang, a_names, a_fps, a_err = _retarget_angles(native, args.urdf, args.robot)
    b_ang, b_names, b_fps, b_err = _retarget_angles(lift, args.urdf, args.robot)
    print(f"retarget IK err: native={a_err} m  lift={b_err} m")

    # 同じ長さに切り詰めて横並びにする。
    t = min(len(a_ang), len(b_ang))
    a_ang, b_ang = a_ang[:t], b_ang[:t]
    base_z = 1.04 if args.robot == "h1" else 0.793

    p.connect(p.DIRECT)
    fa = _render_mesh(args.urdf, args.robot, base_z, a_ang, a_names,
                      a_fps, args.stride, args.width, args.height)
    fb = _render_mesh(args.urdf, args.robot, base_z, b_ang, b_names,
                      b_fps, args.stride, args.width, args.height)
    p.disconnect()

    import cv2

    n = min(len(fa), len(fb))
    label_h = 24
    pw = args.width
    frames = []
    for i in range(n):
        pair = np.hstack([fa[i], fb[i]])
        banner = np.full((label_h, pair.shape[1], 3), 32, dtype=np.uint8)
        cv2.putText(banner, "native (MediaPipe 3D)", (8, 17),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (120, 200, 255), 1, cv2.LINE_AA)
        cv2.putText(banner, f"lift ({args.detector}, planar)", (pw + 8, 17),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (120, 120, 255), 1, cv2.LINE_AA)
        frames.append(np.vstack([banner, pair]))

    args.out.parent.mkdir(parents=True, exist_ok=True)
    imageio.mimsave(args.out, frames, duration=1.0 / max(a_fps / args.stride, 1), loop=0)
    print(f"✓ robot side-by-side (native | lift) → {args.out} "
          f"({args.out.stat().st_size // 1024} KB)")


if __name__ == "__main__":
    main()
