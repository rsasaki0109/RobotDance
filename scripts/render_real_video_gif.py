#!/usr/bin/env python3
"""実動画 → MediaPipe 抽出 → retarget → 実ロボット mesh が同じ動きをする GIF を作る。

README の本命「Shorts to humanoid」hero 用（パッケージ本体ではない）。ライセンスがクリアな
short 動画（public domain / CC0 / CC-BY）をローカルで MediaPipe Pose にかけて canonical RD-MIR を復元し、
actuator-space IK で実 Unitree の関節角へ retarget、pybullet headless TinyRenderer で実メッシュを踊らせる。

出力は 2 本:
  - <out>_skeleton.gif: 抽出した canonical 19-joint スケルトン（人間の動きの復元）
  - <out>_robot.gif:    実ロボットメッシュが同じ動きをする様子

⚠️ **入力動画は repo に同梱しない / 再配布しない**（license-safe）。出力 GIF はパイプライン出力
（抽出 motion / actuator-IK 関節角）の可視化であり、source 動画のピクセルを含まない。
出典はユーザーが README に明記すること（CC-BY なら著者・ライセンス・出典 URL）。

依存（dev のみ）: mediapipe, opencv-python, pybullet, imageio, torch。

使い方:
    python scripts/render_real_video_gif.py clip.mp4 /path/to/g1_23dof.urdf \
        --robot g1 -o assets/readme/real_squat
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))


def _render_mesh(urdf: Path, robot: str, base_z: float, angles, names,
                 fps: float, stride: int, width: int, height: int) -> list:
    """関節角列を実メッシュでレンダリングし RGB フレーム列を返す（pybullet headless）。"""
    import pybullet as p

    urdf_dir = urdf.resolve().parent
    p.setAdditionalSearchPath(str(urdf_dir))
    gv = p.createVisualShape(p.GEOM_BOX, halfExtents=[3, 3, 0.01], rgbaColor=[0.93, 0.93, 0.95, 1])
    gc = p.createCollisionShape(p.GEOM_BOX, halfExtents=[3, 3, 0.01])
    p.createMultiBody(0, gc, gv, basePosition=[0, 0, -0.005])
    rid = p.loadURDF(urdf.name, useFixedBase=True, basePosition=[0, 0, base_z])

    jmap = {}
    for i in range(p.getNumJoints(rid)):
        info = p.getJointInfo(rid, i)
        if info[2] == p.JOINT_REVOLUTE:
            jmap[info[1].decode()] = i
    pairs = [(jmap[n], k) for k, n in enumerate(names) if n in jmap]

    proj = p.computeProjectionMatrixFOV(42, width / height, 0.1, 10)
    cam_target_z = base_z * 0.78
    cam_dist = base_z * 2.45
    frames = []
    t_len = angles.shape[0]
    for f in range(0, t_len, stride):
        for ji, k in pairs:
            p.resetJointState(rid, ji, float(angles[f, k]))
        yaw = 35 + 25 * np.sin(2 * np.pi * f / t_len)
        view = p.computeViewMatrixFromYawPitchRoll([0, 0, cam_target_z], cam_dist, yaw, -10, 0, 2)
        img = p.getCameraImage(width, height, view, proj, renderer=p.ER_TINY_RENDERER,
                               lightDirection=[0.6, 0.7, 1.2], shadow=1)
        frames.append(np.reshape(img[2], (height, width, 4))[:, :, :3].astype(np.uint8))
    return frames


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("video", type=Path, help="ライセンスがクリアな short 動画（ローカル, 非同梱）")
    ap.add_argument("urdf", type=Path, help="実 Unitree URDF（メッシュ付き, ローカル取得）")
    ap.add_argument("--robot", choices=["g1", "h1"], default="g1")
    ap.add_argument("-o", "--out", type=Path, default=Path("assets/readme/real_video"),
                    help="出力 prefix（_skeleton.gif / _robot.gif〔/ --overlay 時 _overlay.gif〕を付与）")
    ap.add_argument("--caption", default=None, help="スケルトン GIF に重ねる説明")
    ap.add_argument("--overlay", action="store_true",
                    help="原動画 + 2D 骨格 overlay GIF も出力（skeleton/robot と同期, 動画ピクセルを含む）")
    ap.add_argument("--base-z", type=float, default=None)
    ap.add_argument("--stride", type=int, default=3)
    ap.add_argument("--width", type=int, default=360)
    ap.add_argument("--height", type=int, default=460)
    args = ap.parse_args()

    import imageio.v2 as imageio
    import pybullet as p

    from robotdance_perception.mediapipe_adapter import extract_motion
    from robotdance_retarget.actuator_ik import actuator_retarget
    from robotdance_unitree.urdf_import import G1_LINK_MAP, H1_LINK_MAP
    from robotdance_viewer.skeleton_view import render_gif

    # 1. 実動画 → canonical RD-MIR（MediaPipe Pose, smoothing 込み）。
    mir = extract_motion(args.video, smooth=True)
    conf = np.array(mir.confidence["joint"])
    print(f"extract: {conf.shape[0]} frames, mean_conf={conf.mean():.3f}, "
          f"jitter_after={mir.quality_metrics.get('jitter_after')}")

    # 2. 抽出スケルトン GIF（人間の動きの復元を見せる）。
    skel_out = Path(str(args.out) + "_skeleton.gif")
    if args.caption is not None:
        mir.semantics = {**(mir.semantics or {}), "action_label": args.caption}
    render_gif(mir, skel_out, stride=args.stride, show_meta=True)
    print(f"✓ skeleton → {skel_out} ({skel_out.stat().st_size // 1024} KB)")

    # 2b. （任意）原動画 + 2D 骨格 overlay GIF。skeleton/robot と**同じ extract・同じ stride**から
    #     描くので、3 段（overlay → skeleton → robot）が完全に同期する。
    if args.overlay:
        from robotdance_viewer.overlay import render_overlay

        ov_out = Path(str(args.out) + "_overlay.gif")
        render_overlay(args.video, mir, ov_out, stride=args.stride)
        print(f"✓ overlay  → {ov_out} ({ov_out.stat().st_size // 1024} KB)")

    # 3. actuator-IK で実 Unitree の関節角へ → 実メッシュ render。
    link_map = H1_LINK_MAP if args.robot == "h1" else G1_LINK_MAP
    robot_name = f"unitree_{args.robot}"
    base_z = args.base_z if args.base_z is not None else (1.04 if args.robot == "h1" else 0.793)
    motion = actuator_retarget(mir, str(args.urdf), steps=250,
                               link_map=link_map, robot_name=robot_name)
    angles = np.asarray(motion.joint_rotations["angles_rad"])
    names = [str(n) for n in motion.joint_rotations["actuated_joint_names"]]
    print(f"retarget: IK err {motion.retarget_metrics['ik_mean_pos_error_m']} m")

    p.connect(p.DIRECT)
    frames = _render_mesh(args.urdf, args.robot, base_z, angles, names,
                          motion.fps, args.stride, args.width, args.height)
    p.disconnect()
    robot_out = Path(str(args.out) + "_robot.gif")
    imageio.mimsave(robot_out, frames, duration=1.0 / max(motion.fps / args.stride, 1), loop=0)
    print(f"✓ robot    → {robot_out} ({robot_out.stat().st_size // 1024} KB)")


if __name__ == "__main__":
    main()
