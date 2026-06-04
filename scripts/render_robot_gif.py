#!/usr/bin/env python3
"""実 Unitree URDF（メッシュ付き）を RobotDance の actuator-IK 関節角で踊らせ、GIF を描く。

README の「映える」hero asset 生成用スクリプト（パッケージ本体ではない）。合成ダンス RD-MIR を
actuator-space IK で実 G1 の 23 関節角へ retarget し、pybullet の headless TinyRenderer（GPU 不要）で
実メッシュをレンダリングする。地面影・カメラスウェイ付き。

⚠️ **メッシュ / URDF は repo に同梱しない**（license-safe）。利用者が unitree_ros 等から取得した
ローカル URDF を指す。出力 GIF は RobotDance パイプライン出力の可視化（render）であり、メッシュ本体の
再配布ではない。

依存（dev のみ。パッケージ依存ではない）: pybullet, imageio, torch（actuator-IK）。

使い方:
    python scripts/render_robot_gif.py /path/to/g1_23dof.urdf -o assets/readme/g1_dance.gif
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

# リポジトリルートを import path に追加（scripts/ から実行されるため）。
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("urdf", type=Path, help="実 Unitree URDF（メッシュ付き, ローカル取得）")
    ap.add_argument("-o", "--out", type=Path, default=Path("robot_dance.gif"))
    ap.add_argument("--robot", choices=["g1", "h1"], default="g1", help="link map / 接地高さの選択")
    ap.add_argument("--duration", type=float, default=3.0)
    ap.add_argument("--bps", type=float, default=1.3, help="beats per second（ダンスの速さ）")
    ap.add_argument("--arm", type=float, default=1.8)
    ap.add_argument("--sway", type=float, default=0.18)
    ap.add_argument("--base-z", type=float, default=0.793, help="pelvis 高さ（足が接地する値）")
    ap.add_argument("--stride", type=int, default=2)
    ap.add_argument("--width", type=int, default=480)
    ap.add_argument("--height", type=int, default=620)
    args = ap.parse_args()

    import imageio.v2 as imageio
    import pybullet as p

    from robotdance_core.synthetic import generate_dance
    from robotdance_retarget.actuator_ik import actuator_retarget
    from robotdance_unitree.urdf_import import G1_LINK_MAP, H1_LINK_MAP

    link_map = H1_LINK_MAP if args.robot == "h1" else G1_LINK_MAP
    robot_name = f"unitree_{args.robot}"
    if args.robot == "h1" and args.base_z == 0.793:
        args.base_z = 1.04   # H1 pelvis 高さ

    # 1. 合成ダンス → actuator-IK で実 URDF の関節角列へ。
    mir = generate_dance(duration=args.duration, beats_per_second=args.bps,
                         arm_amp=args.arm, sway_amp=args.sway)
    motion = actuator_retarget(mir, str(args.urdf), steps=250,
                               link_map=link_map, robot_name=robot_name)
    angles = np.asarray(motion.joint_rotations["angles_rad"])
    names = [str(n) for n in motion.joint_rotations["actuated_joint_names"]]
    fps = motion.fps
    print(f"actuator-IK: {angles.shape[0]} frames, {angles.shape[1]} joints, "
          f"IK err {motion.retarget_metrics['ik_mean_pos_error_m']} m")

    # 2. pybullet headless render（メッシュは URDF 相対パス → 作業 dir を URDF dir にする）。
    urdf_dir = args.urdf.resolve().parent
    p.connect(p.DIRECT)
    p.setAdditionalSearchPath(str(urdf_dir))
    # 影を受ける薄いグレーの床（白背景でも影が出る）。
    gv = p.createVisualShape(p.GEOM_BOX, halfExtents=[3, 3, 0.01], rgbaColor=[0.93, 0.93, 0.95, 1])
    gc = p.createCollisionShape(p.GEOM_BOX, halfExtents=[3, 3, 0.01])
    p.createMultiBody(0, gc, gv, basePosition=[0, 0, -0.005])
    rid = p.loadURDF(args.urdf.name, useFixedBase=True, basePosition=[0, 0, args.base_z])

    jmap = {}
    for i in range(p.getNumJoints(rid)):
        info = p.getJointInfo(rid, i)
        if info[2] == p.JOINT_REVOLUTE:
            jmap[info[1].decode()] = i
    pairs = [(jmap[n], k) for k, n in enumerate(names) if n in jmap]
    print(f"driving {len(pairs)}/{len(names)} joints "
          f"(skip: {[n for n in names if n not in jmap]})")

    proj = p.computeProjectionMatrixFOV(42, args.width / args.height, 0.1, 10)
    cam_target_z = args.base_z * 0.78   # ロボット中心の高さ
    cam_dist = args.base_z * 2.45
    frames = []
    t_len = angles.shape[0]
    for f in range(0, t_len, args.stride):
        for ji, k in pairs:
            p.resetJointState(rid, ji, float(angles[f, k]))
        yaw = 35 + 25 * np.sin(2 * np.pi * f / t_len)   # ゆるやかなカメラスウェイ
        view = p.computeViewMatrixFromYawPitchRoll([0, 0, cam_target_z], cam_dist, yaw, -10, 0, 2)
        img = p.getCameraImage(args.width, args.height, view, proj,
                               renderer=p.ER_TINY_RENDERER,
                               lightDirection=[0.6, 0.7, 1.2], shadow=1)
        frames.append(np.reshape(img[2], (args.height, args.width, 4))[:, :, :3].astype(np.uint8))
    p.disconnect()

    args.out.parent.mkdir(parents=True, exist_ok=True)
    imageio.mimsave(args.out, frames, duration=1.0 / max(fps / args.stride, 1), loop=0)
    print(f"✓ {len(frames)} frames → {args.out} "
          f"({args.out.stat().st_size // 1024} KB)")


if __name__ == "__main__":
    main()
