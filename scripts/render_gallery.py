#!/usr/bin/env python3
"""実 Unitree メッシュが「色々な振付」を踊る GIF ギャラリーを一括生成する。

README の hero ギャラリー用（パッケージ本体ではない）。複数の合成モーション（groove / fast /
march / squat …）をそれぞれ actuator-space IK で実ロボットの関節角へ retarget し、pybullet headless
TinyRenderer（GPU 不要）で実メッシュをレンダリングして 1 振付 = 1 GIF として書き出す。

「色々な short 動画 → ヒューマノイドが踊る」の壁を作るためのスクリプト。⚠️ 実動画はライセンス上同梱
できないので、ここでの「色々な入力」は合成モーション群が代役（README にも明記）。出力 GIF は
RobotDance パイプライン出力（actuator-IK 関節角）の可視化（render）であり、メッシュ本体の再配布ではない。

⚠️ **メッシュ / URDF は repo に同梱しない**（license-safe）。利用者が unitree_ros 等から取得した
ローカル URDF を指す。

依存（dev のみ）: pybullet, imageio, torch（actuator-IK）。

使い方:
    python scripts/render_gallery.py /path/to/g1_23dof.urdf --robot g1 -o assets/readme/gallery
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

# リポジトリルートを import path に追加（scripts/ から実行されるため）。
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))


def _build_motions():
    """ギャラリーに並べる「色々な振付」を (slug, caption, RdMir) で返す。

    遅延 import（パッケージ依存を import-path 解決後にするため）。silhouette が被らないよう
    上半身主体 / 脚主体を混ぜて多様性を出す。
    """
    from robotdance_core.synthetic import generate_dance, generate_march, generate_squat

    return [
        ("groove", "groove",
         generate_dance(duration=3.0, beats_per_second=1.1, arm_amp=1.7, sway_amp=0.20)),
        ("fast", "fast dance",
         generate_dance(duration=3.0, beats_per_second=2.2, arm_amp=2.0, sway_amp=0.16)),
        ("wave", "arm wave",
         generate_dance(duration=3.0, beats_per_second=0.7, arm_amp=2.3, sway_amp=0.10)),
        ("march", "march",
         generate_march(duration=3.0, steps_per_second=1.2, lift=0.9)),
        ("squat", "squat",
         generate_squat(duration=3.0, depth=1.5)),
    ]


def _render_one(p, urdf: Path, robot: str, base_z: float, angles, names,
                fps: float, stride: int, width: int, height: int, caption: str) -> list:
    """1 振付ぶんの関節角列を実メッシュでレンダリングし、RGB フレーム列を返す。"""
    urdf_dir = urdf.resolve().parent
    p.resetSimulation()
    p.setAdditionalSearchPath(str(urdf_dir))
    # 影を受ける薄いグレーの床。
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
    ap.add_argument("urdf", type=Path, help="実 Unitree URDF（メッシュ付き, ローカル取得）")
    ap.add_argument("--robot", choices=["g1", "h1", "h2"], default="g1")
    ap.add_argument("-o", "--out-dir", type=Path, default=Path("assets/readme/gallery"))
    ap.add_argument("--base-z", type=float, default=None,
                    help="pelvis 高さ（既定: g1 0.793 / h1 1.04 / h2 1.055）")
    ap.add_argument("--stride", type=int, default=2)
    ap.add_argument("--width", type=int, default=360)
    ap.add_argument("--height", type=int, default=460)
    args = ap.parse_args()

    import imageio.v2 as imageio
    import pybullet as p

    from robotdance_retarget.actuator_ik import actuator_retarget
    from robotdance_unitree.h2 import H2_LINK_MAP
    from robotdance_unitree.urdf_import import G1_LINK_MAP, H1_LINK_MAP

    link_map = {"h1": H1_LINK_MAP, "h2": H2_LINK_MAP}.get(args.robot, G1_LINK_MAP)
    robot_name = f"unitree_{args.robot}"
    base_z = args.base_z if args.base_z is not None else {"h1": 1.04, "h2": 1.055}.get(
        args.robot, 0.793)

    args.out_dir.mkdir(parents=True, exist_ok=True)
    p.connect(p.DIRECT)

    for slug, caption, mir in _build_motions():
        motion = actuator_retarget(mir, str(args.urdf), steps=250,
                                   link_map=link_map, robot_name=robot_name)
        angles = np.asarray(motion.joint_rotations["angles_rad"])
        names = [str(n) for n in motion.joint_rotations["actuated_joint_names"]]
        ik_err = motion.retarget_metrics["ik_mean_pos_error_m"]
        frames = _render_one(p, args.urdf, args.robot, base_z, angles, names,
                             motion.fps, args.stride, args.width, args.height, caption)
        out = args.out_dir / f"{args.robot}_{slug}.gif"
        imageio.mimsave(out, frames, duration=1.0 / max(motion.fps / args.stride, 1), loop=0)
        print(f"✓ {caption:12s} IK err {ik_err} m  → {out} ({out.stat().st_size // 1024} KB)")

    p.disconnect()


if __name__ == "__main__":
    main()
