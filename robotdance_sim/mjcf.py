"""RobotMorphology から MuJoCo MJCF を生成する（v0）。

canonical 19-joint の rest pose を、bone ごとに独立した ball joint を持つ多体ツリーへ変換する。
各 bone は capsule geom（質量 ∝ bone 長）で表し、足 bone には接地用の box を付ける。
逆動力学（必要トルク）と COM 計算のための**近似質量モデル**であり、実機慣性ではない（v0）。

ツリー構造: bone b=(p→j) を 1 つの body として表し、その ball joint が bone b の向きを制御する。
これにより各 bone 方向を独立に再現でき、keypoints を厳密に復元できる（mujoco_backend 参照）。
"""

from __future__ import annotations

import numpy as np

from robotdance_core.skeleton import JOINT_NAMES, PARENTS
from robotdance_retarget.embodiment import RobotMorphology

# 足 bone（ankle→foot）。接地 box を付与する。
_FOOT_CHILD_JOINTS = {JOINT_NAMES.index("left_foot"), JOINT_NAMES.index("right_foot")}


def build_mjcf(morphology: RobotMorphology, *, total_mass: float = 35.0, ground: bool = True) -> str:
    """morphology から MJCF 文字列を生成する。

    total_mass: 全 bone へ長さ比で配分する概算総質量（kg）。G1≈35, H1≈47 程度。
    ground: 地面 plane を含めるか。逆動力学（mj_inverse）では接触力が混入して
            内部トルクを汚染するため、トルク/COM 計算では ground=False（純浮遊多体）にする。
    """
    rest = morphology.rest_pose
    bone_len = morphology.bone_lengths
    len_sum = float(sum(bone_len[j] for j in range(len(JOINT_NAMES)) if PARENTS[j] >= 0)) or 1.0

    # 各 joint の子 bone リスト（MJCF ネストは joint 親子ツリーと一致）。
    children: dict[int, list[int]] = {i: [] for i in range(len(JOINT_NAMES))}
    for j, p in enumerate(PARENTS):
        if p >= 0:
            children[p].append(j)

    def emit_body(j: int, indent: int) -> str:
        p = PARENTS[j]
        gp = PARENTS[p] if p > 0 else -1
        # body_j 原点 = bone j の始点（= joint p）。親 body 原点（joint gp）からの相対 pos。
        pos = (rest[p] - rest[gp]) if p > 0 else np.zeros(3)
        endpoint = rest[j] - rest[p]  # 自 frame での bone 終点（= o_j）
        mass = max(total_mass * bone_len[j] / len_sum, 0.05)
        pad = "  " * indent
        s = f'{pad}<body name="body_{j}" pos="{_v(pos)}">\n'
        s += f'{pad}  <joint name="jnt_{j}" type="ball"/>\n'
        s += (
            f'{pad}  <geom type="capsule" fromto="0 0 0 {_v(endpoint)}" '
            f'size="0.04" mass="{mass:.4f}"/>\n'
        )
        if j in _FOOT_CHILD_JOINTS:
            # 接地用の足 box（前方に伸ばす）。
            s += (
                f'{pad}  <geom type="box" pos="{_v(endpoint)}" size="0.08 0.04 0.02" '
                f'mass="0.3" friction="1 0.05 0.01"/>\n'
            )
        for c in children[j]:
            s += emit_body(c, indent + 1)
        s += f"{pad}</body>\n"
        return s

    # root = pelvis（free joint）。pelvis 自体に小球の geom。
    body = '    <body name="root" pos="0 0 1">\n'
    body += '      <freejoint name="root"/>\n'
    body += '      <geom type="sphere" size="0.06" mass="3.0"/>\n'
    for c in children[0]:
        body += emit_body(c, 3)
    body += "    </body>\n"

    ground_geom = (
        '    <geom name="ground" type="plane" size="5 5 0.1" friction="1 0.05 0.01"/>\n'
        if ground else ""
    )
    return f"""<mujoco model="{morphology.name}">
  <option gravity="0 0 -9.81" timestep="0.002" integrator="implicitfast"/>
  <worldbody>
{ground_geom}{body}  </worldbody>
</mujoco>
"""


def _v(a: np.ndarray) -> str:
    return f"{a[0]:.5f} {a[1]:.5f} {a[2]:.5f}"
