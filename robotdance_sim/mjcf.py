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

# 接地 box の半寸（m）: 前後 half-length / 左右 half-width / 厚み half-height。
# これが実際に sim される足の接地フットプリント。バランス判定（ZMP 支持多角形）も
# この幅を実フットプリントの単一の出所として参照する（判定と sim の幾何を一致させる）。
FOOT_BOX_HALF_LENGTH = 0.08
FOOT_BOX_HALF_WIDTH = 0.04
_FOOT_BOX_HALF_HEIGHT = 0.02

# セグメント質量比（総質量に対する割合）。出典: Winter, D.A., "Biomechanics and Motor
# Control of Human Movement"（人体計測の標準セグメント質量比）。
# canonical 19-joint の各 bone（親→子, key=子 joint 名）へ写像する。pelvis(=root) はハブに割当。
#
# なぜ bone 長比をやめるか: 旧実装の「質量 ∝ bone 長」は物理的根拠が無く、長い腕 bone に
# 過大な質量を与えていた（実証: H1 は腕 32% > 胴体 19% という非物理分布）。人体もロボットも
# （電池・計算機・アクチュエータが胴体集中）**胴体が最重量部位**。本テーブルで胴体重心の
# 物理的に妥当な分布にする。比は body proportion 不変なので G1/H1 で共通（総質量のみ機種差）。
# 注: 実機 URDF の <inertial> そのものではない（mesh/URDF 本体は license 上同梱しない）人体
# 近似プライアだが、bone 長比より桁違いに妥当。実 URDF 慣性の取り込みは将来 spec。
_SEGMENT_MASS_WEIGHT: dict[str, float] = {
    "pelvis": 0.142,         # 骨盤（root ハブへ）
    "spine": 0.139,          # 腹部 abdomen（pelvis→spine bone）
    "chest": 0.216,          # 胸郭 thorax（spine→chest bone）
    "neck": 0.012,           # 頸部
    "head": 0.069,           # 頭部（head+neck 計 0.081 を頸/頭へ分割）
    "left_shoulder": 0.008,  # 肩甲帯 girdle（chest→shoulder の連結 bone）
    "left_elbow": 0.028,     # 上腕 upper arm（shoulder→elbow）
    "left_wrist": 0.022,     # 前腕+手 forearm+hand（elbow→wrist）
    "right_shoulder": 0.008,
    "right_elbow": 0.028,
    "right_wrist": 0.022,
    "left_hip": 0.008,       # 骨盤→股関節の連結 bone（小）
    "left_knee": 0.100,      # 大腿 thigh（hip→knee）
    "left_ankle": 0.0465,    # 下腿 shank（knee→ankle）
    "left_foot": 0.0145,     # 足部 foot（ankle→toe）
    "right_hip": 0.008,
    "right_knee": 0.100,
    "right_ankle": 0.0465,
    "right_foot": 0.0145,
}
# 正規化（連結 bone を足したので合計 ≈1.03）。これで Σ=1 → 総質量を厳密保存。
_W_SUM = sum(_SEGMENT_MASS_WEIGHT.values())
_SEGMENT_MASS_FRACTION = {k: v / _W_SUM for k, v in _SEGMENT_MASS_WEIGHT.items()}
# 足部質量は capsule（中足）と接地 box（踵/接地）で 50/50 に分ける。
_FOOT_BOX_SHARE = 0.5


def build_mjcf(
    morphology: RobotMorphology, *, total_mass: float = 35.0, ground: bool = True,
    mass_fraction: "dict[str, float] | None" = None,
) -> str:
    """morphology から MJCF 文字列を生成する。

    total_mass: robot の総質量（kg, 実機相当）。実 URDF 総質量は G1≈34, H1≈59 程度。
    mass_fraction: canonical joint 名 → 質量割合（Σ=1 へ再正規化して使用）。None なら
        morphology.mass_distribution（実 URDF inertial 由来があればそれ）→ 無ければ Winter 人体
        計測比（_SEGMENT_MASS_FRACTION）の順でフォールバック。実機分布は脚が重い（股・膝
        アクチュエータ）ので Winter 人体プライアと有意に異なる。いずれも Σ=1 なので生成 MJCF の
        総質量は total_mass に厳密一致する（宣言質量＝実質量）。
    ground: 地面 plane を含めるか。逆動力学（mj_inverse）では接触力が混入して
            内部トルクを汚染するため、トルク/COM 計算では ground=False（純浮遊多体）にする。
    """
    rest = morphology.rest_pose
    raw = mass_fraction or getattr(morphology, "mass_distribution", None) or _SEGMENT_MASS_FRACTION
    fsum = sum(raw.get(name, 0.0) for name in JOINT_NAMES) or 1.0
    frac = {name: raw.get(name, 0.0) / fsum for name in JOINT_NAMES}  # Σ=1 へ再正規化
    pelvis_hub_mass = total_mass * frac["pelvis"]

    def _bone_mass(j: int) -> float:
        """bone j（親→子 joint）のセグメント質量（kg）。足部は box と按分。"""
        f = frac[JOINT_NAMES[j]]
        if j in _FOOT_CHILD_JOINTS:
            f *= 1.0 - _FOOT_BOX_SHARE  # capsule 側（残りは接地 box へ）
        return max(total_mass * f, 0.01)

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
        mass = _bone_mass(j)
        pad = "  " * indent
        s = f'{pad}<body name="body_{j}" pos="{_v(pos)}">\n'
        s += f'{pad}  <joint name="jnt_{j}" type="ball"/>\n'
        s += (
            f'{pad}  <geom type="capsule" fromto="0 0 0 {_v(endpoint)}" '
            f'size="0.04" mass="{mass:.4f}"/>\n'
        )
        if j in _FOOT_CHILD_JOINTS:
            # 接地用の足 box（前方に伸ばす）。足部質量の box 側按分。
            box_mass = total_mass * frac[JOINT_NAMES[j]] * _FOOT_BOX_SHARE
            box_size = f"{FOOT_BOX_HALF_LENGTH} {FOOT_BOX_HALF_WIDTH} {_FOOT_BOX_HALF_HEIGHT}"
            s += (
                f'{pad}  <geom type="box" pos="{_v(endpoint)}" size="{box_size}" '
                f'mass="{box_mass:.4f}" friction="1 0.05 0.01"/>\n'
            )
        for c in children[j]:
            s += emit_body(c, indent + 1)
        s += f"{pad}</body>\n"
        return s

    # root = pelvis（free joint）。pelvis 自体に小球の geom。
    body = '    <body name="root" pos="0 0 1">\n'
    body += '      <freejoint name="root"/>\n'
    body += f'      <geom type="sphere" size="0.06" mass="{pelvis_hub_mass:.4f}"/>\n'
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
