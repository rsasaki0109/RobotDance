"""Unitree H1 embodiment（v0）。

canonical 19-joint と同一トポロジの H1 形態。rest pose は **公式 h1.urdf の実寸**（Unitree
unitree_ros / h1_description）から導いた canonical joint 位置を採用（v0.26 で更新）。H1 は腕が肘で
終わり wrist link が無いため前腕は合成、頭頂・toe も合成（`urdf_import.build_rest_pose`）。
関節オフセット＝寸法の事実のみ使用し mesh/URDF 本体は同梱しない（license-safe）。

⚠️ v0 注意: rest 寸法は実機相当だが kinematic 形態であり実機慣性/アクチュエータモデルではない。
（旧 v0 手書きプロキシは bone 平均相対誤差 ~33% で実機と乖離していた。特に hip 幅・肩を誤っていた。）
"""

from __future__ import annotations

import numpy as np

from robotdance_retarget.embodiment import RobotMorphology, SimDefaults

ROBOT_NAME = "unitree_h1"

# 実 h1.urdf 由来の canonical 19-joint rest pose（z-up, x-forward, y-left, m）。
# 接地に合わせ足先が z≈0.03 になるようシフト。nominal_height ≈ 1.664 m。
H1_REST = np.array(
    [
        [0.000, 0.000, 1.044],   # 0 pelvis
        [0.003, 0.000, 1.259],   # 1 spine
        [0.005, 0.000, 1.474],   # 2 chest
        [0.020, 0.000, 1.584],   # 3 neck
        [0.035, 0.000, 1.694],   # 4 head
        [0.005, 0.155, 1.474],   # 5 left_shoulder
        [0.018, 0.214, 1.151],   # 6 left_elbow
        [0.032, 0.272, 0.827],   # 7 left_wrist (forearm 合成)
        [0.005, -0.155, 1.474],  # 8 right_shoulder
        [0.018, -0.214, 1.151],  # 9 right_elbow
        [0.032, -0.272, 0.827],  # 10 right_wrist (forearm 合成)
        [0.039, 0.203, 0.870],   # 11 left_hip
        [0.039, 0.203, 0.470],   # 12 left_knee
        [0.039, 0.203, 0.070],   # 13 left_ankle
        [0.159, 0.203, 0.030],   # 14 left_foot
        [0.039, -0.203, 0.870],  # 15 right_hip
        [0.039, -0.203, 0.470],  # 16 right_knee
        [0.039, -0.203, 0.070],  # 17 right_ankle
        [0.159, -0.203, 0.030],  # 18 right_foot
    ],
    dtype=np.float64,
)

# 実 h1.urdf 由来の canonical 関節 limit（envelope 集約。位置 rad / 速度 rad·s⁻¹ / トルク N·m）。
# H1 は腕が肘止まりで wrist actuator が無いため wrist は省略（合成前腕は limit 対象外）。
# 肩 yaw は 4.45rad に達し、placeholder ±3.14 は逆に過小評価していた。膝トルク 300N·m と強力。
# test_real_h1_urdf が実 URDF と一致を検証。
H1_JOINT_LIMITS: dict[str, dict[str, object]] = {
    "left_hip": {"position": [-3.14, 2.53], "velocity": 23.0, "torque": 200.0},
    "right_hip": {"position": [-3.14, 2.53], "velocity": 23.0, "torque": 200.0},
    "left_knee": {"position": [-0.26, 2.05], "velocity": 14.0, "torque": 300.0},
    "right_knee": {"position": [-0.26, 2.05], "velocity": 14.0, "torque": 300.0},
    "left_ankle": {"position": [-0.87, 0.52], "velocity": 9.0, "torque": 40.0},
    "right_ankle": {"position": [-0.87, 0.52], "velocity": 9.0, "torque": 40.0},
    "left_shoulder": {"position": [-2.87, 4.45], "velocity": 9.0, "torque": 18.0},
    "right_shoulder": {"position": [-4.45, 2.87], "velocity": 9.0, "torque": 18.0},
    "left_elbow": {"position": [-1.25, 2.61], "velocity": 20.0, "torque": 18.0},
    "right_elbow": {"position": [-1.25, 2.61], "velocity": 20.0, "torque": 18.0},
    "spine": {"position": [-2.35, 2.35], "velocity": 23.0, "torque": 200.0},
}

# 実 h1.urdf の <inertial> 由来の canonical 質量分布（Σ≈1, 数値のみで license-safe）。
# H1 は脚~58%/胴体~30% とさらに脚が重い（実 URDF 総質量は ~59kg）。Winter 人体とは別物。
# torso 質量は spine bone へ集約（H1 の torso_link COM が spine 高さ）。
# test_real_h1_urdf が実 URDF からの算出値と一致を検証。
H1_MASS_FRACTION: dict[str, float] = {
    "pelvis": 0.001, "spine": 0.298, "chest": 0.001, "neck": 0.001, "head": 0.001,
    "left_shoulder": 0.001, "left_elbow": 0.0478, "left_wrist": 0.0125,
    "right_shoulder": 0.001, "right_elbow": 0.0478, "right_wrist": 0.0125,
    "left_hip": 0.1453, "left_knee": 0.083, "left_ankle": 0.0473, "left_foot": 0.0121,
    "right_hip": 0.1453, "right_knee": 0.083, "right_ankle": 0.0473, "right_foot": 0.0121,
}

MORPHOLOGY = RobotMorphology(
    name=ROBOT_NAME,
    rest_pose=H1_REST,
    urdf_ref="unitree_ros h1_description/urdf/h1.urdf（実寸由来, 本体は別途取得）",
    runtime_adapter="unitree_sdk2",
    per_joint_limits=H1_JOINT_LIMITS,
    mass_distribution=H1_MASS_FRACTION,
    # H1 は G1 より背が高く（1.66m）重い（47kg）→ 高い kd が必須（kd=6 では PD 振動で転倒）。
    sim_defaults=SimDefaults(total_mass=47.0, kp=200.0, kd=10.0, torque_limit=160.0),
)

BONE_LENGTHS = MORPHOLOGY.bone_lengths
NOMINAL_HEIGHT = MORPHOLOGY.nominal_height


def embodiment_dict() -> dict:
    """RD-Embodiment v0 schema 適合の dict を返す。"""
    return MORPHOLOGY.to_rd_embodiment()
