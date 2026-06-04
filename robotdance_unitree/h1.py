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

MORPHOLOGY = RobotMorphology(
    name=ROBOT_NAME,
    rest_pose=H1_REST,
    urdf_ref="unitree_ros h1_description/urdf/h1.urdf（実寸由来, 本体は別途取得）",
    runtime_adapter="unitree_sdk2",
    # H1 は G1 より背が高く（1.66m）重い（47kg）→ 高い kd が必須（kd=6 では PD 振動で転倒）。
    sim_defaults=SimDefaults(total_mass=47.0, kp=200.0, kd=10.0, torque_limit=160.0),
)

BONE_LENGTHS = MORPHOLOGY.bone_lengths
NOMINAL_HEIGHT = MORPHOLOGY.nominal_height


def embodiment_dict() -> dict:
    """RD-Embodiment v0 schema 適合の dict を返す。"""
    return MORPHOLOGY.to_rd_embodiment()
