"""Unitree G1 embodiment（v0）。

retarget とビューアを sim なしで動かすための、canonical 19-joint と同一トポロジの G1 形態。
rest pose は **公式 g1_23dof URDF の実寸**（Unitree unitree_ros / g1_description）から導いた
canonical joint 位置を採用（v0.26 で更新）。関節オフセット＝寸法の事実のみ使用し、mesh/URDF 本体は
同梱しない（license-safe）。実 actuator 写像・joint limits・慣性は actuator-space IK（retarget-ik /
import-urdf）が実 URDF から扱う。

⚠️ v0 注意: rest 寸法は実機相当だが、これは kinematic 形態であり実機慣性/アクチュエータモデルではない。
（旧 v0 手書きプロキシは nominal 1.12m・bone 平均相対誤差 ~26% で実機と乖離していた。）
"""

from __future__ import annotations

import numpy as np

from robotdance_retarget.embodiment import RobotMorphology

ROBOT_NAME = "unitree_g1"

# 実 g1_23dof URDF 由来の canonical 19-joint rest pose（z-up, x-forward, y-left, m）。
# 接地に合わせ足先が z≈0.03 になるよう全体を +0.809 m シフト。nominal_height ≈ 1.291 m。
G1_REST = np.array(
    [
        [0.000, 0.000, 0.809],   # 0 pelvis
        [0.000, 0.000, 0.955],   # 1 spine
        [0.000, 0.000, 1.101],   # 2 chest
        [0.015, 0.000, 1.211],   # 3 neck
        [0.030, 0.000, 1.321],   # 4 head
        [0.000, 0.100, 1.101],   # 5 left_shoulder
        [0.016, 0.147, 0.914],   # 6 left_elbow
        [0.116, 0.149, 0.904],   # 7 left_wrist
        [0.000, -0.100, 1.101],  # 8 right_shoulder
        [0.016, -0.147, 0.914],  # 9 right_elbow
        [0.116, -0.149, 0.904],  # 10 right_wrist
        [0.000, 0.064, 0.706],   # 11 left_hip
        [0.000, 0.119, 0.370],   # 12 left_knee
        [0.000, 0.119, 0.070],   # 13 left_ankle
        [0.120, 0.119, 0.030],   # 14 left_foot
        [0.000, -0.064, 0.706],  # 15 right_hip
        [0.000, -0.119, 0.370],  # 16 right_knee
        [0.000, -0.119, 0.070],  # 17 right_ankle
        [0.120, -0.119, 0.030],  # 18 right_foot
    ],
    dtype=np.float64,
)

MORPHOLOGY = RobotMorphology(
    name=ROBOT_NAME,
    rest_pose=G1_REST,
    urdf_ref="unitree_ros g1_description/g1_23dof.urdf（実寸由来, 本体は別途取得）",
    runtime_adapter="unitree_sdk2",
)

# 後方互換のモジュールレベル別名。
BONE_LENGTHS = MORPHOLOGY.bone_lengths
NOMINAL_HEIGHT = MORPHOLOGY.nominal_height


def embodiment_dict() -> dict:
    """RD-Embodiment v0 schema 適合の dict を返す。"""
    return MORPHOLOGY.to_rd_embodiment()
