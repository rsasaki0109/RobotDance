"""Unitree G1 embodiment（v0, 簡略化 kinematic プロキシ）。

⚠️ v0 注意: これは実機 URDF / アクチュエータ写像ではない。retarget とビューアを sim なしで
動かすための、G1 の体格に近い簡略 kinematic 形態（canonical と同一トポロジ）。
実 URDF / SDK2 joint 写像 / joint limits の正確化は Phase 2 で行う（docs/ROADMAP.md）。

G1 は身長 ~1.27m の小型ヒューマノイド。canonical 19-joint 構造を流用し、
G1 に近い link 長（短い四肢・低い腰）を与えて embodiment 差を可視化する。
"""

from __future__ import annotations

import numpy as np

from robotdance_retarget.embodiment import RobotMorphology

ROBOT_NAME = "unitree_g1"

# G1 近似の rest pose（world, z-up, x-forward, y-left, 単位 m）。低く・四肢が短い stocky な体格。
G1_REST = np.array(
    [
        [0.00, 0.00, 0.70],   # 0 pelvis
        [0.00, 0.00, 0.78],   # 1 spine
        [0.00, 0.00, 0.92],   # 2 chest
        [0.00, 0.00, 1.05],   # 3 neck
        [0.00, 0.00, 1.15],   # 4 head
        [0.00, 0.16, 1.00],   # 5 left_shoulder
        [0.00, 0.17, 0.82],   # 6 left_elbow
        [0.00, 0.18, 0.64],   # 7 left_wrist
        [0.00, -0.16, 1.00],  # 8 right_shoulder
        [0.00, -0.17, 0.82],  # 9 right_elbow
        [0.00, -0.18, 0.64],  # 10 right_wrist
        [0.00, 0.09, 0.66],   # 11 left_hip
        [0.00, 0.09, 0.36],   # 12 left_knee
        [0.00, 0.09, 0.06],   # 13 left_ankle
        [0.12, 0.09, 0.03],   # 14 left_foot
        [0.00, -0.09, 0.66],  # 15 right_hip
        [0.00, -0.09, 0.36],  # 16 right_knee
        [0.00, -0.09, 0.06],  # 17 right_ankle
        [0.12, -0.09, 0.03],  # 18 right_foot
    ],
    dtype=np.float64,
)

MORPHOLOGY = RobotMorphology(
    name=ROBOT_NAME,
    rest_pose=G1_REST,
    urdf_ref="TODO(Phase2): unitree_ros2 / g1_description URDF",
    runtime_adapter="unitree_sdk2",
)

# 後方互換のモジュールレベル別名。
BONE_LENGTHS = MORPHOLOGY.bone_lengths
NOMINAL_HEIGHT = MORPHOLOGY.nominal_height


def embodiment_dict() -> dict:
    """RD-Embodiment v0 schema 適合の dict を返す。"""
    return MORPHOLOGY.to_rd_embodiment()
