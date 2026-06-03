"""Unitree H1 embodiment（v0, 簡略化 kinematic プロキシ）。

⚠️ v0 注意: G1 と同様、実機 URDF / アクチュエータ写像ではない簡略 kinematic 形態。
実 URDF / SDK2 joint 写像 / joint limits の正確化は Phase 2（docs/ROADMAP.md）。

H1 は身長 ~1.8m の full-size ヒューマノイド。canonical 19-joint 構造を流用し、
H1 に近い link 長（長い脚・腕、高い腰）を与える。人間より背が高く lanky に見える。
"""

from __future__ import annotations

import numpy as np

from robotdance_retarget.embodiment import RobotMorphology

ROBOT_NAME = "unitree_h1"

# H1 近似の rest pose（world, z-up, x-forward, y-left, 単位 m）。背が高く四肢が長い。
H1_REST = np.array(
    [
        [0.00, 0.00, 1.00],   # 0 pelvis
        [0.00, 0.00, 1.12],   # 1 spine
        [0.00, 0.00, 1.34],   # 2 chest
        [0.00, 0.00, 1.56],   # 3 neck
        [0.00, 0.00, 1.72],   # 4 head
        [0.00, 0.22, 1.50],   # 5 left_shoulder
        [0.00, 0.23, 1.18],   # 6 left_elbow
        [0.00, 0.24, 0.86],   # 7 left_wrist
        [0.00, -0.22, 1.50],  # 8 right_shoulder
        [0.00, -0.23, 1.18],  # 9 right_elbow
        [0.00, -0.24, 0.86],  # 10 right_wrist
        [0.00, 0.11, 0.95],   # 11 left_hip
        [0.00, 0.11, 0.50],   # 12 left_knee
        [0.00, 0.11, 0.06],   # 13 left_ankle
        [0.18, 0.11, 0.03],   # 14 left_foot
        [0.00, -0.11, 0.95],  # 15 right_hip
        [0.00, -0.11, 0.50],  # 16 right_knee
        [0.00, -0.11, 0.06],  # 17 right_ankle
        [0.18, -0.11, 0.03],  # 18 right_foot
    ],
    dtype=np.float64,
)

MORPHOLOGY = RobotMorphology(
    name=ROBOT_NAME,
    rest_pose=H1_REST,
    urdf_ref="TODO(Phase2): unitree_ros2 / h1_description URDF",
    runtime_adapter="unitree_sdk2",
)

BONE_LENGTHS = MORPHOLOGY.bone_lengths
NOMINAL_HEIGHT = MORPHOLOGY.nominal_height


def embodiment_dict() -> dict:
    """RD-Embodiment v0 schema 適合の dict を返す。"""
    return MORPHOLOGY.to_rd_embodiment()
