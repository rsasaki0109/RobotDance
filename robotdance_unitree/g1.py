"""Unitree G1 embodiment（v0, 簡略化 kinematic プロキシ）。

⚠️ v0 注意: これは実機 URDF / アクチュエータ写像ではない。retarget とビューアを sim なしで
動かすための、G1 の体格に近い簡略 kinematic 形態（canonical と同一トポロジ）。
実 URDF / SDK2 joint 写像 / joint limits の正確化は Phase 2 で行う（docs/ROADMAP.md）。

G1 は身長 ~1.27m の小型ヒューマノイド。ここでは canonical 19-joint 構造を流用し、
G1 に近い link 長（短い四肢・低い腰）を与えて embodiment 差を可視化する。
"""

from __future__ import annotations

from typing import Any

import numpy as np

from robotdance_core.skeleton import JOINT_NAMES, PARENTS

ROBOT_NAME = "unitree_g1"

# G1 近似の rest pose（world, z-up, x-forward, y-left, 単位 m）。
# 人間 rest より低く・四肢が短い stocky な体格。
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

# rest の bone 長（親→子）。retarget はこの長さを保って人間の bone 方向を再現する。
BONE_LENGTHS = np.linalg.norm(
    G1_REST - G1_REST[np.array([max(p, 0) for p in PARENTS])], axis=1
)

# rest の概略全高（足先〜頭頂）。height_scale 計算に使う。
NOMINAL_HEIGHT = float(G1_REST[:, 2].max() - G1_REST[:, 2].min())


def embodiment_dict() -> dict[str, Any]:
    """RD-Embodiment v0 schema に適合する dict を返す。

    v0 では canonical joint 名を流用（実 G1 actuator 名への写像は Phase 2）。
    joint_limits は概略のプレースホルダ。
    """
    # 概略の joint limit プレースホルダ（位置 rad / 速度 rad·s⁻¹ / トルク N·m）。
    generic_limit = {"position": [-3.14, 3.14], "velocity": 12.0, "torque": 60.0}
    return {
        "rd_embodiment_version": "0",
        "robot_name": ROBOT_NAME,
        "urdf_ref": "TODO(Phase2): unitree_ros2 / g1_description URDF",
        "joint_names": list(JOINT_NAMES),
        "joint_limits": {name: dict(generic_limit) for name in JOINT_NAMES},
        "link_lengths": {
            JOINT_NAMES[j]: float(BONE_LENGTHS[j]) for j in range(len(JOINT_NAMES)) if PARENTS[j] >= 0
        },
        "end_effectors": ["left_foot", "right_foot", "left_wrist", "right_wrist"],
        "control_modes": ["position", "policy"],
        # nominal_pose（joint 角）は actuator 写像が決まる Phase 2 で設定。v0 は rest を G1_REST で保持。
        "runtime_adapter": "unitree_sdk2",
    }
