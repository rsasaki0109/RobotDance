"""Unitree H2 embodiment（v0）。

canonical 19-joint と同一トポロジの H2 形態。rest pose / joint limit / 慣性テンソルは公式
**H2.urdf（Unitree unitree_ros / h2_description）の実寸**から `urdf_import.urdf_to_morphology`
で導出した数値定数。H2 は腕が wrist まであり（G1/H1 と違い前腕合成不要）、waist(3dof)・head(2dof) を持つ
フル humanoid。nominal_height ≈ 1.758 m / total_mass ≈ 75.6 kg。

関節オフセット・慣性＝寸法/質量の事実のみ使用し、**mesh/URDF 本体は同梱しない**（license-safe）。
H2_LINK_MAP は canonical→H2 リンク名の対応。回帰検証は tests/test_real_h2_urdf.py（URDF 在時のみ）。

⚠️ v0 注意: rest 寸法は実機相当だが kinematic 形態であり実機アクチュエータモデルではない。
慣性テンソルは EMBODIMENT_INERTIA registry 経由で real_inertia=True 時に装着（既定 capsule 近似）。
"""

from __future__ import annotations

import numpy as np

from robotdance_retarget.embodiment import RobotMorphology, SimDefaults

ROBOT_NAME = "unitree_h2"

# canonical 19-joint → H2.urdf リンク名（rest pose 導出用）。
H2_LINK_MAP = {
    "pelvis": "pelvis",
    "left_hip": "left_hip_pitch_link", "right_hip": "right_hip_pitch_link",
    "left_knee": "left_knee_link", "right_knee": "right_knee_link",
    "left_ankle": "left_ankle_pitch_link", "right_ankle": "right_ankle_pitch_link",
    "left_shoulder": "left_shoulder_pitch_link", "right_shoulder": "right_shoulder_pitch_link",
    "left_elbow": "left_elbow_link", "right_elbow": "right_elbow_link",
    "left_wrist": "left_wrist_yaw_link", "right_wrist": "right_wrist_yaw_link",
}

# 実 H2.urdf 由来の canonical 19-joint rest pose（z-up, x-forward, y-left, m）。
# 足先が z≈0.03 になるよう接地シフト。nominal_height ≈ 1.758 m。
H2_REST = np.array(
    [
        [0.000, 0.000, 1.055],   # 0 pelvis
        [0.002, 0.000, 1.296],   # 1 spine
        [0.003, 0.000, 1.538],   # 2 chest
        [0.018, 0.000, 1.648],   # 3 neck
        [0.033, 0.000, 1.758],   # 4 head
        [0.003, 0.110, 1.538],   # 5 left_shoulder
        [0.014, 0.204, 1.283],   # 6 left_elbow
        [0.252, 0.183, 1.269],   # 7 left_wrist
        [0.003, -0.110, 1.538],  # 8 right_shoulder
        [0.014, -0.204, 1.283],  # 9 right_elbow
        [0.252, -0.183, 1.269],  # 10 right_wrist
        [0.000, 0.080, 1.008],   # 11 left_hip
        [0.003, 0.139, 0.567],   # 12 left_knee
        [0.021, 0.079, 0.070],   # 13 left_ankle
        [0.141, 0.079, 0.030],   # 14 left_foot
        [0.000, -0.080, 1.008],  # 15 right_hip
        [0.003, -0.139, 0.567],  # 16 right_knee
        [0.021, -0.079, 0.070],  # 17 right_ankle
        [0.141, -0.079, 0.030],  # 18 right_foot
    ],
    dtype=np.float64,
)

# 実 H2.urdf 由来の canonical 関節 limit（actuator envelope 集約。位置 rad / 速度 rad·s⁻¹ / トルク N·m）。
H2_JOINT_LIMITS: dict[str, dict[str, object]] = {
    "left_hip": {"position": [-2.83, 2.83], "velocity": 20.0, "torque": 360.0},
    "right_hip": {"position": [-2.83, 2.83], "velocity": 20.0, "torque": 360.0},
    "left_knee": {"position": [-0.09, 2.53], "velocity": 20.0, "torque": 360.0},
    "right_knee": {"position": [-0.09, 2.53], "velocity": 20.0, "torque": 360.0},
    "left_ankle": {"position": [-1.13, 0.61], "velocity": 28.6, "torque": 19.0},
    "right_ankle": {"position": [-1.13, 0.61], "velocity": 28.6, "torque": 19.0},
    "left_shoulder": {"position": [-2.62, 2.62], "velocity": 18.7, "torque": 60.0},
    "right_shoulder": {"position": [-2.62, 2.62], "velocity": 18.7, "torque": 60.0},
    "left_elbow": {"position": [-0.99, 3.07], "velocity": 18.7, "torque": 60.0},
    "right_elbow": {"position": [-0.99, 3.07], "velocity": 18.7, "torque": 60.0},
    "left_wrist": {"position": [-2.62, 2.62], "velocity": 18.7, "torque": 10.0},
    "right_wrist": {"position": [-2.62, 2.62], "velocity": 18.7, "torque": 10.0},
    "spine": {"position": [-1.75, 1.75], "velocity": 28.4, "torque": 120.0},
}

# 実 H2.urdf <inertial> 由来の per-bone 慣性テンソル（mass kg / com m / fullinertia kg·m²）。
H2_INERTIA_TENSORS: dict[str, dict] = {
    "pelvis": {"mass": 8.5477, "com": [0.00027, -0.00003, -0.01453], "fullinertia": [0.035274, 0.02269, 0.027237, -0.000148, 0.000118, 8e-06]},
    "spine": {"mass": 0.4514, "com": [0.00449, 0.00019, 0.11271], "fullinertia": [0.000158, 0.000325, 0.000366, 0.0, 1.7e-05, -1e-06]},
    "chest": {"mass": 18.532, "com": [0.00586, 0.00628, 0.09705], "fullinertia": [0.287243, 0.262112, 0.094198, 0.00099, 0.003149, -0.010808]},
    "head": {"mass": 3.12, "com": [-0.00827, -0.00035, 0.05586], "fullinertia": [0.021204, 0.022547, 0.005967, 4e-06, -0.001715, 6.1e-05]},
    "left_elbow": {"mass": 2.6553, "com": [-0.00353, 0.06614, -0.06630], "fullinertia": [0.014151, 0.013467, 0.003006, 1.9e-05, -1e-05, 0.002267]},
    "left_wrist": {"mass": 3.7117, "com": [0.14988, -0.02276, -0.00950], "fullinertia": [0.002844, 0.047116, 0.046901, 0.000666, 0.001287, 4.3e-05]},
    "right_elbow": {"mass": 2.6553, "com": [-0.00353, -0.06614, -0.06630], "fullinertia": [0.014151, 0.013467, 0.003006, -1.9e-05, -1e-05, -0.002267]},
    "right_wrist": {"mass": 3.7124, "com": [0.14995, 0.02276, -0.00951], "fullinertia": [0.002845, 0.047207, 0.046992, -0.000667, 0.001291, -4.3e-05]},
    "left_knee": {"mass": 10.7463, "com": [0.01197, 0.03128, -0.17366], "fullinertia": [0.137629, 0.136959, 0.020569, 0.000197, 0.004836, -0.011036]},
    "left_ankle": {"mass": 4.4766, "com": [0.00491, -0.02256, -0.17623], "fullinertia": [0.06414, 0.064287, 0.003397, -7e-06, 6e-06, -2e-06]},
    "left_foot": {"mass": 0.895, "com": [0.03206, 0.03701, -0.02883], "fullinertia": [0.000608, 0.00411, 0.004375, 0.0, 0.000177, -0.0]},
    "right_knee": {"mass": 10.7124, "com": [0.01193, -0.03126, -0.17393], "fullinertia": [0.137386, 0.136714, 0.020562, -0.0002, 0.004871, 0.011013]},
    "right_ankle": {"mass": 4.4766, "com": [0.00491, 0.02256, -0.17623], "fullinertia": [0.06414, 0.064287, 0.003397, 7e-06, 6e-06, 2e-06]},
    "right_foot": {"mass": 0.895, "com": [0.03206, -0.03701, -0.02883], "fullinertia": [0.000608, 0.00411, 0.004375, -0.0, 0.000177, 0.0]},
}

# 実 H2.urdf 由来の bone 質量分布（合計 1.0 に正規化）。
H2_MASS_FRACTION: dict[str, float] = {
    "pelvis": 0.001, "spine": 0.00595, "chest": 0.2442, "neck": 0.001, "head": 0.04111,
    "left_shoulder": 0.001, "left_elbow": 0.03499, "left_wrist": 0.04891,
    "right_shoulder": 0.001, "right_elbow": 0.03499, "right_wrist": 0.04891,
    "left_hip": 0.05632, "left_knee": 0.14138, "left_ankle": 0.05899, "left_foot": 0.01179,
    "right_hip": 0.05632, "right_knee": 0.14138, "right_ankle": 0.05899, "right_foot": 0.01179,
}

MORPHOLOGY = RobotMorphology(
    name=ROBOT_NAME,
    rest_pose=H2_REST,
    urdf_ref="Unitree unitree_ros h2_description/H2.urdf（実寸由来, BSD-3, 本体は別途取得）",
    runtime_adapter="unitree_sdk2",
    per_joint_limits=H2_JOINT_LIMITS,
    mass_distribution=H2_MASS_FRACTION,
    # inertia_tensors は EMBODIMENT_INERTIA registry 経由で real_inertia=True 時に装着（既定 capsule）。
    # H2 は大型（1.76m/75.6kg）。脚 actuator は 360N·m と強力。H1 相当の kp/kd を採用。
    sim_defaults=SimDefaults(total_mass=75.587, kp=200.0, kd=10.0, torque_limit=200.0),
)

# 後方互換のモジュールレベル別名。
BONE_LENGTHS = MORPHOLOGY.bone_lengths
NOMINAL_HEIGHT = MORPHOLOGY.nominal_height


def embodiment_dict() -> dict:
    """RD-Embodiment v0 schema 適合の dict を返す。"""
    return MORPHOLOGY.to_rd_embodiment()
