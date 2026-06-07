"""Fourier N1 embodiment（v0）。

canonical 19-joint と同一トポロジの Fourier N1 形態（6 機種目, compact ~1.15m / 39.7kg）。
rest pose / 関節 limit（位置）/ 質量分布 / 慣性テンソルは **MuJoCo Menagerie の公式 N1 モデル
（Apache-2.0）の実値**から抽出（数値定数のみ・mesh/MJCF 本体は非同梱, license-safe）。

抽出は MuJoCo にモデルを読み込ませ world frame / world 軸慣性を厳密計算（diaginertia+quat の回転を
MuJoCo が処理）→ 各 body を手動で canonical bone（pelvis/spine/chest/四肢）へ割当て・剛体合成
（平行軸）。N1 は胴体に waist_yaw 1 関節、首は固定（neck/head 関節なし）。

⚠️ v0 注意:
- **トルク（forcerange）は menagerie MJCF に未収載**（actuator forcerange=[0,0]）→ torque feasibility 軸は
  generic fallback。位置 ROM / 質量 / 慣性 / 寸法 / balance は実 N1 値。velocity も未収載。
- runtime adapter は実機 SDK 不明のため sim（mujoco）扱い。

出典: google-deepmind/mujoco_menagerie `fourier_n1`（Apache-2.0, Fourier Intelligence N1 由来）。
"""

from __future__ import annotations

import numpy as np

from robotdance_retarget.embodiment import RobotMorphology, SimDefaults

ROBOT_NAME = "fourier_n1"

# 実 N1 モデル由来の canonical 19-joint rest pose（z-up, x-forward, y-left, m）。MuJoCo qpos0 立位の
# body world xpos から対応づけ、左右対称化。pelvis≈骨盤、chest=torso、head=camera、足先 z≈0.02。
N1_REST = np.array(
    [
        [0.000, 0.000, 0.700],   # 0 pelvis (base_link)
        [0.000, 0.000, 0.726],   # 1 spine (waist_yaw)
        [0.000, 0.000, 0.985],   # 2 chest (torso)
        [0.000, 0.000, 1.075],   # 3 neck (補間, 関節なし)
        [0.071, 0.000, 1.165],   # 4 head (camera)
        [0.000, 0.098, 0.985],   # 5 left_shoulder
        [0.000, 0.176, 0.775],   # 6 left_elbow
        [0.101, 0.176, 0.775],   # 7 left_wrist
        [0.000, -0.098, 0.985],  # 8 right_shoulder
        [0.000, -0.176, 0.775],  # 9 right_elbow
        [0.101, -0.176, 0.775],  # 10 right_wrist
        [0.000, 0.051, 0.633],   # 11 left_hip
        [0.000, 0.120, 0.325],   # 12 left_knee
        [0.000, 0.120, 0.045],   # 13 left_ankle
        [0.100, 0.120, 0.020],   # 14 left_foot
        [0.000, -0.051, 0.633],  # 15 right_hip
        [0.000, -0.120, 0.325],  # 16 right_knee
        [0.000, -0.120, 0.045],  # 17 right_ankle
        [0.100, -0.120, 0.020],  # 18 right_foot
    ],
    dtype=np.float64,
)

# 実 N1 モデル由来の canonical 関節 limit（位置 rad）。複数 DOF → 位置 envelope（最広）。
# **トルクは menagerie MJCF の forcerange=[0,0] のため未収載**（follow-up）。neck/head は関節なし→省略。
# 膝は屈曲のみ [-0.087, 2.356]、肩は広レンジ、足首は狭レンジ。左右非対称（roll 符号）は envelope で吸収。
N1_JOINT_LIMITS: dict[str, dict[str, object]] = {
    "spine": {"position": [-2.617, 2.617]},
    "left_shoulder": {"position": [-2.966, 2.966]},
    "left_elbow": {"position": [-0.349, 1.658]},
    "left_wrist": {"position": [-1.832, 1.832]},
    "right_shoulder": {"position": [-2.966, 2.966]},
    "right_elbow": {"position": [-0.349, 1.658]},
    "right_wrist": {"position": [-1.832, 1.832]},
    "left_hip": {"position": [-2.617, 2.617]},
    "left_knee": {"position": [-0.0872, 2.356]},
    "left_ankle": {"position": [-0.785, 0.785]},
    "right_hip": {"position": [-2.617, 2.617]},
    "right_knee": {"position": [-0.0872, 2.356]},
    "right_ankle": {"position": [-0.785, 0.785]},
}

# 実 N1 <inertial> mass 由来の canonical 質量分布（Σ=1）。胸（torso 8.0kg）が最重量 20%、脚は
# 大腿(knee) > 下腿(ankle) > 股(hip)。総質量 39.727kg。左右対称。
N1_MASS_FRACTION: dict[str, float] = {
    "pelvis": 0.0800, "spine": 0.0800, "chest": 0.2021, "head": 0.0010,
    "left_shoulder": 0.0403, "left_elbow": 0.0230, "left_wrist": 0.0316,
    "right_shoulder": 0.0403, "right_elbow": 0.0230, "right_wrist": 0.0316,
    "left_hip": 0.0670, "left_knee": 0.0848, "left_ankle": 0.0585, "left_foot": 0.0133,
    "right_hip": 0.0670, "right_knee": 0.0848, "right_ankle": 0.0585, "right_foot": 0.0133,
}

# 実 N1 <inertial>（diaginertia+quat）を MuJoCo で world 軸へ展開し canonical bone へ平行軸合成
# （質量 kg / com[3]=joint 自身相対 m / fullinertia[6]=COM まわり世界軸）。
# get_morphology("fourier_n1", real_inertia=True) で使用。
N1_INERTIA_TENSORS: dict[str, dict] = {
    "pelvis": {"mass": 3.18, "com": [0.00025, 1e-05, -0.05502], "fullinertia": [0.011139, 0.007619, 0.0089904, -1e-07, -2.46e-05, -9.8e-06]},
    "spine": {"mass": 3.18, "com": [0.00025, 1e-05, -0.05542], "fullinertia": [0.011139, 0.007619, 0.0089904, -1e-07, -2.46e-05, -9.8e-06]},
    "chest": {"mass": 8.028, "com": [0.00702, 0.00038, -0.04319], "fullinertia": [0.0876598, 0.0765719, 0.0354319, 4.87e-05, -0.000372, 0.0002843]},
    "head": {"mass": 0.0382, "com": [0.01063, -0.00384, -0.00589], "fullinertia": [2.46e-05, 3.7e-06, 2.48e-05, -1.3e-06, 1e-07, -1e-07]},
    "left_shoulder": {"mass": 1.601, "com": [0.00196, 0.06841, -0.01895], "fullinertia": [0.0034927, 0.0031754, 0.0014392, -2.18e-05, -5.91e-05, 0.0002988]},
    "left_elbow": {"mass": 0.913, "com": [0.0, -0.00093, 0.04917], "fullinertia": [0.0028954, 0.0029133, 0.0006053, -1.9e-06, -5e-06, -3.92e-05]},
    "left_wrist": {"mass": 1.2554, "com": [-0.02, 0.0025, -0.00036], "fullinertia": [0.0008152, 0.0052614, 0.0054026, 0.0002869, -3.14e-05, -8e-07]},
    "right_shoulder": {"mass": 1.601, "com": [0.00196, -0.06841, -0.01904], "fullinertia": [0.0034869, 0.0031696, 0.0014387, 2.17e-05, -6.36e-05, -0.0002998]},
    "right_elbow": {"mass": 0.913, "com": [0.0, 0.00091, 0.04917], "fullinertia": [0.0028945, 0.0029142, 0.0006053, -5.1e-06, 3.4e-06, 3.94e-05]},
    "right_wrist": {"mass": 1.2554, "com": [-0.02, -0.00248, 0.0003], "fullinertia": [0.0008138, 0.0052608, 0.0054008, -0.0002863, -5.89e-05, 1.2e-06]},
    "left_hip": {"mass": 2.66, "com": [-0.00991, 0.05167, -0.02439], "fullinertia": [0.0053985, 0.0050605, 0.0057271, 0.0004736, 0.0001358, 0.0009616]},
    "left_knee": {"mass": 3.37, "com": [0.00214, -0.01061, 0.12088], "fullinertia": [0.01475, 0.014824, 0.005307, -8.53e-05, 0.0005867, -0.0008227]},
    "left_ankle": {"mass": 2.323, "com": [0.00264, 0.00484, 0.1375], "fullinertia": [0.0158006, 0.0158232, 0.0016556, 2.85e-05, -0.0002187, -0.00029]},
    "left_foot": {"mass": 0.528, "com": [-0.06892, -0.00011, -0.00686], "fullinertia": [0.0002537, 0.0016804, 0.0018324, 6.2e-06, 9.4e-06, -7e-07]},
    "right_hip": {"mass": 2.66, "com": [-0.0099, -0.05167, -0.02439], "fullinertia": [0.005398, 0.0050606, 0.0057266, -0.0004793, 0.0001358, -0.0009669]},
    "right_knee": {"mass": 3.37, "com": [0.00204, 0.01008, 0.12088], "fullinertia": [0.014825, 0.014882, 0.005312, 4.65e-05, 0.000593, 0.000795]},
    "right_ankle": {"mass": 2.323, "com": [0.00256, -0.00464, 0.13706], "fullinertia": [0.0156132, 0.0156269, 0.0016738, -2.6e-05, -0.00021, 0.0002711]},
    "right_foot": {"mass": 0.528, "com": [-0.06892, 0.00016, -0.00659], "fullinertia": [0.0002537, 0.0016803, 0.0018324, -6.6e-06, 9.4e-06, -1e-07]},
}

MORPHOLOGY = RobotMorphology(
    name=ROBOT_NAME,
    rest_pose=N1_REST,
    urdf_ref="mujoco_menagerie fourier_n1（Apache-2.0, Fourier N1 由来, 本体は別途取得）",
    runtime_adapter="mujoco",
    per_joint_limits=N1_JOINT_LIMITS,
    mass_distribution=N1_MASS_FRACTION,
    # inertia_tensors は EMBODIMENT_INERTIA registry 経由で real_inertia=True 時に装着（既定 capsule）。
    # N1 は compact（1.15m/39.7kg）。トルク未収載のため PD/torque 既定は generic（小型）。
    sim_defaults=SimDefaults(total_mass=39.727, kp=200.0, kd=8.0, torque_limit=60.0),
)

BONE_LENGTHS = MORPHOLOGY.bone_lengths
NOMINAL_HEIGHT = MORPHOLOGY.nominal_height


def embodiment_dict() -> dict:
    """RD-Embodiment v0 schema 適合の dict を返す。"""
    return MORPHOLOGY.to_rd_embodiment()
