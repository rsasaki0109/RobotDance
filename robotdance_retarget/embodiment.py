"""robot 形態の汎用抽象（v0）。

特定ロボットに依存せず、canonical 19-joint トポロジ上の rest pose（link 位置）から
bone 長・概略全高を導き、RD-Embodiment v0 schema 適合の dict を生成する。
各ロボット（G1 / H1 等）は rest pose と名前を与えて `RobotMorphology` を作るだけでよい。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

import numpy as np

from robotdance_core.skeleton import JOINT_NAMES, PARENTS

# actuator の無い合成関節（torso 連鎖・toe 等）向けの概略 joint limit プレースホルダ
# （位置 rad / 速度 rad·s⁻¹ / トルク N·m）。実 actuator がある関節は URDF 由来の
# `per_joint_limits`（urdf_to_morphology / 各 embodiment の *_JOINT_LIMITS）で上書きする。
_GENERIC_LIMIT = {"position": [-3.14, 3.14], "velocity": 12.0, "torque": 60.0}

_PARENT_IDX = np.array([max(p, 0) for p in PARENTS])


@dataclass(frozen=True)
class SimDefaults:
    """embodiment 固有の MuJoCo tracking 既定（概算質量・関節 PD ゲイン）。

    背が高く手足の長い機種ほど実効関節慣性が大きく、**より高い kd（関節ダンピング）**が要る。
    kd が不足すると PD が振動して横倒れする（実証: H1 は G1 の kd=6 だと振動して転倒し、
    kd=10 で upright=1.0 安定）。質量も機種実機相当にする（G1≈35kg, H1≈47kg）。
    これらを embodiment に紐付けることで「既定値が特定機種に隠れ調整される」事故を防ぐ。
    """

    total_mass: float = 35.0
    kp: float = 150.0
    kd: float = 6.0
    torque_limit: float = 80.0


@dataclass
class RobotMorphology:
    """canonical トポロジ上の robot 形態。"""

    name: str
    rest_pose: np.ndarray  # [J, 3] world rest 位置（z-up, x-forward, y-left, m）
    runtime_adapter: str = "unitree_sdk2"
    urdf_ref: str = "TODO(Phase2): real URDF"
    end_effectors: tuple[str, ...] = ("left_foot", "right_foot", "left_wrist", "right_wrist")
    control_modes: tuple[str, ...] = ("position", "policy")
    joint_limit: dict[str, Any] = field(default_factory=lambda: dict(_GENERIC_LIMIT))
    # canonical 関節名 → 実 URDF 由来の {position[lo,hi], velocity, torque}。actuator がある
    # 関節のみ。未設定（None）や未収載の関節は `joint_limit` プレースホルダにフォールバックする。
    per_joint_limits: dict[str, Any] | None = None
    sim_defaults: SimDefaults = field(default_factory=SimDefaults)

    def __post_init__(self) -> None:
        self.rest_pose = np.asarray(self.rest_pose, dtype=np.float64)
        if self.rest_pose.shape != (len(JOINT_NAMES), 3):
            raise ValueError(
                f"rest_pose は {(len(JOINT_NAMES), 3)} 形状が必要: {self.rest_pose.shape}"
            )

    @property
    def bone_lengths(self) -> np.ndarray:
        """親→子の bone 長 [J]（root は 0 相当）。"""
        return np.linalg.norm(self.rest_pose - self.rest_pose[_PARENT_IDX], axis=1)

    @property
    def nominal_height(self) -> float:
        """rest の概略全高（足先〜頭頂）。"""
        return float(self.rest_pose[:, 2].max() - self.rest_pose[:, 2].min())

    def limit_for(self, joint_name: str) -> dict[str, Any]:
        """canonical 関節の limit を返す。実 per-joint limit があればそれ、無ければ placeholder。"""
        if self.per_joint_limits and joint_name in self.per_joint_limits:
            return dict(self.per_joint_limits[joint_name])
        return dict(self.joint_limit)

    def to_rd_embodiment(self) -> dict[str, Any]:
        """RD-Embodiment v0 schema 適合の dict を返す。

        v0 では canonical joint 名を流用（実 actuator 名への写像は Phase 2）。
        joint_limits は実 actuator がある関節は URDF 由来の実値、合成関節は placeholder。
        """
        bones = self.bone_lengths
        return {
            "rd_embodiment_version": "0",
            "robot_name": self.name,
            "urdf_ref": self.urdf_ref,
            "joint_names": list(JOINT_NAMES),
            "joint_limits": {name: self.limit_for(name) for name in JOINT_NAMES},
            "link_lengths": {
                JOINT_NAMES[j]: float(bones[j])
                for j in range(len(JOINT_NAMES))
                if PARENTS[j] >= 0
            },
            "end_effectors": list(self.end_effectors),
            "control_modes": list(self.control_modes),
            # nominal_pose（joint 角）は actuator 写像が決まる Phase 2 で設定。
            "runtime_adapter": self.runtime_adapter,
        }
