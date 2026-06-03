"""RD-Motion の Python データモデル（v0）。

specs/rd-motion/rd-motion.schema.json に対応。RD-MIR を RD-Embodiment へ retarget した
robot-specific モーション（.rdmotion）。v0 は kinematic retarget の link 位置を保持し、
sim_certificate は Phase 2（物理 sim）で埋める。
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Literal, Optional

import numpy as np
from pydantic import BaseModel, ConfigDict, Field

from .rd_mir import Skeleton

ControlMode = Literal["position", "velocity", "torque", "policy", "kinematic_preview"]


class RdMotion(BaseModel):
    """RD-Motion v0。"""

    model_config = ConfigDict(extra="forbid")

    rd_motion_version: Literal["0"] = "0"
    robot_name: str
    fps: float = Field(gt=0)
    duration: float = Field(gt=0)
    source_motion_id: str
    skeleton: Skeleton
    control_mode: ControlMode = "kinematic_preview"

    keypoints_3d: Optional[list[list[list[float]]]] = None
    joint_rotations: Optional[dict[str, Any]] = None
    base_trajectory: Optional[dict[str, Any]] = None
    contact_schedule: Optional[dict[str, list[Any]]] = None
    retarget_metrics: Optional[dict[str, Any]] = None
    safety_envelope: Optional[dict[str, Any]] = None
    sim_certificate: Optional[dict[str, Any]] = None
    source_provenance: Optional[dict[str, Any]] = None

    @property
    def num_frames(self) -> int:
        return round(self.fps * self.duration)

    def keypoints_3d_array(self) -> np.ndarray:
        if self.keypoints_3d is None:
            raise ValueError("keypoints_3d が設定されていません")
        return np.asarray(self.keypoints_3d, dtype=np.float64)

    def to_dict(self) -> dict[str, Any]:
        return self.model_dump(exclude_none=True)

    def save(self, path: str | Path, *, indent: int = 2) -> Path:
        path = Path(path)
        path.write_text(json.dumps(self.to_dict(), indent=indent), encoding="utf-8")
        return path

    @classmethod
    def load(cls, path: str | Path) -> "RdMotion":
        return cls.model_validate(json.loads(Path(path).read_text(encoding="utf-8")))
