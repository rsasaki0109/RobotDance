"""RD-MIR の Python データモデル（v0）。

specs/rd-mir/rd-mir.schema.json を pydantic で表現し、JSON 入出力と numpy 変換を提供する。
metadata は型付きモデルで保持し、keypoints_3d などの大きな配列はネスト list として持つ
（JSON 互換）。実行時は `keypoints_3d_array()` で numpy に変換して使う。
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Literal, Optional

import numpy as np
from pydantic import BaseModel, ConfigDict, Field

LicenseState = Literal[
    "redistributable", "trainable", "commercial_allowed", "research_only", "unknown"
]


class WorldFrame(BaseModel):
    model_config = ConfigDict(extra="forbid")
    up_axis: Literal["x", "y", "z"] = "z"
    forward_axis: Literal["x", "y", "z"] = "x"
    handedness: Literal["right", "left"] = "right"


class Skeleton(BaseModel):
    model_config = ConfigDict(extra="forbid")
    joint_names: list[str] = Field(min_length=1)
    parents: Optional[list[int]] = None


class RdMir(BaseModel):
    """RD-MIR v0。schema-conformant な辞書へ `to_dict()` / `save()` で書き出せる。"""

    model_config = ConfigDict(extra="forbid")

    rd_mir_version: Literal["0"] = "0"
    motion_id: str
    source_ref: dict[str, Any]
    license_state: LicenseState
    fps: float = Field(gt=0)
    duration: float = Field(gt=0)
    world_frame: WorldFrame = Field(default_factory=WorldFrame)
    skeleton: Skeleton

    # 以下は optional。未設定なら出力 JSON から除外され、additionalProperties:false の schema に適合する。
    root_trajectory: Optional[dict[str, Any]] = None
    keypoints_3d: Optional[list[list[list[float]]]] = None
    keypoints_2d: Optional[list[list[list[float]]]] = None
    contacts: Optional[dict[str, list[Any]]] = None
    confidence: Optional[dict[str, Any]] = None
    camera: Optional[dict[str, Any]] = None
    quality_metrics: Optional[dict[str, Any]] = None
    semantics: Optional[dict[str, Any]] = None
    privacy_flags: Optional[dict[str, bool]] = None
    extractor_versions: Optional[dict[str, str]] = None
    retarget_certificates: Optional[list[dict[str, Any]]] = None

    # --- convenience ---

    @property
    def num_frames(self) -> int:
        """canonical なフレーム数（round(fps * duration)）。"""
        return round(self.fps * self.duration)

    def keypoints_3d_array(self) -> np.ndarray:
        """keypoints_3d を [T, J, 3] の numpy 配列で返す。"""
        if self.keypoints_3d is None:
            raise ValueError("keypoints_3d が設定されていません")
        return np.asarray(self.keypoints_3d, dtype=np.float64)

    def to_dict(self) -> dict[str, Any]:
        """schema-conformant な dict（未設定の optional を除外）。"""
        return self.model_dump(exclude_none=True)

    def save(self, path: str | Path, *, indent: int = 2) -> Path:
        path = Path(path)
        path.write_text(json.dumps(self.to_dict(), indent=indent), encoding="utf-8")
        return path

    @classmethod
    def load(cls, path: str | Path) -> "RdMir":
        data = json.loads(Path(path).read_text(encoding="utf-8"))
        return cls.model_validate(data)
