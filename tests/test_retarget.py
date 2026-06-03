"""G1 kinematic retarget の縦スライスを検証する。"""

from __future__ import annotations

import json
from pathlib import Path

import jsonschema
import pytest

from robotdance_core.rd_motion import RdMotion
from robotdance_core.skeleton import FOOT_JOINTS, NUM_JOINTS
from robotdance_core.synthetic import generate_dance
from robotdance_retarget.kinematic import retarget_to_g1
from robotdance_unitree import g1

_ROOT = Path(__file__).resolve().parent.parent


def _schema(name: str) -> dict:
    return json.loads((_ROOT / "specs" / name).read_text(encoding="utf-8"))


def test_g1_embodiment_conforms() -> None:
    jsonschema.Draft202012Validator(
        _schema("rd-embodiment/rd-embodiment.schema.json")
    ).validate(g1.embodiment_dict())


def test_retarget_shapes_and_schema() -> None:
    mir = generate_dance(duration=1.0, fps=30.0)
    motion = retarget_to_g1(mir)
    assert motion.robot_name == "unitree_g1"
    assert motion.keypoints_3d_array().shape == (30, NUM_JOINTS, 3)
    jsonschema.Draft202012Validator(
        _schema("rd-motion/rd-motion.schema.json")
    ).validate(motion.to_dict())


def test_retarget_is_shorter_and_grounded() -> None:
    mir = generate_dance(duration=1.0, fps=30.0)
    motion = retarget_to_g1(mir)
    robot = motion.keypoints_3d_array()
    # G1 プロキシは人間より低い（height_scale < 1）。
    assert motion.retarget_metrics["height_scale"] < 1.0
    # 接地クランプ: 足が地面付近にあり、地面を大きく貫かない。
    foot_idx = [i for pair in FOOT_JOINTS.values() for i in pair]
    assert robot[:, foot_idx, 2].min() >= -1e-6


def test_bone_directions_preserved() -> None:
    """direction-preserving なので人間と robot の bone 方向はほぼ一致（cos≈1）。"""
    mir = generate_dance(duration=1.0, fps=30.0)
    motion = retarget_to_g1(mir)
    assert motion.retarget_metrics["bone_direction_cosine"] > 0.99


def test_roundtrip(tmp_path: Path) -> None:
    mir = generate_dance(duration=1.0, fps=30.0)
    p = retarget_to_g1(mir).save(tmp_path / "g1.rdmotion.json")
    loaded = RdMotion.load(p)
    assert loaded.source_motion_id == mir.motion_id


def test_side_by_side_render(tmp_path: Path) -> None:
    pytest.importorskip("matplotlib")
    from robotdance_viewer.skeleton_view import render_side_by_side

    mir = generate_dance(duration=0.5, fps=20.0)
    motion = retarget_to_g1(mir)
    out = render_side_by_side(
        [
            (mir.keypoints_3d_array(), "human", "#1f77b4"),
            (motion.keypoints_3d_array(), "g1", "#ff7f0e"),
        ],
        tmp_path / "pair.gif",
        fps=20.0,
        stride=2,
    )
    assert out.exists() and out.stat().st_size > 0
