"""multi-embodiment（G1 + H1）と汎用 retarget の検証。"""

from __future__ import annotations

import json
from pathlib import Path

import jsonschema
import pytest

from robotdance_core.skeleton import NUM_JOINTS
from robotdance_core.synthetic import generate_dance
from robotdance_retarget.kinematic import retarget
from robotdance_unitree import EMBODIMENTS, get_morphology

_ROOT = Path(__file__).resolve().parent.parent
_EMB_SCHEMA = json.loads(
    (_ROOT / "specs" / "rd-embodiment" / "rd-embodiment.schema.json").read_text(encoding="utf-8")
)
_MOTION_SCHEMA = json.loads(
    (_ROOT / "specs" / "rd-motion" / "rd-motion.schema.json").read_text(encoding="utf-8")
)


def test_registry_has_g1_and_h1() -> None:
    assert set(EMBODIMENTS) == {"unitree_g1", "unitree_h1"}
    assert get_morphology("unitree_g1").name == "unitree_g1"
    with pytest.raises(KeyError):
        get_morphology("nonexistent_robot")


@pytest.mark.parametrize("name", sorted(EMBODIMENTS))
def test_each_embodiment_conforms(name: str) -> None:
    jsonschema.Draft202012Validator(_EMB_SCHEMA).validate(get_morphology(name).to_rd_embodiment())


def test_h1_is_taller_than_g1_and_human() -> None:
    g1_h = get_morphology("unitree_g1").nominal_height
    h1_h = get_morphology("unitree_h1").nominal_height
    assert h1_h > g1_h  # H1 は full-size、G1 は小型

    mir = generate_dance(duration=1.0, fps=30.0)
    h1_motion = retarget(mir, get_morphology("unitree_h1"))
    # H1 は人間より背が高い → height_scale > 1。
    assert h1_motion.retarget_metrics["height_scale"] > 1.0


@pytest.mark.parametrize("name", sorted(EMBODIMENTS))
def test_generic_retarget_shapes_and_schema(name: str) -> None:
    mir = generate_dance(duration=1.0, fps=30.0)
    motion = retarget(mir, get_morphology(name))
    assert motion.robot_name == name
    assert motion.keypoints_3d_array().shape == (30, NUM_JOINTS, 3)
    assert motion.retarget_metrics["bone_direction_cosine"] > 0.99
    jsonschema.Draft202012Validator(_MOTION_SCHEMA).validate(motion.to_dict())


def test_render_three_panels(tmp_path: Path) -> None:
    pytest.importorskip("matplotlib")
    from robotdance_viewer.skeleton_view import render_side_by_side

    mir = generate_dance(duration=0.5, fps=20.0)
    panels = [(mir.keypoints_3d_array(), "human", "#1f77b4")]
    for name in ("unitree_g1", "unitree_h1"):
        panels.append((retarget(mir, get_morphology(name)).keypoints_3d_array(), name, "#ff7f0e"))
    out = render_side_by_side(panels, tmp_path / "multi.gif", fps=20.0, stride=2)
    assert out.exists() and out.stat().st_size > 0
