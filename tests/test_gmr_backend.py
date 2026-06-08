"""GMR retarget backend — conversion helpers and optional integration."""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pytest

from robotdance_core.skeleton import JOINT_NAMES, index_of
from robotdance_core.synthetic import generate_dance
from robotdance_retarget.backends import GMR, get_retarget_backend
from robotdance_retarget.dispatch import retarget_with_backend
from robotdance_retarget.gmr_backend import (
    ROBOT_TO_GMR,
    canonical_frame_to_gmr,
    gmr_assets_available,
    gmr_available,
    gmr_importable,
    gmr_install_hint,
    gmr_retarget,
)
from robotdance_unitree import get_morphology


def test_gmr_registry_cli_and_robot_map():
    assert GMR.cli == "retarget --backend gmr"
    assert "unitree_g1" in ROBOT_TO_GMR
    assert get_retarget_backend("gmr").name == "gmr"


def test_canonical_frame_to_gmr_has_required_bodies():
    mir = generate_dance(duration=0.2)
    kps = mir.keypoints_3d_array()[0]
    frame = canonical_frame_to_gmr(kps)
    for body in (
        "pelvis", "spine3", "left_hip", "right_hip", "left_knee", "right_knee",
        "left_foot", "right_foot", "left_shoulder", "right_shoulder",
        "left_elbow", "right_elbow", "left_wrist", "right_wrist",
    ):
        assert body in frame
        pos, quat = frame[body]
        assert pos.shape == (3,)
        assert quat.shape == (4,)
        assert abs(np.linalg.norm(quat) - 1.0) < 0.01


def test_dispatch_kinematic_default():
    mir = generate_dance(duration=0.2)
    motion = retarget_with_backend(mir, get_morphology("unitree_g1"), "kinematic")
    assert motion.robot_name == "unitree_g1"
    assert motion.keypoints_3d is not None


def test_dispatch_gmr_unavailable_raises_or_skips():
    mir = generate_dance(duration=0.2)
    if not gmr_available():
        with pytest.raises(RuntimeError, match="GMR"):
            gmr_retarget(mir, get_morphology("unitree_g1"))
    else:
        motion = retarget_with_backend(mir, get_morphology("unitree_g1"), "gmr")
        m = motion.retarget_metrics or {}
        assert m.get("backend") == "gmr"
        assert motion.keypoints_3d is not None


def test_gmr_unsupported_robot():
    if not gmr_available():
        pytest.skip("GMR 未導入")
    mir = generate_dance(duration=0.2)
    with pytest.raises(ValueError, match="未対応"):
        gmr_retarget(mir, get_morphology("apptronik_apollo"))


@pytest.mark.skipif(not gmr_available(), reason="GMR + assets 未導入")
def test_gmr_retarget_g1_metrics():
    mir = generate_dance(duration=0.5)
    motion = gmr_retarget(mir, get_morphology("unitree_g1"), verbose=False)
    m = motion.retarget_metrics or {}
    assert m.get("backend") == "gmr"
    assert m.get("bone_direction_cosine", 0) > 0.3
    kps = motion.keypoints_3d_array()
    assert kps.shape[1] == len(JOINT_NAMES)
    assert kps[:, index_of("pelvis"), 2].min() >= 0.0


def test_retarget_cli_gmr_flag_parses(tmp_path: Path):
    from robotdance_core.cli import main

    mir = generate_dance(duration=0.2)
    p = tmp_path / "in.rdmir.json"
    p.write_text(json.dumps(mir.to_dict()), encoding="utf-8")
    out = tmp_path / "out.rdmotion.json"
    code = main(["retarget", str(p), "-o", str(out), "--backend", "gmr", "--robot", "unitree_g1"])
    if gmr_available():
        assert code == 0
        assert out.is_file()
    else:
        assert code == 1


def test_gmr_install_hint_non_empty():
    assert "github.com/YanjieZe/GMR" in gmr_install_hint()


def test_gmr_availability_helpers_type():
    assert isinstance(gmr_importable(), bool)
    assert isinstance(gmr_assets_available(), bool)
