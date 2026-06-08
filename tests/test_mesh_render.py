"""実 URDF メッシュレンダリング（mesh fight）の検証。"""

from __future__ import annotations

import os
from pathlib import Path

import pytest

from robotdance_sim.mesh_render import resolve_unitree_urdf

_G1 = os.environ.get("ROBOTDANCE_G1_URDF", "") or str(
    Path.home() / "tmp/g1_meshes/unitree_ros/robots/g1_description/g1_23dof.urdf"
)
_has_g1 = Path(_G1).expanduser().is_file()
_skip = pytest.mark.skipif(not _has_g1, reason="実 G1 URDF が無い")


@_skip
def test_resolve_unitree_urdf_g1() -> None:
    path = resolve_unitree_urdf("unitree_g1")
    assert path.is_file()


def test_resolve_unknown_robot_raises() -> None:
    with pytest.raises(ValueError, match="Unitree"):
        resolve_unitree_urdf("fourier_n1")


@_skip
def test_run_fight_mesh_mode() -> None:
    pytest.importorskip("torch")
    pytest.importorskip("mujoco")
    pytest.importorskip("pybullet")
    from robotdance_sim.arena import run_fight
    from robotdance_unitree import get_morphology

    res = run_fight(get_morphology("unitree_g1"), get_morphology("unitree_h1"),
                    name_a="unitree_g1", name_b="unitree_h1", duration=2.0,
                    mesh=True, urdf_a=_G1, urdf_b=resolve_unitree_urdf("unitree_h1"))
    assert res.p1_hits >= 0
    assert len(res.frames) > 0
    assert res.frames[0].shape[2] == 3
