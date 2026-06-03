"""MuJoCo 物理検証（sim_certificate）の縦スライス。

mujoco 未インストール環境では skip する。
"""

from __future__ import annotations

import pytest

pytest.importorskip("mujoco")

from robotdance_core.synthetic import generate_backflip, generate_dance  # noqa: E402
from robotdance_retarget.kinematic import retarget  # noqa: E402
from robotdance_sim.mujoco_backend import certify, simulate_certificate  # noqa: E402
from robotdance_unitree import get_morphology  # noqa: E402


@pytest.mark.parametrize("robot", ["unitree_g1", "unitree_h1"])
def test_safe_dance_passes(robot: str) -> None:
    morph = get_morphology(robot)
    motion = retarget(generate_dance(duration=2.0), morph)
    cert = simulate_certificate(motion, morph)
    assert cert["passed"] is True
    assert cert["verdict"] == "PASS"
    # 接地して支持されている。
    assert cert["metrics"]["airborne_ratio"] == 0.0
    # 典型トルクは物理的に妥当（特異姿勢の peak ではなく p50 で判定）。
    assert cert["metrics"]["torque_ratio"] < 1.5


@pytest.mark.parametrize("robot", ["unitree_g1", "unitree_h1"])
def test_backflip_is_rejected(robot: str) -> None:
    morph = get_morphology(robot)
    motion = retarget(generate_backflip(), morph)
    cert = simulate_certificate(motion, morph)
    assert cert["passed"] is False
    assert cert["verdict"] == "REJECT"
    assert cert["reasons"]  # 理由が付く
    # 滞空（接地なし）を検出している。
    assert cert["metrics"]["airborne_ratio"] > 0.5


def test_certify_attaches_to_motion() -> None:
    morph = get_morphology("unitree_g1")
    motion = retarget(generate_dance(duration=1.0), morph)
    assert motion.sim_certificate is None
    certify(motion, morph)
    assert motion.sim_certificate is not None
    assert motion.sim_certificate["backend"] == "mujoco"
    # certificate 付き motion も RD-Motion schema に適合する。
    import json
    from pathlib import Path

    import jsonschema

    schema = json.loads(
        (Path(__file__).resolve().parent.parent / "specs" / "rd-motion" / "rd-motion.schema.json")
        .read_text(encoding="utf-8")
    )
    jsonschema.Draft202012Validator(schema).validate(motion.to_dict())
