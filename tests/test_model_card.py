"""Model / Motion Card 生成（§7）の検証。

純 Python（numpy 程度）で torch/mujoco 不要 → CI でも走る。
"""

from __future__ import annotations

from robotdance_core.model_card import (
    build_mir_card,
    build_motion_card,
    license_composition,
    render_markdown,
)
from robotdance_core.synthetic import generate_dance
from robotdance_retarget.kinematic import retarget
from robotdance_unitree import get_morphology


def _mir(license_state: str = "research_only"):
    mir = generate_dance(duration=1.0)
    mir.license_state = license_state
    mir.extractor_versions = {"pose": "mediapipe_pose_landmarker_full", "adapter": "robotdance.v0"}
    mir.source_ref = {"local_path": "clip.mp4", "extractor": "mediapipe_pose_full"}
    return mir


def test_mir_card_sections_and_license() -> None:
    card = build_mir_card(_mir("research_only"))
    assert card["card_type"] == "mir"
    assert card["license"]["state"] == "research_only"
    assert card["license"]["redistribution"] is False
    # mediapipe シグナルから perception の failure mode が引かれる。
    assert any(f["area"] == "perception" for f in card["failure_modes"])
    assert card["identity"]["frames"] > 0


def test_motion_card_lineage_and_inherits_license() -> None:
    mir = _mir("redistributable")
    motion = retarget(mir, get_morphology("unitree_g1"))
    card = build_motion_card(motion, mir=mir)
    stages = [s["stage"] for s in card["lineage"]]
    assert stages[0] == "source_mir"
    assert "retarget" in stages
    assert "control_mode" in stages
    # license は source RD-MIR から継承。
    assert card["license"]["state"] == "redistributable"
    assert card["license"]["redistribution"] is True
    # retarget の failure mode が検出される。
    assert any(f["area"] == "retarget" for f in card["failure_modes"])


def test_motion_card_without_mir_is_unknown_license() -> None:
    motion = retarget(_mir(), get_morphology("unitree_g1"))
    card = build_motion_card(motion)  # mir 未指定
    assert card["license"]["state"] == "unknown"
    assert card["license"]["redistribution"] is False


def test_motion_card_surfaces_sim_certificate_when_present() -> None:
    motion = retarget(_mir(), get_morphology("unitree_g1"))
    motion.sim_certificate = {"backend": "mujoco", "verdict": "REJECT", "passed": False,
                              "reasons": ["airborne 80%"], "thresholds": {"airborne_ratio": 0.1},
                              "metrics": {"airborne_ratio": 0.8}}
    card = build_motion_card(motion)
    sc = card["safety_limits"]["sim_certificate"]
    assert sc["verdict"] == "REJECT"
    assert sc["reasons"]
    # simulation の failure mode も引かれる。
    assert any(f["area"] == "simulation" for f in card["failure_modes"])


def test_render_markdown_has_required_headers() -> None:
    card = build_motion_card(retarget(_mir(), get_morphology("unitree_g1")))
    md = render_markdown(card)
    for header in ("# RobotDance MOTION Card", "## Data Lineage", "## License",
                   "## Intended Use", "## Out of Scope",
                   "## Failure Modes / Known Limitations (v0)", "## Safety Limits"):
        assert header in md


def test_license_composition() -> None:
    comp = license_composition(["research_only", "redistributable", "redistributable", "unknown"])
    assert comp["total"] == 4
    assert comp["by_state"]["redistributable"] == 2
    assert comp["redistributable_fraction"] == 0.5
