"""Model / Motion Card 生成（§7）の検証。

純 Python（numpy 程度）で torch/mujoco 不要 → CI でも走る。
"""

from __future__ import annotations

from robotdance_core.model_card import (
    build_mir_card,
    build_motion_card,
    build_policy_card,
    card_for_artifact,
    license_composition,
    render_cards_index,
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


def test_motion_card_surfaces_joint_flexion_feasibility() -> None:
    # G1 morphology は per_joint_limits を持つ → retarget が joint_flexion を出す。
    motion = retarget(_mir(), get_morphology("unitree_g1"))
    assert "joint_flexion" in motion.retarget_metrics
    card = build_motion_card(motion)
    feas = card["safety_limits"]["kinematic_feasibility"]
    assert feas["joint_flexion_violation_ratio"] is not None
    assert set(feas["tracked_joints"]) >= {"left_knee", "left_elbow"}


def test_render_markdown_has_required_headers() -> None:
    card = build_motion_card(retarget(_mir(), get_morphology("unitree_g1")))
    md = render_markdown(card)
    for header in ("# RobotDance MOTION Card", "## Data Lineage", "## License",
                   "## Intended Use", "## Out of Scope",
                   "## Failure Modes / Known Limitations (v0)", "## Safety Limits"):
        assert header in md


def _policy():
    from robotdance_models.policy_export import tracking_policy_artifact

    return tracking_policy_artifact(
        obs_dim=121, action_dim=54, hidden=128, robot="unitree_g1",
        policy_id="rdpolicy-g1", weights_ref="policy.onnx", weights_format="onnx",
        weights_sha256="deadbeef", training={"framework": "ppo", "iterations": 40},
        reference_motion_ids=["dance", "idle"],
    )


def test_policy_card_sections_and_io_contract() -> None:
    card = build_policy_card(_policy())
    assert card["card_type"] == "policy"
    assert card["identity"]["policy_type"] == "tracking"
    assert card["identity"]["action_space"] == "residual_torque"
    assert card["io_contract"]["observation"]["dim"] == 121
    assert card["io_contract"]["action"]["base_actuated"] is False
    assert card["weights"]["format"] == "onnx"
    assert any(f["area"] == "control" for f in card["failure_modes"])
    # reference motions が lineage に出る。
    assert card["lineage"][0]["stage"] == "reference_motions"


def test_policy_card_markdown_has_io_and_weights() -> None:
    md = render_markdown(build_policy_card(_policy()))
    for header in ("# RobotDance POLICY Card", "## I/O Contract", "## Weights",
                   "## Safety Limits", "## Failure Modes / Known Limitations (v0)"):
        assert header in md


def test_card_for_artifact_dispatch(tmp_path) -> None:
    """ファイルから種別（mir/motion/policy）を判別してカードを生成する。"""
    mir = _mir()
    mir_path = mir.save(tmp_path / "c.rdmir.json")
    assert card_for_artifact(mir_path)["card_type"] == "mir"

    motion = retarget(mir, get_morphology("unitree_g1"))
    motion_path = motion.save(tmp_path / "c.rdmotion.json")
    assert card_for_artifact(motion_path)["card_type"] == "motion"

    pol = _policy()
    pol_path = pol.save(tmp_path / "p.rdpolicy.json")
    assert card_for_artifact(pol_path)["card_type"] == "policy"


def test_render_cards_index() -> None:
    rows = [
        {"type": "mir", "id": "m1", "license": "research_only", "failure_modes": 1,
         "summary": "action=dance", "card_file": "m1.CARD.md"},
        {"type": "policy", "id": "p1", "license": "unknown", "failure_modes": 2,
         "summary": "tracking", "card_file": "p1.CARD.md"},
    ]
    md = render_cards_index(rows)
    assert "# RobotDance Model Cards 索引" in md
    assert "research_only=1" in md and "unknown=1" in md  # license composition
    assert "[m1.CARD.md](m1.CARD.md)" in md
    assert "| mir | `m1` |" in md and "| policy | `p1` |" in md


def test_license_composition() -> None:
    comp = license_composition(["research_only", "redistributable", "redistributable", "unknown"])
    assert comp["total"] == 4
    assert comp["by_state"]["redistributable"] == 2
    assert comp["redistributable_fraction"] == 0.5
