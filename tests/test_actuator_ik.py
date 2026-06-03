"""アクチュエータ空間 IK（微分可能 FK + 勾配 IK）の検証。torch 無しは skip。"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pytest

pytest.importorskip("torch")

import torch  # noqa: E402

from robotdance_core.synthetic import generate_dance  # noqa: E402
from robotdance_retarget.actuator_ik import G1Chain, actuator_retarget  # noqa: E402
from robotdance_unitree.urdf_import import link_world_positions, parse_urdf  # noqa: E402

_ROOT = Path(__file__).resolve().parent.parent

# actuator IK 用 fixture: G1_LINK_MAP のリンク名 + revolute 軸 + limit を持つ最小 URDF。
_J = [
    ("left_hip_pitch_link", "pelvis", "0 0.06 -0.10", "0 1 0"),
    ("left_knee_link", "left_hip_pitch_link", "0 0 -0.30", "0 1 0"),
    ("left_ankle_pitch_link", "left_knee_link", "0 0 -0.30", "0 1 0"),
    ("right_hip_pitch_link", "pelvis", "0 -0.06 -0.10", "0 1 0"),
    ("right_knee_link", "right_hip_pitch_link", "0 0 -0.30", "0 1 0"),
    ("right_ankle_pitch_link", "right_knee_link", "0 0 -0.30", "0 1 0"),
    ("torso_link", "pelvis", "0 0 0.20", "0 0 1"),
    ("left_shoulder_pitch_link", "torso_link", "0 0.10 0.10", "0 1 0"),
    ("left_elbow_link", "left_shoulder_pitch_link", "0 0 -0.20", "0 1 0"),
    ("left_wrist_roll_rubber_hand", "left_elbow_link", "0.10 0 -0.05", "1 0 0"),
    ("right_shoulder_pitch_link", "torso_link", "0 -0.10 0.10", "0 1 0"),
    ("right_elbow_link", "right_shoulder_pitch_link", "0 0 -0.20", "0 1 0"),
    ("right_wrist_roll_rubber_hand", "right_elbow_link", "0.10 0 -0.05", "1 0 0"),
]


def _fixture(path: Path) -> Path:
    p = ['<robot name="g1f"><link name="pelvis"/>']
    for n, _, _, _ in _J:
        p.append(f'<link name="{n}"/>')
    for n, parent, xyz, axis in _J:
        p.append(
            f'<joint name="{n}_joint" type="revolute"><parent link="{parent}"/>'
            f'<child link="{n}"/><origin xyz="{xyz}" rpy="0 0 0"/><axis xyz="{axis}"/>'
            f'<limit lower="-2.5" upper="2.5" effort="50" velocity="20"/></joint>'
        )
    p.append("</robot>")
    path.write_text("\n".join(p), encoding="utf-8")
    return path


def test_differentiable_fk_matches_urdf_rest(tmp_path) -> None:
    urdf = _fixture(tmp_path / "g1.urdf")
    chain = G1Chain(urdf)
    pos = chain.fk(torch.zeros(1, chain.n_act))[0].numpy()
    joints, root = parse_urdf(urdf)
    ref = link_world_positions(joints, root)
    err = max(np.linalg.norm(pos[chain.link_index(li.name)] - ref[li.name])
              for li in chain.links[1:])
    assert err < 1e-5         # q=0 FK は URDF rest を厳密再現
    assert chain.n_act == 13


def test_actuator_retarget_outputs_joint_angles(tmp_path) -> None:
    urdf = _fixture(tmp_path / "g1.urdf")
    motion = actuator_retarget(generate_dance(duration=1.0), urdf, steps=120)
    jr = motion.joint_rotations
    angles = np.asarray(jr["angles_rad"])
    assert angles.shape == (motion.num_frames, 13)
    assert len(jr["actuated_joint_names"]) == 13
    # clamp 後は limit 内。
    assert angles.min() >= -2.5 - 1e-5 and angles.max() <= 2.5 + 1e-5
    # IK 誤差は妥当な範囲（参照 IK）。
    assert motion.retarget_metrics["ik_mean_pos_error_m"] < 0.2


def test_actuator_rdmotion_schema(tmp_path) -> None:
    urdf = _fixture(tmp_path / "g1.urdf")
    motion = actuator_retarget(generate_dance(duration=0.7), urdf, steps=60)
    schema = json.loads((_ROOT / "specs" / "rd-motion" / "rd-motion.schema.json").read_text("utf-8"))
    import jsonschema
    jsonschema.Draft202012Validator(schema).validate(motion.to_dict())
