"""URDF importer の検証。実 URDF は同梱しないため、G1 リンク名を持つ合成 URDF で検証する。"""

from __future__ import annotations

import json
from pathlib import Path

import jsonschema
import numpy as np

from robotdance_core.skeleton import NUM_JOINTS, index_of
from robotdance_unitree.urdf_import import (
    G1_LINK_MAP,
    link_world_positions,
    parse_urdf,
    urdf_to_morphology,
)

_ROOT = Path(__file__).resolve().parent.parent

# G1_LINK_MAP が参照するリンクを持つ最小 URDF（origin だけ定義、mesh なし）。
_LINKS = [
    ("pelvis", None, None),
    ("left_hip_pitch_link", "pelvis", "0 0.06 -0.10"),
    ("left_knee_link", "left_hip_pitch_link", "0 0 -0.30"),
    ("left_ankle_pitch_link", "left_knee_link", "0 0 -0.30"),
    ("right_hip_pitch_link", "pelvis", "0 -0.06 -0.10"),
    ("right_knee_link", "right_hip_pitch_link", "0 0 -0.30"),
    ("right_ankle_pitch_link", "right_knee_link", "0 0 -0.30"),
    ("torso_link", "pelvis", "0 0 0.20"),
    ("left_shoulder_pitch_link", "torso_link", "0 0.10 0.10"),
    ("left_elbow_link", "left_shoulder_pitch_link", "0 0 -0.20"),
    ("left_wrist_roll_rubber_hand", "left_elbow_link", "0.10 0 -0.05"),
    ("right_shoulder_pitch_link", "torso_link", "0 -0.10 0.10"),
    ("right_elbow_link", "right_shoulder_pitch_link", "0 0 -0.20"),
    ("right_wrist_roll_rubber_hand", "right_elbow_link", "0.10 0 -0.05"),
]


def _fixture_urdf(path: Path) -> Path:
    parts = ['<robot name="g1_fixture">']
    for name, _, _ in _LINKS:
        parts.append(f'  <link name="{name}"/>')
    for name, parent, xyz in _LINKS:
        if parent is None:
            continue
        parts.append(
            f'  <joint name="{name}_j" type="revolute">'
            f'<parent link="{parent}"/><child link="{name}"/>'
            f'<origin xyz="{xyz}" rpy="0 0 0"/>'
            f'<limit lower="-1" upper="1" effort="50" velocity="10"/></joint>'
        )
    parts.append("</robot>")
    path.write_text("\n".join(parts), encoding="utf-8")
    return path


def test_parse_and_fk(tmp_path: Path) -> None:
    urdf = _fixture_urdf(tmp_path / "g1.urdf")
    joints, root = parse_urdf(urdf)
    assert root == "pelvis"
    pos = link_world_positions(joints, root)
    # FK: 左膝 = hip(−0.10) + (−0.30) = z −0.40。
    np.testing.assert_allclose(pos["left_knee_link"], [0.0, 0.06, -0.40], atol=1e-6)
    np.testing.assert_allclose(pos["pelvis"], [0.0, 0.0, 0.0], atol=1e-6)


def test_morphology_from_urdf(tmp_path: Path) -> None:
    urdf = _fixture_urdf(tmp_path / "g1.urdf")
    morph = urdf_to_morphology(urdf, name="g1_fixture")
    rest = morph.rest_pose
    assert rest.shape == (NUM_JOINTS, 3)
    # z-up: 頭は骨盤より上、足首は下。
    assert rest[index_of("head")][2] > rest[index_of("pelvis")][2]
    assert rest[index_of("left_ankle")][2] < rest[index_of("pelvis")][2]
    # toe は足首より前方（合成）。
    assert rest[index_of("left_foot")][0] > rest[index_of("left_ankle")][0]
    # 実寸由来の bone 長（左脛 = 0.30）。
    knee = index_of("left_knee")
    shin = np.linalg.norm(rest[index_of("left_ankle")] - rest[knee])
    assert abs(shin - 0.30) < 1e-6
    assert morph.nominal_height > 0.5


def test_imported_morphology_schema_and_retarget(tmp_path: Path) -> None:
    from robotdance_core.synthetic import generate_dance
    from robotdance_retarget.kinematic import retarget

    morph = urdf_to_morphology(_fixture_urdf(tmp_path / "g1.urdf"), name="g1_fixture")
    schema = json.loads(
        (_ROOT / "specs" / "rd-embodiment" / "rd-embodiment.schema.json").read_text("utf-8"))
    jsonschema.Draft202012Validator(schema).validate(morph.to_rd_embodiment())
    motion = retarget(generate_dance(duration=1.0), morph)
    assert motion.keypoints_3d_array().shape[1] == NUM_JOINTS


def test_g1_link_map_covers_limbs() -> None:
    # 13 limb joint（pelvis + 各肢）をマップ、torso/toe は合成。
    assert len(G1_LINK_MAP) == 13
    assert "left_wrist" in G1_LINK_MAP and "right_ankle" in G1_LINK_MAP
