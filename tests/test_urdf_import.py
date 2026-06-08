"""URDF importer の検証。実 URDF は同梱しないため、G1 リンク名を持つ合成 URDF で検証する。"""

from __future__ import annotations

import json
from pathlib import Path

import jsonschema
import numpy as np

from robotdance_core.skeleton import NUM_JOINTS, index_of
from robotdance_unitree.urdf_import import (
    G1_LINK_MAP,
    H1_LINK_MAP,
    H2_LINK_MAP,
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


def test_h2_link_map_covers_core_limbs_and_targets_distal_links() -> None:
    # H2 は手首まで持つ（G1 と同型 13 マップ）。足首/手首は遠位 link を指す。
    core = {"pelvis", "left_hip", "right_hip", "left_knee", "right_knee",
            "left_ankle", "right_ankle", "left_shoulder", "right_shoulder",
            "left_elbow", "right_elbow", "left_wrist", "right_wrist"}
    assert core <= set(H2_LINK_MAP)
    # 足首は roll でなく pitch（遠位）、手首は yaw（遠位）を FK ターゲットにする。
    assert H2_LINK_MAP["left_ankle"] == "left_ankle_pitch_link"
    assert H2_LINK_MAP["right_wrist"] == "right_wrist_yaw_link"
    # H1（肘止まり）は wrist を持たない。
    assert "left_wrist" not in H1_LINK_MAP


def test_urdf_import_brings_real_per_joint_limits(tmp_path: Path) -> None:
    """URDF 由来 morphology は actuated 関節に実 limit を持ち、合成関節は placeholder に落ちる。"""
    morph = urdf_to_morphology(_fixture_urdf(tmp_path / "g1.urdf"), name="g1_fixture")
    jl = morph.to_rd_embodiment()["joint_limits"]
    # fixture の actuated 関節は limit [-1,1] / effort 50 / velocity 10 を持つ。
    assert jl["left_knee"] == {"position": [-1.0, 1.0], "velocity": 10.0, "torque": 50.0}
    assert jl["left_shoulder"]["position"] == [-1.0, 1.0]
    # 実 limit を取り込んだので placeholder ±3.14 ではない。
    assert jl["left_knee"]["position"] != [-3.14, 3.14]
    # actuator の無い合成関節（toe / 頭）は placeholder にフォールバック。
    assert jl["left_foot"]["position"] == [-3.14, 3.14]
    assert jl["head"]["position"] == [-3.14, 3.14]


def test_canonical_mass_distribution_symmetric_and_normalized(tmp_path: Path) -> None:
    """URDF inertial → canonical 質量分布が Σ=1・左右対称で、脚 link 質量が脚 bone に乗る。"""
    from robotdance_unitree.urdf_import import canonical_mass_distribution

    def link(name, parent, xyz, mass):
        j = (f'<joint name="{name}_j" type="revolute"><parent link="{parent}"/>'
             f'<child link="{name}"/><origin xyz="{xyz}" rpy="0 0 0"/>'
             f'<limit lower="-1" upper="1" effort="1" velocity="1"/></joint>') if parent else ""
        return (f'<link name="{name}"><inertial><mass value="{mass}"/>'
                f'<origin xyz="0 0 0"/></inertial></link>' + j)

    parts = ['<robot name="g1_fixture">', link("pelvis", None, None, 3.0)]
    for side, sgn in (("left", 1), ("right", -1)):
        y = 0.06 * sgn
        parts += [
            link(f"{side}_hip_pitch_link", "pelvis", f"0 {y} -0.10", 4.0),
            link(f"{side}_knee_link", f"{side}_hip_pitch_link", "0 0 -0.30", 2.0),
            link(f"{side}_ankle_pitch_link", f"{side}_knee_link", "0 0 -0.30", 0.6),
            link(f"{side}_shoulder_pitch_link", "pelvis", f"0 {0.1 * sgn} 0.40", 1.0),
            link(f"{side}_elbow_link", f"{side}_shoulder_pitch_link", "0 0 -0.20", 0.5),
            link(f"{side}_wrist_roll_rubber_hand", f"{side}_elbow_link", "0.10 0 -0.05", 0.3),
        ]
    parts.append("</robot>")
    urdf = tmp_path / "mass.urdf"
    urdf.write_text("\n".join(parts), encoding="utf-8")

    frac, total = canonical_mass_distribution(urdf)
    assert abs(sum(frac.values()) - 1.0) < 1e-9
    assert abs(total - 19.8) < 1e-6   # 3 + 2*(4+2+0.6+1+0.5+0.3)
    # 左右対称。
    for seg in ("hip", "knee", "ankle", "shoulder", "elbow", "wrist"):
        assert abs(frac[f"left_{seg}"] - frac[f"right_{seg}"]) < 1e-9
    # 脚（hip+knee+ankle+foot）が腕（shoulder+elbow+wrist）より重い（脚 link 質量が大きい）。
    legs = sum(frac[k] for k in frac if any(s in k for s in ("hip", "knee", "ankle", "foot")))
    arms = sum(frac[k] for k in frac if any(s in k for s in ("shoulder", "elbow", "wrist")))
    assert legs > arms


def test_canonical_inertia_tensors_symmetric_psd(tmp_path: Path) -> None:
    """URDF inertial → canonical bone 慣性が左右対称・正定値・total 質量保存で返る。"""
    from robotdance_unitree.urdf_import import canonical_inertia_tensors

    def link(name, parent, xyz, mass, ixx, iyy, izz):
        j = (f'<joint name="{name}_j" type="revolute"><parent link="{parent}"/>'
             f'<child link="{name}"/><origin xyz="{xyz}" rpy="0 0 0"/>'
             f'<limit lower="-1" upper="1" effort="1" velocity="1"/></joint>') if parent else ""
        return (f'<link name="{name}"><inertial><mass value="{mass}"/><origin xyz="0 0 0"/>'
                f'<inertia ixx="{ixx}" iyy="{iyy}" izz="{izz}" ixy="0" ixz="0" iyz="0"/>'
                f'</inertial></link>' + j)

    parts = ['<robot name="g1_fixture">', link("pelvis", None, None, 3.0, 0.01, 0.01, 0.01)]
    for side, sgn in (("left", 1), ("right", -1)):
        y = 0.06 * sgn
        parts += [
            link(f"{side}_hip_pitch_link", "pelvis", f"0 {y} -0.10", 2.0, 0.02, 0.02, 0.005),
            link(f"{side}_knee_link", f"{side}_hip_pitch_link", "0 0 -0.30", 1.5, 0.03, 0.03, 0.004),
            link(f"{side}_ankle_pitch_link", f"{side}_knee_link", "0 0 -0.30", 0.6, 0.01, 0.01, 0.002),
        ]
    parts.append("</robot>")
    urdf = tmp_path / "inertia.urdf"
    urdf.write_text("\n".join(parts), encoding="utf-8")

    leg_map = {"pelvis": "pelvis",
               "left_hip": "left_hip_pitch_link", "right_hip": "right_hip_pitch_link",
               "left_knee": "left_knee_link", "right_knee": "right_knee_link",
               "left_ankle": "left_ankle_pitch_link", "right_ankle": "right_ankle_pitch_link"}
    it = canonical_inertia_tensors(urdf, link_map=leg_map)
    assert "pelvis" in it
    total = sum(e["mass"] for e in it.values())
    assert abs(total - (3.0 + 2 * (2.0 + 1.5 + 0.6))) < 1e-3
    for name, e in it.items():
        f = e["fullinertia"]
        mat = np.array([[f[0], f[3], f[4]], [f[3], f[1], f[5]], [f[4], f[5], f[2]]])
        assert float(np.linalg.eigvalsh(mat).min()) > 0, f"{name} 非PD"
    # 左右対称（主慣性が一致）。
    for seg in ("knee", "ankle"):
        lf = np.array(it[f"left_{seg}"]["fullinertia"])
        rf = np.array(it[f"right_{seg}"]["fullinertia"])
        assert np.allclose(lf[:3], rf[:3], atol=1e-6)  # 対角は対称


def test_canonical_envelope_aggregates_multi_dof_limb() -> None:
    """1 canonical 関節に複数 DOF がある場合、位置は最広・速度/トルクは最小に集約される。"""
    from robotdance_unitree.urdf_import import canonical_joint_limits

    actuated = {
        "left_hip_pitch_joint": {"position": [-2.5, 2.9], "velocity": 32.0, "torque": 88.0},
        "left_hip_roll_joint": {"position": [-0.5, 3.0], "velocity": 30.0, "torque": 90.0},
        "left_hip_yaw_joint": {"position": [-2.8, 2.8], "velocity": 32.0, "torque": 88.0},
    }
    out = canonical_joint_limits(actuated)["left_hip"]
    assert out["position"] == [-2.8, 3.0]   # [min lower, max upper]
    assert out["velocity"] == 30.0          # 最も厳しい（min）
    assert out["torque"] == 88.0            # 最も厳しい（min）
