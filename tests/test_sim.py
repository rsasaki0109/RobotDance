"""MuJoCo 物理検証（sim_certificate）の縦スライス。

mujoco 未インストール環境では skip する。
"""

from __future__ import annotations

import pytest

pytest.importorskip("mujoco")

from robotdance_core.synthetic import (  # noqa: E402
    generate_backflip,
    generate_dance,
    generate_overbend,
)
from robotdance_retarget.kinematic import retarget  # noqa: E402
from robotdance_sim.mjcf import FOOT_BOX_HALF_WIDTH, build_mjcf  # noqa: E402
from robotdance_sim.mujoco_backend import (  # noqa: E402
    _foot_footprint,
    _max_bone_angular_speed,
    _zmp_in_support,
    certify,
    simulate_certificate,
)
from robotdance_unitree import get_morphology  # noqa: E402

import numpy as np  # noqa: E402


@pytest.mark.parametrize("robot", ["unitree_g1", "unitree_h1"])
def test_mass_distribution_matches_real_urdf_leg_heavy(robot: str) -> None:
    """質量分布が実 URDF inertial 相当: 実機は脚が最重量（股/膝アクチュエータ）。

    旧実装は Winter 人体計測比（胴体~58%/脚~32%）を全ロボットに適用していたが、これは
    **実ロボットには誤り**だった—実 G1/H1 は脚が ~53-58% で胴体より重い（股・膝に重い
    アクチュエータ）。embodiment.mass_distribution（実 URDF <inertial> 由来）を sim に流し、
    生成 MJCF の質量分布が実機の脚優位な分布になることを担保する。
    """
    import mujoco

    from robotdance_core.skeleton import JOINT_NAMES

    morph = get_morphology(robot)
    model = mujoco.MjModel.from_xml_string(build_mjcf(morph, total_mass=morph.sim_defaults.total_mass))
    groups = {
        "trunk": ("pelvis", "spine", "chest", "neck", "head"),
        "arms": ("shoulder", "elbow", "wrist"),
        "legs": ("hip", "knee", "ankle", "foot"),
    }
    g = {"trunk": float(model.body_mass[model.body("root").id]), "arms": 0.0, "legs": 0.0}
    for j, name in enumerate(JOINT_NAMES):
        try:
            mm = float(model.body_mass[model.body(f"body_{j}").id])
        except Exception:
            continue
        for grp, keys in groups.items():
            if any(k in name for k in keys):
                g[grp] += mm
                break
    tot = sum(g.values())
    trunk, arms, legs = g["trunk"] / tot, g["arms"] / tot, g["legs"] / tot
    # 実機は脚が最重量（胴体・腕より重い）。人体プライアの「胴体最重量」とは逆。
    assert legs > 0.45, f"{robot}: 脚が軽すぎる({legs:.0%})—実 URDF では最重量(~53-58%)"
    assert legs > trunk, f"{robot}: 脚({legs:.0%})が胴体({trunk:.0%})より軽い—実機分布と不一致"
    assert arms < 0.25, f"{robot}: 腕が重すぎる({arms:.0%})"


def test_zmp_support_uses_polygon_not_per_foot_circles() -> None:
    """支持判定は足点の凸包（支持多角形）で行う。広い脚幅でも中心 ZMP を支持と認める。

    旧実装は各足点を半径 margin の円で覆う近似で、脚幅が広い機種（H1: 足点 y=±0.26）では
    両足の中間（バランスの取れた ZMP の定位置）がどの足点からも margin 超になり、
    正しく立っているのに転倒判定していた。凸包内なら距離0で支持とするのが正しい。
    """
    # H1 相当の広い両足支持多角形（ankle + toe, y=±0.26）。
    feet = np.array([[0.06, 0.26], [0.06, -0.26], [0.16, 0.26], [0.16, -0.26]])
    centered = np.array([0.09, 0.0])  # 両足の中間 = バランス点
    assert _zmp_in_support(centered, feet, margin=0.05), "広い脚幅で中心 ZMP が支持外と誤判定"
    # 多角形から十分外（margin 超）は支持外。
    far_out = np.array([0.09, 0.6])
    assert not _zmp_in_support(far_out, feet, margin=0.12), "明らかに支持外の ZMP を支持と誤判定"
    # 単一足（線分）でも近ければ支持、遠ければ支持外。
    one_foot = np.array([[0.06, 0.26], [0.16, 0.26]])
    assert _zmp_in_support(np.array([0.11, 0.30]), one_foot, margin=0.1)
    assert not _zmp_in_support(np.array([0.11, 0.60]), one_foot, margin=0.1)


def test_foot_footprint_has_real_width_for_single_support() -> None:
    """接地足は幅ゼロの線分でなく、実フットプリント（足 box 幅）の矩形として支持に寄与する。

    旧来は ankle/toe の 2 点だけで横幅ゼロ → 片足支持で横バランスが評価できず margin 頼みだった。
    footprint は ankle→toe に直交方向へ box 半幅だけ広がり、片足でも横方向の支持を持つ。
    """
    ankle = np.array([0.0, 0.10])
    toe = np.array([0.12, 0.10])  # 前向き（+x）の足
    corners = np.array(_foot_footprint(ankle, toe))
    # 4 隅で、横（y）方向に ±box半幅の広がりを持つ。
    assert len(corners) == 4
    assert corners[:, 1].max() == pytest.approx(0.10 + FOOT_BOX_HALF_WIDTH)
    assert corners[:, 1].min() == pytest.approx(0.10 - FOOT_BOX_HALF_WIDTH)
    # 片足支持: 足中心からやや横にずれた ZMP も、footprint 幅の内側なら支持（margin 0 でも）。
    lateral = np.array([0.06, 0.10 + FOOT_BOX_HALF_WIDTH * 0.5])
    assert _zmp_in_support(lateral, corners, margin=0.0), "片足 footprint の横幅内が支持外と誤判定"


def test_build_mjcf_uses_real_inertia_tensors_when_present() -> None:
    """inertia_tensors を持つ morphology は capsule 近似でなく実テンソルを MJCF に使う（frame 正）。

    opt-in capability: 既定 morphology は capsule（controller baseline 安定のため）だが、
    inertia_tensors を渡すと explicit <inertial> で実機慣性を使う。MuJoCo の body_inertia
    （principal）が埋め込みテンソルの固有値に一致し、総質量も保存することを担保する。
    """
    import dataclasses

    import mujoco

    from robotdance_core.skeleton import JOINT_NAMES
    from robotdance_unitree.g1 import G1_INERTIA_TENSORS

    base = get_morphology("unitree_g1")
    morph = dataclasses.replace(base, inertia_tensors=G1_INERTIA_TENSORS)
    tm = base.sim_defaults.total_mass
    model = mujoco.MjModel.from_xml_string(build_mjcf(morph, total_mass=tm, ground=False))
    # 総質量は保存。
    assert model.body_mass.sum() == pytest.approx(tm, abs=1e-3)
    # 代表 bone の MuJoCo 主慣性が埋め込みテンソルの固有値に一致（フレーム変換が正しい）。
    for name in ("left_knee", "chest", "left_ankle"):
        j = JOINT_NAMES.index(name)
        bid = model.body(f"body_{j}").id
        f = G1_INERTIA_TENSORS[name]["fullinertia"]
        mat = np.array([[f[0], f[3], f[4]], [f[3], f[1], f[5]], [f[4], f[5], f[2]]])
        scale = tm / 34.13  # 埋め込みは実 URDF 総質量基準
        src_eig = np.sort(np.linalg.eigvalsh(mat)) * scale
        mj_eig = np.sort(model.body_inertia[bid])
        assert np.allclose(mj_eig, src_eig, atol=2e-3), f"{name}: {mj_eig} vs {src_eig}"
    # capsule 既定（inertia_tensors 無し）とは主慣性が異なる（実テンソルが効いている）。
    cap = mujoco.MjModel.from_xml_string(build_mjcf(base, total_mass=tm, ground=False))
    jc = JOINT_NAMES.index("chest")
    assert not np.allclose(model.body_inertia[model.body(f"body_{jc}").id],
                           cap.body_inertia[cap.body(f"body_{jc}").id], atol=1e-3)


@pytest.mark.parametrize("robot", ["unitree_g1", "unitree_h1"])
def test_mjcf_total_mass_is_conserved(robot: str) -> None:
    """生成 MJCF の総質量が宣言 total_mass（embodiment 既定＝実 URDF 総質量）に一致する。

    旧実装は pelvis ハブ(3kg)+足 box(0.6kg) を total_mass の上乗せにしており、
    宣言35kg の G1 を実38.6kg(+10.3%) で sim していた。PD ゲインや逆動力学トルクは
    実質量に依存するため、宣言と実体がズレると「35kg 用に調整したつもりが 38.6kg を制御」
    という隠れ取り違えが起きる。固定質量を bone 配分予算から差し引いて質量を保存する。
    total_mass は各 embodiment の sim_defaults（実 URDF 総質量: G1 34.13 / H1 59.34kg）。
    """
    import mujoco

    morph = get_morphology(robot)
    total_mass = morph.sim_defaults.total_mass
    model = mujoco.MjModel.from_xml_string(build_mjcf(morph, total_mass=total_mass))
    assert model.body_mass.sum() == pytest.approx(total_mass, abs=1e-3), (
        f"{robot}: 宣言 {total_mass}kg と MJCF 実質量 {model.body_mass.sum():.3f}kg が不一致"
    )


def test_certify_uses_per_joint_torque_limits() -> None:
    """certify は実 per-joint actuator トルク上限で負荷率を判定する（単一スカラー流用ではない）。

    旧実装は torque_ratio = max重力トルク / scalar で、強い関節（膝~139）と弱い関節（足首~35）を
    区別できなかった。per-joint 化後は各関節の必要トルク÷その関節上限の最大率を取る。H1 は
    per_joint_limits を持つので既定（torque_limit 未指定）は per-joint で判定し、scalar を明示すると
    全関節へその値を強制する（旧挙動・対比用）。
    """
    import dataclasses

    morph = get_morphology("unitree_h1")
    motion = retarget(generate_dance(duration=1.0), morph)
    default = simulate_certificate(motion, morph)["metrics"]["torque_ratio"]
    # scalar を明示すると per-joint ではなくその値で判定 → 既定（per-joint）とは異なる。
    scalar80 = simulate_certificate(motion, morph, torque_limit=80.0)["metrics"]["torque_ratio"]
    scalar300 = simulate_certificate(motion, morph, torque_limit=300.0)["metrics"]["torque_ratio"]
    assert default != pytest.approx(scalar80), "既定が per-joint ではなく scalar を使っている"
    assert scalar80 > scalar300, "scalar 上限が小さいほど負荷率は高いはず"
    # per_joint_limits を外すと sim_defaults スカラーへフォールバック（per-joint を使っている証左）。
    stripped = dataclasses.replace(morph, per_joint_limits=None)
    fallback = simulate_certificate(motion, stripped)["metrics"]["torque_ratio"]
    assert fallback != pytest.approx(default), "per_joint_limits の有無で torque_ratio が変わらない"


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


def test_bone_angular_speed_is_twist_free() -> None:
    """全 bone を z=0 平面に置き z 軸回りに一定 ω で剛体回転 → 厳密に ω を返す（twist 非依存）。"""
    from scipy.spatial.transform import Rotation as Rot

    dt = 1.0 / 30.0
    n, omega = 10, 2.0
    # 各 joint を z=0 平面の相異なる点に置く（全 bone が非退化・水平 → z 回転で厳密に ω）。
    base = np.array([[0.1 * (j + 1), 0.02 * j, 0.0] for j in range(19)])
    kps = np.stack([Rot.from_euler("z", omega * f * dt).apply(base) for f in range(n)])
    assert _max_bone_angular_speed(kps, dt) == pytest.approx(omega, abs=1e-6)


def test_overbend_angular_speed_has_no_spurious_spike() -> None:
    """過屈曲モーションの角速度は twist-free 化で偽スパイク（~79 rad/s）が消え実速度になる。"""
    morph = get_morphology("unitree_g1")
    motion = retarget(generate_overbend(), morph)
    cert = simulate_certificate(motion, morph)
    assert cert["metrics"]["max_joint_ang_speed_rad_s"] < 30.0  # 偽スパイクなし
    # 動的サブ指標は全てクリーン（足は接地・直立で転倒/滞空なし）。
    assert cert["metrics"]["airborne_ratio"] == 0.0
    assert cert["metrics"]["balance_violation_ratio"] == 0.0


def test_certificate_rejects_rom_violation_and_clamp_remedies() -> None:
    """動的に安定でも実機 ROM 超過なら統合 verdict は REJECT、clamp_flexion で PASS になる。

    sim は動的 feasibility の権威だが、可動域を超える姿勢は指令不能なので運動学的 feasibility
    （joint_flexion）も統合する。overbend G1 は動的にはクリーンだが肘が ROM 超過 → REJECT。
    """
    morph = get_morphology("unitree_g1")
    # 補正なし: 動的指標はクリーンだが ROM 超過で REJECT。
    raw = retarget(generate_overbend(), morph)
    cert = simulate_certificate(raw, morph)
    assert cert["metrics"]["airborne_ratio"] == 0.0
    assert cert["metrics"]["balance_violation_ratio"] == 0.0
    assert cert["metrics"]["max_joint_ang_speed_rad_s"] < 30.0
    assert cert["verdict"] == "REJECT"
    assert cert["metrics"]["joint_flexion_violation_ratio"] > 0.0
    assert any("可動域" in r for r in cert["reasons"])
    # clamp_flexion で可動域内へ収めると PASS。
    fixed = retarget(generate_overbend(), morph, clamp_flexion=True)
    cert2 = simulate_certificate(fixed, morph)
    assert cert2["verdict"] == "PASS"
    assert cert2["metrics"]["joint_flexion_violation_ratio"] == 0.0


def test_certificate_no_rom_metric_without_per_joint_limits() -> None:
    """per_joint_limits が無い morphology では ROM 統合は無効（joint_flexion 指標も出ない）。"""
    import dataclasses

    morph = dataclasses.replace(get_morphology("unitree_g1"), per_joint_limits=None)
    cert = simulate_certificate(retarget(generate_dance(duration=1.0), morph), morph)
    assert "joint_flexion_violation_ratio" not in cert["metrics"]


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
