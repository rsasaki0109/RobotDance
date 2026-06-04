"""MuJoCo 物理検証（sim_certificate）の縦スライス。

mujoco 未インストール環境では skip する。
"""

from __future__ import annotations

import pytest

pytest.importorskip("mujoco")

from robotdance_core.synthetic import generate_backflip, generate_dance  # noqa: E402
from robotdance_retarget.kinematic import retarget  # noqa: E402
from robotdance_sim.mjcf import FOOT_BOX_HALF_WIDTH, build_mjcf  # noqa: E402
from robotdance_sim.mujoco_backend import (  # noqa: E402
    _foot_footprint,
    _zmp_in_support,
    certify,
    simulate_certificate,
)
from robotdance_unitree import get_morphology  # noqa: E402

import numpy as np  # noqa: E402


@pytest.mark.parametrize("robot", ["unitree_g1", "unitree_h1"])
def test_mass_distribution_is_trunk_heavy_anthropometric(robot: str) -> None:
    """質量分布が人体計測（Winter）相当: 胴体が最重量で、腕は胴体より軽い。

    旧実装は質量 ∝ bone 長で配分しており、長い腕 bone に過大質量を与え、H1 では
    腕 32% > 胴体 19% という非物理分布だった（人もロボットも胴体が最重量部位）。
    Winter 比で配分し直し、胴体~58% / 腕~10% / 脚~32% 相当になることを担保する。
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
    trunk, arms = g["trunk"] / tot, g["arms"] / tot
    assert trunk > 0.45, f"{robot}: 胴体が軽すぎる({trunk:.0%})—人体計測では最重量(~58%)"
    assert arms < trunk, f"{robot}: 腕({arms:.0%})が胴体({trunk:.0%})より重い非物理分布"
    assert arms < 0.2, f"{robot}: 腕が重すぎる({arms:.0%})—人体計測では~10%"


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


@pytest.mark.parametrize("robot,total_mass", [("unitree_g1", 35.0), ("unitree_h1", 47.0)])
def test_mjcf_total_mass_is_conserved(robot: str, total_mass: float) -> None:
    """生成 MJCF の総質量が宣言 total_mass に一致する（宣言質量＝実質量）。

    旧実装は pelvis ハブ(3kg)+足 box(0.6kg) を total_mass の上乗せにしており、
    宣言35kg の G1 を実38.6kg(+10.3%) で sim していた。PD ゲインや逆動力学トルクは
    実質量に依存するため、宣言と実体がズレると「35kg 用に調整したつもりが 38.6kg を制御」
    という隠れ取り違えが起きる。固定質量を bone 配分予算から差し引いて質量を保存する。
    """
    import mujoco

    model = mujoco.MjModel.from_xml_string(build_mjcf(get_morphology(robot), total_mass=total_mass))
    assert model.body_mass.sum() == pytest.approx(total_mass, abs=1e-3), (
        f"{robot}: 宣言 {total_mass}kg と MJCF 実質量 {model.body_mass.sum():.3f}kg が不一致"
    )


def test_certify_uses_embodiment_torque_limit_not_g1_default() -> None:
    """certify は morphology.sim_defaults のトルク上限を使う（G1 値の固定流用ではない）。

    旧実装は simulate_certificate に torque_limit=80（G1値）をハードコードしており、
    H1（160N·m）の certify でも 80 で torque_ratio を計算していた（配線漏れ）。
    既定経路（torque_limit 未指定）と H1 値を明示した場合の torque_ratio が一致し、
    かつ G1 値を明示した場合とは異なることで、embodiment 由来であることを担保する。
    """
    morph = get_morphology("unitree_h1")
    motion = retarget(generate_dance(duration=1.0), morph)
    default = simulate_certificate(motion, morph)["metrics"]["torque_ratio"]
    h1_explicit = simulate_certificate(motion, morph, torque_limit=160.0)["metrics"]["torque_ratio"]
    g1_default = simulate_certificate(motion, morph, torque_limit=80.0)["metrics"]["torque_ratio"]
    assert default == pytest.approx(h1_explicit), "既定が H1 のトルク上限(160)を使っていない"
    assert default != pytest.approx(g1_default), "既定が G1 のトルク上限(80)に固定されたまま"


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
