"""実 Unitree G1 URDF に対する回帰検証（real-data validation）。

公式 g1_23dof URDF が**ローカルに存在する場合のみ**実行し、無ければ skip する（CI / 未取得環境では
スキップ＝非破壊）。URDF は環境変数 `ROBOTDANCE_G1_URDF` か既知の候補パスから探す。

検証内容:
  - G1 簡略 morphology（`get_morphology("unitree_g1")`）が**実 URDF の実寸**と一致する
    （v0.26 で手書きプロキシ→実寸由来に更新。旧来は nominal 1.12m・bone 相対誤差 ~26% で乖離）。
  - actuator-space IK（`actuator_retarget`）が実 URDF で収束し、関節 limit 違反が無い（torch 必要）。
"""

from __future__ import annotations

import os
from pathlib import Path

import numpy as np
import pytest

from robotdance_core.skeleton import PARENTS
from robotdance_unitree import get_morphology

_G1_CANDIDATES = [
    os.environ.get("ROBOTDANCE_G1_URDF", ""),
    str(Path.home() / "tmp/g1_meshes/unitree_ros/robots/g1_description/g1_23dof.urdf"),
]
_H1_CANDIDATES = [
    os.environ.get("ROBOTDANCE_H1_URDF", ""),
    str(Path.home() / "tmp/g1_meshes/unitree_ros/robots/h1_description/urdf/h1.urdf"),
]


def _find(cands: list[str]) -> str | None:
    for c in cands:
        if c and Path(c).is_file():
            return c
    return None


_URDF = _find(_G1_CANDIDATES)
_H1_URDF = _find(_H1_CANDIDATES)
_skip = pytest.mark.skipif(_URDF is None, reason="実 G1 URDF が無い（ROBOTDANCE_G1_URDF 未設定）")
_skip_h1 = pytest.mark.skipif(_H1_URDF is None, reason="実 H1 URDF が無い（ROBOTDANCE_H1_URDF 未設定）")


@_skip
def test_proxy_matches_real_urdf_dimensions() -> None:
    """簡略 G1 morphology が実 URDF の実寸に一致する（nominal/bone とも誤差小）。"""
    from robotdance_unitree.urdf_import import urdf_to_morphology

    real = urdf_to_morphology(_URDF, name="g1_real")
    proxy = get_morphology("unitree_g1")
    # 全高は 1cm 以内。
    assert abs(proxy.nominal_height - real.nominal_height) < 0.01
    # bone 長の平均絶対誤差は 1cm 未満（旧プロキシは 3.2cm / 26%）。
    rb, pb = real.bone_lengths, proxy.bone_lengths
    bones = [j for j in range(len(PARENTS)) if PARENTS[j] >= 0]
    mae = float(np.mean([abs(pb[j] - rb[j]) for j in bones]))
    assert mae < 0.01


@_skip
def test_actuator_ik_on_real_urdf_converges() -> None:
    """実 G1 URDF で actuator-space IK が収束し joint limit 違反が無い。"""
    pytest.importorskip("torch")
    from robotdance_core.synthetic import generate_dance
    from robotdance_retarget.actuator_ik import actuator_retarget

    motion = actuator_retarget(generate_dance(duration=1.0, arm_amp=0.6, sway_amp=0.08),
                               _URDF, steps=150)
    mt = motion.retarget_metrics
    assert mt["actuated_joints"] == 23
    assert mt["ik_mean_pos_error_m"] < 0.15   # 限られた DOF での追従誤差の上限
    assert mt["joint_limit_violation_ratio_preclamp"] == 0.0
    ang = np.asarray(motion.joint_rotations["angles_rad"])
    assert ang.shape[1] == 23


@_skip
def test_g1_embedded_joint_limits_match_real_urdf() -> None:
    """g1.py に埋め込んだ G1_JOINT_LIMITS が実 g1_23dof URDF の canonical envelope と一致する。

    数値定数は手で埋め込んでいるため、実 URDF からの算出値と完全一致することを実機で検証し、
    drift（実機更新や転記ミス）を捕捉する。
    """
    from robotdance_unitree.g1 import G1_JOINT_LIMITS
    from robotdance_unitree.urdf_import import canonical_joint_limits, parse_actuated_limits

    derived = canonical_joint_limits(parse_actuated_limits(_URDF))
    assert derived == G1_JOINT_LIMITS
    # 実機の事実: 膝は屈曲のみ（逆屈不可）でトルクは腕より強力。
    assert G1_JOINT_LIMITS["left_knee"]["position"][0] > -0.2
    assert abs(G1_JOINT_LIMITS["left_knee"]["position"][1] - 2.8798) < 1e-4
    assert G1_JOINT_LIMITS["left_knee"]["torque"] == 139.0


@_skip
def test_g1_embedded_mass_distribution_matches_real_urdf() -> None:
    """g1.py の G1_MASS_FRACTION が実 g1_23dof URDF の inertial 由来分布と一致し、脚優位である。"""
    from robotdance_unitree.g1 import G1_MASS_FRACTION
    from robotdance_unitree.urdf_import import canonical_mass_distribution

    frac, total = canonical_mass_distribution(_URDF)
    assert abs(total - 34.13) < 0.1
    for name, v in frac.items():
        assert abs(v - G1_MASS_FRACTION[name]) < 1e-3, name
    legs = sum(frac[k] for k in frac if any(s in k for s in ("hip", "knee", "ankle", "foot")))
    trunk = sum(frac[k] for k in ("pelvis", "spine", "chest", "neck", "head"))
    assert legs > 0.45 and legs > trunk   # 実機は脚が最重量


@_skip_h1
def test_h1_embedded_mass_distribution_matches_real_urdf() -> None:
    """h1.py の H1_MASS_FRACTION が実 h1.urdf 由来分布と一致する（実総質量 ~59kg）。"""
    from robotdance_unitree.h1 import H1_MASS_FRACTION
    from robotdance_unitree.urdf_import import H1_LINK_MAP, canonical_mass_distribution

    frac, total = canonical_mass_distribution(_H1_URDF, link_map=H1_LINK_MAP)
    assert total > 55.0   # H1 実 URDF は ~59kg（SimDefaults の 47 は控えめ）
    for name, v in frac.items():
        assert abs(v - H1_MASS_FRACTION[name]) < 1e-3, name


@_skip
def test_safety_guard_from_real_g1_urdf_clamps_knee_reverse_bend() -> None:
    """実 G1 URDF から構築した safety guard が膝の逆屈コマンドを実下限へクランプする。"""
    import numpy as np

    from robotdance_ros2.safety_guard import SafetyLimits, clamp_joint_trajectory
    from robotdance_unitree.urdf_import import parse_actuated_limits

    actuated = parse_actuated_limits(_URDF)
    limits = SafetyLimits.from_actuated_limits(actuated, max_joint_accel=1e9)
    names = list(actuated.keys())
    knee = next(nm for nm in names if "knee" in nm)
    knee_lo = limits.joint_position_limits[knee][0]
    assert knee_lo > -0.2   # 実機の膝は逆屈不可（generic ±π ではない）
    raw = np.zeros((30, len(names)))
    raw[:, names.index(knee)] = np.linspace(0.0, -1.5, 30)  # 逆屈コマンド
    safe, _ = clamp_joint_trajectory(raw, 1.0 / 30.0, limits, names)
    assert float(safe[:, names.index(knee)].min()) >= knee_lo - 1e-6


@_skip_h1
def test_h1_embedded_joint_limits_match_real_urdf() -> None:
    """h1.py の H1_JOINT_LIMITS が実 h1.urdf と一致し、肩 yaw は ±3.14 を超過する。"""
    from robotdance_unitree.h1 import H1_JOINT_LIMITS
    from robotdance_unitree.urdf_import import canonical_joint_limits, parse_actuated_limits

    derived = canonical_joint_limits(parse_actuated_limits(_H1_URDF))
    assert derived == H1_JOINT_LIMITS
    # H1 肩 yaw は 4.45rad に達する → placeholder ±3.14 は逆に過小だった。
    assert H1_JOINT_LIMITS["left_shoulder"]["position"][1] > 3.14
    assert H1_JOINT_LIMITS["left_knee"]["torque"] == 300.0
    # H1 は wrist actuator が無い（腕が肘止まり）→ 埋め込みにも wrist は無い。
    assert "left_wrist" not in H1_JOINT_LIMITS


@_skip_h1
def test_h1_proxy_matches_real_urdf_dimensions() -> None:
    """簡略 H1 morphology が実 h1.urdf の実寸に一致する（旧プロキシは bone 相対誤差 ~33%）。"""
    from robotdance_unitree.urdf_import import H1_LINK_MAP, urdf_to_morphology

    real = urdf_to_morphology(_H1_URDF, name="h1_real", link_map=H1_LINK_MAP)
    proxy = get_morphology("unitree_h1")
    assert abs(proxy.nominal_height - real.nominal_height) < 0.01
    rb, pb = real.bone_lengths, proxy.bone_lengths
    bones = [j for j in range(len(PARENTS)) if PARENTS[j] >= 0]
    mae = float(np.mean([abs(pb[j] - rb[j]) for j in bones]))
    assert mae < 0.01


@_skip_h1
def test_actuator_ik_on_real_h1_converges() -> None:
    """実 H1 URDF で actuator-space IK が H1_LINK_MAP 指定で収束する（link_map 一般化）。"""
    pytest.importorskip("torch")
    from robotdance_core.synthetic import generate_dance
    from robotdance_retarget.actuator_ik import actuator_retarget
    from robotdance_unitree.urdf_import import H1_LINK_MAP as _HLM

    motion = actuator_retarget(generate_dance(duration=1.0, arm_amp=0.6, sway_amp=0.08),
                               _H1_URDF, steps=120, link_map=_HLM, robot_name="unitree_h1")
    assert motion.robot_name == "unitree_h1"
    mt = motion.retarget_metrics
    assert mt["ik_mean_pos_error_m"] < 0.2
    # H1 は pre-clamp で関節 limit にごく稀（<5%）に触れる（clamp 後は安全）。G1 は 0。
    assert mt["joint_limit_violation_ratio_preclamp"] < 0.05
