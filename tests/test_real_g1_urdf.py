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

_CANDIDATES = [
    os.environ.get("ROBOTDANCE_G1_URDF", ""),
    str(Path.home() / "tmp/g1_meshes/unitree_ros/robots/g1_description/g1_23dof.urdf"),
]


def _find_g1_urdf() -> str | None:
    for c in _CANDIDATES:
        if c and Path(c).is_file():
            return c
    return None


_URDF = _find_g1_urdf()
_skip = pytest.mark.skipif(_URDF is None, reason="実 G1 URDF が無い（ROBOTDANCE_G1_URDF 未設定）")


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
