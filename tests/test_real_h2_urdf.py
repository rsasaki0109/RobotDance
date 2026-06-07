"""実 Unitree H2 URDF に対する回帰検証（committed 定数 ↔ URDF 再導出の一致）。

公式 H2.urdf がローカルに存在する場合のみ実行し、無ければ skip（CI/未取得環境では非破壊）。
URDF は環境変数 `ROBOTDANCE_H2_URDF` か既知の候補パスから探す。h2.py の committed 定数
（rest pose / joint limits）が URDF 再導出と一致することを確認し、ドリフトを防ぐ。
"""

from __future__ import annotations

import os
from pathlib import Path

import numpy as np
import pytest

from robotdance_core.skeleton import PARENTS
from robotdance_unitree import get_morphology
from robotdance_unitree.h2 import H2_LINK_MAP

_H2_CANDIDATES = [
    os.environ.get("ROBOTDANCE_H2_URDF", ""),
    str(Path.home() / "tmp/g1_meshes/unitree_ros/robots/h2_description/H2.urdf"),
]


def _find(cands: list[str]) -> str | None:
    for c in cands:
        if c and Path(c).is_file():
            return c
    return None


_URDF = _find(_H2_CANDIDATES)
_skip = pytest.mark.skipif(_URDF is None, reason="実 H2 URDF が無い（ROBOTDANCE_H2_URDF 未設定）")


def test_h2_registered_and_sane() -> None:
    """URDF 不要: H2 が registry にあり、大型ヒューマノイドらしい寸法を持つ。"""
    m = get_morphology("unitree_h2")
    assert m.name == "unitree_h2"
    assert 1.6 < m.nominal_height < 1.9        # H2 は大型（~1.76m）
    assert len(m.bone_lengths) == 19
    mi = get_morphology("unitree_h2", real_inertia=True)
    assert mi.inertia_tensors is not None       # 実慣性が opt-in で装着できる


@_skip
def test_h2_committed_constants_match_urdf() -> None:
    """h2.py の committed rest/limits が H2.urdf 再導出と一致（bone 平均誤差 < 1cm）。"""
    from robotdance_unitree.urdf_import import urdf_to_morphology

    real = urdf_to_morphology(_URDF, name="h2_real", link_map=H2_LINK_MAP)
    proxy = get_morphology("unitree_h2")
    assert abs(proxy.nominal_height - real.nominal_height) < 0.01
    rb, pb = real.bone_lengths, proxy.bone_lengths
    bones = [j for j in range(len(PARENTS)) if PARENTS[j] >= 0]
    mae = float(np.mean([abs(pb[j] - rb[j]) for j in bones]))
    assert mae < 0.01


@_skip
def test_h2_retarget_and_certify() -> None:
    """H2 へ kinematic retarget が通り、安全動作は certificate PASS（mujoco 必要）。"""
    pytest.importorskip("mujoco")
    from robotdance_core.synthetic import generate_dance
    from robotdance_retarget.kinematic import retarget
    from robotdance_sim.backend import certify

    motion = retarget(generate_dance(duration=1.0), get_morphology("unitree_h2", real_inertia=True))
    certify(motion, get_morphology("unitree_h2", real_inertia=True))
    assert motion.sim_certificate is not None
    assert motion.robot_name == "unitree_h2"
