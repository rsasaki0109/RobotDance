"""HumanoidBattle arena（MuJoCo 2 体ボクシング）の検証。

描画（EGL）は CI で動かないため render=False でスコアリングのみ検証する。boxing 合成は描画不要。
"""

from __future__ import annotations

import numpy as np
import pytest

from robotdance_core.skeleton import index_of
from robotdance_sim.fight_moves import generate_boxing, generate_hook, generate_kick


def test_generate_boxing_is_valid_and_punches_forward() -> None:
    mir = generate_boxing(duration=3.0)
    k = mir.keypoints_3d_array()
    assert k.shape[0] == round(30 * 3.0)
    assert k.shape[1:] == (19, 3)
    # 拳（手首）はガード時より前方(+x)へ伸びる瞬間がある（パンチ）。
    lw = k[:, index_of("left_wrist"), 0]
    rw = k[:, index_of("right_wrist"), 0]
    assert lw.max() > lw.min() + 0.2  # 前後に大きく動く
    assert rw.max() > rw.min() + 0.2
    # 脚はほぼ据え置き（立位）。
    la = k[:, index_of("left_ankle"), :]
    assert np.ptp(la, axis=0).max() < 0.05


def test_run_fight_scores_and_picks_winner() -> None:
    pytest.importorskip("mujoco")
    from robotdance_sim.arena import run_fight
    from robotdance_unitree import get_morphology

    res = run_fight(get_morphology("unitree_g1"), get_morphology("unitree_h1"),
                    name_a="unitree_g1", name_b="unitree_h1", duration=4.0, render=False)
    assert res.p1_hits >= 0 and res.p2_hits >= 0
    assert res.winner in ("unitree_g1", "unitree_h1", "DRAW")
    if res.p1_hits != res.p2_hits:
        hi = "unitree_g1" if res.p1_hits > res.p2_hits else "unitree_h1"
        assert res.winner == hi
    assert not res.frames  # render=False なので空
    # 累積ヒットは単調非減少で最終値に一致。
    assert res.p1_cum and res.p1_cum[-1] == res.p1_hits
    assert all(b >= a for a, b in zip(res.p1_cum, res.p1_cum[1:]))


def test_mirror_match_is_draw() -> None:
    pytest.importorskip("mujoco")
    from robotdance_sim.arena import run_fight
    from robotdance_unitree import get_morphology

    res = run_fight(get_morphology("unitree_g1"), get_morphology("unitree_g1"),
                    name_a="unitree_g1", name_b="unitree_g1", duration=4.0, render=False)
    assert res.p1_hits == res.p2_hits and res.winner == "DRAW"  # 同体・同モーション → 引き分け


def test_run_fight_assisted_p1_scores() -> None:
    pytest.importorskip("mujoco")
    from robotdance_sim.arena import run_fight
    from robotdance_unitree import get_morphology

    res = run_fight(
        get_morphology("unitree_g1"), get_morphology("unitree_h1"),
        name_a="unitree_g1", name_b="unitree_h1", duration=3.0, render=False,
        style="boxing", depth_refine=True, assisted="p1",
    )
    assert res.assisted_corner == "p1"
    assert res.assisted_survival is not None
    assert res.assisted_survival > 0.5
    assert res.winner in ("unitree_g1", "unitree_h1", "DRAW")
    assert res.p1_hits >= 0 and res.p2_hits >= 0


def test_run_fight_assisted_kick_with_depth_refine() -> None:
    pytest.importorskip("mujoco")
    from robotdance_sim.arena import run_fight
    from robotdance_unitree import get_morphology

    res = run_fight(
        get_morphology("unitree_g1"), get_morphology("unitree_h1"),
        name_a="unitree_g1", name_b="unitree_h1", duration=3.0, render=False,
        style="kick", depth_refine=True, assisted="p1",
    )
    assert res.assisted_survival == 1.0


def test_run_fight_assisted_rl_karate() -> None:
    pytest.importorskip("torch")
    pytest.importorskip("mujoco")
    from robotdance_sim.arena import run_fight
    from robotdance_unitree import get_morphology

    res = run_fight(
        get_morphology("unitree_g1"), get_morphology("unitree_h1"),
        name_a="unitree_g1", name_b="unitree_h1", duration=3.0, render=False,
        style="karate", assisted="p1", assisted_mode="rl", rl_iterations=8,
    )
    assert res.assisted_corner == "p1"
    assert res.assisted_mode == "rl"
    assert res.assisted_survival is not None
    assert res.assisted_survival > 0.3
