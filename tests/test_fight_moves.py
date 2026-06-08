"""合成 fight 技（hook / kick / dodge）と体格差バランス。"""

from __future__ import annotations

import pytest

from robotdance_core.skeleton import index_of
from robotdance_sim.fight_moves import (
    FIGHT_STYLE_NAMES,
    effective_hit_radius,
    generate_dodge,
    generate_hook,
    generate_kick,
)


def test_fight_style_registry() -> None:
    assert {"hook", "kick", "dodge", "boxing"}.issubset(FIGHT_STYLE_NAMES)


def test_hook_moves_laterally() -> None:
    k = generate_hook(duration=3.0).keypoints_3d_array()
    lw_y = k[:, index_of("left_wrist"), 1]
    assert lw_y.max() - lw_y.min() > 0.15


def test_kick_extends_foot_forward() -> None:
    k = generate_kick(duration=3.0).keypoints_3d_array()
    fx = k[:, index_of("left_foot"), 0]
    assert fx.max() > fx.min() + 0.25


def test_dodge_reduces_forward_reach() -> None:
    from robotdance_sim.fight_moves import generate_boxing

    box = generate_boxing(duration=3.0).keypoints_3d_array()
    dodge = generate_dodge(duration=3.0).keypoints_3d_array()
    assert dodge[:, index_of("left_wrist"), 0].max() < box[:, index_of("left_wrist"), 0].max()


def test_effective_radius_tall_has_more_reach() -> None:
    r_tall = effective_hit_radius(0.20, 1.66, 1.29, body_target=False)
    r_short = effective_hit_radius(0.20, 1.29, 1.66, body_target=False)
    assert r_tall > r_short


def test_compact_body_precision_bonus() -> None:
    r_body = effective_hit_radius(0.20, 1.29, 1.66, body_target=True)
    r_head = effective_hit_radius(0.20, 1.29, 1.66, body_target=False)
    assert r_body > r_head


def test_run_fight_hook_not_mirror_draw() -> None:
    pytest.importorskip("mujoco")
    from robotdance_sim.arena import run_fight
    from robotdance_unitree import get_morphology

    res = run_fight(
        get_morphology("unitree_g1"), get_morphology("unitree_h1"),
        name_a="unitree_g1", name_b="unitree_h1", style="hook", duration=3.5, render=False,
    )
    assert res.winner in ("unitree_g1", "unitree_h1", "DRAW")
