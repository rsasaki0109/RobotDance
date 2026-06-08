"""Fight HUD — assisted survival 焼き込みの検証。"""

from __future__ import annotations

import numpy as np

from robotdance_core.cli import _fight_hud, _surv_hud_color
from robotdance_sim.arena import FightResult


def test_surv_hud_color_thresholds():
    assert _surv_hud_color(1.0)[1] > _surv_hud_color(0.7)[1]
    assert _surv_hud_color(0.3)[2] > _surv_hud_color(0.7)[2]


def test_fight_hud_assisted_taller_bar():
    frame = np.full((120, 160, 3), 40, np.uint8)
    plain = FightResult(
        "unitree_g1", "unitree_h1", 3, 2, "unitree_g1",
        frames=[frame], p1_cum=[1, 2, 3], p2_cum=[0, 1, 2],
    )
    assisted = FightResult(
        "unitree_g1", "unitree_h1", 3, 2, "unitree_g1",
        frames=[frame], p1_cum=[1, 2, 3], p2_cum=[0, 1, 2],
        assisted_corner="p1", assisted_mode="pd", assisted_survival=0.87,
    )
    h_plain = _fight_hud(plain)[0].shape[0]
    h_asst = _fight_hud(assisted)[0].shape[0]
    assert h_asst == h_plain + 16
    assert h_plain == 120 + 34
    assert h_asst == 120 + 50


def test_fight_hud_p2_assisted_taller_bar():
    frame = np.full((80, 120, 3), 30, np.uint8)
    res = FightResult(
        "unitree_g1", "unitree_h1", 1, 0, "unitree_h1",
        frames=[frame], p1_cum=[1], p2_cum=[0],
        assisted_corner="p2", assisted_mode="rl", assisted_survival=1.0,
    )
    out = _fight_hud(res)[0]
    assert out.shape[0] == 80 + 50
    assert out.shape[1] == 120
