"""Viewer 強化（caption overlay / search montage / title, §6）の検証。

`_mir_caption` は依存なしで CI 検証。描画は matplotlib/imageio を importorskip。
"""

from __future__ import annotations

from pathlib import Path

import pytest

from robotdance_core.synthetic import generate_backflip, generate_dance
from robotdance_viewer.skeleton_view import _mir_caption


def test_mir_caption_extraction() -> None:
    mir = generate_dance(duration=0.5)
    mir.semantics = {"action_label": "a person dances"}
    assert _mir_caption(mir) == "a person dances"
    # "unknown" / 空は None（caption を出さない）。
    mir.semantics = {"action_label": "unknown"}
    assert _mir_caption(mir) is None
    mir.semantics = {}
    assert _mir_caption(mir) is None
    mir.semantics = None
    assert _mir_caption(mir) is None


def test_render_gif_with_caption(tmp_path: Path) -> None:
    pytest.importorskip("matplotlib")
    pytest.importorskip("imageio")
    from robotdance_viewer.skeleton_view import render_gif

    mir = generate_dance(duration=0.5, fps=20.0)
    mir.semantics = {"action_label": "energetic dance"}
    out = render_gif(mir, tmp_path / "cap.gif", stride=2)  # caption は自動
    assert out.exists() and out.stat().st_size > 0


def test_render_search_montage(tmp_path: Path) -> None:
    pytest.importorskip("matplotlib")
    pytest.importorskip("imageio")
    from robotdance_viewer.skeleton_view import render_search_montage

    results = [
        (generate_dance(duration=0.5, beats_per_second=1.6).keypoints_3d_array(), "dance_fast", 0.82),
        (generate_dance(duration=0.5, beats_per_second=0.7).keypoints_3d_array(), "dance_slow", 0.61),
        (generate_backflip(duration=0.6).keypoints_3d_array(), "backflip", 0.10),
    ]
    out = render_search_montage("fast dancing", results, tmp_path / "search.gif", stride=2)
    assert out.exists() and out.stat().st_size > 0


def test_render_side_by_side_with_title(tmp_path: Path) -> None:
    pytest.importorskip("matplotlib")
    pytest.importorskip("imageio")
    from robotdance_viewer.skeleton_view import render_side_by_side

    kp = generate_dance(duration=0.5).keypoints_3d_array()
    out = render_side_by_side([(kp, "panel", "#1f77b4")], tmp_path / "t.gif",
                              fps=20.0, stride=2, title="🔎 query")
    assert out.exists() and out.stat().st_size > 0


def test_render_balance_plot(tmp_path: Path) -> None:
    """balance ビューア: certificate trace を ZMP×支持多角形の上面図 PNG に描く。"""
    pytest.importorskip("matplotlib")
    pytest.importorskip("mujoco")
    from robotdance_retarget.kinematic import retarget
    from robotdance_sim.mujoco_backend import simulate_certificate
    from robotdance_unitree import get_morphology
    from robotdance_viewer.balance_view import render_balance_plot

    morph = get_morphology("unitree_g1")
    from robotdance_core.synthetic import generate_march

    cert = simulate_certificate(retarget(generate_march(), morph), morph, return_trace=True)
    out = render_balance_plot(cert["trace"], tmp_path / "balance.png", title="g1 march")
    assert out.exists() and out.stat().st_size > 0
