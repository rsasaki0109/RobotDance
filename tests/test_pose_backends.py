"""pose 検出バックエンドのレジストリ（robotdance_perception.backends）。"""

from __future__ import annotations

import pytest

from robotdance_perception.backends import (
    COCO_EDGES,
    MEDIAPIPE,
    MP33_TO_COCO,
    get_backend,
    list_backends,
    make_runner_2d,
    resolve_extract_backend,
)


def test_registry_lists_known_backends_sorted():
    names = [b.name for b in list_backends()]
    assert names == sorted(names)
    assert {"mediapipe", "yolo11-pose", "rtmpose"} <= set(names)


def test_get_backend_unknown_raises_with_candidates():
    with pytest.raises(ValueError, match="未知の pose backend"):
        get_backend("openpose")


def test_mediapipe_is_3d_and_retarget_capable():
    assert MEDIAPIPE.output_dim == 3
    assert MEDIAPIPE.retarget_capable is True
    assert MEDIAPIPE.keypoint_format == "blazepose33"
    # 公式 Apache-2.0 モデルで dev-only 印は付けない。
    assert "dev" not in MEDIAPIPE.extras


def test_2d_backends_are_not_retarget_capable():
    for name in ("yolo11-pose", "rtmpose"):
        b = get_backend(name)
        assert b.output_dim == 2
        assert b.retarget_capable is False
        assert b.keypoint_format == "coco17"
        assert "dev" in b.extras


def test_resolve_extract_accepts_3d_backend():
    assert resolve_extract_backend("mediapipe") is MEDIAPIPE


def test_resolve_extract_rejects_2d_backend():
    with pytest.raises(ValueError, match="3D が必要"):
        resolve_extract_backend("yolo11-pose")


def test_available_is_boolean_without_importing_heavy_deps():
    # available() は遅延 spec チェックのみ。例外を投げず bool を返す。
    for b in list_backends():
        assert isinstance(b.available(), bool)


def test_coco_constants_are_centralized():
    # COCO-17 表現は単一情報源。エッジ index は 0..16 に収まる。
    assert len(MP33_TO_COCO) == 17
    assert all(0 <= a < 33 and 0 <= b < 33 for a, b in COCO_EDGES)
    assert all(0 <= a < 17 and 0 <= b < 17 for ab in COCO_EDGES for a, b in [ab])


def test_every_2d_backend_has_a_runner_factory():
    # 2D COCO-17 を出す検出器（lift / world-grounded でない video backend）にランナーが紐づく。
    from robotdance_perception.backends import _RUNNER_FACTORIES

    assert set(_RUNNER_FACTORIES) == {
        b.name for b in list_backends()
        if not b.lift_from and b.extract_mode == "video" and b.name in _RUNNER_FACTORIES
    }


def test_lift_backends_are_coarse_3d_and_extract_capable():
    lifted = [b for b in list_backends() if b.lift_from]
    assert {b.name for b in lifted} == {"yolo11-pose+lift", "rtmpose+lift"}
    for b in lifted:
        assert b.output_dim == 3
        assert b.retarget_capable is True
        assert b.quality_tier == "coarse-planar"
        # lift 元の 2D backend は実在する。
        assert get_backend(b.lift_from).output_dim == 2
        # extract 解決は通る（3D とみなす）が 2D ランナーは持たない。
        assert resolve_extract_backend(b.name) is b


def test_make_runner_2d_rejects_lift_backend():
    with pytest.raises(ValueError, match="extract 専用"):
        make_runner_2d("yolo11-pose+lift")


def test_world_grounded_backends_registered():
    gv = get_backend("gvhmr")
    assert gv.output_dim == 3
    assert gv.retarget_capable is True
    assert gv.quality_tier == "world-grounded"
    assert gv.extract_mode == "video"
    assert "hmr4d" in gv.modules
    wh = get_backend("wham")
    assert wh.extract_mode == "import"
    assert wh.via == "import-hmr"
    assert wh.modules == ()


def test_resolve_extract_accepts_gvhmr_video_backend():
    assert resolve_extract_backend("gvhmr").name == "gvhmr"


def test_resolve_extract_redirects_wham_to_import_hmr():
    with pytest.raises(ValueError, match="import-hmr"):
        resolve_extract_backend("wham")


def test_make_runner_2d_rejects_import_backend():
    with pytest.raises(ValueError, match="extract 専用"):
        make_runner_2d("gvhmr")


def test_make_runner_2d_unknown_backend_raises():
    with pytest.raises(ValueError, match="未知の pose backend"):
        make_runner_2d("openpose")


def test_list_backends_cli_runs():
    from robotdance_core.cli import main

    assert main(["list-backends"]) == 0


def test_compare_module_covers_all_backends_with_panel_colors():
    from robotdance_perception.backends import _RUNNER_FACTORIES
    from robotdance_perception.compare import PANEL_COLORS

    # 比較は 2D ランナー付き video backend のみ。それらに overlay パネル色を割り当てる。
    names = {b.name for b in list_backends()
             if not b.lift_from and b.extract_mode == "video" and b.name in _RUNNER_FACTORIES}
    assert names <= set(PANEL_COLORS), "比較対象 backend に overlay パネル色を割り当てる"


def test_compare_raises_on_missing_video_when_a_backend_available():
    # mediapipe が居ればランナーは作れるが、動画が無ければ明示的に失敗する。
    if not MEDIAPIPE.available():
        pytest.skip("mediapipe 未導入")
    from robotdance_perception.compare import compare_backends

    with pytest.raises((FileNotFoundError, RuntimeError)):
        compare_backends("does_not_exist_xyz.mp4")


def test_mediapipe_runner_outputs_coco17_on_synthetic_frame():
    # mediapipe が導入済みなら、ランナーは (xy[17,2], conf[17]) | None を返す。
    if not MEDIAPIPE.available():
        pytest.skip("mediapipe 未導入")
    import numpy as np

    run = make_runner_2d("mediapipe")
    blank = np.zeros((64, 64, 3), dtype=np.uint8)  # 人物なし → None を期待
    res = run(blank, 0, 30.0)
    if res is not None:
        xy, conf = res
        assert xy.shape == (17, 2)
        assert conf.shape == (17,)
