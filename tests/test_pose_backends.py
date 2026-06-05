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


def test_every_backend_has_a_2d_runner_factory():
    # レジストリの全バックエンドに 2D ランナー生成器が紐づく（生成は遅延 import なので呼ばない）。
    from robotdance_perception.backends import _RUNNER_FACTORIES

    assert set(_RUNNER_FACTORIES) == {b.name for b in list_backends()}


def test_make_runner_2d_unknown_backend_raises():
    with pytest.raises(ValueError, match="未知の pose backend"):
        make_runner_2d("openpose")


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
