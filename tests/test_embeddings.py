"""motion embedding / retrieval / Motion Map の検証。"""

from __future__ import annotations

import numpy as np
import pytest
from scipy.spatial.transform import Rotation as Rot

from robotdance_core.synthetic import generate_backflip, generate_dance
from robotdance_motion.embeddings import EMBEDDING_DIM, MotionIndex, embed


def _cos(a: np.ndarray, b: np.ndarray) -> float:
    return float(a @ b / (np.linalg.norm(a) * np.linalg.norm(b)))


def test_embedding_dim_and_determinism() -> None:
    mir = generate_dance(duration=2.0)
    e1, e2 = embed(mir), embed(mir)
    assert e1.shape == (EMBEDDING_DIM,)
    np.testing.assert_allclose(e1, e2)


def test_embedding_is_translation_scale_yaw_invariant() -> None:
    """root-relative + scale 正規化 + yaw 整列 → 平行移動/スケール/向きに不変。"""
    mir = generate_dance(duration=2.0)
    base = embed(mir)

    kps = mir.keypoints_3d_array()
    # 平行移動 + 等方スケール + z 軸回り 90° 回転。
    transformed = (Rot.from_euler("z", np.pi / 2).apply(kps.reshape(-1, 3)) * 1.5
                   + np.array([3.0, -2.0, 0.5])).reshape(kps.shape)
    data = mir.to_dict()
    data["keypoints_3d"] = transformed.tolist()
    from robotdance_core.rd_mir import RdMir

    moved = embed(RdMir.model_validate(data))
    assert _cos(base, moved) > 0.99


def test_retrieval_ranks_same_class_higher() -> None:
    idx = MotionIndex()
    for mid, mir in {
        "dance_a": generate_dance(beats_per_second=1.0),
        "dance_b": generate_dance(beats_per_second=1.4),
        "backflip": generate_backflip(),
    }.items():
        mir.motion_id = mid
        idx.add_mir(mir)
    ranked = idx.query(embed(generate_dance(beats_per_second=1.2)), k=3)
    ids = [r[0] for r in ranked]
    # dance 2 件が backflip より上位。
    assert ids.index("backflip") == 2


def test_duplicate_detection() -> None:
    idx = MotionIndex()
    for mid in ("orig", "copy", "other"):
        mir = generate_backflip() if mid == "other" else generate_dance(beats_per_second=1.0)
        mir.motion_id = mid
        idx.add_mir(mir)
    dups = idx.duplicates(threshold=0.98)
    pairs = {frozenset((a, b)) for a, b, _ in dups}
    assert frozenset(("orig", "copy")) in pairs  # 同一 dance は重複検出
    assert frozenset(("orig", "other")) not in pairs


def test_project_2d_separates_classes() -> None:
    idx = MotionIndex()
    for mid, mir in {
        "dance_a": generate_dance(beats_per_second=1.0),
        "dance_b": generate_dance(beats_per_second=1.3),
        "backflip_a": generate_backflip(duration=1.6),
        "backflip_b": generate_backflip(duration=1.4),
    }.items():
        mir.motion_id = mid
        idx.add_mir(mir)
    p = idx.project_2d()
    assert p.shape == (4, 2)
    d_intra = np.linalg.norm(p[0] - p[1])           # dance 同士
    d_inter = np.linalg.norm(p[0] - p[2])           # dance vs backflip
    assert d_inter > d_intra


def test_render_motion_map(tmp_path) -> None:
    pytest.importorskip("matplotlib")
    from robotdance_viewer.motion_map import render_motion_map

    pts = np.array([[0.0, 0.0], [0.1, 0.1], [5.0, 5.0]])
    out = render_motion_map(pts, ["a", "b", "c"], tmp_path / "map.png",
                            groups=["x", "x", "y"])
    assert out.exists() and out.stat().st_size > 0
