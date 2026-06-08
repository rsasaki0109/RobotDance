"""深度安定化（stabilize_depth）の検証。

単眼で ill-posed な前後 x 深度を観測性で安定化する。確認項目: (1) 観測軸 y・z を変えない、
(2) 静的な左右脚の front-back split を縮める、(3) 画像内で動いている対称ペアは leveling しない
（実 motion を保つ）、(4) 入力を破壊しない。sim/mediapipe は不要。
"""

from __future__ import annotations

import numpy as np

from robotdance_core.skeleton import index_of
from robotdance_core.synthetic import generate_squat
from robotdance_motion.depth_stabilize import stabilize_depth


def _standing(n: int = 60):
    """静止立位の合成スケルトン（squat の第1フレームを n フレーム複製＋微小ジッタ）。"""
    mir = generate_squat(duration=2.0)
    k = mir.keypoints_3d_array()
    base = k[0]
    rng = np.linspace(0, 1, n)
    arr = np.repeat(base[None], n, axis=0)
    # 全関節に微小な画像面ジッタ（静的判定されるレベル）。
    arr[:, :, 1] += 0.002 * np.sin(rng * 7)[:, None]
    arr[:, :, 2] += 0.002 * np.cos(rng * 5)[:, None]
    out = mir.model_copy(deep=True)
    out.keypoints_3d = arr.tolist()
    return out


def _ankle_split(k: np.ndarray) -> float:
    return float(np.abs(k[:, index_of("left_ankle"), 0] - k[:, index_of("right_ankle"), 0]).mean())


def test_stabilize_reduces_static_leg_split() -> None:
    mir = _standing()
    k = mir.keypoints_3d_array()
    # 静的脚に前後スプリットを注入（左足を後ろ、右足を前へ）。
    k[:, index_of("left_ankle"), 0] -= 0.15
    k[:, index_of("right_ankle"), 0] += 0.05
    mir.keypoints_3d = k.tolist()
    before = _ankle_split(mir.keypoints_3d_array())

    out = stabilize_depth(mir)
    after = _ankle_split(out.keypoints_3d_array())
    assert after < before * 0.6  # 静的脚スプリットが大きく縮む
    ds = out.quality_metrics["depth_stabilize"]
    assert ds["leg_split_after_m"] < ds["leg_split_before_m"]


def test_stabilize_freezes_observed_yz() -> None:
    mir = _standing()
    before = mir.keypoints_3d_array().copy()
    out = stabilize_depth(mir).keypoints_3d_array()
    assert np.allclose(out[:, :, 1], before[:, :, 1], atol=1e-9)
    assert np.allclose(out[:, :, 2], before[:, :, 2], atol=1e-9)


def test_moving_pair_is_not_leveled() -> None:
    # 画像面で大きく動く（=観測性が高い）対称ペアは深度の非対称を保つ（leveling しない）。
    mir = _standing(n=60)
    k = mir.keypoints_3d_array()
    t = np.linspace(0, 1, k.shape[0])
    # 手首を画像面(z)で速く大きく動かす＝観測性が高い（per-frame ≫ motion_eps）。深度は左右非対称に保つ。
    k[:, index_of("left_wrist"), 2] += 0.4 * np.sin(t * 25)
    k[:, index_of("right_wrist"), 2] += 0.4 * np.sin(t * 25)
    k[:, index_of("left_wrist"), 0] -= 0.20
    k[:, index_of("right_wrist"), 0] += 0.20
    mir.keypoints_3d = k.tolist()
    split_before = float(np.abs(k[:, index_of("left_wrist"), 0]
                                - k[:, index_of("right_wrist"), 0]).mean())

    out = stabilize_depth(mir).keypoints_3d_array()
    split_after = float(np.abs(out[:, index_of("left_wrist"), 0]
                               - out[:, index_of("right_wrist"), 0]).mean())
    # 動く手首ペアの深度非対称は概ね保たれる（静的脚ほど leveling されない）。
    assert split_after > split_before * 0.7


def test_stabilize_does_not_mutate_input() -> None:
    mir = _standing()
    before = mir.keypoints_3d_array().copy()
    _ = stabilize_depth(mir)
    assert np.array_equal(mir.keypoints_3d_array(), before)
