"""RD-MIR 健全性チェック（robotdance_motion.doctor）。合成 RD-MIR で純関数を検証。"""

from __future__ import annotations

import numpy as np

from robotdance_core.rd_mir import RdMir, Skeleton
from robotdance_core.skeleton import JOINT_NAMES, NUM_JOINTS, PARENTS, index_of
from robotdance_motion.doctor import diagnose_motion, overall_status


def _standing_kps(t: int = 10, *, mirror: bool = False, planar: bool = False,
                  foot_jiggle: float = 0.0) -> np.ndarray:
    """立位の canonical keypoints [T,19,3]（x:前, y:左, z:上）を合成。"""
    base = np.zeros((NUM_JOINTS, 3))
    base[index_of("pelvis")] = [0, 0, 0.9]
    base[index_of("chest")] = [0, 0, 1.3]
    base[index_of("head")] = [0, 0, 1.6]
    base[index_of("left_hip")] = [0, 0.1, 0.9]
    base[index_of("right_hip")] = [0, -0.1, 0.9]
    base[index_of("left_shoulder")] = [0, 0.2, 1.35]
    base[index_of("right_shoulder")] = [0, -0.2, 1.35]
    base[index_of("left_knee")] = [0.05, 0.1, 0.5]
    base[index_of("right_knee")] = [0.05, -0.1, 0.5]
    base[index_of("left_ankle")] = [0, 0.1, 0.05]
    base[index_of("right_ankle")] = [0, -0.1, 0.05]
    base[index_of("left_foot")] = [0.1, 0.1, 0.0]
    base[index_of("right_foot")] = [0.1, -0.1, 0.0]
    if mirror:
        base[:, 1] *= -1  # 左右反転
    seq = np.repeat(base[None], t, axis=0)
    if planar:
        seq[:, :, 0] = 0.0  # 深度ゼロ
    else:
        seq[:, :, 0] += 0.1 * np.sin(np.linspace(0, 6, t))[:, None]  # 前後に揺らす
    if foot_jiggle:
        seq[:, [index_of("left_foot"), index_of("right_foot")], 2] += \
            foot_jiggle * np.sin(np.linspace(0, 6, t))[:, None]
    return seq


def _mir(kps: np.ndarray, *, quality: dict | None = None) -> RdMir:
    return RdMir(
        motion_id="t", source_ref={}, license_state="unknown",
        fps=30.0, duration=kps.shape[0] / 30.0,
        skeleton=Skeleton(joint_names=list(JOINT_NAMES), parents=list(PARENTS)),
        keypoints_3d=kps.tolist(),
        quality_metrics=quality or {"mean_confidence": 0.9, "jitter_after": 0.01},
    )


def _by_name(checks):
    return {c.name: c for c in checks}


def test_healthy_motion_all_ok():
    checks = _by_name(diagnose_motion(_mir(_standing_kps())))
    assert checks["mirror"].status == "ok"
    assert checks["depth_collapse"].status == "ok"
    assert checks["grounding"].status == "ok"
    assert overall_status(list(checks.values())) == "ok"


def test_mirror_detected():
    checks = _by_name(diagnose_motion(_mir(_standing_kps(mirror=True))))
    assert checks["mirror"].status == "warn"
    assert "反転" in checks["mirror"].message
    assert checks["mirror"].hint


def test_depth_collapse_warns_for_native_but_info_for_lift():
    native = _by_name(diagnose_motion(_mir(_standing_kps(planar=True))))
    assert native["depth_collapse"].status == "warn"
    lift = _by_name(diagnose_motion(_mir(_standing_kps(planar=True),
                                         quality={"lift": "planar-no-depth"})))
    assert lift["depth_collapse"].status == "info"


def test_grounding_warns_on_foot_skate():
    checks = _by_name(diagnose_motion(_mir(_standing_kps(foot_jiggle=0.3))))
    assert checks["grounding"].status == "warn"


def test_low_confidence_and_multi_subject_warn():
    checks = _by_name(diagnose_motion(_mir(
        _standing_kps(), quality={"mean_confidence": 0.4, "n_subjects_max": 3})))
    assert checks["confidence"].status == "warn"
    assert checks["multi_subject"].status == "warn"


def test_overall_status_warn_if_any():
    assert overall_status(diagnose_motion(_mir(_standing_kps(mirror=True)))) == "warn"
