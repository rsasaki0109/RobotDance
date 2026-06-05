"""2D COCO-17 → canonical 3D の解析的 planar lift（v0・粗いベースライン）。

YOLO11-pose / RTMPose などの 2D 検出器は COCO-17 の画像座標しか返さない。本モジュールは
それを canonical 19-joint 3D へ**幾何学的に**持ち上げる。ただし**深度（前後 x）は復元しない**:
2D 検出は正面平面（coronal）の姿勢なので、x≈0 の平面へ埋め込み、hip 幅の人体寸法プライアで
メートル化するだけ。学習済み lifter を使わないため依存も license も軽い。

⚠️ 正直な限界: これは MediaPipe の native 3D（world landmarks）より**明確に粗い**。深度が無いので
**矢状面（sagittal）の動き（しゃがみ等）は潰れる**。正面・冠状面（coronal）の動き（空手の型など）
なら retarget に使える、という位置づけの coarse baseline。native との差は `pose-compare` 系で確認可能。
関連: [[real-video-demo-pipeline]] の「クリップ選定の鉄則」（単眼は深度が最も不確実）。
"""

from __future__ import annotations

from pathlib import Path
from typing import Optional

import numpy as np

from robotdance_core.skeleton import NUM_JOINTS, index_of

# COCO-17 の landmark index。
_COCO = {
    "nose": 0, "l_eye": 1, "r_eye": 2, "l_ear": 3, "r_ear": 4,
    "l_shoulder": 5, "r_shoulder": 6, "l_elbow": 7, "r_elbow": 8,
    "l_wrist": 9, "r_wrist": 10, "l_hip": 11, "r_hip": 12,
    "l_knee": 13, "r_knee": 14, "l_ankle": 15, "r_ankle": 16,
}

# 人体寸法プライア: 左右 ASIS（腰骨）間の幅 [m]。メートル化のスケール基準。
DEFAULT_HIP_WIDTH_M = 0.26


def compose_canonical_coco(pts: np.ndarray) -> np.ndarray:
    """COCO-17 の点列 [17, K] から canonical 19-joint [19, K] を合成する（座標系非依存）。

    pelvis/spine/chest/neck/head は補間、foot は ankle から膝方向の逆へ少し延長して近似。
    """
    g = lambda k: pts[_COCO[k]]  # noqa: E731
    pelvis = 0.5 * (g("l_hip") + g("r_hip"))
    chest = 0.5 * (g("l_shoulder") + g("r_shoulder"))
    head = 0.5 * (g("l_ear") + g("r_ear"))
    spine = pelvis + 0.5 * (chest - pelvis)
    neck = chest + 0.4 * (head - chest)

    out = np.zeros((NUM_JOINTS, pts.shape[1]))
    out[index_of("pelvis")] = pelvis
    out[index_of("spine")] = spine
    out[index_of("chest")] = chest
    out[index_of("neck")] = neck
    out[index_of("head")] = head
    for jn, mk in [
        ("left_shoulder", "l_shoulder"), ("right_shoulder", "r_shoulder"),
        ("left_elbow", "l_elbow"), ("right_elbow", "r_elbow"),
        ("left_wrist", "l_wrist"), ("right_wrist", "r_wrist"),
        ("left_hip", "l_hip"), ("right_hip", "r_hip"),
        ("left_knee", "l_knee"), ("right_knee", "r_knee"),
        ("left_ankle", "l_ankle"), ("right_ankle", "r_ankle"),
    ]:
        out[index_of(jn)] = g(mk)
    # COCO に足先は無い。ankle から膝の逆方向へ 0.3 延長して足先を近似する。
    for side in ("left", "right"):
        ankle = out[index_of(f"{side}_ankle")]
        knee = out[index_of(f"{side}_knee")]
        out[index_of(f"{side}_foot")] = ankle + 0.3 * (ankle - knee)
    return out


def compose_canonical_conf_coco(conf17: np.ndarray) -> np.ndarray:
    """COCO-17 の confidence [17] → canonical 19-joint [19]。"""
    v = lambda k: conf17[_COCO[k]]  # noqa: E731
    out = np.zeros(NUM_JOINTS)
    out[index_of("pelvis")] = 0.5 * (v("l_hip") + v("r_hip"))
    out[index_of("spine")] = out[index_of("pelvis")]
    out[index_of("chest")] = 0.5 * (v("l_shoulder") + v("r_shoulder"))
    out[index_of("neck")] = out[index_of("chest")]
    out[index_of("head")] = 0.5 * (v("l_ear") + v("r_ear"))
    for jn, mk in [
        ("left_shoulder", "l_shoulder"), ("right_shoulder", "r_shoulder"),
        ("left_elbow", "l_elbow"), ("right_elbow", "r_elbow"),
        ("left_wrist", "l_wrist"), ("right_wrist", "r_wrist"),
        ("left_hip", "l_hip"), ("right_hip", "r_hip"),
        ("left_knee", "l_knee"), ("right_knee", "r_knee"),
        ("left_ankle", "l_ankle"), ("right_ankle", "r_ankle"),
    ]:
        out[index_of(jn)] = v(mk)
    for side in ("left", "right"):
        out[index_of(f"{side}_foot")] = out[index_of(f"{side}_ankle")]
    return out


def lift_coco17_to_canonical(
    xy: np.ndarray,
    conf: np.ndarray,
    *,
    hip_width_m: float = DEFAULT_HIP_WIDTH_M,
) -> tuple[np.ndarray, np.ndarray]:
    """COCO-17 の画像座標 [17,2]（x:右, y:下, px）→ canonical 19-joint 3D [19,3]。

    深度は復元せず x（前後）=0 の平面へ埋め込む。canonical は x:前, y:左, z:上。
    画像 x(右)→ -y、画像 y(下)→ -z に写し、hip 幅で metric 化、足を z=0 へ接地する。
    返り値: (kps[19,3], conf[19])。
    """
    if xy.shape != (17, 2):
        raise ValueError(f"COCO-17 xy は [17,2] が必要: {xy.shape}")
    can2d = compose_canonical_coco(xy)  # [19, 2]（px, x右/y下）

    # hip 幅（px）でメートルスケールを決める。0 割は回避。
    lh, rh = can2d[index_of("left_hip")], can2d[index_of("right_hip")]
    px_hip = float(np.linalg.norm(lh - rh))
    scale = hip_width_m / px_hip if px_hip > 1e-6 else 0.0

    pelvis_x = can2d[index_of("pelvis"), 0]
    out = np.zeros((NUM_JOINTS, 3))
    out[:, 0] = 0.0  # 前後（深度）は未復元 → 平面
    out[:, 1] = -(can2d[:, 0] - pelvis_x) * scale  # 左右（pelvis を原点に）
    out[:, 2] = -(can2d[:, 1]) * scale  # 上下（画像下方向が負 → 上が正）
    out[:, 2] -= out[[index_of("left_foot"), index_of("right_foot")], 2].min()  # 足接地 z=0

    return out, compose_canonical_conf_coco(conf)


def extract_via_lift(
    video_path: str | Path,
    *,
    detector: str = "yolo11-pose",
    motion_id: Optional[str] = None,
    hip_width_m: float = DEFAULT_HIP_WIDTH_M,
    smooth: bool = True,
):
    """2D 検出器 + planar lift で local 動画から canonical RD-MIR を抽出する（coarse baseline）。

    検出器（COCO-17 2D）を毎フレーム走らせ planar lift で 3D 化する。深度は未復元なので
    quality_metrics に lift="planar-no-depth" を記録し、native 3D（mediapipe）と区別する。
    入力動画は再配布しない（license_state="unknown"）。
    """
    import cv2

    from robotdance_core.rd_mir import RdMir, Skeleton
    from robotdance_core.skeleton import JOINT_NAMES, PARENTS
    from robotdance_perception.backends import make_runner_2d
    from robotdance_perception.mediapipe_adapter import _estimate_contacts

    run = make_runner_2d(detector)
    path = Path(video_path)
    cap = cv2.VideoCapture(str(path))
    if not cap.isOpened():
        raise FileNotFoundError(f"動画を開けません: {path}")
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)) or 0
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)) or 0

    kps_frames: list[np.ndarray] = []
    conf_frames: list[np.ndarray] = []
    idx = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        res = run(frame, idx, fps)
        if res is not None:
            xy, c = res
            k3d, c19 = lift_coco17_to_canonical(np.asarray(xy), np.asarray(c),
                                                hip_width_m=hip_width_m)
            kps_frames.append(k3d)
            conf_frames.append(c19)
        idx += 1
    cap.release()

    if not kps_frames:
        raise RuntimeError(f"人物姿勢を検出できませんでした: {path}")

    kps = np.stack(kps_frames)
    conf = np.stack(conf_frames)

    quality: dict[str, object] = {
        "mean_confidence": round(float(conf.mean()), 3),
        "lift": "planar-no-depth",
        "lift_detector": detector,
    }
    if smooth:
        from robotdance_motion.smoothing import jitter, savgol_smooth

        quality["jitter_before"] = round(jitter(kps), 5)
        kps = savgol_smooth(kps)
        quality["jitter_after"] = round(jitter(kps), 5)
        quality["smoothing"] = "savgol(window=7,polyorder=2)"

    n = kps.shape[0]
    return RdMir(
        motion_id=motion_id or f"rdmir-lift-{path.stem}",
        source_ref={"local_path": str(path), "extractor": f"{detector}+planar_lift"},
        license_state="unknown",
        fps=float(fps),
        duration=float(n / fps),
        world_frame={"up_axis": "z", "forward_axis": "x", "handedness": "right"},
        skeleton=Skeleton(joint_names=list(JOINT_NAMES), parents=list(PARENTS)),
        root_trajectory={"position": kps[:, index_of("pelvis"), :].tolist()},
        keypoints_3d=kps.tolist(),
        contacts=_estimate_contacts(kps),
        confidence={"joint": conf.tolist()},
        camera={"image_width": width, "image_height": height},
        privacy_flags={"face_visible": True, "synthetic": False},
        quality_metrics=quality,
        extractor_versions={"pose": f"{detector}_planar_lift", "adapter": "robotdance.v0"},
        semantics={"action_label": "unknown"},
    )
