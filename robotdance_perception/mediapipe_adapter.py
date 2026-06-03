"""MediaPipe Pose による local video → RD-MIR 抽出（v0）。

MediaPipe Pose Landmarker の pose_world_landmarks（33点・メートル単位の3D）を canonical
19-joint skeleton にマップして RD-MIR を生成する。これが "Shorts to humanoid" の入口。

⚠️ ライセンス: 入力動画の権利はユーザー責任。本アダプタは動画を再配布せず、
抽出した RD-MIR の license_state は既定で "unknown"（source 未確認）にする。

MediaPipe world landmark の座標系（x:右, y:下, z:カメラ手前が負, 原点:腰中心）を、
canonical（x:前, y:左, z:上）へ変換する: canonical = (-z, +x, -y)。
"""

from __future__ import annotations

import os
from pathlib import Path
from typing import Optional

import numpy as np

from robotdance_core.rd_mir import RdMir, Skeleton
from robotdance_core.skeleton import JOINT_NAMES, NUM_JOINTS, PARENTS, index_of

# 公式モデル（full）。実機 weights ではなく Google 配布の Apache-2.0 モデル。
DEFAULT_MODEL_URL = (
    "https://storage.googleapis.com/mediapipe-models/pose_landmarker/"
    "pose_landmarker_full/float16/latest/pose_landmarker_full.task"
)
DEFAULT_MODEL_PATH = Path.home() / ".cache" / "robotdance" / "models" / "pose_landmarker_full.task"

# MediaPipe Pose の 33 landmark index。
_MP = {
    "nose": 0, "l_ear": 7, "r_ear": 8,
    "l_shoulder": 11, "r_shoulder": 12, "l_elbow": 13, "r_elbow": 14,
    "l_wrist": 15, "r_wrist": 16,
    "l_hip": 23, "r_hip": 24, "l_knee": 25, "r_knee": 26,
    "l_ankle": 27, "r_ankle": 28, "l_foot": 31, "r_foot": 32,
}


def _to_canonical_frame(v: np.ndarray) -> np.ndarray:
    """MediaPipe world 座標 [.., 3] を canonical（x:前, y:左, z:上）へ。"""
    x, y, z = v[..., 0], v[..., 1], v[..., 2]
    return np.stack([-z, x, -y], axis=-1)


def mp_world_landmarks_to_canonical(world: np.ndarray) -> np.ndarray:
    """MediaPipe world landmarks [33, 3] → canonical 19-joint keypoints [19, 3]。

    direct な landmark は対応点を、pelvis/spine/chest/neck/head は補間で求める。
    """
    if world.shape != (33, 3):
        raise ValueError(f"world landmarks は [33,3] が必要: {world.shape}")
    c = _to_canonical_frame(world)  # [33,3] canonical 座標
    g = lambda k: c[_MP[k]]  # noqa: E731

    pelvis = 0.5 * (g("l_hip") + g("r_hip"))
    shoulder_c = 0.5 * (g("l_shoulder") + g("r_shoulder"))
    head = 0.5 * (g("l_ear") + g("r_ear"))
    chest = shoulder_c
    spine = pelvis + 0.5 * (chest - pelvis)
    neck = chest + 0.4 * (head - chest)

    out = np.zeros((NUM_JOINTS, 3))
    out[index_of("pelvis")] = pelvis
    out[index_of("spine")] = spine
    out[index_of("chest")] = chest
    out[index_of("neck")] = neck
    out[index_of("head")] = head
    out[index_of("left_shoulder")] = g("l_shoulder")
    out[index_of("right_shoulder")] = g("r_shoulder")
    out[index_of("left_elbow")] = g("l_elbow")
    out[index_of("right_elbow")] = g("r_elbow")
    out[index_of("left_wrist")] = g("l_wrist")
    out[index_of("right_wrist")] = g("r_wrist")
    out[index_of("left_hip")] = g("l_hip")
    out[index_of("right_hip")] = g("r_hip")
    out[index_of("left_knee")] = g("l_knee")
    out[index_of("right_knee")] = g("r_knee")
    out[index_of("left_ankle")] = g("l_ankle")
    out[index_of("right_ankle")] = g("r_ankle")
    out[index_of("left_foot")] = g("l_foot")
    out[index_of("right_foot")] = g("r_foot")
    return out


def _joint_visibility(vis33: np.ndarray) -> np.ndarray:
    """33 landmark の visibility → canonical 19 joint の confidence。"""
    v = lambda k: vis33[_MP[k]]  # noqa: E731
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
        ("left_foot", "l_foot"), ("right_foot", "r_foot"),
    ]:
        out[index_of(jn)] = v(mk)
    return out


def ensure_model(model_path: Optional[str | Path] = None) -> Path:
    """MediaPipe pose model を用意する。無ければ公式 URL から cache へ DL する。"""
    path = Path(model_path or os.environ.get("ROBOTDANCE_POSE_MODEL", DEFAULT_MODEL_PATH))
    if path.exists():
        return path
    import urllib.request

    path.parent.mkdir(parents=True, exist_ok=True)
    try:
        urllib.request.urlretrieve(DEFAULT_MODEL_URL, path)  # noqa: S310
    except Exception as e:  # pragma: no cover - ネットワーク依存
        raise RuntimeError(
            f"pose model を取得できません（{e}）。手動で {DEFAULT_MODEL_URL} を "
            f"{path} に置くか ROBOTDANCE_POSE_MODEL で指定してください。"
        ) from e
    return path


def extract_motion(
    video_path: str | Path,
    *,
    model_path: Optional[str | Path] = None,
    motion_id: Optional[str] = None,
    ground_align: bool = True,
) -> RdMir:
    """local 動画から canonical RD-MIR を抽出する。

    入力動画は再配布しない。license_state は "unknown"（source 未確認）。
    """
    import cv2
    import mediapipe as mp
    from mediapipe.tasks.python import BaseOptions, vision

    path = Path(video_path)
    model = ensure_model(model_path)
    cap = cv2.VideoCapture(str(path))
    if not cap.isOpened():
        raise FileNotFoundError(f"動画を開けません: {path}")
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0

    options = vision.PoseLandmarkerOptions(
        base_options=BaseOptions(model_asset_path=str(model)),
        running_mode=vision.RunningMode.VIDEO,
        num_poses=1,
    )
    landmarker = vision.PoseLandmarker.create_from_options(options)

    kps_frames: list[np.ndarray] = []
    conf_frames: list[np.ndarray] = []
    idx = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        ts_ms = int(idx * 1000.0 / fps)
        res = landmarker.detect_for_video(
            mp.Image(image_format=mp.ImageFormat.SRGB, data=np.ascontiguousarray(rgb)), ts_ms
        )
        if res.pose_world_landmarks:
            wl = res.pose_world_landmarks[0]
            world = np.array([[p.x, p.y, p.z] for p in wl])
            vis = np.array([p.visibility for p in res.pose_landmarks[0]])
            kps_frames.append(mp_world_landmarks_to_canonical(world))
            conf_frames.append(_joint_visibility(vis))
        idx += 1
    cap.release()

    if not kps_frames:
        raise RuntimeError(f"人物姿勢を検出できませんでした: {path}")

    kps = np.stack(kps_frames)  # [T, 19, 3]
    conf = np.stack(conf_frames)

    if ground_align:
        # 足の最下点を地面(z=0)付近へ。全フレーム共通オフセット。
        kps[:, :, 2] -= kps[:, [index_of("left_ankle"), index_of("right_ankle")], 2].min()

    n = kps.shape[0]
    duration = n / fps
    contacts = _estimate_contacts(kps)

    return RdMir(
        motion_id=motion_id or f"rdmir-video-{path.stem}",
        source_ref={"local_path": str(path), "extractor": "mediapipe_pose_full"},
        license_state="unknown",  # source 動画の権利は未確認 → 派生 motion を公開しない
        fps=float(fps),
        duration=float(duration),
        skeleton=Skeleton(joint_names=list(JOINT_NAMES), parents=list(PARENTS)),
        root_trajectory={"position": kps[:, index_of("pelvis"), :].tolist()},
        keypoints_3d=kps.tolist(),
        contacts=contacts,
        confidence={"joint": conf.tolist()},
        privacy_flags={"face_visible": True, "synthetic": False},
        quality_metrics={"mean_confidence": round(float(conf.mean()), 3)},
        extractor_versions={"pose": "mediapipe_pose_landmarker_full", "adapter": "robotdance.v0"},
        semantics={"action_label": "unknown"},
    )


def _estimate_contacts(kps: np.ndarray) -> dict[str, list[bool]]:
    """ankle 高さから接地を推定（最下点付近を接地とみなす単純ヒューリスティック）。"""
    out: dict[str, list[bool]] = {}
    for side in ("left", "right"):
        z = kps[:, index_of(f"{side}_ankle"), 2]
        thresh = float(z.min()) + 0.07
        out[f"{side}_foot"] = (z < thresh).tolist()
    return out
