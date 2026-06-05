"""pose 検出バックエンドのレジストリ（v0）。

RobotDance の抽出は MediaPipe Pose（BlazePose, 3D world landmarks）が既定だが、pose 検出器は
複数ある。本モジュールは各バックエンドの**能力メタデータ**（出力次元・keypoint 形式・retarget 可否・
必要依存）を 1 か所に束ね、CLI/スクリプトから一覧・選択できるようにする。

重要な設計上の正直さ:
- **MediaPipe のみ 3D world landmarks** を返し、actuator retarget（実機関節角）に直結できる。
- **YOLO11-pose / RTMPose は 2D**（COCO-17）。検出は速い/軽いが、3D 化には別途 lifting が要る。
  そのため `retarget_capable=False` として登録し、`extract`（フル抽出）には使えないことを型で表す。

heavy 依存（mediapipe/ultralytics/rtmlib）は**モジュール読み込み時に import しない**。
`available()` だけが遅延 import で可否を判定する（dev 環境にしか無くても本モジュールは壊れない）。
比較スクリプトは [[real-video-demo-pipeline]] / scripts/compare_pose_backends.py を参照。
"""

from __future__ import annotations

import importlib.util
from dataclasses import dataclass, field


@dataclass(frozen=True)
class PoseBackend:
    """1 つの pose 検出バックエンドの能力メタデータ。"""

    name: str
    output_dim: int  # 2（画像座標のみ）or 3（world landmarks）
    keypoint_format: str  # "blazepose33" / "coco17" など
    retarget_capable: bool  # 3D 関節角 retarget に直結できるか
    modules: tuple[str, ...]  # 必要な import モジュール名（遅延チェック用）
    description: str = ""
    extras: tuple[str, ...] = field(default_factory=tuple)  # 公開 PyPI 由来でない dev 専用なら ("dev",)

    def available(self) -> bool:
        """必要モジュールが import 可能か（heavy 依存を実際には読み込まずに判定）。"""
        return all(importlib.util.find_spec(m) is not None for m in self.modules)


# 既定 3D バックエンド（フル抽出に使える唯一の登録）。
MEDIAPIPE = PoseBackend(
    name="mediapipe",
    output_dim=3,
    keypoint_format="blazepose33",
    retarget_capable=True,
    modules=("mediapipe", "cv2"),
    description="MediaPipe Pose Landmarker (BlazePose, 33pt). 3D world landmarks → canonical RD-MIR.",
)

# 2D-only バックエンド（検出比較・overlay 用。フル抽出には lifting が必要）。
YOLO11_POSE = PoseBackend(
    name="yolo11-pose",
    output_dim=2,
    keypoint_format="coco17",
    retarget_capable=False,
    modules=("ultralytics", "cv2"),
    description="Ultralytics YOLO11-pose (COCO-17, 2D). 高速/軽量。3D 化には lifting が必要。",
    extras=("dev",),
)
RTMPOSE = PoseBackend(
    name="rtmpose",
    output_dim=2,
    keypoint_format="coco17",
    retarget_capable=False,
    modules=("rtmlib", "cv2"),
    description="RTMPose via rtmlib (COCO-17, 2D, onnxruntime). 3D 化には lifting が必要。",
    extras=("dev",),
)

_REGISTRY: dict[str, PoseBackend] = {
    b.name: b for b in (MEDIAPIPE, YOLO11_POSE, RTMPOSE)
}


def list_backends() -> list[PoseBackend]:
    """登録済みバックエンドを名前順で返す。"""
    return [_REGISTRY[k] for k in sorted(_REGISTRY)]


def get_backend(name: str) -> PoseBackend:
    """名前からバックエンドを取得。未知なら候補を添えて ValueError。"""
    try:
        return _REGISTRY[name]
    except KeyError:
        known = ", ".join(sorted(_REGISTRY))
        raise ValueError(f"未知の pose backend '{name}'。利用可能: {known}") from None


def resolve_extract_backend(name: str) -> PoseBackend:
    """フル抽出（extract）用にバックエンドを解決する。

    retarget 非対応（2D-only）の指定は、3D が必要な旨を明示して拒否する。
    """
    b = get_backend(name)
    if not b.retarget_capable:
        raise ValueError(
            f"backend '{name}' は 2D（{b.keypoint_format}）で、RD-MIR フル抽出には 3D が必要です。"
            f" 現状 retarget 可能なのは: "
            f"{', '.join(x.name for x in list_backends() if x.retarget_capable)}。"
            f" 2D 検出器の比較は scripts/compare_pose_backends.py を使ってください。"
        )
    return b


# ---------------------------------------------------------------------------
# 共通 COCO-17 表現と 2D ランナー（検出器を横並び比較する際の単一情報源）
# ---------------------------------------------------------------------------

# COCO-17 の骨格エッジ（overlay 描画用）。
COCO_EDGES: tuple[tuple[int, int], ...] = (
    (5, 7), (7, 9), (6, 8), (8, 10), (5, 6), (5, 11), (6, 12), (11, 12),
    (11, 13), (13, 15), (12, 14), (14, 16), (0, 5), (0, 6), (0, 1), (0, 2), (1, 3), (2, 4),
)
# MediaPipe BlazePose 33 → COCO 17 の対応 index。
MP33_TO_COCO: tuple[int, ...] = (0, 2, 5, 7, 8, 11, 12, 13, 14, 15, 16, 23, 24, 25, 26, 27, 28)


def _largest_person(kxy, kconf):
    """多人数検出から bbox 面積が最大の人物（前景被写体）を選ぶ。"""
    import numpy as np

    best, area = 0, -1.0
    for i in range(len(kxy)):
        pts = kxy[i][kconf[i] > 0.2]
        if len(pts) < 4:
            continue
        a = (pts[:, 0].max() - pts[:, 0].min()) * (pts[:, 1].max() - pts[:, 1].min())
        if a > area:
            area, best = a, i
    return np.asarray(kxy[best]), np.asarray(kconf[best])


def _mediapipe_runner():
    import numpy as np
    import mediapipe as mp
    from mediapipe.tasks.python import BaseOptions, vision

    opt = vision.PoseLandmarkerOptions(
        base_options=BaseOptions(model_asset_path=str(ensure_model())),
        running_mode=vision.RunningMode.VIDEO, num_poses=1)
    lm = vision.PoseLandmarker.create_from_options(opt)

    def run(frame_bgr, idx, fps):
        import cv2

        h, w = frame_bgr.shape[:2]
        rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        res = lm.detect_for_video(
            mp.Image(image_format=mp.ImageFormat.SRGB, data=np.ascontiguousarray(rgb)),
            int(idx * 1000.0 / fps))
        if not res.pose_landmarks:
            return None
        nl = res.pose_landmarks[0]
        full = np.array([[p.x * w, p.y * h] for p in nl])
        vis = np.array([p.visibility for p in nl])
        return full[list(MP33_TO_COCO)], vis[list(MP33_TO_COCO)]
    return run


def _yolo_runner():
    from ultralytics import YOLO

    yolo = YOLO("yolo11n-pose.pt")

    def run(frame_bgr, idx, fps):
        kp = yolo(frame_bgr, verbose=False)[0].keypoints
        if kp is None or not len(kp.data):
            return None
        return _largest_person(kp.xy.cpu().numpy(), kp.conf.cpu().numpy())
    return run


def _rtmpose_runner():
    import numpy as np
    from rtmlib import Body

    body = Body(mode="lightweight", backend="onnxruntime", device="cpu")

    def run(frame_bgr, idx, fps):
        kxy, ksc = body(frame_bgr)
        if not len(kxy):
            return None
        return _largest_person(np.array(kxy), np.array(ksc))
    return run


_RUNNER_FACTORIES = {
    "mediapipe": _mediapipe_runner,
    "yolo11-pose": _yolo_runner,
    "rtmpose": _rtmpose_runner,
}


def make_runner_2d(name: str):
    """バックエンドの 2D 検出ランナー `run(frame_bgr, idx, fps) -> (xy[17,2], conf[17]) | None` を返す。

    出力は全バックエンド共通の COCO-17。heavy 依存はこの呼び出し時にのみ遅延 import される。
    検出器が未導入なら ImportError を投げる（available() で事前判定可能）。
    """
    get_backend(name)  # 未知名なら ValueError
    factory = _RUNNER_FACTORIES.get(name)
    if factory is None:  # pragma: no cover - レジストリと辞書は同期している
        raise ValueError(f"backend '{name}' に 2D ランナーがありません")
    return factory()


# 遅延 import の循環を避けるため、ensure_model はここで局所 import する。
def ensure_model(*a, **k):
    """MediaPipe pose model を用意する（mediapipe_adapter.ensure_model への薄い委譲）。"""
    from robotdance_perception.mediapipe_adapter import ensure_model as _em

    return _em(*a, **k)
