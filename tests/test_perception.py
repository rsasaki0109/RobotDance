"""MediaPipe アダプタの検証。

マッピングは純関数なので mediapipe なしでテストできる。実ピクセル統合テストは
scikit-image の astronaut（NASA, public domain）を使い、未インストールなら skip。
"""

from __future__ import annotations

import numpy as np
import pytest

from robotdance_core.skeleton import NUM_JOINTS, index_of
from robotdance_perception.mediapipe_adapter import (
    _MP,
    mp_world_landmarks_to_canonical,
)


def _standing_world_landmarks() -> np.ndarray:
    """立ち姿の MediaPipe world landmarks [33,3]（x:右, y:下, z:手前負, 腰原点）を合成。"""
    w = np.zeros((33, 3))
    # y は下向きが正。頭は上 = y 負。腰中心が原点。
    w[_MP["nose"]] = [0.0, -0.75, -0.1]
    w[_MP["l_ear"]] = [0.07, -0.78, 0.0]
    w[_MP["r_ear"]] = [-0.07, -0.78, 0.0]
    w[_MP["l_shoulder"]] = [0.18, -0.55, 0.0]
    w[_MP["r_shoulder"]] = [-0.18, -0.55, 0.0]
    w[_MP["l_elbow"]] = [0.20, -0.30, 0.0]
    w[_MP["r_elbow"]] = [-0.20, -0.30, 0.0]
    w[_MP["l_wrist"]] = [0.21, -0.05, 0.0]
    w[_MP["r_wrist"]] = [-0.21, -0.05, 0.0]
    w[_MP["l_hip"]] = [0.10, 0.0, 0.0]
    w[_MP["r_hip"]] = [-0.10, 0.0, 0.0]
    w[_MP["l_knee"]] = [0.10, 0.45, 0.0]
    w[_MP["r_knee"]] = [-0.10, 0.45, 0.0]
    w[_MP["l_ankle"]] = [0.10, 0.90, 0.0]
    w[_MP["r_ankle"]] = [-0.10, 0.90, 0.0]
    w[_MP["l_foot"]] = [0.10, 0.93, -0.12]
    w[_MP["r_foot"]] = [-0.10, 0.93, -0.12]
    return w


def test_mapping_shape() -> None:
    c = mp_world_landmarks_to_canonical(_standing_world_landmarks())
    assert c.shape == (NUM_JOINTS, 3)


def test_mapping_z_up_orientation() -> None:
    """canonical は z-up: 頭は腰より上、足首は腰より下。"""
    c = mp_world_landmarks_to_canonical(_standing_world_landmarks())
    assert c[index_of("head")][2] > c[index_of("pelvis")][2]
    assert c[index_of("left_ankle")][2] < c[index_of("pelvis")][2]


def test_mapping_left_right_handedness() -> None:
    """canonical は y-left: person の左関節は y が正側。"""
    c = mp_world_landmarks_to_canonical(_standing_world_landmarks())
    assert c[index_of("left_shoulder")][1] > c[index_of("right_shoulder")][1]
    assert c[index_of("left_hip")][1] > c[index_of("right_hip")][1]


def test_mapping_forward_axis() -> None:
    """canonical は x-forward: つま先は足首より前（x 大）。"""
    c = mp_world_landmarks_to_canonical(_standing_world_landmarks())
    assert c[index_of("left_foot")][0] > c[index_of("left_ankle")][0]


def test_invalid_shape_raises() -> None:
    with pytest.raises(ValueError):
        mp_world_landmarks_to_canonical(np.zeros((10, 3)))


def test_real_pixels_astronaut(tmp_path) -> None:
    """実写人物（skimage astronaut, public domain）で検出 → canonical を検証。"""
    pytest.importorskip("mediapipe")
    pytest.importorskip("cv2")
    data = pytest.importorskip("skimage.data")
    import numpy as np
    from mediapipe import Image, ImageFormat
    from mediapipe.tasks.python import BaseOptions, vision

    from robotdance_perception.mediapipe_adapter import _joint_visibility, ensure_model

    img = data.astronaut()
    model = ensure_model()
    lmk = vision.PoseLandmarker.create_from_options(
        vision.PoseLandmarkerOptions(
            base_options=BaseOptions(model_asset_path=str(model)),
            running_mode=vision.RunningMode.IMAGE,
            num_poses=1,
        )
    )
    res = lmk.detect(Image(image_format=ImageFormat.SRGB, data=np.ascontiguousarray(img)))
    assert res.pose_world_landmarks, "実写人物を検出できなかった"
    world = np.array([[p.x, p.y, p.z] for p in res.pose_world_landmarks[0]])
    c = mp_world_landmarks_to_canonical(world)
    assert c.shape == (NUM_JOINTS, 3)
    # ポートレートでも上半身は高 confidence。
    vis = _joint_visibility(np.array([p.visibility for p in res.pose_landmarks[0]]))
    assert vis[index_of("left_shoulder")] > 0.5
    # 頭は肩より上。
    assert c[index_of("head")][2] > c[index_of("chest")][2]
