"""RD-MIR の temporal smoothing と jitter 指標（v0）。

MediaPipe など monocular pose の出力は frame ごとに jittery。Savitzky-Golay フィルタで
keypoints_3d / root を時間方向に平滑化し、jitter（フレーム間加速度）を定量化する。
"""

from __future__ import annotations

import numpy as np

from robotdance_core.rd_mir import RdMir


def add_jitter(mir: RdMir, *, sigma: float = 0.02, seed: int = 0) -> RdMir:
    """keypoints_3d にガウスノイズを加えた RD-MIR を返す（smoothing デモ用）。"""
    rng = np.random.default_rng(seed)
    kps = mir.keypoints_3d_array()
    noisy = kps + rng.normal(0.0, sigma, size=kps.shape)
    data = mir.to_dict()
    data["motion_id"] = f"{mir.motion_id}-jittered"
    data["keypoints_3d"] = noisy.tolist()
    data["root_trajectory"] = {"position": noisy[:, 0, :].tolist()}
    return RdMir.model_validate(data)


def jitter(kps: np.ndarray) -> float:
    """時間的 jitter = フレーム間加速度の平均ノルム（小さいほど滑らか）。

    kps: [T, J, 3]。単位は keypoints と同じ（m なら m/frame^2）。
    """
    if kps.shape[0] < 3:
        return 0.0
    acc = kps[2:] - 2 * kps[1:-1] + kps[:-2]  # 二階差分 [T-2, J, 3]
    return float(np.linalg.norm(acc, axis=2).mean())


def savgol_smooth(kps: np.ndarray, *, window: int = 7, polyorder: int = 2) -> np.ndarray:
    """[T, J, 3] を時間軸に Savitzky-Golay 平滑化する。短いクリップは自動で縮める。"""
    from scipy.signal import savgol_filter

    t = kps.shape[0]
    if t < 3:
        return kps.copy()
    win = min(window, t if t % 2 == 1 else t - 1)  # window は奇数かつ <= T
    if win < 3:
        return kps.copy()
    poly = min(polyorder, win - 1)
    return savgol_filter(kps, window_length=win, polyorder=poly, axis=0)


def smooth_rdmir(mir: RdMir, *, window: int = 7, polyorder: int = 2) -> RdMir:
    """RD-MIR の keypoints_3d / root_trajectory を平滑化した新しい RD-MIR を返す。

    quality_metrics に jitter_before / jitter_after を記録する。
    """
    kps = mir.keypoints_3d_array()
    smoothed = savgol_smooth(kps, window=window, polyorder=polyorder)

    qm = dict(mir.quality_metrics or {})
    qm["jitter_before"] = round(jitter(kps), 5)
    qm["jitter_after"] = round(jitter(smoothed), 5)
    qm["smoothing"] = f"savgol(window={window},polyorder={polyorder})"

    data = mir.to_dict()
    data["keypoints_3d"] = smoothed.tolist()
    data["root_trajectory"] = {"position": smoothed[:, 0, :].tolist()}
    data["quality_metrics"] = qm
    return RdMir.model_validate(data)
