"""単眼抽出の深度(前後 x)を観測性で安定化する（static-leg front-back split の抑制）。

単眼 3D lifting は**画像面（横 y・高さ z）は良いが前後 x 深度が ill-posed**。特に**画像内で動かない
関節**（立位の脚・体幹など）は深度の手がかりがゼロに近く、monocular が脚を前後に割る（spurious
front-back split）・深度がジッタする。これは shoulder press 等「腕は動くが脚は静止」のクリップで
ロボットが破綻する主因（[[real-video-demo-pipeline]] v0.125 で実証: sp30 で左右足首の前後差が
mean 0.172 m, grounding 振れ 0.192 m）。

`stabilize_depth` は**観測軸 y・z を一切変えず**、未観測の x のみを観測性（画像面の動きの大きさ）で
重み付けして安定化する:
  1. **temporal depth low-pass**: 画像内で静的なフレーム/関節ほど、深度の時間ジッタを smooth へ寄せる
     （静的なら深度は本来一定のはず＝ジッタは noise）。動いている関節はそのまま（実 motion を壊さない）。
  2. **static symmetric-pair leveling** (opt-in): 左右対称ペア（足首/膝/股/肩/手首）が**両方とも画像内で
     静的**なとき、その前後 x の非対称は観測不能＝spurious とみなし、両者を共有平均 x へ寄せる
     （持続スプリットの除去）。立位/運動の「足が横並び」前提——保持された lunge 等には適用しない想定。

⚠️ over-smoothing で見かけ良くする gimmick ではない（[[real-urdf-deepdive-thread]] ankle-strategy 却下と
同方針）。**本質的に未観測な深度だけ**を観測性で重み付けする。画像面 y,z は凍結し、**動いている関節は
触らない**（観測性が高い＝深度も比較的信頼できるため）。leveling の対称前提は opt-in・strength 制御・
本注記で明示する。[[real-video-demo-pipeline]] の retarget 側 balance prior（depth_refine）と相補的。
"""

from __future__ import annotations

import numpy as np

from robotdance_core.rd_mir import RdMir
from robotdance_core.skeleton import index_of
from robotdance_motion.smoothing import savgol_smooth

# 前後スプリットを起こしやすい左右対称ペア（leveling 対象）。
_SYM_PAIRS = [
    ("left_ankle", "right_ankle"), ("left_knee", "right_knee"),
    ("left_hip", "right_hip"), ("left_shoulder", "right_shoulder"),
    ("left_wrist", "right_wrist"), ("left_foot", "right_foot"),
    ("left_elbow", "right_elbow"),
]


def _image_plane_speed(kps: np.ndarray) -> np.ndarray:
    """各 frame・各 joint の画像面(y,z)速度 [T, J]（深度 x は除く）。端点は前方差分を複製。"""
    yz = kps[:, :, 1:3]                       # [T, J, 2]
    d = np.linalg.norm(np.diff(yz, axis=0), axis=2)   # [T-1, J]
    if d.shape[0] == 0:
        return np.zeros((kps.shape[0], kps.shape[1]))
    return np.vstack([d[:1], d])              # [T, J] に揃える


def stabilize_depth(
    mir: RdMir, *, strength: float = 0.7, motion_eps: float = 0.04,
    level_static_pairs: bool = True, smooth: bool = True,
) -> RdMir:
    """未観測の前後 x 深度のみを観測性で安定化した RD-MIR を返す（y, z は不変）。

    strength: 安定化の強さ（0=無補正, 1=静的関節で完全に smooth/leveling）。
    motion_eps: 画像面速度がこの値[m/frame]以下なら「静的＝深度が観測不能」とみなす基準。
    level_static_pairs: 静的左右対称ペアの前後スプリットを共有平均へ寄せる（持続スプリット除去）。
    smooth: temporal depth low-pass を行うか。
    入力 mir は変更せず deep copy を返す。
    """
    if mir.keypoints_3d is None:
        raise ValueError("keypoints_3d が無いため深度安定化できません")
    out = mir.model_copy(deep=True)
    kps = out.keypoints_3d_array().copy()     # [T, J, 3]
    n = kps.shape[0]
    x0 = kps[:, :, 0].copy()

    vimg = _image_plane_speed(kps)            # [T, J]
    # 観測性の低さ（=静的さ）s∈[0,1]: 画像面が動かないほど深度は信頼できない → s 大。
    static = np.clip(1.0 - vimg / max(motion_eps, 1e-9), 0.0, 1.0)   # [T, J]

    # 1. temporal depth low-pass（静的な関節・フレームほど深度ジッタを smooth へ寄せる）。
    if smooth and n >= 5:
        x_s = savgol_smooth(kps[:, :, 0:1]).reshape(n, -1)            # [T, J] 深度のみ平滑
        kps[:, :, 0] = kps[:, :, 0] + strength * static * (x_s - kps[:, :, 0])

    # 2. static symmetric-pair leveling（両方静的なペアの前後スプリットを平均へ）。
    split_before = _mean_leg_split(x0)
    if level_static_pairs:
        for a, b in _SYM_PAIRS:
            ia, ib = index_of(a), index_of(b)
            # ペアの per-joint 静的さ（全クリップ平均）。両方静的なときだけ強く効かせる。
            sa = float(np.clip(1.0 - vimg[:, ia].mean() / max(motion_eps, 1e-9), 0.0, 1.0))
            sb = float(np.clip(1.0 - vimg[:, ib].mean() / max(motion_eps, 1e-9), 0.0, 1.0))
            w = strength * min(sa, sb)
            if w <= 0.0:
                continue
            mean_x = 0.5 * (kps[:, ia, 0] + kps[:, ib, 0])
            kps[:, ia, 0] = kps[:, ia, 0] + w * (mean_x - kps[:, ia, 0])
            kps[:, ib, 0] = kps[:, ib, 0] + w * (mean_x - kps[:, ib, 0])
    split_after = _mean_leg_split(kps[:, :, 0])

    out.keypoints_3d = kps.tolist()
    out.root_trajectory = {"position": kps[:, index_of("pelvis"), :].tolist()}
    depth_jitter_before = _depth_jitter(x0)
    depth_jitter_after = _depth_jitter(kps[:, :, 0])
    q = dict(out.quality_metrics or {})
    q["depth_stabilize"] = {
        "applied": True,
        "strength": strength,
        "motion_eps": motion_eps,
        "leveled_pairs": bool(level_static_pairs),
        "leg_split_before_m": round(split_before, 4),
        "leg_split_after_m": round(split_after, 4),
        "depth_jitter_before": round(depth_jitter_before, 5),
        "depth_jitter_after": round(depth_jitter_after, 5),
    }
    out.quality_metrics = q
    return out


def _mean_leg_split(x: np.ndarray) -> float:
    """左右足首の前後(x)差の平均絶対値 [m]（front-back split 指標）。x は [T, J]。"""
    la = x[:, index_of("left_ankle")]
    ra = x[:, index_of("right_ankle")]
    return float(np.abs(la - ra).mean())


def _depth_jitter(x: np.ndarray) -> float:
    """深度 x の frame 間変化の平均（全関節）[m/frame]。x は [T, J]。"""
    if x.shape[0] < 2:
        return 0.0
    return float(np.abs(np.diff(x, axis=0)).mean())


__all__ = ["stabilize_depth"]
