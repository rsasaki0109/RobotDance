"""Motion embedding / retrieval / Motion Map（v0, 特徴量ベース）。

RD-MIR を固定長ベクトルに符号化し、類似動作検索・重複除去・2D マップを可能にする。

⚠️ v0 は **学習済み encoder ではなく決定的な手作り特徴量**。root-relative・scale 正規化・
per-frame yaw 整列で位置/向き/スケール不変にし、joint 分布と運動エネルギーを集約する。
学習 encoder（masked modeling / contrastive）は Phase 3 でこの interface を差し替える。
"""

from __future__ import annotations

import numpy as np

from robotdance_core.rd_mir import RdMir
from robotdance_core.skeleton import NUM_JOINTS, index_of

_EPS = 1e-8
_PELVIS = index_of("pelvis")
_HEAD = index_of("head")
_LHIP = index_of("left_hip")
_RHIP = index_of("right_hip")


def _yaw_align(rel: np.ndarray) -> np.ndarray:
    """各フレームを z 軸回りに回し、腰の左右ベクトルを +y に揃える（向き不変化）。"""
    hip = rel[:, _LHIP, :] - rel[:, _RHIP, :]  # [T,3] 右→左
    ang = np.arctan2(hip[:, 0], hip[:, 1])  # +y に向ける回転角
    c, s = np.cos(ang), np.sin(ang)
    out = rel.copy()
    x, y = rel[..., 0], rel[..., 1]
    out[..., 0] = c[:, None] * x - s[:, None] * y
    out[..., 1] = s[:, None] * x + c[:, None] * y
    return out


def embed(mir: RdMir) -> np.ndarray:
    """RD-MIR を固定長 motion embedding（生特徴ベクトル）に符号化する。"""
    kps = mir.keypoints_3d_array()  # [T, J, 3]
    rel = kps - kps[:, _PELVIS:_PELVIS + 1, :]  # root-relative

    # スケール正規化（pelvis→head 距離の時間平均）。
    scale = float(np.linalg.norm(rel[:, _HEAD], axis=1).mean())
    rel = rel / max(scale, _EPS)
    rel = _yaw_align(rel)

    fps = mir.fps
    vel = np.diff(rel, axis=0) * fps if rel.shape[0] > 1 else np.zeros((1, NUM_JOINTS, 3))
    speed = np.linalg.norm(vel, axis=2)  # [T-1, J]

    # 接地比率。
    contacts = mir.contacts or {}
    cr = [
        float(np.mean(np.asarray(contacts.get(f"{s}_foot", [0]), dtype=float)))
        for s in ("left", "right")
    ]

    features = np.concatenate([
        rel.mean(axis=0).reshape(-1),   # 平均ポーズ分布 (J*3)
        rel.std(axis=0).reshape(-1),    # ポーズの広がり (J*3)
        speed.mean(axis=0),             # joint ごとの平均速度 (J)
        np.array(cr),                   # 接地比率 (2)
        np.array([
            float(speed.mean()),                         # 全体運動エネルギー
            float(kps[:, _PELVIS, 2].std() / max(scale, _EPS)),  # 上下バウンス
        ]),
    ])
    return features.astype(np.float64)


EMBEDDING_DIM = NUM_JOINTS * 3 * 2 + NUM_JOINTS + 2 + 2


class MotionIndex:
    """motion embedding の索引。類似検索・重複検出・2D 射影を提供する。"""

    def __init__(self) -> None:
        self.ids: list[str] = []
        self._raw: list[np.ndarray] = []
        self._mean: np.ndarray | None = None
        self._std: np.ndarray | None = None

    def add(self, motion_id: str, embedding: np.ndarray) -> None:
        self.ids.append(motion_id)
        self._raw.append(np.asarray(embedding, dtype=np.float64))
        self._mean = None  # 統計を無効化（次回 query 時に再計算）

    def add_mir(self, mir: RdMir) -> None:
        self.add(mir.motion_id, embed(mir))

    def _matrix(self) -> np.ndarray:
        return np.stack(self._raw) if self._raw else np.zeros((0, EMBEDDING_DIM))

    def _standardized(self) -> np.ndarray:
        """全 embedding を per-feature z-score 化（距離を安定化）。"""
        m = self._matrix()
        if self._mean is None:
            self._mean = m.mean(axis=0)
            self._std = m.std(axis=0)
            self._std[self._std < _EPS] = 1.0
        return (m - self._mean) / self._std

    def query(self, embedding: np.ndarray, k: int = 5) -> list[tuple[str, float]]:
        """embedding に近い順に (id, cosine 類似度) を最大 k 件返す。"""
        if not self._raw:
            return []
        z = self._standardized()
        q = (np.asarray(embedding, dtype=np.float64) - self._mean) / self._std
        zn = z / np.maximum(np.linalg.norm(z, axis=1, keepdims=True), _EPS)
        qn = q / max(np.linalg.norm(q), _EPS)
        sims = zn @ qn
        order = np.argsort(sims)[::-1][:k]
        return [(self.ids[i], float(sims[i])) for i in order]

    def duplicates(self, threshold: float = 0.98) -> list[tuple[str, str, float]]:
        """cosine 類似度が threshold 以上のペア（near-duplicate）を返す。"""
        if len(self._raw) < 2:
            return []
        z = self._standardized()
        zn = z / np.maximum(np.linalg.norm(z, axis=1, keepdims=True), _EPS)
        sims = zn @ zn.T
        pairs = []
        for i in range(len(self.ids)):
            for j in range(i + 1, len(self.ids)):
                if sims[i, j] >= threshold:
                    pairs.append((self.ids[i], self.ids[j], float(sims[i, j])))
        return pairs

    def project_2d(self) -> np.ndarray:
        """standardized embedding を PCA で 2D に射影する（numpy SVD, 決定的）。[N, 2]。"""
        z = self._standardized()
        if z.shape[0] < 2:
            return np.zeros((z.shape[0], 2))
        centered = z - z.mean(axis=0)
        _, _, vt = np.linalg.svd(centered, full_matrices=False)
        return centered @ vt[:2].T
