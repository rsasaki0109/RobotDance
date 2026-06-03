"""AIST++ (.pkl, SMPL pose params) → canonical RD-MIR ローダ（skeleton-first, v0）。

AIST++ は AIST Dance DB 由来の multi-view dance motion。motion/*.pkl は SMPL の
smpl_poses [N,72]・smpl_trans [N,3]・smpl_scaling を持つ。SMPL body の先頭 22 joint を
FK して canonical 19 へ変換する（AMASS と同じ skeleton-first 経路、SMPL model file は不要）。

⚠️ ライセンス: AIST++ の annotations は CC BY 4.0 だが、元動画・音楽は AIST Dance DB の Terms に従う。
既定 license_state は "research_only"。.pkl / 動画は repo に含めない。利用者が各自取得する。
"""

from __future__ import annotations

import pickle
from pathlib import Path
from typing import Optional

import numpy as np

from robotdance_core.rd_mir import LicenseState, RdMir, Skeleton
from robotdance_core.skeleton import JOINT_NAMES, PARENTS, index_of

from .smpl import smpl_poses_to_canonical


def load_aist_pkl(
    pkl_path: str | Path,
    *,
    license_state: LicenseState = "research_only",
    src_fps: float = 60.0,
    target_fps: Optional[float] = 30.0,
    motion_id: Optional[str] = None,
) -> RdMir:
    """AIST++ motion .pkl から canonical RD-MIR を生成する（AIST++ は 60fps）。"""
    path = Path(pkl_path)
    with path.open("rb") as f:
        data = pickle.load(f)  # noqa: S301 — 利用者持ち込みの AIST++ 公式形式
    if "smpl_poses" not in data:
        raise ValueError(f"AIST++ pkl に 'smpl_poses' がありません: {path}")
    poses = np.asarray(data["smpl_poses"], dtype=np.float64)  # [N, 72]
    trans = np.asarray(data["smpl_trans"], dtype=np.float64) if "smpl_trans" in data else None
    if trans is not None and "smpl_scaling" in data:
        scaling = float(np.asarray(data["smpl_scaling"]).reshape(-1)[0])
        if scaling:
            trans = trans / scaling  # メートルスケールへ

    body = poses[:, :66].reshape(poses.shape[0], 22, 3)

    fps = src_fps
    if target_fps and src_fps > target_fps * 1.3:
        stride = max(1, int(round(src_fps / target_fps)))
        body = body[::stride]
        trans = trans[::stride] if trans is not None else None
        fps = src_fps / stride

    kps = smpl_poses_to_canonical(body, trans)
    kps[:, :, 2] -= kps[:, [index_of("left_ankle"), index_of("right_ankle")], 2].min()

    n = kps.shape[0]
    contacts = {
        f"{s}_foot": (kps[:, index_of(f"{s}_ankle"), 2]
                      < float(kps[:, index_of(f"{s}_ankle"), 2].min()) + 0.07).tolist()
        for s in ("left", "right")
    }
    return RdMir(
        motion_id=motion_id or f"rdmir-aist-{path.stem}",
        source_ref={"dataset_name": "aist++", "local_path": str(path), "extractor": "smpl_fk"},
        license_state=license_state,
        fps=float(fps),
        duration=float(n / fps),
        skeleton=Skeleton(joint_names=list(JOINT_NAMES), parents=list(PARENTS)),
        root_trajectory={"position": kps[:, index_of("pelvis"), :].tolist()},
        keypoints_3d=kps.tolist(),
        contacts=contacts,
        privacy_flags={"synthetic": False},
        extractor_versions={"source": "aist++_smpl", "adapter": "robotdance.data.aist.v0"},
        semantics={"action_label": "dance", "source_dataset": "aist++"},
    )
