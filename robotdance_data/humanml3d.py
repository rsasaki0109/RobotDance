"""HumanML3D (text-motion) → canonical RD-MIR ローダ（skeleton-first, v0）。

HumanML3D は AMASS + HumanAct12 のモーションに **各モーション 3 件の自然文記述**を付けた
text-motion データセット。配布形式は前処理済みの joint 位置（`new_joints/<id>.npy`,
[T, 22, 3]）+ テキスト（`texts/<id>.txt`）。本ローダはその joint 位置を canonical 19-joint に
変換し、キャプションを `semantics` に格納して RD-MIR にする（SMPL model file は使わない）。

⚠️ ライセンス: HumanML3D は AMASS 由来で研究用途中心。既定 license_state は "research_only"。
.npy / テキストは repo に含めない（利用者が各自取得）。v0 は frame 正規化が近似（HumanML3D の
前処理 frame を SMPL frame とみなす）で、betas/shape は未使用。
"""

from __future__ import annotations

from pathlib import Path
from typing import Optional

import numpy as np

from robotdance_core.rd_mir import LicenseState, RdMir, Skeleton
from robotdance_core.skeleton import JOINT_NAMES, PARENTS, index_of

from .smpl import smpl_joints_to_canonical


def humanml3d_to_mir(
    joints: np.ndarray,
    texts: list[str] | str | None = None,
    *,
    fps: float = 20.0,
    license_state: LicenseState = "research_only",
    motion_id: Optional[str] = None,
    ground_align: bool = True,
) -> RdMir:
    """HumanML3D の joint 位置 [T, 22, 3] と記述文から canonical RD-MIR を生成する。

    HumanML3D の標準 fps は 20。texts の先頭をプライマリ caption（action_label）に使う。
    """
    joints = np.asarray(joints, dtype=np.float64)
    if joints.ndim != 3 or joints.shape[1] < 22:
        raise ValueError(f"HumanML3D joints は [T, >=22, 3] が必要: {joints.shape}")
    kps = smpl_joints_to_canonical(joints[:, :22, :])  # [T, 19, 3]
    if ground_align:
        kps[:, :, 2] -= kps[:, [index_of("left_ankle"), index_of("right_ankle")], 2].min()

    if isinstance(texts, str):
        texts = [texts]
    caption = (texts[0] if texts else "unknown").strip() or "unknown"
    n = kps.shape[0]
    return RdMir(
        motion_id=motion_id or "rdmir-humanml3d",
        source_ref={"dataset_name": "humanml3d", "extractor": "smpl_joints"},
        license_state=license_state,
        fps=float(fps),
        duration=float(n / fps),
        world_frame={"up_axis": "z", "forward_axis": "x", "handedness": "right"},
        skeleton=Skeleton(joint_names=list(JOINT_NAMES), parents=list(PARENTS)),
        root_trajectory={"position": kps[:, index_of("pelvis"), :].tolist()},
        keypoints_3d=kps.tolist(),
        contacts=_estimate_contacts(kps),
        privacy_flags={"synthetic": False},
        extractor_versions={"source": "humanml3d", "adapter": "robotdance.data.humanml3d.v0"},
        semantics={"action_label": caption, "captions": list(texts or []),
                   "source_dataset": "humanml3d"},
    )


def load_humanml3d(
    joints_path: str | Path, text_path: str | Path | None = None, *,
    fps: float = 20.0, license_state: LicenseState = "research_only",
    motion_id: Optional[str] = None,
) -> RdMir:
    """HumanML3D の `new_joints/<id>.npy`（+ `texts/<id>.txt`）→ RD-MIR。"""
    joints_path = Path(joints_path)
    joints = np.load(joints_path)
    texts: list[str] = []
    if text_path is not None and Path(text_path).exists():
        # HumanML3D の texts は "caption#tokens#start#end" 形式。caption だけ取る。
        for line in Path(text_path).read_text(encoding="utf-8").splitlines():
            line = line.strip()
            if line:
                texts.append(line.split("#", 1)[0].strip())
    return humanml3d_to_mir(joints, texts, fps=fps, license_state=license_state,
                            motion_id=motion_id or f"rdmir-humanml3d-{joints_path.stem}")


def _estimate_contacts(kps: np.ndarray) -> dict[str, list[bool]]:
    out: dict[str, list[bool]] = {}
    for side in ("left", "right"):
        z = kps[:, index_of(f"{side}_ankle"), 2]
        out[f"{side}_foot"] = (z < float(z.min()) + 0.07).tolist()
    return out
