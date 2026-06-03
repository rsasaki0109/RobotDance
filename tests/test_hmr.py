"""HMR adapter（4DHumans/GVHMR SMPL 出力 → canonical RD-MIR）の検証。

skeleton-first（SMPL FK）なので torch/モデル weight 不要。numpy/scipy のみで CI でも走る。
"""

from __future__ import annotations

import json
from pathlib import Path

import jsonschema
import numpy as np
from scipy.spatial.transform import Rotation as Rot

from robotdance_core.rd_mir import RdMir
from robotdance_perception.hmr import (
    from_4dhumans,
    from_gvhmr,
    hmr_smpl_to_mir,
    load_hmr_npz,
)


def _synthetic_smpl(t: int = 30):
    """合成 SMPL 列（global_orient [T,3] axis-angle, body_pose [T,21,3], transl [T,3]）。"""
    go = np.zeros((t, 3))
    go[:, 2] = 0.1 * np.sin(np.linspace(0, 6, t))
    base = np.random.RandomState(1).randn(21, 3)
    bp = 0.15 * np.sin(np.linspace(0, 4, t))[:, None, None] * np.ones((t, 21, 3)) * base[None]
    transl = np.cumsum(0.01 * np.ones((t, 3)) * [1, 0, 0], axis=0)
    return go, bp, transl


_SCHEMA = json.loads(
    (Path(__file__).resolve().parent.parent / "specs" / "rd-mir" / "rd-mir.schema.json")
    .read_text(encoding="utf-8")
)


def _valid(mir: RdMir) -> None:
    jsonschema.Draft202012Validator(_SCHEMA).validate(mir.to_dict())


def test_core_axis_angle_to_mir() -> None:
    go, bp, transl = _synthetic_smpl()
    mir = hmr_smpl_to_mir(go, bp.reshape(len(go), 63), transl, fps=30.0, source="test")
    assert mir.num_frames == len(go)
    assert mir.license_state == "unknown"  # in-the-wild 由来は既定 unknown
    assert np.array(mir.keypoints_3d).shape == (len(go), 19, 3)
    _valid(mir)


def test_rotmat_matches_axis_angle() -> None:
    """rotation-matrix 入力（4DHumans 形式）と axis-angle 入力が一致する。"""
    go, bp, transl = _synthetic_smpl()
    t = len(go)
    mir_aa = hmr_smpl_to_mir(go, bp.reshape(t, 63), transl, source="aa", smooth=False)
    go_r = Rot.from_rotvec(go).as_matrix()
    bp_r = Rot.from_rotvec(bp.reshape(-1, 3)).as_matrix().reshape(t, 21, 3, 3)
    mir_r = hmr_smpl_to_mir(go_r, bp_r, transl, source="rotmat", smooth=False)
    assert np.allclose(np.array(mir_aa.keypoints_3d), np.array(mir_r.keypoints_3d), atol=1e-9)


def test_from_gvhmr_dict() -> None:
    go, bp, transl = _synthetic_smpl()
    t = len(go)
    result = {"smpl_params_global": {"global_orient": go, "body_pose": bp.reshape(t, 63),
                                     "transl": transl}}
    mir = from_gvhmr(result)
    assert mir.extractor_versions["hmr"] == "gvhmr"
    assert mir.num_frames == t
    _valid(mir)


def test_from_4dhumans_dict_rotmat_23joints() -> None:
    """4DHumans の rotmat・23 body joint 出力を受理し、先頭 21 を使う。"""
    go, bp, transl = _synthetic_smpl()
    t = len(go)
    go_r = Rot.from_rotvec(go).as_matrix()
    bp23 = np.concatenate([bp.reshape(t, 21, 3), np.zeros((t, 2, 3))], axis=1)
    bp23_r = Rot.from_rotvec(bp23.reshape(-1, 3)).as_matrix().reshape(t, 23, 3, 3)
    result = {"smpl": {"global_orient": go_r, "body_pose": bp23_r}, "pred_cam_t": transl}
    mir = from_4dhumans(result)
    assert mir.extractor_versions["hmr"] == "4dhumans"
    assert mir.num_frames == t
    _valid(mir)


def test_load_hmr_npz_roundtrip(tmp_path: Path) -> None:
    go, bp, transl = _synthetic_smpl()
    t = len(go)
    p = tmp_path / "hmr.npz"
    np.savez(p, global_orient=go, body_pose=bp.reshape(t, 63), transl=transl,
             fps=30.0, source="gvhmr")
    mir = load_hmr_npz(p)
    assert mir.num_frames == t
    assert mir.source_ref["local_path"] == str(p)
    _valid(mir)


def test_hmr_output_retargets() -> None:
    """HMR 由来 RD-MIR が retarget パイプラインに流せる（schema 互換の実証）。"""
    from robotdance_retarget.kinematic import retarget
    from robotdance_unitree import get_morphology

    go, bp, transl = _synthetic_smpl()
    mir = hmr_smpl_to_mir(go, bp.reshape(len(go), 63), transl, source="test")
    motion = retarget(mir, get_morphology("unitree_g1"))
    assert motion.keypoints_3d is not None
    assert len(motion.keypoints_3d) == len(go)
