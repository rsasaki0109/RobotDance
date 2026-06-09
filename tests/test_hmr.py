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
    from_dict,
    from_gvhmr,
    hmr_smpl_to_mir,
    load_hmr_file,
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


# --- betas (shape conditioning) ---

def _rest_pose():
    t = 20
    return np.zeros((t, 3)), np.zeros((t, 63))


def test_betas_scale_skeleton_height() -> None:
    """betas[0]>0 で骨格が高くなる（shape-conditioning, v0 近似）。"""
    go, bp = _rest_pose()
    kp0 = np.array(hmr_smpl_to_mir(go, bp).keypoints_3d)
    kp_tall = np.array(hmr_smpl_to_mir(go, bp, betas=np.array([3.0] + [0] * 9)).keypoints_3d)
    h0 = kp0[:, :, 2].max() - kp0[:, :, 2].min()
    ht = kp_tall[:, :, 2].max() - kp_tall[:, :, 2].min()
    assert ht > h0
    # betas が付くと quality に記録される。
    assert hmr_smpl_to_mir(go, bp, betas=np.array([1.0])).quality_metrics["shape_conditioned"] is True
    assert hmr_smpl_to_mir(go, bp).quality_metrics["shape_conditioned"] is False


def test_per_frame_betas_averaged() -> None:
    go, bp = _rest_pose()
    mir = hmr_smpl_to_mir(go, bp, betas=np.tile([2.0] + [0] * 9, (len(go), 1)))  # [T,10]
    assert mir.num_frames == len(go)
    _valid(mir)


# --- native loaders (.pkl/.npz/dict) ---

def test_from_dict_dispatches_gvhmr() -> None:
    go, bp = _rest_pose()
    t = len(go)
    result = {"smpl_params_global": {"global_orient": go, "body_pose": bp,
                                     "transl": np.zeros((t, 3)), "betas": np.full((t, 10), 1.5)}}
    mir = from_dict(result, source="ignored")  # source は gvhmr 分岐で無視される
    assert mir.extractor_versions["hmr"] == "gvhmr"
    assert mir.quality_metrics["shape_conditioned"] is True


def test_load_hmr_file_pickle(tmp_path: Path) -> None:
    import pickle

    go, bp = _rest_pose()
    t = len(go)
    result = {"smpl_params_global": {"global_orient": go, "body_pose": bp,
                                     "betas": np.full((t, 10), 2.0)}}
    p = tmp_path / "gvhmr.pkl"
    with open(p, "wb") as f:
        pickle.dump(result, f)
    mir = load_hmr_file(p)
    assert mir.extractor_versions["hmr"] == "gvhmr"
    assert mir.quality_metrics["shape_conditioned"] is True
    _valid(mir)


def test_load_hmr_file_npz_with_betas(tmp_path: Path) -> None:
    go, bp = _rest_pose()
    t = len(go)
    p = tmp_path / "h.npz"
    np.savez(p, global_orient=go, body_pose=bp, betas=np.full((t, 10), 2.0), source="gvhmr")
    mir = load_hmr_file(p)
    assert mir.quality_metrics["shape_conditioned"] is True
    _valid(mir)


def test_cli_import_hmr_then_motion_doctor_gvhmr_workflow(tmp_path: Path) -> None:
    """v0.94 で登録した gvhmr backend の文書化ワークフローを CLI で通す:
    外部ツール(GVHMR)出力を模した .npy → `import-hmr` → RD-MIR → `motion-doctor`。
    """
    from robotdance_core.cli import main

    go, bp, transl = _synthetic_smpl()
    t = len(go)
    # GVHMR の native 出力（dict）を .npy で保存（load_hmr_file が dict を判別）。
    result = {"smpl_params_global": {"global_orient": go, "body_pose": bp.reshape(t, 63),
                                     "transl": transl}}
    smpl = tmp_path / "gvhmr_out.npy"
    np.save(smpl, np.array(result, dtype=object), allow_pickle=True)

    out = tmp_path / "from_gvhmr.rdmir.json"
    assert main(["import-hmr", str(smpl), "--source", "gvhmr", "-o", str(out)]) == 0
    assert out.exists()

    mir = RdMir.load(out)
    assert mir.num_frames == t
    assert mir.extractor_versions["hmr"] == "gvhmr"
    _valid(mir)

    # 取り込んだ RD-MIR は motion-doctor を通せる（exit 0=健全 / 1=warn のどちらか）。
    assert main(["motion-doctor", str(out)]) in (0, 1)


def test_cli_extract_gvhmr_unavailable_shows_install_hint(tmp_path: Path, capsys) -> None:
    """extract --backend gvhmr は未導入時に install 案内（import-hmr 代替付き）。"""
    from robotdance_core.cli import main
    from robotdance_perception.gvhmr_backend import gvhmr_available

    if gvhmr_available():
        pytest.skip("GVHMR 導入済み")
    rc = main(["extract", str(tmp_path / "nope.mp4"), "--backend", "gvhmr"])
    assert rc == 1
    assert "import-hmr" in capsys.readouterr().out
