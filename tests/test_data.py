"""dataset ローダ（AMASS / SMPL FK）と RD-Manifest license firewall の検証。

実 AMASS は登録制でここでは使えないため、本物の .npz 形式を模した合成フィクスチャでテストする。
"""

from __future__ import annotations

import json
from pathlib import Path

import jsonschema
import numpy as np

from robotdance_core.skeleton import NUM_JOINTS, index_of
from robotdance_data import dataset as ds
from robotdance_data import manifest as mf
from robotdance_data.amass import load_amass_npz
from robotdance_data.smpl import smpl_poses_to_canonical

_ROOT = Path(__file__).resolve().parent.parent


def _write_fake_amass(path: Path, *, frames: int = 120, fps: float = 120.0) -> Path:
    """本物の AMASS npz 形式を模した合成データ（SMPL-H poses[156] + trans + framerate）。"""
    poses = np.zeros((frames, 156))
    t = np.linspace(0, 4 * np.pi, frames)
    poses[:, 16 * 3 + 2] = 0.7 * np.sin(t)   # l_shoulder
    poses[:, 17 * 3 + 2] = -0.7 * np.sin(t)  # r_shoulder
    np.savez(path, poses=poses, trans=np.zeros((frames, 3)), mocap_framerate=fps)
    return path


def _write_fake_aist(path: Path, *, frames: int = 120, phase: float = 0.0) -> Path:
    """本物の AIST++ pkl 形式を模した合成データ（SMPL poses[72] + trans + scaling）。"""
    import pickle

    poses = np.zeros((frames, 72))
    t = np.linspace(0, 4 * np.pi, frames) + phase
    poses[:, 16 * 3 + 2] = 0.7 * np.sin(t)
    poses[:, 17 * 3 + 2] = -0.7 * np.sin(t)
    with path.open("wb") as f:
        pickle.dump({"smpl_poses": poses, "smpl_trans": np.zeros((frames, 3)),
                     "smpl_scaling": np.array([100.0])}, f)
    return path


# --- SMPL FK / canonical mapping ---

def test_smpl_rest_pose_orientation() -> None:
    c = smpl_poses_to_canonical(np.zeros((1, 22, 3)))[0]
    assert c[index_of("head")][2] > c[index_of("pelvis")][2]          # z-up
    assert c[index_of("left_ankle")][2] < c[index_of("pelvis")][2]
    assert c[index_of("left_shoulder")][1] > c[index_of("right_shoulder")][1]  # y-left


# --- AMASS loader ---

def test_amass_loader_downsamples_and_validates(tmp_path: Path) -> None:
    npz = _write_fake_amass(tmp_path / "ACCAD_walk_poses.npz", frames=120, fps=120.0)
    mir = load_amass_npz(npz, license_state="research_only", target_fps=30.0)
    # 120fps → ~30fps に間引かれる。
    assert abs(mir.fps - 30.0) < 1.0
    assert mir.keypoints_3d_array().shape[1] == NUM_JOINTS
    assert mir.license_state == "research_only"
    schema = json.loads((_ROOT / "specs" / "rd-mir" / "rd-mir.schema.json").read_text("utf-8"))
    jsonschema.Draft202012Validator(schema).validate(mir.to_dict())


def test_amass_flows_into_retarget(tmp_path: Path) -> None:
    from robotdance_retarget.kinematic import retarget
    from robotdance_unitree import get_morphology

    npz = _write_fake_amass(tmp_path / "amass.npz")
    mir = load_amass_npz(npz)
    motion = retarget(mir, get_morphology("unitree_g1"))
    assert motion.keypoints_3d_array().shape[1] == NUM_JOINTS


# --- AIST++ loader ---

def test_aist_loader_downsamples_and_validates(tmp_path: Path) -> None:
    from robotdance_data.aist import load_aist_pkl

    pkl = _write_fake_aist(tmp_path / "gBR_sBM.pkl", frames=120)
    mir = load_aist_pkl(pkl, src_fps=60.0, target_fps=30.0)
    assert abs(mir.fps - 30.0) < 1.0           # 60fps → ~30fps
    assert mir.keypoints_3d_array().shape[1] == NUM_JOINTS
    assert mir.semantics["source_dataset"] == "aist++"
    schema = json.loads((_ROOT / "specs" / "rd-mir" / "rd-mir.schema.json").read_text("utf-8"))
    jsonschema.Draft202012Validator(schema).validate(mir.to_dict())


# --- license firewall ---

def test_firewall_blocks_unknown_license() -> None:
    d = mf.evaluate({"license_declared": "unknown", "derived_motion_allowed": True})
    assert d.can_export_derived is False
    assert d.license_state == "unknown"


def test_firewall_blocks_derived_not_allowed() -> None:
    d = mf.evaluate({"license_declared": "custom", "derived_motion_allowed": False})
    assert d.can_export_derived is False


def test_firewall_allows_cc_and_sets_state() -> None:
    d = mf.evaluate({
        "license_declared": "creativeCommon", "derived_motion_allowed": True,
        "training_allowed": True,
    })
    assert d.can_export_derived is True
    assert d.license_state == "trainable"


# --- dataset build + Data Bill of Materials ---

def test_build_dataset_respects_firewall(tmp_path: Path) -> None:
    _write_fake_amass(tmp_path / "walk.npz")
    manifests = [
        {  # 公開可（CC, derived 可）
            "manifest_version": "0", "clip_id": "ok_clip", "source_type": "dataset",
            "source_uri": "dataset://amass/walk.npz", "license_declared": "creativeCommon",
            "derived_motion_allowed": True, "training_allowed": True,
            "rebuild_method": "manual_download", "status": "active",
        },
        {  # firewall で withheld（license unknown）
            "manifest_version": "0", "clip_id": "blocked_clip", "source_type": "dataset",
            "source_uri": "dataset://amass/walk.npz", "license_declared": "unknown",
            "derived_motion_allowed": True, "rebuild_method": "manual_download", "status": "active",
        },
    ]
    out = tmp_path / "build"
    report = ds.build_dataset(manifests, data_root=tmp_path, out_dir=out)
    assert report["exported"] == 1
    assert report["withheld"] == 1
    assert (out / "ok_clip.rdmir.json").exists()
    assert not (out / "blocked_clip.rdmir.json").exists()
    # Data Bill of Materials が出る。
    card = (out / "DATA_CARD.md").read_text("utf-8")
    assert "Data Bill of Materials" in card and "ok_clip" in card and "blocked_clip" in card
    bom = {r["clip_id"]: r for r in report["bill_of_materials"]}
    assert bom["ok_clip"]["exported"] is True
    assert bom["blocked_clip"]["exported"] is False


def test_build_dataset_dedupe(tmp_path: Path) -> None:
    """同一振付の clip を motion embedding で 1 本に集約する。"""
    _write_fake_aist(tmp_path / "orig.pkl", phase=0.0)
    _write_fake_aist(tmp_path / "dup.pkl", phase=0.0)       # orig と同一
    _write_fake_aist(tmp_path / "diff.pkl", phase=1.7)      # 別振付

    def m(clip: str) -> dict:
        return {
            "manifest_version": "0", "clip_id": clip, "source_type": "dataset",
            "source_uri": f"dataset://aist/{clip}.pkl", "license_declared": "creativeCommon",
            "derived_motion_allowed": True, "training_allowed": True,
            "rebuild_method": "manual_download", "status": "active",
        }

    out = tmp_path / "build"
    report = ds.build_dataset([m("orig"), m("dup"), m("diff")],
                              data_root=tmp_path, out_dir=out, dedupe=True)
    assert report["exported"] == 2          # orig + diff（dup は除去）
    bom = {r["clip_id"]: r for r in report["bill_of_materials"]}
    assert bom["dup"]["exported"] is False
    assert "near-duplicate" in bom["dup"]["reason"]
    assert not (out / "dup.rdmir.json").exists()
    assert (out / "diff.rdmir.json").exists()
