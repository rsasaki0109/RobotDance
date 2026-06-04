"""HumanML3D / BABEL adapter（§4.1, text-motion データ入口）の検証。

skeleton-first（SMPL FK 再利用）なので torch/モデル不要。numpy のみで CI でも走る。
"""

from __future__ import annotations

import json
from pathlib import Path

import jsonschema
import numpy as np

from robotdance_core.rd_mir import RdMir
from robotdance_core.skeleton import NUM_JOINTS
from robotdance_data.babel import babel_entry_to_mir, load_babel
from robotdance_data.humanml3d import humanml3d_to_mir, load_humanml3d
from robotdance_data.motionx import _extract_body, load_motionx, motionx_to_mir

_SCHEMA = json.loads(
    (Path(__file__).resolve().parent.parent / "specs" / "rd-mir" / "rd-mir.schema.json")
    .read_text(encoding="utf-8")
)


def _valid(mir: RdMir) -> None:
    jsonschema.Draft202012Validator(_SCHEMA).validate(mir.to_dict())


def _synthetic_joints(t: int = 40):
    rng = np.random.default_rng(0)
    j = rng.normal(0, 0.3, size=(t, 22, 3))
    j[:, 0] = 0.0          # pelvis at origin
    j[:, :, 1] += 1.0      # SMPL y-up
    return j


# --- HumanML3D ---

def test_humanml3d_core_to_mir() -> None:
    j = _synthetic_joints()
    mir = humanml3d_to_mir(j, ["a person walks forward", "someone strides"], fps=20.0)
    assert mir.num_frames == len(j)
    assert np.array(mir.keypoints_3d).shape == (len(j), NUM_JOINTS, 3)
    assert mir.fps == 20.0
    assert mir.license_state == "research_only"
    assert mir.semantics["action_label"] == "a person walks forward"
    assert mir.semantics["source_dataset"] == "humanml3d"
    _valid(mir)


def test_humanml3d_requires_22_joints() -> None:
    import pytest

    with pytest.raises(ValueError, match=r"\[T, >=22, 3\]"):
        humanml3d_to_mir(np.zeros((10, 12, 3)))


def test_load_humanml3d_parses_text_format(tmp_path: Path) -> None:
    """HumanML3D の 'caption#tokens#start#end' 形式から caption だけ取り出す。"""
    np.save(tmp_path / "000.npy", _synthetic_joints())
    (tmp_path / "000.txt").write_text(
        "a person waves#a/DET person/NOUN waves/VERB#0.0#2.0\nwaving hello\n", encoding="utf-8")
    mir = load_humanml3d(tmp_path / "000.npy", tmp_path / "000.txt")
    assert mir.semantics["action_label"] == "a person waves"   # # 以降は除去
    assert mir.semantics["captions"] == ["a person waves", "waving hello"]
    _valid(mir)


# --- BABEL ---

def _write_amass(root: Path, rel: str) -> None:
    rng = np.random.default_rng(1)
    (root / rel).parent.mkdir(parents=True, exist_ok=True)
    np.savez(root / rel, poses=rng.normal(0, 0.1, size=(30, 66)),
             trans=np.zeros((30, 3)), mocap_framerate=120.0)


def _babel_entry(rel: str):
    return {
        "feat_p": rel,
        "babel_sid": "12345",
        "dur": 0.25,
        "seq_ann": {"labels": [{"raw_label": "walk", "act_cat": ["walk", "locomotion"]}]},
        "frame_ann": {"labels": [{"act_cat": ["walk"], "start_t": 0.0, "end_t": 0.25}]},
    }


def test_babel_entry_attaches_labels(tmp_path: Path) -> None:
    rel = "CMU/01/01_01_poses.npz"
    _write_amass(tmp_path, rel)
    mir = babel_entry_to_mir(_babel_entry(rel), tmp_path)
    assert mir.semantics["action_label"] == "walk"
    assert mir.semantics["babel_labels"] == ["walk", "locomotion"]
    # 構造化セグメント（label 必須）。
    assert len(mir.semantics["segments"]) == 1
    assert mir.semantics["segments"][0]["label"] == "walk"
    assert mir.semantics["segments"][0]["start_t"] == 0.0
    assert mir.semantics["babel_sid"] == "12345"
    assert mir.source_ref["babel_sid"] == "12345"
    _valid(mir)


def test_load_babel_skips_missing_amass(tmp_path: Path) -> None:
    rel = "CMU/01/01_01_poses.npz"
    _write_amass(tmp_path, rel)
    entries = {
        "12345": _babel_entry(rel),
        "99999": {"feat_p": "MISSING/x.npz", "seq_ann": {"labels": [{"raw_label": "jump"}]}},
    }
    bj = tmp_path / "babel.json"
    bj.write_text(json.dumps(entries), encoding="utf-8")
    out = load_babel(bj, tmp_path)
    # AMASS が存在する 1 件だけ。
    assert len(out) == 1
    assert out[0].semantics["action_label"] == "walk"


# --- Motion-X ---

def test_motionx_322dim_extracts_body() -> None:
    """322 次元表現から root_orient+pose_body(66) と trans を取り出す。"""
    rng = np.random.default_rng(0)
    t = 40
    m = rng.normal(0, 0.1, size=(t, 322))
    m[:, 309:312] = np.cumsum(0.01 * np.ones((t, 3)), axis=0)
    pose, trans, betas = _extract_body(m)
    assert pose.shape == (t, 22, 3)
    assert trans is not None and trans.shape == (t, 3)
    assert betas is not None and betas.shape == (10,)
    # root_orient(0:3) + pose_body(3:66) を使っている。
    assert np.allclose(pose.reshape(t, 66)[:, :66], m[:, :66])
    assert np.allclose(trans, m[:, 309:312])


def test_motionx_to_mir_and_schema() -> None:
    rng = np.random.default_rng(1)
    m = rng.normal(0, 0.1, size=(30, 322))
    mir = motionx_to_mir(m, ["a person dances", "dancing"], fps=30.0)
    assert mir.num_frames == 30
    assert np.array(mir.keypoints_3d).shape == (30, NUM_JOINTS, 3)
    assert mir.license_state == "research_only"
    assert mir.semantics["action_label"] == "a person dances"
    assert mir.semantics["source_dataset"] == "motionx"
    _valid(mir)


def test_motionx_accepts_66dim_and_rejects_short() -> None:
    rng = np.random.default_rng(2)
    mir = motionx_to_mir(rng.normal(0, 0.1, size=(10, 66)), "walk")
    assert mir.num_frames == 10
    import pytest

    with pytest.raises(ValueError, match="次元が不足"):
        motionx_to_mir(rng.normal(0, 0.1, size=(10, 30)))


def test_load_motionx(tmp_path: Path) -> None:
    rng = np.random.default_rng(3)
    np.save(tmp_path / "000.npy", rng.normal(0, 0.1, size=(20, 322)))
    (tmp_path / "000.txt").write_text("a person jumps\nleaping\n", encoding="utf-8")
    mir = load_motionx(tmp_path / "000.npy", tmp_path / "000.txt")
    assert mir.semantics["action_label"] == "a person jumps"
    assert mir.semantics["captions"] == ["a person jumps", "leaping"]
    _valid(mir)


def test_babel_output_retargets(tmp_path: Path) -> None:
    """BABEL 由来 RD-MIR が retarget パイプラインに流せる。"""
    from robotdance_retarget.kinematic import retarget
    from robotdance_unitree import get_morphology

    rel = "CMU/01/01_01_poses.npz"
    _write_amass(tmp_path, rel)
    mir = babel_entry_to_mir(_babel_entry(rel), tmp_path)
    motion = retarget(mir, get_morphology("unitree_g1"))
    assert motion.keypoints_3d is not None
    assert len(motion.keypoints_3d) == mir.num_frames
