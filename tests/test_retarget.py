"""G1 kinematic retarget の縦スライスを検証する。"""

from __future__ import annotations

import json
from pathlib import Path

import jsonschema
import pytest

from robotdance_core.rd_motion import RdMotion
from robotdance_core.skeleton import FOOT_JOINTS, NUM_JOINTS
from robotdance_core.synthetic import generate_dance
from robotdance_retarget.kinematic import retarget_to_g1
from robotdance_unitree import g1

_ROOT = Path(__file__).resolve().parent.parent


def _schema(name: str) -> dict:
    return json.loads((_ROOT / "specs" / name).read_text(encoding="utf-8"))


def test_g1_embodiment_conforms() -> None:
    jsonschema.Draft202012Validator(
        _schema("rd-embodiment/rd-embodiment.schema.json")
    ).validate(g1.embodiment_dict())


def test_retarget_shapes_and_schema() -> None:
    mir = generate_dance(duration=1.0, fps=30.0)
    motion = retarget_to_g1(mir)
    assert motion.robot_name == "unitree_g1"
    assert motion.keypoints_3d_array().shape == (30, NUM_JOINTS, 3)
    jsonschema.Draft202012Validator(
        _schema("rd-motion/rd-motion.schema.json")
    ).validate(motion.to_dict())


def test_retarget_is_shorter_and_grounded() -> None:
    mir = generate_dance(duration=1.0, fps=30.0)
    motion = retarget_to_g1(mir)
    robot = motion.keypoints_3d_array()
    # G1 プロキシは人間より低い（height_scale < 1）。
    assert motion.retarget_metrics["height_scale"] < 1.0
    # 接地クランプ: 足が地面付近にあり、地面を大きく貫かない。
    foot_idx = [i for pair in FOOT_JOINTS.values() for i in pair]
    assert robot[:, foot_idx, 2].min() >= -1e-6


def test_bone_directions_preserved() -> None:
    """direction-preserving なので人間と robot の bone 方向はほぼ一致（cos≈1）。"""
    mir = generate_dance(duration=1.0, fps=30.0)
    motion = retarget_to_g1(mir)
    assert motion.retarget_metrics["bone_direction_cosine"] > 0.99


def test_gate_directions_holds_low_confidence_frames() -> None:
    """_gate_directions: 低信頼フレームの方向を直近の高信頼方向へ hold（先頭は back-fill）。"""
    import numpy as np

    from robotdance_retarget.kinematic import _gate_directions

    t, j = 8, 2
    dirs = np.zeros((t, j, 3))
    # joint 0: 各フレーム別方向。conf は frame 3,4 が低信頼。
    for f in range(t):
        dirs[f, 0] = [float(f), 0.0, 0.0]
    dirs[:, 1] = [0.0, 1.0, 0.0]
    conf = np.ones((t, j))
    conf[3, 0] = conf[4, 0] = 0.1                   # 中間の遮蔽窓
    conf[0, 0] = conf[1, 0] = 0.1                   # 先頭の連続低信頼（back-fill 対象）

    gated = _gate_directions(dirs, conf, gate=0.5)
    # 中間窓 3,4 は共に直前の高信頼 frame 2 の方向で hold。
    assert np.allclose(gated[3, 0], dirs[2, 0])
    assert np.allclose(gated[4, 0], dirs[2, 0])
    # 先頭 0,1 は最初の高信頼 frame 2 の方向に back-fill。
    assert np.allclose(gated[0, 0], dirs[2, 0])
    assert np.allclose(gated[1, 0], dirs[2, 0])
    # 高信頼フレームは不変、別 joint も不変。
    assert np.allclose(gated[5, 0], dirs[5, 0])
    assert np.allclose(gated[:, 1], dirs[:, 1])


def test_confidence_gate_suppresses_occluded_joint_spike() -> None:
    """低信頼（遮蔽）フレームの手首を破損させても、conf_gate が暴れを直近の高信頼方向へ
    hold して retarget 後の前腕方向スパイクを抑える。off では暴れが残る（回帰ガード）。"""
    import numpy as np

    from robotdance_core.skeleton import index_of
    from robotdance_retarget.kinematic import retarget

    clean = generate_dance(duration=1.5, fps=30.0)
    kp = np.array(clean.keypoints_3d)               # [T, J, 3]
    t, j = kp.shape[0], kp.shape[1]
    wj, elbow = index_of("left_wrist"), index_of("left_elbow")
    bad = range(t // 3, t // 3 + 5)                 # 連続する遮蔽窓

    conf = np.ones((t, j))
    corrupted = kp.copy()
    for f in bad:                                   # 遮蔽時の誤検出を模擬: 肘の反対側へ飛ばす
        corrupted[f, wj] = corrupted[f, elbow] + np.array([-0.8, 0.0, 0.8])
        conf[f, wj] = 0.05                          # その窓だけ低信頼

    mir = clean.model_copy(update={
        "keypoints_3d": corrupted.tolist(),
        "confidence": {"joint": conf.tolist()},
    })

    f0 = list(bad)[0]
    pre = f0 - 1                                             # 窓直前の高信頼フレーム

    def forearm_dir(motion, frame):
        r = motion.keypoints_3d_array()
        v = r[frame, wj] - r[frame, elbow]
        return v / np.maximum(np.linalg.norm(v), 1e-9)

    off = retarget(mir, g1.MORPHOLOGY)                       # gating なし → garbage 方向のまま
    on = retarget(mir, g1.MORPHOLOGY, conf_gate=0.5)         # gating あり → 窓直前の方向を hold

    garbage = np.array([-0.8, 0.0, 0.8])
    garbage = garbage / np.linalg.norm(garbage)
    # off: 窓内の前腕は破損方向（garbage）に一致してしまう。
    assert float(forearm_dir(off, f0) @ garbage) > 0.97
    # on: 窓内の前腕は窓直前の高信頼方向へ hold され、garbage からは離れる。
    assert float(forearm_dir(on, f0) @ garbage) < 0.8
    assert float(forearm_dir(on, f0) @ forearm_dir(off, pre)) > 0.97   # 直前方向を保持
    # メトリクスに gate 情報が記録される（off では付かない）。
    cg = on.retarget_metrics["confidence_gate"]
    assert cg["gate"] == 0.5 and cg["gated_direction_ratio"] > 0.0
    assert "confidence_gate" not in off.retarget_metrics


def test_roundtrip(tmp_path: Path) -> None:
    mir = generate_dance(duration=1.0, fps=30.0)
    p = retarget_to_g1(mir).save(tmp_path / "g1.rdmotion.json")
    loaded = RdMotion.load(p)
    assert loaded.source_motion_id == mir.motion_id


def test_side_by_side_render(tmp_path: Path) -> None:
    pytest.importorskip("matplotlib")
    from robotdance_viewer.skeleton_view import render_side_by_side

    mir = generate_dance(duration=0.5, fps=20.0)
    motion = retarget_to_g1(mir)
    out = render_side_by_side(
        [
            (mir.keypoints_3d_array(), "human", "#1f77b4"),
            (motion.keypoints_3d_array(), "g1", "#ff7f0e"),
        ],
        tmp_path / "pair.gif",
        fps=20.0,
        stride=2,
    )
    assert out.exists() and out.stat().st_size > 0


def test_retarget_reports_joint_flexion_within_limits() -> None:
    """retarget_metrics に膝・肘の屈曲メトリクスが付き、ダンスは実可動域内（違反0）。"""
    import numpy as np

    motion = retarget_to_g1(generate_dance(duration=2.0))
    jf = motion.retarget_metrics["joint_flexion"]
    assert set(jf["tracked"]) == {"left_knee", "right_knee", "left_elbow", "right_elbow"}
    assert jf["any_violation_ratio"] == 0.0
    for d in jf["per_joint"].values():
        assert d["max_flexion_rad"] <= d["limit_upper_rad"]
        assert 0.0 <= d["max_flexion_rad"] <= np.pi


def test_joint_flexion_detects_over_bend() -> None:
    """関節を実可動域上限を超えて屈曲させると violation_ratio が立つ。"""
    import numpy as np

    from robotdance_core.skeleton import NUM_JOINTS, index_of
    from robotdance_retarget.kinematic import _joint_flexion_metrics
    from robotdance_unitree import get_morphology

    morph = get_morphology("unitree_g1")
    kps = np.zeros((4, NUM_JOINTS, 3))
    # 左脚: hip 上、knee 下、ankle を knee の真上へ折り畳む（屈曲≈π > 膝上限 2.88）。
    kps[:, index_of("left_hip")] = [0.0, 0.1, 1.0]
    kps[:, index_of("left_knee")] = [0.0, 0.1, 0.6]
    kps[:, index_of("left_ankle")] = [0.0, 0.1, 1.0]   # ankle が hip と同位置＝完全折り畳み
    # 右脚・腕は直伸（違反なし）にしておく。
    for prox, mid, dist in [("right_hip", "right_knee", "right_ankle"),
                            ("left_shoulder", "left_elbow", "left_wrist"),
                            ("right_shoulder", "right_elbow", "right_wrist")]:
        kps[:, index_of(prox)] = [0.0, -0.1, 1.0]
        kps[:, index_of(mid)] = [0.0, -0.1, 0.6]
        kps[:, index_of(dist)] = [0.0, -0.1, 0.2]
    jf = _joint_flexion_metrics(kps, morph)
    assert jf["per_joint"]["left_knee"]["violation_ratio"] == 1.0   # 全フレーム超過
    assert jf["per_joint"]["right_knee"]["violation_ratio"] == 0.0  # 直伸は違反なし
    assert jf["any_violation_ratio"] == 1.0


def test_overbend_synthetic_violates_via_real_retarget() -> None:
    """合成 overbend を実 retarget に通すと肘が実可動域上限を超え violation_ratio>0。

    手組み keypoints ではなく synthetic→retarget の実経路で違反が出ることを確認する
    （direction-preserving FK が人間の過屈曲を robot bone にそのまま写すことの検証）。
    """
    from robotdance_core.synthetic import generate_overbend

    jf = retarget_to_g1(generate_overbend()).retarget_metrics["joint_flexion"]
    assert jf["any_violation_ratio"] > 0.0
    for side in ("left_elbow", "right_elbow"):
        d = jf["per_joint"][side]
        assert d["max_flexion_rad"] > d["limit_upper_rad"]  # 上限超過
        assert d["violation_ratio"] > 0.0
    # 脚は曲げていない → 膝は違反なし。
    assert jf["per_joint"]["left_knee"]["violation_ratio"] == 0.0


def test_clamp_flexion_brings_overbend_within_limits() -> None:
    """clamp_flexion=True で overbend の肘違反が 0 になり、補正量が記録され bone 長が保存される。"""
    import numpy as np

    from robotdance_core.skeleton import BONES
    from robotdance_core.synthetic import generate_overbend
    from robotdance_retarget.kinematic import retarget

    mir = generate_overbend()
    raw = retarget(mir, g1.MORPHOLOGY)
    clamped = retarget(mir, g1.MORPHOLOGY, clamp_flexion=True)

    assert raw.retarget_metrics["joint_flexion"]["any_violation_ratio"] > 0.0
    jf = clamped.retarget_metrics["joint_flexion"]
    assert jf["any_violation_ratio"] == 0.0
    # 補正後の肘屈曲は上限ちょうど（以下）。
    for side in ("left_elbow", "right_elbow"):
        d = jf["per_joint"][side]
        assert d["max_flexion_rad"] <= d["limit_upper_rad"] + 1e-6
    # 補正サマリが記録される。
    clamp = jf["clamp"]
    assert clamp["applied"] is True
    assert clamp["corrected_frame_ratio"] > 0.0
    assert clamp["per_joint"]["left_elbow"]["pre_clamp_max_flexion_rad"] > clamp["per_joint"]["left_elbow"]["limit_upper_rad"]
    # bone 長は剛体回転なので保存される。
    rk, ck = np.array(raw.keypoints_3d), np.array(clamped.keypoints_3d)
    for c, p in BONES:
        assert np.allclose(np.linalg.norm(rk[:, c] - rk[:, p], axis=1),
                           np.linalg.norm(ck[:, c] - ck[:, p], axis=1), atol=1e-9)


def test_clamp_flexion_noop_when_within_limits() -> None:
    """可動域内のダンスでは clamp_flexion は補正せず（corrected_frame_ratio=0）keypoints も不変。"""
    import numpy as np

    from robotdance_retarget.kinematic import retarget

    mir = generate_dance(duration=2.0)
    base = retarget(mir, g1.MORPHOLOGY)
    clamped = retarget(mir, g1.MORPHOLOGY, clamp_flexion=True)
    assert clamped.retarget_metrics["joint_flexion"]["clamp"]["corrected_frame_ratio"] == 0.0
    assert np.allclose(np.array(base.keypoints_3d), np.array(clamped.keypoints_3d), atol=1e-9)


def test_cli_retarget_clamp_flexion_flag(tmp_path: Path) -> None:
    """CLI の --clamp-flexion で書き出した RD-Motion の屈曲違反が 0 になる。"""
    from robotdance_core.cli import main
    from robotdance_core.synthetic import generate_overbend

    src = tmp_path / "ob.rdmir.json"
    generate_overbend().save(src)
    out = tmp_path / "rc.rdmotion.json"
    rc = main(["retarget", str(src), "-o", str(out), "--robot", "unitree_g1", "--clamp-flexion"])
    assert rc == 0
    motion = RdMotion.load(out)
    jf = motion.retarget_metrics["joint_flexion"]
    assert jf["any_violation_ratio"] == 0.0
    assert jf["clamp"]["corrected_frame_ratio"] > 0.0


def test_cli_validate_sim_clamp_remedies_rom(tmp_path: Path) -> None:
    """CLI validate-sim: overbend は ROM 超過で rc=1、--clamp-flexion で rc=0（remedy）。"""
    pytest.importorskip("mujoco")
    from robotdance_core.cli import main
    from robotdance_core.synthetic import generate_overbend

    src = tmp_path / "ob.rdmir.json"
    generate_overbend().save(src)
    assert main(["validate-sim", str(src), "--robot", "unitree_g1"]) == 1
    assert main(["validate-sim", str(src), "--robot", "unitree_g1", "--clamp-flexion"]) == 0


def test_clamp_flexion_noop_without_per_joint_limits() -> None:
    """per_joint_limits が無い morphology では clamp は no-op（屈曲メトリクス自体も出ない）。"""
    import dataclasses

    from robotdance_retarget.kinematic import retarget
    from robotdance_unitree import get_morphology

    stripped = dataclasses.replace(get_morphology("unitree_g1"), per_joint_limits=None)
    motion = retarget(generate_dance(duration=1.0), stripped, clamp_flexion=True)
    assert "joint_flexion" not in motion.retarget_metrics


def test_joint_flexion_absent_without_per_joint_limits() -> None:
    """per_joint_limits が無い morphology では屈曲メトリクスは出ない（測れない）。"""
    import dataclasses

    from robotdance_retarget.kinematic import retarget
    from robotdance_unitree import get_morphology

    stripped = dataclasses.replace(get_morphology("unitree_g1"), per_joint_limits=None)
    motion = retarget(generate_dance(duration=1.0), stripped)
    assert "joint_flexion" not in motion.retarget_metrics
