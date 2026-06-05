"""MuJoCo 物理検証（sim_certificate）の縦スライス。

mujoco 未インストール環境では skip する。
"""

from __future__ import annotations

import pytest

pytest.importorskip("mujoco")

from robotdance_core.synthetic import (  # noqa: E402
    generate_backflip,
    generate_dance,
    generate_overbend,
)
from robotdance_retarget.kinematic import retarget  # noqa: E402
from robotdance_sim.mjcf import FOOT_BOX_HALF_WIDTH, build_mjcf  # noqa: E402
from robotdance_sim.mujoco_backend import (  # noqa: E402
    _foot_footprint,
    _max_bone_angular_speed,
    _zmp_in_support,
    certify,
    simulate_certificate,
)
from robotdance_unitree import get_morphology  # noqa: E402

import numpy as np  # noqa: E402


@pytest.mark.parametrize("robot", ["unitree_g1", "unitree_h1"])
def test_mass_distribution_matches_real_urdf_leg_heavy(robot: str) -> None:
    """質量分布が実 URDF inertial 相当: 実機は脚が最重量（股/膝アクチュエータ）。

    旧実装は Winter 人体計測比（胴体~58%/脚~32%）を全ロボットに適用していたが、これは
    **実ロボットには誤り**だった—実 G1/H1 は脚が ~53-58% で胴体より重い（股・膝に重い
    アクチュエータ）。embodiment.mass_distribution（実 URDF <inertial> 由来）を sim に流し、
    生成 MJCF の質量分布が実機の脚優位な分布になることを担保する。
    """
    import mujoco

    from robotdance_core.skeleton import JOINT_NAMES

    morph = get_morphology(robot)
    model = mujoco.MjModel.from_xml_string(build_mjcf(morph, total_mass=morph.sim_defaults.total_mass))
    groups = {
        "trunk": ("pelvis", "spine", "chest", "neck", "head"),
        "arms": ("shoulder", "elbow", "wrist"),
        "legs": ("hip", "knee", "ankle", "foot"),
    }
    g = {"trunk": float(model.body_mass[model.body("root").id]), "arms": 0.0, "legs": 0.0}
    for j, name in enumerate(JOINT_NAMES):
        try:
            mm = float(model.body_mass[model.body(f"body_{j}").id])
        except Exception:
            continue
        for grp, keys in groups.items():
            if any(k in name for k in keys):
                g[grp] += mm
                break
    tot = sum(g.values())
    trunk, arms, legs = g["trunk"] / tot, g["arms"] / tot, g["legs"] / tot
    # 実機は脚が最重量（胴体・腕より重い）。人体プライアの「胴体最重量」とは逆。
    assert legs > 0.45, f"{robot}: 脚が軽すぎる({legs:.0%})—実 URDF では最重量(~53-58%)"
    assert legs > trunk, f"{robot}: 脚({legs:.0%})が胴体({trunk:.0%})より軽い—実機分布と不一致"
    assert arms < 0.25, f"{robot}: 腕が重すぎる({arms:.0%})"


def test_zmp_support_uses_polygon_not_per_foot_circles() -> None:
    """支持判定は足点の凸包（支持多角形）で行う。広い脚幅でも中心 ZMP を支持と認める。

    旧実装は各足点を半径 margin の円で覆う近似で、脚幅が広い機種（H1: 足点 y=±0.26）では
    両足の中間（バランスの取れた ZMP の定位置）がどの足点からも margin 超になり、
    正しく立っているのに転倒判定していた。凸包内なら距離0で支持とするのが正しい。
    """
    # H1 相当の広い両足支持多角形（ankle + toe, y=±0.26）。
    feet = np.array([[0.06, 0.26], [0.06, -0.26], [0.16, 0.26], [0.16, -0.26]])
    centered = np.array([0.09, 0.0])  # 両足の中間 = バランス点
    assert _zmp_in_support(centered, feet, margin=0.05), "広い脚幅で中心 ZMP が支持外と誤判定"
    # 多角形から十分外（margin 超）は支持外。
    far_out = np.array([0.09, 0.6])
    assert not _zmp_in_support(far_out, feet, margin=0.12), "明らかに支持外の ZMP を支持と誤判定"
    # 単一足（線分）でも近ければ支持、遠ければ支持外。
    one_foot = np.array([[0.06, 0.26], [0.16, 0.26]])
    assert _zmp_in_support(np.array([0.11, 0.30]), one_foot, margin=0.1)
    assert not _zmp_in_support(np.array([0.11, 0.60]), one_foot, margin=0.1)


def test_foot_footprint_has_real_width_for_single_support() -> None:
    """接地足は幅ゼロの線分でなく、実フットプリント（足 box 幅）の矩形として支持に寄与する。

    旧来は ankle/toe の 2 点だけで横幅ゼロ → 片足支持で横バランスが評価できず margin 頼みだった。
    footprint は ankle→toe に直交方向へ box 半幅だけ広がり、片足でも横方向の支持を持つ。
    """
    ankle = np.array([0.0, 0.10])
    toe = np.array([0.12, 0.10])  # 前向き（+x）の足
    corners = np.array(_foot_footprint(ankle, toe))
    # 4 隅で、横（y）方向に ±box半幅の広がりを持つ。
    assert len(corners) == 4
    assert corners[:, 1].max() == pytest.approx(0.10 + FOOT_BOX_HALF_WIDTH)
    assert corners[:, 1].min() == pytest.approx(0.10 - FOOT_BOX_HALF_WIDTH)
    # 片足支持: 足中心からやや横にずれた ZMP も、footprint 幅の内側なら支持（margin 0 でも）。
    lateral = np.array([0.06, 0.10 + FOOT_BOX_HALF_WIDTH * 0.5])
    assert _zmp_in_support(lateral, corners, margin=0.0), "片足 footprint の横幅内が支持外と誤判定"


def test_build_mjcf_uses_real_inertia_tensors_when_present() -> None:
    """inertia_tensors を持つ morphology は capsule 近似でなく実テンソルを MJCF に使う（frame 正）。

    opt-in capability: 既定 morphology は capsule（controller baseline 安定のため）だが、
    inertia_tensors を渡すと explicit <inertial> で実機慣性を使う。MuJoCo の body_inertia
    （principal）が埋め込みテンソルの固有値に一致し、総質量も保存することを担保する。
    """
    import dataclasses

    import mujoco

    from robotdance_core.skeleton import JOINT_NAMES
    from robotdance_unitree.g1 import G1_INERTIA_TENSORS

    base = get_morphology("unitree_g1")
    morph = dataclasses.replace(base, inertia_tensors=G1_INERTIA_TENSORS)
    tm = base.sim_defaults.total_mass
    model = mujoco.MjModel.from_xml_string(build_mjcf(morph, total_mass=tm, ground=False))
    # 総質量は保存。
    assert model.body_mass.sum() == pytest.approx(tm, abs=1e-3)
    # 代表 bone の MuJoCo 主慣性が埋め込みテンソルの固有値に一致（フレーム変換が正しい）。
    for name in ("left_knee", "chest", "left_ankle"):
        j = JOINT_NAMES.index(name)
        bid = model.body(f"body_{j}").id
        f = G1_INERTIA_TENSORS[name]["fullinertia"]
        mat = np.array([[f[0], f[3], f[4]], [f[3], f[1], f[5]], [f[4], f[5], f[2]]])
        scale = tm / 34.13  # 埋め込みは実 URDF 総質量基準
        src_eig = np.sort(np.linalg.eigvalsh(mat)) * scale
        mj_eig = np.sort(model.body_inertia[bid])
        assert np.allclose(mj_eig, src_eig, atol=2e-3), f"{name}: {mj_eig} vs {src_eig}"
    # capsule 既定（inertia_tensors 無し）とは主慣性が異なる（実テンソルが効いている）。
    cap = mujoco.MjModel.from_xml_string(build_mjcf(base, total_mass=tm, ground=False))
    jc = JOINT_NAMES.index("chest")
    assert not np.allclose(model.body_inertia[model.body(f"body_{jc}").id],
                           cap.body_inertia[cap.body(f"body_{jc}").id], atol=1e-3)


def test_booster_t1_real_inertia_mjcf_frame_and_mass() -> None:
    """Booster T1 の実 URDF 慣性テンソルが正しいフレームで MJCF に入る（body_inertia 一致・質量保存）。

    T1 の慣性は per-link を平行軸合成して canonical bone へ写像（URDF inertial は回転なし）。MuJoCo の
    body_inertia（principal）が埋め込みテンソルの固有値に一致し、総質量も保存することを担保する。
    """
    import mujoco

    from robotdance_core.skeleton import JOINT_NAMES
    from robotdance_unitree.booster_t1 import T1_INERTIA_TENSORS

    real = get_morphology("booster_t1", real_inertia=True)
    tm = real.sim_defaults.total_mass
    model = mujoco.MjModel.from_xml_string(build_mjcf(real, total_mass=tm, ground=False))
    assert model.body_mass.sum() == pytest.approx(tm, abs=1e-3)
    # floor 込みの raw 合計で scale される（_inertial_xml と同じ）。
    floor = 0.02
    raw_sum = sum(
        T1_INERTIA_TENSORS[n]["mass"] if n in T1_INERTIA_TENSORS else floor for n in JOINT_NAMES
    )
    iscale = tm / raw_sum
    for name in ("chest", "left_knee", "left_ankle"):
        f = T1_INERTIA_TENSORS[name]["fullinertia"]
        mat = np.array([[f[0], f[3], f[4]], [f[3], f[1], f[5]], [f[4], f[5], f[2]]])
        bid = model.body(f"body_{JOINT_NAMES.index(name)}").id
        src_eig = np.sort(np.linalg.eigvalsh(mat)) * iscale
        mj_eig = np.sort(model.body_inertia[bid])
        assert np.allclose(mj_eig, src_eig, atol=2e-3), f"{name}: {mj_eig} vs {src_eig}"


def test_get_morphology_real_inertia_opt_in() -> None:
    """get_morphology(real_inertia=True) は実 URDF 慣性テンソルを装着、既定は capsule（None）。"""
    base = get_morphology("unitree_g1")
    real = get_morphology("unitree_g1", real_inertia=True)
    assert getattr(base, "inertia_tensors", None) in (None, {})
    assert real.inertia_tensors and len(real.inertia_tensors) > 0
    # 慣性以外は不変（質量・rest など）。
    assert real.sim_defaults.total_mass == base.sim_defaults.total_mass
    assert np.array_equal(real.rest_pose, base.rest_pose)


def test_certificate_uses_real_inertia_by_default() -> None:
    """certificate は既定で実 URDF 慣性で検証（approximate_inertia=False）、real_inertia=False で capsule。

    capsule は COM を幾何中心に置き subtree COM→重力トルクを誤推定する。実慣性は実 <inertial> の
    COM オフセットで正し、H1 では torque_ratio を補正（capsule が過大評価）。morphology が
    inertia_tensors を持たなくても embodiment registry から名前で装着する（tracking/PPO 経路は不変）。
    """
    morph = get_morphology("unitree_h1")  # capsule 既定 morphology（inertia_tensors なし）
    motion = retarget(generate_dance(duration=1.0), morph)
    real = simulate_certificate(motion, morph)                    # 既定 real_inertia=True
    cap = simulate_certificate(motion, morph, real_inertia=False)  # capsule 再現
    assert real["approximate_inertia"] is False
    assert cap["approximate_inertia"] is True
    # capsule は COM を幾何中心に置きトルクを過大評価する（実慣性の方が低い負荷率）。
    assert real["metrics"]["torque_ratio"] < cap["metrics"]["torque_ratio"]
    # 過大評価は borderline 運動で verdict を反転させうる（実慣性の価値）: capsule は実機より保守的。
    assert cap["metrics"]["torque_ratio"] >= real["metrics"]["torque_ratio"]


@pytest.mark.parametrize("robot", ["unitree_g1", "unitree_h1"])
def test_pd_tracking_stable_with_real_inertia(robot: str) -> None:
    """実慣性でも PD-only 追従は安定（転倒せず RMSE もほぼ不変）。v0.37 の崩壊は PPO 限定。

    実慣性は物理的に正しいが、v0.37 では PPO tracking が崩壊したため opt-in に留めた。本テストは
    **PD baseline（学習なし）は実慣性で退行しない**ことを担保し、feasibility 検証や PD 追従で
    real_inertia を安全に使えることを保証する（PPO 再学習は別タスク）。
    """
    from robotdance_sim.tracking_env import TrackingEnv

    ref = generate_dance(duration=1.0)

    def survive_rmse(morph):  # noqa: ANN001, ANN202
        env = TrackingEnv(ref, morph)
        env.reset()
        rmses = []
        survived = 0
        for t in range(env.T - 1):
            _o, _r, _d, info = env.step(np.zeros(env.action_dim))
            rmses.append(info["pose_rmse"])
            if info["fallen"]:
                break
            survived = t + 1
        return survived / (env.T - 1), float(np.mean(rmses))

    cap_s, cap_r = survive_rmse(get_morphology(robot))
    real_s, real_r = survive_rmse(get_morphology(robot, real_inertia=True))
    assert cap_s == 1.0 and real_s == 1.0       # どちらも転倒しない
    assert abs(real_r - cap_r) < 0.05           # RMSE 退行なし（実測 ≤0.006）


@pytest.mark.parametrize("robot", ["unitree_g1", "unitree_h1"])
def test_mjcf_total_mass_is_conserved(robot: str) -> None:
    """生成 MJCF の総質量が宣言 total_mass（embodiment 既定＝実 URDF 総質量）に一致する。

    旧実装は pelvis ハブ(3kg)+足 box(0.6kg) を total_mass の上乗せにしており、
    宣言35kg の G1 を実38.6kg(+10.3%) で sim していた。PD ゲインや逆動力学トルクは
    実質量に依存するため、宣言と実体がズレると「35kg 用に調整したつもりが 38.6kg を制御」
    という隠れ取り違えが起きる。固定質量を bone 配分予算から差し引いて質量を保存する。
    total_mass は各 embodiment の sim_defaults（実 URDF 総質量: G1 34.13 / H1 59.34kg）。
    """
    import mujoco

    morph = get_morphology(robot)
    total_mass = morph.sim_defaults.total_mass
    model = mujoco.MjModel.from_xml_string(build_mjcf(morph, total_mass=total_mass))
    assert model.body_mass.sum() == pytest.approx(total_mass, abs=1e-3), (
        f"{robot}: 宣言 {total_mass}kg と MJCF 実質量 {model.body_mass.sum():.3f}kg が不一致"
    )


def test_certify_uses_per_joint_torque_limits() -> None:
    """certify は実 per-joint actuator トルク上限で負荷率を判定する（単一スカラー流用ではない）。

    旧実装は torque_ratio = max重力トルク / scalar で、強い関節（膝~139）と弱い関節（足首~35）を
    区別できなかった。per-joint 化後は各関節の必要トルク÷その関節上限の最大率を取る。H1 は
    per_joint_limits を持つので既定（torque_limit 未指定）は per-joint で判定し、scalar を明示すると
    全関節へその値を強制する（旧挙動・対比用）。
    """
    import dataclasses

    morph = get_morphology("unitree_h1")
    motion = retarget(generate_dance(duration=1.0), morph)
    default = simulate_certificate(motion, morph)["metrics"]["torque_ratio"]
    # scalar を明示すると per-joint ではなくその値で判定 → 既定（per-joint）とは異なる。
    scalar80 = simulate_certificate(motion, morph, torque_limit=80.0)["metrics"]["torque_ratio"]
    scalar300 = simulate_certificate(motion, morph, torque_limit=300.0)["metrics"]["torque_ratio"]
    assert default != pytest.approx(scalar80), "既定が per-joint ではなく scalar を使っている"
    assert scalar80 > scalar300, "scalar 上限が小さいほど負荷率は高いはず"
    # per_joint_limits を外すと sim_defaults スカラーへフォールバック（per-joint を使っている証左）。
    stripped = dataclasses.replace(morph, per_joint_limits=None)
    fallback = simulate_certificate(motion, stripped)["metrics"]["torque_ratio"]
    assert fallback != pytest.approx(default), "per_joint_limits の有無で torque_ratio が変わらない"


@pytest.mark.parametrize("robot", ["unitree_g1", "unitree_h1"])
def test_safe_dance_passes(robot: str) -> None:
    morph = get_morphology(robot)
    motion = retarget(generate_dance(duration=2.0), morph)
    cert = simulate_certificate(motion, morph)
    assert cert["passed"] is True
    assert cert["verdict"] == "PASS"
    # 接地して支持されている。
    assert cert["metrics"]["airborne_ratio"] == 0.0
    # 典型トルクは物理的に妥当（特異姿勢の peak ではなく p50 で判定）。
    assert cert["metrics"]["torque_ratio"] < 1.5


@pytest.mark.parametrize("robot", ["unitree_g1", "unitree_h1"])
def test_backflip_is_rejected(robot: str) -> None:
    morph = get_morphology(robot)
    motion = retarget(generate_backflip(), morph)
    cert = simulate_certificate(motion, morph)
    assert cert["passed"] is False
    assert cert["verdict"] == "REJECT"
    assert cert["reasons"]  # 理由が付く
    # 滞空（接地なし）を検出している。
    assert cert["metrics"]["airborne_ratio"] > 0.5


def test_bone_angular_speed_is_twist_free() -> None:
    """全 bone を z=0 平面に置き z 軸回りに一定 ω で剛体回転 → 厳密に ω を返す（twist 非依存）。"""
    from scipy.spatial.transform import Rotation as Rot

    dt = 1.0 / 30.0
    n, omega = 10, 2.0
    # 各 joint を z=0 平面の相異なる点に置く（全 bone が非退化・水平 → z 回転で厳密に ω）。
    base = np.array([[0.1 * (j + 1), 0.02 * j, 0.0] for j in range(19)])
    kps = np.stack([Rot.from_euler("z", omega * f * dt).apply(base) for f in range(n)])
    assert _max_bone_angular_speed(kps, dt) == pytest.approx(omega, abs=1e-6)


def test_overbend_angular_speed_has_no_spurious_spike() -> None:
    """過屈曲モーションの角速度は twist-free 化で偽スパイク（~79 rad/s）が消え実速度になる。"""
    morph = get_morphology("unitree_g1")
    motion = retarget(generate_overbend(), morph)
    cert = simulate_certificate(motion, morph)
    assert cert["metrics"]["max_joint_ang_speed_rad_s"] < 30.0  # 偽スパイクなし
    # 動的サブ指標は全てクリーン（足は接地・直立で転倒/滞空なし）。
    assert cert["metrics"]["airborne_ratio"] == 0.0
    assert cert["metrics"]["balance_violation_ratio"] == 0.0


def test_poses_to_qpos_removes_twist_spike_keeping_positions() -> None:
    """時系列 qpos 復元は twist を時間連続化し、偽スパイクを消しつつ位置を厳密保存する。

    過屈曲では観測 bone 方向が rest と反平行付近に滞在し、フレーム独立の shortest-arc 復元は
    特異点を踏んで bone 軸 twist が ~80 rad/s 跳ねる（reference 速度・PD 追従誤差を汚染）。
    `_poses_to_qpos` は連続フレーム間の swing だけで前進させ twist を注入しない。bone 方向は
    厳密に再現されるので FK 位置は単フレーム版と一致（twist は不可観測かつ位置不変）。
    """
    import mujoco
    from scipy.spatial.transform import Rotation as Rot

    from robotdance_core.skeleton import JOINT_NAMES
    from robotdance_sim.mujoco_backend import _pose_to_qpos, _poses_to_qpos

    morph = get_morphology("unitree_g1")
    motion = retarget(generate_overbend(), morph)
    kps = motion.keypoints_3d_array()
    n = kps.shape[0]
    dt = 1.0 / motion.fps
    model = mujoco.MjModel.from_xml_string(
        build_mjcf(morph, total_mass=morph.sim_defaults.total_mass, ground=False)
    )
    data = mujoco.MjData(model)

    per_frame = np.stack([_pose_to_qpos(model, morph, kps[f]) for f in range(n)])
    temporal = _poses_to_qpos(model, morph, kps)

    def max_qpos_diff_speed(qp: np.ndarray) -> float:
        worst = 0.0
        for j in range(1, len(JOINT_NAMES)):
            adr = model.joint(f"jnt_{j}").qposadr[0]
            qs = qp[:, adr:adr + 4]
            for f in range(n - 1):
                q0 = Rot.from_quat([qs[f, 1], qs[f, 2], qs[f, 3], qs[f, 0]])
                q1 = Rot.from_quat([qs[f + 1, 1], qs[f + 1, 2], qs[f + 1, 3], qs[f + 1, 0]])
                worst = max(worst, (q1 * q0.inv()).magnitude() / dt)
        return worst

    # フレーム独立版はスパイクを持ち、時間連続版は実速度（_max_bone_angular_speed）に収束する。
    spike = max_qpos_diff_speed(per_frame)
    stable = max_qpos_diff_speed(temporal)
    truth = _max_bone_angular_speed(kps, dt)
    assert spike > 50.0                       # 偽スパイクの存在を確認（回帰ガード）
    assert stable < 10.0                      # 連続化でスパイク消滅
    assert stable == pytest.approx(truth, abs=1.0)  # 実 bone 速度と整合

    # FK 位置は完全一致（twist のみの差・位置不変）。
    def positions(qp: np.ndarray) -> np.ndarray:
        out = np.zeros((n, len(JOINT_NAMES), 3))
        for f in range(n):
            data.qpos[:] = qp[f]
            data.qvel[:] = 0
            mujoco.mj_forward(model, data)
            for j in range(1, len(JOINT_NAMES)):
                out[f, j] = data.xpos[model.body(f"body_{j}").id]
        return out

    assert np.abs(positions(per_frame) - positions(temporal)).max() < 1e-9


def test_poses_to_qpos_single_frame_matches_pose_to_qpos() -> None:
    """2D 入力（単フレーム）では `_pose_to_qpos` に委譲して同一 qpos を返す。"""
    import mujoco

    from robotdance_sim.mujoco_backend import _pose_to_qpos, _poses_to_qpos

    morph = get_morphology("unitree_g1")
    kps = retarget(generate_dance(duration=0.5), morph).keypoints_3d_array()
    model = mujoco.MjModel.from_xml_string(
        build_mjcf(morph, total_mass=morph.sim_defaults.total_mass, ground=False)
    )
    assert np.allclose(
        _poses_to_qpos(model, morph, kps[0]), _pose_to_qpos(model, morph, kps[0])
    )


def test_reference_velocity_report_quantifies_twist_spike_removal() -> None:
    """過屈曲の reference 速度は単フレーム復元で偽スパイク、時系列復元で実 bone 速度に収束する。"""
    from robotdance_sim.reference_quality import reference_velocity_report

    morph = get_morphology("unitree_g1")
    r = reference_velocity_report(retarget(generate_overbend(), morph), morph)
    assert r["per_frame_max_rad_s"] > 50.0          # 単フレーム復元はスパイクを持つ
    assert r["temporal_max_rad_s"] < 10.0           # 時系列復元で消える
    # 時系列復元は物理的に真の bone 方向速度（twist-free）に整合。
    assert r["temporal_max_rad_s"] == pytest.approx(r["bone_truth_rad_s"], abs=1.0)
    assert r["spike_factor"] > 5.0                  # 偽スパイクが実速度の数倍


def test_reference_velocity_temporal_never_worse_than_per_frame() -> None:
    """スイート全 motion で時系列復元の reference 速度は単フレーム復元以下（退行ガード）。

    通常運動（反平行付近に滞在する bone 無し）は特異点を踏まないので両者ほぼ一致、過屈曲・
    宙返りなど一部のみ偽スパイクが顕在化する。いずれにせよ temporal が per_frame を上回らない。
    """
    from robotdance_benchmarks.suite import default_motion_suite
    from robotdance_sim.reference_quality import reference_velocity_report

    morph = get_morphology("unitree_g1")
    for name, mir in default_motion_suite().items():
        r = reference_velocity_report(retarget(mir, morph), morph)
        assert r["temporal_max_rad_s"] <= r["per_frame_max_rad_s"] + 0.5, name


def test_reference_trackability_temporal_within_real_velocity_limits() -> None:
    """偽 twist スパイクは実機速度上限を超え reference を追従不能にするが、時系列復元は包絡内。

    backflip は per-frame 復元だと実 URDF アクチュエータ速度上限（per_joint_limits.velocity）を
    超える要求を一部フレームで出す（untrackable>0）。時系列復元（v0.47）はその偽要求を消し、
    全フレームで速度上限内（untrackable=0, max demand<1）= コントローラに渡せる trackable な reference。
    """
    from robotdance_sim.reference_quality import reference_trackability_report

    morph = get_morphology("unitree_g1")
    t = reference_trackability_report(retarget(generate_backflip(), morph), morph)
    # per-frame は実速度上限超過（追従不能フレームあり）。
    assert t["per_frame_untrackable_ratio"] > 0.0
    assert t["per_frame_max_demand_ratio"] > 1.0
    # 時系列復元は全フレームで速度包絡内。
    assert t["temporal_untrackable_ratio"] == 0.0
    assert t["temporal_max_demand_ratio"] < 1.0


def test_reference_trackability_temporal_trackable_across_suite() -> None:
    """スイート全 motion で時系列復元 reference は実機速度上限内（追従不能フレーム 0）。"""
    from robotdance_benchmarks.suite import default_motion_suite
    from robotdance_sim.reference_quality import reference_trackability_report

    for rob in ("unitree_g1", "unitree_h1"):
        morph = get_morphology(rob)
        for name, mir in default_motion_suite().items():
            t = reference_trackability_report(retarget(mir, morph), morph)
            # 時系列復元 reference は全フレームで実機速度上限内（追従可能）。
            assert t["temporal_untrackable_ratio"] == 0.0, f"{rob}/{name}"
            assert t["temporal_max_demand_ratio"] < 1.0, f"{rob}/{name}"


def test_certificate_rejects_rom_violation_and_clamp_remedies() -> None:
    """動的に安定でも実機 ROM 超過なら統合 verdict は REJECT、clamp_flexion で PASS になる。

    sim は動的 feasibility の権威だが、可動域を超える姿勢は指令不能なので運動学的 feasibility
    （joint_flexion）も統合する。overbend G1 は動的にはクリーンだが肘が ROM 超過 → REJECT。
    """
    morph = get_morphology("unitree_g1")
    # 補正なし: 動的指標はクリーンだが ROM 超過で REJECT。
    raw = retarget(generate_overbend(), morph)
    cert = simulate_certificate(raw, morph)
    assert cert["metrics"]["airborne_ratio"] == 0.0
    assert cert["metrics"]["balance_violation_ratio"] == 0.0
    assert cert["metrics"]["max_joint_ang_speed_rad_s"] < 30.0
    assert cert["verdict"] == "REJECT"
    assert cert["metrics"]["joint_flexion_violation_ratio"] > 0.0
    assert any("可動域" in r for r in cert["reasons"])
    # clamp_flexion で可動域内へ収めると PASS。
    fixed = retarget(generate_overbend(), morph, clamp_flexion=True)
    cert2 = simulate_certificate(fixed, morph)
    assert cert2["verdict"] == "PASS"
    assert cert2["metrics"]["joint_flexion_violation_ratio"] == 0.0


def test_certificate_per_joint_velocity_catches_slow_actuator_limit() -> None:
    """関節速度 feasibility は実 per-joint 速度上限で判定し、一律 30 rad/s が見逃す違反を捕える。

    H1 肩の実 actuator 速度上限は 9 rad/s。高速腕運動は肩を ~11 rad/s で駆動し実上限を超える（指令
    不能）。bone 方向の世界角速度は 30 rad/s 未満なので旧来の全関節一律 30 では PASS してしまうが、
    per-joint 実値（v0.38）との比較がこれを捕える（= v0.36 のスカラ→per-joint トルクと同型）。
    """
    morph = get_morphology("unitree_h1")
    fast = generate_dance(beats_per_second=3.0, sway_amp=0.6, arm_amp=0.9)
    cert = simulate_certificate(retarget(fast, morph), morph)
    assert cert["metrics"]["joint_velocity_ratio"] > 1.0       # 実速度上限超過
    assert cert["verdict"] == "REJECT"
    assert any("速度上限超過" in r for r in cert["reasons"])
    # 旧来の全関節一律 30 rad/s なら見逃す（bone 世界角速度 < 30）ことを明示。
    assert cert["metrics"]["max_joint_ang_speed_rad_s"] < 30.0


def test_certificate_velocity_within_limits_for_normal_motion() -> None:
    """通常運動は実 per-joint 速度上限内（joint_velocity_ratio<1）で velocity 理由は出ない。"""
    for rob in ("unitree_g1", "unitree_h1"):
        morph = get_morphology(rob)
        cert = simulate_certificate(retarget(generate_dance(duration=1.0), morph), morph)
        assert cert["metrics"]["joint_velocity_ratio"] < 1.0
        assert not any("速度上限超過" in r for r in cert["reasons"])


def test_certificate_no_velocity_ratio_without_per_joint_limits() -> None:
    """per_joint_limits が無い morphology では per-joint velocity 判定は無効（指標も出ない）。"""
    import dataclasses

    morph = dataclasses.replace(get_morphology("unitree_g1"), per_joint_limits=None)
    cert = simulate_certificate(retarget(generate_dance(duration=1.0), morph), morph)
    assert "joint_velocity_ratio" not in cert["metrics"]


def test_dynamic_torque_exceeds_static_for_fast_motion() -> None:
    """torque_ratio は重力＋慣性（動的）。速い運動では静的（重力保持）を上回り過小評価を是正する。

    H1 の速いダンスは静的トルクでは actuator 上限内（過小評価）だが、慣性トルクを含めると上限を
    超え torque で REJECT する（v0.62）。遅い同型運動では動的≈静的で PASS。
    """
    from robotdance_core.skeleton import JOINT_NAMES

    morph = get_morphology("unitree_h1")
    fast = simulate_certificate(retarget(generate_dance(duration=2.0, beats_per_second=1.6), morph), morph)
    slow = simulate_certificate(retarget(generate_dance(duration=2.0, beats_per_second=0.5), morph), morph)
    # 動的トルクは静的（重力保持）以上（速い運動で顕著）。
    assert fast["metrics"]["dynamic_torque_nm"] > fast["metrics"]["gravity_torque_nm"]
    # 速いダンスは慣性トルクで実 actuator 上限を超過 → torque で REJECT。
    assert fast["metrics"]["torque_ratio"] > 1.0
    assert fast["verdict"] == "REJECT"
    assert any("トルク" in r for r in fast["reasons"])
    # 律速関節（どの関節が effort 上限を律速するか）が metric・reason に出る。
    limiting = fast["metrics"]["torque_limiting_joint"]
    assert limiting in JOINT_NAMES
    torque_reason = next(r for r in fast["reasons"] if "トルク" in r)
    assert limiting in torque_reason and "N·m" in torque_reason
    # PASS の遅い運動でも律速関節は情報として出る（最も上限に近い関節）。
    assert slow["metrics"]["torque_limiting_joint"] in JOINT_NAMES
    # 遅い運動は動的≈静的で torque は上限内。
    assert slow["metrics"]["torque_ratio"] < 1.0


def test_squat_is_feasible_and_march_exercises_balance() -> None:
    """新 motion: squat は接地のまま深屈曲で feasible（膝 ROM/トルク exercise）、march は単脚支持で
    balance 軸を発火させる（airborne ではなく ZMP 違反）。feasibility 軸の判別力を広い運動で確認。"""
    from robotdance_core.synthetic import generate_march, generate_squat

    morph = get_morphology("unitree_h1")
    sq = simulate_certificate(retarget(generate_squat(), morph), morph)
    assert sq["verdict"] == "PASS"                       # 接地・対称で動的にクリーン
    assert sq["metrics"]["airborne_ratio"] == 0.0
    assert sq["metrics"]["torque_ratio"] > 0.0           # 屈曲保持トルクが乗る
    mar = simulate_certificate(retarget(generate_march(), morph), morph)
    assert mar["metrics"]["balance_violation_ratio"] > 0.0  # 単脚支持で ZMP が支持外
    assert mar["metrics"]["airborne_ratio"] == 0.0          # 常に片足は接地（滞空ではない）


def test_gentle_march_lowers_torque_and_passes_on_narrow_stance() -> None:
    """歩調を落とした march（低速・低い持ち上げ）は慣性トルクが下がり、狭股機種（G1）は重心が
    支持内に収まり PASS。広股機種（H1）は受動準静的モデルではなお balance 違反（足首戦略の能動
    バランスが要る — v0 未モデル）。march の feasibility が歩調＋形態で決まることを実証。"""
    from robotdance_core.synthetic import generate_march

    naive = generate_march()
    gentle = generate_march(steps_per_second=0.5, lift=0.5)

    g1 = get_morphology("unitree_g1")
    g1_naive = simulate_certificate(retarget(naive, g1), g1)
    g1_gentle = simulate_certificate(retarget(gentle, g1), g1)
    # 狭股 G1: naive は REJECT、緩やかにすると PASS（歩調で feasible に転じる）。
    assert g1_naive["verdict"] == "REJECT"
    assert g1_gentle["verdict"] == "PASS"

    h1 = get_morphology("unitree_h1")
    h1_naive = simulate_certificate(retarget(naive, h1), h1)
    h1_gentle = simulate_certificate(retarget(gentle, h1), h1)
    # 歩調を落とすと慣性トルクは全機種で下がる（H1 も torque_ratio 低下）。
    assert h1_gentle["metrics"]["torque_ratio"] < h1_naive["metrics"]["torque_ratio"]
    # ただし広股 H1 は受動モデルでなお balance 違反（能動バランス未モデルの境界）。
    assert h1_gentle["metrics"]["balance_violation_ratio"] > 0.3


def test_certificate_balance_trace_matches_verdict() -> None:
    """return_trace=True で per-frame の ZMP/支持多角形/in-support を返し、可視化が certificate と
    同じ値を使えること（single source of truth）。trace の支持外率は balance_violation_ratio と整合。"""
    from robotdance_core.synthetic import generate_march

    morph = get_morphology("unitree_g1")
    cert = simulate_certificate(retarget(generate_march(), morph), morph, return_trace=True)
    tr = cert["trace"]
    n = len(tr["zmp_xy"])
    assert n > 0
    assert len(tr["support_polys"]) == n and len(tr["in_support"]) == n
    # 滞空でないフレームは支持多角形が空でない（march は常に片足接地）。
    assert all(len(p) > 0 for p in tr["support_polys"])
    # trace の支持外率 = balance_violation_ratio（airborne 含む整合）。
    out_ratio = 1.0 - sum(tr["in_support"]) / n
    assert out_ratio == pytest.approx(cert["metrics"]["balance_violation_ratio"], abs=0.02)
    # trace を返さない既定では trace キーは無い（肥大化させない）。
    assert "trace" not in simulate_certificate(retarget(generate_march(), morph), morph)


def test_certificate_no_rom_metric_without_per_joint_limits() -> None:
    """per_joint_limits が無い morphology では ROM 統合は無効（joint_flexion 指標も出ない）。"""
    import dataclasses

    morph = dataclasses.replace(get_morphology("unitree_g1"), per_joint_limits=None)
    cert = simulate_certificate(retarget(generate_dance(duration=1.0), morph), morph)
    assert "joint_flexion_violation_ratio" not in cert["metrics"]


def test_certify_attaches_to_motion() -> None:
    morph = get_morphology("unitree_g1")
    motion = retarget(generate_dance(duration=1.0), morph)
    assert motion.sim_certificate is None
    certify(motion, morph)
    assert motion.sim_certificate is not None
    assert motion.sim_certificate["backend"] == "mujoco"
    # certificate 付き motion も RD-Motion schema に適合する。
    import json
    from pathlib import Path

    import jsonschema

    schema = json.loads(
        (Path(__file__).resolve().parent.parent / "specs" / "rd-motion" / "rd-motion.schema.json")
        .read_text(encoding="utf-8")
    )
    jsonschema.Draft202012Validator(schema).validate(motion.to_dict())
