"""MuJoCo を使った物理ベースの feasibility 検証（v0）。

受動ヒューマノイドはバランス制御なしでは何でも倒れるため、forward sim は判別力を持たない。
本バックエンドは **参照運動そのものが物理的に実現可能か** を MuJoCo の動力学で検証する:

  1. keypoints を ball-joint 多体モデルの qpos に厳密復元
  2. 逆動力学（mj_inverse）で各 joint の必要トルク → torque saturation
  3. 質量モデルの COM → ZMP を計算し、接地足の支持多角形を外れる/滞空で balance violation

⚠️ v0 注意: 質量分布・慣性テンソルは実 URDF <inertial> 由来（v0.34/v0.52, 既定で実慣性）だが、
link→bone は世界 COM 最近傍集約・bone は capsule/点質量プロキシ。トルクは重力保持（準静的）で動的
トルクは含まない。出力 sim_certificate は "physically-informed feasibility" であって実機保証ではない
（近似と境界の詳細は docs/SIM_TO_REAL.md）。
"""

from __future__ import annotations

import dataclasses
from typing import Any

import numpy as np
from scipy.spatial.transform import Rotation as Rot

from robotdance_core.rd_motion import RdMotion
from robotdance_core.skeleton import FOOT_JOINTS, JOINT_NAMES, PARENTS
from robotdance_retarget.embodiment import RobotMorphology

from .mjcf import FOOT_BOX_HALF_WIDTH, build_mjcf

_G = 9.81
_EPS = 1e-9


def _min_rot_quat(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """単位ベクトル a を b に重ねる最小回転（quaternion, wxyz）。"""
    a = a / max(np.linalg.norm(a), _EPS)
    b = b / max(np.linalg.norm(b), _EPS)
    c = float(np.dot(a, b))
    if c > 1.0 - 1e-8:
        return np.array([1.0, 0.0, 0.0, 0.0])
    if c < -1.0 + 1e-8:
        # 反平行: a に直交する任意軸で 180°。
        axis = np.cross(a, np.array([1.0, 0.0, 0.0]))
        if np.linalg.norm(axis) < 1e-6:
            axis = np.cross(a, np.array([0.0, 1.0, 0.0]))
        axis /= np.linalg.norm(axis)
        return np.array([0.0, *axis])
    axis = np.cross(a, b)
    q = np.array([1.0 + c, *axis])
    return q / np.linalg.norm(q)


def _quat_to_mat(q_wxyz: np.ndarray) -> np.ndarray:
    return Rot.from_quat([q_wxyz[1], q_wxyz[2], q_wxyz[3], q_wxyz[0]]).as_matrix()


def _mat_to_quat(m: np.ndarray) -> np.ndarray:
    x, y, z, w = Rot.from_matrix(m).as_quat()
    return np.array([w, x, y, z])


def _max_bone_angular_speed(kps: np.ndarray, dt: float) -> float:
    """各 bone（親→子）方向の連続フレーム間角度変化率の最大値 [rad/s]。

    keypoints から ball joint の qpos を再構成して差分すると、bone 軸まわりの twist は
    keypoints に拘束されず（向きが任意に決まる）、再構成 quaternion の不連続が**データに無い
    偽の角速度スパイク**を生む（特に手首など leaf joint・極端な屈曲で顕著）。bone 方向は 2-DOF で
    twist を含まないため、その変化率を直接測ることで twist アーティファクトを排した実速度を得る。
    """
    if kps.shape[0] < 2:
        return 0.0
    par = np.array([max(p, 0) for p in PARENTS])
    d = kps - kps[:, par, :]                                  # [T, J, 3] 親→子
    d = d / np.maximum(np.linalg.norm(d, axis=2, keepdims=True), _EPS)
    dot = (d[:-1] * d[1:]).sum(axis=2)                        # [T-1, J]
    ang = np.arccos(np.clip(dot, -1.0, 1.0)) / dt
    root_cols = [j for j, p in enumerate(PARENTS) if p < 0]
    ang[:, root_cols] = 0.0
    return float(ang.max())


def _max_joint_velocity_ratio(
    model, qpos: np.ndarray, dt: float, morphology: RobotMorphology
) -> "tuple[float, tuple[str, float, float] | None]":
    """temporal qpos の各 ball joint の親相対角速度 / 実 per-joint 速度上限 の最大比と詳細。

    アクチュエータが駆動するのは親リンク相対の関節角速度。MuJoCo の tangent 差分
    （mj_differentiatePos）で連続フレーム間の関節速度を取り、**実 URDF 速度上限を持つ関節のみ**
    （per_joint_limits.velocity。generic placeholder は除外）でその上限と比較する。>1.0 で指令速度が
    actuator 速度上限を超え物理的に追従不能。bone 方向の世界角速度ではなく**関節相対**速度を見るのが
    要諦。twist が時間連続化された qpos（_poses_to_qpos）を前提とする（v0.43 の偽スパイク是正済み）。

    返り値: (max_ratio, detail)。detail=(joint_name, speed_rad_s, limit_rad_s) または None。
    """
    import mujoco

    pjl = morphology.per_joint_limits or {}
    limits = []  # (dofadr, limit, joint_name) — 実 velocity 値を持つ関節のみ
    for jid in range(model.njnt):
        name = model.joint(jid).name
        if name and name.startswith("jnt_"):
            jn = JOINT_NAMES[int(name[4:])]
            v = pjl.get(jn, {}).get("velocity")
            if v:
                limits.append((int(model.jnt_dofadr[jid]), float(v), jn))
    if qpos.shape[0] < 2 or not limits:
        return 0.0, None
    dq = np.zeros(model.nv)
    best_ratio = 0.0
    best_detail: "tuple[str, float, float] | None" = None
    for f in range(qpos.shape[0] - 1):
        mujoco.mj_differentiatePos(model, dq, 1.0, qpos[f], qpos[f + 1])
        for adr, lim, jn in limits:
            speed = float(np.linalg.norm(dq[adr:adr + 3])) / dt
            ratio = speed / lim
            if ratio > best_ratio:
                best_ratio = ratio
                best_detail = (jn, speed, lim)
    return best_ratio, best_detail


def _pose_to_qpos(model, morphology: RobotMorphology, kps: np.ndarray) -> np.ndarray:
    """1 フレームの keypoints [J, 3] を qpos に厳密復元する。

    ⚠️ 単フレーム版は各 bone を rest 方向から shortest-arc で独立復元する。bone 軸まわりの
    twist は keypoints に拘束されず、極端な屈曲（観測方向が rest と反平行付近）では shortest-arc の
    回転軸 o×d が悪条件となり、フレーム間で twist が不連続に跳ぶ。**時系列を復元するなら
    `_poses_to_qpos`** を使う（twist を時間方向に伝播し連続化する）。位置（COM/ZMP）はどちらでも
    厳密一致する（twist は不可観測かつ位置不変）。
    """
    rest = morphology.rest_pose
    qpos = np.zeros(model.nq)
    qpos[0:3] = kps[0]
    qpos[3:7] = [1.0, 0.0, 0.0, 0.0]  # base 向きは identity
    r_body: dict[int, np.ndarray] = {}
    for j in range(1, len(JOINT_NAMES)):
        p = PARENTS[j]
        r_parent = np.eye(3) if p == 0 else r_body[p]
        o = rest[j] - rest[p]
        t = kps[j] - kps[p]
        local_t = r_parent.T @ (t / max(np.linalg.norm(t), _EPS))
        q = _min_rot_quat(o, local_t)
        adr = model.joint(f"jnt_{j}").qposadr[0]
        qpos[adr:adr + 4] = q
        r_body[j] = r_parent @ _quat_to_mat(q)
    return qpos


def _poses_to_qpos(model, morphology: RobotMorphology, kps: np.ndarray) -> np.ndarray:
    """keypoints 時系列 [T, J, 3] を **twist が時間方向に連続な** qpos 列 [T, nq] へ復元する。

    単フレーム復元（_pose_to_qpos）を各フレーム独立に適用すると、観測方向が rest と反平行
    付近に滞在する極端な屈曲で shortest-arc の特異点を踏み、bone 軸まわりに**データに無い偽の
    twist スパイク**（実測: 手首で ~80 rad/s）が出る。これは qpos を差分する全経路（RL tracking の
    reference 速度・PD 追従誤差・export 軌道）を汚染する。

    本関数は各 bone の world 向きフレームを **frame 0 は rest 基準の shortest-arc で seed し、
    以降は連続フレーム間の小さな swing（向き変化そのもの）だけで前進**させる（時間方向の平行移動）。
    連続フレームの向きは常に近接するので特異点を踏まず、twist は注入されない。bone 向きは厳密に
    再現されるため位置（COM/ZMP/トルク）は単フレーム版と完全一致し、変わるのは不可観測な twist
    成分の連続性のみ。
    """
    kps = np.asarray(kps, dtype=np.float64)
    if kps.ndim == 2:  # 単フレーム → 単フレーム版に委譲
        return _pose_to_qpos(model, morphology, kps)
    rest = morphology.rest_pose
    n = kps.shape[0]
    qpos = np.zeros((n, model.nq))
    qpos[:, 0:3] = kps[:, 0, :]
    qpos[:, 3:7] = [1.0, 0.0, 0.0, 0.0]  # base 向きは identity

    par = np.array([max(p, 0) for p in PARENTS])
    d = kps - kps[:, par, :]                                   # [T, J, 3] 親→子
    d = d / np.maximum(np.linalg.norm(d, axis=2, keepdims=True), _EPS)

    # 各 joint の world 姿勢 r_world[j][f]（r_world[j][f] @ o_j = d[f,j] を満たす）。
    # root（pelvis）は free joint で base 向き identity。
    r_world: dict[int, np.ndarray] = {0: np.broadcast_to(np.eye(3), (n, 3, 3))}
    for j in range(1, len(JOINT_NAMES)):
        p = PARENTS[j]
        o = rest[j] - rest[p]
        o = o / max(np.linalg.norm(o), _EPS)
        rj = np.zeros((n, 3, 3))
        rj[0] = _quat_to_mat(_min_rot_quat(o, d[0, j]))       # rest 基準 seed
        for f in range(1, n):
            # 連続向き間の swing（小角・常に好条件）だけで前進 → twist を注入しない
            rsw = _quat_to_mat(_min_rot_quat(d[f - 1, j], d[f, j]))
            rj[f] = rsw @ rj[f - 1]
        r_world[j] = rj
        adr = model.joint(f"jnt_{j}").qposadr[0]
        rp = r_world[p]
        for f in range(n):
            qpos[f, adr:adr + 4] = _mat_to_quat(rp[f].T @ rj[f])  # 親相対 = local quat
    return qpos


def simulate_certificate(
    motion: RdMotion,
    morphology: RobotMorphology,
    *,
    total_mass: float | None = None,
    torque_limit: float | None = None,
    support_margin: float = 0.05,
    real_inertia: bool = True,
) -> dict[str, Any]:
    """RD-Motion を MuJoCo 物理で検証し sim_certificate dict を返す。

    質量・トルク上限は embodiment 固有の既定（morphology.sim_defaults）から取る。
    caller が明示すればそれを優先。旧実装は G1 値（35kg / 80N·m）をハードコードしており、
    H1（47kg / 160N·m）の certify でも G1 のトルク上限で torque_ratio を判定していた
    （= v0.27 の SimDefaults を導入したのにこの経路だけ配線漏れ）。

    real_inertia: feasibility 検証を **実 URDF `<inertial>` 慣性テンソル**で行う（既定 True, v0.52）。
        morphology が inertia_tensors を持たなければ EMBODIMENT_INERTIA から名前で装着する。capsule 近似は
        COM を幾何中心に置き subtree COM→重力トルクを誤推定する（実測: H1 で torque_ratio を ~22% 過大評価）。
        実慣性は逆動力学のみで PD-safe（v0.51）。tracking/PPO は別経路で本フラグの影響を受けない。
        False で旧来の capsule 近似に戻せる（再現用）。

    support_margin: ZMP が支持多角形（実フットプリント矩形の凸包）の外へ許容される距離（m）。
        旧既定 0.12 は支持多角形に足の横幅が無かった分（半幅~0.04）を margin で誤魔化していた。
        本実装は足幅を実フットプリントとして明示するので、margin は純粋なスラック（ZMP 推定誤差＋
        未モデルの踵 ~0.05）に縮小（実測: 安定なダンスの ZMP は足面から最大 4.4mm）。
    """
    import mujoco

    # 実慣性で検証する（既定）。morphology に無ければ embodiment registry から名前で装着。
    # sim→unitree の import cycle を避けるため lazy import（unitree は sim を読まない）。
    if real_inertia and not getattr(morphology, "inertia_tensors", None):
        try:
            from robotdance_unitree import EMBODIMENT_INERTIA

            tensors = EMBODIMENT_INERTIA.get(morphology.name)
        except Exception:
            tensors = None
        if tensors:
            morphology = dataclasses.replace(morphology, inertia_tensors=tensors)
    used_real_inertia = bool(getattr(morphology, "inertia_tensors", None))

    sd = morphology.sim_defaults
    total_mass = sd.total_mass if total_mass is None else total_mass
    # torque_limit を明示した場合は全関節へその scalar を強制（旧挙動）。未指定なら関節ごとに
    # 実 actuator 上限（per_joint_limits）を使い、無い関節のみ sim_defaults スカラーへ落ちる。

    # 地面なしの純浮遊多体: mj_inverse に接触力が混入せず、内部トルクが純 RNEA になる。
    # バランスは motion の contact_schedule と keypoints から別途計算するため地面は不要。
    model = mujoco.MjModel.from_xml_string(
        build_mjcf(morphology, total_mass=total_mass, ground=False)
    )
    data = mujoco.MjData(model)
    root_id = model.body("root").id

    kps = motion.keypoints_3d_array()  # [T, J, 3]
    n = kps.shape[0]
    dt = 1.0 / motion.fps

    # 重力保持トルクは subtree COM から解析的に計算する。短い足先 bone（toe）は除外。
    toe_joints = {JOINT_NAMES.index("left_foot"), JOINT_NAMES.index("right_foot")}
    grav_bodies = [model.body(f"body_{j}").id for j in range(1, len(JOINT_NAMES))
                   if j not in toe_joints]
    # grav_bodies と同順の canonical joint 名（律速関節を reason に出すため）。
    grav_joint_names = [JOINT_NAMES[j] for j in range(1, len(JOINT_NAMES))
                        if j not in toe_joints]
    sub_mass = {bid: float(model.body_subtreemass[bid]) for bid in grav_bodies}
    # 各 joint の **実 actuator トルク上限**（per-joint。実値が無ければ sim 既定スカラー）。
    # 強い関節（膝~139）と弱い関節（足首~35）を区別して負荷率を判定するため。
    body_torque_limit = {
        model.body(f"body_{j}").id: (
            torque_limit if torque_limit is not None
            else morphology.joint_torque_limit(JOINT_NAMES[j])
        )
        for j in range(1, len(JOINT_NAMES)) if j not in toe_joints
    }

    # 各フレームの qpos / COM / 各 subtree COM・anchor 軌道。twist は時間方向に連続化（_poses_to_qpos）。
    # 位置は単フレーム版と厳密一致するので COM/ZMP は不変。
    qpos = _poses_to_qpos(model, morphology, kps)
    # reference 速度（tangent 差分）: subtree 角運動量を得るため。twist は時間連続化済み（v0.47）で clean。
    qvel = np.zeros((n, model.nv))
    for f in range(n - 1):
        dq = np.zeros(model.nv)
        mujoco.mj_differentiatePos(model, dq, 1.0, qpos[f], qpos[f + 1])
        qvel[f] = dq / dt
    com = np.zeros((n, 3))
    csub_traj = np.zeros((n, len(grav_bodies), 3))   # 各 body の subtree COM（world）
    anc_traj = np.zeros((n, len(grav_bodies), 3))     # 各 body の joint anchor（world）
    angmom = np.zeros((n, len(grav_bodies), 3))       # subtree COM まわり角運動量（world, mj_subtreeVel）
    for f in range(n):
        data.qpos[:] = qpos[f]
        data.qvel[:] = qvel[f]
        mujoco.mj_forward(model, data)
        mujoco.mj_subtreeVel(model, data)             # subtree_angmom / subtree_linvel を計算
        com[f] = data.subtree_com[root_id]
        for i, bid in enumerate(grav_bodies):
            csub_traj[f, i] = data.subtree_com[bid]
            anc_traj[f, i] = data.xpos[bid]
            angmom[f, i] = data.subtree_angmom[bid]

    # 関節トルク（Newton-Euler）= COM まわり couple（角運動量変化）＋ アンカーでの力モーメント:
    #   τ = dL_com/dt + r × m·(a_com − g)
    # 第2項は重力＋並進慣性（v0.62）、第1項が **subtree 回転慣性の反作用**（dL_com/dt, v0.63 追加）。
    # a_com（ZMP と同じ中心差分）と dL_com/dt も中心差分。mj_inverse は ball-joint 浮遊モデルで特異性に
    # より非物理値を出すため使わず、subtree COM/角運動量から robust に解析計算（剛体 subtree 近似）。
    g_vec = np.array([0.0, 0.0, -_G])
    a_com = np.zeros((n, len(grav_bodies), 3))
    dL = np.zeros((n, len(grav_bodies), 3))
    if n >= 3:
        a_com[1:-1] = (csub_traj[2:] - 2 * csub_traj[1:-1] + csub_traj[:-2]) / (dt * dt)
        dL[1:-1] = (angmom[2:] - angmom[:-2]) / (2.0 * dt)
    r_arm = csub_traj - anc_traj                                  # [n, B, 3]
    limits = np.array([body_torque_limit[bid] for bid in grav_bodies])
    sub_m = np.array([sub_mass[bid] for bid in grav_bodies])
    # 静的（重力保持）: τ = m·|r × (−g)| = m·g·d_horiz。報告用。
    tau_stat = np.linalg.norm(
        np.cross(r_arm, sub_m[None, :, None] * (-g_vec)[None, None, :]), axis=2)
    # 動的（重力＋並進慣性＋回転慣性）: 判定用。
    f_dyn = sub_m[None, :, None] * (a_com - g_vec[None, None, :])
    tau_dyn = np.linalg.norm(dL + np.cross(r_arm, f_dyn), axis=2)  # [n, B]
    gravity_torque = float(tau_stat.max()) if tau_stat.size else 0.0
    dynamic_torque = float(tau_dyn.max()) if tau_dyn.size else 0.0
    # 律速関節: per-joint 負荷率 tau_dyn/limit が最大の body（＝どの関節が effort 上限を律速するか）。
    torque_joint = None       # canonical joint 名
    torque_joint_nm = 0.0     # その関節のピーク動的トルク
    torque_joint_lim = 0.0    # その関節の実 effort 上限
    if tau_dyn.size:
        ratio_mat = tau_dyn / limits[None, :]            # [n, B]
        torque_ratio = float(ratio_mat.max())
        col = int(ratio_mat.max(axis=0).argmax())        # 最大負荷率の body 列
        torque_joint = grav_joint_names[col]
        torque_joint_nm = float(tau_dyn[:, col].max())
        torque_joint_lim = float(limits[col])
    else:
        torque_ratio = 0.0

    # joint 角速度: bone 方向の変化率（twist-free, _max_bone_angular_speed 参照）。
    # 旧来は再構成 qpos の差分だったが、leaf joint の未拘束 twist が偽スパイクを生んでいた。
    max_joint_ang_speed = _max_bone_angular_speed(kps, dt)

    # COM 加速度 → ZMP（平地・総質量点近似, ground z=0）。
    com_acc = np.zeros((n, 3))
    com_acc[1:-1] = (com[2:] - 2 * com[1:-1] + com[:-2]) / (dt * dt)
    denom = com_acc[:, 2] + _G
    zmp = np.zeros((n, 2))
    safe = np.abs(denom) > 1e-3
    zmp[safe, 0] = com[safe, 0] - com[safe, 2] * com_acc[safe, 0] / denom[safe]
    zmp[safe, 1] = com[safe, 1] - com[safe, 2] * com_acc[safe, 1] / denom[safe]

    # 接地 / バランス判定。支持多角形は接地足の**実フットプリント矩形**（ankle→toe を長辺、
    # 足 box 幅を短辺）の凸包。旧来は ankle/toe の 2 点（＝幅ゼロの前後線分）だけで、足の横幅を
    # 無視し margin で誤魔化していた。特に片足支持では幅ゼロになり横バランスが評価できなかった。
    contacts = motion.contact_schedule or {}
    airborne = 0
    unsupported = 0
    for f in range(n):
        corners: list[np.ndarray] = []
        for side, (ankle, toe) in FOOT_JOINTS.items():
            if np.asarray(contacts.get(f"{side}_foot", [False] * n), dtype=bool)[f]:
                corners += _foot_footprint(kps[f][ankle][:2], kps[f][toe][:2])
        if not corners:
            airborne += 1
            unsupported += 1
            continue
        if not _zmp_in_support(zmp[f], np.array(corners), support_margin):
            unsupported += 1

    airborne_ratio = airborne / n
    balance_violation_ratio = unsupported / n
    # max_joint_ang_speed は上で bone 方向から算出済み（twist-free）。
    # torque_ratio は per-joint 負荷率の最大（上のループで算出済み）。

    # 運動学的 feasibility（関節可動域）: retarget が算出した joint_flexion 違反を取り込む。
    # sim は動的 feasibility（転倒/トルク/滞空）の権威だが、実機可動域を超える姿勢は
    # 動的に安定でも「指令不能」なので、ここで統合して REJECT 理由に含める（per_joint_limits
    # を持つ embodiment のみ。retarget 側で測れない場合は None で判定対象外）。
    jf = (motion.retarget_metrics or {}).get("joint_flexion") or {}
    flexion_violation = jf.get("any_violation_ratio")

    # 関節速度 feasibility: temporal qpos の関節相対速度を**実 per-joint 速度上限**と比較。
    # per_joint_limits があるときのみ per-joint 判定（無ければ従来の全関節一律 30 rad/s）。旧来の
    # スカラ 30 は実機差（9-37 rad/s）を無視し、H1 足首/肩のような遅い限界（9 rad/s）の超過を
    # 見逃していた（= v0.36 のスカラトルク問題と同型）。v0.47 で twist を時間連続化したので
    # reconstructed qpos の関節相対速度を安全に使える。
    velocity_ratio = None
    velocity_detail = None
    if morphology.per_joint_limits:
        velocity_ratio, velocity_detail = _max_joint_velocity_ratio(model, qpos, dt, morphology)

    reasons: list[str] = []
    if airborne_ratio > 0.1:
        reasons.append(f"airborne {airborne_ratio:.0%}（接地なしで支持不能）")
    if balance_violation_ratio > 0.3:
        reasons.append(f"ZMP が支持多角形外 {balance_violation_ratio:.0%}（転倒リスク）")
    if torque_ratio > 1.0:
        reasons.append(
            f"関節トルク ×{torque_ratio:.2f}（{torque_joint} {torque_joint_nm:.0f}>"
            f"{torque_joint_lim:.0f} N·m, 重力＋慣性, 実 actuator 限界超過）"
        )
    if velocity_ratio is not None:
        if velocity_ratio > 1.0:
            jn, sp, lim = velocity_detail
            reasons.append(
                f"関節速度過大 ×{velocity_ratio:.2f}（{jn} {sp:.0f}>{lim:.0f} rad/s "
                "実機 actuator 速度上限超過）"
            )
    elif max_joint_ang_speed > 30.0:
        reasons.append(f"関節角速度過大 {max_joint_ang_speed:.0f} rad/s")
    if flexion_violation is not None and flexion_violation > 0.0:
        reasons.append(
            f"関節可動域超過 {flexion_violation:.0%}（膝・肘が実機 ROM を超過 — "
            "retarget の clamp_flexion で補正可）"
        )

    passed = not reasons
    metrics = {
        "airborne_ratio": round(airborne_ratio, 3),
        "balance_violation_ratio": round(balance_violation_ratio, 3),
        "gravity_torque_nm": round(gravity_torque, 1),
        "dynamic_torque_nm": round(dynamic_torque, 1),
        "torque_ratio": round(torque_ratio, 3),
        "max_joint_ang_speed_rad_s": round(max_joint_ang_speed, 2),
    }
    # 律速関節（PASS でも情報として出す: どの関節が effort 上限に最も近いか）。
    if torque_joint is not None:
        metrics["torque_limiting_joint"] = torque_joint
    if velocity_ratio is not None:
        metrics["joint_velocity_ratio"] = round(velocity_ratio, 3)
    if flexion_violation is not None:
        metrics["joint_flexion_violation_ratio"] = round(flexion_violation, 3)
    return {
        "backend": "mujoco",
        "mujoco_version": mujoco.__version__,
        "approximate_inertia": not used_real_inertia,
        "passed": passed,
        "verdict": "PASS" if passed else "REJECT",
        "metrics": metrics,
        "thresholds": {
            "airborne_ratio": 0.1,
            "balance_violation_ratio": 0.3,
            "gravity_torque_ratio": 1.0,
            "max_joint_ang_speed_rad_s": 30.0,
            "joint_velocity_ratio": 1.0,
            "joint_flexion_violation_ratio": 0.0,
        },
        "reasons": reasons,
        "note": (
            "physically-informed feasibility（"
            + ("実 URDF 慣性" if used_real_inertia else "capsule 近似慣性")
            + ", v0）— 実機保証ではない。動的（転倒/トルク/滞空/速度）＋運動学的（関節可動域）の両"
            " feasibility を統合。torque_ratio は subtree COM への重力＋慣性力（a_com 由来）の関節モーメント"
            "／実 actuator 上限（mj_inverse の ball-joint 特異性を回避した robust な解析法。回転慣性項は省く）。"
        ),
    }


def certify(motion: RdMotion, morphology: RobotMorphology, **kwargs: Any) -> RdMotion:
    """sim_certificate を計算して motion に格納し、同じ motion を返す。"""
    motion.sim_certificate = simulate_certificate(motion, morphology, **kwargs)
    return motion


def _foot_footprint(ankle_xy: np.ndarray, toe_xy: np.ndarray) -> list[np.ndarray]:
    """1 足の接地フットプリント矩形 4 隅（xy）。長辺=ankle→toe, 短辺=足 box 幅。

    旧来は ankle/toe の 2 点だけを支持点にしており、足の横幅（実 sim の foot box 幅）を
    無視していた。これにより片足支持では幅ゼロの線分になり横バランスが評価できなかった。
    ankle→toe 方向に直交する向きへ box 半幅だけ広げ、実フットプリント相当の面を与える。
    """
    fwd = toe_xy - ankle_xy
    norm = float(np.linalg.norm(fwd))
    # 足が xy で潰れている（真上から踏む）退避: 既定の前向き。
    fdir = fwd / norm if norm > 1e-6 else np.array([1.0, 0.0])
    perp = np.array([-fdir[1], fdir[0]]) * FOOT_BOX_HALF_WIDTH
    return [ankle_xy + perp, ankle_xy - perp, toe_xy + perp, toe_xy - perp]


def _zmp_in_support(zmp_xy: np.ndarray, foot_pts: np.ndarray, margin: float) -> bool:
    """ZMP が接地足の**支持多角形（足点の凸包）**の内側、または辺から margin 以内か。

    旧実装は「各足点を半径 margin の円で覆う」近似で、足点の集合との最近傍距離 ≤ margin を
    支持とみなしていた。これは脚幅が広い機種で破綻する: 両足の中間（＝バランスの取れた ZMP の
    定位置）がどの足点からも margin 超になり、**正しく立っているのに転倒判定**された
    （実証: H1 は股幅0.52mで足点 y=±0.26 → 中心の ZMP が全フレーム支持外、balance_viol=1.0)。
    正しくは凸包内なら距離0で支持、margin は接地点より外側（有限の足サイズ分）への許容とする。
    """
    pts = np.unique(np.round(np.asarray(foot_pts, dtype=float), 6), axis=0)
    if len(pts) == 1:
        return bool(np.linalg.norm(pts[0] - zmp_xy) <= margin)
    if len(pts) == 2:
        return bool(_dist_point_segment(zmp_xy, pts[0], pts[1]) <= margin)
    return bool(_dist_point_to_convex_polygon(zmp_xy, _convex_hull_2d(pts)) <= margin)


def _dist_point_segment(p: np.ndarray, a: np.ndarray, b: np.ndarray) -> float:
    """点 p と線分 ab の距離。"""
    ab = b - a
    denom = float(ab @ ab)
    t = 0.0 if denom == 0.0 else float(np.clip((p - a) @ ab / denom, 0.0, 1.0))
    return float(np.linalg.norm(p - (a + t * ab)))


def _convex_hull_2d(pts: np.ndarray) -> np.ndarray:
    """2D 点群の凸包を CCW 順で返す（Andrew monotone chain）。"""
    pts = pts[np.lexsort((pts[:, 1], pts[:, 0]))]

    def _cross(o: np.ndarray, a: np.ndarray, b: np.ndarray) -> float:
        return float((a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]))

    lower: list[np.ndarray] = []
    for pt in pts:
        while len(lower) >= 2 and _cross(lower[-2], lower[-1], pt) <= 0:
            lower.pop()
        lower.append(pt)
    upper: list[np.ndarray] = []
    for pt in pts[::-1]:
        while len(upper) >= 2 and _cross(upper[-2], upper[-1], pt) <= 0:
            upper.pop()
        upper.append(pt)
    return np.array(lower[:-1] + upper[:-1])


def _dist_point_to_convex_polygon(p: np.ndarray, hull: np.ndarray) -> float:
    """点 p と CCW 凸多角形 hull の距離（内側なら 0）。"""
    inside = True
    for i in range(len(hull)):
        a, b = hull[i], hull[(i + 1) % len(hull)]
        # CCW 凸包では内側の点は全辺の左側（cross ≥ 0）。
        if (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0]) < 0:
            inside = False
            break
    if inside:
        return 0.0
    return min(
        _dist_point_segment(p, hull[i], hull[(i + 1) % len(hull)]) for i in range(len(hull))
    )
