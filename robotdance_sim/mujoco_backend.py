"""MuJoCo を使った物理ベースの feasibility 検証（v0）。

受動ヒューマノイドはバランス制御なしでは何でも倒れるため、forward sim は判別力を持たない。
本バックエンドは **参照運動そのものが物理的に実現可能か** を MuJoCo の動力学で検証する:

  1. keypoints を ball-joint 多体モデルの qpos に厳密復元
  2. 逆動力学（mj_inverse）で各 joint の必要トルク → torque saturation
  3. 質量モデルの COM → ZMP を計算し、接地足の支持多角形を外れる/滞空で balance violation

⚠️ v0 の質量・慣性は近似（bone 長比）であり実機値ではない。出力 sim_certificate は
"physically informed feasibility" であって実機保証ではない。
"""

from __future__ import annotations

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


def _pose_to_qpos(model, morphology: RobotMorphology, kps: np.ndarray) -> np.ndarray:
    """1 フレームの keypoints [J, 3] を qpos に厳密復元する。"""
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


def simulate_certificate(
    motion: RdMotion,
    morphology: RobotMorphology,
    *,
    total_mass: float | None = None,
    torque_limit: float | None = None,
    support_margin: float = 0.05,
) -> dict[str, Any]:
    """RD-Motion を MuJoCo 物理で検証し sim_certificate dict を返す。

    質量・トルク上限は embodiment 固有の既定（morphology.sim_defaults）から取る。
    caller が明示すればそれを優先。旧実装は G1 値（35kg / 80N·m）をハードコードしており、
    H1（47kg / 160N·m）の certify でも G1 のトルク上限で torque_ratio を判定していた
    （= v0.27 の SimDefaults を導入したのにこの経路だけ配線漏れ）。

    support_margin: ZMP が支持多角形（実フットプリント矩形の凸包）の外へ許容される距離（m）。
        旧既定 0.12 は支持多角形に足の横幅が無かった分（半幅~0.04）を margin で誤魔化していた。
        本実装は足幅を実フットプリントとして明示するので、margin は純粋なスラック（ZMP 推定誤差＋
        未モデルの踵 ~0.05）に縮小（実測: 安定なダンスの ZMP は足面から最大 4.4mm）。
    """
    import mujoco

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

    # 各フレームの qpos / COM / 重力保持トルク。
    qpos = np.stack([_pose_to_qpos(model, morphology, kps[f]) for f in range(n)])
    com = np.zeros((n, 3))
    per_frame_torque = []
    for f in range(n):
        data.qpos[:] = qpos[f]
        data.qvel[:] = 0
        mujoco.mj_forward(model, data)
        com[f] = data.subtree_com[root_id]
        # joint j の重力保持トルク = m_subtree·g·(joint anchor と subtree COM の水平距離)。
        # mj_inverse の ball-joint 特異性を避ける robust な解析計算。
        # 負荷率 = 必要トルク / その関節の実 actuator 上限。最大「率」の関節が律速（強弱を区別）。
        tmax = 0.0       # 絶対トルク最大（報告用）
        rmax = 0.0       # 負荷率最大（判定用）
        for bid in grav_bodies:
            anchor = data.xpos[bid]
            csub = data.subtree_com[bid]
            d_horiz = float(np.hypot(csub[0] - anchor[0], csub[1] - anchor[1]))
            tau_j = sub_mass[bid] * _G * d_horiz
            tmax = max(tmax, tau_j)
            rmax = max(rmax, tau_j / body_torque_limit[bid])
        per_frame_torque.append((tmax, rmax))
    gravity_torque = float(np.max([t for t, _ in per_frame_torque])) if per_frame_torque else 0.0
    torque_ratio = float(np.max([r for _, r in per_frame_torque])) if per_frame_torque else 0.0

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

    reasons: list[str] = []
    if airborne_ratio > 0.1:
        reasons.append(f"airborne {airborne_ratio:.0%}（接地なしで支持不能）")
    if balance_violation_ratio > 0.3:
        reasons.append(f"ZMP が支持多角形外 {balance_violation_ratio:.0%}（転倒リスク）")
    if torque_ratio > 1.0:
        reasons.append(f"重力保持トルク ×{torque_ratio:.2f}（actuator 限界超過）")
    if max_joint_ang_speed > 30.0:
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
        "torque_ratio": round(torque_ratio, 3),
        "max_joint_ang_speed_rad_s": round(max_joint_ang_speed, 2),
    }
    if flexion_violation is not None:
        metrics["joint_flexion_violation_ratio"] = round(flexion_violation, 3)
    return {
        "backend": "mujoco",
        "mujoco_version": mujoco.__version__,
        "approximate_inertia": True,
        "passed": passed,
        "verdict": "PASS" if passed else "REJECT",
        "metrics": metrics,
        "thresholds": {
            "airborne_ratio": 0.1,
            "balance_violation_ratio": 0.3,
            "gravity_torque_ratio": 1.0,
            "max_joint_ang_speed_rad_s": 30.0,
            "joint_flexion_violation_ratio": 0.0,
        },
        "reasons": reasons,
        "note": (
            "physically-informed feasibility（近似質量, v0）— 実機保証ではない。動的（転倒/トルク/"
            "滞空/角速度）＋運動学的（関節可動域）の両 feasibility を統合。gravity_torque は subtree"
            " COM から解析計算（mj_inverse の ball-joint 特異性を回避した robust 値）。"
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
