"""2-body contact sparring — 共有 MuJoCo arena で両者を PD 物理追従。

v0.158: ルートは underactuated（TrackingEnv と同様、free joint へトルクなし）だが
関節 PD + ``mj_step`` で limb 接触・反動が生じる。ヒット採点は kinematic 版と同じ幾何判定
（honest scope: 接触力は sim だがスコアは geometric のまま）。
"""

from __future__ import annotations

import math

import numpy as np

from robotdance_core.skeleton import index_of
from robotdance_retarget.embodiment import RobotMorphology
from robotdance_sim.fight_moves import effective_hit_radius


def _arena_ref_qpos(
    f: int,
    qa: np.ndarray,
    qb: np.ndarray,
    *,
    info: dict,
    separation: float,
    ha: float,
    hb: float,
    a_adr: list[int],
    b_adr: list[int],
    sa_adr: list[int],
    sb_adr: list[int],
    nq: int,
) -> np.ndarray:
    """単体 qpos 列から arena 目標 qpos を 1 フレーム分構築する。"""
    q180 = np.array([math.cos(math.pi / 2), 0, 0, math.sin(math.pi / 2)])
    ref = np.zeros(nq, dtype=np.float64)
    ref[info["q_a_adr"]:info["q_a_adr"] + 3] = [-separation, 0, ha]
    ref[info["q_a_adr"] + 3:info["q_a_adr"] + 7] = [1, 0, 0, 0]
    ref[info["q_b_adr"]:info["q_b_adr"] + 3] = [separation, 0, hb]
    ref[info["q_b_adr"] + 3:info["q_b_adr"] + 7] = q180
    for k in range(18):
        ref[a_adr[k]:a_adr[k] + 4] = qa[f, sa_adr[k]:sa_adr[k] + 4]
        ref[b_adr[k]:b_adr[k] + 4] = qb[f, sb_adr[k]:sb_adr[k] + 4]
    return ref


def _keypoints_from_arena(
    model,
    data,
    prefix: str,
    morph: RobotMorphology,
    endpoint: dict[int, np.ndarray],
) -> np.ndarray:
    """arena 上の prefix ロボットから canonical keypoints を復元する [J, 3]。"""
    from robotdance_core.skeleton import JOINT_NAMES

    kp = np.zeros((len(JOINT_NAMES), 3), dtype=np.float64)
    kp[0] = data.xpos[model.body(f"{prefix}root").id]
    for j in range(1, len(JOINT_NAMES)):
        bid = model.body(f"{prefix}body_{j}").id
        rmat = data.xmat[bid].reshape(3, 3)
        kp[j] = data.xpos[bid] + rmat @ endpoint[j]
    return kp


def _upright_root(model, data, root_name: str) -> float:
    jid = model.joint(root_name).id
    adr = model.jnt_qposadr[jid]
    w, x, y, _z = data.qpos[adr + 3:adr + 7]
    return float(1.0 - 2.0 * (x * x + y * y))


def play_sparring(
    model,
    data,
    ma,
    mb,
    qa: np.ndarray,
    qb: np.ndarray,
    info: dict,
    separation: float,
    morph_a: RobotMorphology,
    morph_b: RobotMorphology,
    style_cfg: dict,
    fps: float,
    width: int,
    height: int,
    render: bool,
) -> tuple[int, int, int, int, list, list[int], list[int], float, float]:
    """共有 arena で PD-only 2 体 sparring を再生し採点する。

    返り値: p1_hits, p2_hits, p1_body, p2_body, frames, p1_cum, p2_cum, p1_survival, p2_survival
    """
    import mujoco

    from robotdance_core.skeleton import JOINT_NAMES, PARENTS

    n = min(qa.shape[0], qb.shape[0])
    strikers = tuple(index_of(j) for j in style_cfg["strikers"])
    base_r = float(style_cfg["base_radius"])
    head_i = index_of("head")
    body_targets = (index_of("chest"), index_of("spine"))
    targets = (head_i, *body_targets)

    a_adr = [model.joint(f"a_jnt_{j}").qposadr[0] for j in range(1, 19)]
    b_adr = [model.joint(f"b_jnt_{j}").qposadr[0] for j in range(1, 19)]
    sa_adr = [ma.joint(f"jnt_{j}").qposadr[0] for j in range(1, 19)]
    sb_adr = [mb.joint(f"jnt_{j}").qposadr[0] for j in range(1, 19)]

    ha, hb = morph_a.nominal_height, morph_b.nominal_height
    sd_a, sd_b = morph_a.sim_defaults, morph_b.sim_defaults
    kp = 0.5 * (sd_a.kp + sd_b.kp)
    kd = 0.5 * (sd_a.kd + sd_b.kd)
    torque_limit = min(sd_a.torque_limit, sd_b.torque_limit)
    dt = 1.0 / fps
    n_sub = max(1, round(dt / model.opt.timestep))

    # free joint DOF にはトルクを掛けない（TrackingEnv と同じ underactuated base）。
    torque_mask = np.ones(model.nv, dtype=bool)
    for jname in ("a_root", "b_root"):
        adr = model.jnt_dofadr[model.joint(jname).id]
        torque_mask[adr:adr + 6] = False
    torque_cap = np.full(model.nv, float(torque_limit), dtype=np.float64)

    rest_a = morph_a.rest_pose
    rest_b = morph_b.rest_pose
    ep_a = {j: rest_a[j] - rest_a[PARENTS[j]] for j in range(1, len(JOINT_NAMES))}
    ep_b = {j: rest_b[j] - rest_b[PARENTS[j]] for j in range(1, len(JOINT_NAMES))}

    fall_a = 0.5 * ha
    fall_b = 0.5 * hb

    renderer = None
    cam = None
    if render:
        renderer = mujoco.Renderer(model, height, width)
        cam = mujoco.MjvCamera()
        mujoco.mjv_defaultCamera(cam)
        cam.distance = 2.6
        cam.azimuth = 90
        cam.elevation = -8
        cam.lookat = [0, 0, 1.0]

    zw = np.array([1.0, 1.0, 0.5])

    def _strike(att, dfn, fr, att_h, def_h):
        best = 1e9
        best_body = False
        for w in strikers:
            for tg in targets:
                d = float(np.linalg.norm((att[fr, w] - dfn[fr, tg]) * zw))
                is_body = tg in body_targets
                if d < best:
                    best, best_body = d, is_body
        return best, best_body

    # 初期姿勢
    data.qpos[:] = _arena_ref_qpos(
        0, qa, qb, info=info, separation=separation, ha=ha, hb=hb,
        a_adr=a_adr, b_adr=b_adr, sa_adr=sa_adr, sb_adr=sb_adr, nq=model.nq,
    )
    data.qvel[:] = 0.0
    mujoco.mj_forward(model, data)

    p1_hits = p2_hits = 0
    p1_body = p2_body = 0
    a_cd = b_cd = 0
    frames: list = []
    p1_cum: list[int] = []
    p2_cum: list[int] = []
    p1_alive = p2_alive = True
    p1_surv_frames = p2_surv_frames = 0

    ak_a = np.zeros((n, len(JOINT_NAMES), 3), dtype=np.float64)
    ak_b = np.zeros_like(ak_a)

    for f in range(n):
        if p1_alive:
            p1_surv_frames += 1
        if p2_alive:
            p2_surv_frames += 1

        ak_a[f] = _keypoints_from_arena(model, data, "a_", morph_a, ep_a)
        ak_b[f] = _keypoints_from_arena(model, data, "b_", morph_b, ep_b)

        a_dist, a_body = _strike(ak_a, ak_b, f, ha, hb)
        b_dist, b_body = _strike(ak_b, ak_a, f, hb, ha)
        a_lim = effective_hit_radius(base_r, ha, hb, body_target=a_body)
        b_lim = effective_hit_radius(base_r, hb, ha, body_target=b_body)
        a_cd = max(0, a_cd - 1)
        b_cd = max(0, b_cd - 1)
        if a_dist < a_lim and a_cd == 0 and p1_alive:
            p1_hits += 1
            if a_body:
                p1_body += 1
            a_cd = int(0.4 * fps)
        if b_dist < b_lim and b_cd == 0 and p2_alive:
            p2_hits += 1
            if b_body:
                p2_body += 1
            b_cd = int(0.4 * fps)
        p1_cum.append(p1_hits)
        p2_cum.append(p2_hits)

        if render:
            renderer.update_scene(data, cam)
            frames.append(renderer.render().copy())

        if f >= n - 1:
            break

        target = _arena_ref_qpos(
            f + 1, qa, qb, info=info, separation=separation, ha=ha, hb=hb,
            a_adr=a_adr, b_adr=b_adr, sa_adr=sa_adr, sb_adr=sb_adr, nq=model.nq,
        )
        err = np.zeros(model.nv, dtype=np.float64)
        mujoco.mj_differentiatePos(model, err, 1.0, data.qpos, target)
        tau = kp * err - kd * data.qvel
        tau = np.clip(tau, -torque_cap, torque_cap)
        tau[~torque_mask] = 0.0
        data.qfrc_applied[:] = tau
        for _ in range(n_sub):
            mujoco.mj_step(model, data)

        # 転倒判定（各 root）
        a_z = float(data.qpos[info["q_a_adr"] + 2])
        b_z = float(data.qpos[info["q_b_adr"] + 2])
        if p1_alive and (a_z < fall_a or _upright_root(model, data, "a_root") < 0.3):
            p1_alive = False
        if p2_alive and (b_z < fall_b or _upright_root(model, data, "b_root") < 0.3):
            p2_alive = False

    if renderer is not None:
        del renderer

    p1_survival = round(p1_surv_frames / max(n, 1), 3)
    p2_survival = round(p2_surv_frames / max(n, 1), 3)
    return (
        p1_hits, p2_hits, p1_body, p2_body, frames, p1_cum, p2_cum,
        p1_survival, p2_survival,
    )


__all__ = ["play_sparring"]
