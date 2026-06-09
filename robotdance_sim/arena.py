"""HumanoidBattle arena — 2 体を MuJoCo シーンで対面させ、ボクシング動作を物理エンジン上で再生し、
拳が相手の頭/胴に届いたかを幾何で判定する「実際に殴り合う」GIF を作る。

⚠️ 設計（正直な範囲）: 完全 forward dynamics で動かすと、バランス制御は v0 未解決のため両者倒れる
（[[real-video-demo-pipeline]] の depth/balance frontier）。そこで本 arena は **kinematic playback**:
毎フレーム両者の qpos を retarget 結果から設定して `mj_forward`（FK＋衝突検出は走るが時間積分しない＝
倒れない）。ヒットは MuJoCo 衝突系ではなく、**拳(手首)と相手の頭/胸の幾何距離**で判定（堅牢・調整可）。
「振り付けされたボクシングを実物理エンジンの 3D で描き、ヒットは幾何で採点」——接触ダイナミクス
（打撃の反動で相手がよろける）は v0 では出さない。これは honest な妥協で docstring に明示する。

スコア: 各ファイターが相手にクリーンヒットさせた回数（同一パンチは cooldown で 1 ヒット）。多い方が勝者。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from robotdance_core.rd_mir import RdMir
from robotdance_core.skeleton import index_of

_FPS = 30.0

from robotdance_sim.fight_moves import (  # noqa: E402
    STYLE_CONFIG,
    SYNTH_FIGHT_GENERATORS,
    effective_hit_radius,
    generate_boxing,
    generate_dodge,
    generate_hook,
    generate_kick,
)

_Z180 = np.array([[-1.0, 0, 0], [0, -1.0, 0], [0, 0, 1.0]])  # z 軸 180° 回転（対面）。


@dataclass
class FightResult:
    p1: str
    p2: str
    p1_hits: int
    p2_hits: int
    winner: str
    frames: list = field(repr=False, default_factory=list)
    fps: float = _FPS
    p1_cum: list = field(repr=False, default_factory=list)  # 各フレーム時点の累積ヒット
    p2_cum: list = field(repr=False, default_factory=list)
    assisted_corner: str | None = None  # "p1" | "p2" — 物理追従コーナー
    assisted_mode: str | None = None  # "pd" | "rl"
    assisted_survival: float | None = None
    sparring: bool = False  # 2 体同時 PD 物理（接触あり）
    p1_survival: float | None = None
    p2_survival: float | None = None


def _single_model(morph):
    import mujoco

    from .mjcf import build_mjcf
    return mujoco.MjModel.from_xml_string(build_mjcf(morph, ground=False))


def _arena_kps(robot_kps: np.ndarray, R: np.ndarray, stance_xy: np.ndarray) -> np.ndarray:
    """robot kps[T,J,3] を pelvis 基準へ中心化→R 回転→足が地面 z=0 に来るよう配置した arena 座標。"""
    pel = robot_kps[:, index_of("pelvis"), :][:, None, :]
    centered = (robot_kps - pel) @ R.T
    z0 = centered[:, :, 2].min()
    out = centered.copy()
    out[:, :, 0] += stance_xy[0]
    out[:, :, 1] += stance_xy[1]
    out[:, :, 2] += -z0
    return out


def motion_for_style(style: str, *, duration: float = 4.0, fps: float = _FPS, lead: str = "left") -> RdMir:
    """fight / arena 用の RD-MIR を返す（合成 fight 技 or 実動画フィクスチャ）。"""
    if style == "boxing":
        return generate_boxing(duration=duration, fps=fps, lead=lead)
    if style in SYNTH_FIGHT_GENERATORS:
        return SYNTH_FIGHT_GENERATORS[style](duration=duration, fps=fps)
    from robotdance_benchmarks.real_motions import get_real_motion
    return get_real_motion(style)


def _motion_pair(style: str, *, duration: float, fps: float) -> tuple[RdMir, RdMir]:
    if style == "boxing":
        return (
            generate_boxing(duration=duration, fps=fps, lead="left"),
            generate_boxing(duration=duration, fps=fps, lead="right"),
        )
    mir = motion_for_style(style, duration=duration, fps=fps)
    return mir, mir


def _assisted_trajectory(
    morph, reference, n: int, *, mode: str = "pd", rl_iterations: int = 20,
) -> tuple[np.ndarray, np.ndarray, float]:
    """物理追従 → robot kps [T,J,3]、単体 qpos [T,nq]、survival ratio。"""
    from robotdance_sim.assisted_playback import rollout_pd_only, rollout_rl

    if mode == "rl":
        result = rollout_rl(reference, morph, iterations=rl_iterations)
    else:
        result = rollout_pd_only(reference, morph)
    kps = result.keypoints
    if kps.shape[0] < n:
        kps = np.concatenate([kps, np.repeat(kps[-1:], n - kps.shape[0], axis=0)], axis=0)
    else:
        kps = kps[:n]
    ma = _single_model(morph)
    qa = _poses_to_qpos_arena(ma, morph, kps)
    return kps, qa, result.survival_ratio


def run_fight(morph_a, morph_b, *, name_a: str, name_b: str, separation: float = 0.17,
              hit_radius: float = 0.20, duration: float = 4.0, fps: float = _FPS,
              style: str = "boxing", width: int = 480, height: int = 360,
              render: bool = True, mesh: bool = False,
              urdf_a: str | None = None, urdf_b: str | None = None,
              depth_refine: bool = False,
              retarget_backend: str = "kinematic",
              assisted: str | None = None,
              assisted_mode: str = "pd",
              rl_iterations: int = 20,
              sparring: bool = False) -> FightResult:
    """2 体を対面させ motion を再生し、拳→相手頭/胸の幾何ヒットを採点して GIF フレームを返す。

    style: `boxing`/`hook`/`kick`/`dodge` または `karate`/`kathak`（実動画）。
    mesh: True で pybullet 実 URDF メッシュ描画（ヒット判定は MuJoCo 幾何のまま）。
    retarget_backend: "kinematic" または "gmr"（`retarget --backend gmr` と同系）。
    assisted: "p1" または "p2" — 指定コーナーだけ物理追従、相手は kinematic のまま。
    assisted_mode: "pd"（残差ゼロ）または "rl"（PPO tracking）。
    sparring: True で両者を共有 arena 上で PD 物理追従（limb 接触あり）。assisted と併用不可。
    """
    if sparring and assisted:
        raise ValueError("sparring と assisted は併用できません（v0.158）")
    import mujoco

    from robotdance_retarget.dispatch import check_retarget_backend_for_robots, retarget_with_backend

    check_retarget_backend_for_robots([name_a, name_b], retarget_backend)
    cfg = STYLE_CONFIG.get(style, STYLE_CONFIG["boxing"])

    # 1. 各ファイターの motion → robot kps → 単体 qpos。
    box_a, box_b = _motion_pair(style, duration=duration, fps=fps)
    if depth_refine:
        from robotdance_motion.fight_refinement import refine_for_fight

        box_a = refine_for_fight(box_a)
        box_b = refine_for_fight(box_b)
    fps = float(box_a.fps)
    motion_a = retarget_with_backend(box_a, morph_a, retarget_backend)
    motion_b = retarget_with_backend(box_b, morph_b, retarget_backend)
    rk_a = motion_a.keypoints_3d_array()
    rk_b = motion_b.keypoints_3d_array()
    n = min(rk_a.shape[0], rk_b.shape[0])
    rk_a, rk_b = rk_a[:n], rk_b[:n]

    ma, mb = _single_model(morph_a), _single_model(morph_b)
    assisted_survival = None
    track_mode = assisted_mode if assisted else None
    if assisted == "p1":
        rk_a, qa, assisted_survival = _assisted_trajectory(
            morph_a, motion_a, n, mode=assisted_mode, rl_iterations=rl_iterations,
        )
        qb = _poses_to_qpos_arena(mb, morph_b, rk_b)
    elif assisted == "p2":
        rk_b, qb, assisted_survival = _assisted_trajectory(
            morph_b, motion_b, n, mode=assisted_mode, rl_iterations=rl_iterations,
        )
        qa = _poses_to_qpos_arena(ma, morph_a, rk_a)
    else:
        qa = _poses_to_qpos_arena(ma, morph_a, rk_a)
        qb = _poses_to_qpos_arena(mb, morph_b, rk_b)

    ak_a = _arena_kps(rk_a, np.eye(3), np.array([-separation, 0.0]))
    ak_b = _arena_kps(rk_b, _Z180, np.array([+separation, 0.0]))

    # 2. arena 組み立て（MjSpec.attach, ライト+コーナーカラー）。
    model, info = _build_arena(morph_a, morph_b, separation, ak_a, ak_b)
    data = mujoco.MjData(model)

    p1_survival = p2_survival = None
    if sparring:
        from .sparring import play_sparring

        (
            p1_hits, p2_hits, p1_body, p2_body, frames, p1_cum, p2_cum,
            p1_survival, p2_survival,
        ) = play_sparring(
            model, data, ma, mb, qa, qb, info, separation,
            morph_a, morph_b, cfg, fps, width, height, render and not mesh,
        )
    else:
        # 3. ヒット判定（striker→的の幾何距離, 体格差で reach/precision 補正）。
        p1_hits, p2_hits, p1_body, p2_body, frames, p1_cum, p2_cum = _play_and_score(
            model, data, ma, mb, qa, qb, ak_a, ak_b, info, separation,
            morph_a, morph_b, cfg, fps, width, height, render and not mesh)

    if mesh and (assisted or sparring):
        mesh = False  # assisted/sparring は MuJoCo カプセル描画のみ（v0.148）

    if mesh:
        from pathlib import Path

        from .mesh_render import render_fight_mesh, resolve_unitree_urdf

        path_a = Path(urdf_a) if urdf_a else resolve_unitree_urdf(name_a)
        path_b = Path(urdf_b) if urdf_b else resolve_unitree_urdf(name_b)
        mesh_sep = max(separation, 0.45)  # 実メッシュは幅があるため間合いを広げる
        import pybullet as p
        p.connect(p.DIRECT)
        try:
            frames = render_fight_mesh(
                box_a, box_b, robot_a=name_a, robot_b=name_b,
                urdf_a=path_a, urdf_b=path_b, separation=mesh_sep, n_frames=n,
                width=width, height=height, stride=2)
        finally:
            p.disconnect()

    if p1_hits > p2_hits:
        winner = name_a
    elif p2_hits > p1_hits:
        winner = name_b
    else:
        winner = _resolve_draw(
            name_a, name_b, morph_a, morph_b, p1_body, p2_body,
        )
    return FightResult(
        name_a, name_b, p1_hits, p2_hits, winner, frames, fps, p1_cum, p2_cum,
        assisted_corner=assisted, assisted_mode=track_mode,
        assisted_survival=assisted_survival,
        sparring=sparring, p1_survival=p1_survival, p2_survival=p2_survival,
    )


def _resolve_draw(name_a, name_b, morph_a, morph_b, body_a: int, body_b: int) -> str:
    """同点時: ボディヒット多い方（compact precision）、それでも同点なら背が高い方（reach）。"""
    if body_a > body_b:
        return name_a
    if body_b > body_a:
        return name_b
    ha, hb = morph_a.nominal_height, morph_b.nominal_height
    if ha > hb:
        return name_a
    if hb > ha:
        return name_b
    return "DRAW"


def _poses_to_qpos_arena(single_model, morph, robot_kps: np.ndarray) -> np.ndarray:
    from .mujoco_backend import _poses_to_qpos
    return _poses_to_qpos(single_model, morph, robot_kps)


def _build_arena(morph_a, morph_b, separation, ak_a, ak_b):
    """2 体を対面配置した MuJoCo モデルを MjSpec.attach で生成（ライト+赤/青コーナー）。"""
    import mujoco

    from .mjcf import build_mjcf

    spec = mujoco.MjSpec()
    spec.worldbody.add_geom(type=mujoco.mjtGeom.mjGEOM_PLANE, size=[5, 5, 0.1],
                            rgba=[0.55, 0.57, 0.6, 1.0], name="ground")
    spec.worldbody.add_light(pos=[0, 0, 4.0], dir=[0, 0, -1])
    spec.worldbody.add_light(pos=[2.0, 2.0, 3.0], dir=[-0.5, -0.5, -1])
    q180 = [math.cos(math.pi / 2), 0, 0, math.sin(math.pi / 2)]
    for pfx, morph, pos, quat in (
        ("a_", morph_a, [-separation, 0, 0], [1, 0, 0, 0]),
        ("b_", morph_b, [separation, 0, 0], q180),
    ):
        child = mujoco.MjSpec.from_string(build_mjcf(morph, ground=False))
        fr = spec.worldbody.add_frame(pos=pos, quat=quat)
        spec.attach(child, prefix=pfx, frame=fr)
    model = spec.compile()
    # コーナーカラー: a_=赤, b_=青。
    for g in range(model.ngeom):
        bname = model.body(model.geom_bodyid[g]).name
        if bname.startswith("a_"):
            model.geom_rgba[g] = [0.85, 0.22, 0.22, 1.0]
        elif bname.startswith("b_"):
            model.geom_rgba[g] = [0.22, 0.4, 0.9, 1.0]
    info = {"q_a_adr": model.joint("a_root").qposadr[0],
            "q_b_adr": model.joint("b_root").qposadr[0]}
    return model, info


def _play_and_score(model, data, ma, mb, qa, qb, ak_a, ak_b, info, separation,
                    morph_a, morph_b, style_cfg, fps, width, height, render):
    import mujoco

    n = min(qa.shape[0], qb.shape[0])
    strikers = tuple(index_of(j) for j in style_cfg["strikers"])
    base_r = float(style_cfg["base_radius"])
    head_i = index_of("head")
    body_targets = (index_of("chest"), index_of("spine"))
    targets = (head_i, *body_targets)
    # 各 fighter の ball-joint quat を arena qpos に書くためのアドレス対応。
    a_adr = [model.joint(f"a_jnt_{j}").qposadr[0] for j in range(1, 19)]
    b_adr = [model.joint(f"b_jnt_{j}").qposadr[0] for j in range(1, 19)]
    sa_adr = [ma.joint(f"jnt_{j}").qposadr[0] for j in range(1, 19)]
    sb_adr = [mb.joint(f"jnt_{j}").qposadr[0] for j in range(1, 19)]

    # 立ち高さ（足が地面 z=0 へ来るよう root z を上げる）。
    ha = float(-_centered_min_z(ma, qa))
    hb = float(-_centered_min_z(mb, qb))

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
    ha, hb = morph_a.nominal_height, morph_b.nominal_height

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

    p1_hits = p2_hits = 0
    p1_body = p2_body = 0
    a_cd = b_cd = 0
    frames = []
    p1_cum: list[int] = []
    p2_cum: list[int] = []
    q180 = np.array([math.cos(math.pi / 2), 0, 0, math.sin(math.pi / 2)])
    for f in range(n):
        # arena qpos を構成（root を stance に固定, ball joint をコピー）。
        data.qpos[info["q_a_adr"]:info["q_a_adr"] + 3] = [-separation, 0, ha]
        data.qpos[info["q_a_adr"] + 3:info["q_a_adr"] + 7] = [1, 0, 0, 0]
        data.qpos[info["q_b_adr"]:info["q_b_adr"] + 3] = [separation, 0, hb]
        data.qpos[info["q_b_adr"] + 3:info["q_b_adr"] + 7] = q180
        for k in range(18):
            data.qpos[a_adr[k]:a_adr[k] + 4] = qa[f, sa_adr[k]:sa_adr[k] + 4]
            data.qpos[b_adr[k]:b_adr[k] + 4] = qb[f, sb_adr[k]:sb_adr[k] + 4]
        mujoco.mj_forward(model, data)

        a_dist, a_body = _strike(ak_a, ak_b, f, ha, hb)
        b_dist, b_body = _strike(ak_b, ak_a, f, hb, ha)
        a_lim = effective_hit_radius(base_r, ha, hb, body_target=a_body)
        b_lim = effective_hit_radius(base_r, hb, ha, body_target=b_body)
        a_cd = max(0, a_cd - 1)
        b_cd = max(0, b_cd - 1)
        if a_dist < a_lim and a_cd == 0:
            p1_hits += 1
            if a_body:
                p1_body += 1
            a_cd = int(0.4 * fps)
        if b_dist < b_lim and b_cd == 0:
            p2_hits += 1
            if b_body:
                p2_body += 1
            b_cd = int(0.4 * fps)
        p1_cum.append(p1_hits)
        p2_cum.append(p2_hits)

        if render:
            renderer.update_scene(data, cam)
            frames.append(renderer.render().copy())
    if renderer is not None:
        del renderer
    return p1_hits, p2_hits, p1_body, p2_body, frames, p1_cum, p2_cum


def _centered_min_z(single_model, q: np.ndarray) -> float:
    """qpos[0] を単体モデルに与え root を原点に置いたときの最下 body z（立ち高さ補正用）。"""
    import mujoco

    d = mujoco.MjData(single_model)
    d.qpos[:] = q[0]
    d.qpos[0:3] = [0, 0, 0]
    d.qpos[3:7] = [1, 0, 0, 0]
    mujoco.mj_forward(single_model, d)
    return float(d.xpos[:, 2].min())


__all__ = [
    "generate_boxing", "generate_dodge", "generate_hook", "generate_kick",
    "motion_for_style", "run_fight", "FightResult",
]
