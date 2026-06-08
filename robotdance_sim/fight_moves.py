"""HumanoidBattle 用の合成 fight motion（boxing / hook / kick / dodge）。"""

from __future__ import annotations

import math

import numpy as np

from robotdance_core.rd_mir import RdMir, Skeleton
from robotdance_core.skeleton import JOINT_NAMES, PARENTS, index_of

_FPS = 30.0
_REF_HEIGHT = 1.45  # reach バランスの基準身長 [m]


def _timeline(duration: float, fps: float) -> tuple[int, np.ndarray, np.ndarray]:
    from robotdance_core.synthetic import _REST

    n = round(fps * duration)
    t = np.arange(n) / fps
    kps = np.repeat(_REST[None].astype(np.float64), n, axis=0)
    return n, t, kps


def _pulse(t: np.ndarray, center: float, width: float) -> np.ndarray:
    x = (t - center) / width
    return np.where(np.abs(x) < 1.0, np.cos(x * math.pi / 2) ** 2, 0.0)


def _mk_mir(kps: np.ndarray, fps: float, duration: float, action: str, mid: str) -> RdMir:
    return RdMir(
        motion_id=mid,
        source_ref={"dataset_name": "robotdance-synthetic", "generator": f"fight_moves.{action}"},
        license_state="redistributable",
        fps=fps,
        duration=duration,
        skeleton=Skeleton(joint_names=JOINT_NAMES, parents=PARENTS),
        root_trajectory={"position": kps[:, 0, :].tolist()},
        keypoints_3d=kps.tolist(),
        privacy_flags={"synthetic": True, "face_visible": False},
        semantics={"action_label": action, "style_tag": "synthetic_fight"},
        extractor_versions={"generator": "robotdance.fight_moves.v0"},
    )


def generate_boxing(*, duration: float = 4.0, fps: float = _FPS, lead: str = "left",
                    motion_id: str = "rdmir-synth-boxing-0001") -> RdMir:
    """ジャブ→クロス→フックのコンビ（立位・腕中心）。"""
    n, t, kps = _timeline(duration, fps)
    li, ri = index_of("left_wrist"), index_of("right_wrist")
    lei, rei = index_of("left_elbow"), index_of("right_elbow")
    guard = {
        li: np.array([0.16, 0.07, 1.36]), lei: np.array([0.10, 0.15, 1.20]),
        ri: np.array([0.16, -0.07, 1.36]), rei: np.array([0.10, -0.15, 1.20]),
    }
    ext = {
        li: np.array([0.62, 0.03, 1.34]), lei: np.array([0.34, 0.10, 1.30]),
        ri: np.array([0.62, -0.03, 1.34]), rei: np.array([0.34, -0.10, 1.30]),
    }
    period, left_lead = 1.6, lead == "left"
    left_p = np.zeros(n)
    right_p = np.zeros(n)
    for c in range(max(1, int(duration / period))):
        base = c * period + 0.3
        (left_p if left_lead else right_p)[:] += _pulse(t, base, 0.22)
        (right_p if left_lead else left_p)[:] += _pulse(t, base + 0.55, 0.22)
        (left_p if left_lead else right_p)[:] += _pulse(t, base + 1.05, 0.26)
    left_p = np.clip(left_p, 0, 1)
    right_p = np.clip(right_p, 0, 1)
    for f in range(n):
        for w, e, p in ((li, lei, left_p[f]), (ri, rei, right_p[f])):
            kps[f, w] = guard[w] * (1 - p) + ext[w] * p
            kps[f, e] = guard[e] * (1 - p) + ext[e] * p
        kps[f, :11, 0] += 0.02 * math.sin(2 * math.pi * t[f] / period)
    return _mk_mir(kps, fps, duration, "boxing", motion_id)


def generate_hook(*, duration: float = 4.0, fps: float = _FPS,
                  motion_id: str = "rdmir-synth-hook-0001") -> RdMir:
    """横方向に振るフック（y 成分大）。頭高さを狙う。"""
    n, t, kps = _timeline(duration, fps)
    li, ri = index_of("left_wrist"), index_of("right_wrist")
    lei, rei = index_of("left_elbow"), index_of("right_elbow")
    guard = {
        li: np.array([0.14, 0.12, 1.38]), lei: np.array([0.08, 0.18, 1.22]),
        ri: np.array([0.14, -0.12, 1.38]), rei: np.array([0.08, -0.18, 1.22]),
    }
    ext = {
        li: np.array([0.42, 0.38, 1.40]), lei: np.array([0.22, 0.28, 1.28]),
        ri: np.array([0.42, -0.38, 1.40]), rei: np.array([0.22, -0.28, 1.28]),
    }
    period = 1.4
    lp = np.zeros(n)
    rp = np.zeros(n)
    for c in range(max(1, int(duration / period))):
        base = c * period + 0.25
        lp[:] += _pulse(t, base, 0.24)
        rp[:] += _pulse(t, base + 0.65, 0.24)
    lp, rp = np.clip(lp, 0, 1), np.clip(rp, 0, 1)
    for f in range(n):
        for w, e, p in ((li, lei, lp[f]), (ri, rei, rp[f])):
            kps[f, w] = guard[w] * (1 - p) + ext[w] * p
            kps[f, e] = guard[e] * (1 - p) + ext[e] * p
    return _mk_mir(kps, fps, duration, "hook", motion_id)


def generate_kick(*, duration: float = 4.0, fps: float = _FPS,
                  motion_id: str = "rdmir-synth-kick-0001") -> RdMir:
    """前蹴り（足先を +x へ）。ヒット判定は足先で行う。"""
    n, t, kps = _timeline(duration, fps)
    lk, rk = index_of("left_knee"), index_of("right_knee")
    la, ra = index_of("left_ankle"), index_of("right_ankle")
    lf, rf = index_of("left_foot"), index_of("right_foot")
    stance = {
        lk: np.array([0.02, 0.10, 0.52]), la: np.array([0.04, 0.10, 0.10]), lf: np.array([0.15, 0.10, 0.06]),
        rk: np.array([0.02, -0.10, 0.52]), ra: np.array([0.04, -0.10, 0.10]), rf: np.array([0.15, -0.10, 0.06]),
    }
    kick_r = {
        rk: np.array([0.38, -0.10, 0.62]), ra: np.array([0.58, -0.10, 0.42]), rf: np.array([0.72, -0.10, 0.38]),
    }
    kick_l = {
        lk: np.array([0.38, 0.10, 0.62]), la: np.array([0.58, 0.10, 0.42]), lf: np.array([0.72, 0.10, 0.38]),
    }
    period = 1.8
    kp = np.zeros(n)
    for c in range(max(1, int(duration / period))):
        base = c * period + 0.35
        kp[:] += _pulse(t, base, 0.28)
        kp[:] += _pulse(t, base + 0.85, 0.28)
    kp = np.clip(kp, 0, 1)
    for f in range(n):
        phase = kp[f]
        use_left = (f // round(fps * period)) % 2 == 0
        for j in (lk, la, lf, rk, ra, rf):
            kps[f, j] = stance[j]
        if use_left:
            for j in (lk, la, lf):
                kps[f, j] = stance[j] * (1 - phase) + kick_l[j] * phase
        else:
            for j in (rk, ra, rf):
                kps[f, j] = stance[j] * (1 - phase) + kick_r[j] * phase
    return _mk_mir(kps, fps, duration, "kick", motion_id)


def generate_dodge(*, duration: float = 4.0, fps: float = _FPS,
                   motion_id: str = "rdmir-synth-dodge-0001") -> RdMir:
    """スリップ・ディフェンス（上体を後方/y にずらし、カウンターは短いジャブ）。"""
    n, t, kps = _timeline(duration, fps)
    spine, chest = index_of("spine"), index_of("chest")
    li, ri = index_of("left_wrist"), index_of("right_wrist")
    period = 1.2
    for f in range(n):
        slip = 0.12 * math.sin(2 * math.pi * t[f] / period)
        lean = -0.06 - 0.04 * abs(math.sin(2 * math.pi * t[f] / period))
        kps[f, spine, 0] += lean
        kps[f, chest, 0] += lean
        kps[f, :5, 1] += slip
        kps[f, li, 0] = 0.10 + lean
        kps[f, ri, 0] = 0.10 + lean
        kps[f, li, 2] = 1.42
        kps[f, ri, 2] = 1.42
        jab = _pulse(np.array([t[f]]), t[f] % period + 0.1, 0.15)[0]
        kps[f, li, 0] += 0.35 * jab
        kps[f, ri, 0] += 0.28 * jab
    return _mk_mir(kps, fps, duration, "dodge", motion_id)


SYNTH_FIGHT_GENERATORS = {
    "boxing": generate_boxing,
    "hook": generate_hook,
    "kick": generate_kick,
    "dodge": generate_dodge,
}

# style → ヒット判定パラメータ（striker 関節・基準半径）。
STYLE_CONFIG: dict[str, dict] = {
    "boxing": {"strikers": ("left_wrist", "right_wrist"), "base_radius": 0.20},
    "hook": {"strikers": ("left_wrist", "right_wrist"), "base_radius": 0.22},
    "kick": {"strikers": ("left_foot", "right_foot"), "base_radius": 0.19},
    "dodge": {"strikers": ("left_wrist", "right_wrist"), "base_radius": 0.17},
    "karate": {"strikers": ("left_wrist", "right_wrist"), "base_radius": 0.21},
    "kathak": {"strikers": ("left_wrist", "right_wrist"), "base_radius": 0.21},
}

FIGHT_STYLE_NAMES = frozenset({*SYNTH_FIGHT_GENERATORS, "karate", "kathak"})


def effective_hit_radius(
    base: float,
    att_height: float,
    def_height: float,
    *,
    body_target: bool,
) -> float:
    """体格差バランス: 高い=リーチ有利、低い=ボディへの精密さ。"""
    r = base * (att_height / _REF_HEIGHT) ** 0.35
    if att_height < def_height and body_target:
        r *= 1.10
    return r


__all__ = [
    "FIGHT_STYLE_NAMES",
    "STYLE_CONFIG",
    "SYNTH_FIGHT_GENERATORS",
    "effective_hit_radius",
    "generate_boxing",
    "generate_dodge",
    "generate_hook",
    "generate_kick",
]
