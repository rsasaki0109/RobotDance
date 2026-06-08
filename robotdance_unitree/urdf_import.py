"""URDF → RobotMorphology importer（v0）。

実機 URDF（例: Unitree g1_description）の zero-config FK でリンク世界位置を求め、canonical
19-joint の rest pose を実寸から構築する。これにより retarget / sim の**寸法が実物由来**になる
（手作りの近似プロポーションを脱却）。

⚠️ v0 の限界（正直に）:
  - URDF のリンク frame は関節位置であり解剖学的中心ではない。canonical の torso 連鎖
    （spine/chest/neck/head）と toe は、肩・骨盤・足首から **合成** する（URDF に該当リンクが無いため）。
  - ball-joint sim の質量は依然近似。アクチュエータ空間 retarget（実 G1 関節角への IK）は今後。
  - URDF / mesh は repo に含めない。利用者が各自取得する（g1_description 等）。
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Optional

import numpy as np
from scipy.spatial.transform import Rotation as Rot

from robotdance_core.skeleton import JOINT_NAMES, PARENTS, index_of
from robotdance_retarget.embodiment import RobotMorphology

# canonical limb joint → Unitree G1 (23dof) URDF link。torso 連鎖・toe は合成する。
G1_LINK_MAP = {
    "pelvis": "pelvis",
    "left_hip": "left_hip_pitch_link", "right_hip": "right_hip_pitch_link",
    "left_knee": "left_knee_link", "right_knee": "right_knee_link",
    "left_ankle": "left_ankle_pitch_link", "right_ankle": "right_ankle_pitch_link",
    "left_shoulder": "left_shoulder_pitch_link", "right_shoulder": "right_shoulder_pitch_link",
    "left_elbow": "left_elbow_link", "right_elbow": "right_elbow_link",
    "left_wrist": "left_wrist_roll_rubber_hand", "right_wrist": "right_wrist_roll_rubber_hand",
}

# canonical limb joint → Unitree H1 URDF link。H1（無印 h1.urdf）は腕が肘で終わり wrist link が
# 無いため wrist は省略（build_rest_pose が前腕を合成する）。
H1_LINK_MAP = {
    "pelvis": "pelvis",
    "left_hip": "left_hip_pitch_link", "right_hip": "right_hip_pitch_link",
    "left_knee": "left_knee_link", "right_knee": "right_knee_link",
    "left_ankle": "left_ankle_link", "right_ankle": "right_ankle_link",
    "left_shoulder": "left_shoulder_pitch_link", "right_shoulder": "right_shoulder_pitch_link",
    "left_elbow": "left_elbow_link", "right_elbow": "right_elbow_link",
}

# canonical limb joint → Unitree H2 URDF link（H2.urdf）。H2 は ankle が roll+pitch の 2 link、
# wrist は roll/pitch/yaw を持つ。FK ターゲットは各 limb の遠位 link（足首は pitch、手首は yaw）。
H2_LINK_MAP = {
    "pelvis": "pelvis",
    "left_hip": "left_hip_pitch_link", "right_hip": "right_hip_pitch_link",
    "left_knee": "left_knee_link", "right_knee": "right_knee_link",
    "left_ankle": "left_ankle_pitch_link", "right_ankle": "right_ankle_pitch_link",
    "left_shoulder": "left_shoulder_pitch_link", "right_shoulder": "right_shoulder_pitch_link",
    "left_elbow": "left_elbow_link", "right_elbow": "right_elbow_link",
    "left_wrist": "left_wrist_yaw_link", "right_wrist": "right_wrist_yaw_link",
}


# canonical 関節 → その limb に属する actuated URDF 関節名の prefix。実機は 1 canonical
# ball joint に複数 DOF（hip=pitch/roll/yaw 等）が対応するため、その limb の全 DOF を envelope
# 集約して canonical 1 関節の概略 limit にする（v0 ball-joint 近似に対する正直な要約）。
_CANONICAL_ACTUATOR_PREFIX: dict[str, tuple[str, ...]] = {
    "left_hip": ("left_hip",), "right_hip": ("right_hip",),
    "left_knee": ("left_knee",), "right_knee": ("right_knee",),
    "left_ankle": ("left_ankle",), "right_ankle": ("right_ankle",),
    "left_shoulder": ("left_shoulder",), "right_shoulder": ("right_shoulder",),
    "left_elbow": ("left_elbow",), "right_elbow": ("right_elbow",),
    "left_wrist": ("left_wrist",), "right_wrist": ("right_wrist",),
    "spine": ("waist", "torso"),  # 胴体 yaw（G1: waist_yaw / H1: torso）
}


def parse_actuated_limits(path: str | Path) -> dict[str, dict[str, object]]:
    """URDF の revolute 関節ごとに {position[lo,hi], velocity, torque} を返す（actuated 名がキー）。"""
    root = ET.parse(Path(path)).getroot()
    out: dict[str, dict[str, object]] = {}
    for j in root.findall("joint"):
        if j.get("type") != "revolute":
            continue
        lim = j.find("limit")
        if lim is None or lim.get("lower") is None:
            continue
        out[j.get("name")] = {
            "position": [float(lim.get("lower")), float(lim.get("upper"))],
            "velocity": float(lim.get("velocity", 0.0)),
            "torque": float(lim.get("effort", 0.0)),
        }
    return out


def canonical_joint_limits(actuated: dict[str, dict[str, object]]) -> dict[str, dict[str, object]]:
    """actuated 関節 limit を canonical 関節へ envelope 集約する。

    位置は limb 内 DOF の最広レンジ [min lower, max upper]、速度・トルクは最も厳しい（min）値を採る
    （feasibility を過大評価しない保守側）。actuator が 1 つも無い canonical は省く。
    """
    out: dict[str, dict[str, object]] = {}
    for canon, prefixes in _CANONICAL_ACTUATOR_PREFIX.items():
        dofs = [v for name, v in actuated.items()
                if any(name.startswith(p + "_") for p in prefixes)]
        if not dofs:
            continue
        out[canon] = {
            "position": [round(min(d["position"][0] for d in dofs), 4),
                         round(max(d["position"][1] for d in dofs), 4)],
            "velocity": round(min(float(d["velocity"]) for d in dofs), 2),
            "torque": round(min(float(d["torque"]) for d in dofs), 2),
        }
    return out


def parse_urdf(path: str | Path) -> tuple[dict[str, tuple[str, np.ndarray, np.ndarray]], str]:
    """URDF を読み、child_link → (parent_link, origin_xyz, origin_rpy) と root link を返す。"""
    root = ET.parse(Path(path)).getroot()
    joints: dict[str, tuple[str, np.ndarray, np.ndarray]] = {}
    children = set()
    parents = set()
    for j in root.findall("joint"):
        o = j.find("origin")
        xyz = _vec(o.get("xyz") if o is not None else None)
        rpy = _vec(o.get("rpy") if o is not None else None)
        parent = j.find("parent").get("link")
        child = j.find("child").get("link")
        joints[child] = (parent, xyz, rpy)
        children.add(child)
        parents.add(parent)
    root_link = next(iter(parents - children), "pelvis")
    return joints, root_link


def link_world_positions(
    joints: dict[str, tuple[str, np.ndarray, np.ndarray]], root_link: str
) -> dict[str, np.ndarray]:
    """zero-config（全関節 0）での各リンク frame の世界位置を FK で求める。"""
    cache: dict[str, tuple[np.ndarray, np.ndarray]] = {root_link: (np.zeros(3), np.eye(3))}

    def world(link: str) -> tuple[np.ndarray, np.ndarray]:
        if link in cache:
            return cache[link]
        parent, xyz, rpy = joints[link]
        pp, pr = world(parent)
        cache[link] = (pp + pr @ xyz, pr @ Rot.from_euler("xyz", rpy).as_matrix())
        return cache[link]

    return {link: world(link)[0] for link in list(joints) + [root_link]}


def link_world_frames(
    joints: dict[str, tuple[str, np.ndarray, np.ndarray]], root_link: str
) -> dict[str, tuple[np.ndarray, np.ndarray]]:
    """zero-config での各リンク frame の世界 (位置, 回転) を FK で求める。"""
    cache: dict[str, tuple[np.ndarray, np.ndarray]] = {root_link: (np.zeros(3), np.eye(3))}

    def world(link: str) -> tuple[np.ndarray, np.ndarray]:
        if link in cache:
            return cache[link]
        parent, xyz, rpy = joints[link]
        pp, pr = world(parent)
        cache[link] = (pp + pr @ xyz, pr @ Rot.from_euler("xyz", rpy).as_matrix())
        return cache[link]

    return {link: world(link) for link in list(joints) + [root_link]}


def parse_link_inertials(path: str | Path) -> dict[str, tuple[float, np.ndarray]]:
    """各 link の (質量 kg, link frame での COM xyz) を返す（inertial が無い link は省く）。"""
    root = ET.parse(Path(path)).getroot()
    out: dict[str, tuple[float, np.ndarray]] = {}
    for link in root.findall("link"):
        inj = link.find("inertial")
        if inj is None:
            continue
        m = inj.find("mass")
        if m is None or m.get("value") is None:
            continue
        o = inj.find("origin")
        com = _vec(o.get("xyz") if o is not None else None)
        out[link.get("name")] = (float(m.get("value")), com)
    return out


def parse_link_inertia_tensors(path: str | Path) -> dict[str, tuple[float, np.ndarray, np.ndarray]]:
    """各 link の (質量, link frame の COM xyz, link frame の慣性テンソル 3x3) を返す。

    URDF の <inertia> は inertial origin（COM, rpy 回転後の frame）まわりの値。rpy で link frame
    へ回しておく（COM まわり・link 軸）。
    """
    root = ET.parse(Path(path)).getroot()
    out: dict[str, tuple[float, np.ndarray, np.ndarray]] = {}
    for link in root.findall("link"):
        inj = link.find("inertial")
        if inj is None:
            continue
        m = inj.find("mass")
        it = inj.find("inertia")
        if m is None or it is None or m.get("value") is None:
            continue
        o = inj.find("origin")
        com = _vec(o.get("xyz") if o is not None else None)
        rpy = _vec(o.get("rpy") if o is not None else None)

        def g(k: str) -> float:
            return float(it.get(k, 0.0))

        inertia = np.array([
            [g("ixx"), g("ixy"), g("ixz")],
            [g("ixy"), g("iyy"), g("iyz")],
            [g("ixz"), g("iyz"), g("izz")],
        ])
        r = Rot.from_euler("xyz", rpy).as_matrix()
        inertia = r @ inertia @ r.T  # inertial frame → link frame（COM まわり）
        out[link.get("name")] = (float(m.get("value")), com, inertia)
    return out


def canonical_inertia_tensors(
    path: str | Path, *, link_map: Optional[dict[str, str]] = None
) -> dict[str, dict[str, object]]:
    """URDF から canonical 19-joint の各 bone の (質量, COM, 慣性テンソル) を集約して返す。

    各 link を世界 COM 最近傍の canonical bone（or pelvis）へ割当て、剛体合成（平行軸の定理）で
    bone ごとに **その bone の合成 COM まわり・世界軸**の慣性テンソルにまとめる。返り値は
    {canonical_joint_name: {"mass": kg, "com": [x,y,z] world, "fullinertia": [ixx,iyy,izz,ixy,ixz,iyz]}}。
    MJCF の explicit <inertial>（capsule 近似ではなく実機慣性）に使う。
    """
    lmap = link_map or G1_LINK_MAP
    joints, root_link = parse_urdf(path)
    frames = link_world_frames(joints, root_link)
    rest = build_rest_pose(link_world_positions(joints, root_link), lmap)
    tensors = parse_link_inertia_tensors(path)

    segments = [(j, rest[PARENTS[j]], rest[j]) for j in range(len(JOINT_NAMES)) if PARENTS[j] >= 0]
    groups: dict[str, list[tuple[float, np.ndarray, np.ndarray]]] = {n: [] for n in JOINT_NAMES}
    for link, (m, com_local, inertia_local) in tensors.items():
        if link not in frames:
            continue
        pos, rot = frames[link]
        com_w = pos + rot @ com_local
        inertia_w = rot @ inertia_local @ rot.T  # link 軸 → 世界軸
        if link == root_link:
            best_name = "pelvis"  # root link（骨盤本体）は必ず pelvis ハブへ
        else:
            best_name, best_d = "pelvis", float(np.linalg.norm(com_w - rest[0]))
            for j, a, b in segments:
                d = _point_segment_distance(com_w, a, b)
                if d < best_d:
                    best_name, best_d = JOINT_NAMES[j], d
        groups[best_name].append((m, com_w, inertia_w))

    out: dict[str, dict[str, object]] = {}
    for name, items in groups.items():
        if not items:
            continue
        mass = sum(m for m, _, _ in items)
        com = sum(m * c for m, c, _ in items) / mass
        inertia = np.zeros((3, 3))
        for m, c, iw in items:
            d = c - com
            inertia += iw + m * (float(d @ d) * np.eye(3) - np.outer(d, d))  # 平行軸
        # COM は bone の body 原点（= 親 joint。pelvis は自身）相対で持つ。これにより rest pose の
        # 全体並進（URDF FK frame ↔ 接地シフトした embodiment rest）に不変になる。慣性テンソルは
        # 世界軸・COM まわりで並進不変なのでそのまま。
        idx = JOINT_NAMES.index(name)
        origin = rest[PARENTS[idx]] if PARENTS[idx] >= 0 else rest[0]
        com_rel = com - origin
        out[name] = {
            "mass": round(float(mass), 5),
            "com": [round(float(x), 5) for x in com_rel],
            "fullinertia": [round(float(v), 6) for v in (
                inertia[0, 0], inertia[1, 1], inertia[2, 2],
                inertia[0, 1], inertia[0, 2], inertia[1, 2])],
        }
    return out


def _point_segment_distance(p: np.ndarray, a: np.ndarray, b: np.ndarray) -> float:
    """点 p と線分 ab の距離。"""
    ab = b - a
    denom = float(ab @ ab)
    t = 0.0 if denom < 1e-12 else float(np.clip((p - a) @ ab / denom, 0.0, 1.0))
    return float(np.linalg.norm(p - (a + t * ab)))


# canonical 質量分布で、link が 1 つも割当たらない関節にも与える最小割合（mjcf の 0 質量 body 回避 +
# 総質量厳密保存のため。総質量×この値 > mjcf の 0.01kg floor になるよう十分大きく取る）。
_MIN_SEGMENT_FRACTION = 0.001


def canonical_mass_distribution(
    path: str | Path, *, link_map: Optional[dict[str, str]] = None
) -> tuple[dict[str, float], float]:
    """URDF の <inertial> から canonical 19-joint の質量分布（fraction, Σ=1）と総質量を返す。

    各 link の世界 COM を最近傍の canonical bone セグメント（親→子）または pelvis ハブ（点）へ
    割当てて集約する。これにより人体計測（Winter）プライアではなく**実機の実分布**になる
    （実機は股・膝アクチュエータで脚が重く、人体より胴体比が低い）。数値のみで license-safe。
    """
    lmap = link_map or G1_LINK_MAP
    joints, root_link = parse_urdf(path)
    frames = link_world_frames(joints, root_link)
    rest = build_rest_pose(link_world_positions(joints, root_link), lmap)
    inertials = parse_link_inertials(path)

    segments = [(j, rest[PARENTS[j]], rest[j]) for j in range(len(JOINT_NAMES)) if PARENTS[j] >= 0]
    mass = {name: 0.0 for name in JOINT_NAMES}
    for link, (m, com_local) in inertials.items():
        if link not in frames:
            continue
        pos, rot = frames[link]
        com = pos + rot @ com_local
        best_name, best_d = "pelvis", float(np.linalg.norm(com - rest[0]))  # pelvis ハブ（点）
        for j, a, b in segments:
            d = _point_segment_distance(com, a, b)
            if d < best_d:
                best_name, best_d = JOINT_NAMES[j], d
        mass[best_name] += m

    # 左右対称化: link COM が境界付近で左右にフリップする割当ゆらぎを除き、物理的対称性を担保する。
    for name in list(mass):
        if name.startswith("left_"):
            mirror = "right_" + name[len("left_"):]
            avg = 0.5 * (mass[name] + mass.get(mirror, 0.0))
            mass[name] = mass[mirror] = avg

    total = sum(mass.values())
    if total <= 0.0:
        raise ValueError("URDF に <inertial> 質量が無い—質量分布を算出できない")
    floor = total * _MIN_SEGMENT_FRACTION
    floored = {name: max(m, floor) for name, m in mass.items()}  # 0 質量関節にも最小値
    fsum = sum(floored.values())
    fraction = {name: m / fsum for name, m in floored.items()}  # Σ=1 に正規化
    return fraction, total


def build_rest_pose(link_pos: dict[str, np.ndarray], link_map: dict[str, str]) -> np.ndarray:
    """リンク世界位置から canonical 19-joint rest pose [19, 3] を作る（torso・toe は合成）。"""
    out = np.zeros((len(JOINT_NAMES), 3))
    for canon, link in link_map.items():
        out[index_of(canon)] = link_pos[link]

    # wrist が map に無い URDF（例: H1 無印は腕が肘止まり）では前腕を上腕と同程度に合成。
    for side in ("left", "right"):
        if f"{side}_wrist" not in link_map:
            sh, el = out[index_of(f"{side}_shoulder")], out[index_of(f"{side}_elbow")]
            out[index_of(f"{side}_wrist")] = el + (el - sh)

    pelvis = out[index_of("pelvis")]
    chest = 0.5 * (out[index_of("left_shoulder")] + out[index_of("right_shoulder")])
    out[index_of("chest")] = chest
    out[index_of("spine")] = 0.5 * (pelvis + chest)
    head = chest + np.array([0.03, 0.0, 0.22])      # URDF に解剖頭頂が無いため合成
    out[index_of("head")] = head
    out[index_of("neck")] = 0.5 * (chest + head)
    # toe を足首から前方へ合成（URDF に toe リンクが無いため）。
    for side in ("left", "right"):
        out[index_of(f"{side}_foot")] = out[index_of(f"{side}_ankle")] + np.array([0.12, 0.0, -0.04])
    return out


def urdf_to_morphology(
    path: str | Path,
    *,
    name: str = "unitree_g1",
    link_map: Optional[dict[str, str]] = None,
    urdf_ref: Optional[str] = None,
) -> RobotMorphology:
    """URDF から実寸 rest pose を持つ RobotMorphology を構築する。"""
    lmap = link_map or G1_LINK_MAP
    joints, root_link = parse_urdf(path)
    link_pos = link_world_positions(joints, root_link)
    rest = build_rest_pose(link_pos, lmap)
    per_joint = canonical_joint_limits(parse_actuated_limits(path))
    try:
        mass_dist, _ = canonical_mass_distribution(path, link_map=lmap)
        inertia = canonical_inertia_tensors(path, link_map=lmap)
    except ValueError:
        mass_dist = inertia = None  # <inertial> 無し URDF（合成 fixture 等）
    return RobotMorphology(
        name=name, rest_pose=rest,
        urdf_ref=urdf_ref or str(path), runtime_adapter="unitree_sdk2",
        per_joint_limits=per_joint or None,
        mass_distribution=mass_dist,
        inertia_tensors=inertia or None,
    )


def _vec(s: Optional[str]) -> np.ndarray:
    if not s:
        return np.zeros(3)
    return np.array([float(v) for v in s.split()], dtype=np.float64)
