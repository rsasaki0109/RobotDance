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

from robotdance_core.skeleton import JOINT_NAMES, index_of
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


def build_rest_pose(link_pos: dict[str, np.ndarray], link_map: dict[str, str]) -> np.ndarray:
    """リンク世界位置から canonical 19-joint rest pose [19, 3] を作る（torso・toe は合成）。"""
    out = np.zeros((len(JOINT_NAMES), 3))
    for canon, link in link_map.items():
        out[index_of(canon)] = link_pos[link]

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
    joints, root_link = parse_urdf(path)
    link_pos = link_world_positions(joints, root_link)
    rest = build_rest_pose(link_pos, link_map or G1_LINK_MAP)
    return RobotMorphology(
        name=name, rest_pose=rest,
        urdf_ref=urdf_ref or str(path), runtime_adapter="unitree_sdk2",
    )


def _vec(s: Optional[str]) -> np.ndarray:
    if not s:
        return np.zeros(3)
    return np.array([float(v) for v in s.split()], dtype=np.float64)
