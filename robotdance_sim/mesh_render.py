"""実 Unitree URDF メッシュでのレンダリング（pybullet headless）。

`scripts/render_real_video_gif._render_mesh` の共有実装。HumanoidBattle mesh fight でも使用。
URDF / mesh は repo に同梱しない — `resolve_unitree_urdf` でローカルパスを解決する。
"""

from __future__ import annotations

import os
from pathlib import Path

import numpy as np

from robotdance_core.rd_mir import RdMir

# robot 名 → (short, env var, 既定候補パス)
_URDF_LOOKUP: dict[str, tuple[str, str, str]] = {
    "unitree_g1": (
        "g1",
        "ROBOTDANCE_G1_URDF",
        "~/tmp/g1_meshes/unitree_ros/robots/g1_description/g1_23dof.urdf",
    ),
    "unitree_h1": (
        "h1",
        "ROBOTDANCE_H1_URDF",
        "~/tmp/g1_meshes/unitree_ros/robots/h1_description/urdf/h1.urdf",
    ),
    "unitree_h2": (
        "h2",
        "ROBOTDANCE_H2_URDF",
        "~/tmp/g1_meshes/unitree_ros/robots/h2_description/H2.urdf",
    ),
}

_DEFAULT_BASE_Z = {"g1": 0.793, "h1": 1.04, "h2": 1.10}

_CORNER_RGBA = {
    "a": [0.85, 0.22, 0.22, 1.0],
    "b": [0.22, 0.4, 0.9, 1.0],
}


def resolve_unitree_urdf(robot: str) -> Path:
    """`unitree_g1` 等から実 URDF パスを解決する。見つからなければ FileNotFoundError。"""
    if robot not in _URDF_LOOKUP:
        raise ValueError(
            f"--mesh は Unitree G1/H1/H2 のみ対応です（'{robot}'）。"
            f" 利用可能: {sorted(_URDF_LOOKUP)}"
        )
    _, env_key, default = _URDF_LOOKUP[robot]
    for cand in (os.environ.get(env_key, ""), default):
        if cand:
            path = Path(cand).expanduser()
            if path.is_file():
                return path
    raise FileNotFoundError(
        f"{robot} の URDF が見つかりません。{env_key} を設定するか、"
        f"Unitree unitree_ros の URDF を {default} に配置してください。"
    )


def default_base_z(robot: str) -> float:
    short, _, _ = _URDF_LOOKUP[robot]
    return _DEFAULT_BASE_Z[short]


def render_mesh_trajectory(
    urdf: Path,
    robot_short: str,
    base_z: float,
    angles: np.ndarray,
    joint_names: list[str],
    *,
    fps: float,
    stride: int = 2,
    width: int = 360,
    height: int = 460,
) -> list[np.ndarray]:
    """単体ロボットの関節角列を実メッシュでレンダリングし RGB フレーム列を返す。"""
    import pybullet as p

    urdf = urdf.resolve()
    urdf_dir = urdf.parent
    p.setAdditionalSearchPath(str(urdf_dir))
    gv = p.createVisualShape(p.GEOM_BOX, halfExtents=[3, 3, 0.01], rgbaColor=[0.93, 0.93, 0.95, 1])
    gc = p.createCollisionShape(p.GEOM_BOX, halfExtents=[3, 3, 0.01])
    p.createMultiBody(0, gc, gv, basePosition=[0, 0, -0.005])
    rid = p.loadURDF(urdf.name, useFixedBase=True, basePosition=[0, 0, base_z])

    jmap = _revolute_map(rid)
    pairs = [(jmap[n], k) for k, n in enumerate(joint_names) if n in jmap]
    n_links = p.getNumJoints(rid)
    proj = p.computeProjectionMatrixFOV(42, width / height, 0.1, 10)
    cam_target_z = base_z * 0.78
    cam_dist = base_z * 2.45

    def _lowest_z() -> float:
        zmin = p.getAABB(rid, -1)[0][2]
        for i in range(n_links):
            zmin = min(zmin, p.getAABB(rid, i)[0][2])
        return zmin

    frames: list[np.ndarray] = []
    t_len = angles.shape[0]
    for f in range(0, t_len, stride):
        p.resetBasePositionAndOrientation(rid, [0, 0, base_z], [0, 0, 0, 1])
        for ji, k in pairs:
            p.resetJointState(rid, ji, float(angles[f, k]))
        p.performCollisionDetection()
        p.resetBasePositionAndOrientation(rid, [0, 0, base_z - _lowest_z() + 0.005], [0, 0, 0, 1])
        yaw = 35 + 25 * np.sin(2 * np.pi * f / max(t_len, 1))
        view = p.computeViewMatrixFromYawPitchRoll([0, 0, cam_target_z], cam_dist, yaw, -10, 0, 2)
        img = p.getCameraImage(width, height, view, proj, renderer=p.ER_TINY_RENDERER,
                               lightDirection=[0.6, 0.7, 1.2], shadow=1)
        frames.append(np.reshape(img[2], (height, width, 4))[:, :, :3].astype(np.uint8))
    return frames


def render_fight_mesh(
    mir_a: RdMir,
    mir_b: RdMir,
    *,
    robot_a: str,
    robot_b: str,
    urdf_a: Path,
    urdf_b: Path,
    separation: float = 0.55,
    n_frames: int,
    width: int = 480,
    height: int = 360,
    stride: int = 2,
) -> list[np.ndarray]:
    """2 体を対面配置し、実メッシュで fight フレーム列を返す（pybullet DIRECT）。"""
    import pybullet as p

    from robotdance_retarget.actuator_ik import actuator_retarget
    from robotdance_unitree.urdf_import import G1_LINK_MAP, H1_LINK_MAP, H2_LINK_MAP

    link_maps = {"unitree_g1": G1_LINK_MAP, "unitree_h1": H1_LINK_MAP, "unitree_h2": H2_LINK_MAP}
    short_a, _, _ = _URDF_LOOKUP[robot_a]
    short_b, _, _ = _URDF_LOOKUP[robot_b]
    base_a = default_base_z(robot_a)
    base_b = default_base_z(robot_b)

    mot_a = actuator_retarget(mir_a, urdf_a, link_map=link_maps[robot_a], robot_name=robot_a, steps=200)
    mot_b = actuator_retarget(mir_b, urdf_b, link_map=link_maps[robot_b], robot_name=robot_b, steps=200)
    ang_a = np.asarray(mot_a.joint_rotations["angles_rad"])[:n_frames]
    ang_b = np.asarray(mot_b.joint_rotations["angles_rad"])[:n_frames]
    names_a = [str(n) for n in mot_a.joint_rotations["actuated_joint_names"]]
    names_b = [str(n) for n in mot_b.joint_rotations["actuated_joint_names"]]

    urdf_a, urdf_b = urdf_a.resolve(), urdf_b.resolve()
    p.setAdditionalSearchPath(str(urdf_a.parent))
    gv = p.createVisualShape(p.GEOM_BOX, halfExtents=[4, 4, 0.01], rgbaColor=[0.55, 0.57, 0.6, 1])
    gc = p.createCollisionShape(p.GEOM_BOX, halfExtents=[4, 4, 0.01])
    p.createMultiBody(0, gc, gv, basePosition=[0, 0, -0.005])

    rid_a = _load_tinted_robot(urdf_a, [-separation, 0, base_a], [0, 0, 0, 1], "a")
    p.setAdditionalSearchPath(str(urdf_b.parent))
    rid_b = _load_tinted_robot(urdf_b, [separation, 0, base_b], [0, 0, 1, 0], "b")

    pairs_a = _joint_pairs(rid_a, names_a)
    pairs_b = _joint_pairs(rid_b, names_b)
    proj = p.computeProjectionMatrixFOV(50, width / height, 0.1, 12)
    cam_target = [0, 0, max(base_a, base_b) * 0.75]
    cam_dist = max(base_a, base_b) * 2.8

    frames: list[np.ndarray] = []
    for f in range(0, n_frames, stride):
        _pose_robot(rid_a, pairs_a, ang_a, f, [-separation, 0, base_a], [0, 0, 0, 1])
        _pose_robot(rid_b, pairs_b, ang_b, f, [separation, 0, base_b], [0, 0, 1, 0])
        view = p.computeViewMatrixFromYawPitchRoll(cam_target, cam_dist, 90, -8, 0, 2)
        img = p.getCameraImage(width, height, view, proj, renderer=p.ER_TINY_RENDERER,
                               lightDirection=[0.5, 0.6, 1.0], shadow=1)
        frames.append(np.reshape(img[2], (height, width, 4))[:, :, :3].astype(np.uint8))
    return frames


def _revolute_map(rid: int) -> dict[str, int]:
    import pybullet as p

    jmap: dict[str, int] = {}
    for i in range(p.getNumJoints(rid)):
        info = p.getJointInfo(rid, i)
        if info[2] == p.JOINT_REVOLUTE:
            jmap[info[1].decode()] = i
    return jmap


def _joint_pairs(rid: int, names: list[str]) -> list[tuple[int, int]]:
    jmap = _revolute_map(rid)
    return [(jmap[n], k) for k, n in enumerate(names) if n in jmap]


def _load_tinted_robot(urdf: Path, pos: list[float], orn: list[float], corner: str) -> int:
    import pybullet as p

    rid = p.loadURDF(urdf.name, useFixedBase=True, basePosition=pos, baseOrientation=orn)
    tint = _CORNER_RGBA[corner]
    for i in range(-1, p.getNumJoints(rid)):
        try:
            p.changeVisualShape(rid, i, rgbaColor=tint)
        except Exception:
            pass
    return rid


def _pose_robot(rid: int, pairs: list[tuple[int, int]], angles: np.ndarray, f: int,
                base_pos: list[float], base_orn: list[float]) -> None:
    import pybullet as p

    base_z = base_pos[2]
    p.resetBasePositionAndOrientation(rid, base_pos, base_orn)
    for ji, k in pairs:
        p.resetJointState(rid, ji, float(angles[f, k]))
    p.performCollisionDetection()
    zmin = p.getAABB(rid, -1)[0][2]
    for i in range(p.getNumJoints(rid)):
        zmin = min(zmin, p.getAABB(rid, i)[0][2])
    p.resetBasePositionAndOrientation(
        rid, [base_pos[0], base_pos[1], base_z - zmin + 0.005], base_orn)


__all__ = [
    "default_base_z",
    "render_fight_mesh",
    "render_mesh_trajectory",
    "resolve_unitree_urdf",
]
