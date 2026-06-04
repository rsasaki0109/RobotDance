"""Actuator-space retarget（実 URDF への微分可能 IK, v0）。

kinematic retarget は canonical link 位置（ball-joint 近似）を出すが、実機は **実 G1 の
アクチュエータ関節角**を要求する。本モジュールは実 URDF の微分可能 FK を構成し、勾配 IK で
**実 G1 の 23 関節角**を解く。出力はそのまま ROS2/SDK2 が command できる joint trajectory。

⚠️ v0: これは参照 IK（位置合わせ）であり、バランス制御 policy ではない。動的実現可能性は
sim_certificate（robotdance_sim）が別途検証する。torso 連鎖・toe は合成 target なので IK 対象外。
torch が必要（`[learn]` extra）。
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import numpy as np
import torch

from robotdance_core.rd_mir import RdMir
from robotdance_core.rd_motion import RdMotion, Skeleton
from robotdance_core.skeleton import JOINT_NAMES, PARENTS, index_of
from robotdance_unitree.urdf_import import G1_LINK_MAP


@dataclass
class _Link:
    name: str
    parent: Optional[str]
    origin_xyz: np.ndarray
    origin_rpy: np.ndarray
    axis: Optional[np.ndarray]   # revolute のみ
    act_idx: int                 # actuated 関節 index（fixed は -1）
    limit: Optional[tuple[float, float]]


class G1Chain:
    """URDF から作る微分可能 FK チェーン。"""

    def __init__(self, urdf_path: str | Path) -> None:
        root = ET.parse(Path(urdf_path)).getroot()
        joints = {}
        children, parents = set(), set()
        for j in root.findall("joint"):
            o = j.find("origin")
            ax = j.find("axis")
            lim = j.find("limit")
            child = j.find("child").get("link")
            joints[child] = {
                "parent": j.find("parent").get("link"),
                "xyz": _vec(o.get("xyz") if o is not None else None),
                "rpy": _vec(o.get("rpy") if o is not None else None),
                "type": j.get("type"),
                "axis": _vec(ax.get("xyz")) if ax is not None else None,
                "limit": (float(lim.get("lower")), float(lim.get("upper")))
                if lim is not None and lim.get("lower") else None,
            }
            children.add(child)
            parents.add(j.find("parent").get("link"))
        self.root = next(iter(parents - children), "pelvis")

        # topological 順（親が先）にリンクを並べる。
        order: list[str] = [self.root]
        added = {self.root}
        while len(added) < len(joints) + 1:
            for c, jd in joints.items():
                if c not in added and jd["parent"] in added:
                    order.append(c)
                    added.add(c)
        self.links: list[_Link] = [_Link(self.root, None, np.zeros(3), np.zeros(3), None, -1, None)]
        act = 0
        for name in order[1:]:
            jd = joints[name]
            is_rev = jd["type"] == "revolute"
            axis = None
            if is_rev:
                axis = jd["axis"] if jd["axis"] is not None else np.array([0.0, 0.0, 1.0])
            self.links.append(_Link(name, jd["parent"], jd["xyz"], jd["rpy"], axis,
                                    act if is_rev else -1, jd["limit"]))
            if is_rev:
                act += 1
        self.n_act = act
        self.act_names = [link_actuated_name(li) for li in self.links if li.act_idx >= 0]
        self._index = {li.name: i for i, li in enumerate(self.links)}
        self.limits = np.array(
            [li.limit or (-3.14, 3.14) for li in self.links if li.act_idx >= 0], dtype=np.float64)

    def link_index(self, name: str) -> int:
        return self._index[name]

    def fk(self, q: torch.Tensor) -> torch.Tensor:
        """q [B, n_act] → 各リンクの world 位置 [B, n_link, 3]。"""
        b = q.shape[0]
        dev = q.device
        eye = torch.eye(4, device=dev).expand(b, 4, 4)
        world = [eye] * len(self.links)
        for i, li in enumerate(self.links):
            if li.parent is None:
                world[i] = eye
                continue
            origin = _homog(_rpy_mat(li.origin_rpy, dev), torch.tensor(li.origin_xyz, device=dev,
                                                                       dtype=torch.float32)).expand(b, 4, 4)
            if li.act_idx >= 0:
                jrot = _axis_rot(torch.tensor(li.axis, device=dev, dtype=torch.float32), q[:, li.act_idx])
                local = origin @ _homog_b(jrot)
            else:
                local = origin
            world[i] = world[self._index[li.parent]] @ local
        pos = torch.stack([w[:, :3, 3] for w in world], dim=1)  # [B, n_link, 3]
        return pos


def actuator_retarget(
    mir: RdMir,
    urdf_path: str | Path,
    *,
    steps: int = 300,
    lr: float = 0.1,
    smooth_w: float = 0.5,
    device: Optional[str] = None,
    link_map: Optional[dict[str, str]] = None,
    robot_name: str = "unitree_g1",
) -> RdMotion:
    """RD-MIR を実 URDF のアクチュエータ関節角へ IK retarget して RD-Motion を返す。

    link_map（canonical joint → URDF link）で G1 以外（H1 等）にも対応する（既定 G1_LINK_MAP）。
    """
    from robotdance_retarget.kinematic import retarget
    from robotdance_unitree.urdf_import import urdf_to_morphology

    lmap = link_map or G1_LINK_MAP
    dev = device or ("cuda" if torch.cuda.is_available() else "cpu")
    chain = G1Chain(urdf_path)
    morph = urdf_to_morphology(urdf_path, name=robot_name, link_map=lmap)

    # kinematic retarget の link 位置を IK target にする（pelvis 相対）。
    kin = retarget(mir, morph)
    kp = kin.keypoints_3d_array()  # [T, 19, 3]
    pelvis = kp[:, index_of("pelvis"):index_of("pelvis") + 1, :]
    target_rel = kp - pelvis

    # IK 対象 = 実リンクにマップされる limb（pelvis 除く）。
    pairs = [(index_of(c), chain.link_index(L)) for c, L in lmap.items() if c != "pelvis"]
    canon_idx = [p[0] for p in pairs]
    link_idx = [p[1] for p in pairs]
    tgt = torch.tensor(target_rel[:, canon_idx, :], dtype=torch.float32, device=dev)  # [T, M, 3]

    t = kp.shape[0]
    q = torch.zeros(t, chain.n_act, device=dev, requires_grad=True)
    lo = torch.tensor(chain.limits[:, 0], device=dev, dtype=torch.float32)
    hi = torch.tensor(chain.limits[:, 1], device=dev, dtype=torch.float32)
    opt = torch.optim.Adam([q], lr=lr)
    link_idx_t = torch.tensor(link_idx, device=dev)
    for _ in range(steps):
        pos = chain.fk(q)                                  # [T, n_link, 3]（pelvis=root=0）
        fk_rel = pos[:, link_idx_t, :]
        pos_loss = ((fk_rel - tgt) ** 2).sum(-1).mean()
        limit_pen = (torch.relu(q - hi) + torch.relu(lo - q)).pow(2).mean()
        smooth = ((q[1:] - q[:-1]) ** 2).mean() if t > 1 else torch.zeros((), device=dev)
        loss = pos_loss + 10.0 * limit_pen + smooth_w * smooth
        opt.zero_grad()
        loss.backward()
        opt.step()

    with torch.no_grad():
        q_final = torch.clamp(q, lo, hi)
        fk_rel = chain.fk(q_final)[:, link_idx_t, :]
        err = torch.linalg.norm(fk_rel - tgt, dim=-1)       # [T, M] m
        viol = ((q.detach() < lo) | (q.detach() > hi)).float().mean()
    q_np = q_final.cpu().numpy()

    metrics = {
        "method": "differentiable_fk_gradient_ik",
        "actuated_joints": chain.n_act,
        "ik_mean_pos_error_m": round(float(err.mean()), 4),
        "ik_max_pos_error_m": round(float(err.max()), 4),
        "joint_limit_violation_ratio_preclamp": round(float(viol), 4),
        "note": "参照 IK（位置合わせ）。バランス policy ではない。torso/toe は IK 対象外（合成 target）。",
    }
    return RdMotion(
        robot_name=robot_name,
        fps=mir.fps,
        duration=mir.duration,
        source_motion_id=mir.motion_id,
        skeleton=Skeleton(joint_names=list(JOINT_NAMES), parents=list(PARENTS)),
        control_mode="position",
        keypoints_3d=kin.keypoints_3d,           # 可視化用 link 位置
        joint_rotations={"actuated_joint_names": chain.act_names, "angles_rad": q_np.tolist()},
        contact_schedule=kin.contact_schedule,
        retarget_metrics=metrics,
        source_provenance={"rd_mir_motion_id": mir.motion_id, "urdf": str(urdf_path),
                           "method": "actuator_space_ik"},
    )


# --- helpers ---

def link_actuated_name(li: _Link) -> str:
    return li.name.replace("_link", "_joint")


def _vec(s: Optional[str]) -> np.ndarray:
    return np.zeros(3) if not s else np.array([float(v) for v in s.split()], dtype=np.float64)


def _rpy_mat(rpy: np.ndarray, dev) -> torch.Tensor:
    from scipy.spatial.transform import Rotation as Rot

    m = Rot.from_euler("xyz", rpy).as_matrix()
    return torch.tensor(m, device=dev, dtype=torch.float32)


def _homog(rot: torch.Tensor, trans: torch.Tensor) -> torch.Tensor:
    h = torch.eye(4, device=rot.device)
    h[:3, :3] = rot
    h[:3, 3] = trans
    return h


def _homog_b(rot: torch.Tensor) -> torch.Tensor:
    """[B,3,3] → [B,4,4]。"""
    b = rot.shape[0]
    h = torch.eye(4, device=rot.device).expand(b, 4, 4).clone()
    h[:, :3, :3] = rot
    return h


def _axis_rot(axis: torch.Tensor, theta: torch.Tensor) -> torch.Tensor:
    """固定軸 axis, 角 theta [B] → 回転行列 [B,3,3]（Rodrigues）。"""
    a = axis / torch.linalg.norm(axis)
    b = theta.shape[0]
    K = torch.zeros(3, 3, device=axis.device)
    K[0, 1], K[0, 2], K[1, 0] = -a[2], a[1], a[2]
    K[1, 2], K[2, 0], K[2, 1] = -a[0], -a[1], a[0]
    K = K.expand(b, 3, 3)
    eye = torch.eye(3, device=axis.device).expand(b, 3, 3)
    s = torch.sin(theta).view(b, 1, 1)
    c = (1 - torch.cos(theta)).view(b, 1, 1)
    return eye + s * K + c * (K @ K)
