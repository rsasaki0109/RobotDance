"""MuJoCo physics 上の reference-tracking 環境（RL baseline, v0）。

sim_certificate は「参照運動が物理的に実現可能か」を kinematic に判定するが、その次の段階は
**倒れずに追従できる方策**を学習することだ。本環境は参照運動（RD-Motion の keypoints →
qpos 列）を **forward 物理シミュレーション上で追従**するための RL 環境を提供する。

制御モデル:
  - base（pelvis）は free joint で **非駆動**。関節トルクと足の接地のみで姿勢を支える
    → 「バランスを取りながら追う」ことが本質的に必要（underactuated）。
  - 駆動 DOF は **関節空間 PD**（参照 qpos へアンカー）+ 方策の **残差トルク**。
    残差ゼロでも PD が参照を追うので、方策は「PD だけでは倒れる」分を学習で補う。
  - 一般化力は `qfrc_applied[6:]` に直接与える（free joint の 0:6 は常にゼロ＝非駆動）。

報酬 = 姿勢追従 + 直立 + 生存 - 制御コスト。転倒（base 高さ低下 / 大きな傾き）で終了。

⚠️ v0: 近似質量・近似接地、報酬/終了条件は素朴で、SOTA tracking（DeepMimic/AMP 等）ではない。
これは **学習基盤（baseline）** であり実機保証ではない。mujoco が必要。
"""

from __future__ import annotations

from typing import Any

import numpy as np

from robotdance_core.rd_motion import RdMotion
from robotdance_core.skeleton import JOINT_NAMES, PARENTS
from robotdance_retarget.embodiment import RobotMorphology

from .mjcf import build_mjcf
from .mujoco_backend import _pose_to_qpos


class TrackingEnv:
    """参照 RD-Motion を物理上で追従する RL 環境（gym 風 reset/step）。"""

    def __init__(
        self,
        reference: RdMotion,
        morphology: RobotMorphology,
        *,
        total_mass: float = 35.0,
        kp: float = 60.0,
        kd: float = 3.0,
        torque_limit: float = 80.0,
        residual_scale: float = 6.0,
        fall_height_ratio: float = 0.5,
        upright_min: float = 0.3,
    ) -> None:
        import mujoco

        self._mj = mujoco
        self.reference = reference
        self.morph = morphology
        self.model = mujoco.MjModel.from_xml_string(
            build_mjcf(morphology, total_mass=total_mass, ground=True)
        )
        self.data = mujoco.MjData(self.model)
        self.kp = kp
        self.kd = kd
        self.torque_limit = torque_limit
        self.residual_scale = residual_scale
        self.upright_min = upright_min

        self.fps = float(reference.fps)
        self.dt = 1.0 / self.fps
        self.n_substeps = max(1, round(self.dt / self.model.opt.timestep))

        # 参照 keypoints を厳密に qpos 列へ復元（mujoco_backend と同じ写像）。
        kps = reference.keypoints_3d_array()  # [T, J, 3]
        self.ref_qpos = np.stack(
            [_pose_to_qpos(self.model, morphology, kps[f]) for f in range(len(kps))]
        )  # [T, nq]
        self.T = int(self.ref_qpos.shape[0])

        self.nv = int(self.model.nv)
        self.n_act = self.nv - 6  # free joint の 6-DOF を除く駆動 DOF 数
        self.action_dim = self.n_act
        self.root_id = self.model.body("root").id
        self.fall_height = fall_height_ratio * float(self.ref_qpos[0, 2])

        # sim qpos → canonical keypoints 復元用（各 bone の終点 = 子 joint 位置）。
        rest = morphology.rest_pose
        self._endpoint = {j: rest[j] - rest[PARENTS[j]] for j in range(1, len(JOINT_NAMES))}
        self._body_id = {j: self.model.body(f"body_{j}").id for j in range(1, len(JOINT_NAMES))}

        obs = self.reset()
        self.obs_dim = int(obs.shape[0])

    # --- gym 風 API ---

    def reset(self) -> np.ndarray:
        self._mj.mj_resetData(self.model, self.data)
        self.data.qpos[:] = self.ref_qpos[0]
        self.data.qvel[:] = 0.0
        self._mj.mj_forward(self.model, self.data)
        self.t = 0
        return self._make_obs()

    def step(self, action: np.ndarray) -> tuple[np.ndarray, float, bool, dict[str, Any]]:
        a = np.clip(np.asarray(action, dtype=np.float64), -1.0, 1.0) * self.residual_scale
        target = self.ref_qpos[min(self.t + 1, self.T - 1)]
        err = self._err_to(target)  # nv: target ⊖ current（tangent 空間）
        tau = self.kp * err[6:] - self.kd * self.data.qvel[6:] + a
        tau = np.clip(tau, -self.torque_limit, self.torque_limit)
        self.data.qfrc_applied[:] = 0.0
        self.data.qfrc_applied[6:] = tau
        for _ in range(self.n_substeps):
            self._mj.mj_step(self.model, self.data)
        self.t += 1

        ref_now = self.ref_qpos[min(self.t, self.T - 1)]
        perr = self._err_to(ref_now)
        pose_rmse = float(np.sqrt(np.mean(perr[6:] ** 2)))
        # 報酬は姿勢追従を主軸に（残差で PD を壊すと悪化する）。直立/生存は補助。
        pose_term = float(np.exp(-4.0 * np.mean(perr[6:] ** 2)))
        root_h_err = abs(float(self.data.qpos[2]) - float(ref_now[2]))
        root_term = float(np.exp(-10.0 * root_h_err ** 2))
        up = self._upright()
        up_term = max(0.0, up)
        effort = 5e-4 * float(np.sum(a ** 2))

        reward = 1.0 * pose_term + 0.15 * root_term + 0.1 * up_term + 0.05 - effort
        fallen = (float(self.data.qpos[2]) < self.fall_height) or (up < self.upright_min)
        if fallen:
            reward -= 1.0
        done = bool(fallen or self.t >= self.T - 1)
        info = {"pose_rmse": pose_rmse, "fallen": bool(fallen), "upright": float(up)}
        return self._make_obs(), float(reward), done, info

    # --- 観測・補助 ---

    def _make_obs(self) -> np.ndarray:
        err = self._err_to(self.ref_qpos[min(self.t + 1, self.T - 1)])
        phase = self.t / max(self.T - 1, 1)
        return np.concatenate(
            [
                [float(self.data.qpos[2])],   # base 高さ
                self.data.qpos[3:7],          # base 向き quat (wxyz)
                [self._upright()],            # 直立度
                self.data.qvel[:6],           # base 速度（並進+角）
                self.data.qvel[6:],           # 関節速度
                err[6:],                      # 次フレームへの姿勢誤差（追従ターゲット）
                [phase],                      # 進行度
            ]
        ).astype(np.float32)

    def _err_to(self, qpos_target: np.ndarray) -> np.ndarray:
        """tangent 空間での差分 (qpos_target ⊖ qpos_current)。ball/free joint も正しく扱う。"""
        err = np.zeros(self.nv)
        self._mj.mj_differentiatePos(self.model, err, 1.0, self.data.qpos, qpos_target)
        return err

    def _upright(self) -> float:
        """base 局所 z 軸の world-z 成分（1=直立, 0=横倒れ, <0=逆さ）。"""
        w, x, y, _z = self.data.qpos[3:7]
        return float(1.0 - 2.0 * (x * x + y * y))

    def current_keypoints(self) -> np.ndarray:
        """現在の sim 状態から canonical 19-joint の world 位置を復元する [J, 3]。"""
        kp = np.zeros((len(JOINT_NAMES), 3))
        kp[0] = self.data.xpos[self.root_id]
        for j in range(1, len(JOINT_NAMES)):
            bid = self._body_id[j]
            rmat = self.data.xmat[bid].reshape(3, 3)
            kp[j] = self.data.xpos[bid] + rmat @ self._endpoint[j]
        return kp
