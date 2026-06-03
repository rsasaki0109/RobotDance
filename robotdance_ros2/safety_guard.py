"""Safety Guard（§5.6, ROS2 非依存, v0）。

「動画を入れたら即ロボットが踊る」を防ぐ最後の gate。certified でない motion を弾き、
過大な速度/加速度をクランプし、転倒を検知し、E-stop / speed scaling を提供する。

Cartesian（link 位置）空間に加え、**joint（actuator）空間の limit enforcement** を行う:
actuator-space IK / tracking policy が出す関節角列を、実機に送る直前に **位置 limit・速度・
加速度** へクランプする（§5.6）。これは sim_certificate（物理的妥当性, robotdance_sim）の
**先**にある最終 gate で、コマンド自体を機構的に安全な範囲へ整形する。

⚠️ v0: 位置/速度は厳密に bound する。加速度は best-effort 平滑化（位置 clamp との相互作用で
減速側に残りうる）。トルク/電流 limit は実機モデルが入る Phase 4+ で追加する。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional

import numpy as np

from robotdance_core.rd_motion import RdMotion

from .messages import MotionFrame, SafetyState, SafetyStatus


@dataclass
class SafetyLimits:
    """安全包絡線。"""

    max_link_speed: float = 6.0       # m/s（link 位置の許容速度）
    max_link_accel: float = 120.0     # m/s^2
    warn_link_speed: float = 4.0      # 警告閾値
    max_base_tilt_drop: float = 0.35  # base がこの割合以上沈むと転倒とみなす
    require_certificate: bool = True   # sim_certificate PASS を必須にするか

    # --- joint（actuator）空間 limit（§5.6）---
    enforce_joint_limits: bool = True
    max_joint_speed: float = 12.0      # rad/s（関節角速度の許容上限）
    warn_joint_speed: float = 8.0      # rad/s 警告閾値
    max_joint_accel: float = 200.0     # rad/s^2（関節角加速度の上限, best-effort）
    default_joint_range: float = float(np.pi)  # 位置 limit 未指定時の対称範囲 ±rad
    # actuator 名 → (lower, upper) rad。未指定なら ±default_joint_range。
    joint_position_limits: Optional[dict[str, tuple[float, float]]] = field(default=None)


def _limit_arrays(
    limits: SafetyLimits, joint_names: Optional[list[str]], n: int
) -> tuple[np.ndarray, np.ndarray]:
    """関節順に並べた位置 limit (lo[n], hi[n]) を返す。未指定は ±default_joint_range。"""
    d = limits.default_joint_range
    lo = np.full(n, -d, dtype=np.float64)
    hi = np.full(n, d, dtype=np.float64)
    pl = limits.joint_position_limits
    if pl and joint_names:
        for i, name in enumerate(joint_names[:n]):
            if name in pl:
                lo[i], hi[i] = pl[name]
    return lo, hi


def _limit_step(
    prev_ang: np.ndarray, prev_vel: np.ndarray, target: np.ndarray,
    pos_lo: np.ndarray, pos_hi: np.ndarray, v_max: float, a_max: float, dt: float,
) -> tuple[np.ndarray, np.ndarray, bool, bool, bool]:
    """1 ステップの関節 limit 整形: 加速度 → 速度 → 積分 → 位置 limit の順にクランプ。

    返り値 (ang, vel, pos_hit, vel_clamped, acc_clamped)。
    """
    desired_vel = (target - prev_ang) / dt
    # 加速度 limit（速度変化を制限）。
    dvel = np.clip(desired_vel - prev_vel, -a_max * dt, a_max * dt)
    acc_clamped = bool(np.any(np.abs(desired_vel - prev_vel) > a_max * dt + 1e-9))
    vel = prev_vel + dvel
    # 速度 limit。
    vel_clamped = bool(np.any(np.abs(vel) > v_max + 1e-9))
    vel = np.clip(vel, -v_max, v_max)
    ang = prev_ang + vel * dt
    # 位置 limit（最後に厳密 clamp）。
    angc = np.clip(ang, pos_lo, pos_hi)
    pos_hit = bool(np.any(angc != ang))
    ang = angc
    vel = (ang - prev_ang) / dt  # clamp 後の整合速度
    return ang, vel, pos_hit, vel_clamped, acc_clamped


def clamp_joint_trajectory(
    angles: np.ndarray, dt: float, limits: SafetyLimits,
    joint_names: Optional[list[str]] = None,
) -> tuple[np.ndarray, dict]:
    """関節角列 [T, n] を limit へ整形し、(safe_angles, report) を返す（offline export 用）。

    実機に送る前に joint trajectory 全体を一括検査・整形する。report に raw/safe の
    最大速度・加速度と、各 limit に当たったフレーム数を記録する。
    """
    angles = np.asarray(angles, dtype=np.float64)
    if angles.ndim != 2:
        raise ValueError(f"angles は [T, n] が必要: {angles.shape}")
    t_len, n = angles.shape
    pos_lo, pos_hi = _limit_arrays(limits, joint_names, n)

    out = np.empty_like(angles)
    out[0] = np.clip(angles[0], pos_lo, pos_hi)
    prev_ang = out[0].copy()
    prev_vel = np.zeros(n)
    pos_hits = vel_clamps = acc_clamps = 0
    for t in range(1, t_len):
        ang, vel, ph, vc, ac = _limit_step(
            prev_ang, prev_vel, angles[t], pos_lo, pos_hi,
            limits.max_joint_speed, limits.max_joint_accel, dt,
        )
        out[t] = ang
        prev_ang, prev_vel = ang, vel
        pos_hits += int(ph)
        vel_clamps += int(vc)
        acc_clamps += int(ac)

    def _max_speed(a: np.ndarray) -> float:
        return float(np.abs(np.diff(a, axis=0) / dt).max()) if t_len > 1 else 0.0

    def _max_accel(a: np.ndarray) -> float:
        return float(np.abs(np.diff(a, axis=0, n=2) / (dt * dt)).max()) if t_len > 2 else 0.0

    report = {
        "frames": t_len,
        "joints": n,
        "dt": dt,
        "raw_max_joint_speed_rad_s": round(_max_speed(angles), 3),
        "safe_max_joint_speed_rad_s": round(_max_speed(out), 3),
        "raw_max_joint_accel_rad_s2": round(_max_accel(angles), 1),
        "safe_max_joint_accel_rad_s2": round(_max_accel(out), 1),
        "position_limit_frames": pos_hits,
        "velocity_clamp_frames": vel_clamps,
        "accel_clamp_frames": acc_clamps,
        "max_joint_speed": limits.max_joint_speed,
        "max_joint_accel": limits.max_joint_accel,
        "note": "位置/速度は厳密 bound、加速度は best-effort（位置 clamp の減速で残りうる）。",
    }
    return out, report


class SafetyGuard:
    """motion frame を安全に整形する gate。"""

    def __init__(self, limits: SafetyLimits | None = None, *, speed_scale: float = 1.0) -> None:
        self.limits = limits or SafetyLimits()
        self.speed_scale = float(np.clip(speed_scale, 0.0, 1.0))
        self._estopped = False
        self._nominal_base_z: float | None = None
        self._prev_joint_ang: np.ndarray | None = None
        self._prev_joint_vel: np.ndarray | None = None

    # --- 制御 ---

    def estop(self) -> None:
        """緊急停止。以降のフレームは ABORT になる。"""
        self._estopped = True

    def reset(self) -> None:
        self._estopped = False
        self._nominal_base_z = None
        self._prev_joint_ang = None
        self._prev_joint_vel = None

    def set_speed_scale(self, scale: float) -> None:
        self.speed_scale = float(np.clip(scale, 0.0, 1.0))

    # --- gate ---

    def check_certificate(self, motion: RdMotion) -> SafetyState:
        """再生前チェック: sim_certificate が PASS でなければ ABORT。"""
        if self._estopped:
            return SafetyState(SafetyStatus.ABORT, self.speed_scale, ["E-stop 作動中"])
        cert = motion.sim_certificate
        if not self.limits.require_certificate:
            return SafetyState(SafetyStatus.OK, self.speed_scale, [])
        if cert is None:
            return SafetyState(SafetyStatus.ABORT, self.speed_scale,
                               ["sim_certificate 無し（物理検証されていない）"])
        if not cert.get("passed", False):
            reasons = ["sim_certificate REJECT"] + list(cert.get("reasons", []))
            return SafetyState(SafetyStatus.ABORT, self.speed_scale, reasons)
        return SafetyState(SafetyStatus.OK, self.speed_scale, [])

    def filter_frame(
        self, target: MotionFrame, prev: MotionFrame | None, dt: float
    ) -> tuple[MotionFrame, SafetyState]:
        """1 フレームを安全に整形し、(safe_frame, state) を返す。"""
        if self._estopped:
            held = prev or target
            return held, SafetyState(SafetyStatus.ABORT, self.speed_scale, ["E-stop 作動中"])

        reasons: list[str] = []
        status = SafetyStatus.OK
        kp = target.keypoints.astype(np.float64).copy()

        if prev is not None and dt > 0:
            # 速度クランプ（link ごと）。
            delta = kp - prev.keypoints
            speed = np.linalg.norm(delta, axis=1) / dt  # [J]
            peak = float(speed.max()) if speed.size else 0.0
            if peak > self.limits.max_link_speed:
                scale = self.limits.max_link_speed / peak
                kp = prev.keypoints + delta * scale
                reasons.append(f"link 速度クランプ {peak:.1f}→{self.limits.max_link_speed:.1f} m/s")
                status = SafetyStatus.WARNING
            elif peak > self.limits.warn_link_speed:
                reasons.append(f"link 速度 {peak:.1f} m/s（警告）")
                status = SafetyStatus.WARNING

        # joint（actuator）空間 limit enforcement（§5.6）。
        joint_angles = target.joint_angles
        if (joint_angles is not None and self.limits.enforce_joint_limits and dt > 0):
            joint_angles, status = self._clamp_joints(
                np.asarray(joint_angles, dtype=np.float64), target.joint_names, dt,
                reasons, status,
            )

        # 転倒検知（base が nominal から大きく沈む）。
        bz = float(target.base_position[2])
        if self._nominal_base_z is None:
            self._nominal_base_z = bz
        elif self._nominal_base_z > 0 and bz < self._nominal_base_z * (1 - self.limits.max_base_tilt_drop):
            reasons.append(f"base 沈下 {bz:.2f} < {self._nominal_base_z:.2f}（転倒検知）")
            return (prev or target), SafetyState(SafetyStatus.ABORT, self.speed_scale, reasons)

        safe = MotionFrame(
            index=target.index, time=target.time, keypoints=kp,
            base_position=target.base_position, contacts=target.contacts, phase=target.phase,
            joint_names=target.joint_names, joint_angles=joint_angles,
        )
        return safe, SafetyState(status, self.speed_scale, reasons)

    def _clamp_joints(
        self, angles: np.ndarray, names: list[str], dt: float,
        reasons: list[str], status: SafetyStatus,
    ) -> tuple[np.ndarray, SafetyStatus]:
        """1 フレームの関節角を位置/速度/加速度 limit へクランプ（stateful）。"""
        n = angles.shape[0]
        pos_lo, pos_hi = _limit_arrays(self.limits, names, n)
        if self._prev_joint_ang is None or self._prev_joint_ang.shape[0] != n:
            ang = np.clip(angles, pos_lo, pos_hi)
            if bool(np.any(ang != angles)):
                reasons.append("関節 位置 limit クランプ")
                status = SafetyStatus.WARNING
            self._prev_joint_ang = ang.copy()
            self._prev_joint_vel = np.zeros(n)
            return ang, status

        ang, vel, ph, vc, ac = _limit_step(
            self._prev_joint_ang, self._prev_joint_vel, angles, pos_lo, pos_hi,
            self.limits.max_joint_speed, self.limits.max_joint_accel, dt,
        )
        if ph:
            reasons.append("関節 位置 limit クランプ")
            status = SafetyStatus.WARNING
        if vc:
            reasons.append(f"関節 速度クランプ ≤{self.limits.max_joint_speed:.0f} rad/s")
            status = SafetyStatus.WARNING
        if ac:
            reasons.append(f"関節 加速度クランプ ≤{self.limits.max_joint_accel:.0f} rad/s²")
            status = SafetyStatus.WARNING
        # 速度のみ（clamp なし）の警告。
        peak_v = float(np.abs((angles - self._prev_joint_ang) / dt).max())
        if not vc and peak_v > self.limits.warn_joint_speed:
            reasons.append(f"関節 速度 {peak_v:.1f} rad/s（警告）")
            status = SafetyStatus.WARNING if status is SafetyStatus.OK else status
        self._prev_joint_ang = ang.copy()
        self._prev_joint_vel = vel.copy()
        return ang, status
