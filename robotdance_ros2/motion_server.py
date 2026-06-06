"""Motion Server コア（ROS2 非依存, v0）。

RD-Motion(.rdmotion) を MotionFrame の系列に展開し、SafetyGuard を通して安全なフレームを
逐次供給する。speed scaling / pause / phase 制御を持つ。ROS2 ノードはこのコアを駆動するだけ。
"""

from __future__ import annotations

from collections.abc import Iterator

import numpy as np

from robotdance_core.rd_motion import RdMotion
from robotdance_core.skeleton import index_of

from .messages import MotionFrame, SafetyState, SafetyStatus
from .safety_guard import SafetyGuard


class MotionServer:
    """certified RD-Motion を安全に再生するサーバ（Mode B: motion playback）。"""

    def __init__(self, motion: RdMotion, guard: SafetyGuard | None = None) -> None:
        self.motion = motion
        self.guard = guard or SafetyGuard()
        self._kps = motion.keypoints_3d_array()  # [T, J, 3]
        self._pelvis = index_of("pelvis")
        self.paused = False
        self._cursor = 0  # 現在の再生フレーム（seek/pause が操作する）
        self._seeked = False  # 直近に seek されたか（自然前進と区別）
        # アクチュエータ関節角（actuator-space IK の出力があれば）。
        jr = motion.joint_rotations or {}
        self._joint_names: list[str] = list(jr.get("actuated_joint_names", []))
        self._joint_angles = (
            np.asarray(jr["angles_rad"], dtype=np.float64) if jr.get("angles_rad") else None
        )

    def _frame_at(self, i: int) -> MotionFrame:
        cs = self.motion.contact_schedule or {}
        contacts = {
            k: bool(np.asarray(v, dtype=bool)[i]) if i < len(v) else False
            for k, v in cs.items()
        }
        return MotionFrame(
            index=i,
            time=i / self.motion.fps,
            keypoints=self._kps[i],
            base_position=self._kps[i, self._pelvis],
            contacts=contacts,
            phase=i / max(self._kps.shape[0] - 1, 1),
            joint_names=self._joint_names,
            joint_angles=self._joint_angles[i] if self._joint_angles is not None else None,
        )

    def precheck(self) -> SafetyState:
        """再生前の certificate ゲート。"""
        return self.guard.check_certificate(self.motion)

    # --- 再生制御（ROS2 ノード/対話再生から呼ぶ） ---

    def pause(self) -> None:
        self.paused = True

    def resume(self) -> None:
        self.paused = False

    def seek_frame(self, index: int) -> int:
        """再生位置をフレーム index へ移動（範囲はクランプ）。実 cursor を返す。"""
        n = self._kps.shape[0]
        self._cursor = int(max(0, min(index, n - 1)))
        self._seeked = True  # 次フレームはこの位置を表示（自然前進の +1 を打ち消す）
        return self._cursor

    def seek_phase(self, phase: float) -> int:
        """再生位置を phase（0..1）へ移動。実 cursor を返す。"""
        n = self._kps.shape[0]
        return self.seek_frame(round(float(np.clip(phase, 0.0, 1.0)) * (n - 1)))

    def stream(self) -> Iterator[tuple[MotionFrame, SafetyState]]:
        """安全フレームを逐次 yield する。ABORT が出たら停止する。

        speed_scale により実時間 dt は変わるが、フレーム列自体は等間隔（time は元クリップ基準）。
        **pause 中は cursor を進めず同じフレームを保持して yield し続ける**（resume で再開）。
        消費側は yield 間に `pause`/`resume`/`seek_*` を呼んで対話的に再生位置を操作できる。
        cursor は最後まで進むと終了する（pause を解除しない限り無限保持＝呼び出し側責任）。
        """
        pre = self.precheck()
        if pre.is_abort:
            return
        n = self._kps.shape[0]
        base_dt = 1.0 / self.motion.fps
        prev_safe: MotionFrame | None = None
        self._cursor = 0
        self._seeked = False
        while 0 <= self._cursor < n:
            target = self._frame_at(self._cursor)
            dt = base_dt / max(self.guard.speed_scale, 1e-3)
            safe, state = self.guard.filter_frame(target, prev_safe, dt)
            yield safe, state
            if state.status is SafetyStatus.ABORT:
                return
            prev_safe = safe
            # 次フレーム決定: seek されたらその位置を表示 / pause 中は据え置き / 通常は前進。
            if self._seeked:
                self._seeked = False
            elif not self.paused:
                self._cursor += 1

    def export_frames(self) -> list[tuple[MotionFrame, SafetyState]]:
        """Mode A: 全フレームを安全整形してリストで返す（offline export / bag 用）。"""
        return list(self.stream())
