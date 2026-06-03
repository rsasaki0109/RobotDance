"""ROS2 runtime コア（safety guard / motion server）の検証。

コアは ROS2 非依存なので rclpy なしでテストできる。rclpy ノードは importorskip でスモーク。
"""

from __future__ import annotations

import numpy as np
import pytest

from robotdance_core.rd_motion import RdMotion
from robotdance_core.skeleton import NUM_JOINTS
from robotdance_core.synthetic import generate_dance
from robotdance_retarget.kinematic import retarget
from robotdance_ros2.messages import MotionFrame, SafetyStatus
from robotdance_ros2.motion_server import MotionServer
from robotdance_ros2.safety_guard import (
    SafetyGuard,
    SafetyLimits,
    clamp_joint_trajectory,
)
from robotdance_unitree import get_morphology


def _certified(passed: bool, *, with_joints: bool = False) -> RdMotion:
    motion = retarget(generate_dance(duration=1.0), get_morphology("unitree_g1"))
    motion.sim_certificate = {"passed": passed, "verdict": "PASS" if passed else "REJECT",
                              "reasons": [] if passed else ["airborne"]}
    if with_joints:
        n = motion.num_frames
        names = [f"j{i}" for i in range(23)]
        motion.joint_rotations = {"actuated_joint_names": names,
                                  "angles_rad": np.zeros((n, 23)).tolist()}
    return motion


# --- safety guard: certificate gate ---

def test_guard_blocks_missing_certificate() -> None:
    motion = retarget(generate_dance(duration=1.0), get_morphology("unitree_g1"))
    assert motion.sim_certificate is None
    state = SafetyGuard().check_certificate(motion)
    assert state.status is SafetyStatus.ABORT


def test_guard_blocks_rejected_certificate() -> None:
    assert SafetyGuard().check_certificate(_certified(False)).is_abort


def test_guard_passes_certified() -> None:
    assert SafetyGuard().check_certificate(_certified(True)).status is SafetyStatus.OK


def test_guard_allow_uncertified_when_configured() -> None:
    motion = retarget(generate_dance(duration=1.0), get_morphology("unitree_g1"))
    guard = SafetyGuard(SafetyLimits(require_certificate=False))
    assert guard.check_certificate(motion).status is SafetyStatus.OK


def test_estop_aborts() -> None:
    guard = SafetyGuard()
    guard.estop()
    assert guard.check_certificate(_certified(True)).is_abort


# --- safety guard: frame filtering ---

def _frame(i: int, z: float = 0.7, x: float = 0.0) -> MotionFrame:
    kp = np.zeros((NUM_JOINTS, 3))
    kp[:, 0] = x
    kp[:, 2] = z
    return MotionFrame(index=i, time=i * 0.033, keypoints=kp, base_position=np.array([0.0, 0.0, z]))


def test_velocity_clamp() -> None:
    guard = SafetyGuard(SafetyLimits(max_link_speed=6.0))
    prev = _frame(0, x=0.0)
    target = _frame(1, x=10.0)  # 10m を 0.033s → 約 300 m/s（過大）
    safe, state = guard.filter_frame(target, prev, dt=0.033)
    peak = np.linalg.norm(safe.keypoints - prev.keypoints, axis=1).max() / 0.033
    assert peak <= 6.0 + 1e-6
    assert state.status is SafetyStatus.WARNING


def test_fall_detection_aborts() -> None:
    guard = SafetyGuard(SafetyLimits(max_base_tilt_drop=0.35))
    guard.filter_frame(_frame(0, z=0.7), None, dt=0.033)        # nominal z=0.7
    _, state = guard.filter_frame(_frame(1, z=0.3), _frame(0, z=0.7), dt=0.033)  # 0.3 < 0.7*0.65
    assert state.is_abort


# --- safety guard: joint-space limit enforcement (§5.6) ---

def test_clamp_joint_trajectory_bounds_position_and_velocity() -> None:
    """関節角列の位置・速度が limit 内に整形される（offline export）。"""
    dt = 1.0 / 30.0
    raw = np.zeros((40, 4))
    raw[:, 0] = np.linspace(0.0, 5.0, 40)  # 緩やかに位置 limit(±2)超過 → 位置クランプ
    raw[20, 1] = 9.0                        # 単発スパイク → 速度クランプ
    # 加速度を実質無制限にし、速度クランプを主役にする（位置 vs 速度を分離して検査）。
    limits = SafetyLimits(
        max_joint_speed=10.0, max_joint_accel=1e9,
        joint_position_limits={f"j{i}": (-2.0, 2.0) for i in range(4)},
    )
    safe, rep = clamp_joint_trajectory(raw, dt, limits, [f"j{i}" for i in range(4)])
    # 位置は ±2.0 に bound。
    assert float(np.abs(safe).max()) <= 2.0 + 1e-9
    # 速度は max_joint_speed に bound。
    assert rep["safe_max_joint_speed_rad_s"] <= 10.0 + 1e-6
    # raw は明確に超過していた。
    assert rep["raw_max_joint_speed_rad_s"] > 10.0
    assert rep["position_limit_frames"] > 0
    assert rep["velocity_clamp_frames"] > 0


def test_clamp_joint_trajectory_bounds_acceleration() -> None:
    """加速度 limit が往復ジャークの加速度を下げる（best-effort）。"""
    dt = 1.0 / 30.0
    raw = np.zeros((30, 2))
    raw[15, 0] = 2.0
    raw[16, 0] = -2.0  # 大きなジャーク
    soft = SafetyLimits(max_joint_speed=1e9, max_joint_accel=50.0, default_joint_range=100.0)
    hard = SafetyLimits(max_joint_speed=1e9, max_joint_accel=1e12, default_joint_range=100.0)
    _, rep_soft = clamp_joint_trajectory(raw, dt, soft)
    _, rep_hard = clamp_joint_trajectory(raw, dt, hard)
    assert rep_soft["accel_clamp_frames"] > 0
    assert rep_soft["safe_max_joint_accel_rad_s2"] < rep_hard["safe_max_joint_accel_rad_s2"]


def test_clamp_default_range_when_no_limits() -> None:
    """位置 limit 未指定なら ±default_joint_range で clamp する。"""
    dt = 1.0 / 30.0
    raw = np.zeros((5, 2))
    raw[2, 0] = 100.0
    limits = SafetyLimits(default_joint_range=1.0, max_joint_speed=1e9, max_joint_accel=1e12)
    safe, _ = clamp_joint_trajectory(raw, dt, limits)
    assert float(np.abs(safe).max()) <= 1.0 + 1e-9


def test_guard_clamps_joint_angles_in_frame() -> None:
    """filter_frame が関節角の過大速度をクランプし WARNING を出す。"""
    limits = SafetyLimits(max_joint_speed=5.0, require_certificate=False,
                          joint_position_limits=None, default_joint_range=100.0)
    guard = SafetyGuard(limits)
    names = [f"j{i}" for i in range(3)]
    f0 = MotionFrame(index=0, time=0.0, keypoints=np.zeros((NUM_JOINTS, 3)),
                     base_position=np.array([0.0, 0.0, 0.7]),
                     joint_names=names, joint_angles=np.zeros(3))
    f1 = MotionFrame(index=1, time=0.033, keypoints=np.zeros((NUM_JOINTS, 3)),
                     base_position=np.array([0.0, 0.0, 0.7]),
                     joint_names=names, joint_angles=np.array([1.0, 0.0, 0.0]))  # 1rad/0.033s≈30rad/s
    guard.filter_frame(f0, None, dt=0.033)
    safe, state = guard.filter_frame(f1, f0, dt=0.033)
    realized_speed = float(np.abs(safe.joint_angles - f0.joint_angles).max() / 0.033)
    assert realized_speed <= 5.0 + 1e-6
    assert state.status is SafetyStatus.WARNING


def test_guard_passes_safe_joint_motion() -> None:
    """limit 内の関節運動はクランプされず OK のまま通る。"""
    limits = SafetyLimits(max_joint_speed=12.0, require_certificate=False, warn_joint_speed=8.0,
                          default_joint_range=100.0)
    guard = SafetyGuard(limits)
    names = [f"j{i}" for i in range(3)]
    f0 = MotionFrame(index=0, time=0.0, keypoints=np.zeros((NUM_JOINTS, 3)),
                     base_position=np.array([0.0, 0.0, 0.7]),
                     joint_names=names, joint_angles=np.zeros(3))
    f1 = MotionFrame(index=1, time=0.033, keypoints=np.zeros((NUM_JOINTS, 3)),
                     base_position=np.array([0.0, 0.0, 0.7]),
                     joint_names=names, joint_angles=np.array([0.05, -0.05, 0.05]))
    guard.filter_frame(f0, None, dt=0.033)
    safe, state = guard.filter_frame(f1, f0, dt=0.033)
    assert np.allclose(safe.joint_angles, f1.joint_angles)
    assert state.status is SafetyStatus.OK


# --- motion server ---

def test_server_streams_certified() -> None:
    server = MotionServer(_certified(True))
    frames = server.export_frames()
    assert len(frames) == 30  # 1s @ 30fps
    assert all(f.keypoints.shape == (NUM_JOINTS, 3) for f, _ in frames)


def test_server_carries_joint_angles() -> None:
    """actuator IK の関節角が MotionFrame に載って流れる。"""
    server = MotionServer(_certified(True, with_joints=True))
    frames = server.export_frames()
    f0, _ = frames[0]
    assert f0.joint_angles is not None and f0.joint_angles.shape == (23,)
    assert len(f0.joint_names) == 23


def test_server_blocks_uncertified() -> None:
    motion = retarget(generate_dance(duration=1.0), get_morphology("unitree_g1"))
    assert MotionServer(motion).export_frames() == []


def test_speed_scale_clamped() -> None:
    guard = SafetyGuard(speed_scale=5.0)
    assert guard.speed_scale == 1.0
    guard.set_speed_scale(-1.0)
    assert guard.speed_scale == 0.0


# --- ROS2 node smoke ---

def test_ros2_node_publishes() -> None:
    rclpy = pytest.importorskip("rclpy")
    from rclpy.executors import SingleThreadedExecutor
    from std_msgs.msg import String
    from visualization_msgs.msg import MarkerArray

    from robotdance_ros2.motion_server_node import MotionServerNode

    motion = _certified(True)
    motion.keypoints_3d = motion.keypoints_3d[:10]
    motion.duration = 10 / motion.fps

    rclpy.init()
    try:
        node = MotionServerNode(motion)
        listener = rclpy.node.Node("test_listener")
        got = {"skel": 0}
        listener.create_subscription(
            MarkerArray, "/robotdance/skeleton", lambda m: got.__setitem__("skel", got["skel"] + 1), 10)
        listener.create_subscription(String, "/robotdance/safety", lambda m: None, 10)
        exe = SingleThreadedExecutor()
        exe.add_node(node)
        exe.add_node(listener)
        for _ in range(40):
            try:
                exe.spin_once(timeout_sec=0.05)
            except SystemExit:
                break
        assert got["skel"] >= 5
        node.destroy_node()
        listener.destroy_node()
    finally:
        rclpy.shutdown()


def test_ros2_node_publishes_joint_states() -> None:
    """actuator 関節角を持つ motion は /joint_states を配信する（robot_state_publisher 連携）。"""
    rclpy = pytest.importorskip("rclpy")
    from rclpy.executors import SingleThreadedExecutor
    from sensor_msgs.msg import JointState

    from robotdance_ros2.motion_server_node import MotionServerNode

    motion = _certified(True, with_joints=True)
    motion.keypoints_3d = motion.keypoints_3d[:8]
    motion.joint_rotations["angles_rad"] = motion.joint_rotations["angles_rad"][:8]
    motion.duration = 8 / motion.fps

    rclpy.init()
    try:
        node = MotionServerNode(motion)
        listener = rclpy.node.Node("js_listener")
        got = {"js": [], "names": 0}

        def on_js(m: "JointState") -> None:
            got["js"].append(m)
            got["names"] = len(m.name)

        listener.create_subscription(JointState, "/joint_states", on_js, 10)
        exe = SingleThreadedExecutor()
        exe.add_node(node)
        exe.add_node(listener)
        for _ in range(40):
            try:
                exe.spin_once(timeout_sec=0.05)
            except SystemExit:
                break
        assert len(got["js"]) >= 3
        assert got["names"] == 23
        node.destroy_node()
        listener.destroy_node()
    finally:
        rclpy.shutdown()
