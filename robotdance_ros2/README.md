# robotdance_ros2

messages, motion server, safety guard, robot adapters — 安全な motion runtime としての ROS2 層。

## 実装状況（ROS2 Jazzy 想定）

| module | 役割 | ROS2 依存 |
| --- | --- | --- |
| `messages.py` | MotionFrame / SafetyState 等の dataclass 契約 | なし |
| `safety_guard.py` | **Safety Guard（§5.6）**: certificate gate, Cartesian 速度クランプ, **joint 空間 位置/速度/加速度クランプ**, 転倒検知, E-stop, speed scaling | なし |
| `motion_server.py` | **Motion Server**: .rdmotion → 安全フレーム逐次供給（pause/speed/phase） | なし |
| `motion_server_node.py` | rclpy ノード: MarkerArray(skeleton) + SafetyState を配信、E-stop 購読 | rclpy |

```bash
robotdance serve g1.rdmotion.json --speed 0.5        # dry-run（ROS2 不要）で安全ゲート検証
robotdance serve g1.rdmotion.json --ros2             # ROS2 配信（RViz で可視化）
robotdance demo-runtime                              # certificate PASS は再生 / REJECT は遮断
robotdance demo-joint-safety                         # 関節 位置/速度/加速度クランプを実演（§5.6）
```

ROS2 topic（`--ros2`）:
- `/joint_states` `sensor_msgs/JointState`（**実 G1 の実関節角**, actuator-space IK 出力がある場合）
- `/robotdance/skeleton` `visualization_msgs/MarkerArray`（canonical bone, RViz 可視化）
- `/robotdance/safety` `std_msgs/String`（SafetyState JSON）
- `/robotdance/estop` `std_msgs/Bool`（True で緊急停止）

### 実 G1 メッシュを RViz で動かす

`retarget-ik` の実関節角 → `/joint_states` → `robot_state_publisher` + 実 URDF → RViz で本物の G1 が動く:

```bash
robotdance retarget-ik dance.rdmir.json --urdf g1_23dof.urdf -o g1_joints.rdmotion.json
ros2 launch robotdance_ros2/launch/g1_rviz.launch.py \
    urdf:=g1_23dof.urdf rdmotion:=g1_joints.rdmotion.json
```

## 安全設計（§5.6）

motion artifact は **certificate gate → Cartesian 速度/転倒チェック → joint 空間 limit クランプ → E-stop**
を通って初めて配信される。`sim_certificate` が無い / REJECT の motion は **再生前に ABORT**
（`demo-runtime` で実演）。

**joint 空間 limit enforcement（§5.6）**: actuator-space IK / tracking policy が出す関節角列を、
実機コマンド直前に **位置 limit・速度・加速度**へクランプする。`sim_certificate`（物理的妥当性）の
**先**にある最終 gate で、コマンド自体を機構的に安全な範囲へ整形する:
- frame 逐次: `SafetyGuard.filter_frame()` が `MotionFrame.joint_angles` をクランプ（stateful）
- 一括 export: `clamp_joint_trajectory(angles, dt, limits, names)` が軌道全体を整形し report 返却
- `SafetyLimits` に `max_joint_speed` / `max_joint_accel` / `joint_position_limits` を設定

```bash
robotdance demo-joint-safety    # raw の過大速度/加速度/位置超過を limit 内へ整形して可視化
```

> ⚠️ **v0:** core は ROS2 非依存で完全テスト可能。ノードは ROS2 Jazzy（primary target, §5.1）で動作。
> **sim-first** — 実機 bridge（unitree_sdk2）は安全レビュー後に別途接続。位置/速度は厳密 bound、
> 加速度は best-effort（位置 clamp の減速で残りうる）。**トルク/電流 limit**・ros2_control 連携・
> 実機再生は今後。本パッケージは pip monorepo に同居するが、custom .msg の colcon 化も今後の課題。
