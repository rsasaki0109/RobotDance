# RD-Motion (`.rdmotion`)

> **status:** v0 (draft) · スキーマ [`rd-motion.schema.json`](rd-motion.schema.json) あり

robot-specific な実行可能モーション artifact。[RD-MIR](../rd-mir/) を特定の
[RD-Embodiment](../rd-embodiment/) へ retarget した結果で、sim / runtime で再生・評価できます。

v0 では kinematic retarget の link 位置（`keypoints_3d`）と接地スケジュールを保持し、
`sim_certificate` は Phase 2（物理 sim）で埋めます。主なフィールド:

| フィールド | 内容 |
| --- | --- |
| `robot_name` | 対象ロボット |
| `duration` | motion 長 |
| `joint_trajectory` | joint 目標軌道 |
| `base_trajectory` | base（root）軌道 |
| `contact_schedule` | 接地スケジュール |
| `control_mode` | position / velocity / torque / policy |
| `safety_envelope` | 許容運動範囲 |
| `retarget_metrics` | ik_error, foot_slip, joint_limit_rate 等 |
| `sim_certificate` | sim tracking の pass/fail と metrics |
| `source_provenance` | 元 RD-MIR / manifest への参照 |

## `sim_certificate`（v0, MuJoCo 物理検証）

`robotdance_sim` の MuJoCo backend が埋める。`null` = 未検証。例:

```json
{
  "backend": "mujoco",
  "approximate_inertia": true,
  "passed": false,
  "verdict": "REJECT",
  "metrics": {
    "airborne_ratio": 0.875,
    "balance_violation_ratio": 0.917,
    "joint_torque_nm_p50": 1646.2,
    "joint_torque_nm_peak": 1668.4,
    "torque_ratio": 20.58,
    "max_joint_ang_speed_rad_s": 38.36
  },
  "reasons": ["airborne 88%（接地なしで支持不能）", "ZMP が支持多角形外 92%（転倒リスク）", "..."],
  "note": "physically-informed feasibility（近似慣性）— 実機保証ではない（v0）"
}
```

> ⚠️ 安全設計: `.rdmotion` は static safety check → kinematic feasibility → physics validation
> (`sim_certificate`) → runtime safety guard の gate を通って初めて robot bridge に流れる
> （README / 設計方針 §5.6）。v0 の `sim_certificate` は近似慣性で実機保証ではない。
