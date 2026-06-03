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

> ⚠️ 安全設計: `.rdmotion` は static safety check → kinematic feasibility → physics validation →
> runtime safety guard の gate を通って初めて robot bridge に流れる（README / 設計方針 §5.6）。
