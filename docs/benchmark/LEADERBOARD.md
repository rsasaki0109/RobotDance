# RobotDance Benchmark

motions: **4** × robots: **2** = 8 runs · sim: **on**

> ⚠️ v0: 近似形態プロキシ + 近似慣性。実機保証ではない（各 README 参照）。

## Leaderboard（robot 別集計）

| robot | runs | PASS率 | 平均 bone方向cos | 平均 foot_sliding | 平均 height_scale | 平均 屈曲違反率 |
| --- | --- | --- | --- | --- | --- | --- |
| unitree_g1 | 4 | 0.500 | 1.000 | 0.025 | 0.920 | 0.000 |
| unitree_h1 | 4 | 0.750 | 1.000 | 0.032 | 1.186 | 0.000 |

## 全 run（motion × robot）

| motion | class | robot | verdict | airborne | balance | torque× | 角速度 | foot_slide | bone_cos | 屈曲違反 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| dance_normal | dance | unitree_g1 | PASS | 0.000 | 0.000 | 0.239 | 4.950 | 0.004 | 1.000 | 0.000 |
| dance_normal | dance | unitree_h1 | PASS | 0.000 | 0.000 | 0.627 | 4.980 | 0.004 | 1.000 | 0.000 |
| dance_fast | dance | unitree_g1 | REJECT | 0.000 | 0.375 | 0.239 | 7.950 | 0.006 | 1.000 | 0.000 |
| dance_fast | dance | unitree_h1 | PASS | 0.000 | 0.000 | 0.627 | 7.990 | 0.006 | 1.000 | 0.000 |
| idle | dance | unitree_g1 | PASS | 0.000 | 0.000 | 0.131 | 0.770 | 0.002 | 1.000 | 0.000 |
| idle | dance | unitree_h1 | PASS | 0.000 | 0.000 | 0.339 | 0.770 | 0.002 | 1.000 | 0.000 |
| backflip | backflip | unitree_g1 | REJECT | 0.875 | 0.938 | 0.414 | 17.200 | 0.089 | 1.000 | 0.000 |
| backflip | backflip | unitree_h1 | REJECT | 0.875 | 0.938 | 0.453 | 23.720 | 0.116 | 1.000 | 0.000 |
