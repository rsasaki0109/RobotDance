# RobotDance Benchmark

motions: **8** × robots: **4** = 32 runs · sim: **on**

> ⚠️ v0: 近似形態プロキシ。sim は実 URDF 慣性テンソルで検証（v0.52）。実機保証ではない（各 README 参照）。

## Leaderboard（robot 別集計）

| robot | runs | PASS率 | 平均 bone方向cos | 平均 foot_sliding | 平均 height_scale | 平均 屈曲違反率 | 平均 動的tq(N·m) |
| --- | --- | --- | --- | --- | --- | --- | --- |
| unitree_g1 | 8 | 0.500 | 1.000 | 0.016 | 0.886 | 0.031 | 35.200 |
| unitree_h1 | 8 | 0.500 | 1.000 | 0.019 | 1.142 | 0.000 | 60.775 |
| booster_t1 | 8 | 0.625 | 1.000 | 0.014 | 0.671 | 0.023 | 23.850 |
| apptronik_apollo | 8 | 0.625 | 1.000 | 0.020 | 1.111 | 0.000 | 124.625 |

## 全 run（motion × robot）

> `torque×` = 動的tq / 実 per-joint effort 上限の最大（>1.0 で REJECT）。`重力tq` は重力保持（準静的）
> 成分、`動的tq` は重力＋並進＋回転慣性の合計（v0.62/v0.63）。両者の差が**慣性寄与**で、速い運動ほど開く。

| motion | class | robot | verdict | airborne | balance | torque× | 重力tq | 動的tq | 角速度 | foot_slide | bone_cos | 屈曲違反 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| dance_normal | dance | unitree_g1 | PASS | 0.000 | 0.000 | 0.424 | 9.400 | 34.600 | 7.260 | 0.004 | 1.000 | 0.000 |
| dance_normal | dance | unitree_h1 | PASS | 0.000 | 0.000 | 0.863 | 27.700 | 53.900 | 7.260 | 0.004 | 1.000 | 0.000 |
| dance_normal | dance | booster_t1 | PASS | 0.000 | 0.000 | 0.906 | 6.000 | 27.200 | 7.260 | 0.004 | 1.000 | 0.000 |
| dance_normal | dance | apptronik_apollo | PASS | 0.000 | 0.000 | 0.950 | 39.300 | 114.000 | 7.260 | 0.004 | 1.000 | 0.000 |
| dance_fast | dance | unitree_g1 | REJECT | 0.000 | 0.458 | 0.668 | 9.400 | 31.400 | 11.630 | 0.006 | 1.000 | 0.000 |
| dance_fast | dance | unitree_h1 | REJECT | 0.000 | 0.000 | 1.702 | 27.700 | 55.500 | 11.630 | 0.006 | 1.000 | 0.000 |
| dance_fast | dance | booster_t1 | PASS | 0.000 | 0.000 | 0.677 | 6.000 | 24.000 | 11.630 | 0.006 | 1.000 | 0.000 |
| dance_fast | dance | apptronik_apollo | PASS | 0.000 | 0.000 | 0.957 | 39.300 | 114.900 | 11.630 | 0.006 | 1.000 | 0.000 |
| idle | dance | unitree_g1 | PASS | 0.000 | 0.000 | 0.152 | 9.400 | 10.000 | 0.930 | 0.002 | 1.000 | 0.000 |
| idle | dance | unitree_h1 | PASS | 0.000 | 0.000 | 0.361 | 27.700 | 28.900 | 0.930 | 0.002 | 1.000 | 0.000 |
| idle | dance | booster_t1 | PASS | 0.000 | 0.000 | 0.216 | 6.000 | 6.500 | 0.930 | 0.002 | 1.000 | 0.000 |
| idle | dance | apptronik_apollo | PASS | 0.000 | 0.000 | 0.343 | 39.300 | 41.200 | 0.930 | 0.002 | 1.000 | 0.000 |
| backflip | backflip | unitree_g1 | REJECT | 0.875 | 0.938 | 0.521 | 34.200 | 45.800 | 4.010 | 0.089 | 1.000 | 0.000 |
| backflip | backflip | unitree_h1 | REJECT | 0.875 | 0.938 | 1.198 | 62.400 | 95.400 | 4.010 | 0.116 | 1.000 | 0.000 |
| backflip | backflip | booster_t1 | REJECT | 0.875 | 0.958 | 1.646 | 39.700 | 49.400 | 4.010 | 0.072 | 1.000 | 0.000 |
| backflip | backflip | apptronik_apollo | REJECT | 0.875 | 0.938 | 1.127 | 86.800 | 135.200 | 4.010 | 0.121 | 1.000 | 0.000 |
| overbend | overbend | unitree_g1 | REJECT | 0.000 | 0.000 | 0.226 | 9.200 | 9.200 | 3.920 | 0.000 | 1.000 | 0.250 |
| overbend | overbend | unitree_h1 | PASS | 0.000 | 0.000 | 0.498 | 27.700 | 27.700 | 3.920 | 0.000 | 1.000 | 0.000 |
| overbend | overbend | booster_t1 | REJECT | 0.000 | 0.000 | 0.221 | 6.000 | 6.000 | 3.920 | 0.000 | 1.000 | 0.183 |
| overbend | overbend | apptronik_apollo | PASS | 0.000 | 0.000 | 0.324 | 38.900 | 38.900 | 3.920 | 0.000 | 1.000 | 0.000 |
| squat | squat | unitree_g1 | PASS | 0.000 | 0.000 | 0.283 | 13.200 | 13.800 | 2.200 | 0.001 | 1.000 | 0.000 |
| squat | squat | unitree_h1 | PASS | 0.000 | 0.000 | 0.582 | 28.500 | 32.600 | 2.200 | 0.000 | 1.000 | 0.000 |
| squat | squat | booster_t1 | PASS | 0.000 | 0.000 | 0.266 | 7.400 | 7.800 | 2.200 | 0.001 | 1.000 | 0.000 |
| squat | squat | apptronik_apollo | PASS | 0.000 | 0.000 | 0.447 | 48.100 | 53.600 | 2.200 | 0.000 | 1.000 | 0.000 |
| march | march | unitree_g1 | REJECT | 0.000 | 0.583 | 1.189 | 15.600 | 104.600 | 5.610 | 0.018 | 1.000 | 0.000 |
| march | march | unitree_h1 | REJECT | 0.000 | 0.450 | 1.071 | 29.900 | 142.600 | 5.610 | 0.018 | 1.000 | 0.000 |
| march | march | booster_t1 | REJECT | 0.000 | 0.450 | 1.781 | 9.300 | 53.400 | 5.610 | 0.016 | 1.000 | 0.000 |
| march | march | apptronik_apollo | REJECT | 0.000 | 0.583 | 3.169 | 54.100 | 380.300 | 5.610 | 0.019 | 1.000 | 0.000 |
| march_gentle | march | unitree_g1 | PASS | 0.000 | 0.150 | 0.366 | 12.400 | 32.200 | 1.570 | 0.008 | 1.000 | 0.000 |
| march_gentle | march | unitree_h1 | REJECT | 0.000 | 0.850 | 0.349 | 28.500 | 49.600 | 1.570 | 0.008 | 1.000 | 0.000 |
| march_gentle | march | booster_t1 | PASS | 0.000 | 0.017 | 0.550 | 7.500 | 16.500 | 1.570 | 0.007 | 1.000 | 0.000 |
| march_gentle | march | apptronik_apollo | REJECT | 0.000 | 0.850 | 0.991 | 46.300 | 118.900 | 1.570 | 0.008 | 1.000 | 0.000 |
