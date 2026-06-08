# Assisted Survival Benchmark

PD-only（残差ゼロ）物理追従での fight motion 生存率。
depth-refine = `stabilize_depth` + `balance_depth_refine` を retarget 前に適用。

- robots: unitree_g1, unitree_h1, unitree_h2, booster_t1, apptronik_apollo, fourier_n1
- styles: boxing, dodge, hook, karate, kathak, kick
- duration: 3.0s（karate/kathak はフィクスチャ長）

## Raw vs depth-refine

| robot | style | raw surv | ref surv | Δ surv | raw RMSE | ref RMSE |
|-------|-------|----------|----------|--------|----------|----------|
| apptronik_apollo | boxing | 1.000 | 1.000 | +0.000 | 0.631 | 0.646 |
| apptronik_apollo | dodge | 0.011 | 0.011 | +0.000 | 1.191 | 1.193 |
| apptronik_apollo | hook | 1.000 | 1.000 | +0.000 | 0.621 | 0.626 |
| apptronik_apollo | karate | 1.000 | 1.000 | +0.000 | 0.659 | 0.674 |
| apptronik_apollo | kathak | 1.000 | 1.000 | +0.000 | 0.632 | 0.617 |
| apptronik_apollo | kick | 1.000 | 1.000 | +0.000 | 0.325 | 0.323 |
| booster_t1 | boxing | 1.000 | 1.000 | +0.000 | 0.759 | 0.767 |
| booster_t1 | dodge | 1.000 | 1.000 | +0.000 | 0.811 | 0.813 |
| booster_t1 | hook | 1.000 | 1.000 | +0.000 | 0.725 | 0.727 |
| booster_t1 | karate | 1.000 | 1.000 | +0.000 | 0.612 | 0.640 |
| booster_t1 | kathak | 1.000 | 1.000 | +0.000 | 0.504 | 0.485 |
| booster_t1 | kick | 0.022 | 1.000 | +0.978 | 0.755 | 0.472 |
| fourier_n1 | boxing | 1.000 | 1.000 | +0.000 | 0.413 | 0.410 |
| fourier_n1 | dodge | 1.000 | 1.000 | +0.000 | 0.497 | 0.497 |
| fourier_n1 | hook | 1.000 | 1.000 | +0.000 | 0.381 | 0.379 |
| fourier_n1 | karate | 1.000 | 1.000 | +0.000 | 0.628 | 0.642 |
| fourier_n1 | kathak | 1.000 | 1.000 | +0.000 | 0.599 | 0.585 |
| fourier_n1 | kick | 1.000 | 1.000 | +0.000 | 0.455 | 0.454 |
| unitree_g1 | boxing | 1.000 | 1.000 | +0.000 | 0.406 | 0.401 |
| unitree_g1 | dodge | 1.000 | 1.000 | +0.000 | 0.488 | 0.490 |
| unitree_g1 | hook | 1.000 | 1.000 | +0.000 | 0.380 | 0.383 |
| unitree_g1 | karate | 1.000 | 1.000 | +0.000 | 0.621 | 0.633 |
| unitree_g1 | kathak | 1.000 | 1.000 | +0.000 | 0.593 | 0.579 |
| unitree_g1 | kick | 0.022 | 1.000 | +0.978 | 0.409 | 0.419 |
| unitree_h1 | boxing | 0.011 | 1.000 | +0.989 | 0.567 | 0.489 |
| unitree_h1 | dodge | 1.000 | 1.000 | +0.000 | 0.664 | 0.665 |
| unitree_h1 | hook | 1.000 | 1.000 | +0.000 | 0.508 | 0.503 |
| unitree_h1 | karate | 1.000 | 1.000 | +0.000 | 0.598 | 0.619 |
| unitree_h1 | kathak | 1.000 | 1.000 | +0.000 | 0.555 | 0.542 |
| unitree_h1 | kick | 1.000 | 1.000 | +0.000 | 0.257 | 0.260 |
| unitree_h2 | boxing | 1.000 | 1.000 | +0.000 | 0.376 | 0.372 |
| unitree_h2 | dodge | 1.000 | 0.011 | -0.989 | 0.488 | 1.379 |
| unitree_h2 | hook | 1.000 | 1.000 | +0.000 | 0.351 | 0.339 |
| unitree_h2 | karate | 1.000 | 1.000 | +0.000 | 0.575 | 0.594 |
| unitree_h2 | kathak | 1.000 | 1.000 | +0.000 | 0.548 | 0.534 |
| unitree_h2 | kick | 0.022 | 1.000 | +0.978 | 0.343 | 0.415 |

## Rescued by depth-refine

- **unitree_h1 / boxing**: 0.011 → 1.000 (Δ +0.989)
- **booster_t1 / kick**: 0.022 → 1.000 (Δ +0.978)
- **unitree_g1 / kick**: 0.022 → 1.000 (Δ +0.978)
- **unitree_h2 / kick**: 0.022 → 1.000 (Δ +0.978)

## Regressed (honest)

- **unitree_h2 / dodge**: 1.000 → 0.011 (Δ -0.989)

> ⚠️ v0 baseline: PD-only assisted playback。真の 2 体接触スパーリングは未対応。
