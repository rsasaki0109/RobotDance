# robotdance_benchmarks

extraction / retarget / sim tracking benchmark — 抽出・retarget・追従の評価。

## 実装状況

- `suite.py` — `run_benchmark(motions, robots)`: motion × robot を full pipeline
  （retarget → MuJoCo 物理検証）に通し、既存の全指標を 1 行 = 1 (motion, robot) に集約。
  `default_motion_suite()` は権利クリーンな合成スイート（dance/idle/backflip + **overbend**＝肘を
  実機可動域上限超で折り `joint_flexion_violation>0` を出す ROM 違反デモ）。`run_from_dir()` で `*.rdmir.json` も可。
- `report.py` — CSV 出力 + Markdown **leaderboard**（robot 別 PASS率・平均指標）。
- `extraction.py` — **抽出 adapter benchmark**（§4.1）: MediaPipe / HMR(4DHumans/GVHMR) 等の
  video→RD-MIR 抽出を**共通 GT に対し定量比較**。`extraction_metrics(gt, pred)` が **MPJPE**
  （root-relative）/ **PA-MPJPE**（Umeyama 相似整列後）/ **PCK@5cm·10cm** / **MPJVE**（速度誤差）/
  **jitter**（滑らかさ）/ **bone-length MAE** を計算。`compare_extractions(gt, {name: pred})` →
  MPJPE 昇順の leaderboard（CSV/Markdown）。純 numpy・画像不要。

```bash
robotdance benchmark --robots unitree_g1 unitree_h1 -o out/   # retarget×sim leaderboard
robotdance benchmark-extraction --out-md extraction.md        # 抽出品質 leaderboard（§4.1）
```

集約する指標: retarget（height_scale, bone_direction_cosine, foot_sliding,
**joint_flexion_violation** = 膝・肘の屈曲が実 per-joint 可動域上限を超えたフレーム比, G1/H1 のみ）、
sim_certificate（verdict, airborne, balance, torque_ratio, **gravity_torque_nm**＝重力保持成分 /
**dynamic_torque_nm**＝重力＋並進＋回転慣性の合計 — 両者の差が慣性寄与, ang_speed）、source 品質（confidence, jitter）、
extraction（MPJPE / PA-MPJPE / PCK / MPJVE / jitter / bone-length MAE）。

サンプル結果（合成スイート × G1/H1）: [`../docs/benchmark/LEADERBOARD.md`](../docs/benchmark/LEADERBOARD.md)。

> ⚠️ v0 は近似形態プロキシ + 近似質量。mujoco 未インストール時は sim 指標が None になる
> （retarget 指標のみ）。**extraction benchmark** は評価ハーネスで、同梱デモは合成 GT への模擬劣化
> （MediaPipe 風 jitter / HMR 風 骨長近似）であり実モデルの精度主張ではない — 実比較は実 video の
> 抽出結果と GT を渡して行う。実動画 GT スイート・leaderboard 提出フローは今後。
