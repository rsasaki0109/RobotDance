# robotdance_sim

Isaac Lab / MuJoCo / Genesis-style backend adapters — 物理シミュレーション backend の adapter 層。

## 実装状況

- `mjcf.py` — `RobotMorphology` → MuJoCo MJCF（bone ごとに独立 ball joint を持つ多体ツリー、
  質量 ∝ bone 長）。`ground=False` で純浮遊多体（逆動力学に接触力が混入しない）。
- `backend.py` — **sim backend 抽象 + registry**（§4.3）。`SimBackend` 契約（`passed` / `verdict` /
  `backend` / `metrics` / `reasons` の certificate dict）と registry を提供し、
  `certify(motion, morphology, backend="mujoco")` で backend を選べる。`backend_status()` で利用可否を
  確認。MuJoCo を参照実装、Isaac Lab を contract のみの scaffold（未インストールなら明示エラー）として登録。
  **Isaac Lab 本体（Omniverse 依存・大容量）は同梱・実行しない**（license/容量 safe）。
- `mujoco_backend.py` — **MuJoCo 物理ベースの feasibility 検証**。`simulate_certificate(motion, morphology)`
  / `certify(motion, morphology)` が [RD-Motion](../specs/rd-motion/) の `sim_certificate` を計算する。
- `tracking_env.py` — **RL tracking 環境**（§4.5）。`TrackingEnv(reference, morphology)` が参照運動を
  **forward 物理シミュレーション上で追従**する gym 風 env。base（pelvis）は free joint で**非駆動**、
  駆動 DOF は関節空間 PD（参照 qpos へアンカー）+ 方策の残差トルク（`qfrc_applied[6:]`）。
  報酬 = 姿勢追従 + 直立 + 生存 − 制御コスト、転倒で終了。学習は
  [`robotdance_models.tracking_policy`](../robotdance_models/) の PPO で行う。
  `MultiTrackingEnv(references, morphology)` は**参照スイート**を保持し、エピソードごとに参照を
  round-robin で切り替える（`reset(idx)` で `ref_qpos`/`T` を rebind）→ **1 方策が複数運動を追従**。

検証する量（受動 forward sim は判別力がないため、**参照運動の実現可能性**を検証）。
**動的**（転倒/トルク/滞空/角速度）に加え、per_joint_limits を持つ embodiment では**運動学的**
（関節可動域）feasibility も統合し、いずれか違反で REJECT:

| 信号 | 手法 | 判定 |
| --- | --- | --- |
| torque saturation | 逆動力学 `mj_inverse`（純 RNEA）→ 内部 joint トルク | p50 が actuator 限界超過 |
| balance / 転倒 | 質量モデルの COM → ZMP vs 接地足の支持多角形 | ZMP が支持外 >30% |
| 滞空 | contact_schedule に接地なし | airborne >10% |
| 過大運動 | bone 方向角速度（twist-free） | >30 rad/s |
| 可動域超過 | retarget の joint_flexion 違反（膝・肘 vs 実機 ROM） | 違反フレーム >0% |

> **qpos 復元の twist 安定化**: keypoints → qpos は `_poses_to_qpos` で時系列復元する。frame 0 を
> rest 基準の shortest-arc で seed し、以降は連続フレーム間の swing だけで bone フレームを前進させる
> ため、極端な屈曲（観測方向が rest と反平行付近）でも shortest-arc 特異点を踏まず、bone 軸まわりの
> 偽 twist スパイク（実測 ~80 rad/s）が出ない。bone 方向は厳密再現で位置・COM・トルクは不変。これにより
> RL tracking の reference 速度・PD 追従誤差や export 軌道など qpos を差分する全経路が clean に保たれる。
> 効果の定量比較は [`docs/sim/REFERENCE_QUALITY.md`](../docs/sim/REFERENCE_QUALITY.md)（overbend 20×・
> backflip 5-7× の偽スパイク除去）。`python3 -m robotdance_sim.reference_quality` で再生成。

```python
from robotdance_sim.backend import certify, backend_status
certify(motion, morphology, backend="mujoco")   # backend を選んで sim_certificate を格納
backend_status()   # [{'name':'isaaclab','available':False}, {'name':'mujoco','available':True}]
```
```bash
robotdance sim-backends                              # 登録 backend と利用可否
robotdance validate-sim dance.rdmir.json --backend mujoco
```

> ⚠️ **v0 注意:** 質量・慣性は bone 長比の**近似**であり実機値ではない。ball-joint 近似のため
> 腕が頭上で rest と反平行になる特異姿勢でトルク peak がスパイクする（判定は robust な median を使用）。
> 出力は "physically-informed feasibility" であって**実機保証ではない**。Isaac Lab backend / 実 URDF・
> 慣性は今後。`pip install -e ".[sim]"` で mujoco を入れる。
>
> **TrackingEnv（v0 baseline）注意:** 近似質量・素朴な報酬/終了条件の **baseline 足場**であり、
> SOTA tracking（DeepMimic/AMP 等）ではない。短い feasible クリップでは関節 PD だけで概ねバランス
> するため、v0 の PPO 残差は PD を**壊さず追従する**ことを学ぶ（PD 超えの tracking 精度・多様 motion
> 汎化・摂動頑健性・実機転移は今後）。`[sim]` + `[learn]` extra が必要。
