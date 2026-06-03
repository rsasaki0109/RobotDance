# robotdance_sim

Isaac Lab / MuJoCo / Genesis-style backend adapters — 物理シミュレーション backend の adapter 層。

## 実装状況

- `mjcf.py` — `RobotMorphology` → MuJoCo MJCF（bone ごとに独立 ball joint を持つ多体ツリー、
  質量 ∝ bone 長）。`ground=False` で純浮遊多体（逆動力学に接触力が混入しない）。
- `mujoco_backend.py` — **MuJoCo 物理ベースの feasibility 検証**。`simulate_certificate(motion, morphology)`
  / `certify(motion, morphology)` が [RD-Motion](../specs/rd-motion/) の `sim_certificate` を計算する。

検証する物理量（受動 forward sim は判別力がないため、**参照運動の実現可能性**を検証）:

| 信号 | 手法 | 判定 |
| --- | --- | --- |
| torque saturation | 逆動力学 `mj_inverse`（純 RNEA）→ 内部 joint トルク | p50 が actuator 限界超過 |
| balance / 転倒 | 質量モデルの COM → ZMP vs 接地足の支持多角形 | ZMP が支持外 >30% |
| 滞空 | contact_schedule に接地なし | airborne >10% |
| 過大運動 | ball joint 角速度 | >30 rad/s |

```python
from robotdance_sim.mujoco_backend import certify
certify(motion, morphology)        # motion.sim_certificate にPASS/REJECTと指標を格納
```

> ⚠️ **v0 注意:** 質量・慣性は bone 長比の**近似**であり実機値ではない。ball-joint 近似のため
> 腕が頭上で rest と反平行になる特異姿勢でトルク peak がスパイクする（判定は robust な median を使用）。
> 出力は "physically-informed feasibility" であって**実機保証ではない**。Isaac Lab backend / 実 URDF・
> 慣性は今後。`pip install -e ".[sim]"` で mujoco を入れる。
