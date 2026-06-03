# robotdance_retarget

IK, morphology mapping, contact-preserving retargeting — 人間 motion（[RD-MIR](../specs/rd-mir/)）→ robot embodiment（[RD-Embodiment](../specs/rd-embodiment/)）への retarget。

## 実装状況

- `kinematic.py` — **v0 kinematic retarget**: direction-preserving FK + morphology normalization + ground clamp。
  `retarget_to_g1(mir) -> RdMotion` で [RD-Motion](../specs/rd-motion/) を生成する。
  物理 sim は通さない（運動学プレビュー）。

```python
from robotdance_core.synthetic import generate_dance
from robotdance_retarget.kinematic import retarget_to_g1

motion = retarget_to_g1(generate_dance())
print(motion.retarget_metrics)  # height_scale, bone_direction_cosine, foot_sliding, ...
```

## 今後（Phase 2）

contact-preserving IK / joint limit optimizer、torque-aware 調整、sim tracking による feasibility 検証
（`sim_certificate` を埋める）。
