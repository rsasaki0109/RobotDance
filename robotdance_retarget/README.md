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

- `actuator_ik.py` — **アクチュエータ空間 retarget**（torch, `[learn]` extra）。実 URDF の
  **微分可能 FK** を構成し、勾配 IK で **実 G1 の 23 関節角**を解く。出力 `.rdmotion` の
  `joint_rotations` に実機が command できる joint 角を格納。

```bash
robotdance retarget-ik dance.rdmir.json --urdf g1_23dof.urdf -o g1_joints.rdmotion.json
```

IK 位置誤差は「人間の動きを実 G1 の限られた DOF でどれだけ追従できるか」を表す正直な指標
（例: dance ~0.07m、backflip ~0.16m = G1 では追従困難）。

## 今後

contact-preserving IK 拡張、torque-aware 最適化、アクチュエータ空間と sim_certificate の連携、
実機 bridge（unitree_sdk2）。バランス制御は参照 IK ではなく RL policy（Phase 3）が担う。
