# robotdance_retarget

IK, morphology mapping, contact-preserving retargeting — 人間 motion（[RD-MIR](../specs/rd-mir/)）→ robot embodiment（[RD-Embodiment](../specs/rd-embodiment/)）への retarget。

## 実装状況

- `kinematic.py` — **v0 kinematic retarget**: direction-preserving FK + morphology normalization + ground clamp。
  `retarget_to_g1(mir) -> RdMotion` で [RD-Motion](../specs/rd-motion/) を生成する。
  物理 sim は通さない（運動学プレビュー）。`retarget_metrics.joint_flexion` で膝・肘の屈曲が実機可動域
  上限を超えていないか測る（per_joint_limits を持つ embodiment のみ）。

```python
from robotdance_core.synthetic import generate_dance
from robotdance_retarget.kinematic import retarget_to_g1

motion = retarget_to_g1(generate_dance())
print(motion.retarget_metrics)  # height_scale, bone_direction_cosine, foot_sliding, joint_flexion, ...
```

  **屈曲補正**（検出→治療）: `retarget(mir, morph, clamp_flexion=True)` で可動域超過フレームの遠位
  サブチェーンを hinge 中心に剛体回転させ屈曲角を上限へ収める（bone 長保存）。CLI は
  `robotdance retarget --clamp-flexion`。可動域順守と引き換えに bone 方向忠実度がわずかに下がる
  （`retarget_metrics.joint_flexion.clamp` に補正量を記録）。

- `actuator_ik.py` — **アクチュエータ空間 retarget**（torch, `[learn]` extra）。実 URDF の
  **微分可能 FK** を構成し、勾配 IK で **実 G1 の 23 関節角**を解く。出力 `.rdmotion` の
  `joint_rotations` に実機が command できる joint 角を格納。

```bash
robotdance retarget-ik dance.rdmir.json --urdf g1_23dof.urdf -o g1_joints.rdmotion.json
```

IK 位置誤差は「人間の動きを実 G1 の限られた DOF でどれだけ追従できるか」を表す正直な指標
（例: dance ~0.07m、backflip ~0.16m = G1 では追従困難）。

- `gmr_backend.py` — **GMR 外部 retarget**（v0.153）。MIT [GMR](https://github.com/YanjieZe/GMR) の
  mink IK を `retarget --backend gmr` で呼ぶ。GMR repo を clone し `pip install -e GMR/` すること
  （PyPI wheel のみでは robot XML assets が無い）。対応: unitree_g1/h1/h2, booster_t1, fourier_n1。

```bash
robotdance list-retargeters
robotdance retarget dance.rdmir.json --backend gmr --robot unitree_g1 -o g1_gmr.rdmotion.json
```

## 今後

builtin vs GMR の fight benchmark 列、contact-preserving IK 拡張、torque-aware 最適化、
実機 bridge（unitree_sdk2）。バランス制御は参照 IK ではなく RL policy（Phase 3）が担う。
