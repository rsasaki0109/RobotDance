# robotdance_unitree

G1/H1 configs, URDF mapping, SDK2/ROS2 bridge assumptions — Unitree を primary target とする embodiment 統合。

## 実装状況

- `g1.py` / `h1.py` — **Unitree G1（~1.29m）/ H1（full-size）の kinematic morphology**。
  各 `MORPHOLOGY`（[`RobotMorphology`](../robotdance_retarget/embodiment.py)）が rest pose から
  bone 長・全高を導き、[RD-Embodiment](../specs/rd-embodiment/) v0 schema 適合の dict を返す。
  **G1 の rest pose は v0.26 で公式 g1_23dof URDF の実寸由来に更新**（旧手書きプロキシは nominal 1.12m・
  bone 平均相対誤差 ~26% で乖離 → 実 URDF 一致の nominal 1.291m・誤差 ~0%。`tests/test_real_g1_urdf.py`
  が実 URDF 在環境で回帰検証）。関節オフセット＝寸法の事実のみ採用し mesh/URDF 本体は同梱しない。
- registry — `from robotdance_unitree import get_morphology, EMBODIMENTS`。新機種は registry に追加するだけ。
- `configs/*.rdembodiment.json` — エクスポート済み embodiment（schema 検証済み）。
- `urdf_import.py` — **実 URDF → 実寸 RobotMorphology**。zero-config FK でリンク世界位置を求め
  canonical 19-joint rest を**実物寸法**から構築（手作りプロポーションを脱却）。

```bash
# 利用者が g1_description などを取得し:
robotdance import-urdf g1_23dof.urdf --name unitree_g1 --save configs/g1_real.rdembodiment.json
```

```python
from robotdance_unitree.urdf_import import urdf_to_morphology
morph = urdf_to_morphology("g1_23dof.urdf")   # nominal_height ≈ 1.29 m（実 G1 23dof）
```

実 G1 URDF からは脚・腕の bone 長が実物になる（例: 前腕+手 0.10m など、手作りの過大値を修正）。

```python
from robotdance_unitree import get_morphology
from robotdance_retarget.kinematic import retarget
from robotdance_core.synthetic import generate_dance

motion = retarget(generate_dance(), get_morphology("unitree_h1"))
```

> ⚠️ **v0 注意:** 既定の `get_morphology` は canonical 19-joint 構造の retarget 用プロキシ
> （`g1.py`/`h1.py` は手作り体格）。**`import-urdf` を使えば寸法は実 URDF 由来になる**（脚・腕は実物、
> torso 連鎖・toe は合成、質量は近似）。ただし **アクチュエータ空間 retarget（実 G1 関節角への IK）・
> SDK2/ROS2 joint 写像・実機慣性での sim は今後**。URDF / mesh は repo に含めない（利用者が取得）。
