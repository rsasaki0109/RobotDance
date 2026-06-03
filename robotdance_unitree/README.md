# robotdance_unitree

G1/H1 configs, URDF mapping, SDK2/ROS2 bridge assumptions — Unitree を primary target とする embodiment 統合。

## 実装状況

- `g1.py` / `h1.py` — **Unitree G1（小型 ~1.27m）/ H1（full-size ~1.8m）の v0 簡略 kinematic プロキシ**。
  各 `MORPHOLOGY`（[`RobotMorphology`](../robotdance_retarget/embodiment.py)）が rest pose から
  bone 長・全高を導き、[RD-Embodiment](../specs/rd-embodiment/) v0 schema 適合の dict を返す。
- registry — `from robotdance_unitree import get_morphology, EMBODIMENTS`。新機種は registry に追加するだけ。
- `configs/*.rdembodiment.json` — エクスポート済み embodiment（schema 検証済み）。

```python
from robotdance_unitree import get_morphology
from robotdance_retarget.kinematic import retarget
from robotdance_core.synthetic import generate_dance

motion = retarget(generate_dance(), get_morphology("unitree_h1"))
```

> ⚠️ **v0 注意:** これは実機 URDF / アクチュエータ写像ではない。canonical 19-joint 構造を流用し、
> 各機種に近い体格を与えた retarget 用プロキシ。
> 実 URDF（`g1_description` / `h1_description`）・SDK2/ROS2 joint 写像・正確な joint limits は Phase 2。
