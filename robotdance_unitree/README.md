# robotdance_unitree

G1/H1 configs, URDF mapping, SDK2/ROS2 bridge assumptions — Unitree を primary target とする embodiment 統合。

## 実装状況

- `g1.py` — **Unitree G1 の v0 簡略 kinematic プロキシ**。`embodiment_dict()` が
  [RD-Embodiment](../specs/rd-embodiment/) v0 schema 適合の dict を返す。
- `configs/unitree_g1.rdembodiment.json` — エクスポート済み embodiment（schema 検証済み）。

> ⚠️ **v0 注意:** これは実機 URDF / アクチュエータ写像ではない。canonical 19-joint 構造を流用し、
> G1 に近い体格（身長 ~1.27m, 短い四肢）を与えた retarget 用プロキシ。
> 実 URDF（`g1_description`）・SDK2/ROS2 joint 写像・正確な joint limits・H1 対応は Phase 2。
