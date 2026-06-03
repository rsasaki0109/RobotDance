# Contributing to RobotDance

> 🚧 Pre-v0.1。仕様と雛形を整備中です。大きな実装に入る前に Issue で方針合意を推奨します。

## 基本原則

1. **仕様は実装より偉い。** schema を変える PR は [`specs/`](specs/) と該当 README を必ず更新する。
2. **ライセンス安全性は最優先。** raw video を repo に入れない。source license が `unknown` の派生 motion を公開しない。
3. **skeleton-first。** SMPL/SMPL-X を core 必須依存にしない（optional plugin）。
4. **sim-first / safety-first。** 実機 path は安全 gate の後ろにのみ置く。

## 開発環境

```bash
pip install -e ".[dev]"     # core + demo(scipy/matplotlib/imageio) + sim(mujoco) + perception(mediapipe)
ruff check .
pytest                       # CI と同じ。ROS2(rclpy) が無い環境では runtime ノードのテストは自動 skip
```

extras: `demo`（合成/可視化）, `sim`（MuJoCo 物理検証）, `perception`（MediaPipe）。
ROS2(`rclpy`) は pip ではなく ROS2 Jazzy のインストールから入る（[`robotdance_ros2`](robotdance_ros2/)）。

PR は **ruff clean + pytest green** が必須（GitHub Actions が自動チェック）。

## CLI の動作確認

```bash
robotdance validate mir examples/minimal_mir.json   # spec 検証
robotdance demo-multi -o many.gif                   # same motion, many humanoids
robotdance demo-safety -o safety.gif                # unsafe motion rejected
robotdance demo-motion-map -o map.png               # Motion Map
robotdance demo-runtime                             # safety guard の遮断デモ
```

## 拡張ポイント（plugin）

新しい対応を追加しやすい registry / adapter 構造になっています:

| 追加したいもの | 方法 |
| --- | --- |
| 新しいロボット | [`robotdance_unitree`](robotdance_unitree/) に rest pose を定義し `EMBODIMENTS` registry に 1 行（`RobotMorphology`） |
| 新しい dataset | [`robotdance_data`](robotdance_data/) に `load_*` adapter を書き `dataset.ADAPTERS` に登録 |
| 新しい pose backend | [`robotdance_perception`](robotdance_perception/) に landmark → canonical マッピングを追加 |
| 新しい sim backend | [`robotdance_sim`](robotdance_sim/) に `sim_certificate` を返す backend を追加 |

いずれも **canonical 19-joint skeleton**（[`robotdance_core/skeleton.py`](robotdance_core/skeleton.py)）を介して接続します。
schema を変える場合は [`specs/`](specs/) と該当 README を必ず更新してください（仕様は実装より偉い）。

## ライセンス

- Code: Apache-2.0。コントリビューションは同ライセンス下で提供したものとみなされます。
- データセット・モデル weights の利用許諾は source ごとに別管理。Data Bill of Materials を伴わない
  データ contribution は受け付けません。
