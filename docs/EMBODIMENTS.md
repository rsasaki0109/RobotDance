# 対応ロボットとデータ provenance

RobotDance の各 embodiment（morphology）は、**公開 URDF の実測値**を canonical 19-joint スケルトンへ
写像したものです。本プロジェクトの誠実さの中核として、**数値定数（rest pose / joint limit / 質量 /
慣性テンソル）のみ**を抽出し、**mesh / URDF 本体は再配布物に同梱しません**（license-safe）。出典・
ライセンス・抽出方法・実データのカバレッジを以下に明示します。

## ロボット一覧

| robot | 出典 URDF | ライセンス | 総質量 | nominal 身長 | runtime adapter | PD 既定 (kp/kd) |
| --- | --- | --- | ---: | ---: | --- | --- |
| `unitree_g1` | unitree_ros `g1_description/g1_23dof.urdf` | BSD 3-Clause | 34.13 kg | 1.291 m | unitree_sdk2 | 150 / 6 |
| `unitree_h1` | unitree_ros `h1_description/urdf/h1.urdf` | BSD 3-Clause | 59.34 kg | 1.664 m | unitree_sdk2 | 200 / 10 |
| `unitree_h2` | unitree_ros `h2_description/H2.urdf` | BSD 3-Clause | 75.59 kg | 1.758 m | unitree_sdk2 | 200 / 10 |
| `booster_t1` | BoosterRobotics `booster_gym resources/T1/T1_serial.urdf` | Apache-2.0 | 31.614 kg | 0.978 m | booster_sdk | 300 / 6 |
| `apptronik_apollo` | mujoco_menagerie `apptronik_apollo`（Apollo 由来） | Apache-2.0 | 80.898 kg | 1.618 m | mujoco | 400 / 12 |

いずれも permissive license（BSD-3 / Apache-2.0）で、数値定数の抽出＋attribution は再配布可。

> **license-safe の判断例**: Fourier GR-1 / N1（FFTAI Wiki-GRx-Models）は **GPL-3.0（copyleft）**のため、
> Apache-2.0 の本プロジェクトには取り込まない（permissive な代替として Apollo を採用）。

## 実 URDF データのカバレッジ（7 軸）

各 morphology が実 URDF 値を持つ feasibility 軸。✅=実 URDF 値、〜=virtual/placeholder。

| 軸 | データ | G1 | H1 | T1 | Apollo |
| --- | --- | :-: | :-: | :-: | :-: |
| 寸法 | rest pose / bone 長 | ✅ | ✅ | ✅ | ✅ |
| 位置 ROM | per-joint position limit | ✅ (13) | ✅ (11) | ✅ (13) | ✅ (15) |
| 速度 | per-joint velocity limit | ✅ (13) | ✅ (11) | ✅ (13) | 〜 |
| トルク | per-joint effort limit | ✅ (13) | ✅ (11) | ✅ (13) | ✅ (15) |
| 質量分布 | per-link mass → bone | ✅ | ✅ | ✅ | ✅ |
| 慣性 | per-bone inertia tensor | ✅ (14) | ✅ (14) | ✅ (15) | ✅ (17) |
| balance | 実フットプリント / COM | ✅ | ✅ | ✅ | ✅ |

括弧内は実値を持つ canonical 関節 / bone の数（残りは合成 toe 等の placeholder/floor）。Apollo の速度（〜）は
menagerie MJCF に actuator velocity 情報が無いため未収載（generic fallback）= follow-up。

## 抽出方法（再現可能）

実 URDF から canonical 19-joint morphology への写像手順（T1 で確立、`robotdance_unitree/booster_t1.py`）:

1. **生 URDF を直接取得**（`curl`）し **Python で厳密パース**（XML）。要約ツールは数値転記が不正確に
   なるため使わない。
2. **rest pose**: 各リンクの累積 frame 原点を計算し canonical 関節へ写像（z-up/x-forward/y-left,
   足先を接地 z≈0.03 へ平行移動）。torso DOF が少ない機種は spine 等を区間中点の virtual 関節に。
3. **per-joint limit**: 1 canonical ball joint に複数 URDF DOF が対応 → envelope 集約（位置=最広,
   速度/トルク=min）。
4. **質量 / 慣性**: 各リンクを**世界 COM 最近傍の canonical bone（区間中点）**へ割当て、剛体合成
   （平行軸定理）。慣性は COM まわり世界軸テンソル、COM は親 joint 相対で格納。
   ⚠️ リンク名（例: T1 "Shank"=下腿）と canonical 関節名（"knee"）は一致しないので、**名前でなく
   世界 COM 位置で割当てる**。
5. **検証**: 生成 MJCF の `body_inertia`（principal）が埋め込みテンソルの固有値に一致・総質量保存・
   PD 追従が安定、をテストで担保（`tests/test_sim.py`, `tests/test_multi_embodiment.py`）。

## Attribution

- **Unitree G1 / H1 / H2**: robot descriptions © HangZhou YuShu TECHNOLOGY CO.,LTD. ("Unitree Robotics"),
  BSD 3-Clause（[unitreerobotics/unitree_ros](https://github.com/unitreerobotics/unitree_ros)）。
  H2 は v0.112 で h2_description/H2.urdf 由来の実寸/慣性から追加（nominal 1.76m / 75.6kg）。
- **Booster T1**: robot description © Booster Robotics, Apache-2.0
  （[BoosterRobotics/booster_gym](https://github.com/BoosterRobotics)）。
- **Apptronik Apollo**: MuJoCo Menagerie model（Apollo 由来）, Apache-2.0
  （[google-deepmind/mujoco_menagerie](https://github.com/google-deepmind/mujoco_menagerie)）。

RobotDance は上記から**数値定数のみ**を派生利用し、mesh/URDF 本体は同梱しない。各上流の著作権表示・
ライセンス条項を尊重する。
