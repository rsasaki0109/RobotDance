# Changelog

All notable changes to RobotDance are documented here.
This project adheres to [Semantic Versioning](https://semver.org/).

## [Unreleased]

## [0.105.1] - 2026-06-07

### Fixed

- `test_cli_search_motion_with_learned_encoder` が torch 未導入の CI で失敗していた問題を修正
  （`pytest.importorskip("torch")` を追加。torch は optional dep なので他の encoder 系テストと同様に skip）。

## [0.105.0] - 2026-06-07

### Added

- `search-motion --encoder <checkpoint>`: 手作り特徴の代わりに**学習済み motion encoder**
  （`train-encoder` の masked 再構成 checkpoint, `LearnedMotionEncoder`）で索引・検索する。
  `MotionIndex(embed_fn=...)` を差し替えるだけで quality-aware フィルタ（`--healthy-only`）とも併用可。
  出力に encoder 種別（handcrafted / learned(...)）を表示。`tests/test_embeddings.py` に 1 テスト。計 335。

## [0.104.0] - 2026-06-07

ROS2 runtime のコア MotionServer に pause / seek 制御を実装（ROADMAP Phase 5）。

### Fixed

- `MotionServer.stream()` が docstring で謳う **pause を実装していなかった**（`self.paused` を未参照で、
  常に最後まで再生）バグを修正。pause 中は cursor を進めず最後のフレームを保持し続ける。

### Added

- `MotionServer` に `pause()` / `resume()` / `seek_frame(i)` / `seek_phase(p)` を追加。stream の yield 間に
  呼んで対話的に再生位置を操作できる（scrubbing）。seek は自然前進の +1 と区別（`_seeked` フラグ）。
- `tests/test_runtime.py` に pause 保持・resume 前進・seek_frame クランプ・seek_phase の 3 テスト。計 334。

## [0.103.0] - 2026-06-07

model↔schema 同期チェックを全 spec に拡大（Stable Specs）。

### Added

- `tests/test_specs.py` の model↔schema 同期テストを **RD-MIR / RD-Motion / RD-Policy** に parametrize で
  一般化（全て properties 完全一致＝ドリフト無しを確認）。さらに **RobotMorphology.to_rd_embodiment()** が
  rd-embodiment schema に適合し、出力キーが schema properties の部分集合であることを検証。
  RD-MIR 以外もドリフトすると CI が落ちる。RD-Manifest は schema 直接利用（pydantic モデル無し）。

## [0.102.0] - 2026-06-07

RD-MIR の pydantic モデルと JSON schema のドリフトを修正（Stable Specs）。

### Fixed

- **schema↔model ドリフト**: `rd-mir.schema.json` は `joint_rotations` / `smpl_params` を許可
  （additionalProperties:false）するが、`RdMir` モデル（extra="forbid"）に両フィールドが無く、
  **schema 適合の RD-MIR がモデルで load 失敗していた**。両 optional フィールドをモデルに追加し解消。

### Added

- `tests/test_specs.py` に **RdMir モデル ↔ rd-mir schema の properties 完全一致**テストを追加。
  以後フィールドがどちらか一方だけに増えると CI が落ち、Stable Specs のドリフトを防ぐ。

## [0.101.0] - 2026-06-07

spec の machine-readable バージョン整備と一覧 CLI（v1.0/Stable Specs に向けて）。

### Added

- 5 つの spec schema（rd-mir / rd-manifest / rd-embodiment / rd-motion / rd-policy）に
  トップレベル `"version": "0"` を追加（`$id` の `/v0/` と整合）。
- CLI `specs`: 各 spec の title / version / properties 数 / required 数 / `$id` を一覧表示。
- `tests/test_version_consistency.py` に「各 spec が $schema/$id/title/version を宣言し version が
  $id の vN と一致する」テストを追加。README 機能表に `specs` / `validate`（spec 行）を追記。

## [0.100.0] - 2026-06-07

リリースメタデータの整合性チェックを追加（v1.0/Stable Specs に向けた release hygiene）。

### Added

- `tests/test_version_consistency.py`: pyproject / CITATION / CHANGELOG の **version 一致**、
  **CITATION date-released と CHANGELOG 最新エントリの日付一致**、5 つの spec schema（rd-mir /
  rd-manifest / rd-embodiment / rd-motion / rd-policy）の存在と JSON Schema 妥当性を CI で検証。

### Fixed

- CITATION.cff の `date-released` が 2026-06-05 のまま古かった（0.99.0 は 06-07）ドリフトを修正。
  以後は上記テストがドリフトを検出する。

## [0.99.0] - 2026-06-07

quality-aware retrieval（ROADMAP Phase 3）— motion 検索を motion-doctor の健全性と接続。

### Added

- `MotionIndex` にエントリ単位のメタデータと述語フィルタを追加: `add(..., meta=)` / `add_mir(..., meta=,
  diagnose=)` / `meta_of(id)` / `query(..., where=lambda meta: ...)`。`diagnose=True` で health（ok/warn）と
  warns をメタに格納し、`where` で **quality-aware / action-label 絞り込み**ができる。後方互換（meta 任意）。
- CLI `search-motion <query.rdmir> <corpus_dir> [-k] [--healthy-only]`: query に似た motion を corpus から
  検索。`--healthy-only` は motion-doctor で warn の無い motion のみ返す。`tests/test_embeddings.py` に 3 テスト。

## [0.98.0] - 2026-06-07

### Added

- `build-dataset` に **motion-doctor 健全性 QC を統合**。export 済み（dedupe 後に残った）RD-MIR を
  診断し、`build_report.json` の `report["health"]`（checked/healthy/warn/skipped/warn_breakdown）と
  各 BOM 行の `health`、`DATA_CARD.md` の "Health (motion-doctor)" 節に集計する。CLI は health 一行サマリを表示。
- `build-dataset --no-qc` で QC をスキップ可能（既定 on）。`build_dataset(..., qc=True)` /
  `build_from_file(..., qc=True)` 引数。`tests/test_data.py` に QC テスト 2 件。計 320。

## [0.97.0] - 2026-06-06

### Added

- CLI `motion-doctor <dir>`: ディレクトリを渡すと配下の RD-MIR(.json) を**一括診断**し、per-file の
  warn 種別と「healthy/warn/error」集計・warn 内訳（mirror×N 等）を出す。warn/error が 1 件でも exit 1。
  データセット curation の QC に有用（schema/manifest.json は除外、壊れたファイルは error として継続）。
- `robotdance_motion.doctor.warn_names(checks)` ヘルパ。`tests/test_doctor.py` に corpus テスト 3 件。

## [0.96.0] - 2026-06-06

接地クリーンアップに foot-skate（接地足の水平滑り）除去を追加（pre-alpha, ROADMAP の contact-preserving 一歩）。

### Added

- `ground_contact_cleanup(..., lock_horizontal=False)`: opt-in で接地足の水平滑りを除去。各フレームで
  支持足（接地中・最下, ヒステリシス付き）を選び、その補正後 xy が一定になるよう全関節 xy を平行移動
  （切替時は飛び無く再アンカー）。foot_skate_before/after_m を quality_metrics に記録。
- CLI `validate-sim --lock-foot-xy`（`--ground-clean` と併用）。`tests/test_grounding.py` に 2 テスト。

### Notes

- **正直な所見**: 実 squat で foot-skate は 0.0158→0.0059 m/frame（約 63% 減）まで縮むが、**balance は
  0.82 のまま REJECT**。深度（前後 x）誤差由来の balance は水平 lock では直らず、深度復元が別軸の課題
  であることを再確認。既定挙動・README certificate 数値は不変。

## [0.95.1] - 2026-06-06

### Changed

- README（en/ja）の機能表に最近のコマンドを追記し発見性を改善: `list-backends` / `pose-compare` /
  `motion-doctor`（新行 "pose backends & QC"）、`list-retargeters`（retarget 行）、`extract --backend`。

## [0.95.0] - 2026-06-06

retarget バックエンドをレジストリ化し、GMR を一級の外部バックエンドとして登録（pre-alpha）。

### Added

- `robotdance_retarget/backends.py`: retarget バックエンドのレジストリ（pose レジストリと同作法）。
  builtin **kinematic**（速い直接マップ, CLI `retarget`）/ **actuator-ik**（実 URDF へ IK, 実機向け既定,
  CLI `retarget-ik`）と、外部 OSS **gmr**（General Motion Retargeting, MIT, 18 機種, CPU 実時間）を
  能力メタデータ（method / real_urdf / modules / cli / url）付きで登録。`available()` は遅延 spec チェック。
- CLI `list-retargeters`: 上記を手法・実 URDF 要否・導入状況付きで一覧。
- `tests/test_retarget_backends.py` 6 テスト。RELATED_WORK.md の GMR ギャップを「登録済み・残りは実行配線」に更新。

## [0.94.1] - 2026-06-06

### Added

- v0.94 で登録した `gvhmr` backend の文書化ワークフローを **CLI で end-to-end smoke test 化**:
  GVHMR 出力を模した `.npy` → `import-hmr` → RD-MIR → `motion-doctor` を `tests/test_hmr.py` で検証。
  `extract --backend gvhmr` が import-hmr へ誘導し exit 2 を返すことも確認。
- docs/POSE_BACKENDS.md に world-grounded ワークフローの具体コマンド例を追記。

## [0.94.0] - 2026-06-06

世界座標（world-grounded）抽出をレジストリの一級バックエンドに（pre-alpha）。

### Added

- pose レジストリに **`gvhmr` / `wham`**（quality_tier="world-grounded", output_dim=3,
  retarget_capable, extract_mode="import"）を登録。深度・グローバル軌跡が入り単眼の深度律速を緩和する
  本命経路。`PoseBackend` に `extract_mode`（"video"/"import"）と `via` フィールドを追加。
- `extract --backend gvhmr`（import 系）は、外部ツールで SMPL を出力し `import-hmr` で取り込む
  ワークフローへ誘導する（RobotDance 内では推論しない＝重い weights/repo を依存に持ち込まない）。
- `list-backends` に mode（video/import）列を追加。`tests/test_pose_backends.py` に world-grounded
  backend のテスト 3 件（登録/extract 誘導/2D ランナー拒否）。

### Changed

- `compare_backends` と関連テストの「比較対象」フィルタを **video-mode の 2D 検出器のみ**に明確化
  （import 系は 2D ランナーを持たないため除外）。docs/POSE_BACKENDS.md・RELATED_WORK.md に反映。

## [0.93.0] - 2026-06-06

### Added

- `docs/RELATED_WORK.md`: *人間動画→ヒューマノイド* 周辺の研究/OSS マップ（抽出: GVHMR/WHAM/TRAM、
  retarget: GMR、制御/teleop: H2O/OmniH2O/ExBody/TWIST 等、物理模倣: PHC、データセット curation:
  PHUMA/OpenT2M）と、RobotDance の差別化（透明な解析的 feasibility certificate / license-safe な
  provenance / コンパイラ統合 / 単眼限界への正直さ）・取り込むべきギャップを整理。README からリンク。

## [0.92.0] - 2026-06-06

### Added

- CLI `extract` が抽出直後に `motion-doctor` の健全性チェックを自動実行し、warn を簡潔に表示
  （mirror/深度/接地/多人数 等）。`--no-check` でスキップ可能。手動の多人数警告は doctor に統合。

## [0.91.0] - 2026-06-06

単眼抽出のデバッグ知見を再利用可能な健全性チェックに製品化（pre-alpha）。

### Added

- `robotdance_motion/doctor.py`: `diagnose_motion(mir)` が RD-MIR のよくある破綻を診断する純関数群。
  **mirror**（左右反転＝hip 幅 y の符号, 背面撮影で発生）、**depth_collapse**（前後 x の分散; lift では
  info）、**grounding**（足最下点の振れ＝foot skate）、**multi_subject**（n_subjects_max>1）、
  **low_confidence / jitter** を ok/warn/info と対処ヒント付きで返す。
- CLI `motion-doctor <rdmir>`: 上記を表示し、warn が 1 つでもあれば exit 1。README quickstart に追記。
- `tests/test_doctor.py`（healthy/mirror/depth(lift 区別)/foot skate/低 conf・多人数/総合の 6 テスト）。

これは v0.89.1（overlay ズレ＝多人数取り違え）・v0.90.1（squat ミラー）で手作業で当てた診断を、
誰でも `motion-doctor` 一発で得られるようにしたもの。

## [0.90.1] - 2026-06-05

### Fixed

- "More clips" の squat → G1 が不自然（脚が交差・腕が頭上）だった問題を、**より単眼向きの
  squat クリップに差し替え**て改善。旧クリップは**背面のバーベル squat**で、単眼が左右を取り違え
  （hip 幅 y が負＝ミラー）腕もバーベルで頭上に張り付いていた。新クリップは**正面のゴブレット
  squat**（Taco Fleur, CC BY-SA 4.0）で、ミラー解消（hip 幅 y +0.06）・脚が左右対称・腕も自然な前方に。
- 出典表記を更新（squat の clips デモ = Taco Fleur / 検出器・物理デモ = FitnessScape の 2 ソースを明記）。
  検出器比較（pose_compare）と物理 certificate は、深度律速 REJECT のストーリーが綺麗な旧 rear-view
  squat のまま据え置き（新クリップは腕スイングで torque 律速になり物理デモの主旨が薄れるため）。

## [0.90.0] - 2026-06-05

多人数シーンの被写体取り違えを抽出側で根治（pre-alpha）。

### Added

- `extract_motion(..., num_poses=4)`: MediaPipe を複数人検出に切り替え、**最大人物を起点に
  前フレーム被写体を最近接で追跡**する（`_select_subject`）。多人数シーンで背景の人へ乗り移らず
  前景の主被写体に固定。検出最大人数を `quality_metrics["n_subjects_max"]` に記録、>1 で警告。
- CLI `extract --num-poses N`。出力に `subjects=` を表示し、複数人検出時は crop を促す注意を出す。
- `tests/test_perception.py` に `_select_subject` の 3 テスト（初フレーム最大選択・追跡・空入力）。

### Fixed

- v0.89.1 で crop して回避していた多人数 overlay ズレを、**crop なしでも**前景被写体に追従するよう
  抽出側で解消（実 karate 引き shot で 2D overlay が演武者に乗ることを確認）。

## [0.89.1] - 2026-06-05

### Fixed

- real-video hero の ① source video + skeleton overlay がズレていた問題を修正。元の karate クリップが
  **多人数の道場引き shot**で、MediaPipe（num_poses=1）が前景の演武者でなく小さい/別の人物を掴み、
  2D overlay が被写体外に固まっていた（3D skeleton だけは「人の形」に見えるため見落とされていた）。
  演武者領域に**クロップした版から ①②③ を一括再生成**（mean_conf 0.918→0.923、overlay が演武者に追従）。
  overlay GIF も 64 色最適化で 2.3MB→1.1MB に圧縮。出典・ライセンス（Sdcsabac, CC BY-SA 4.0）は不変。

## [0.89.0] - 2026-06-05

README の pose セクションを整理し、詳細を専用 docs ページへ移動（pre-alpha）。

### Added

- `docs/POSE_BACKENDS.md`: pose 検出バックエンドの詳細ページ。CLI の使い方、3 検出器の比較、
  `*+lift` coarse baseline の説明・kata 定量比較・robot 横並びデモ（GIF 3 本＋指標表）を集約。

### Changed

- README（en/ja）の "Pose detection" 節を要約＋比較表＋`docs/POSE_BACKENDS.md` へのリンクに圧縮
  （GIF 3 本＋表 2 つで肥大化していた pose 詳細を docs へ退避し、hero の流れを読みやすく）。
  lift_vs_native_*.gif は docs から参照され引き続き利用。

## [0.88.0] - 2026-06-05

planar lift を実ロボットまで通し、native と横並びで実証（pre-alpha）。

### Added

- `scripts/render_lift_vs_native_robot.py`: 同一動画から native(MediaPipe 3D) と lift(2D→planar) を
  両方 actuator-IK で実 Unitree G1 へ retarget し、実メッシュを横並びレンダリング（native | lift）。
  ラベル付き GIF を出力。`_render_mesh`（render_real_video_gif）を再利用。
- README に robot 横並び GIF を追加。**2D 検出器だけ（native 3D なし）でも G1 上で識別可能な型**になる
  ことを実証。retarget IK 誤差 lift=0.097m vs native=0.071m（約 38% 悪い）と正直に併記。

## [0.87.0] - 2026-06-05

planar lift を実 kata クリップで native 3D と定量比較し、スケールを堅牢化（pre-alpha）。

### Added

- `scripts/compare_lift_vs_native.py`: 同一動画に native(MediaPipe 3D) と lift(2D→planar) を当て、
  深度 x の std・MPJPE（full / frontal y-z）を算出し、native|lift の canonical skeleton 横並び GIF を出力。
- README に kata クリップでの定量比較（GIF + 指標表）を追加。native 深度 std 0.175m に対し
  lift は 0.000m（平面）、MPJPE full 0.274m / frontal 0.222m。差の約 0.16m が深度由来であることを明示。

### Changed

- **lift のスケール基準を hip 幅 → 胴体長（pelvis→chest）に変更**。hip 幅は被写体が横を向くと画像上で
  0 に潰れ per-frame スケールが発散していた（z レンジ 34m）。縦方向の胴体長はヨー回転に強く、
  実 kata で z レンジが native 並み（1.86m vs 1.74m）に収まる。`DEFAULT_TORSO_M=0.50`。
- lift の左右（y）符号を native MediaPipe と同手系に統一（画像右→ -y）。実データで frontal MPJPE が
  最小になる向きを採用（0.222 vs 反転 0.292）。`lift_coco17_to_canonical` の引数 `hip_width_m`→`torso_m`。

## [0.86.0] - 2026-06-05

2D 検出器を 3D 化する解析的 planar lift backend を追加（pre-alpha・coarse baseline）。

### Added

- `robotdance_perception/lifting.py`: 2D COCO-17 → canonical 19-joint 3D の解析的 planar lift。
  `lift_coco17_to_canonical(xy[17,2], conf[17])` は**深度を復元せず**正面平面（x=0）へ埋め込み、
  hip 幅の人体寸法プライアでメートル化、足を z=0 接地する。`extract_via_lift(video, detector=...)` で
  2D 検出器 + lift から RD-MIR を生成（quality_metrics に `lift="planar-no-depth"` を記録）。
- レジストリに lift 派生 backend `yolo11-pose+lift` / `rtmpose+lift`（output_dim=3,
  retarget_capable=True, quality_tier="coarse-planar"）を登録。`extract --backend <name>+lift` で利用。
- `tests/test_lifting.py`（planar/接地/metric スケール/左右/不正形状/0 割の 7 テスト）と
  `tests/test_pose_backends.py` の lift 関連テスト。計 293 → 全 green。

### Changed

- `PoseBackend` に `quality_tier`（native / coarse-planar）と `lift_from` フィールドを追加。
- CLI `extract` が lift 派生 backend を `extract_via_lift` に振り分け、`list-backends` に tier 列を追加。
- README に `*+lift` backend の位置づけ（**深度なし・冠状面向けの coarse baseline**）を明記。

## [0.85.0] - 2026-06-05

pose 検出器の比較を CLI の正規コマンドに昇格（pre-alpha）。

### Added

- CLI `pose-compare <clip> [-o out.gif] [--stride] [--width]`: available な全 pose backend を
  同一動画で比較し、overlay GIF（任意）と検出率・平均 confidence・推論時間の指標表を出す。
- `robotdance_perception/compare.py`: 比較ロジックを `compare_backends(video, out_gif=...)` に集約
  （CLI と `scripts/compare_pose_backends.py` の共通実装）。未導入の検出器は自動スキップし `skipped` で報告。
- `tests/test_pose_backends.py` に 3 テスト追加（list-backends CLI 実行・全 backend のパネル色・
  動画不在時のエラー）。計 14 テスト。

### Changed

- `scripts/compare_pose_backends.py` を `compare_backends` を呼ぶ薄いラッパに簡素化。
- README に `pose-compare` の使い方を追記。

## [0.84.0] - 2026-06-05

pose 検出バックエンドのレジストリを「メタデータ」から「機能的」へ（pre-alpha）。

### Added

- `robotdance_perception.backends` に共通 COCO-17 表現（`COCO_EDGES` / `MP33_TO_COCO`）と
  2D ランナー生成器 `make_runner_2d(name)` を追加。全バックエンドを統一シグネチャ
  `run(frame_bgr, idx, fps) -> (xy[17,2], conf[17]) | None` で呼べる。heavy 依存は生成時のみ遅延 import。
- `tests/test_pose_backends.py` に 4 テスト追加（COCO 定数の妥当性・全 backend にランナー紐付け・
  未知名エラー・MediaPipe ランナーの COCO-17 出力形状）。計 11 テスト。

### Changed

- `scripts/compare_pose_backends.py` を全面リファクタ。3 検出器のランナー・COCO エッジ・人物選択を
  レジストリ（`make_runner_2d` / `COCO_EDGES`）から取得する単一情報源方式に統一。未導入の検出器は
  自動スキップし、飛ばした検出器を明示表示する（silent cap を回避）。

## [0.83.0] - 2026-06-05

pose 検出バックエンドを能力付きレジストリで抽象化（pre-alpha）。

### Added

- `robotdance_perception/backends.py`: pose 検出バックエンドのレジストリ。各バックエンドの
  能力メタデータ（出力次元 2D/3D・keypoint 形式・retarget 可否・必要モジュール・dev 印）を 1 か所に束ねる。
  heavy 依存（mediapipe/ultralytics/rtmlib）は読み込み時に import せず、`available()` が遅延 spec チェックで可否判定。
- CLI `list-backends`: 登録済みバックエンドと能力（次元/形式/retarget 可否/導入状況）を一覧表示。
- CLI `extract --backend <name>`: 抽出バックエンドを選択。2D-only 検出器（yolo11-pose/rtmpose）は
  3D world landmarks を返さないため、`resolve_extract_backend` がフル抽出で明示的に拒否する。
- `tests/test_pose_backends.py`: レジストリ一覧/未知名エラー/3D-2D 能力区別/抽出解決/遅延判定の 7 テスト。

### Changed

- `extract_motion(..., backend="mediapipe")` 引数を追加し、抽出前にバックエンド能力を検証。
- README に backend レジストリと `list-backends` / `extract --backend` の使い方を追記。

## [0.82.0] - 2026-06-05

非空手デモを残しつつ、複数 OSS pose 検出器の比較を追加（pre-alpha）。

### Added

- `scripts/compare_pose_backends.py`: 同一実動画に **MediaPipe / YOLO11-pose(Ultralytics) /
  RTMPose(rtmlib)** を当て、各骨格を共通 COCO-17 に揃えて 3 パネル overlay GIF と
  検出率・平均 confidence・推論時間を出力する比較スクリプト（dev 専用依存。パッケージ依存には含めない）。
- README に "Pose detection — swap in different OSS detectors" 節を追加（比較 GIF + 指標表）。
  MediaPipe のみ 3D world landmarks を返し robot retarget に使える点を明記（YOLO/RTMPose は 2D）。

### Changed

- README の "More clips" 行に squat→G1（動的接地版）を復帰させ、karate だけでなく
  squat・kathak の非空手デモも併記（多様性を維持）。

## [0.81.0] - 2026-06-05

実動画→ロボットの動きの一致を改善（pre-alpha）。「人間とロボットの動きが合っていない」を 2 点で修正。

### Fixed
- **動的接地**: ロボット render が `useFixedBase` で骨盤を固定高さに置いていたため、しゃがむ動作で
  体が沈まず足が浮いていた。`_render_mesh`（`render_real_video_gif.py`）に毎フレームの接地
  （最下点の AABB を床へ合わせて base 高さを下げる）を追加。屈伸・踏み込みが人間と一致するように。

### Changed
- 実動画 3 段ヒーローを **squat → karate kata** に差し替え。squat は単眼で最も深度が不確実な動きで
  retarget が脚を正しく復元できず人間と一致しなかった。karate は近接フレーム＋正面の型で overlay・
  スケルトン・ロボットが明確に対応（同一 extract から生成, IK 0.071m, conf 0.92）。
- 「実クリップ」行を kathak → G1 / H1（接地版）に整理。squat は物理検証セクションの例として継続。
- 旧 floating 版の実動画ロボット GIF を接地版に置換し、孤立アセットを削除（assets/readme/real: 約7M）。

## [0.80.0] - 2026-06-05

実動画 3 段デモの同期を修正（pre-alpha）。README の overlay が skeleton/robot と別トリム由来でズレていた
（「全然違う」）のを、同一 extract・同一 stride から作り直して 3 段（overlay→skeleton→robot）を同期。

### Fixed
- `assets/readme/real/squat_g1_overlay.gif`: skeleton/robot と**同じ squat_30 フレーム・同じ mir・stride 3**から
  再生成し、3 段が同一モーション・同一タイミングで対応するように修正。旧 `squat_overlay.gif`（別トリム）を削除。

### Added
- `scripts/render_real_video_gif.py --overlay`: 同一 extract から overlay GIF も出力し、3 段の同期を保証
  （以後のクリップは 1 コマンドで overlay/skeleton/robot を揃えられる）。

## [0.79.0] - 2026-06-05

README を英語化（pre-alpha, docs のみ）。OSS の裾野を広げるため英語を主 README に。

### Changed
- `README.md` を英語に翻訳（v0.77 の簡潔版がベース、構成・画像・コードブロック・ライセンス注記は不変）。
- 既存の日本語版を `README.ja.md` として保存。両 README 冒頭に言語スイッチャー（English · 日本語）を追加。

## [0.78.0] - 2026-06-05

未参照アセットの整理（pre-alpha, repo 軽量化）。v0.77 の README 簡潔化で表示されなくなった GIF/PNG を削除。

### Removed
- README から表示が外れ、かつ docs/各パッケージ README で画像埋め込みされていない 9 アセットを削除
  （`g1_dance` `many_humanoids` `many_humanoids_mesh` `smoothing` `safety_check` `pose_overlay_astronaut`
  `motion_map_learned` `real/karate_g1_skeleton` `real/kathak_g1_skeleton`）。assets/readme: 17M → 12M。
- いずれもコマンド出力例のファイル名（実行で再生成可）か完全孤立で、表示中の 19 枚（README 参照）は不変。

## [0.77.0] - 2026-06-05

README を簡潔化（pre-alpha, docs のみ）。521 → 203 行。

### Changed
- 冗長な `<sub>` 長文キャプションを 1 行へ短縮、重複していた「動画→即ロボット」警告を集約。
- 30 行超の機能表を `<details>` の領域別サマリへ折りたたみ。埋め込み/生成の詳細節も `<details>` に集約。
- 陳腐化していた「ステータス」段落（v0.39.0 のまま肥大）を 1 行 + CHANGELOG リンクへ置換。
- ヒーロー GIF を厳選（gallery と重複する単体 GIF を README から外す。アセット自体は保持）。
- 内容（パイプライン・実動画デモ・物理検証・安全方針・ライセンス）は維持。

## [0.76.0] - 2026-06-05

実動画の接地クリーンアップ（pre-alpha）。v0.75 で実証した「単眼抽出 → certificate REJECT」の主因のうち
**接地アーティファクト（airborne 誤検出 / foot skate）**を除去する foot-locking を追加。balance の残差が
**深度律速**であることを切り分けて可視化する。

### Added
- `robotdance_motion/grounding.py` の `ground_contact_cleanup(mir)`: 接地足を毎フレーム z=0 に固定し、
  接地フラグを高さから再生成（grounded performance 前提・跳躍未対応）。軽い再平滑も任意。入力は非破壊。
- `validate-sim --ground-clean`: retarget 前に接地クリーンアップを適用。
- test: 接地足が毎フレーム z≈0 に固定される・接地フラグ/メトリクス再生成・入力非破壊を検証。

### Changed
- README の実動画 certificate ブロックを before→after 比較に更新。実 squat で **airborne 0.484→0.000・
  torque 0.878→0.615** と接地アーティファクトが消え支持多角形が安定。**balance は 0.601→0.474 だが閾値 0.3 超で
  REJECT のまま**で、残った ZMP のはみ出しは前後 x（単眼で最も不確実な深度）方向に偏る。cleaned balance plot 掲載。

### Notes
- 接地（contact）は cleanup で直せるが **balance は深度復元の精度律速**という v0 の frontier を明示。完全 PASS には
  深度推定 / contact-aware retarget の改善が要る（過平滑で見かけ PASS にする gimmick は採らない方針を堅持）。

## [0.75.0] - 2026-06-05

実動画ショーケースを拡充（pre-alpha）。実動画パイプラインを ①overlay ②武道/ダンス多機種 ③physics 検証
の 3 方向に広げ、README の「Real video → humanoid」を本命デモへ。

### Added
- **2D overlay**: 実 squat 動画に MediaPipe 2D 骨格を重ねた検出確認 GIF（`extract`+`overlay`、400px/10fps）。
  3 段パイプライン（overlay → canonical スケルトン → 実 G1）を README に並置。
- **武道・ダンスの実クリップ → 多機種**: karate kata（CC BY-SA 4.0, conf 0.92）→ G1、kathak dance
  （CC BY-SA 4.0, conf 0.95）→ G1（IK 0.068m）+ H1（IK 0.113m）。`render_real_video_gif.py` で生成。
- **実動画の physics 検証**: 抽出 squat を MuJoCo feasibility certificate にかけ **REJECT**（airborne 48% /
  ZMP 支持外 60%、torque 0.88・速度 0.31 は範囲内）+ balance plot を README に掲載。単眼抽出の接地・根高さの
  不確実性を certificate が捉え「動画→即実機」を gate する様子を実証。`validate-sim --balance-plot` で生成。

### Notes
- overlay のみソース動画ピクセルを含む派生物（CC-BY 出典明記で可）。スケルトン/ロボット/balance plot は
  パイプライン出力の可視化でピクセル非含有。入力動画は全て repo 非同梱。Sources: FitnessScape (CC BY 3.0) /
  Sdcsabac (CC BY-SA 4.0) / Suyash Dwivedi (CC BY-SA 4.0), via Wikimedia Commons。

## [0.74.0] - 2026-06-05

本命「Shorts to humanoid」を**実動画**で実証（pre-alpha）。合成代役ではなく、ライセンスがクリアな実
スポーツ動画を MediaPipe Pose にかけて canonical RD-MIR を復元 → actuator-IK で実 G1 へ retarget →
実メッシュ render まで通し、README に実例 GIF を掲載。

### Added
- `scripts/render_real_video_gif.py`: 実動画 → `extract_motion`（MediaPipe）→ 抽出スケルトン GIF +
  `actuator_retarget` → 実 Unitree メッシュ GIF を一括生成（`--robot g1|h1`）。
- `assets/readme/real/squat_g1_{skeleton,robot}.gif`: 実スポーツ動画から復元した squat 動作
  （mean confidence 0.88 / smoothed jitter 0.005 / IK 位置誤差 0.094m）。
- README「Real video → humanoid」節に実例 GIF（① 抽出スケルトン → ② 実 G1）と CC-BY 出典。

### Notes
- 入力動画は repo に同梱せず（license-safe）。出力 GIF はパイプライン出力の可視化で**ソース動画の
  ピクセルを含まない**。Source: 『Squat – exercise demonstration video』by FitnessScape, CC BY 3.0,
  via Wikimedia Commons。「実動画は処理できるが生ファイルは非同梱・renderのみコミット」を実証。

## [0.73.0] - 2026-06-05

README Gallery を 2 機種化（pre-alpha）。v0.72 の実 G1 ギャラリーに実 H1 行を追加し、「同じ振付 ×
別 morphology」で身長・DOF の違いがそのまま出ることを見せる。

### Added
- `assets/readme/gallery/h1_*.gif`: 5 振付（groove / fast / wave / march / squat）を実 H1（19 関節, 1.66m）
  メッシュでレンダリング（`render_gallery.py --robot h1`、IK 位置誤差 0.09〜0.11m）。
- README「🎬 Gallery」を G1 行（1.29m）+ H1 行（1.66m）の 2 段テーブルに拡張。同一振付で実寸 morphology の
  違いが見える旨を注記。

## [0.72.0] - 2026-06-05

README に「色々な振付 → 実 G1 が踊る」GIF ギャラリーを追加（pre-alpha）。「色々な short 動画を入れたら
ヒューマノイドが色々に踊る」という一行説明を、複数振付の hero GIF の壁で見せる。

### Added
- `scripts/render_gallery.py`: 複数の合成モーション（groove / fast / wave / march / squat）を actuator-space IK で
  実 Unitree の関節角へ retarget し、pybullet headless TinyRenderer で実メッシュを 1 振付 = 1 GIF として一括生成。
  `render_robot_gif.py` の単発レンダリングをループ化し、振付定義を `_build_motions()` に集約。
- README に「🎬 Gallery」セクション（実 g1_23dof メッシュの 5 振付 GIF、IK 誤差 0.05〜0.09m）。
  ⚠️ 実動画はライセンス上同梱不可のため「色々な入力」は合成モーション群が代役、メッシュは Unitree（BSD-3, 非同梱）
  と正直に注記。GIF はパイプライン出力（render）でメッシュ再配布ではない。

## [0.71.0] - 2026-06-05

律速軸（binding axis）を benchmark leaderboard に出す（pre-alpha）。v0.70 で Model Card に出した
「どの feasibility 軸が一番詰まっているか」を、標準成果物の leaderboard へ集約し、機種×運動の弱点軸を
一覧化する。

### Added
- `BenchmarkRow` に `binding_axis` / `binding_util`（`_feasibility_headroom` 由来。各 run で律速する軸と
  その限界使用率）。CSV 列・全 run 表（`律速軸(util)` 列）に出す。
- robot 別集計に `top_binding_axis`（機種の**最頻**律速軸＝系統的な弱点）。leaderboard に「最頻 律速軸」列。
- test: speed dance は torque が binding（util>1）、律速軸列が CSV・表に出ることを検証。leaderboard 32 run 再生成。

## [0.70.0] - 2026-06-05

feasibility 三軸の legibility を 1 ビューに統合（pre-alpha）。v0.64-69 で torque/velocity/balance を
個別に可読化したのを、各軸の「限界に対する使用率」で並べて**律速軸（binding）**を一目で示す統合
サマリにまとめる。設計者が「どの軸が一番危ないか・PASS でもどの軸が詰まっているか」を即座に把握できる。

### Added
- Model Card の `executability` に `feasibility`（各軸の `util = metric / 閾値`＝1.0 で境界・>1.0 で違反、
  と律速軸 `binding_axis`/`binding_util`）。torque/velocity/balance/airborne/joint_rom を共通スケールで並べる
  （ROM は閾値 0 なので util=1+違反率）。markdown に「**feasibility headroom**」セクション（util% 降順・binding 明示）。
  実例: H1 dance_fast は binding=torque 177%（left_shoulder 超過）、march は binding=balance 150%（torque 107% も超過）、
  PASS な dance_normal は binding=torque 86%（余裕 14%）。
- test: PASS は全軸 util<1 で binding=最大 util 軸、overbend は binding=joint_rom（util>1）。

## [0.69.0] - 2026-06-05

velocity 軸の律速関節を torque と対称に集約（pre-alpha）。feasibility 三軸（torque/velocity/balance）の
legibility を揃える: torque は v0.65/v0.66 で律速関節を metrics・Model Card に出していたが、velocity は
reason のみで非対称だった。Apollo の velocity は menagerie MJCF に実値が無く generic fallback のまま
（存在しないデータは捏造せず正直に残す）。

### Added
- `simulate_certificate` の metrics に `velocity_limiting_joint`（実 per-joint 速度上限に最も近い関節。
  PASS でも出す。`torque_limiting_joint` と対称）。
- Model Card の `executability` に `tightest_velocity`（律速関節・`ratio`・余裕 `headroom`）と markdown
  「**律速関節（速度）**: {関節} ×{ratio}（余裕 {±headroom}, 余裕/超過）」行。実例: H1 fast dance は
  torque律速 left_shoulder ×1.77（超過）／velocity律速 left_elbow ×0.40（余裕）。
- test: velocity_limiting_joint が metrics に出る（REJECT/PASS 両方）、Card に tightest_velocity が出る。

## [0.68.0] - 2026-06-05

balance（ZMP×支持多角形）の可視化を追加（pre-alpha）。トルクを可読化した v0.64-66 と対に、これまで
`balance_violation_ratio` の単一数値だった balance を「どのフレームで・どこで ZMP が支持を外れるか」まで
診断可能にする。

### Added
- `simulate_certificate(..., return_trace=True)` で per-frame の **ZMP 軌跡・支持多角形・in/out 判定**を
  `result["trace"]` に返す（可視化が再計算せず certificate と同じ値を使う single source of truth。既定は trace なし）。
- `robotdance_viewer/balance_view.py` の `render_balance_plot`: trace を**上面図 PNG**に描く（支持多角形＝
  接地足フットプリント、ZMP 軌跡、支持内=緑/支持外=赤×）。軸は足位置で固定し、準静的 ZMP の外れ値で
  スケールが崩れないようにする。
- CLI `validate-sim --balance-plot out.png`（mujoco backend）。trace は保存ファイルへ持ち込まない。
- test: trace の長さ・支持外率が `balance_violation_ratio` と整合すること、PNG が生成されることを検証。

### Notes
- 実験的に確認した負の結果も記録（SIM_TO_REAL）: 広股機種の balance を**剛体並進**で救えない（支持足も
  動く）し、**剛体全身傾斜の足首戦略は balance を一部下げるが torque を悪化**させる（真の足首戦略は足首
  のみ屈曲し torso を低加速で保つ）。v0 はこの能動バランスを未モデルのまま正直に残す。

## [0.67.0] - 2026-06-05

合成スイートに `march_gentle` を追加し、**march の feasibility が歩調＋形態＋能動バランスで決まる**ことを
実証（pre-alpha）。単脚支持が「v0 では原理的に不可能」ではなく「適切な歩調なら狭股機種で実現可能」で
あることを示し、広股機種で残る balance 違反の理由（足首戦略の未モデル）を明文化する。

### Added
- `default_motion_suite` に `march_gentle`（低速・低い持ち上げの足踏み）。歩調を落とすと慣性トルクが
  下がり、**狭股機種（G1/T1）は重心が支持多角形内に収まり PASS**（naive march は全機種 REJECT）。
  広股機種（H1/Apollo）は受動準静的モデルでなお balance 違反（torque は全機種で解消）。benchmark は 32 run に。
- test: gentle march が G1 を REJECT→PASS に転じ、H1 の torque_ratio を下げつつ広股ゆえ balance は残ることを検証。

### Changed
- `docs/SIM_TO_REAL.md` の march 節を拡張: feasibility は歩調（慣性）＋形態（股幅）＋能動バランスの
  有無で決まると明記。**剛体並進では支持足も動き COM-足の相対が変わらず balance を改善できない**（足を
  接地したまま上体を傾ける足首戦略 IK が要る）ことを実装上の注意として記録。v0 はこの能動バランスを未モデル。

## [0.66.0] - 2026-06-05

実データ深掘り（律速関節を Model Card の executability に集約, pre-alpha）。v0.65 で certificate に出した
律速関節を、設計者が見る **Model Card** まで伝播し、診断チェーン（cert→card）を完成させる。

### Added
- Model Card の `executability` に `tightest_torque`（律速関節・負荷率 `ratio`・余裕 `headroom = 1 − ratio`）。
  **PASS でも**「どの関節が effort 上限に最も近いか」を設計者へ示す（REJECT のときは blocker と併せて表示）。
- Markdown 描画に「**律速関節（トルク）**: {関節} ×{ratio}（余裕 {±headroom}, 余裕/超過）」行を追加。
  実例: H1 dance_fast は left_shoulder ×1.77（余裕 −0.77, 超過）、安全運動は余裕 > 0 で表示。

### Notes
- `tightest_torque` は sim_certificate の `torque_limiting_joint`/`torque_ratio` 由来（剛体 subtree 近似, v0.65）。

## [0.65.0] - 2026-06-05

実データ深掘り（トルクの律速関節を certificate に明示, pre-alpha）。v0.64 でトルク大きさを可視化したのに
続き、**どの関節が effort 上限を律速するか**を出して REJECT 理由を診断可能にする。

### Added
- `simulate_certificate` の metrics に `torque_limiting_joint`（per-joint 負荷率 `dynamic_torque/実 effort
  上限` が最大の canonical 関節。PASS でも最も上限に近い関節として出す）。
- torque REJECT 理由に律速関節を併記: 「{関節名} {動的tq}>{上限} N·m, 重力＋慣性, 実 actuator 限界超過」。
  実例: H1 dance_fast は **right_shoulder 31>18 N·m**（速い腕振りで肩が律速）。velocity reason と同流儀。

### Notes
- 律速関節は body subtree 負荷率の argmax で、剛体 subtree 近似に基づく（衝撃・接触・摩擦は未モデル）。

## [0.64.0] - 2026-06-05

実データ深掘り（重力 vs 動的トルクを leaderboard に分離露出, pre-alpha）。v0.62/v0.63 で実装した
「重力保持」と「重力＋並進＋回転慣性」の 2 成分を benchmark CSV / leaderboard に出し、各 motion×robot
が**重力支配か慣性支配か**を可読化する。

### Added
- benchmark の各 run に `gravity_torque_nm`（重力保持＝準静的成分）と `dynamic_torque_nm`（重力＋並進
  ＋回転慣性の合計）を記録（`BenchmarkRow`／CSV 列／leaderboard 表）。両者の差が**慣性寄与**で、速い
  運動ほど開く（例: idle は重力≈動的、dance は動的≫重力）。`torque×` は従来どおり動的tq/実 effort 上限。
- robot 別集計に「平均 動的tq(N·m)」列を追加（機種の重さ・慣性負荷の比較が一目で可能。Apollo 125 / H1 62
  / G1 36 / T1 25 N·m）。
- benchmark テストに gravity/dynamic torque の CSV・leaderboard 伝播と「動的 ≥ 重力」検証を追加。

## [0.63.0] - 2026-06-05

実データ深掘り（トルクに回転慣性反作用を追加, pre-alpha）。v0.62 で省いた回転慣性項を加え、関節
トルク評価を Newton-Euler の完全形に近づける。

### Changed
- `simulate_certificate` の `torque_ratio` を **`τ = dL_com/dt + r × m·(a_com − g)`** へ拡張。第2項（重力
  ＋並進慣性, v0.62）に、第1項 **subtree の COM まわり角運動量変化 `dL_com/dt`（回転慣性反作用）**を追加。
  subtree_angmom は reference 速度から `mj_subtreeVel` で取得（mj_inverse の非物理値を避けた robust な解析法）、
  dL/dt は中心差分。実例: H1 dance_fast の torque_ratio が 1.25→1.70 に精緻化（回転慣性で更に上振れ）。
  slow/準静的運動（idle/squat）は dL≈0 でほぼ不変。全 verdict は v0.62 から不変（magnitude のみ精緻化）。

### Notes
- 副次的発見: capsule 近似慣性は borderline 運動で torque_ratio を実慣性より高く出し verdict を反転
  させうる（実慣性の価値。capsule は実機より保守的）。test を更新。

## [0.62.0] - 2026-06-05

実データ深掘り（トルク評価に動的（慣性）成分を取り込み, pre-alpha）。v0.61 の sim-to-real doc で
「最大の近似」と挙げた **重力保持（準静的）トルクの過小評価**を是正。速い運動の慣性トルクを含める。

### Changed
- `simulate_certificate` の `torque_ratio` を **重力＋慣性トルク**へ拡張。各 joint の負荷を subtree COM に
  働く力 `m·(a_com − g)`（a_com=subtree COM 加速度, ZMP と同じ中心差分）の関節まわりモーメントとして
  算出し、実 per-joint effort 上限と比較。重力保持（準静的）だけでは速い運動で過小評価していた。
  **mj_inverse はこの ball-joint 浮遊モデルで特異性により非物理値（数千 N·m）を出すため使わず**、点質量
  at COM の robust な解析法を採用（回転慣性×角加速度の項は二次的として省く）。metric に `dynamic_torque_nm`
  を追加（`gravity_torque_nm`=静的も併記）。
- 実例: **H1 dance_fast は静的トルク 0.63（PASS）だったが、慣性込みで 1.25 となり torque で REJECT**。
  leaderboard を再生成（速い運動・足踏み・宙返りで torque× が上昇）。idle/squat 等の準静的運動は不変。

## [0.61.0] - 2026-06-05

新展開（sim-to-real ギャップを明文化, pre-alpha）。多機能・多機種が揃った今、v0 の近似と「feasibility
≠ 実機保証」の境界を文書化し、プロジェクトの誠実さを締める。

### Added
- `docs/SIM_TO_REAL.md`: certificate が**検証すること/しないこと**の対比表、パイプライン段階別の v0 近似
  （retarget の twist 規約 / capsule 質量プロキシ / 準静的 ZMP・平地 / **重力保持トルク（動的トルク
  含まず）** / 接地の運動学判定 / PD baseline）、未モデル項目（actuator 動力学・遅延・センサノイズ等）、
  実機へ渡す前の必須手順（balance 制御＋safety guard＋漸進検証）を明示。README / sim README からリンク。

### Fixed
- ステールなコメントを是正（誠実さ）: `mujoco_backend` の docstring「質量・慣性は近似（bone 長比）」→
  実 URDF 由来（v0.34/v0.52）に。sim README のトルク手法「mj_inverse 純 RNEA」→ 実際の**重力保持
  トルク（準静的・解析計算, 動的トルク含まず）**に修正。

## [0.60.0] - 2026-06-05

新展開（合成モーションスイートを拡充, pre-alpha）。4 機種が揃った feasibility 検証を、より多様な運動で
exercise する。squat（膝 ROM/保持トルク）と march（単脚支持 balance）を追加。

### Added
- `synthetic.generate_squat`: 両脚対称の深屈曲＋接地保持。膝 ROM と屈曲位の保持トルクを exercise
  （動的にクリーン → feasible。torque_ratio は機種差: G1 0.28 / H1 0.56 / Apollo 0.40）。
- `synthetic.generate_march`: その場足踏み（片足を交互に持ち上げ、支持脚上へ root を横移動）。
  **単脚支持の balance（ZMP vs 支持多角形）**を exercise（airborne ではなく balance 軸で REJECT）。
- `_ground_and_contacts` ヘルパ（足を接地高さへ z シフト）。両 motion を default_motion_suite に追加。
  benchmark leaderboard を 7 motion × 4 robot = 28 runs で再生成。

## [0.59.0] - 2026-06-05

新展開（4 機種目 Apptronik Apollo を追加, pre-alpha）。full-size humanoid（~1.62m / 80.9kg）を実
menagerie モデルから追加し、フレームワークの機種非依存性をさらに実証。

### Added
- `robotdance_unitree.apptronik_apollo`: canonical 19-joint へ写像した Apollo morphology。rest pose /
  位置 ROM / **実 forcerange トルク**（膝336/肘114/股120）/ 質量分布（総 80.9kg, 胴体 19.3kg）/ 慣性
  テンソルを **MuJoCo Menagerie の Apollo モデル（Apache-2.0）の実値**から抽出。**抽出は MuJoCo に
  モデルを読ませ world frame / world 軸慣性を厳密計算**（diaginertia+quat の回転を MuJoCo が処理）→
  最近傍ボーン中点割当・平行軸合成。`real_inertia=True` 対応（body_inertia 一致をテストで担保）。
  PD 既定 kp=400/kd=12。benchmark 既定に追加（20 runs, Apollo PASS 率 0.8）。
- `docs/EMBODIMENTS.md` に Apollo 行を追加（7 軸カバレッジ）。velocity は menagerie MJCF に無く未収載
  （6 軸が実 Apollo 値, follow-up）。

### Fixed
- `_joint_flexion_metrics` / `_clamp_flexion_to_limits` の屈曲限界を `position[1]`（上限）から
  **`max(|lo|,|hi|)`** に修正。屈曲角は arccos（非負・方向不問）なので、屈曲を負方向に取る機種
  （Apollo 肘 `[-2.618, 0.175]`）で `position[1]=0.175` を限界とすると全フレーム誤検出していた。
  屈曲側が上限の機種（G1/H1/T1）では `max(|lo|,|hi|)=position[1]` で**後方互換**。

### Notes
- **license-safety**: Fourier GR-1 / N1（FFTAI Wiki-GRx-Models）は **GPL-3.0（copyleft）**のため Apache-2.0
  の本プロジェクトには取り込まず、permissive な代替として Apollo（Apache-2.0）を採用。上流 LICENSE を直接確認。

## [0.58.0] - 2026-06-05

新展開（対応ロボットの data provenance を明文化, pre-alpha）。実データ・ライセンス・抽出方法・7 軸
カバレッジを文書化し、本プロジェクトの「license-safe・実 URDF データ」という誠実さの再現性を担保する。

### Added
- `docs/EMBODIMENTS.md`: 3 機種（G1 / H1 / Booster T1）の provenance 表。出典 URDF・**ライセンス**
  （Unitree=BSD 3-Clause, Booster T1=Apache-2.0, いずれも上流 LICENSE を直接確認）・総質量・身長・
  runtime adapter・**7 軸の実データカバレッジ matrix**・再現可能な抽出手順・attribution を明記。
  数値定数のみ派生利用し mesh/URDF 本体は非同梱という方針を明示。
- tests: provenance doc が全 embodiment とライセンス出典を載せることを担保（文書化漏れガード）。
  README の multi-embodiment 行から doc へリンク。

## [0.57.0] - 2026-06-05

新展開（benchmark leaderboard を 3 機種化, pre-alpha）。7 軸フル実データの 3 機種（G1 / H1 / Booster T1）を
canonical leaderboard に並べ、機種固有の feasibility 差を可視化する。

### Changed
- benchmark 既定 robot を G1/H1/T1 の 3 機種に拡張（`benchmark` CLI default + `_ROBOT_COLORS`）。
  leaderboard / csv を 15 runs（5 motion × 3 robot, 全機種 実慣性）で再生成。
- 可視化された機種差: **T1 backflip は torque× 1.325**（弱いアクチュエータ: 膝 60N·m で backflip トルク
  不能 → G1/H1 とは別理由で REJECT）。**T1 dance_fast は PASS**（G1 は balance で REJECT — 小型軽量で
  COM 加速度が小さく ZMP が支持内）。T1 height_scale 0.686（小型）。PASS 率 G1 0.4 / H1 0.8 / T1 0.6。

## [0.56.0] - 2026-06-05

新展開（Booster T1 を 7 軸フル実データに, pre-alpha）。v0.55 で追加した T1 に**実 URDF 慣性テンソル**を
収載し、G1/H1 と同格の `real_inertia=True` 対応にした。あわせて v0.55 の質量割当の取り違えを是正。

### Added
- `booster_t1.T1_INERTIA_TENSORS`: 実 Booster T1 URDF `<inertial>` を canonical bone へ平行軸合成した
  per-bone 慣性（T1 URDF は inertial frame 無回転なので並進のみ）。`EMBODIMENT_INERTIA` に登録 →
  `get_morphology("booster_t1", real_inertia=True)` で実慣性 sim（certificate approximate_inertia=False、
  PD 追従も安定）。MuJoCo body_inertia が埋め込み固有値に一致することをテストで担保。

### Fixed
- T1 質量分布のセグメント取り違えを是正。v0.55 は **世界 COM 最近傍ボーンでなく明示セグメントで割当て、
  大腿(Hip_Yaw)を hip・下腿(Shank)を knee bone に置いていた**（1 区間上にズレ）。最近傍ボーン中点へ
  再割当てし、大腿=knee bone（0.080）/ 下腿=ankle bone（0.055）と物理的に正しい縦位置に修正（総質量不変）。

## [0.55.0] - 2026-06-05

新展開（3 機種目 Booster T1 を実 URDF から追加, pre-alpha）。深掘りで G1/H1 に整えた「近似→実 URDF
データ」基盤の**機種非依存な汎用性**を、別ベンダの小型機 Booster T1（~0.98m / 31.6kg）で実証する。

### Added
- `robotdance_unitree.booster_t1`: canonical 19-joint へ写像した T1 morphology。rest pose / 関節
  limit（位置・速度・**実 effort トルク**: 膝60/股45/腕18）/ 質量分布（総 31.614kg, 胴体 37%）は
  **公式 Booster T1 URDF（BoosterRobotics booster_gym, Apache-2.0）の実値**から抽出（数値定数のみ・
  mesh/URDF 本体は非同梱, license-safe・attribution 付き）。`get_morphology("booster_t1")` で利用。
- T1 の PD 既定ゲインを実測スイープで決定（kp=300: 小型でも短 bone で実効慣性が小さく G1 の kp=150 では
  転倒。kp≥300 で全振幅 survival 1.0/upright 1.0）。retarget / certificate（6 軸: 寸法/質量/トルク/速度/
  ROM/balance）/ PD tracking が機種非依存に機能。tests + RD-Embodiment schema に `booster_sdk` adapter 追加。

### Notes
- T1 の**慣性テンソルは未収載**（per-link rpy 回転＋平行軸合成を要するため follow-up）。`real_inertia=True`
  でも T1 は registry に無く capsule にフォールバック（certificate は approximate_inertia=True）。geometry /
  limit / 質量は実 URDF 値。benchmark 既定 robot は G1/H1 のまま（leaderboard 不変）。

## [0.54.0] - 2026-06-05

実データ深掘り（PPO tracking を実慣性で安定化, pre-alpha）。v0.37 で「実慣性を入れると PPO tracking が
崩壊（survival ~0.03）」したため慣性を opt-in に留めていた最後の宿題。原因は当時の reference qpos が
twist アーティファクトで関節速度スパイクを持ち、実慣性のダイナミクスでコントローラが発散したこと。
**v0.47 で reference を twist 連続化したので、実慣性でも PPO は安定して学習・追従する**ことを実証した。

### Added
- CLI `train-tracking --real-inertia`: 実 URDF 慣性テンソルで RL tracking policy を学習する
  （`get_morphology(robot, real_inertia=True)`）。既定は capsule（v0 baseline 互換）。
- tests: 実慣性 PPO 学習が安定（return 有限・survival>0.3, 実測 capsule と同等 survival 1.0 /
  rmse 0.372）であることを担保（v0.37 崩壊の回帰ガード）。

### Notes
- v0.37 の「実慣性で controller 再チューニングが必要」は **reference 品質（v0.47 twist 連続化）の
  問題であり、controller 自体の再チューニングは不要**だった。capsule と実慣性で PPO 学習結果は同等。
  深掘りスレッドの最後の宿題が解け、「近似→実 URDF データ」が retarget/sim/safety/品質/tracking 全経路で完了。

## [0.53.0] - 2026-06-05

実データ深掘り（三軸 feasibility を Model Card に明示, pre-alpha）。v0.50 で velocity を certificate の
第3 feasibility 軸にしたが、Model Card の `executability.checked_axes` は dynamics/joint_rom のみで
「速度も実機値で検証済み」が表に出ていなかった。consumer が**どの実 URDF 軸を検証したか**を一目で
分かるよう velocity 軸を表出した。

### Changed
- `model_card._executability`: `checked_axes` に `joint_velocity` を追加（cert に joint_velocity_ratio
  metric があるとき＝per_joint_limits を持つ embodiment）。安全な motion は
  `["dynamics", "joint_velocity", "joint_rom"]` の三軸全てを検証したと表示。per_joint_limits が無ければ
  従来どおり dynamics のみ。CLI `validate-sim` の executability 出力にも自動反映。

## [0.52.0] - 2026-06-05

実データ深掘り（feasibility 検証を既定で実 URDF 慣性に, pre-alpha）。深掘りの集大成。v0.51 で PD
経路が実慣性で安全と実証したので、sim_certificate を **既定で実 URDF `<inertial>` 慣性テンソル**で
検証するよう切替えた（capsule 近似は COM を幾何中心に置き subtree COM→重力トルクを誤推定していた）。
全 motion×robot で **verdict 反転ゼロ**を確認した上での切替で、tracking/PPO 経路は構造的に不変。

### Changed
- `simulate_certificate`（と `certify`）: `real_inertia: bool = True` を追加し既定で実慣性検証。
  morphology が inertia_tensors を持たなければ `EMBODIMENT_INERTIA` から名前で装着する（sim→unitree は
  lazy import で cycle 回避）。`approximate_inertia` は実慣性使用時 False を返すよう正直化。`real_inertia=False`
  で旧 capsule 近似を再現可能。tracking/PPO は自前で capsule モデルを組むため**本フラグの影響を受けない**。
- benchmark leaderboard / csv を実慣性で再生成（torque× が補正、例: H1 dance 0.627→0.513 で capsule が
  ~22% 過大評価、balance も実 COM で微変）。**PASS 率・全 verdict は不変**（G1 0.4 / H1 0.8）。

## [0.51.0] - 2026-06-04

実データ深掘り（実慣性を first-class 化、PD 経路で安全と実証, pre-alpha）。v0.37 で実 URDF 慣性
テンソルを追加したが「controller が崩壊する」として opt-in（private dict を手で差し替え）に留めていた。
本版でその崩壊が **PPO 学習ポリシー限定**であり、**PD baseline は実慣性で全く退行しない**ことを実証し、
実慣性を選択する first-class API を整えた。

### Added
- `get_morphology(name, *, real_inertia=False)`: `real_inertia=True` で実 URDF `<inertial>` 由来の
  per-bone 慣性テンソルを装着する first-class API（旧来は `dataclasses.replace` で private dict を手で
  差し替える必要があった）。`EMBODIMENT_INERTIA` registry を追加。
- tests: PD-only 追従が実慣性で安定（G1/H1 とも survival 1.0・RMSE 退行 ≤0.006）であることを担保。

### Notes
- 既定は capsule 慣性のまま（PPO baseline 退行回避）。**PD 追従・feasibility 検証では `real_inertia=True`
  を安全に使える**。実慣性は certificate のトルク評価も補正する（実測: H1 torque_ratio 0.627→0.513 で
  capsule が ~22% 過大評価、gravity 32.5→27.7 N·m — 実 COM オフセットが subtree COM を正す）。
  既定の capsule→実慣性への切替（benchmark 再生成・verdict 再検証を伴う）は次の増分候補。

## [0.50.0] - 2026-06-04

実データ深掘り（関節速度 feasibility を実 per-joint 速度上限へ配線, pre-alpha）。sim_certificate の
速度判定は全関節一律ハードコード 30 rad/s で、実機差（9-37 rad/s）を無視し H1 足首/肩のような遅い
actuator 限界（9 rad/s）の超過を見逃していた。v0.49 で「reference は実速度上限内で trackable」を
示した流れを certificate 本体へ統合し、velocity を dynamics/ROM に続く第3の feasibility 軸にした。

### Changed
- `simulate_certificate`: 速度判定を per_joint_limits があるとき **実 per-joint 速度上限**
  （`per_joint_limits.velocity`, v0.38）との比較へ置換（無ければ従来の一律 30 rad/s）。temporal qpos
  （v0.47 で twist 連続化済み）の**関節相対速度**（actuator が駆動する量）を `mj_differentiatePos` で取り、
  実値を持つ関節のみ（generic placeholder 除外）で判定。metric `joint_velocity_ratio`（>1.0 で
  追従不能）+ threshold 1.0 を追加。実例: H1 高速腕運動は肩を ~11 rad/s で駆動し実上限 9 を超える
  （bone 世界角速度は 30 未満なので旧一律では見逃し）。= v0.36 のスカラ→per-joint トルクと同型の是正。

### Added
- `robotdance_sim.mujoco_backend._max_joint_velocity_ratio`: temporal qpos の関節相対速度 / 実
  per-joint 速度上限 の最大比と worst joint 詳細を返すヘルパー。

## [0.49.0] - 2026-06-04

実データ深掘り（reference の追従可能性を実機速度上限で判定, pre-alpha）。v0.48 は reference 速度の
大きさを測ったが、それが「実機で追従可能か」までは踏み込んでいなかった。本版で reference の要求
関節速度を**実 URDF アクチュエータ速度上限**（v0.38 取り込みの `per_joint_limits.velocity`）と比較し、
偽 twist スパイクが reference を物理的に追従不能にしていること、時系列復元がそれを解消することを示した。

### Added
- `robotdance_sim.reference_quality.reference_trackability_report`: reference の要求関節速度を実機
  速度上限と比較し `{per_frame, temporal}_untrackable_ratio`（上限超過フレーム率）と `max_demand_ratio`
  （要求速度/上限の最大）を返す。`>1.0` で追従不能。
- `docs/sim/REFERENCE_QUALITY.md` に追従可能性テーブルを追加。**backflip は per-frame 復元だと実機の
  アクチュエータ速度上限を最大 2.4× 超える要求を ~10% のフレームで出す（追従不能）が、時系列復元は
  全 motion で 0%・≤0.4×**（速度包絡内）。コントローラ学習・PD 追従に渡せる trackable な reference を保証。

## [0.48.0] - 2026-06-04

実データ深掘り（twist 安定化の効果を定量化, pre-alpha）。v0.47 で reference qpos の twist を時間
連続化したが、その恩恵がどの motion でどれだけ効くかは未計測だった。本版で reference 関節速度を
MuJoCo 自身の tangent 空間差分（PD が追うベクトルそのもの）で測り、単フレーム復元 vs 時系列復元を
default_motion_suite × G1/H1 で比較した。

### Added
- `robotdance_sim.reference_quality`: `reference_velocity_report`（単フレーム vs 時系列復元の
  reference 最大関節速度・bone 真値・spike factor）と `reference_quality_table`（スイート全体の
  markdown 表）。`python3 -m robotdance_sim.reference_quality` で doc 再生成。
- `docs/sim/REFERENCE_QUALITY.md`: 生成された比較表。overbend G1 で 79.9→3.92 rad/s（20.4× の
  偽スパイク除去）、**backflip でも G1 5.3×・H1 7.3×**（現実的な動的運動でも効く）。通常ダンスは
  ~1.0×（特異点を踏まないため差なし）。位置・COM・verdict は不変で、差はすべて不可観測な twist。

## [0.47.0] - 2026-06-04

実データ深掘り（再構成 qpos の twist を時間方向に安定化, pre-alpha）。v0.43 で sim の角速度指標は
bone 方向ベース（twist-free）に是正したが、その根である「keypoints から復元した qpos 自体の
twist 不連続」は残っていた。本版で qpos 復元を時系列対応にし、reference 速度・PD 追従誤差・export
軌道など **qpos を差分する全経路**から偽 twist スパイクを除去した。

### Changed
- `robotdance_sim.mujoco_backend` に時系列 qpos 復元 `_poses_to_qpos`（[T,J,3]→[T,nq]）を追加。
  各 bone の world フレームを **frame 0 は rest 基準の shortest-arc で seed し、以降は連続フレーム間の
  swing だけで前進**させて twist を注入しない。極端な屈曲（観測 bone 方向が rest と反平行付近に滞在）で
  フレーム独立復元が踏む shortest-arc 特異点を回避し、手首で ~80 rad/s の偽 twist スパイクを実 bone
  速度（~4 rad/s）へ是正。bone 方向は厳密再現のため FK 位置・COM・ZMP・トルク・verdict は完全不変
  （位置差 ~5e-17 m）。
- `simulate_certificate` と `TrackingEnv`/`MultiTrackingEnv` の `ref_qpos` 構築を `_poses_to_qpos`
  経由に変更。RL tracking の reference 速度と PD 追従誤差（`mj_differentiatePos`）が偽スパイクで
  汚染されなくなった。単フレーム復元 `_pose_to_qpos` は後方互換のため残置（twist 規約の注意を docstring 化）。

## [0.46.0] - 2026-06-04

実データ深掘り（executability を CLI に露出, pre-alpha）。v0.45 で Model Card に追加した統合
executability サマリを CLI でも見えるようにし、ユーザーが `validate-sim` だけで実行可否と
remedy を確認できるようにした。

### Added
- CLI `validate-sim`（`robotdance_core.cli`）: 動的＋運動学的を集約した `executable: ✅/❌/❔`
  サマリを出力。ROM が blocker のときは `--clamp-flexion` での補正を誘導。
- CLI `validate-sim --clamp-flexion`: 膝・肘を実機可動域へ補正してから検証（ROM 違反の remedy）。
  overbend G1 は補正なしで rc=1（REJECT）、`--clamp-flexion` で rc=0（PASS）。

## [0.45.0] - 2026-06-04

実データ深掘り（Model Card に統合 executability サマリ, pre-alpha）。v0.44 で動的＋運動学的
feasibility を sim_certificate に統合したが、Motion Card では sim_certificate と
kinematic_feasibility が別々に埋まり、consumer が「結局この motion は実機で実行してよいか」を
一目で判断しづらかった。本版で両軸を集約した executability サマリをカード上位に追加した。

### Added
- Motion Card（`robotdance_core.model_card`）に `executability` を追加:
  `{executable: true|false|null, checked_axes, blockers, remedy?}`。sim_certificate があれば
  その verdict（v0.44 で ROM 統合済み）が権威、無ければ executable=null（動的未検証）だが可動域は
  joint_flexion から報告。ROM が blocker のときは clamp_flexion の remedy を併記。Markdown は
  ✅/❌/❔ と blocker・remedy を表示。

## [0.44.0] - 2026-06-04

実データ深掘り（動的＋運動学的 feasibility の統合, pre-alpha）。v0.43 で overbend が動的には安定
（sim PASS）だが実機 ROM 超過と判明したが、`sim_certificate.verdict` は動的判定のみだったため
**verdict=PASS を信じた consumer が実機に不可能な肘角を指令しうる**穴があった。本版で運動学的
feasibility（joint_flexion）を verdict に統合し、検出→可視化→検証→補正→**enforce** までを閉じた。

### Changed
- `simulate_certificate`（`robotdance_sim.mujoco_backend`）: `motion.retarget_metrics.joint_flexion`
  の可動域違反を REJECT 理由に統合（per_joint_limits を持つ embodiment のみ）。動的に安定でも実機
  ROM を超える姿勢は「指令不能」として REJECT する。`metrics.joint_flexion_violation_ratio` と
  `thresholds.joint_flexion_violation_ratio` を追加。REJECT 理由には clamp_flexion での補正を誘導。

### Notable
- overbend G1 は動的指標が全てクリーン（airborne/balance/角速度 OK）だが肘 ROM 超過 0.25 → REJECT。
  `clamp_flexion=True` で補正すると PASS になり、**clamp が feasibility 上の remedy として機能する**
  ことを end-to-end で確認。sample leaderboard を再生成して反映（G1 PASS 率 0.4）。

## [0.43.0] - 2026-06-04

実データ深掘り（sim_certificate の角速度を twist アーティファクトから是正, pre-alpha）。clamp の sim
効果を検証中に、`max_joint_ang_speed` が滑らかな動きに対し偽の巨大スパイク（overbend で ~79 rad/s）を
出し overbend を誤って REJECT していたことを発見・修正した。

### Fixed
- `max_joint_ang_speed_rad_s`（`robotdance_sim.mujoco_backend`）: keypoints から再構成した ball-joint
  quaternion の差分で算出していたが、**bone 軸まわりの twist は keypoints から定まらず**（向きが任意）、
  手首など leaf joint・極端な屈曲で再構成 quaternion が不連続化し、データに存在しない偽の角速度を
  生んでいた。bone 方向（2-DOF, twist-free）の変化率から算出する `_max_bone_angular_speed` に置換。
  backflip の角速度は 17–24→4.0 rad/s に是正（剛体回転中の偽 twist 除去）。verdict 変化は overbend
  G1 の REJECT→PASS のみで、他の verdict（dance PASS / dance_fast・backflip REJECT）は不変。

### Notable
- overbend G1 が **sim PASS（動的に安定）かつ joint_flexion 違反 0.25（運動学的に可動域超過）** となり、
  動的 feasibility（sim_certificate）と運動学的 feasibility（joint_flexion）が**直交・相補的な 2 軸**で
  あることを leaderboard 上で実証。sample leaderboard を再生成して反映。

## [0.42.0] - 2026-06-04

実データ深掘り（屈曲違反の検出→自動補正, pre-alpha）。v0.39〜0.41 で「膝・肘の屈曲が実機可動域を
超える」ことを検出・可視化・検証してきたが、診断のみだった。本版で **kinematic retarget に可動域内へ
収める補正オプション**を追加し、診断から治療まで一貫させた。

### Added
- `retarget(..., clamp_flexion=True)`（`robotdance_retarget.kinematic`）: 屈曲が実 per-joint 可動域
  上限を超えるフレームで、遠位サブチェーンを hinge 中心に剛体回転させ屈曲角を上限ちょうどへ収める
  （d1-d2 平面内 slerp で目標方向→Rodrigues 回転、**bone 長は厳密保存**）。補正量は
  `retarget_metrics.joint_flexion.clamp`（pre_clamp_max_flexion / corrected_frame_ratio）に記録。
  膝補正は接地クランプ前に行い再接地。per_joint_limits 無や可動域内では no-op。
- CLI: `robotdance retarget --clamp-flexion`。屈曲違反と補正量を標準出力に表示。

### Notable
- 可動域順守と忠実度のトレードオフは正直に表示: overbend を G1 で補正すると肘違反 0.25→0.00 になる
  代わり bone_direction_cosine が 1.000→0.999 へわずかに低下する。

## [0.41.0] - 2026-06-04

実データ深掘り（joint-flexion 違反の end-to-end 検証, pre-alpha）。v0.39/0.40 で作った屈曲メトリクスが
「実際に可動域違反を検出し benchmark / Model Card まで伝播する」ことを、過屈曲する合成モーションで
単離検証した。これまで違反>0 のテストは手組み keypoints のみで、synthetic→retarget→集計の実経路は
未検証だった。

### Added
- `robotdance_core.synthetic.generate_overbend()`: 肘を実機可動域上限を超えて折り畳む合成 RD-MIR。
  脚は接地・直立を保ち「運動学的に可動域だけを超える」ケースを単離。benchmark の `default_motion_suite()`
  に `overbend` として追加。
- 統合テスト: synthetic→retarget で肘違反>0（`test_retarget`）、benchmark leaderboard への伝播
  （`test_benchmark`）、Model Card `kinematic_feasibility` への伝播（`test_model_card`）。

### Notable
- 同じ overbend モーションが **G1（肘上限 2.09）では屈曲違反 0.25、H1（肘上限 2.61）では 0.00** と
  embodiment 固有の可動域を区別して検出する。sample leaderboard を再生成して反映。

## [0.40.0] - 2026-06-04

実データ深掘り（joint-flexion メトリクスを benchmark / Model Card で可視化, pre-alpha）。v0.39 で
作った `retarget_metrics.joint_flexion`（膝・肘の屈曲角 vs 実 per-joint 可動域上限）は JSON に埋まる
だけだったため、benchmark leaderboard と Motion Card に表出させて「retarget が実機可動域を超えていないか」
を一目で見えるようにした。

### Added
- benchmark（`robotdance_benchmarks`）: `BenchmarkRow.joint_flexion_violation`（= `joint_flexion`
  の `any_violation_ratio`）を追加。CSV 新列・leaderboard の robot 別「平均 屈曲違反率」・全 run 表の
  「屈曲違反」列に表出。per_joint_limits を持つ embodiment（G1/H1）でのみ値が入る。
- Model Card（`robotdance_core.model_card`）: Motion Card の `safety_limits.kinematic_feasibility`
  に `joint_flexion_violation_ratio` と対象関節（膝・肘）を表出。>0 は実機可動域超過フレーム有 →
  retarget 要見直しのシグナル。

## [0.39.0] - 2026-06-04

実データ深掘り（per-joint limit を retarget 品質評価へ, pre-alpha）。kinematic retarget は keypoints
のみ出すため可動域チェックが無かった（actuator-space IK は実 limit で clamp 済み）。膝・肘の屈曲角を
bone 方向から導出し、実 per-joint 可動域の超過を `retarget_metrics` に記録するようにした。

### Added
- `retarget_metrics.joint_flexion`（`robotdance_retarget.kinematic`）: 1-DOF ヒンジ（膝・肘）の屈曲角を
  近位/遠位 bone のなす角として導出し、embodiment の per_joint_limits（実 URDF 由来上限）と比較。
  各関節の max_flexion・上限・violation_ratio と全体の any_violation_ratio を出す。per_joint_limits が
  無い morphology では出さない（測れない）。股・肩は 3-DOF で屈曲角が一意でないため対象外（v0）。

## [0.38.0] - 2026-06-04

実データ深掘り（safety guard の per-joint 化を完成, pre-alpha）。guard は位置・トルクが per-joint
実値だったが、角速度だけ単一スカラー（全関節を最厳の min で一律クランプ）だった。実機は関節ごとに
速度上限が大きく異なる（H1: 足首 9 vs 股 23 rad/s）ため、速度も per-joint 化した。

### Fixed
- **safety guard の角速度クランプを per-joint へ**（`robotdance_ros2.safety_guard`）:
  `SafetyLimits.joint_speed_limits`（actuator 名 → rad/s）を追加し、`clamp_joint_trajectory` /
  フレーム clamp が関節ごとの実速度上限で整形する。`from_actuated_limits` が URDF の実速度を
  per-joint に流す（未収載関節は scalar 既定にフォールバック）。これで速い関節（肩 37）に不要な
  低速クランプをかけず、遅い関節（足首 9）は実上限で抑える。位置・速度・トルクが全て per-joint に。

## [0.37.0] - 2026-06-04

実データ深掘り（実 URDF 慣性テンソル取り込み, pre-alpha）。sim の bone 慣性は capsule 形状から
MuJoCo が自動算出する軸対称近似（棒）で、実機の三軸非対称な分布（胴体は太い箱で慣性 2-5 倍）を
表せなかった。実 URDF の `<inertial>` テンソルを canonical bone へ集約して MJCF の explicit
`<inertial>` に使う capability を追加した。

⚠️ **opt-in**: 実慣性は物理的に正しいが PPO tracking baseline（capsule 慣性で調整済み）を不安定化
させるため、既定 morphology は capsule のまま（controller 再チューニングは別途）。`inertia_tensors`
を設定するか URDF-import 経由で実慣性 sim を使える。

### Added
- `urdf_import.canonical_inertia_tensors` / `parse_link_inertia_tensors`: 各 link の慣性テンソルを
  世界 COM 最近傍の canonical bone へ割当て、剛体合成（並進・回転＋平行軸の定理）で bone ごとの
  (質量, COM=親 joint 相対, 世界軸 fullinertia) にまとめる。root link は pelvis ハブへ明示ルート。
- `RobotMorphology.inertia_tensors` フィールド + `build_mjcf` の explicit `<inertial>` 対応
  （あれば capsule 近似でなく実テンソル。総質量へスケールし宣言＝実質量を保存、退化 bone は floor）。
- `g1.G1_INERTIA_TENSORS` / `h1.H1_INERTIA_TENSORS`: 実 URDF 由来の bone 慣性定数（数値のみ,
  license-safe, opt-in）。`urdf_to_morphology` は自動で inertia_tensors を取り込む。
  `test_real_g1_urdf` が実 URDF からの算出値・MuJoCo body_inertia との一致を検証。

## [0.36.0] - 2026-06-04

実データ深掘り（joint limit × 質量スレッドの交点, pre-alpha）。sim のトルク判定・クランプは単一
スカラー（G1 80 / H1 160 N·m）で全関節を扱っており、強い関節（膝~139）と弱い関節（足首~35）を
区別できなかった。v0.31 で取り込んだ **per-joint actuator トルク上限**を sim へ配線した。

### Fixed
- **sim certificate のトルク判定を per-joint 負荷率へ**（`robotdance_sim.mujoco_backend`）:
  `torque_ratio` を「max(各関節の重力保持トルク ÷ その関節の実 actuator 上限)」に変更。旧スカラーは
  律速する弱い関節（足首）の負荷を 2-3 倍過小評価していた（実測: G1 0.117→0.239, H1 0.203→0.627）。
- **tracking env のトルククランプを per-DOF へ**（`robotdance_sim.tracking_env`）: 各 ball joint の
  3 DOF にその関節の実 actuator 上限を割り当て、弱い関節へ非現実的な大トルクを通さない。

### Added
- `RobotMorphology.joint_torque_limit(name)`: 実 per-joint トルク上限（無ければ sim_defaults スカラー）。
- `simulate_certificate(..., torque_limit=X)` 明示時は全関節へその scalar を強制（旧挙動・対比用）、
  未指定なら per-joint。

## [0.35.0] - 2026-06-04

実データ深掘り（質量スレッドの締め, pre-alpha）。v0.34 の質量分布取り込みで実 H1 URDF 総質量が
**~59kg**（SimDefaults の 47 は実機より 26% 過小）と判明したのを受け、各 embodiment の
`sim_defaults.total_mass` を実 URDF 総質量へ補正した（G1 35→34.13, H1 47→59.34）。

### Fixed
- **embodiment の total_mass を実 URDF 総質量へ**（`robotdance_unitree.g1` / `h1`）: G1 34.13kg /
  H1 59.34kg。宣言質量＝実 URDF 質量になり、PD ゲイン・逆動力学トルク・ZMP 判定が実機質量で
  駆動する。H1 は kd=10 のまま PD-only で安定を維持（survival 全フレーム・upright 1.0 を実測確認）。

### Changed
- `test_mjcf_total_mass_is_conserved` を sim_defaults（実 URDF 総質量）駆動に変更（ハードコード
  35/47 を撤廃）。`test_real_g1_urdf` に sim_defaults.total_mass＝実 URDF 総質量の一致検証を追加。

## [0.34.0] - 2026-06-04

実データ深掘り（質量スレッド再開, pre-alpha）。sim の質量分布は v0.29 以降 Winter 人体計測比を
**全ロボットに**適用していたが、これは実ロボットには誤りだった—実 G1/H1 は股・膝アクチュエータで
**脚が最重量（脚 ~53-58% > 胴体 ~29%）**で、Winter 人体（胴体 ~58%/脚 ~32%）とは逆。実 URDF の
`<inertial>` から実機の質量分布を取り込んで置き換えた。

### Fixed
- **sim の質量分布を実 URDF inertial 由来へ**（`robotdance_sim.mjcf` / `robotdance_unitree`）:
  `RobotMorphology.mass_distribution` を追加し、`build_mjcf` は実分布があればそれ、無ければ
  Winter プライアへフォールバック（Σ=1 再正規化で総質量は厳密保存）。実機の脚優位な分布が
  sim の COM / ZMP / 重力トルクに反映される。

### Added
- `urdf_import.canonical_mass_distribution` / `parse_link_inertials` / `link_world_frames`:
  各 link の世界 COM を最近傍の canonical bone へ割当て・左右対称化して canonical 19-joint の
  質量分布（Σ=1）と総質量を算出する。
- `g1.G1_MASS_FRACTION` / `h1.H1_MASS_FRACTION`: 実 URDF 由来の質量分布定数（数値のみ,
  license-safe）。`test_real_g1_urdf` が実 URDF からの算出値と一致を検証。
- `import-urdf` CLI が取り込んだ実質量分布（脚/胴体比）を表示。実 H1 URDF 総質量は ~59kg と判明。

## [0.33.0] - 2026-06-04

実データ深掘りの締め（pre-alpha）。v0.32 で実 URDF limit から `SafetyLimits` を作る primitive を
追加したが、それを使っていたのは demo だけで、**本番 ROS2 ランタイム（`serve` / motion server）は
依然 generic 既定（±π）で guard を構築**していた。この最後の配線を埋め、実 limit が
「embodiment 記述 → 最終 gate → 本番ランタイム」まで一貫して流れるようにした。

### Added
- `serve --urdf <path>` / `robotdance_motion_server --urdf <path>`: 実 URDF の joint limit で
  safety guard を構築して再生する（dry-run / ROS2 配信の両経路）。
- `safety_guard.build_safety_limits(urdf=None, ...)`: serve / motion server / CLI が共有する
  SafetyLimits builder。URDF 指定で実 actuator limit から、無指定で generic 既定を返す
  （`robotdance_unitree` への依存は遅延 import）。

## [0.32.0] - 2026-06-04

実データ深掘りの継続（pre-alpha）。v0.31 で embodiment の joint limit を実 URDF 由来にしたが、
その実 limit が**最終 gate の safety guard には届いておらず**、guard は依然 generic（位置 ±π /
速度 12 / トルク 60）で全関節を一律にクランプしていた（膝を ±π＝逆屈可で通してしまう）。
この配線ギャップを埋め、実 actuator limit から `SafetyLimits` を構築できるようにした。

### Fixed
- **safety guard が実 URDF の per-actuator limit を消費できる**（`robotdance_ros2.safety_guard`）:
  `SafetyLimits.from_actuated_limits()` を追加。`parse_actuated_limits` 等の出力（actuator 名 →
  position/velocity/torque）から、位置・トルクは per-joint の実値、速度は最も厳しい min（保守）で
  包絡線を構築する。これで膝の逆屈コマンドが実下限（≈-0.087 rad）へクランプされる（generic ±π は素通し）。

### Added
- `demo-joint-safety --urdf <path>`: 実 URDF の joint limit で guard を構築し、膝の逆屈コマンドが
  実下限へクランプされる様子を実演する（generic との対比）。
- `SafetyLimits.from_actuated_limits` の `position_margin`（位置 limit に安全余裕 rad を取る）。

## [0.31.0] - 2026-06-04

実データ深掘りの継続リリース（pre-alpha）。embodiment の joint limit が ±3.14 rad の placeholder
だった（実機の膝は屈曲のみ・足首は狭レンジ・トルクは膝139/腕25 N·m と桁違い、なのに全関節 ±3.14・
速度 12・トルク 60 を一律出力していた）欠陥を、**実 G1/H1 URDF 由来の per-joint limit** へ置き換えた。
1 canonical ball joint に複数 DOF が対応するため envelope 集約（位置=最広レンジ、速度/トルク=最も厳しい
min）し、actuator の無い合成関節（torso 連鎖・toe）だけ placeholder に残す。数値定数のみ埋め込み
（mesh/URDF 非同梱, license-safe）で、URDF が無い CI でも既定 embodiment が実 limit を報告する。

### Fixed
- **RD-Embodiment の joint_limits を実 URDF 由来へ**（`robotdance_retarget.embodiment` /
  `robotdance_unitree.urdf_import` / `g1` / `h1`）: `RobotMorphology` に `per_joint_limits` を追加し、
  `to_rd_embodiment()` は actuator がある関節は実値、合成関節のみ placeholder を出力。実機の事実が
  そのまま出る（G1 膝 [-0.087, 2.880]・トルク 139、H1 肩 yaw は 4.45rad で ±3.14 を超過、等）。

### Added
- `urdf_import.parse_actuated_limits` / `canonical_joint_limits`: URDF の revolute 関節 limit を
  読み、canonical 関節へ envelope 集約する。`urdf_to_morphology` が自動で取り込む。
- `g1.G1_JOINT_LIMITS` / `h1.H1_JOINT_LIMITS`: 実 URDF 由来の canonical 関節 limit 定数。
  `test_real_g1_urdf` が実 URDF からの算出値と完全一致を検証（drift 検出）。
- `import-urdf` CLI が取り込んだ実 joint limit の関節数を表示。
- README hero asset「Human → Humanoid」横並び GIF と生成スクリプト
  [`scripts/render_human_vs_robot_gif.py`](scripts/render_human_vs_robot_gif.py)。**同一の合成ダンス**を、
  左は canonical 19-joint 人間スケルトン（matplotlib）、右は actuator-space IK で実 G1 の 23 関節角へ
  retarget した実メッシュ（pybullet）として描き、同期フレームを横連結する。人間側は合成 RD-MIR、
  メッシュは render（再配布ではない）で license-safe。

### Changed
- CI の actions を Node 24 対応版へ更新（`actions/checkout@v4→v6`, `actions/setup-python@v5→v6`）。
  GitHub が 2026-06-16 から Node 20 actions を Node 24 へ強制するため、事前に追従（パッケージ不変）。

## [0.30.0] - 2026-06-04

実データ深掘りの継続リリース（pre-alpha）。v0.29 でバランス判定を凸包化した流れで、**支持多角形が
足の横幅を無視していた**欠陥を実測で特定・修正。支持点を ankle/toe の 2 点（＝幅ゼロの前後線分）
から、実 sim の足 box 幅に基づく**実フットプリント矩形**へ拡張し、`support_margin` を足幅の二重計上
から純粋なスラックへ実データ根拠で縮小した。

### Fixed
- **支持多角形を実フットプリント矩形に**（`robotdance_sim.mujoco_backend`）: 旧来は接地足を ankle/toe の
  2 点だけで表現しており（両点が同 y ＝幅ゼロの前後線分）、足の横幅（実 sim の foot box 幅 0.08m）を
  無視していた。特に**片足支持では横幅ゼロになり横バランスが評価不能**で、margin で誤魔化していた。
  ankle→toe に直交方向へ box 半幅だけ広げた 4 隅の矩形（の凸包）を支持多角形とすることで、判定の
  幾何を実 sim の接地形状と一致させ、片足支持にも実フットプリント相当の横支持を与える。
- **support_margin を実データ根拠で 0.12→0.05 に**（`robotdance_sim.mujoco_backend`）: 旧 margin は支持
  多角形に足幅が無かった分（半幅~0.04）を含む二重計上だった。足幅を明示したので margin は ZMP 推定
  誤差＋未モデルの踵（~0.05）の純粋なスラックに縮小（実測: 安定なダンスの ZMP は足面から最大 4.4mm、
  backflip は支持外 ~92%＋airborne で確実に REJECT）。判別力が向上。

### Changed
- **足 box 寸法を名前付き定数化**（`robotdance_sim.mjcf.FOOT_BOX_HALF_LENGTH` / `FOOT_BOX_HALF_WIDTH`）:
  MJCF 生成とバランス判定が同一のフットプリント幅を単一の出所として参照する（判定と sim の幾何一致）。

### Added
- **フットプリント横幅の回帰テスト**（`tests/test_sim.py::test_foot_footprint_has_real_width_for_single_support`）:
  接地足が幅ゼロの線分でなく box 幅の矩形として支持に寄与し、片足支持でも横方向の支持を持つことを担保。

## [0.29.0] - 2026-06-04

実データ深掘りの継続リリース（pre-alpha）。MJCF の質量**分布**を人体計測（Winter）の実測値で
検証したところ、旧来の「質量 ∝ bone 長」配分が**物理的に破綻**していた（H1 で腕32% > 胴体19% と、
腕が胴体より重い非物理分布）。Winter のセグメント質量比で配分し直した結果、**質量バグが隠して
いた 2 件目のバグ＝バランス判定の支持多角形近似の欠陥**が露呈した。連鎖して両方を修正。

### Fixed
- **質量分布を人体計測（Winter）ベースに**（`robotdance_sim.mjcf`）: 質量 ∝ bone 長は無根拠で、
  長い腕 bone に過大質量を与えていた（H1: 腕32% > 胴体19%）。Winter, D.A. *Biomechanics and Motor
  Control of Human Movement* のセグメント質量比（Σ=1 に正規化）で各部位へ配分。胴体~56% / 腕~11% /
  脚~33% と人体計測（58/10/32）に一致し、胴体重心の物理的に妥当な分布になった。総質量保存も維持。
  比は body proportion 不変なので G1/H1 共通（総質量のみ機種差）。※実機 URDF の `<inertial>` そのもの
  ではない人体近似プライアだが、bone 長比より桁違いに妥当（実 URDF 慣性取り込みは将来 spec）。
- **バランス判定を実支持多角形（凸包）に**（`robotdance_sim.mujoco_backend._zmp_in_support`）: 旧実装は
  各足点を半径 margin の円で覆う近似で、足点集合との最近傍距離 ≤ margin を支持とみなしていた。これは
  脚幅が広い機種で破綻し、**両足の中間（＝バランスの取れた ZMP の定位置）がどの足点からも margin 超に
  なり、正しく立っているのに転倒判定**された（H1: 股幅0.52m, 足点 y=±0.26 → 中心 ZMP が全フレーム
  支持外）。足点の凸包（支持多角形）内なら距離0で支持、margin は多角形外への許容、と正しい意味に修正。
  この欠陥は旧質量バグ（脚が過剰に重く COM が足元へ大きく振れて偶然円内に入る）に隠蔽されていた。

### Added
- **質量分布の回帰テスト**（`tests/test_sim.py::test_mass_distribution_is_trunk_heavy_anthropometric`）:
  G1/H1 で胴体が最重量かつ腕 < 胴体（人体計測相当）を担保。
- **支持多角形判定の回帰テスト**（`tests/test_sim.py::test_zmp_support_uses_polygon_not_per_foot_circles`）:
  広い脚幅でも中心 ZMP を支持と認め、明らかに外側の ZMP は支持外、単一足（線分）でも正しく判定。

## [0.28.0] - 2026-06-04

実データ深掘りの継続リリース（pre-alpha）。MJCF 質量モデルを実測検証したところ、**生成される
robot の総質量が宣言 `total_mass` と一致していなかった**（pelvis ハブ 3kg + 足 box 0.6kg を
total_mass に加算せず上乗せ → G1 宣言35kg を実38.6kg(+10.3%) で sim）バグと、**certify 経路が
v0.27 の `SimDefaults` 配線から漏れて G1 値（35kg/80N·m）を H1 にも流用していた**バグを発見・修正。
いずれも「宣言と実体がズレる／既定が特定機種に隠れ固定される」隠れ取り違えの一種で、v0.27 と同じ
失敗クラスを別経路で塞いだ。

### Fixed
- **MJCF 総質量の保存**（`robotdance_sim.mjcf.build_mjcf`）: pelvis ハブ・足 box の固定質量を
  bone 長比配分の予算から差し引くよう変更。生成 MJCF の総質量が宣言 `total_mass` に厳密一致する
  （G1: 38.60→35.000kg, H1: 50.60→47.000kg, Δ=0）。PD ゲインや逆動力学トルクは実質量に依存する
  ため、宣言質量＝実質量でないと「35kg 用に調整したつもりが 38.6kg を制御」という取り違えが起きる。
- **certify のトルク上限・質量を embodiment 由来に**（`robotdance_sim.mujoco_backend.simulate_certificate`）:
  旧実装は `total_mass=35` / `torque_limit=80`（G1値）をハードコードしており、H1（47kg/160N·m）の
  certify でも G1 のトルク上限で torque_ratio を算定していた（v0.27 で `SimDefaults` を導入したのに
  この経路だけ配線漏れ）。既定を `None` とし `morphology.sim_defaults` から取得するよう変更。

### Added
- **MJCF 質量保存の回帰テスト**（`tests/test_sim.py::test_mjcf_total_mass_is_conserved`）: G1/H1 の
  生成 MJCF 総質量が宣言 total_mass に一致することを MuJoCo 実ロードで担保。
- **certify トルク上限配線の回帰テスト**（`tests/test_sim.py::test_certify_uses_embodiment_torque_limit_not_g1_default`）:
  H1 の既定 certify が H1 のトルク上限(160)を使い、G1 値(80)固定ではないことを担保。

## [0.27.0] - 2026-06-04

実データ深掘りの継続リリース（pre-alpha）。v0.26 で G1/H1 の体格を実寸化した結果、**より背が高く
（1.66m）手足の長い H1 が、G1 専用に調整された TrackingEnv 既定ゲインでは PD 振動で横倒れする**
バグを発見・修正。embodiment 固有の sim 既定値（質量・PD ゲイン）を morphology に紐付け、
「既定値が特定機種に隠れ調整される」事故構造そのものを解消した。

### Added
- **embodiment 固有の sim 既定値 `SimDefaults`**（`robotdance_retarget.embodiment`）: 機種ごとの概算
  質量・関節 PD ゲイン（`total_mass` / `kp` / `kd` / `torque_limit`）を `RobotMorphology.sim_defaults`
  として保持。G1=（35kg, kp150, kd6, tl80）、H1=（47kg, kp200, **kd10**, tl160）。`TrackingEnv` は
  caller が明示しない限りこの morphology 既定を採用する（G1 既定の H1 への誤流用を防止）。
- **H1 sim 安定性の回帰テスト**（`tests/test_tracking.py::test_h1_pd_baseline_is_stable_with_morphology_defaults`）:
  H1 が morphology 既定だけで複数ダンス振幅で PD-only 追従し、全フレーム生存かつ upright>0.9 を保つこと、
  および H1 の既定 kd / 質量が G1 より大きいことを担保（合成ダンス＋プロキシで完結 → CI 実行）。

### Changed
- **TrackingEnv の質量・PD ゲインを morphology 由来に変更**（`robotdance_sim.tracking_env`）: 旧来は
  G1 専用の固定既定（mass=35, kp=150, kd=6）をハードコードしており、H1 に流用すると **kd 不足で PD が
  振動して横倒れ**していた（H1: G1 既定で 15/29 転倒・upright 0.18 へ崩壊）。`total_mass` / `kp` / `kd` /
  `torque_limit` の既定を `None` とし、未指定時は `morphology.sim_defaults` から取得するよう変更。
  これにより H1 は明示ゲイン無しでも全ダンス振幅で安定（29/29 生存・upright 1.0・pose RMSE 0.21〜0.23）、
  G1 は従来挙動を維持。real-data validation で「実寸化が露呈させた機種取り違えバグ」を closed。

## [0.26.0] - 2026-06-04

実機忠実度の節目リリース（pre-alpha）。「広く v0 を積む」から方針転換し、**実 Unitree URDF で深掘り検証**
した結果、G1/H1 の体格データが実機と 26〜33% 乖離していたバグを発見・修正。あわせて実機メッシュが
踊る README hero GIF と H1 actuator IK 対応を追加した。

### Added
- **実 Unitree メッシュの hero GIF（README 刷新, §6）**（`scripts/render_robot_gif.py`,
  `assets/readme/g1_dance.gif` / `many_humanoids_mesh.gif`）: 棒人間スケルトンに代え、**公式 G1/H1 の
  実メッシュ**が RobotDance の **actuator-space IK 関節角**で踊る GIF を README hero に採用。pybullet
  headless（TinyRenderer, GPU 不要）で地面影付きレンダリング。G1 単体 + G1|H1 横並び（実寸ゆえ身長差が
  見える）。メッシュ/URDF は repo 非同梱（render のみ, © Unitree unitree_ros BSD-3）。
- **actuator-space IK を H1 等に一般化**（`robotdance_retarget.actuator_retarget`）: `link_map` /
  `robot_name` 引数を追加し、G1 以外の URDF（H1 = 19 DOF, `H1_LINK_MAP`）でも IK retarget 可能に
  （実 H1 URDF で収束: IK err 0.098m, pre-clamp limit 違反 <0.5%）。既定は従来どおり G1。

### Changed
- **G1 morphology を実 URDF 実寸に修正（real-data validation, §4.2）**（`robotdance_unitree.g1`）:
  v0 の手書き G1 プロキシは**実機と乖離していた**（公式 g1_23dof URDF と比較し nominal_height
  1.120m vs 実 1.291m = **17cm 過小**、bone 長**平均相対誤差 ~26%**、肩/手首は ~80% 過大）。これを
  公式 URDF 由来の実寸 canonical rest pose に更新し、nominal_height 1.291m・bone 誤差 **~0%** に一致させた
  （関節オフセット＝寸法の事実のみ採用、mesh/URDF 本体は非同梱 = license-safe）。あわせて実 G1 URDF で
  actuator-space IK が収束（IK mean err 0.062m, joint-limit 違反 0）することを確認。
- **H1 morphology も実 URDF 実寸に修正（real-data validation）**（`robotdance_unitree.h1`,
  `urdf_import.H1_LINK_MAP`）: G1 と同じ手書き由来の H1 プロキシも**実機と乖離していた**（公式 h1.urdf と
  比較し bone 長**平均相対誤差 ~33%**、特に hip 幅・肩を誤っていた）。公式 URDF 由来の実寸 rest pose に
  更新し誤差 **~0%**（nominal 1.664m）に一致させた。H1 は腕が肘止まりで wrist link が無いため
  `build_rest_pose` が前腕を合成する（G1 等 wrist 在 URDF には非影響）。`H1_LINK_MAP` を追加。
- **TrackingEnv の PD ゲイン再チューニング**（`robotdance_sim.tracking_env`）: 上記 G1 実寸化（背が高く
  COM が上がった）に伴い、旧 kp=60 では関節 PD が実寸 G1 を支えきれず PD-only baseline が転倒していた
  ため、既定を **kp=150 / kd=6** に更新（PD-only が gentle 参照で 59/59 生存に回復）。tracking 系テストを
  実寸 G1 で再 green 化。これは「近似プロキシに合わせて隠れていた調整が、実寸化で露呈した」例。

### Added
- **実 G1 URDF 回帰テスト**（`tests/test_real_g1_urdf.py`）: 公式 g1_23dof URDF がローカルに在る場合のみ
  実行し（`ROBOTDANCE_G1_URDF` か既知パスから探索、無ければ skip = CI 非破壊）、簡略 morphology が
  実 URDF 寸法（nominal 1cm 以内 / bone MAE < 1cm）と一致し、actuator IK が収束することを検証する。

[0.30.0]: https://github.com/rsasaki0109/RobotDance/releases/tag/v0.30.0
[0.29.0]: https://github.com/rsasaki0109/RobotDance/releases/tag/v0.29.0
[0.28.0]: https://github.com/rsasaki0109/RobotDance/releases/tag/v0.28.0
[0.27.0]: https://github.com/rsasaki0109/RobotDance/releases/tag/v0.27.0
[0.26.0]: https://github.com/rsasaki0109/RobotDance/releases/tag/v0.26.0

## [0.25.0] - 2026-06-04

spec の質向上の節目リリース（pre-alpha）。自由 dict だった RD-MIR `semantics` を action_label /
style_tag / captions / segments（連続行動）として構造化し、後方互換を保ったまま型付きの行動・
テキスト情報を載せられるようにした。

### Added
- **RD-MIR semantics の構造化（§3）**（`robotdance_core.semantics`, rd-mir schema）: これまで自由 dict
  だった `RdMir.semantics` を **action_label / style_tag / captions / segments（連続行動
  `[{label, start_t, end_t}]`）/ source_dataset** として spec 化。`Semantics` / `Segment` pydantic +
  `build_semantics(...)`（正規化・segments の label 必須を検証）/ `validate_semantics` / `segment_labels`。
  rd-mir schema の `semantics` に構造を文書化（**後方互換のため `additionalProperties: true` を維持**＝
  旧来の自由 dict もそのまま適合）。BABEL adapter が frame_ann を標準 `segments` として出力するよう更新。
  pydantic のみで **CI 検証**。

[0.25.0]: https://github.com/rsasaki0109/RobotDance/releases/tag/v0.25.0

## [0.24.0] - 2026-06-04

公開準備の節目リリース（pre-alpha）。ディレクトリ内 artifact の Model Card 一括生成 + 索引
（CARDS_INDEX.md）と CITATION.cff の充実で、データセット/モデルを責任を持って公開する土台を整えた。

### Added
- **Cards index + CITATION 充実（§7, 公開準備）**（`cards-index`,
  `robotdance_core.model_card.card_for_artifact` / `render_cards_index`）: ディレクトリ内の
  RD-MIR/Motion/Policy artifact を種別自動判別して **Model Card を一括生成**し、**索引
  `CARDS_INDEX.md`**（type / id / license / failure_modes / summary / card リンク + license
  composition）を出力する。`card_for_artifact(path)` で 3 種の dispatch を集約（`model-card` も使用）。
  `CITATION.cff` に version / date-released と keywords（text-to-motion / sim-to-real /
  reinforcement-learning 等）を追加。純 Python で **CI 検証**。

[0.24.0]: https://github.com/rsasaki0109/RobotDance/releases/tag/v0.24.0

## [0.23.0] - 2026-06-04

データ品質の節目リリース（pre-alpha）。motion-embedding 重複除去を**任意の RD-MIR コレクション**
（HumanML3D/BABEL/Motion-X 等の import 出力）へ汎用化し、入口を跨いだ near-duplicate 除去を可能にした。

### Added
- **汎用 near-duplicate 除去の text-motion 拡張（§4.1）**（`robotdance_motion.dedupe`,
  `import-babel --dedupe`, `dedupe-dir`）: これまで manifest 駆動ビルド（`build-dataset --dedupe`）に
  限定されていた motion-embedding 重複除去を、**任意の RD-MIR コレクション**（HumanML3D/BABEL/Motion-X
  等の import 出力）に適用できるよう汎用化。`dedupe_mirs(mirs, threshold)` は I/O を持たない純粋関数で、
  near-duplicate を cosine 類似度でグループ化し各グループ 1 本（最長フレーム）を代表に残す。
  `import-babel --dedupe` で保存前に除去、`dedupe-dir <dir> [--move]` で既存ディレクトリの
  `*.rdmir.json` を一括 dedupe（重複は `duplicates/` へ移動）。numpy のみで **CI 検証**。

[0.23.0]: https://github.com/rsasaki0109/RobotDance/releases/tag/v0.23.0

## [0.22.0] - 2026-06-04

見せ場（デモ品質）の節目リリース（pre-alpha）。RD-MIR の caption をオーバーレイし、text 検索の
top-k を類似度バッジ付きモンタージュ GIF に描けるようにし、「言葉でモーションを探す」を視覚的に
伝えられるようにした。

### Added
- **Viewer 強化: caption overlay + 検索結果モンタージュ（§6）**（`robotdance_viewer.skeleton_view`,
  `search-text --gif`）: デモ品質（見せ場）を向上。
  - `render_gif` に **caption オーバーレイ**（None なら `semantics.action_label` を自動）+ 下部メタ行
    （license_state / fps / frames）を追加。`view` が自動で caption 付き GIF を出す。
  - `render_search_montage(query, results)` を追加: **text 検索の top-k** をクエリをタイトルに・
    類似度（cosine）をバッジにして横並び描画。`search-text --gif` で出力。
  - `render_side_by_side` に図全体の `title`（suptitle）を追加。
  - `_mir_caption` は依存なしで **CI 検証**、描画は matplotlib/imageio を importorskip。

[0.22.0]: https://github.com/rsasaki0109/RobotDance/releases/tag/v0.22.0

## [0.21.0] - 2026-06-04

入口精度向上の節目リリース（pre-alpha）。HMR/Motion-X の **SMPL betas** で骨格の個体差を反映し、
HMR ツールの native 出力（.pkl/.pt 等）を直接取り込めるようにした。

### Added
- **HMR betas（shape conditioning）+ native loader（§4.1）**（`robotdance_perception.hmr`,
  `robotdance_data.smpl`）: HMR/Motion-X が回帰した **SMPL betas** で rest offset を shape-conditioning
  し、骨格の個体差（身長 β0 / 体幅 β1）を first-order で反映する（`smpl.py` の `fk_smpl_body` /
  `smpl_poses_to_canonical` が betas 引数を受理）。**真の SMPL blend shapes ではなく粗い線形プロキシ**
  （model file 不要 = license-safe）。`from_gvhmr` / `from_4dhumans` / `load_hmr_npz` / Motion-X(322 次元の
  betas)が betas を自動利用。あわせて **native loader** `load_hmr_file(path)` を追加し、
  `.npz/.npy/.pkl/.pt` を読んで dict 構造から GVHMR/4DHumans/汎用を自動判別（`from_dict`）。
  `import-hmr` が native 形式を受理。numpy/scipy のみで **CI 検証**。

[0.21.0]: https://github.com/rsasaki0109/RobotDance/releases/tag/v0.21.0

## [0.20.0] - 2026-06-04

全体価値を一望できる節目リリース（pre-alpha）。RobotDance の主要スタックを **1 コマンド**で繋ぐ
end-to-end ショーケース（`demo-pipeline`）を追加した。20 番目のメジャー pre-alpha リリースで、
入口（動画/データ）→ RD-MIR → retarget → 物理検証 → 学習/配布 → 説明責任カードが一本道で通る。

### Added
- **End-to-end pipeline ショーケース（§6）**（`demo-pipeline`, `robotdance_core.pipeline`）: RobotDance の
  主要スタックを **1 コマンド**で繋ぐ統合デモ — `(data/synth) → RD-MIR → retarget → sim_certificate
  → [tracking policy 学習 + RD-Policy/ONNX export] → Model/Policy Card`。各成果物（RD-MIR / RD-Motion /
  RD-Policy）と説明責任カードを出力ディレクトリに書き出す。`--mir` で既存 RD-MIR（import-* の出力）を
  入口に、`--train-policy` で policy 学習+export まで実行。重い段は**依存が無ければ graceful にスキップ**
  （sim=mujoco / policy=torch+mujoco）。core 段（RD-MIR→retarget→card）は依存なしで **CI 検証**。

[0.20.0]: https://github.com/rsasaki0109/RobotDance/releases/tag/v0.20.0

## [0.19.0] - 2026-06-04

text-motion データ網羅の節目リリース（pre-alpha）。HumanML3D / BABEL に続き **Motion-X**
（whole-body）adapter を追加し、主要な text-motion データセットを一通りカバーした。

### Added
- **Motion-X adapter（§4.1, whole-body text-motion）**（`import-motionx`,
  `robotdance_data.motionx`）: SMPL-X（whole-body）+ 記述文の大規模 text-motion データセットを
  canonical RD-MIR 化する。標準の **322 次元**表現から **body 66 次元（root_orient + pose_body）+
  trans** を取り出し、既存の skeleton-first SMPL FK で canonical 化（手/顔/betas は未使用）。322 /
  66 次元 / [T,22,3] 形式を自動判別。記述文を `semantics` に格納。numpy のみで **CI 検証**。
  データセット本体は同梱せず利用者が各自取得（license_state=research_only）。

[0.19.0]: https://github.com/rsasaki0109/RobotDance/releases/tag/v0.19.0

## [0.18.0] - 2026-06-04

説明責任を policy まで広げた節目リリース（pre-alpha）。RD-MIR / RD-Motion に続き **RD-Policy** の
Model Card 生成を追加し、生成 → 学習 → 配布の各 artifact が統一インターフェース（`model-card`）で
lineage・license・failure modes・safety limits（policy は I/O Contract・Weights も）を備えた。

### Added
- **Policy Card（§7, model_card 拡張）**（`model-card <policy.rdpolicy.json>`,
  `robotdance_core.model_card.build_policy_card`）: RD-Policy の Model Card を生成する。Motion/MIR
  カードに加え、policy 向けに **I/O Contract**（observation components / action space・dim・
  base_actuated / control / architecture）と **Weights**（format / ref / sha256）セクションを追加。
  lineage（reference motions → training → weights）・license・failure_modes（policy 保持分を優先、
  無ければ手法 registry で補完）・safety_limits（下流 safety guard 強制）を出力。`model-card` CLI は
  artifact 種別（mir / motion / policy）を自動判別。純 Python で **CI 検証**。

[0.18.0]: https://github.com/rsasaki0109/RobotDance/releases/tag/v0.18.0

## [0.17.0] - 2026-06-03

実機橋渡しの節目リリース（pre-alpha）。予定フィールドのみだった **RD-Policy** spec を v0 で確定し、
学習済み tracking policy を I/O 規約・安全制約・weights 参照付きの配布 artifact（+ONNX）として
export できるようにした。spec 一式（RD-MIR/Manifest/Embodiment/Motion/**Policy**）が出揃った。

### Added
- **RD-Policy spec 確定 + policy export（§3/§4.5）**（`export-policy`, `validate policy`,
  `robotdance_core.rd_policy`, `robotdance_models.policy_export`）: 学習済み motion policy の **配布
  artifact**（`.rdpolicy`）の v0 JSON Schema + pydantic モデルを確定（これまで予定フィールドのみ）。
  policy の **I/O 規約**（observation/action の dim・space・base_actuated）・**アーキテクチャ**・
  **学習来歴**・**安全制約**・**weights 参照**（format/ref/sha256、本体は非埋め込み）を 1 つの spec 適合
  JSON にまとめる。`export-policy` が tracking policy checkpoint(.pt) から生成し、任意で **ONNX**
  （決定論方策の mean、onnxruntime で実行可能 = 実機ランタイム橋渡し）も書き出す。failure_modes は
  model_card registry を再利用、safety_limits は下流の joint-space safety guard で強制する旨を記録。
  weights を埋め込まず参照する **license/容量 safe** 設計。spec/モデル/assembly は依存なしで **CI 検証**
  （ONNX/checkpoint export は torch）。`validate` の対象に `motion` / `policy` を追加。

[0.17.0]: https://github.com/rsasaki0109/RobotDance/releases/tag/v0.17.0

## [0.16.0] - 2026-06-03

text-motion データ入口拡充の節目リリース（pre-alpha）。AMASS/AIST++ に続き、**HumanML3D / BABEL**
adapter を追加し、データ入口に「動き + 自然文記述 / 行動ラベル」の実データが入った。contrastive
検索・text2motion の実データ化への布石。

### Added
- **HumanML3D / BABEL adapter（§4.1, text-motion データ入口）**（`import-humanml3d` / `import-babel`,
  `robotdance_data.humanml3d` / `robotdance_data.babel`）: text 注釈付き motion データセットを
  既存の **skeleton-first SMPL 経路**で canonical RD-MIR 化する。
  - **HumanML3D**: 前処理済み SMPL joint 位置 [T,22,3]（`new_joints/*.npy`）を canonical 19-joint に
    変換し、記述文（`texts/*.txt` の `caption#tokens#start#end` を parse）を `semantics` に格納。
    smpl.py に位置版マップ `smpl_joints_to_canonical` を追加。
  - **BABEL**: AMASS へ付与された**行動ラベル**（sequence/frame-level）を読み、対応 AMASS を
    `load_amass_npz` で読んで `semantics`（action_label / babel_labels / babel_segments）に付与。
    AMASS .npz が見つからない entry はスキップ。
  どちらも numpy のみで **CI でも検証**。データセット本体は同梱せず利用者が各自取得（license_state は
  research_only）。HumanML3D は frame 正規化が近似・betas 未使用。Motion-X は今後。

[0.16.0]: https://github.com/rsasaki0109/RobotDance/releases/tag/v0.16.0

## [0.15.0] - 2026-06-03

sim スタック汎用化の節目リリース（pre-alpha）。物理検証 backend を **pluggable** にする
`SimBackend` 抽象 + registry を追加し、MuJoCo 一択から Isaac Lab/Genesis 等へ差し替え可能な
土台を整えた（MuJoCo 参照実装 + Isaac Lab scaffold、本体は license/容量 safe のため非同梱）。

### Added
- **Sim backend 抽象 + registry（§4.3）**（`sim-backends`, `validate-sim --backend`,
  `robotdance_sim.backend`）: sim_certificate の物理 backend を **pluggable** にする。`SimBackend`
  契約（`passed` / `verdict` / `backend` / `metrics` / `reasons` の certificate dict）と registry
  （`register_backend` / `get_backend` / `backend_status` / `certify(..., backend=...)`）を提供。
  MuJoCo を参照実装として登録し、**Isaac Lab** を contract のみの scaffold として登録（未インストール
  なら導入手順を示す明示エラー）。dispatch 時に契約キーを検証。registry/contract/scaffold は依存なしで
  **CI でも検証**。**Isaac Lab 本体（NVIDIA Omniverse 依存・大容量）は同梱・実行しない**（license/容量
  safe）— 実装は利用者環境で contract に従って行う。Isaac Lab/Genesis 実装・GPU 並列 sim は今後。

[0.15.0]: https://github.com/rsasaki0109/RobotDance/releases/tag/v0.15.0

## [0.14.0] - 2026-06-03

生成スタック深化の節目リリース（pre-alpha）。causal token prior（続きを作る）に、**双方向
denoiser**（全体の文脈で埋める/直す）と **長尺生成**（sliding-window）を加え、motion foundation
model スタックが「生成 + 補間/ノイズ除去 + 長尺」を備えた。

### Added
- **Motion denoiser / in-betweening + 長尺生成（§4.2 拡張）**（`train-denoiser` / `demo-denoise`,
  `robotdance_models.denoiser`, torch）: token prior の causal 生成に対し、**双方向 Transformer** を
  **masked token modeling**（BERT 風）で学習し、`MotionDenoiser.denoise()` が尤度の低い外れトークンを
  mask→双方向充填で**ノイズ除去**、`inbetween()` が両端を残し中間を埋めて**補間（in-betweening / 中割り）**
  する。foundation model スタックが「生成（prior）+ 補間/除去（denoiser）」を備える。合成 corpus で
  masked-token 復元精度がランダム（~0.8%）を大きく上回る（~50%）。あわせて prior の
  **長尺生成**を明示化（`MotionGenerator.generate(length=...)` が seq_len 超を sliding-window 自己回帰で
  生成、`demo-generate --length`）。256 frames でも jitter ~0.035 と滑らか。torch tests は CI で skip。
  生成物は物理的に妥当とは限らず retarget → sim_certificate を必ず通す。長尺学習・betas・実データ規模は今後。

[0.14.0]: https://github.com/rsasaki0109/RobotDance/releases/tag/v0.14.0

## [0.13.0] - 2026-06-03

入口品質の可視化の節目リリース（pre-alpha）。抽出 adapter（MediaPipe / HMR）を共通 ground-truth に
対し定量比較する **extraction benchmark** を追加し、retarget/sim の leaderboard に続いて
**video→RD-MIR の質**も MPJPE / PA-MPJPE / PCK / jitter 等で評価できるようになった。

### Added
- **Extraction benchmark（§4.1）**（`benchmark-extraction`, `robotdance_benchmarks.extraction`）:
  video→RD-MIR の抽出 adapter（MediaPipe / HMR(4DHumans/GVHMR) 等）を **共通 ground-truth に対し
  定量比較**する評価ハーネス。指標は **MPJPE**（root-relative）/ **PA-MPJPE**（Umeyama 相似整列後）/
  **PCK@5cm·10cm** / **MPJVE**（速度誤差）/ **jitter**（時間的滑らかさ）/ **bone-length MAE**。
  `extraction_metrics(gt, pred)` / `compare_extractions(gt, {name: pred})` → MPJPE 昇順の
  leaderboard（CSV/Markdown）。純 numpy・画像不要で **CI でも検証**。同梱デモは合成 GT に
  MediaPipe 風（奥行きノイズ+jitter）/ HMR 風（骨長近似+時間的に滑らか）の劣化を加えて harness を
  実演する（実 adapter 比較は実 video の抽出結果と GT を渡して行う。**実モデルの精度主張ではない**）。

[0.13.0]: https://github.com/rsasaki0109/RobotDance/releases/tag/v0.13.0

## [0.12.0] - 2026-06-03

実機安全 gate 完成の節目リリース（pre-alpha）。joint-space safety guard に**アクチュエータ トルク
limit** を追加し、実機コマンド直前の最終 gate が **位置・速度・加速度・トルク** まで揃った。
`sim_certificate`（物理的妥当性）→ safety guard（機構的安全）の二段構えが完成。

### Added
- **Joint-space safety guard にトルク limit を追加（§5.6）**（`robotdance_ros2.safety_guard`）:
  v0.9 の位置/速度/加速度クランプに加え、**アクチュエータのトルク上限**を強制する。**必要トルク
  ≈ I_eff·θ̈ + 重力負荷** を per-joint 実効慣性モデルで見積もり、トルク上限から加速度上限
  (τ_max−grav)/I_eff を導いて加速度クランプに織り込む（過大加速度＝過大トルクを抑制）。
  `SafetyLimits` に `enforce_torque_limit` / `max_joint_torque` / `joint_torque_limits` /
  `joint_inertia` / `default_joint_inertia` / `joint_gravity_load` を追加。`clamp_joint_trajectory`
  の report に raw/safe 推定最大トルク・`torque_violation_frames` を追加。`demo-joint-safety` が
  推定トルクを raw 7199 → safe 40 N·m に整形する様子を表示。**CI でも検証**。v0 は粗い実効慣性
  モデルの計画段階 guard で、完全な剛体動力学でもモータ電流飽和の代替でもない（電流は τ/Kt で
  モータ制御器が担う）。

[0.12.0]: https://github.com/rsasaki0109/RobotDance/releases/tag/v0.12.0

## [0.11.0] - 2026-06-03

説明責任の節目リリース（pre-alpha）。10 リリースで機能が揃ったのを受け、成果物の **Model / Motion
Card** 生成を追加し、data lineage・license・**failure modes（既知の v0 限界）**・safety limits を
構造化して出力できるようにした。「v0 の限界を正直に明示する」方針をツール化し、責任ある公開・利用の
土台を整えた。

### Added
- **Model / Motion Cards（§7）**（`model-card`, `robotdance_core.model_card`）: RD-MIR / RD-Motion から
  責任ある公開・利用に必要な情報を構造化したカードを生成する — **data lineage**（source → extractor →
  retarget → sim_certificate → control_mode の連鎖）・**license**（state から再配布/商用の可否を導出、
  RD-Motion は source RD-MIR から継承）・**failure modes**（使用手法のシグナルから curated registry を
  引いて既知の v0 限界を列挙）・**safety limits**（sim_certificate の verdict/thresholds、actuator なら
  joint-space safety guard 必須を明記）・metrics。`build_mir_card` / `build_motion_card` /
  `render_markdown`（Markdown）+ 機械可読 JSON、collection 用 `license_composition`。純 Python で
  **CI でも検証**。dataset 全体の license firewall 内訳は既存の Data Bill of Materials（DATA_CARD.md）が担う。

[0.11.0]: https://github.com/rsasaki0109/RobotDance/releases/tag/v0.11.0

## [0.10.0] - 2026-06-03

動画入口の高品質化の節目リリース（pre-alpha）。MediaPipe（2D→近似 3D landmark）に加え、
**HMR（Human Mesh Recovery）adapter** を追加し、4DHumans / GVHMR が回帰した SMPL 出力を
canonical RD-MIR に取り込めるようになった。オクルージョン・奥行き・world-grounded な global
trajectory に強い入口が揃い、in-the-wild 動画の品質がパイプライン全体に効く。

### Added
- **HMR adapter（§4.1, 4DHumans / GVHMR → RD-MIR）**（`import-hmr`, `robotdance_perception.hmr`）:
  HMR モデルが画像から回帰した **per-frame SMPL パラメータ**（global_orient / body_pose / transl）を、
  既存の **skeleton-first SMPL FK**（`robotdance_data.smpl`）で canonical 19-joint RD-MIR に変換する。
  MediaPipe（2D→近似 3D landmark）よりオクルージョン・奥行き・**world-grounded な global trajectory**
  に強い動画入口を追加。共通 core `hmr_smpl_to_mir(...)` + 出力構造別 entry point
  `from_gvhmr(dict)`（axis-angle・world）/ `from_4dhumans(dict)`（rotation-matrix・23 body joint）/
  汎用 `load_hmr_npz(path)`。axis-angle と rotation-matrix は形状から自動判別（一致を検証）。
  **モデル weight / SMPL body model file は同梱・実行しない**（license-safe, in-the-wild 由来は
  `license_state="unknown"`）。numpy/scipy のみで **CI でも検証**。v0 は skeleton-first（近似 rest
  offset・betas/shape 未使用）で、特定モデル版 pin ではなく文書化された出力構造に対する検証。
  native `.pkl`/`.pt` 直接ロード・multi-person・betas 反映は今後。

[0.10.0]: https://github.com/rsasaki0109/RobotDance/releases/tag/v0.10.0

## [0.9.0] - 2026-06-03

実機安全の節目リリース（pre-alpha）。Safety Guard に **actuator（関節）空間の limit enforcement**
を追加し、生成/retarget/tracking した関節角列を**実機コマンド直前に位置/速度/加速度へクランプ**
できるようになった。`sim_certificate`（物理的妥当性）の先にある最終 gate が揃い、実機への橋渡しが
一段進んだ。なお摂動頑健性は調査の上で誇張を避けて見送った（下記 Notes）。

### Added
- **Joint-space safety guard（§5.6）**（`demo-joint-safety`, `robotdance_ros2.safety_guard`）:
  Safety Guard に **actuator（関節）空間の limit enforcement** を追加。actuator-space IK /
  tracking policy が出す関節角列を、実機コマンド直前に **位置 limit・速度・加速度**へクランプする。
  これは `sim_certificate`（物理的妥当性）の**先**にある最終 gate で、コマンド自体を機構的に安全な
  範囲へ整形する。`SafetyGuard.filter_frame()` が `MotionFrame.joint_angles` を stateful にクランプし、
  `clamp_joint_trajectory(angles, dt, limits, names)` が軌道全体を一括整形して report（raw/safe の
  最大速度・加速度、各 limit 発火フレーム数）を返す。`SafetyLimits` に `max_joint_speed` /
  `max_joint_accel` / `joint_position_limits` を追加。純 numpy・ROS2 非依存で **CI でも検証**。
  v0 は位置/速度を厳密 bound・加速度は best-effort。トルク/電流 limit は実機モデルが入る Phase 4+。

### Notes
- **摂動頑健性（perturbation robustness）を調査したが見送り**: v0 の両足接地＋剛 PD モデルは静的な
  転倒閾値が鋭く、閾値超過のトルク shove は 1 制御ステップで倒れ切る（残差トルクでは間に合わず、
  回復には踏み出し＝接地変更が必要だが planted 参照＋追従報酬がそれを許さない）。domain randomization
  で学習しても摂動下 survival は PD と同等（PD 0.43 / RL 0.43, 改善 +0.00）。**正直な RL>PD の優位が
  存在しない**ため、誇張を避けて本機能は出さず、より確実な価値のある joint-space safety guard を採用した。

[0.9.0]: https://github.com/rsasaki0109/RobotDance/releases/tag/v0.9.0

## [0.8.0] - 2026-06-03

制御スタック汎化の節目リリース（pre-alpha）。v0.7 の RL tracking 方策は**単一参照専用**だったが、
本リリースで **1 つの方策が運動スイート全体を追従**できるよう汎化した（reference-conditioned）。
「1 運動 = 1 方策」から「1 方策が運動に応じて追従を変える」へ進み、汎用 tracking policy の足場が整った。

### Added
- **Multi-motion tracking policy**（`train-tracking --suite` / `demo-track-multi`,
  `robotdance_sim.MultiTrackingEnv` + `robotdance_models.train_multi_tracking_policy`, torch/mujoco）:
  v0.7 の tracking 方策は**単一参照専用**だった。本機能は参照スイート（gentle/normal/fast dance + idle）を
  保持する `MultiTrackingEnv` を追加し、エピソードごとに参照を round-robin で切り替えて **1 つの方策が
  複数運動を追従**できるよう汎化する。観測に「次フレームへの姿勢誤差」が入る reference-conditioned 設計
  なので、方策は運動に応じて追従を変える。`TrackingPolicy.rollout(idx)` で各参照を指定ロールアウトできる。
  合成 4 運動スイートで **全運動 survival 100%**（1 方策）を達成。PPO コアを `_ppo_train` に抽出し
  単一・複数で共有。v0 は依然 baseline 足場で、PD 超えの tracking 精度・摂動頑健性・実機転移は今後。

[0.8.0]: https://github.com/rsasaki0109/RobotDance/releases/tag/v0.8.0

## [0.7.0] - 2026-06-03

制御スタックの節目リリース（pre-alpha）。学習スタック（検索・トークン化・生成）に続き、
**生成/retarget したモーションを物理シミュレーション上で倒れずに追従する** RL tracking policy の
ベースラインを追加。設計書 §4.5（Sim-to-Real Policy Stack）の起点で、「物理的に妥当か」
（sim_certificate）の判定の先にある「**実際にバランスを取って動かせるか**」を扱う最初の足場。

### Added
- **RL tracking policy baseline**（`train-tracking` / `demo-track`,
  `robotdance_sim.tracking_env` + `robotdance_models.tracking_policy`, torch/mujoco）:
  参照運動を **MuJoCo forward 物理シミュレーション上で倒れずに追従**する方策を小型 **PPO** で学習する。
  学習スタック（検索・トークン化・生成）の次にある**制御スタック**の最初の足場で、設計書 §4.5
  （Sim-to-Real Policy Stack）に当たる。base（pelvis）は free joint で**非駆動**、駆動 DOF は関節空間 PD
  （参照 qpos へアンカー）+ 方策の残差トルク（`qfrc_applied[6:]`）→ 「バランスを取りながら追う」ことが
  本質的に必要（underactuated）。報酬 = 姿勢追従 + 直立 + 生存 − 制御コスト、転倒で終了。
  `TrackingPolicy.rollout()` が物理ロールアウトを RD-Motion（`control_mode="policy"`）として返し、
  viewer / sim_certificate / ROS2 の既存パイプラインに流せる。合成 gentle 参照で **survival 100%**・
  pose RMSE ~0.37 を達成。**v0 は baseline 足場**: 短い feasible クリップでは関節 PD だけで概ねバランス
  するため、残差 PPO は **PD を壊さず追従する**ことを学ぶ（PD 超えの tracking 精度・多様 motion 汎化・
  摂動頑健性・AMP/敵対報酬・実機転移は今後）。近似質量ゆえ実機保証ではない。

[0.7.0]: https://github.com/rsasaki0109/RobotDance/releases/tag/v0.7.0

## [0.6.0] - 2026-06-03

Text-to-motion の節目リリース（pre-alpha）。**言葉でモーションを生成**できるようになり、
v0.4 の contrastive 検索（text → 既存 motion）と合わせて **text↔motion 双方向**が揃った。
生成物は schema 適合の RD-MIR なので、そのまま retarget → 物理検証 → ROS2 の安全パイプラインに流せる。

### Added
- **Text-conditioned motion generation**（`train-text2motion` / `generate-text`,
  `robotdance_models.text2motion`, torch）: token prior を**テキスト特徴で条件付け**し、
  **caption → モーション**を生成する（"a person doing a backflip" → バックフリップ）。
  `text.py`（テキスト特徴）+ `tokenizer.py`（VQ-VAE）+ `prior.py`（生成）を 1 本に繋ぐ集大成。
  caption の特徴を系列先頭の conditioning トークンとして与え、causal Transformer が沿うトークン列を
  生成。合成 corpus で next-token 精度 ~94%、生成は caption の action 群に応じて変化
  （backflip→energy ~0.26 / standing still→~0.02）。生成物は schema 適合の RD-MIR で、
  retarget → sim_certificate の安全パイプラインに流せる。v0 は語彙・新規 caption 汎化が限定的。

[0.6.0]: https://github.com/rsasaki0109/RobotDance/releases/tag/v0.6.0

## [0.5.0] - 2026-06-03

Motion generation の節目リリース（pre-alpha）。離散トークン化（VQ-VAE）と生成 prior が揃い、
**モーションを離散トークンに圧縮 → 生成・補完**できるようになった。検索・整合・トークン化・生成と、
設計書 §4.2 Motion Intelligence Stack の中核が一段落。

### Added
- **Motion VQ-VAE トークナイザ**（`train-tokenizer` / `demo-tokenizer`,
  `robotdance_models.tokenizer`, torch）: motion window を時間方向に 4× 圧縮した潜在列に符号化し、
  **EMA codebook** の最近傍コードに量子化して**離散トークン列**にする。decoder で復元。
  `MotionTokenizer.encode(mir) -> tokens` / `decode_to_mir(tokens) -> RD-MIR` で、1 本のモーションが
  「離散トークンの列」になり、将来の autoregressive 生成・補完・テキスト条件付け（VLA 接続）の足場になる。
  **データ依存初期化 + dead-code 復活**で codebook collapse を回避し、合成 corpus で再構成 MSE
  0.055→0.0007・codebook 使用率 ~49%・再構成 RMSE ~0.03（正規化空間）を達成。
  v0 は符号化⇄復号のみ（トークン列の生成 prior は別途・residual VQ / 可変長は今後）。
- **Motion token prior / 生成・補完**（`train-prior` / `demo-generate`,
  `robotdance_models.prior`, torch）: VQ-VAE トークン列上で **GPT 風 causal Transformer** を
  next-token 予測で学習し、`MotionGenerator.generate()` で BOS から**新規モーション生成**、
  `complete()` で prefix を保持した**補完**を行う。tokenizer と prior が揃って生成が動く。
  合成 corpus で next-token 精度 ~92%・生成は滑らか（jitter ~0.03）。v0 は多様性/新規性が限定的・
  テキスト条件付けは今後。**生成物は物理的に妥当とは限らず** retarget → sim_certificate で検証する。

[0.5.0]: https://github.com/rsasaki0109/RobotDance/releases/tag/v0.5.0

## [0.4.0] - 2026-06-03

Motion intelligence の節目リリース（pre-alpha）。手作り embedding → 学習 encoder に続き、
**自然文でモーションを意味検索**できる contrastive text-motion を追加。**テキスト → モーション**の
橋渡しが通り、検索・条件付け・VLA 接続の足場ができた。

### Added
- **Contrastive text-motion 検索**（`train-text-motion` / `search-text`,
  `robotdance_models.contrastive`, torch）: motion encoder（masked modeling の再利用）と
  決定的ハッシュ n-gram テキスト特徴（`robotdance_models.text`, 依存なし）を**共有埋め込み空間**に
  射影し、(motion, caption) を **multi-positive InfoNCE** で整合させる。学習後は
  `embed_text("a person doing a backflip")` と `embed_motion(rd_mir)` が同じ単位球面に乗り、
  自然文でモーションを意味検索できる。合成 corpus で **caption→motion を action 群レベル top-1 100%**
  で引け、学習に無い言い回し（"flipping backwards through the air" → backflip 等）にも汎化する。
  v0 は事前学習言語モデルなし・合成 corpus（実キャプション規模・CLIP 等への差し替えは今後）。

[0.4.0]: https://github.com/rsasaki0109/RobotDance/releases/tag/v0.4.0

## [0.3.0] - 2026-06-03

実機パスの節目リリース（pre-alpha）。**人間動画/mocap → 実 G1 の実関節角 → ROS2 /joint_states 配信**
までが繋がり、実機コマンドの直前（RViz で実 G1 メッシュが動く）まで到達した。

### Added
- **ROS2 `/joint_states` 配信 + RViz launch**: actuator-space IK の実関節角を `sensor_msgs/JointState`
  で配信。`robot_state_publisher` + 実 URDF と合わせると RViz で**本物の G1 メッシュ**が動く
  （`robotdance_ros2/launch/g1_rviz.launch.py` 付属）。MotionFrame が関節角を運び、safety guard を通る。
- **アクチュエータ空間 retarget**（`retarget-ik` / `robotdance_retarget.actuator_ik`, torch）:
  実 URDF の微分可能 FK を構成し、勾配 IK で**実 G1 の 23 関節角**を解く。出力 `.rdmotion` の
  `joint_rotations` に実機（ROS2/SDK2）が command できる joint trajectory を格納。IK 位置誤差が
  実 G1 の限られた DOF での追従性を示す（dance ~0.07m / backflip ~0.16m）。参照 IK であり
  バランス policy ではない（動的実現可能性は sim_certificate が別途検証）。

[0.3.0]: https://github.com/rsasaki0109/RobotDance/releases/tag/v0.3.0

## [0.2.0] - 2026-06-03

データ系統・学習・実機忠実度の深化リリース（pre-alpha）。v0.1.0 のエンドツーエンド骨格に、
実データ adapter・学習 encoder・実 URDF 取り込みを積み増した。

### Added
- **AIST++ dataset adapter**（`load_aist_pkl`）: ダンス mocap（SMPL .pkl, 60fps）→ canonical RD-MIR。
  AMASS と同じ skeleton-first 経路（SMPL model file 不要）。`dataset://aist/...` で manifest 指定可。
- **dataset 重複除去**（`build-dataset --dedupe`）: motion embedding で near-duplicate を検出し
  各グループ 1 本だけ残す。除去内訳は Data Bill of Materials に記録。
- **実 URDF 取り込み**（`import-urdf` / `robotdance_unitree.urdf_import`）: 実機 URDF の zero-config FK で
  リンク世界位置を求め、canonical 19-joint rest を**実寸**から構築（Unitree G1 23dof で nominal_height≈1.29m、
  実 bone 長）。retarget/sim が実物寸法で動く。torso 連鎖・toe は合成、質量・アクチュエータ空間 retarget は今後。
- **学習 motion encoder**（masked motion modeling, `[learn]` extra / torch）: 小型 Transformer を
  マスク再構成で自己教師あり学習（`train-encoder`）。手作りと同じ前処理・`embed` interface で
  `MotionIndex(embed_fn=...)` に差し込める（`demo-motion-map --checkpoint`）。合成 corpus で
  loss 低下・クラス分離を実証（v0 は基盤提供であり手作り baseline 超えは主張しない）。

[0.2.0]: https://github.com/rsasaki0109/RobotDance/releases/tag/v0.2.0

## [0.1.0] - 2026-06-03

最初の公開リリース（pre-alpha）。空リポジトリから、人間動画/mocap をヒューマノイド運動資産へ
変換するエンドツーエンドのパイプライン骨格を、specs-first・license-safe・sim-first の方針で構築した。

> ⚠️ v0.1 は近似形態プロキシ・近似質量・特徴量ベース embedding を用いる **pre-alpha**。
> **実機保証ではない**。各パッケージ README に v0 の限界を明記している。

### Specs（仕様は実装より偉い）
- **RD-MIR**（中核 motion IR）・**RD-Manifest**・**RD-Embodiment**・**RD-Motion** の v0 JSON Schema
- RD-Policy は予定フィールドを定義

### データ入口（3 系統 → 同一 canonical RD-MIR に合流）
- **合成モーション**生成（dance / backflip、決定的・権利クリーン）
- **実動画 → RD-MIR**: MediaPipe Pose（world landmarks → canonical 19-joint）+ temporal smoothing + 2D overlay
- **mocap → RD-MIR**: AMASS ローダ（SMPL FK, skeleton-first, SMPL model file 不要）

### データ基盤
- **RD-Manifest license firewall**（unknown / derived 非許可は派生 motion を遮断）
- **Data Bill of Materials**（どの source が・どの権利で・公開されたか）

### Retarget & 物理検証
- **kinematic retarget**（direction-preserving FK + morphology normalization + ground clamp）
- **multi-embodiment**: Unitree G1 / H1（registry で拡張可能）
- **MuJoCo 物理検証**（sim_certificate）: COM/ZMP バランス・滞空・解析的重力トルク → PASS/REJECT

### Motion intelligence
- **motion embeddings**（位置/向き/スケール不変）+ 類似検索 + near-duplicate 検出 + **Motion Map**（PCA）
- **benchmark**（motion × robot leaderboard, CSV/Markdown）

### ランタイム
- **ROS2 runtime**（Jazzy）: Safety Guard（certificate gate / 速度クランプ / 転倒検知 / E-stop）
  + Motion Server + rclpy ノード（MarkerArray/SafetyState 配信）。core は ROS2 非依存で完全テスト可

### ビューア / デモ
- 3D skeleton GIF・multi-panel side-by-side・原動画 overlay・Motion Map・PASS/REJECT バッジ
- 「映える」デモ: Many Humanoids / Unsafe Rejected / Motion Map / smoothing

### 開発基盤
- 22+ CLI サブコマンド（`robotdance ...`）
- GitHub Actions CI（ruff + pytest, 69 passed）/ CITATION.cff / CONTRIBUTING（拡張ポイント）
- Apache-2.0

### v0.1 の既知の限界（今後）
- ロボットは簡略 kinematic プロキシ（実 URDF / 実機慣性は今後）
- embedding は手作り特徴量（学習 encoder で差し替え予定）
- ROS2 は Cartesian 空間 / sim-first（joint-space limit clamp・実機 bridge は安全レビュー後）
- dataset adapter は AMASS のみ（AIST++ / Motion-X / 重複除去統合は今後）

[0.1.0]: https://github.com/rsasaki0109/RobotDance/releases/tag/v0.1.0
