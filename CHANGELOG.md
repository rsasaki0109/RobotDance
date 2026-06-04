# Changelog

All notable changes to RobotDance are documented here.
This project adheres to [Semantic Versioning](https://semver.org/).

## [Unreleased]

### Added
- **RD-MIR semantics の構造化（§3）**（`robotdance_core.semantics`, rd-mir schema）: これまで自由 dict
  だった `RdMir.semantics` を **action_label / style_tag / captions / segments（連続行動
  `[{label, start_t, end_t}]`）/ source_dataset** として spec 化。`Semantics` / `Segment` pydantic +
  `build_semantics(...)`（正規化・segments の label 必須を検証）/ `validate_semantics` / `segment_labels`。
  rd-mir schema の `semantics` に構造を文書化（**後方互換のため `additionalProperties: true` を維持**＝
  旧来の自由 dict もそのまま適合）。BABEL adapter が frame_ann を標準 `segments` として出力するよう更新。
  pydantic のみで **CI 検証**。

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
