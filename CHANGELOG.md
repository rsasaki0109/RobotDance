# Changelog

All notable changes to RobotDance are documented here.
This project adheres to [Semantic Versioning](https://semver.org/).

## [Unreleased]

### Added
- **Text-conditioned motion generation**（`train-text2motion` / `generate-text`,
  `robotdance_models.text2motion`, torch）: token prior を**テキスト特徴で条件付け**し、
  **caption → モーション**を生成する（"a person doing a backflip" → バックフリップ）。
  `text.py`（テキスト特徴）+ `tokenizer.py`（VQ-VAE）+ `prior.py`（生成）を 1 本に繋ぐ集大成。
  caption の特徴を系列先頭の conditioning トークンとして与え、causal Transformer が沿うトークン列を
  生成。合成 corpus で next-token 精度 ~94%、生成は caption の action 群に応じて変化
  （backflip→energy ~0.26 / standing still→~0.02）。生成物は schema 適合の RD-MIR で、
  retarget → sim_certificate の安全パイプラインに流せる。v0 は語彙・新規 caption 汎化が限定的。

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
