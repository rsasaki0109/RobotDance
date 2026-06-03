# RobotDance Roadmap

OSS としての勝ち筋は「READMEを見た瞬間に Star したくなること」。
研究的な正しさより先に、`local video → 3D motion → G1 sim side-by-side` を出す。

## 公開順序（リリース軸）

| Version | テーマ | 主要 deliverable |
| --- | --- | --- |
| v0.1 | Video to G1 Sim | local video input, AIST++ demo, 2D/3D pose adapter, RD-MIR export, web viewer, G1 sim retarget, quality report |
| v0.2 | Dataset Builder | RD-Manifest, URL-based reproducible build, YouTube CC adapter, license firewall, dataset card, WebDataset/HF export |
| v0.3 | Motion Embeddings | motion encoder, retrieval demo, duplicate detection, motion map, action clustering |
| v0.4 | Humanoid Retarget Benchmark | G1/H1 benchmark, IK/sim metrics, leaderboard, `.rdmotion` format |
| v0.5 | ROS2 Runtime (experimental) | ROS2 motion server, safety guard, Unitree bridge, sim-first replay |
| v1.0 | Stable Specs | RD-MIR / RD-Manifest / RD-Embodiment 安定化, model zoo, governance |

## 実装 workstream（Phase 軸 = §8）

### Phase 1 — Core Spec & Legal-safe Visual MVP
- [x] specs v0 ドラフト（RD-Manifest / RD-MIR / RD-Embodiment / RD-Motion / RD-Policy）
- [x] RD-MIR Python データモデル（pydantic）+ canonical skeleton + 合成モーション生成
- [x] core CLI: `validate` / `synth` / `view`（`build` / `extract` / `score` は今後）
- [x] viewer: 3D skeleton GIF + multi-panel side-by-side + **原動画 2D overlay**（`overlay`）
- [x] local video pipeline: **MediaPipe Pose** で decode → detect → world landmarks → canonical RD-MIR
      + **temporal smoothing**（Savitzky-Golay, jitter 指標）。HMR / multi-person tracking は今後
- [x] seed dataset adapters: **AMASS / AIST++（skeleton-first, SMPL FK）** ✅。HumanML3D / BABEL / Motion-X は今後
- [x] license firewall: raw 非再配布, manifest rights 検証, **dataset build report + Data Bill of Materials** ✅
- [x] 重複除去: **motion embedding による near-duplicate 検出**を dataset build に統合（`--dedupe`）✅

**Acceptance:** local video で RD-MIR + web 可視化を出力。原動画と side-by-side。repo に著作権 raw video を含めない。

### Phase 2 — Unitree G1/H1 Retargeting & Simulation
- [x] RD-Embodiment: 汎用 `RobotMorphology` 抽象 + G1 / H1 config（v0 簡略 kinematic プロキシ / generic limits）。実 URDF・SDK2 写像は今後
- [x] retargeting engine v0: 汎用 `retarget(mir, morphology)` = direction-preserving FK + morphology normalization + ground clamp
- [x] `.rdmotion` artifact + `rd-motion.schema.json` 確定（v0）
- [x] viewer: human ↔ robot side-by-side / **multi-embodiment**（`view-pair` / `demo-g1` / `demo-multi`）
      → "Same motion, many humanoids"（§6.2 Demo 2）を実現
- [x] simulation backend: **MuJoCo** で sim_certificate を埋める（逆動力学トルク + COM/ZMP バランス + 滞空）
      → safe dance=PASS / backflip=REJECT（§6.2 Demo 4, `demo-safety`）。Isaac Lab backend は今後
- [ ] contact-preserving IK / joint limit optimizer（v0 は方向コピーのみ）
- [x] **実 URDF 取り込み**（`import-urdf`）: zero-config FK で実リンク寸法から RobotMorphology を構築
      （Unitree G1 23dof で nominal_height≈1.29m）。実機慣性・アクチュエータ空間 retarget は今後
- [x] retarget/sim benchmark + **leaderboard**（`benchmark`）: PASS率・bone_cos・foot_sliding・balance・torque を
      motion × robot で CSV/Markdown 集計（docs/benchmark/）。extraction benchmark（実動画）は今後

**Acceptance:** 1 つの RD-MIR から G1/H1 `.rdmotion` を生成し human/robot を side-by-side ✅、
MuJoCo で safe/rejected を判定し sim_certificate に記録 ✅（実機再生は ROS2 runtime 後）。

### Phase 3 — Motion Embeddings & Learning Stack
- [x] motion embedding v0（特徴量ベース: root-relative + scale 正規化 + yaw 整列）+ `MotionIndex`
      （類似検索 / near-duplicate 検出 / PCA 2D Motion Map）→ §6.2 Demo 3（`demo-motion-map`）
- [x] 学習 motion encoder v0: **masked motion modeling**（小型 Transformer, PyTorch）。手作りと同じ
      前処理・`embed` interface で `MotionIndex` に差し込める（`train-encoder` / `demo-motion-map --checkpoint`）。
      合成 corpus で loss 低下・クラス分離を実証（実データ規模・contrastive 拡張は今後）
- [x] **contrastive text-motion v0**（`train-text-motion` / `search-text`）: motion encoder + ハッシュ
      n-gram テキスト特徴を共有空間で multi-positive InfoNCE 整合 → 自然文でモーション意味検索
      （合成 corpus で caption→motion を action 群 top-1 100%、未知の言い回しにも汎化）。
      事前学習言語モデル・実キャプション規模・**video**-text-motion は今後
- [x] **motion tokenizer / VQ-VAE v0**（`train-tokenizer` / `demo-tokenizer`）: motion window を
      EMA codebook で離散トークン列に符号化・復元（4× 時間圧縮・collapse 回避・再構成 RMSE ~0.03）。
      生成 prior（トークン列の言語モデル）・residual VQ・可変長は今後
- [x] **motion token prior / 生成・補完 v0**（`train-prior` / `demo-generate`）: VQ-VAE トークン列上の
      GPT 風 causal Transformer（next-token 精度 ~92%）で新規生成・補完
- [x] **text-conditioned 生成 v0**（`train-text2motion` / `generate-text`）: token prior をテキスト特徴で
      条件付け → caption → モーション生成（"a backflip" → バックフリップ, action 群に応じ生成変化）。
      語彙・新規 caption 汎化は今後
- [ ] motion foundation model（denoising / 長尺）、RL tracking baseline

#### Phase 3 詳細（当初計画）
- [x] **motion tokenizer**（VQ-VAE, EMA codebook, 4× 時間圧縮）✅ — contact-aware / root-body 分離 / 可変長は今後
- [ ] motion encoder（masked modeling, contrastive video/text-motion, quality-aware）
- [ ] motion retrieval UI（upload → extract → similar 検索 → motion map）
- [x] **motion foundation model baseline v0**: VQ-VAE + token prior で **completion / 短い生成** ✅
      （denoising / テキスト条件付きは今後）
- [ ] RL tracking baseline（G1/H1 tracking task, AMP/ASE-style prior option）
- [ ] model cards（data lineage, license composition, failure modes, safety limits）

### Phase 4 — ROS2 Runtime, Real Robot Path, Ecosystem
- [x] ROS2 runtime v0（Jazzy）: messages 契約 + **Safety Guard**（certificate gate / 速度クランプ /
      転倒検知 / E-stop / speed scaling）+ **Motion Server**（.rdmotion → 安全フレーム配信）+
      rclpy ノード（MarkerArray/SafetyState 配信, E-stop 購読）。core は ROS2 非依存で完全テスト可
- [x] **アクチュエータ空間 retarget**（`retarget-ik`）: 実 URDF の微分可能 FK + 勾配 IK で実 G1 の
      23 関節角を出力（ROS2/SDK2 が command できる joint trajectory）。実機への本丸の橋渡し
- [x] アクチュエータ角の **ROS2 `/joint_states` 配信** + RViz launch（robot_state_publisher + 実 URDF で
      本物の G1 メッシュが動く）。実機の一歩手前
- [ ] custom .msg の colcon パッケージ化 / ros2_control 連携 / Unitree SDK2 bridge / 実機再生プロトコル
- [ ] plugin ecosystem templates / governance

#### Phase 4 詳細（当初計画）
- [ ] ROS2 messages（MotionClip/Frame/Latent, RobotMotionPlan, MotionQuality, SafetyState, PolicyAction）
- [ ] motion server（load `.rdmotion`, stream, pause/resume, speed scaling, phase control）
- [ ] safety guard（joint/velocity/accel/torque guard, fall detector, E-stop, sim certificate check）
- [ ] Unitree bridge（G1/H1, SDK2/ROS2, state feedback, command publish, logging）
- [ ] real robot evaluation protocol（tethered low-speed, operator checklist, required sim pass）
- [ ] plugin ecosystem templates + governance

> ROS2 primary target は **Jazzy**（LTS, ~2029-05）。既存 SDK 互換のため Humble を secondary support。

## MVP でやらないこと

TikTok/Instagram scraper、実機での派手なダンス、巨大 foundation model、SMPL 必須化、全ロボット対応、
end-to-end VLA。いずれも軸がぼやける / 法務・安全リスクが高いため後回し。
