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
- [x] viewer: 3D skeleton GIF + multi-panel side-by-side（original video overlay / quality timeline は今後）
- [x] local video pipeline: **MediaPipe Pose** で decode → detect → world landmarks → canonical RD-MIR
      （HMR / temporal smoothing / multi-person tracking は今後）
- [ ] seed dataset adapters: AIST++ (annotations), AMASS (skeleton-first), HumanML3D, BABEL (optional)
- [ ] license firewall: raw video 非再配布, manifest rights 検証, dataset build report, card template

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
- [ ] 実 URDF・実機慣性の取り込み（v0 は近似慣性・ball-joint 近似）
- [ ] retarget benchmark: foot slip, fall rate, tracking error, torque saturation proxy（指標は揃った、leaderboard 化は今後）

**Acceptance:** 1 つの RD-MIR から G1/H1 `.rdmotion` を生成し human/robot を side-by-side ✅、
MuJoCo で safe/rejected を判定し sim_certificate に記録 ✅（実機再生は ROS2 runtime 後）。

### Phase 3 — Motion Embeddings & Learning Stack
- [ ] motion tokenizer（contact-aware, root/body 分離, 可変長）
- [ ] motion encoder（masked modeling, contrastive video/text-motion, quality-aware）
- [ ] motion retrieval UI（upload → extract → similar 検索 → motion map）
- [ ] motion foundation model baseline（denoising / completion / 短い生成 / 条件付き）
- [ ] RL tracking baseline（G1/H1 tracking task, AMP/ASE-style prior option）
- [ ] model cards（data lineage, license composition, failure modes, safety limits）

### Phase 4 — ROS2 Runtime, Real Robot Path, Ecosystem
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
