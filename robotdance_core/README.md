# robotdance_core

schemas, validators, metadata, CLI, config — RobotDance の中核。RD-MIR/RD-Motion 等のスキーマ検証と共通基盤。

## 主なモジュール

- `rd_mir.py` / `rd_motion.py` — RD-MIR / RD-Motion の pydantic データモデル（spec 適合 / load/save）。
- `skeleton.py` — canonical 19-joint skeleton（embodiment 非依存の標準）。
- `synthetic.py` — 決定的・権利クリーンな合成モーション生成（dance / backflip）。
- `cli.py` — `robotdance` CLI（validate / synth / view / retarget / sim / train-* / serve / …）。
- `rd_policy.py` — **RD-Policy** の pydantic モデル（学習済み policy の配布 artifact: I/O 規約・
  weights 参照・安全制約）。spec は `specs/rd-policy/`、export は `robotdance_models.policy_export`。
- `semantics.py` — **RD-MIR semantics の構造化（§3）**: `action_label` / `style_tag` / `captions` /
  `segments`（連続行動 `[{label, start_t, end_t}]`）/ `source_dataset` を pydantic（`Semantics` /
  `Segment`）で定義。`build_semantics(...)` で正規化 dict 化（segments は label 必須を検証）、
  `validate_semantics`。後方互換のため `RdMir.semantics` は dict のまま・schema は additionalProperties 許可。
- `model_card.py` — **Model / Motion / Policy Card 生成（§7）**: RD-MIR/RD-Motion/**RD-Policy** から
  **data lineage・license・failure modes・safety limits**（policy は **I/O Contract・Weights** も）を
  Markdown + 機械可読 JSON で出力する。failure modes は手法シグナル（extractor / retarget / sim
  backend / control_mode）から curated registry を引いて検出。dataset 全体の license firewall 内訳は
  `robotdance_data` の Data BOM が担う。

```bash
robotdance model-card g1.rdmotion.json --mir clip.rdmir.json -o MODEL_CARD.md --json card.json
robotdance model-card policy.rdpolicy.json -o POLICY_CARD.md   # RD-Policy も同じコマンドで
robotdance cards-index out/                                    # dir 内 artifact 全カード + 索引(CARDS_INDEX.md)
```

> ⚠️ v0 pre-alpha。カードは「既知の限界を正直に明示する」ためのもので、実機保証ではない。
> 詳細なアーキテクチャ上の役割は、ルートの設計方針およびロードマップ（Phase 1〜4）を参照。
