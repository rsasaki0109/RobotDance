# robotdance_core

schemas, validators, metadata, CLI, config — RobotDance の中核。RD-MIR/RD-Motion 等のスキーマ検証と共通基盤。

## 主なモジュール

- `rd_mir.py` / `rd_motion.py` — RD-MIR / RD-Motion の pydantic データモデル（spec 適合 / load/save）。
- `skeleton.py` — canonical 19-joint skeleton（embodiment 非依存の標準）。
- `synthetic.py` — 決定的・権利クリーンな合成モーション生成（dance / backflip）。
- `cli.py` — `robotdance` CLI（validate / synth / view / retarget / sim / train-* / serve / …）。
- `model_card.py` — **Model / Motion Card 生成（§7）**: RD-MIR/RD-Motion から **data lineage・
  license composition・failure modes・safety limits** を Markdown + 機械可読 JSON で出力する。
  failure modes は手法シグナル（extractor / retarget / sim backend / control_mode）から curated
  registry を引いて検出。dataset 全体の license firewall 内訳は `robotdance_data` の Data BOM が担う。

```bash
robotdance model-card g1.rdmotion.json --mir clip.rdmir.json -o MODEL_CARD.md --json card.json
```

> ⚠️ v0 pre-alpha。カードは「既知の限界を正直に明示する」ためのもので、実機保証ではない。
> 詳細なアーキテクチャ上の役割は、ルートの設計方針およびロードマップ（Phase 1〜4）を参照。
