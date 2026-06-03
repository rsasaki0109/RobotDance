# robotdance_data

manifests, source adapters, dataset builder, dedupe, license firewall — URL/manifest 駆動のデータパイプライン。raw video は再配布しない。

## 実装状況

| module | 役割 |
| --- | --- |
| `smpl.py` | SMPL/SMPL-H body skeleton の FK（**SMPL model file 不要**の skeleton-first）+ canonical 19 へのマップ |
| `amass.py` | `load_amass_npz(path) -> RdMir`。AMASS の SMPL pose を canonical RD-MIR 化 |
| `aist.py` | `load_aist_pkl(path) -> RdMir`。AIST++（ダンス, 60fps）の SMPL pose を canonical RD-MIR 化 |
| `manifest.py` | RD-Manifest 読込・schema 検証 + **license firewall**（`evaluate(manifest) -> FirewallDecision`） |
| `dataset.py` | manifest 駆動ビルダー。firewall + **motion embedding 重複除去** を通し、**Data Bill of Materials** を出力 |

```bash
robotdance build-dataset manifests.json --data-root /path/to/data --dedupe -o build/
# → build/<clip>.rdmir.json と build/DATA_CARD.md（Data Bill of Materials）
```

source_uri は `dataset://<name>/<相対パス>` 形式（`<name>` = `amass` / `aist`）で dataset とローカル位置を指定。
`--dedupe` は motion embedding の near-duplicate を検出し、各グループ 1 本だけ残す（残りは BOM に
`near-duplicate of <id>` として記録）。

## ライセンスファイアウォール

- `license_declared=unknown` または `derived_motion_allowed=false` → 派生 motion を**書き出さない**。
- 公開可の clip には manifest の権利フラグから `license_state`（redistributable/trainable/research_only…）を付与。
- raw source（動画・mocap）は再配布しない。manifest（URL/再構築手順）と派生 motion のみ扱う。
- ビルドのたびに **Data Bill of Materials** を出力し、どの source が・どの権利で・公開されたかを明示。

> ⚠️ **v0 注意:** SMPL FK の rest offset は近似（正確な shape-conditioned joint regressor は未使用）。
> retarget は direction-preserving なので下流に影響は小さい。AIST++ / Motion-X 等の adapter、
> 重複除去（perceptual/motion hash）は今後。実 AMASS は登録制で同梱せず、利用者が各自取得する。
