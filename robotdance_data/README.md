# robotdance_data

manifests, source adapters, dataset builder, dedupe, license firewall — URL/manifest 駆動のデータパイプライン。raw video は再配布しない。

## 実装状況

| module | 役割 |
| --- | --- |
| `smpl.py` | SMPL/SMPL-H body skeleton の FK（**SMPL model file 不要**の skeleton-first）+ canonical 19 へのマップ。**betas で rest offset を shape-conditioning**（身長/体幅の粗い線形プロキシ, v0） |
| `amass.py` | `load_amass_npz(path) -> RdMir`。AMASS の SMPL pose を canonical RD-MIR 化 |
| `aist.py` | `load_aist_pkl(path) -> RdMir`。AIST++（ダンス, 60fps）の SMPL pose を canonical RD-MIR 化 |
| `humanml3d.py` | **HumanML3D**（text-motion）。`load_humanml3d(npy, txt)`: 前処理済み SMPL joint 位置 [T,22,3] を canonical 化し、**記述文を `semantics` に格納** |
| `babel.py` | **BABEL**（AMASS への行動ラベル）。`babel_entry_to_mir` / `load_babel(json, amass_root)`: 注釈から AMASS を読み、**行動ラベル（seq/frame）を `semantics` に付与** |
| `motionx.py` | **Motion-X**（whole-body text-motion）。`load_motionx(npy, txt)`: SMPL-X 322 次元表現の **body 66 次元（root_orient+pose_body）+ trans** を canonical 化し記述文を `semantics` に格納（手/顔/betas は未使用） |
| `manifest.py` | RD-Manifest 読込・schema 検証 + **license firewall**（`evaluate(manifest) -> FirewallDecision`） |
| `dataset.py` | manifest 駆動ビルダー。firewall + **motion embedding 重複除去** を通し、**Data Bill of Materials** を出力 |

```bash
robotdance build-dataset manifests.json --data-root /path/to/data --dedupe -o build/
# → build/<clip>.rdmir.json と build/DATA_CARD.md（Data Bill of Materials）

# text-motion データセット（§4.1）
robotdance import-humanml3d new_joints/000.npy --text texts/000.txt -o clip.rdmir.json
robotdance import-babel babel_v1.0/train.json --amass-root /path/to/amass --limit 100 --out-dir out/
robotdance import-motionx motion/000.npy --text texts/000.txt -o clip.rdmir.json
```

source_uri は `dataset://<name>/<相対パス>` 形式（`<name>` = `amass` / `aist`）で dataset とローカル位置を指定。
`--dedupe` は motion embedding の near-duplicate を検出し、各グループ 1 本だけ残す（残りは BOM に
`near-duplicate of <id>` として記録）。**text-motion adapter（HumanML3D/BABEL/Motion-X）等の任意の
RD-MIR コレクション**にも `robotdance_motion.dedupe.dedupe_mirs(mirs)`（汎用・I/O なし）/
`import-babel --dedupe` / `dedupe-dir <dir>` で同じ重複除去を適用できる。

## ライセンスファイアウォール

- `license_declared=unknown` または `derived_motion_allowed=false` → 派生 motion を**書き出さない**。
- 公開可の clip には manifest の権利フラグから `license_state`（redistributable/trainable/research_only…）を付与。
- raw source（動画・mocap）は再配布しない。manifest（URL/再構築手順）と派生 motion のみ扱う。
- ビルドのたびに **Data Bill of Materials** を出力し、どの source が・どの権利で・公開されたかを明示。

> ⚠️ **v0 注意:** SMPL FK の rest offset は近似（正確な shape-conditioned joint regressor は未使用）。
> retarget は direction-preserving なので下流に影響は小さい。**HumanML3D** は frame 正規化が近似
> （前処理 frame を SMPL frame とみなす）、**BABEL** は AMASS .npz が見つかる entry のみ変換。
> **Motion-X** は SMPL-X の body 66 次元のみ使用（手/顔/betas は未使用）。重複除去
> （perceptual hash）は今後。実 AMASS/HumanML3D/BABEL/Motion-X は登録制で同梱せず、
> 利用者が各自取得する（license_state は research_only）。
