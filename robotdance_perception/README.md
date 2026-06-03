# robotdance_perception

pose adapters, human tracking, HMR adapters, smoothing — 2D/3D pose・人間メッシュ復元を adapter 方式で束ねる。

## 実装状況

- `mediapipe_adapter.py` — **MediaPipe Pose による local 動画 → RD-MIR**。
  `extract_motion(video) -> RdMir` が pose_world_landmarks（33点・メートル3D）を canonical
  19-joint へマップする。`mp_world_landmarks_to_canonical` は純関数で単体テスト可能。
- `hmr.py` — **HMR（Human Mesh Recovery）adapter: SMPL 出力 → RD-MIR**。4DHumans（HMR2.0/PHALP,
  rotmat）/ GVHMR（axis-angle・world-grounded）が回帰した **per-frame SMPL パラメータ**
  （global_orient / body_pose / transl）を、既存の **skeleton-first SMPL FK**（`robotdance_data.smpl`）で
  canonical 19-joint に変換する。MediaPipe（2D→近似 3D）よりオクルージョン・奥行き・world trajectory に
  強い入口。`from_gvhmr(dict)` / `from_4dhumans(dict)` / `load_hmr_npz(path)` / 共通 core
  `hmr_smpl_to_mir(...)`。axis-angle / rotation-matrix は形状から自動判別。

```python
from robotdance_perception.hmr import from_gvhmr, load_hmr_npz
mir = from_gvhmr(gvhmr_result)             # GVHMR の出力 dict → RD-MIR（world trajectory）
mir = load_hmr_npz("clip_hmr.npz")         # 汎用 .npz 交換フォーマット → RD-MIR

from robotdance_perception.mediapipe_adapter import extract_motion
mir = extract_motion("my_clip.mp4")        # → RD-MIR（license_state="unknown"）
```

```bash
robotdance import-hmr clip_hmr.npz --source gvhmr -o clip.rdmir.json   # SMPL(.npz) → RD-MIR
```

- 座標変換: MediaPipe world（x:右, y:下, z:手前負, 腰原点）→ canonical（x:前, y:左, z:上）= `(-z, x, -y)`。
- HMR: SMPL frame（x:左, y:上, z:前）→ canonical = `(z, x, y)`（`robotdance_data.smpl` と共通）。
- モデル（Google 配布 Apache-2.0 の `.task`）は `~/.cache/robotdance/models/` へ自動 DL（`ROBOTDANCE_POSE_MODEL` で上書き可）。

> ⚠️ **ライセンス:** 入力動画の権利はユーザー責任。アダプタは動画を再配布せず、抽出 RD-MIR の
> `license_state` は既定で `"unknown"`（source 未確認 → 派生 motion を公開しない）。
> 検証は landmark→canonical の単体テスト + scikit-image の astronaut（NASA, public domain）実写で行う。
> **HMR adapter（v0）:** モデル weight / SMPL body model file は**同梱・実行しない**（HMR 推論は
> ツール側）。本 adapter は出力 SMPL パラメータ → canonical の変換のみを担い、numpy/scipy だけで
> CI 検証する。skeleton-first（近似 rest offset・betas/shape は未使用）で、特定モデル版に pin した
> 精度検証ではなく**文書化された出力構造**に対する検証。native `.pkl`/`.pt` の直接ロード・
> multi-person tracking・betas 反映・temporal smoothing 強化は今後。
> `pip install -e ".[perception]"` で mediapipe / opencv を入れる（HMR adapter は不要）。
