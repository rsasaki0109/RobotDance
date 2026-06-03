# robotdance_perception

pose adapters, human tracking, HMR adapters, smoothing — 2D/3D pose・人間メッシュ復元を adapter 方式で束ねる。

## 実装状況

- `mediapipe_adapter.py` — **MediaPipe Pose による local 動画 → RD-MIR**。
  `extract_motion(video) -> RdMir` が pose_world_landmarks（33点・メートル3D）を canonical
  19-joint へマップする。`mp_world_landmarks_to_canonical` は純関数で単体テスト可能。

```python
from robotdance_perception.mediapipe_adapter import extract_motion
mir = extract_motion("my_clip.mp4")   # → RD-MIR（license_state="unknown"）
```

- 座標変換: MediaPipe world（x:右, y:下, z:手前負, 腰原点）→ canonical（x:前, y:左, z:上）= `(-z, x, -y)`。
- モデル（Google 配布 Apache-2.0 の `.task`）は `~/.cache/robotdance/models/` へ自動 DL（`ROBOTDANCE_POSE_MODEL` で上書き可）。

> ⚠️ **ライセンス:** 入力動画の権利はユーザー責任。アダプタは動画を再配布せず、抽出 RD-MIR の
> `license_state` は既定で `"unknown"`（source 未確認 → 派生 motion を公開しない）。
> 検証は landmark→canonical の単体テスト + scikit-image の astronaut（NASA, public domain）実写で行う。
> HMR（4DHumans / GVHMR 等）adapter・temporal smoothing・multi-person tracking は今後。
> `pip install -e ".[perception]"` で mediapipe / opencv を入れる。
