# robotdance_motion

canonicalization, contacts, embeddings, retrieval — canonical skeleton 化と motion embedding / 検索。

## 実装状況

- `smoothing.py` — **temporal smoothing**（Savitzky-Golay）と jitter 指標。
  monocular pose の jittery な出力を平滑化する。`smooth_rdmir(mir)` は quality_metrics に
  `jitter_before/after` を記録。`jitter(kps)` = フレーム間加速度の平均ノルム。

```python
from robotdance_motion.smoothing import smooth_rdmir
clean = smooth_rdmir(mir)   # extract_motion(smooth=True) でも自動適用
```

- `embeddings.py` — **motion embedding / retrieval / Motion Map**（v0, 特徴量ベース）。
  `embed(mir)` が RD-MIR を固定長ベクトル化し、`MotionIndex` が類似検索・重複検出・2D 射影（PCA）を提供。

```python
from robotdance_motion.embeddings import MotionIndex, embed
idx = MotionIndex()
idx.add_mir(mir_a); idx.add_mir(mir_b)
idx.query(embed(query_mir), k=5)   # 類似動作検索
idx.duplicates(threshold=0.98)     # near-duplicate 検出
idx.project_2d()                   # Motion Map 用 2D 射影
```

> ⚠️ **v0 注意:** embedding は**学習済み encoder ではなく決定的な手作り特徴量**
> （root-relative + scale 正規化 + per-frame yaw 整列で位置/向き/スケール不変、joint 分布と
> 運動エネルギーを集約）。学習 encoder（masked modeling / contrastive）は Phase 3 でこの
> interface を差し替える。retrieval/contacts の高度化も今後。
