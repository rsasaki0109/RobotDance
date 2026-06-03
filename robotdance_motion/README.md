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

> embeddings / retrieval / contacts の高度化（motion encoder 等）は今後（ロードマップ Phase 3）。
