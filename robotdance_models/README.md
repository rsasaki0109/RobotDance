# robotdance_models

tokenizer, encoder, diffusion/autoregressive model, policy training — Motion Encoder / Foundation Model / Policy 学習。

## 実装状況

- `encoder.py` — **Masked Motion Modeling encoder**（小型 Transformer）。canonical motion window を
  マスク再構成で自己教師あり学習する。
- `train.py` — 学習ループ + checkpoint + `LearnedMotionEncoder`。手作り特徴量と**同じ前処理**
  （`robotdance_motion.normalized_keypoints`）・**同じ `embed(mir)` interface**で `MotionIndex` に差し込める。

```bash
pip install -e ".[learn]"          # torch を入れる
robotdance train-encoder -o motion_encoder.pt --epochs 40
robotdance demo-motion-map --checkpoint motion_encoder.pt -o map_learned.png
```

```python
from robotdance_models.train import LearnedMotionEncoder
from robotdance_motion.embeddings import MotionIndex
enc = LearnedMotionEncoder("motion_encoder.pt")
idx = MotionIndex(embed_fn=enc.embed)   # 検索・重複除去・Motion Map が学習表現で動く
```

> ⚠️ **v0:** 学習**基盤**の提供が目的。合成 corpus で masked 再構成 loss が下がり（例: 0.36→0.02）、
> 学習 embedding で dance/backflip を分離できることを示すが、**手作り baseline を超えると主張するものではない**
> （要・実データ規模）。weights は repo に同梱しない（`robotdance-*` weight family の方針）。
> tokenizer / VQ-VAE・foundation model・RL tracking・contrastive video/text-motion は今後。
