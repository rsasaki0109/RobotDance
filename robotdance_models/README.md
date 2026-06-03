# robotdance_models

tokenizer, encoder, diffusion/autoregressive model, policy training — Motion Encoder / Foundation Model / Policy 学習。

## 実装状況

- `encoder.py` — **Masked Motion Modeling encoder**（小型 Transformer）。canonical motion window を
  マスク再構成で自己教師あり学習する。
- `train.py` — 学習ループ + checkpoint + `LearnedMotionEncoder`。手作り特徴量と**同じ前処理**
  （`robotdance_motion.normalized_keypoints`）・**同じ `embed(mir)` interface**で `MotionIndex` に差し込める。
- `text.py` — **決定的ハッシュ n-gram テキスト特徴**（依存なし）。caption → 固定長ベクトル。
- `contrastive.py` — **Contrastive text-motion アライメント**（CLIP 風）。motion encoder と text MLP を
  共有埋め込み空間に射影し、(motion, caption) を multi-positive InfoNCE で整合させる。学習後は
  `embed_text` / `embed_motion` が同じ単位球面に乗り、**テキスト → モーション検索**が可能。

```bash
pip install -e ".[learn]"          # torch を入れる
robotdance train-encoder -o motion_encoder.pt --epochs 40
robotdance demo-motion-map --checkpoint motion_encoder.pt -o map_learned.png

# テキスト → モーション検索（contrastive）
robotdance train-text-motion -o text_motion.pt --epochs 200
robotdance search-text "a person doing a backflip" --checkpoint text_motion.pt
```

```python
from robotdance_models.train import LearnedMotionEncoder
from robotdance_motion.embeddings import MotionIndex
enc = LearnedMotionEncoder("motion_encoder.pt")
idx = MotionIndex(embed_fn=enc.embed)   # 検索・重複除去・Motion Map が学習表現で動く

from robotdance_models.contrastive import TextMotionModel
model = TextMotionModel("text_motion.pt")
model.search("flipping backwards in the air", suite)   # → backflip が top-1
```

> ⚠️ **v0:** 学習**基盤**の提供が目的。
> - **masked encoder**: 合成 corpus で再構成 loss が下がり（例: 0.36→0.02）dance/backflip を分離できることを示すが、
>   **手作り baseline を超えると主張するものではない**（要・実データ規模）。
> - **contrastive text-motion**: 小さな合成 corpus・ハッシュ n-gram テキスト特徴（**事前学習言語モデルなし**）で
>   caption→motion を **action 群レベル top-1 100%**（exact は variant が可換なため低い）で引けることを示す。
>   実キャプション・データ規模・CLIP/sentence-transformers への差し替えは今後。
>
> weights は repo に同梱しない（`robotdance-*` weight family の方針）。tokenizer / VQ-VAE・
> foundation model・RL tracking・contrastive **video**-text-motion は今後。
