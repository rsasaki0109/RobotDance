"""Hugging Face Hub からモーションデータセット等のファイルを取得する（v0）。

`extract`（動画→姿勢）がライセンス上扱いにくい YouTube/TikTok を避け、ライセンスが明示された
HF Hub のデータセット（HumanML3D / Motion-X / BABEL 由来 等）を取り込むための薄い fetch 層。

license-safe 方針: 取得物は repo に同梱しない（HF キャッシュ = repo 外）。多くのモーションデータ
セットは AMASS 由来で **研究用途限定（research_only）**。取り込んだ RD-MIR の license_state は
import-* 側で research_only 等に設定される。再配布・商用は各データセットのライセンスに従うこと。
"""

from __future__ import annotations

from pathlib import Path
from typing import Optional


def fetch_from_hub(
    repo_id: str,
    filename: str,
    *,
    repo_type: str = "dataset",
    revision: Optional[str] = None,
    cache_dir: Optional[str | Path] = None,
) -> Path:
    """HF Hub の 1 ファイルを取得し、ローカルキャッシュのパスを返す。

    repo_type: "dataset"（既定）/ "model" / "space"。
    生ファイルは HF キャッシュ（既定 ~/.cache/huggingface、repo 外）に置かれる。返り値の
    パスを `import-humanml3d` / `import-motionx` 等に渡して RD-MIR 化する。
    """
    try:
        from huggingface_hub import hf_hub_download
    except ImportError as e:  # pragma: no cover - 依存欠如時の案内
        raise RuntimeError(
            "huggingface_hub が必要です。`pip install huggingface_hub` を実行してください。"
        ) from e

    path = hf_hub_download(
        repo_id=repo_id,
        filename=filename,
        repo_type=repo_type,
        revision=revision,
        cache_dir=str(cache_dir) if cache_dir is not None else None,
    )
    return Path(path)
