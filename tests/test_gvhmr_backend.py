"""GVHMR in-process extraction backend."""

from __future__ import annotations

from pathlib import Path
from unittest.mock import patch

import numpy as np
import pytest

from robotdance_perception.backends import get_backend
from robotdance_perception.gvhmr_backend import (
    _pred_to_gvhmr_result,
    gvhmr_available,
    gvhmr_importable,
    gvhmr_install_hint,
)


def test_gvhmr_registry_video_mode():
    b = get_backend("gvhmr")
    assert b.extract_mode == "video"
    assert b.quality_tier == "world-grounded"
    assert "hmr4d" in b.modules


def test_gvhmr_install_hint_mentions_repo():
    assert "github.com/zju3dv/GVHMR" in gvhmr_install_hint()


def test_gvhmr_availability_helpers_type():
    assert isinstance(gvhmr_importable(), bool)
    assert isinstance(gvhmr_available(), bool)


def test_pred_to_gvhmr_result_numpy():
    pytest.importorskip("torch")
    import torch

    t = 5
    pred = {
        "smpl_params_global": {
            "global_orient": torch.zeros(t, 3),
            "body_pose": torch.zeros(t, 63),
            "transl": torch.zeros(t, 3),
        },
        "smpl_params_incam": {
            "global_orient": torch.zeros(t, 3),
            "body_pose": torch.zeros(t, 63),
            "transl": torch.zeros(t, 3),
        },
    }
    result = _pred_to_gvhmr_result(pred)
    assert result["smpl_params_global"]["global_orient"].shape == (t, 3)
    assert isinstance(result["smpl_params_global"]["body_pose"], np.ndarray)


def test_cli_extract_gvhmr_unavailable(tmp_path: Path, capsys) -> None:
    from robotdance_core.cli import main

    if gvhmr_available():
        pytest.skip("GVHMR 導入済み — unavailable テストはスキップ")
    rc = main(["extract", str(tmp_path / "clip.mp4"), "--backend", "gvhmr"])
    assert rc == 1
    out = capsys.readouterr().out
    assert "import-hmr" in out or "GVHMR" in out


def test_cli_extract_gvhmr_wiring_with_mock(tmp_path: Path) -> None:
    from robotdance_core.rd_mir import RdMir
    from robotdance_core.cli import main
    from robotdance_core.synthetic import generate_dance

    fake = generate_dance(duration=0.3)
    video = tmp_path / "v.mp4"
    video.write_bytes(b"\x00")  # existence only; mock replaces extract

    with patch("robotdance_perception.gvhmr_backend.gvhmr_available", return_value=True), \
         patch("robotdance_perception.gvhmr_backend.extract_gvhmr_video", return_value=fake):
        out = tmp_path / "out.rdmir.json"
        assert main(["extract", str(video), "--backend", "gvhmr", "-o", str(out), "--no-check"]) == 0
    assert out.is_file()
    assert RdMir.load(out).num_frames == fake.num_frames
