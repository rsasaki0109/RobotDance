"""Hugging Face Hub fetch のテスト（ネットワーク不要・monkeypatch でモック）。"""

from __future__ import annotations

import builtins

import pytest


def test_fetch_from_hub_calls_hf_hub_download(monkeypatch, tmp_path):
    huggingface_hub = pytest.importorskip("huggingface_hub")

    import robotdance_data.hf_fetch as hf

    fake = tmp_path / "data.npy"
    fake.write_text("x")
    calls: dict = {}

    def fake_download(**kw):
        calls.update(kw)
        return str(fake)

    monkeypatch.setattr(huggingface_hub, "hf_hub_download", fake_download)
    p = hf.fetch_from_hub("EricGuo5513/HumanML3D", "new_joints/000.npy", repo_type="dataset")
    assert p == fake
    assert calls["repo_id"] == "EricGuo5513/HumanML3D"
    assert calls["filename"] == "new_joints/000.npy"
    assert calls["repo_type"] == "dataset"


def test_fetch_from_hub_missing_dependency(monkeypatch):
    import robotdance_data.hf_fetch as hf

    real_import = builtins.__import__

    def blocked(name, *a, **k):
        if name == "huggingface_hub":
            raise ImportError("simulated missing huggingface_hub")
        return real_import(name, *a, **k)

    monkeypatch.setattr(builtins, "__import__", blocked)
    with pytest.raises(RuntimeError, match="huggingface_hub"):
        hf.fetch_from_hub("r/x", "f.npy")


def test_cli_download_hf_copies_to_out(monkeypatch, tmp_path):
    from robotdance_core.cli import main

    import robotdance_data.hf_fetch as hf

    cached = tmp_path / "cached.npy"
    cached.write_text("y")
    monkeypatch.setattr(hf, "fetch_from_hub", lambda *a, **k: cached)

    out = tmp_path / "local.npy"
    rc = main(["download-hf", "repo/x", "file.npy", "-o", str(out)])
    assert rc == 0
    assert out.exists() and out.read_text() == "y"
