"""retarget バックエンドレジストリ（robotdance_retarget.backends）。"""

from __future__ import annotations

import pytest

from robotdance_retarget.backends import (
    GMR,
    KINEMATIC,
    get_retarget_backend,
    list_retarget_backends,
)


def test_registry_lists_known_sorted():
    names = [b.name for b in list_retarget_backends()]
    assert names == sorted(names)
    assert {"kinematic", "actuator-ik", "gmr"} <= set(names)


def test_get_unknown_raises_with_candidates():
    with pytest.raises(ValueError, match="未知の retarget backend"):
        get_retarget_backend("openpose")


def test_kinematic_is_builtin_no_urdf():
    assert KINEMATIC.method == "kinematic"
    assert KINEMATIC.real_urdf is False
    assert KINEMATIC.cli == "retarget"
    assert "external" not in KINEMATIC.extras
    assert KINEMATIC.available() is True  # builtin（依存なし）


def test_actuator_ik_uses_real_urdf():
    b = get_retarget_backend("actuator-ik")
    assert b.method == "actuator-ik"
    assert b.real_urdf is True
    assert b.cli == "retarget-ik"


def test_gmr_is_external_with_url():
    assert GMR.method == "external"
    assert "external" in GMR.extras
    assert GMR.url.startswith("https://github.com/")
    assert GMR.cli == ""  # builtin CLI は持たない
    # available は遅延 spec チェックで bool（未導入なら False）。例外は投げない。
    assert isinstance(GMR.available(), bool)


def test_list_retargeters_cli_runs():
    from robotdance_core.cli import main

    assert main(["list-retargeters"]) == 0
