"""Colab quickstart ノートブックが壊れていない / CLI からドリフトしていないことを確認する。

重依存（nbformat 等）不要——素の json で読み、参照している CLI サブコマンドと robot 名が
実在することだけを軽量に検証する。ノートブックのコマンドが将来 CLI 変更で無効化するのを防ぐ。
"""

from __future__ import annotations

import json
import re
from pathlib import Path

from robotdance_unitree import EMBODIMENTS

_ROOT = Path(__file__).resolve().parent.parent
_NB = _ROOT / "notebooks" / "quickstart.ipynb"


def _code_text() -> str:
    nb = json.loads(_NB.read_text())
    assert nb["nbformat"] == 4
    lines: list[str] = []
    for cell in nb["cells"]:
        if cell["cell_type"] == "code":
            lines.extend(cell["source"])
    return "".join(lines)


def test_notebook_is_valid_json_with_cells() -> None:
    nb = json.loads(_NB.read_text())
    assert nb["cells"], "ノートブックにセルが無い"
    assert any(c["cell_type"] == "code" for c in nb["cells"])


def test_install_cell_points_at_this_repo() -> None:
    code = _code_text()
    assert "git+https://github.com/rsasaki0109/HumanoidBattle.git" in code
    assert "robotdance[demo]" in code  # 軽量 extra でツアーが回る


def test_referenced_cli_subcommands_exist() -> None:
    code = _code_text()
    used = set(re.findall(r"robotdance_core\.cli\s+([a-z][a-z0-9-]+)", code))
    assert used, "ノートブックが CLI を呼んでいない"
    cli_src = (_ROOT / "robotdance_core" / "cli.py").read_text()
    registered = set(re.findall(r'add_parser\("([a-z0-9-]+)"', cli_src))
    missing = used - registered
    assert not missing, f"ノートブックが未登録の CLI コマンドを参照: {missing}"


def test_demo_multi_robots_are_known_embodiments() -> None:
    code = _code_text()
    m = re.search(r"--robots\s+([a-z0-9_ \\\n]+?)\n", code.replace("\\\n", " "))
    assert m, "demo-multi の --robots が見つからない"
    robots = [r for r in m.group(1).split() if r and not r.startswith("-")]
    assert len(robots) >= 6
    unknown = [r for r in robots if r not in EMBODIMENTS]
    assert not unknown, f"未知の robot を参照: {unknown}"
