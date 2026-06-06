"""リリースメタデータの整合性（pyproject / CITATION / CHANGELOG / specs）。

毎リリースで pyproject.toml・CITATION.cff・CHANGELOG.md を手作業同期しているため、
バージョン/日付のドリフトを CI で検出する。v1.0（Stable Specs）に向けた release hygiene。
"""

from __future__ import annotations

import json
import re
from pathlib import Path

import jsonschema

_ROOT = Path(__file__).resolve().parent.parent


def _pyproject_version() -> str:
    text = (_ROOT / "pyproject.toml").read_text(encoding="utf-8")
    m = re.search(r'^version\s*=\s*"([^"]+)"', text, re.MULTILINE)
    assert m, "pyproject.toml に version が無い"
    return m.group(1)


def _citation() -> tuple[str, str]:
    text = (_ROOT / "CITATION.cff").read_text(encoding="utf-8")
    v = re.search(r'^version:\s*"?([0-9][^"\s]*)"?', text, re.MULTILINE)
    d = re.search(r'^date-released:\s*"?([0-9]{4}-[0-9]{2}-[0-9]{2})"?', text, re.MULTILINE)
    assert v and d, "CITATION.cff に version / date-released が無い"
    return v.group(1), d.group(1)


def _changelog_top() -> tuple[str, str]:
    text = (_ROOT / "CHANGELOG.md").read_text(encoding="utf-8")
    m = re.search(r'^##\s*\[([0-9][^\]]*)\]\s*-\s*([0-9]{4}-[0-9]{2}-[0-9]{2})', text, re.MULTILINE)
    assert m, "CHANGELOG.md に最新リリースエントリが無い"
    return m.group(1), m.group(2)


def test_versions_match_across_metadata() -> None:
    py = _pyproject_version()
    cff_v, _ = _citation()
    cl_v, _ = _changelog_top()
    assert py == cff_v == cl_v, (
        f"version drift: pyproject={py} CITATION={cff_v} CHANGELOG={cl_v}")


def test_citation_date_matches_changelog() -> None:
    _, cff_date = _citation()
    _, cl_date = _changelog_top()
    assert cff_date == cl_date, (
        f"date drift: CITATION date-released={cff_date} vs CHANGELOG top={cl_date}")


def test_all_spec_schemas_present_and_valid() -> None:
    names = ["rd-mir", "rd-manifest", "rd-embodiment", "rd-motion", "rd-policy"]
    for n in names:
        p = _ROOT / "specs" / n / f"{n}.schema.json"
        assert p.exists(), f"spec schema が無い: {p}"
        schema = json.loads(p.read_text(encoding="utf-8"))
        # メタスキーマに対して妥当（壊れた JSON Schema を弾く）。
        jsonschema.Draft202012Validator.check_schema(schema)


def test_spec_schemas_declare_version_matching_id() -> None:
    """各 spec が $schema / $id / title / version を宣言し、version が $id の /vN/ と一致する。"""
    names = ["rd-mir", "rd-manifest", "rd-embodiment", "rd-motion", "rd-policy"]
    for n in names:
        schema = json.loads((_ROOT / "specs" / n / f"{n}.schema.json").read_text("utf-8"))
        for key in ("$schema", "$id", "title", "version"):
            assert key in schema, f"{n}: '{key}' が無い"
        m = re.search(r"/v([^/]+)/", schema["$id"])
        assert m, f"{n}: $id に /vN/ が無い: {schema['$id']}"
        assert str(schema["version"]) == m.group(1), (
            f"{n}: version={schema['version']} が $id の v{m.group(1)} と不一致")
