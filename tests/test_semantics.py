"""RD-MIR semantics の構造化（§3, robotdance_core.semantics）の検証。

pydantic のみで CI 検証可能。
"""

from __future__ import annotations

import json
from pathlib import Path

import jsonschema
import pytest

from robotdance_core.semantics import (
    Semantics,
    build_semantics,
    segment_labels,
    validate_semantics,
)
from robotdance_core.synthetic import generate_dance

_SCHEMA = json.loads(
    (Path(__file__).resolve().parent.parent / "specs" / "rd-mir" / "rd-mir.schema.json")
    .read_text(encoding="utf-8")
)


def test_build_semantics_normalizes() -> None:
    sem = build_semantics(
        action_label="walk", style_tag="casual", captions=["walking", "a person walks"],
        segments=[{"label": "walk", "start_t": 0.0, "end_t": 1.0}, {"label": "stop"}],
        source_dataset="babel", babel_sid="123",
    )
    assert sem["action_label"] == "walk"
    assert sem["style_tag"] == "casual"
    assert sem["captions"] == ["walking", "a person walks"]
    assert len(sem["segments"]) == 2
    assert sem["babel_sid"] == "123"           # extra も保持
    assert segment_labels(sem) == ["walk", "stop"]


def test_action_label_defaults_to_first_caption() -> None:
    assert build_semantics(captions=["a person jumps"])["action_label"] == "a person jumps"
    assert build_semantics()["action_label"] == "unknown"


def test_segment_requires_label() -> None:
    with pytest.raises(Exception):  # noqa: B017 (pydantic ValidationError)
        build_semantics(segments=[{"start_t": 0.0}])


def test_validate_semantics() -> None:
    v: Semantics = validate_semantics(
        {"action_label": "dance", "segments": [{"label": "spin", "start_t": 0.5}]})
    assert v.action_label == "dance"
    assert v.segments[0].label == "spin"
    assert v.segments[0].end_t is None


def test_structured_semantics_schema_valid() -> None:
    mir = generate_dance(duration=1.0)
    mir.semantics = build_semantics(
        action_label="dance", captions=["a dance"],
        segments=[{"label": "sway", "start_t": 0.0, "end_t": 2.0}], source_dataset="synthetic")
    jsonschema.Draft202012Validator(_SCHEMA).validate(mir.to_dict())


def test_free_form_semantics_still_valid() -> None:
    """後方互換: 旧来の自由 dict semantics も schema 適合（additionalProperties）。"""
    mir = generate_dance(duration=1.0)
    mir.semantics = {"action_label": "x", "custom_field": 42, "tempo": 1.6}
    jsonschema.Draft202012Validator(_SCHEMA).validate(mir.to_dict())
