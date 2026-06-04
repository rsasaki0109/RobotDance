"""汎用 RD-MIR near-duplicate 除去（§4.1, robotdance_motion.dedupe）の検証。

motion embedding は numpy のみ → CI 検証可能。
"""

from __future__ import annotations

from robotdance_core.synthetic import generate_backflip, generate_dance
from robotdance_motion.dedupe import dedupe_groups, dedupe_mirs


def _collection():
    a = generate_dance(beats_per_second=1.0)
    a.motion_id = "dance_a"
    b = generate_dance(beats_per_second=1.0)   # a と同一モーション・別 id
    b.motion_id = "dance_b_dup"
    c = generate_dance(beats_per_second=0.7)
    c.motion_id = "dance_slow"
    d = generate_backflip(duration=1.6)
    d.motion_id = "backflip"
    return [a, b, c, d]


def test_dedupe_removes_near_duplicate() -> None:
    res = dedupe_mirs(_collection(), threshold=0.98)
    assert res["total"] == 4
    assert res["kept_count"] == 3
    assert res["removed_count"] == 1
    assert "dance_b_dup" in res["removed"]
    kept_ids = {m.motion_id for m in res["kept"]}
    assert "dance_slow" in kept_ids and "backflip" in kept_ids
    # 重複グループが記録される。
    dup = [g for g in res["groups"] if g["size"] > 1]
    assert len(dup) == 1
    assert set(dup[0]["members"]) == {"dance_a", "dance_b_dup"}


def test_dedupe_representative_is_longest() -> None:
    short = generate_dance(duration=1.0, beats_per_second=1.0)
    short.motion_id = "short"
    long = generate_dance(duration=3.0, beats_per_second=1.0)
    long.motion_id = "long"
    res = dedupe_mirs([short, long], threshold=0.95)
    if res["kept_count"] == 1:  # 同一とみなされた場合、代表は長い方。
        assert res["kept"][0].motion_id == "long"
        assert "short" in res["removed"]


def test_distinct_motions_not_merged() -> None:
    a = generate_dance(beats_per_second=1.0)
    a.motion_id = "dance"
    b = generate_backflip(duration=1.6)
    b.motion_id = "flip"
    res = dedupe_mirs([a, b], threshold=0.98)
    assert res["kept_count"] == 2
    assert res["removed_count"] == 0


def test_edge_cases() -> None:
    assert dedupe_groups([]) == []
    one = generate_dance()
    res = dedupe_mirs([one])
    assert res["kept_count"] == 1
    assert res["removed_count"] == 0
