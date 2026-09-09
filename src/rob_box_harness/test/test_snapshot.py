"""Tests for the SessionSnapshot value object.

Covers:

* ``to_dict`` produces a JSON-serialisable dict.
* ``to_json`` round-trips through ``from_dict``.
* Missing or wrong-typed fields raise ``ValueError``.
* Naive datetimes are coerced to UTC on parse.
"""

from __future__ import annotations

import json
from datetime import datetime, timezone

import pytest

from rob_box_harness.snapshot import SessionSnapshot


def _make_snapshot() -> SessionSnapshot:
    """A canonical snapshot for tests."""
    return SessionSnapshot(
        harness_name="echo_main",
        harness_kind="echo",
        captured_at=datetime(2026, 7, 24, 12, 0, 0, tzinfo=timezone.utc),
        state={"count": 3, "flag": True},
        extensions={"last_assistant_text": "hi"},
    )


def test_to_dict_is_jsonable() -> None:
    """``to_dict`` produces something that ``json.dumps`` accepts."""
    snap = _make_snapshot()
    payload = snap.to_dict()
    json.dumps(payload)  # no exception
    assert payload["harness_name"] == "echo_main"
    assert payload["harness_kind"] == "echo"
    assert payload["state"] == {"count": 3, "flag": True}


def test_to_json_round_trip() -> None:
    """``to_json`` then ``from_dict`` recovers the snapshot."""
    snap = _make_snapshot()
    parsed = SessionSnapshot.from_dict(json.loads(snap.to_json()))
    assert parsed.harness_name == snap.harness_name
    assert parsed.harness_kind == snap.harness_kind
    assert parsed.captured_at == snap.captured_at
    assert parsed.state == snap.state
    assert parsed.extensions == snap.extensions


def test_from_dict_accepts_naive_datetime() -> None:
    """Naive datetimes in the payload are treated as UTC."""
    payload = {
        "harness_name": "x",
        "harness_kind": "echo",
        "captured_at": "2026-07-24T12:00:00",  # no tz
        "state": {},
        "extensions": {},
    }
    snap = SessionSnapshot.from_dict(payload)
    assert snap.captured_at.tzinfo is not None
    assert snap.captured_at == datetime(2026, 7, 24, 12, 0, 0, tzinfo=timezone.utc)


def test_from_dict_rejects_non_mapping() -> None:
    """A non-mapping payload raises ``ValueError``."""
    with pytest.raises(ValueError, match="must be a mapping"):
        SessionSnapshot.from_dict(["nope"])  # type: ignore[arg-type]


def test_from_dict_rejects_bad_captured_at() -> None:
    """A missing or wrong-typed ``captured_at`` raises ``ValueError``."""
    with pytest.raises(ValueError, match="captured_at"):
        SessionSnapshot.from_dict(
            {"harness_name": "x", "harness_kind": "y", "state": {}}
        )


def test_from_dict_defaults_state_and_extensions() -> None:
    """Missing ``state`` / ``extensions`` default to empty mappings."""
    snap = SessionSnapshot.from_dict(
        {
            "harness_name": "x",
            "harness_kind": "y",
            "captured_at": "2026-07-24T12:00:00+00:00",
        }
    )
    assert snap.state == {}
    assert snap.extensions == {}


def test_snapshot_extensions_are_independent_of_input() -> None:
    """``to_dict`` returns a shallow copy of the extensions mapping."""
    snap = _make_snapshot()
    payload = snap.to_dict()
    payload["extensions"]["last_assistant_text"] = "mutated"
    # The original snapshot is untouched.
    assert snap.extensions["last_assistant_text"] == "hi"
