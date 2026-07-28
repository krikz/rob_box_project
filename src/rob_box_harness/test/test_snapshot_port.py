"""Tests for SnapshotStore port + TelegramUpdate dataclass parsing.

RED-GREEN-REFACTOR for P1.4 M0.
"""

from __future__ import annotations

import asyncio
from typing import Any

import pytest

from rob_box_harness.snapshot_store import (
    InMemorySnapshotStore,
    SnapshotStore,
    parse_telegram_update,
)
from rob_box_harness.transport import TelegramUpdate


def _run(coro):
    return asyncio.run(coro)


# ---------------------------------------------------------------------------
# RED-1: SnapshotStore port ABC + InMemorySnapshotStore impl
# ---------------------------------------------------------------------------


def test_snapshot_store_put_then_get_latest() -> None:
    store = InMemorySnapshotStore()
    key = _run(store.put(scope="tg:123", payload=b"jpeg-bytes-1"))
    assert isinstance(key, str) and key
    latest = _run(store.get_latest(scope="tg:123"))
    assert latest is not None
    assert latest.payload == b"jpeg-bytes-1"


def test_snapshot_store_get_latest_missing_returns_none() -> None:
    store = InMemorySnapshotStore()
    assert _run(store.get_latest(scope="nope")) is None


def test_snapshot_store_get_latest_picks_newest() -> None:
    store = InMemorySnapshotStore()
    _run(store.put(scope="tg:1", payload=b"first"))
    _run(store.put(scope="tg:1", payload=b"second"))
    _run(store.put(scope="tg:1", payload=b"third"))
    latest = _run(store.get_latest(scope="tg:1"))
    assert latest is not None and latest.payload == b"third"


def test_snapshot_store_isolates_scopes() -> None:
    store = InMemorySnapshotStore()
    _run(store.put(scope="tg:1", payload=b"a"))
    _run(store.put(scope="tg:2", payload=b"b"))
    a = _run(store.get_latest(scope="tg:1"))
    b = _run(store.get_latest(scope="tg:2"))
    assert a is not None and b is not None
    assert a.payload == b"a"
    assert b.payload == b"b"


def test_snapshot_store_aclose_idempotent() -> None:
    store = InMemorySnapshotStore()
    _run(store.aclose())
    _run(store.aclose())  # second call no-op


# ---------------------------------------------------------------------------
# RED-2: parse_telegram_update normalises dict updates into TelegramUpdate
# ---------------------------------------------------------------------------


def test_parse_telegram_update_from_minimal_dict() -> None:
    update = parse_telegram_update({"chat_id": "123", "user_id": "456", "command": "/start"})
    assert isinstance(update, TelegramUpdate)
    assert update.payload["chat_id"] == "123"
    assert update.payload["user_id"] == "456"
    assert update.payload["command"] == "/start"
    assert update.kind == "message"


def test_parse_telegram_update_handles_already_typed() -> None:
    raw = TelegramUpdate(update_id=1, kind="message", payload={"chat_id": "1", "user_id": "2"})
    out = parse_telegram_update(raw)
    assert out is raw  # identity preserved


def test_parse_telegram_update_handles_plain_string() -> None:
    update = parse_telegram_update("hello world")
    assert update.kind == "message"
    assert update.payload["text"] == "hello world"


def test_parse_telegram_update_normalises_command_and_args() -> None:
    update = parse_telegram_update({
        "chat_id": "123",
        "user_id": "456",
        "command": "/voice",
        "args": "поговори со мной",
    })
    assert update.payload["command"] == "/voice"
    assert update.payload["args"] == "поговори со мной"


def test_parse_telegram_update_rejects_non_mapping_non_string() -> None:
    with pytest.raises(TypeError):
        parse_telegram_update(42)  # type: ignore[arg-type]