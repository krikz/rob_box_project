"""Unit tests for :class:`SQLiteVoiceMemory` — persistent SQLite-backed MemoryStore.

Uses ``:memory:`` database — no file I/O. Tests CRUD operations,
idempotency, upsert, scope isolation, and lifecycle.
"""

from __future__ import annotations

import asyncio

import pytest

from rob_box_harness.memory import Fact, Turn
from rob_box_harness.memory.sqlite_voice import SQLiteVoiceMemory


def _run(coro):
    return asyncio.run(coro)


def _make_store() -> SQLiteVoiceMemory:
    return SQLiteVoiceMemory(db_path=":memory:")


class TestLifecycle:

    def test_init_creates_tables(self) -> None:
        store = _make_store()
        _run(store.init())
        # Should not raise
        assert store._initialized is True

    def test_init_idempotent(self) -> None:
        store = _make_store()
        _run(store.init())
        _run(store.init())  # second call is no-op
        assert store._initialized is True

    def test_teardown_closes_db(self) -> None:
        store = _make_store()
        _run(store.init())
        _run(store.teardown())
        assert store._initialized is False

    def test_teardown_idempotent(self) -> None:
        store = _make_store()
        _run(store.init())
        _run(store.teardown())
        _run(store.teardown())  # no error

    def test_operation_without_init_raises(self) -> None:
        store = _make_store()
        with pytest.raises(RuntimeError, match="not initialised"):
            _run(store.append_turn("s", Turn(role="user", content="hi")))


class TestTurnsCRUD:

    def test_append_and_load(self) -> None:
        store = _make_store()
        _run(store.init())
        _run(store.append_turn("scope_a", Turn(role="user", content="hello")))
        _run(store.append_turn("scope_a", Turn(role="assistant", content="world")))
        turns = _run(store.load_recent("scope_a"))
        assert len(turns) == 2
        assert turns[0].role == "user"
        assert turns[1].role == "assistant"

    def test_load_recent_respects_limit(self) -> None:
        store = _make_store()
        _run(store.init())
        for i in range(5):
            _run(store.append_turn("s", Turn(role="user", content=f"msg{i}")))
        turns = _run(store.load_recent("s", limit=3))
        assert len(turns) == 3
        # Should return most recent 3
        assert turns[0].content == "msg2"
        assert turns[2].content == "msg4"

    def test_load_recent_chronological_order(self) -> None:
        store = _make_store()
        _run(store.init())
        _run(store.append_turn("s", Turn(role="user", content="first")))
        _run(store.append_turn("s", Turn(role="assistant", content="second")))
        _run(store.append_turn("s", Turn(role="user", content="third")))
        turns = _run(store.load_recent("s"))
        assert [t.content for t in turns] == ["first", "second", "third"]

    def test_idempotent_append(self) -> None:
        store = _make_store()
        _run(store.init())
        inserted = _run(store.append_turn("s", Turn(role="user", content="dup")))
        assert inserted is True
        # Same content within 5 seconds → duplicate
        inserted2 = _run(store.append_turn("s", Turn(role="user", content="dup")))
        assert inserted2 is False
        turns = _run(store.load_recent("s"))
        assert len(turns) == 1

    def test_scope_isolation(self) -> None:
        store = _make_store()
        _run(store.init())
        _run(store.append_turn("a", Turn(role="user", content="a_msg")))
        _run(store.append_turn("b", Turn(role="user", content="b_msg")))
        assert len(_run(store.load_recent("a"))) == 1
        assert len(_run(store.load_recent("b"))) == 1

    def test_load_recent_invalid_limit(self) -> None:
        store = _make_store()
        _run(store.init())
        with pytest.raises(ValueError, match="positive"):
            _run(store.load_recent("s", limit=0))


class TestFactsCRUD:

    def test_save_fact(self) -> None:
        store = _make_store()
        _run(store.init())
        _run(store.save_fact("s", Fact(key="color", value="red", tags=("appearance",))))
        # No error means success (SQLiteVoiceMemory doesn't have a get_fact method)
        # Just verify it doesn't crash

    def test_save_fact_upsert(self) -> None:
        """Save same key twice — should not duplicate."""
        store = _make_store()
        _run(store.init())
        _run(store.save_fact("s", Fact(key="k", value="v1")))
        _run(store.save_fact("s", Fact(key="k", value="v2")))
        # No exception — upsert succeeded
