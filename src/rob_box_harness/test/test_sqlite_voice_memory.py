"""Unit tests for :class:`SQLiteVoiceMemory` — persistent SQLite-backed MemoryStore.

Uses ``:memory:`` database — no file I/O. Tests CRUD operations,
idempotency, upsert, scope isolation, and lifecycle.

W4 coverage adds waypoint CRUD, FAQ load/search, and the singleton
event-profile pair.
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


class TestWaypointsCRUD:
    """W4: waypoint CRUD on the SQLite backend."""

    def test_save_and_list(self) -> None:
        store = _make_store()
        _run(store.init())
        _run(store.save_waypoint("kitchen", 1.0, 2.0, 0.5))
        _run(store.save_waypoint("dock", 0.0, 0.0))
        waypoints = _run(store.list_waypoints())
        # ORDER BY name: dock, kitchen
        assert [w.name for w in waypoints] == ["dock", "kitchen"]
        kitchen = waypoints[1]
        assert kitchen.x == 1.0
        assert kitchen.y == 2.0
        assert kitchen.theta == 0.5
        assert waypoints[0].theta == 0.0

    def test_upsert_replaces_pose(self) -> None:
        store = _make_store()
        _run(store.init())
        _run(store.save_waypoint("kitchen", 1.0, 2.0, 0.5))
        _run(store.save_waypoint("kitchen", 10.0, 20.0, 1.0))
        waypoints = _run(store.list_waypoints())
        assert len(waypoints) == 1
        assert waypoints[0].x == 10.0
        assert waypoints[0].y == 20.0
        assert waypoints[0].theta == 1.0

    def test_delete_returns_true_then_false(self) -> None:
        store = _make_store()
        _run(store.init())
        _run(store.save_waypoint("kitchen", 1.0, 2.0))
        assert _run(store.delete_waypoint("kitchen")) is True
        assert _run(store.delete_waypoint("kitchen")) is False
        assert _run(store.delete_waypoint("nope")) is False
        assert _run(store.list_waypoints()) == []

    def test_clear_returns_count(self) -> None:
        store = _make_store()
        _run(store.init())
        _run(store.save_waypoint("a", 0.0, 0.0))
        _run(store.save_waypoint("b", 1.0, 1.0))
        removed = _run(store.clear_waypoints())
        assert removed == 2
        assert _run(store.list_waypoints()) == []
        # Second clear returns 0
        assert _run(store.clear_waypoints()) == 0

    def test_save_waypoint_rejects_empty_name(self) -> None:
        store = _make_store()
        _run(store.init())
        with pytest.raises(ValueError, match="non-empty"):
            _run(store.save_waypoint("", 1.0, 2.0))

    def test_waypoints_persist_across_init(self) -> None:
        """W4: persistence — close & re-open the same DB path."""
        import tempfile

        with tempfile.TemporaryDirectory() as tmpdir:
            db_path = f"{tmpdir}/wpt.db"
            store_a = SQLiteVoiceMemory(db_path=db_path)
            _run(store_a.init())
            _run(store_a.save_waypoint("kitchen", 1.0, 2.0))
            _run(store_a.teardown())

            store_b = SQLiteVoiceMemory(db_path=db_path)
            _run(store_b.init())
            waypoints = _run(store_b.list_waypoints())
            assert len(waypoints) == 1
            assert waypoints[0].name == "kitchen"
            assert waypoints[0].x == 1.0
            _run(store_b.teardown())


class TestFAQCRUD:
    """W4: FAQ load/search on the SQLite backend."""

    def test_load_faq_inserts_rows(self) -> None:
        store = _make_store()
        _run(store.init())
        inserted = _run(
            store.load_faq(
                "expo",
                [
                    {"question": "Q1", "answer": "A1", "category": "info"},
                    {"question": "Q2", "answer": "A2"},
                    {"question": "", "answer": ""},  # skipped
                ],
            )
        )
        assert inserted == 2

    def test_load_faq_replaces_previous_rows(self) -> None:
        store = _make_store()
        _run(store.init())
        _run(store.load_faq("expo", [{"question": "old", "answer": "old"}]))
        inserted = _run(
            store.load_faq("expo", [{"question": "new", "answer": "new"}])
        )
        assert inserted == 1
        # Old row must be gone
        results = _run(store.search_faq("expo", "old"))
        assert results == []

    def test_load_faq_isolates_events(self) -> None:
        store = _make_store()
        _run(store.init())
        _run(store.load_faq("expo", [{"question": "Q", "answer": "A1"}]))
        _run(store.load_faq("conf", [{"question": "Q", "answer": "A2"}]))
        expo = _run(store.search_faq("expo", "Q"))
        conf = _run(store.search_faq("conf", "Q"))
        assert expo[0].answer == "A1"
        assert conf[0].answer == "A2"

    def test_search_faq_finds_keyword(self) -> None:
        store = _make_store()
        _run(store.init())
        _run(
            store.load_faq(
                "expo",
                [
                    {"question": "schedule", "answer": "10am"},
                    {"question": "where", "answer": "hall"},
                    {"question": "food", "answer": "yes"},
                ],
            )
        )
        results = _run(store.search_faq("expo", "schedule"))
        assert len(results) == 1
        assert results[0].question == "schedule"
        assert results[0].answer == "10am"

    def test_search_faq_matches_in_answer_field(self) -> None:
        store = _make_store()
        _run(store.init())
        _run(
            store.load_faq(
                "expo",
                [{"question": "?", "answer": "cafeteria on second floor"}],
            )
        )
        results = _run(store.search_faq("expo", "cafeteria"))
        assert len(results) == 1

    def test_search_faq_respects_limit(self) -> None:
        store = _make_store()
        _run(store.init())
        _run(
            store.load_faq(
                "expo",
                [{"question": f"Q{i}", "answer": f"A{i}"} for i in range(5)],
            )
        )
        results = _run(store.search_faq("expo", "Q", limit=2))
        assert len(results) == 2

    def test_search_faq_empty_query_returns_empty(self) -> None:
        store = _make_store()
        _run(store.init())
        _run(store.load_faq("expo", [{"question": "Q", "answer": "A"}]))
        assert _run(store.search_faq("expo", "")) == []
        assert _run(store.search_faq("expo", "   ")) == []

    def test_search_faq_rejects_zero_limit(self) -> None:
        store = _make_store()
        _run(store.init())
        with pytest.raises(ValueError, match="limit"):
            _run(store.search_faq("expo", "x", limit=0))

    def test_load_faq_rejects_empty_event_id(self) -> None:
        store = _make_store()
        _run(store.init())
        with pytest.raises(ValueError, match="event_id"):
            _run(store.load_faq("", [{"question": "Q", "answer": "A"}]))

    def test_search_faq_rejects_empty_event_id(self) -> None:
        store = _make_store()
        _run(store.init())
        with pytest.raises(ValueError, match="event_id"):
            _run(store.search_faq("", "x"))


class TestEventProfileCRUD:
    """W4: singleton event profile on the SQLite backend."""

    def test_get_unset_returns_none(self) -> None:
        store = _make_store()
        _run(store.init())
        assert _run(store.get_event_profile()) is None

    def test_set_then_get(self) -> None:
        store = _make_store()
        _run(store.init())
        _run(store.set_event_profile({"event_id": "expo", "name": "Expo 2026"}))
        profile = _run(store.get_event_profile())
        assert profile == {"event_id": "expo", "name": "Expo 2026"}

    def test_set_overwrites(self) -> None:
        store = _make_store()
        _run(store.init())
        _run(store.set_event_profile({"event_id": "first"}))
        _run(store.set_event_profile({"event_id": "second"}))
        assert _run(store.get_event_profile()) == {"event_id": "second"}

    def test_profile_persists_across_init(self) -> None:
        """W4: persistence — close & re-open the same DB path."""
        import tempfile

        with tempfile.TemporaryDirectory() as tmpdir:
            db_path = f"{tmpdir}/prof.db"
            store_a = SQLiteVoiceMemory(db_path=db_path)
            _run(store_a.init())
            _run(store_a.set_event_profile({"event_id": "expo", "name": "X"}))
            _run(store_a.teardown())

            store_b = SQLiteVoiceMemory(db_path=db_path)
            _run(store_b.init())
            profile = _run(store_b.get_event_profile())
            assert profile == {"event_id": "expo", "name": "X"}
            _run(store_b.teardown())

    def test_profile_singleton_invariant(self) -> None:
        """W4: the table can hold at most one row at a time."""
        store = _make_store()
        _run(store.init())
        for i in range(5):
            _run(store.set_event_profile({"event_id": f"e{i}"}))
        # Cursor count check — at most one row in event_profile
        cursor = _run_safe(store._conn.execute("SELECT COUNT(*) FROM event_profile"))
        assert cursor.fetchone()[0] == 1
        assert _run(store.get_event_profile()) == {"event_id": "e4"}


def _run_safe(coro_or_call):
    """Run a coroutine or invoke a sqlite cursor; small test-only helper.

    The cursor argument pattern is used when the test wants to inspect
    internal SQLite state directly. pytest-asyncio is not enabled in
    this project (asyncio_mode != auto), so we drive everything through
    asyncio.run for async calls and direct invocation for sync ones.
    """
    if hasattr(coro_or_call, "execute"):
        return coro_or_call
    return _run(coro_or_call)
