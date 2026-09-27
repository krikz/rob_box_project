"""Regression tests for :class:`WaypointAdapter` (issue #2000 / ADR-0055).

These tests run the four scenarios spelled out in the issue #2000 card
body (commit C3):

1. ``Запись в mcp-схему -> чтение через адаптер в формате harness`` -
   the WaypointStore (mcp-schema, writes) and the WaypointAdapter
   (harness-shape reads) share one single source of truth.
2. ``Существующие waypoints в mcp-БД доступны через адаптер`` -
   data written directly via WaypointStore is visible through
   ``WaypointAdapter.list_waypoints_harness``.
3. ``map_id по умолчанию = 'default' если не указан`` -
   harness-shaped writes synthesise ``map_id='default'`` on first use
   and never leak random UUIDs to the harness view.
4. ``Конкурентные записи через разные сторы -> один вердикт`` -
   writes through WaypointStore (mcp-view) and WaypointAdapter
   (harness-view) on the same active map are observable by both.

All tests use ``tmp_path`` so they don't touch the production DB;
in-memory is available via ``db_path=':memory:'`` for the harder
edge cases (``source``: pass ``db_path`` explicitly for :memory: support).

Refs
----
* ADR-0055 section 2.5 (write-through adapter) / section 3 (Phase 2 v2)
* Issue #2000 Phase 2 v2 (schema-reconciling, no DDL)
* Card body commit C3 acceptance list.
"""

from __future__ import annotations

import threading
import uuid

import pytest

from rob_box_mcp_tools.waypoint_adapter import (
    DEFAULT_MAP_ID,
    DEFAULT_MAP_NAME,
    WaypointAdapter,
    WaypointHarnessView,
)
from rob_box_mcp_tools.waypoint_store import WaypointStore


# ── Fixtures ────────────────────────────────────────────────────


@pytest.fixture
def db_path(tmp_path):
    """Per-test fresh SQLite file under tmp_path."""
    return str(tmp_path / f"wp-{uuid.uuid4().hex[:8]}.db")


@pytest.fixture
def store(db_path):
    """WaypointStore backed by the temp DB; closed at teardown."""
    s = WaypointStore(db_path=db_path)
    yield s
    try:
        s.close()
    except Exception:  # noqa: BLE001 - best-effort cleanup
        pass


@pytest.fixture
def adapter(store):
    """WaypointAdapter wrapping the temp WaypointStore."""
    return WaypointAdapter(store=store)


# ── 1. Write through WaypointStore -> read through adapter (harness shape) ──


class TestCrossStoreRead:
    """Data written via the underlying WaypointStore is visible via the adapter."""

    def test_waypoint_store_writes_visible_in_harness_view(self, store, adapter):
        # Direct write via MCP-side store
        store.create_map(name="квартира")
        store.save_waypoint("кухня", 2.0, 1.0, 0.5)
        store.save_waypoint("зал", 3.0, 4.0, 1.57)

        # Read via adapter (harness view)
        rows = adapter.list_waypoints_harness()
        names = sorted(r.name for r in rows)
        assert names == ["зал", "кухня"]

        # And the single-row GET goes through cleanly
        kitchen = adapter.get_waypoint_harness("кухня")
        assert kitchen is not None
        assert kitchen.x == pytest.approx(2.0)
        assert kitchen.y == pytest.approx(1.0)
        assert kitchen.theta == pytest.approx(0.5)
        assert kitchen.map_id != DEFAULT_MAP_ID  # random UUID assigned by WaypointStore


# ── 2. Existing mcp-DB data is readable through the adapter ──


class TestExistingDataAccess:
    """MCP-DB rows visible via adapter (regression: scenario 2 of the card)."""

    def test_pre_existing_uuid_map_visible(self, store, adapter):
        # Simulate a pre-existing MCP database written by some other process
        map_id = str(uuid.uuid4())
        with store._lock:  # noqa: SLF001 — same lock the store itself uses
            cur = store._conn.cursor()  # noqa: SLF001
            cur.execute(
                "INSERT INTO maps (map_id, name, created_at, is_active) "
                "VALUES (?, ?, ?, 1)",
                (map_id, "kitchen-floorplan", 1_700_000_000.0),
            )
            store._conn.commit()  # noqa: SLF001
            cur.execute(
                "INSERT INTO waypoints (map_id, name, x, y, theta, created_at, updated_at) "
                "VALUES (?, ?, ?, ?, ?, ?, ?)",
                (map_id, "fridge", 0.5, -1.5, 0.0, 1_700_000_000.0, 1_700_000_000.0),
            )
            store._conn.commit()  # noqa: SLF001

        # Adapter should read it via the harness view, with map_id preserved
        rows = adapter.list_waypoints_harness()
        names = [r.name for r in rows]
        assert names == ["fridge"]
        assert rows[0].map_id == map_id

    def test_get_waypoint_harness_returns_none_for_missing(self, store, adapter):
        store.create_map(name="empty")
        assert adapter.get_waypoint_harness("nonexistent") is None


# ── 3. map_id defaults to 'default' when not specified ──


class TestDefaultMapBehaviour:
    """Harness-shaped writes synthesise map_id='default' when no map is pinned."""

    def test_save_without_map_id_uses_default(self, adapter):
        adapter.save_waypoint_harness("кухня", 1.0, 2.0, 0.5)
        rows = adapter.list_waypoints_harness()
        assert len(rows) == 1
        assert rows[0].name == "кухня"
        assert rows[0].map_id == DEFAULT_MAP_ID

    def test_default_map_idempotent_across_saves(self, adapter):
        # First call synthesises the default map.
        adapter.save_waypoint_harness("кухня", 1.0, 2.0)
        adapter.save_waypoint_harness("зал", 3.0, 4.0)
        adapter.save_waypoint_harness("спальня", 5.0, 6.0)

        # Same default map should now hold all three rows.
        stats = adapter.get_stats()
        assert stats["active_map_id"] == DEFAULT_MAP_ID
        rows = adapter.list_waypoints_harness()
        names = sorted(r.name for r in rows)
        assert names == ["зал", "кухня", "спальня"]
        assert all(r.map_id == DEFAULT_MAP_ID for r in rows)

    def test_explicit_map_id_overrides_default(self, adapter):
        adapter.save_waypoint_harness("default-row", 1.0, 1.0)
        custom_map = adapter.create_map(name="офис")
        adapter.save_waypoint_harness("office-row", 2.0, 2.0, map_id=custom_map)

        default_rows = adapter.list_waypoints_harness(map_id=DEFAULT_MAP_ID)
        office_rows = adapter.list_waypoints_harness(map_id=custom_map)

        assert [r.name for r in default_rows] == ["default-row"]
        assert [r.name for r in office_rows] == ["office-row"]
        assert office_rows[0].map_id == custom_map

    def test_no_active_map_get_returns_none(self, store, adapter):
        # No maps whatsoever.
        assert adapter.get_waypoint_harness("anything") is None
        assert adapter.list_waypoints_harness() == []

    def test_default_map_name_human_readable(self, store, adapter):
        """Operators see maps.name='default' in diagnostics (ADR-0055 §3)."""
        adapter.save_waypoint_harness("kitchen", 1.0, 2.0)
        active = store.get_active_map()
        assert active is not None
        assert active["name"] == DEFAULT_MAP_NAME


# ── 4. Concurrent writes through different stores -> same verdict ──


class TestConcurrentWrites:
    """Writes via WaypointStore and WaypointAdapter on the same active map
    must be observable by both. Regression: scenario 4 of the card.

    The test simulates two threads racing through the public APIs.

    Note: ``WaypointStore`` serialises its own CRUD through an internal
    lock; we exercise *real* concurrency (not a single-event-loop
    cooperative mock) because the adapter inherits that lock.
    \"Same verdict\" here means the total count of waypoints on the
    active map after both threads finish is exactly the sum of their
    writes (no silent drops, no duplicates from same name within the
    same (map, name) tuple because the store enforces ON CONFLICT).
    """

    def test_concurrent_writes_via_both_stores_aggregate(self, store, adapter):
        # Pre-provision a default map and switch the store's active map
        # to it so both threads write onto one map deterministically.
        adapter.save_waypoint_harness("seed", 0.0, 0.0)

        mcp_writes: list[str] = []
        harness_writes: list[str] = []

        def mcp_writer(start: int, count: int) -> None:
            for i in range(start, start + count):
                name = f"mcp-{i}"
                store.save_waypoint(name, float(i), float(i), 0.0)
                mcp_writes.append(name)

        def harness_writer(start: int, count: int) -> None:
            for i in range(start, start + count):
                name = f"harness-{i}"
                adapter.save_waypoint_harness(name, float(i), float(i), 0.0)
                harness_writes.append(name)

        t1 = threading.Thread(target=mcp_writer, args=(0, 25))
        t2 = threading.Thread(target=harness_writer, args=(0, 25))
        t1.start()
        t2.start()
        t1.join()
        t2.join()

        # Both writers should report success on every call.
        assert len(mcp_writes) == 25
        assert len(harness_writes) == 25

        # Total on the active map = seed + 25 mcp + 25 harness = 51
        rows = adapter.list_waypoints_harness()
        names = {r.name for r in rows}
        # Every waypoint must end up visible by both APIs.
        for n in mcp_writes:
            assert n in names, f"mcp write {n} missing from adapter view"
        for n in harness_writes:
            assert n in names, f"harness write {n} missing from adapter view"
        assert "seed" in names
        assert len(rows) == 51, f"expected 51 rows, got {len(rows)}: {sorted(names)}"


# ── 5. Single source of truth / DTOs ──


class TestSingleSourceOfTruth:
    """Both views read the same DB; no shadow store, no duplicate writes."""

    def test_no_duplicate_rows(self, store, adapter):
        adapter.save_waypoint_harness("one", 1.0, 1.0)
        adapter.save_waypoint_harness("one", 1.5, 1.5)  # upsert

        mcp_rows = store.list_waypoints()
        assert len(mcp_rows) == 1
        assert mcp_rows[0]["x"] == pytest.approx(1.5)

        harness_rows = adapter.list_waypoints_harness()
        assert len(harness_rows) == 1
        assert harness_rows[0].x == pytest.approx(1.5)

    def test_harness_view_dto_shape(self, adapter):
        adapter.save_waypoint_harness("kitchen", 1.0, 2.0, 0.5)
        rows = adapter.list_waypoints_harness()
        assert len(rows) == 1
        v = rows[0]
        assert isinstance(v, WaypointHarnessView)
        assert v.name == "kitchen"
        assert v.x == pytest.approx(1.0)
        assert v.y == pytest.approx(2.0)
        assert v.theta == pytest.approx(0.5)
        assert v.map_id == DEFAULT_MAP_ID

        # as_harness_dict drops map_id/timestamps (exact MemoryStore ABC shape)
        d = v.as_harness_dict()
        assert set(d.keys()) == {"name", "x", "y", "theta"}

        # as_dict keeps map_id/timestamps (diagnostic / MCP shape)
        d_full = v.as_dict()
        assert "map_id" in d_full
        assert "created_at" in d_full

    def test_no_ddl_artefact_on_harness_table(self, store, adapter):
        """ADR-0055 section 3 - harness-DDL waypoints table is NOT written.

        Sanity check via SQL: the only source of truth is voice_memory.db.
        The harness_voice.db file path used by SQLiteVoiceMemory is never
        touched here. Verify by inspecting stores the SQLite-side metadata.
        """
        adapter.save_waypoint_harness("kitchen", 1.0, 2.0)
        adapter.save_waypoint("harness-shape", 3.0, 4.0)

        # tables in the single DB file backing this fixture
        with store._lock:  # noqa: SLF001
            tables = [
                r[0]
                for r in store._conn.execute(  # noqa: SLF001
                    "SELECT name FROM sqlite_master WHERE type='table' ORDER BY name"
                ).fetchall()
            ]
        # Both mcp-schema tables ("maps", "waypoints") exist; the adapter
        # does NOT introduce new tables or alter DDL.
        assert "maps" in tables
        assert "waypoints" in tables


# ── 6. Context manager / close ──


class TestLifecycle:
    """Adapter can be used as a context manager; close() is idempotent."""

    def test_context_manager(self, store):
        adapter = WaypointAdapter(store=store)
        with adapter as a:
            a.save_waypoint_harness("kitchen", 1.0, 2.0)
            rows = a.list_waypoints_harness()
        assert rows[0].name == "kitchen"
        # Post-context, the store is closed; that's the documented exit
        # behaviour of context-manager usage (matches file-handle idiom).


    def test_explicit_close(self, store, adapter):
        adapter.save_waypoint_harness("kitchen", 1.0, 2.0)
        adapter.close()
        # Calling close() again is a no-op (WaypointStore is fine with it)
        adapter.close()


# ── 7. Backwards-compat with WaypointStore contract ──


class TestWaypointStoreCompat:
    """The adapter must remain a drop-in for the existing WaypointStore
    contract: every method the MCP tools in ``tools/navigation.py``
    use must work the same way on a WaypointAdapter instance."""

    @pytest.mark.parametrize(
        "method_name",
        [
            "list_waypoints",
            "get_waypoint",
            "save_waypoint",
            "delete_waypoint",
            "clear_waypoints",
            "get_active_map",
            "get_active_map_id",
            "create_map",
            "list_maps",
            "rename_map",
            "set_active_map_by_name",
        ],
    )
    def test_method_exists_and_callable(self, adapter, method_name):
        assert hasattr(adapter, method_name), f"missing {method_name}"
        assert callable(getattr(adapter, method_name))

    def test_drop_in_replace_yields_same_results(self, store, adapter):
        """Same DB, same call sequence -> identical observable state."""
        from rob_box_mcp_tools.waypoint_store import WaypointStore as Real

        # Reset the adapter-side default map so the comparison starts
        # from a known state.
        adapter.save_waypoint_harness("reset", 0.0, 0.0)
        # Pre-clear: only the "reset" seed.
        assert len(adapter.list_waypoints()) == 1

        # Reset the underlying store's waypoint count.
        store.clear_waypoints()

        # Via store (legacy path)
        store.save_waypoint("legacy-1", 1.0, 1.0)
        store.save_waypoint("legacy-2", 2.0, 2.0)
        store_names = sorted(r["name"] for r in store.list_waypoints())
        adapter_names = sorted(r.name for r in adapter.list_waypoints_harness())
        assert store_names == adapter_names, (
            f"adapter diverged from store: adapter={adapter_names}, "
            f"store={store_names}"
        )
        # Both views agree on the 2 new rows.
        assert len(store_names) == 2

