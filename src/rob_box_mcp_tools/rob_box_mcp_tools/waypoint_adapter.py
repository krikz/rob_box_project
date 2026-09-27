"""WaypointAdapter — schema-reconciling facade for waypoint storage.

Issue #2000 / ADR-0055 §3 — Phase 2 v2 (schema-reconciling adapter,
no DDL changes).

The robot ships two SQLite databases on ``/data`` with **incompatible
``waypoints`` schemas**:

* ``voice_memory.db`` (the MCP-side file, ``WaypointStore``'s home):
  ``waypoints(id PK, map_id NOT NULL FK -> maps, name, x, y, theta,
  created_at, updated_at, UNIQUE(map_id, name))``. Each waypoint
  belongs to a **map** (a mapping session identified by UUID).
* ``harness_voice.db`` (the dialogue-node file, ``SQLiteVoiceMemory``'s
  home): ``waypoints(name PK, x, y, theta, created_at, updated_at)``.
  No concept of maps. This card **must not** modify its schema
  (ADR-0055 §3 forbids DDL edits; the directive Shifu confirmed at
  commit ``52eb826a7``).

This adapter is the **single public API for waypoint persistence** at
the MCP layer. It reads & writes only through ``WaypointStore`` (which
already owns the MCP-schema DDL — see
``migrations/003_waypoints.sql`` and the inline DDL in
``waypoint_store.py``). It exposes **two views** of the same data so
existing and future callers don't have to know which file they're
talking to:

* **MCP view** (``list_waypoints`` / ``get_waypoint`` / …) — every
  row carries ``map_id``. Matches the existing ``WaypointStore``
  contract 1:1, so the MCP tools in ``tools/navigation.py`` keep
  working unchanged.
* **Harness view** (``list_waypoints_harness`` /
  ``get_waypoint_harness`` / …) — rows shaped like
  ``rob_box_harness.memory.Waypoint`` (``name, x, y, theta``), with
  ``map_id`` defaulted to ``"default"`` when callers don't pin a
  specific map. This is the contract Phase 2 (AgentCore step 03)
  will use from ``SQLiteVoiceMemory``-side code without ever having
  to write to ``harness_voice.db.waypoints``.

Why the harness view, if we never write there?
----------------------------------------------
Forward-compatibility for the step-03 ``AgentCore`` namespace work.
``SQLiteVoiceMemory`` will gain an ``agent`` column and a unified
``waypoints`` view that AgentCore tools can read. When that lands,
AgentCore will read through *this* adapter, not the harness-side
table — preserving the single-source-of-truth invariant (mcp-DB) and
sidestepping the schema collision that blocks a naive
``ATTACH+INSERT...SELECT`` data-migration today (ADR-0055 §1.3, §3).

**Proxy writes**
----------------
The card body (commit C2) prescribes: «если пишут в harness-стор →
транслируем в mcp-стор (создаём map_id='default')». The adapter
implements that as :meth:`save_waypoint_harness`: it accepts the
harness signature ``(name, x, y, theta)``, resolves an active map
(synthesising ``map_id='default'`` if none exists), and forwards to
``WaypointStore.save_waypoint`` — which writes through to the
mcp-schema. The harness-DDL side is **not** written to by this
adapter (DL on that table is forbidden and there's no data-migration
in scope). When the future Phase 2 work eventually moves reads off
the harness table, ``save_waypoint_harness`` keeps working without
code changes — it already targets mcp-DB.

Thread safety
-------------
``WaypointStore`` is itself thread-safe (its own ``self._lock``
serialises every CRUD call). The adapter inherits that — it does
NOT add a second lock (double-locking would just serialise the same
connection's I/O twice and produce no benefit).
"""

from __future__ import annotations

import logging
from dataclasses import dataclass, field
from typing import Any, Dict, Iterator, List, Optional

from .waypoint_store import WaypointStore

_logger = logging.getLogger(__name__)


# ``map_id`` synthesised by the adapter when a harness-shaped caller
# (``save_waypoint_harness``) does not pin one. Maps to a row in
# ``WaypointStore``'s ``maps`` table; the row's ``name`` matches this
# constant so operators can spot it in diagnostics. Stable across
# runs by virtue of the UNIQUE(map_id) constraint.
DEFAULT_MAP_ID = "default"

# Name shown for the synthesized default map. Stored as ``maps.name``
# so ``list_maps`` returns something human-readable.
DEFAULT_MAP_NAME = "default"


# ── DTOs ────────────────────────────────────────────────────────


@dataclass(frozen=True)
class WaypointHarnessView:
    """Waypoint in ``rob_box_harness.memory.Waypoint``-compatible shape.

    Used by AgentCore / operator-agent side code that expects the
    harness MemoryStore contract (``name, x, y, theta``). ``map_id``
    is a read-only diagnostic — harness callers can ignore it; the
    value is whichever map the row lives under, defaulted to
    :data:`DEFAULT_MAP_ID` when the row was created via
    :meth:`WaypointAdapter.save_waypoint_harness`.
    """

    name: str
    x: float
    y: float
    theta: float
    map_id: str = DEFAULT_MAP_ID
    created_at: Optional[float] = None
    updated_at: Optional[float] = None

    def as_dict(self) -> Dict[str, Any]:
        """Plain-dict for JSON serialisation in MCP tool results."""
        return {
            "name": self.name,
            "x": self.x,
            "y": self.y,
            "theta": self.theta,
            "map_id": self.map_id,
            "created_at": self.created_at,
            "updated_at": self.updated_at,
        }

    def as_harness_dict(self) -> Dict[str, Any]:
        """Strip ``map_id``/timestamps — exact shape ``Waypoint`` uses."""
        return {"name": self.name, "x": self.x, "y": self.y, "theta": self.theta}


# ── Adapter ────────────────────────────────────────────────────


class WaypointAdapter:
    """Schema-reconciling facade over :class:`WaypointStore`.

    Reads and writes always go through the MCP-schema file
    (``voice_memory.db``). The harness-side
    ``harness_voice.db.waypoints`` table is **not** touched — ADR-0055
    §3 forbids DDL changes there. Future AgentCore-side code reads
    via the harness-view methods of this adapter (``map_id`` defaults
    to ``"default"`` for rows created through ``save_waypoint_harness``).

    Parameters
    ----------
    store:
        A live :class:`WaypointStore`. Pass ``None`` to construct a
        default one (which reads ``VOICE_MEMORY_DB_PATH`` / defaults
        to ``/data/voice_memory.db`` — same fallback
        :class:`WaypointStore` already uses today; this adapter does
        NOT introduce a new knob).
    default_map_id:
        ``map_id`` used by :meth:`save_waypoint_harness` when no map
        exists yet. Defaults to :data:`DEFAULT_MAP_ID`. Override only
        in tests.
    """

    def __init__(
        self,
        store: Optional[WaypointStore] = None,
        *,
        default_map_id: str = DEFAULT_MAP_ID,
        default_map_name: str = DEFAULT_MAP_NAME,
    ) -> None:
        self._store: WaypointStore = store if store is not None else WaypointStore()
        self._default_map_id: str = default_map_id
        self._default_map_name: str = default_map_name
        _logger.info(
            "WaypointAdapter initialised (db=%s, default_map_id=%s)",
            self._store._db_path,  # noqa: SLF001 — diagnostic only
            self._default_map_id,
        )

    # ── store access ─────────────────────────────────────────────

    @property
    def store(self) -> WaypointStore:
        """The underlying :class:`WaypointStore` (lazy-mutable for tests)."""
        return self._store

    # ── MCP view (existing WaypointStore contract) ──────────────

    def list_waypoints(self) -> List[Dict[str, Any]]:
        """MCP shape — ``[{"name", "x", "y", "theta"}]`` on active map."""
        return self._store.list_waypoints()

    def get_waypoint(self, name: str) -> Optional[Dict[str, Any]]:
        """MCP shape — ``{"name", "x", "y", "theta"}`` or ``None``."""
        return self._store.get_waypoint(name)

    def save_waypoint(
        self,
        name: str,
        x: float,
        y: float,
        theta: float = 0.0,
    ) -> bool:
        """Write through to MCP schema (active map). Returns ``True``."""
        return self._store.save_waypoint(name, x, y, theta)

    def delete_waypoint(self, name: str) -> bool:
        """Delete on the active map. ``True`` if a row was deleted."""
        return self._store.delete_waypoint(name)

    def clear_waypoints(self) -> int:
        """Wipe every waypoint on the active map. Returns count removed."""
        return self._store.clear_waypoints()

    def get_active_map(self) -> Optional[Dict[str, Any]]:
        """Active map row (``map_id, name, created_at``) or ``None``."""
        return self._store.get_active_map()

    def get_active_map_id(self) -> Optional[str]:
        """Active ``map_id`` UUID or ``None``. Mirrors ``WaypointStore``."""
        return self._store.get_active_map_id()

    def set_active_map_by_name(self, name: str) -> bool:
        """Switch active map by name (case-insensitive). Pass-through."""
        return self._store.set_active_map_by_name(name)

    def rename_map(self, map_id: str, name: str) -> bool:
        """Rename a map by ``map_id``. Pass-through."""
        return self._store.rename_map(map_id, name)

    def list_maps(self) -> List[Dict[str, Any]]:
        """All maps, newest first — convenience pass-through."""
        return self._store.list_maps()

    def create_map(self, name: Optional[str] = None) -> str:
        """Create a new map, make it active. Returns the new ``map_id``."""
        return self._store.create_map(name=name)

    def ensure_active_map(self) -> str:
        """Return the active ``map_id``, creating a default map if none."""
        if (existing := self._store.get_active_map_id()) is not None:
            return existing
        return self._store.create_map(name=self._default_map_name)

    # ── Harness view (for Phase 2 / AgentCore step 03) ──────────

    def list_waypoints_harness(
        self,
        *,
        map_id: Optional[str] = None,
    ) -> List[WaypointHarnessView]:
        """Return rows shaped like :class:`WaypointHarnessView`.

        If ``map_id`` is ``None`` (default), reads from the active
        map; if there is no active map, returns ``[]`` (mirrors
        :meth:`WaypointStore.list_waypoints` — empty-list rather than
        auto-create, so read-only callers don't accidentally materialise
        state).
        """
        target_map_id = map_id or self._store.get_active_map_id()
        if target_map_id is None:
            return []

        rows = self._store.list_waypoints()  # active map
        if map_id is not None and map_id != self._store.get_active_map_id():
            # Non-active map requested — re-query directly via SQL
            # through the store (no public helper, so we go through
            # list_waypoints on a switched active map; switching
            # changes global state, so we avoid it and do a targeted
            # read instead).
            rows = self._read_waypoints_for_map(target_map_id)

        return [
            WaypointHarnessView(
                name=row["name"],
                x=row["x"],
                y=row["y"],
                theta=row["theta"],
                map_id=target_map_id,
            )
            for row in rows
        ]

    def get_waypoint_harness(
        self,
        name: str,
        *,
        map_id: Optional[str] = None,
    ) -> Optional[WaypointHarnessView]:
        """Harness-shape lookup. ``None`` if missing on the resolved map."""
        target_map_id = map_id or self._store.get_active_map_id()
        if target_map_id is None:
            return None

        if map_id is None:
            row = self._store.get_waypoint(name)
        else:
            row = self._read_waypoint_for_map(name, target_map_id)

        if row is None:
            return None
        return WaypointHarnessView(
            name=row["name"],
            x=row["x"],
            y=row["y"],
            theta=row["theta"],
            map_id=target_map_id,
        )

    def save_waypoint_harness(
        self,
        name: str,
        x: float,
        y: float,
        theta: float = 0.0,
        *,
        map_id: Optional[str] = None,
    ) -> bool:
        """Harness-shaped write — translated to MCP schema.

        * ``map_id=None`` (default) → resolves to :data:`DEFAULT_MAP_ID`,
          created on first use (``maps.name="default"``). Subsequent
          writes hit the existing map.
        * ``map_id=<uuid>`` → delegates to :meth:`WaypointStore.save_waypoint`
          after ensuring that map exists; ties into mcp-side
          ``create_map``/``ensure_active_map`` machinery.
        """
        target_map_id = map_id or self._resolve_or_create_default_map()
        # Delegate to the underlying store. WaypointStore always writes
        # to the *active* map, so we set active = target_map_id first
        # when it differs. ``WaypointStore`` does not expose a public
        # ``set_active_map(map_id)`` yet (only ``set_active_map_by_name``),
        # so we use the name-based setter when the target isn't already
        # active.
        current_active = self._store.get_active_map_id()
        if current_active != target_map_id:
            if not self._activate_map(target_map_id):
                raise RuntimeError(
                    f"WaypointAdapter: cannot activate target map_id={target_map_id!r}"
                )

        return self._store.save_waypoint(name, x, y, theta)

    def delete_waypoint_harness(
        self,
        name: str,
        *,
        map_id: Optional[str] = None,
    ) -> bool:
        target_map_id = map_id or self._store.get_active_map_id()
        if target_map_id is None:
            return False
        if map_id is None:
            return self._store.delete_waypoint(name)
        current_active = self._store.get_active_map_id()
        if current_active != target_map_id:
            if not self._activate_map(target_map_id):
                return False
        return self._store.delete_waypoint(name)

    def clear_waypoints_harness(
        self,
        *,
        map_id: Optional[str] = None,
    ) -> int:
        target_map_id = map_id or self._store.get_active_map_id()
        if target_map_id is None:
            return 0
        if map_id is None:
            return self._store.clear_waypoints()
        current_active = self._store.get_active_map_id()
        if current_active != target_map_id:
            if not self._activate_map(target_map_id):
                return 0
        return self._store.clear_waypoints()

    # ── diagnostics ──────────────────────────────────────────────

    def get_stats(self) -> Dict[str, Any]:
        """Compact diagnostic — used by acceptance tests."""
        active_map = self._store.get_active_map()
        waypoints = self._store.list_waypoints()
        return {
            "active_map_id": self._store.get_active_map_id(),
            "active_map_name": (active_map or {}).get("name"),
            "waypoint_count": len(waypoints),
            "db_path": self._store._db_path,  # noqa: SLF001 — diagnostic only
        }

    # ── context manager (lets tests/operators close cleanly) ────

    def close(self) -> None:
        """Close the underlying connection (idempotent)."""
        self._store.close()

    def __enter__(self) -> "WaypointAdapter":
        return self

    def __exit__(self, exc_type: Any, exc: Any, tb: Any) -> None:
        self.close()

    # ── private helpers ──────────────────────────────────────────

    def _resolve_or_create_default_map(self) -> str:
        """Return the ``map_id`` to use as the default for harness-shaped writes.

        Strategy:

        1. If a row with ``map_id == self._default_map_id`` (e.g.
           pre-seeded by a previous run, or by an operator) exists,
           activate it and return — no new UUID is allocated.
        2. Otherwise, if there are no maps at all, inject the default
           map **before** anyone calls ``WaypointStore.save_waypoint``
           (which would otherwise auto-create a UUID-named map and
           forever leak that UUID into the harness view).
        3. Otherwise, if an active map already exists but it's not
           the default map, reuse it — operators generally intended
           that map for actual waypoints.

        The synthesis is idempotent: the ``maps.map_id`` UNIQUE
        constraint would reject duplicates anyway.
        """
        # 1. Default map exists? Activate + return.
        active = self._store.get_active_map_id()
        for m in self._store.list_maps():
            if m.get("map_id") == self._default_map_id:
                if active != m["map_id"]:
                    self._activate_map(m["map_id"])
                return m["map_id"]

        # 2. No maps at all → insert the default map row directly with
        #    the explicit map_id; WaypointStore.create_map would otherwise
        #    allocate a fresh UUID. We still need to mark it is_active=1
        #    so subsequent WaypointStore.save_waypoint (which always
        #    targets the active map) writes onto it.
        try:
            now = __import__("time").time()
            with self._store._lock:  # noqa: SLF001
                cur = self._store._conn.cursor()  # noqa: SLF001
                cur.execute("UPDATE maps SET is_active = 0 WHERE is_active = 1")
                cur.execute(
                    "INSERT INTO maps (map_id, name, created_at, is_active) "
                    "VALUES (?, ?, ?, 1)",
                    (self._default_map_id, self._default_map_name, now),
                )
                self._store._conn.commit()  # noqa: SLF001
            return self._default_map_id
        except Exception:  # noqa: BLE001 — race: someone else just made one
            pass

        # 3. Fallback — reuse whatever's active (or re-scan by name).
        active = self._store.get_active_map_id()
        if active is not None:
            return active
        for m in self._store.list_maps():
            if m.get("name") == self._default_map_name:
                return m["map_id"]
        raise RuntimeError("WaypointAdapter: failed to provision default map")

    def _activate_map(self, map_id: str) -> bool:
        """Switch the active map to ``map_id`` via the only public setter that
        works by id (``set_active_map_by_name`` only takes a name).

        WaypointStore currently has no ``set_active_map(map_id)``,
        so we toggle via the raw connection under the store's lock.
        This is acceptable: it's the same connection WaypointStore
        already owns and uses synchronously.
        """
        try:
            with self._store._lock:  # noqa: SLF001 — same lock as store's own CRUD
                cur = self._store._conn.cursor()  # noqa: SLF001
                cur.execute("UPDATE maps SET is_active = 0 WHERE is_active = 1")
                cur.execute(
                    "UPDATE maps SET is_active = 1 WHERE map_id = ?",
                    (map_id,),
                )
                self._store._conn.commit()  # noqa: SLF001
                return cur.rowcount > 0 or self._store.get_active_map_id() == map_id
        except Exception as exc:  # noqa: BLE001 — best-effort activation
            _logger.warning("WaypointAdapter: failed to activate map %r: %s", map_id, exc)
            return False

    def _read_waypoints_for_map(self, map_id: str) -> List[Dict[str, Any]]:
        """Read every waypoint on ``map_id`` regardless of active state."""
        try:
            with self._store._lock:  # noqa: SLF001
                rows = self._store._conn.execute(  # noqa: SLF001
                    "SELECT name, x, y, theta FROM waypoints "
                    "WHERE map_id = ? ORDER BY name",
                    (map_id,),
                ).fetchall()
            return [
                {"name": r["name"], "x": r["x"], "y": r["y"], "theta": r["theta"]}
                for r in rows
            ]
        except Exception as exc:  # noqa: BLE001
            _logger.warning("WaypointAdapter: read failed for map %r: %s", map_id, exc)
            return []

    def _read_waypoint_for_map(
        self,
        name: str,
        map_id: str,
    ) -> Optional[Dict[str, Any]]:
        try:
            with self._store._lock:  # noqa: SLF001
                row = self._store._conn.execute(  # noqa: SLF001
                    "SELECT name, x, y, theta FROM waypoints "
                    "WHERE map_id = ? AND name = ?",
                    (map_id, name.strip().lower()),
                ).fetchone()
            if row is None:
                return None
            return {
                "name": row["name"],
                "x": row["x"],
                "y": row["y"],
                "theta": row["theta"],
            }
        except Exception as exc:  # noqa: BLE001
            _logger.warning("WaypointAdapter: get failed for %r/%r: %s", map_id, name, exc)
            return None


__all__ = [
    "DEFAULT_MAP_ID",
    "DEFAULT_MAP_NAME",
    "WaypointAdapter",
    "WaypointHarnessView",
]
