#!/usr/bin/env python3
"""
waypoint_store.py — SQLite-backed CRUD for named waypoints, scoped per map.

Reuses the same database file as VoiceMemory (the tools DB, path from
``rob_box_core.tools_db.waypoint_db_path``: the tools DB unless ``WAYPOINT_DB_PATH``
moves the map/waypoint registry to its own file — HIL, #4371/#4392).
Migration ``003_waypoints.sql`` is applied idempotently on first init.

Кадр точки (#4390): колонка ``waypoints.frame_id`` — в каком кадре TF записаны
``x, y, theta`` (``save_waypoint`` пишет ``map`` из TF ``map ← base_footprint``).
В DDL 003 её нет: на существующей БД её добавляет
:meth:`WaypointStore._ensure_frame_id_column` (``ALTER TABLE ... ADD COLUMN``,
идемпотентно). Строки, записанные до этого, получают ``NULL`` — кадр
неизвестен (их писали из ``/odom``); данные не переписываются, трактовку
выбирает потребитель (``navigation.NavigateToWaypointTool``).

NOT on ``/data/harness_voice.db`` (issue #2000 / ADR-0055): the dialogue
node's ``SQLiteVoiceMemory`` already creates its own ``waypoints`` table
there with an incompatible schema (``name`` PRIMARY KEY, no ``map_id``).
Pointing this class at that file would make ``CREATE TABLE IF NOT EXISTS``
a no-op against the wrong schema and every insert here would then fail
with "no such column: map_id". Deliberate, documented blocker — not an
oversight. See the ``sqlite_db_path`` comment in
``docker/sound/config/voice_assistant/dialogue_node.yaml`` and
``docs/adr/0055-voice-memory-db-unify-with-harness.md``.

Thread-safe: every public method acquires ``self._lock``.

Usage (inside MCPServer)::

    store = WaypointStore()                    # waypoint_db_path()
    map_id = store.create_map("квартира")      # new mapping session
    store.save_waypoint("кухня", 2.0, 1.0, 0.0)
    store.list_waypoints()                     # → [{"name": "кухня", ...}]
    store.get_waypoint("кухня")                # → {"name": ..., "x": ..., ...}
    store.delete_waypoint("кухня")             # → True
    store.clear_waypoints()                    # → count deleted
"""

import os
import sqlite3
import threading
import time
import uuid
from typing import Dict, List, Optional

from rob_box_core import migrations as core_migrations
from rob_box_core.tools_db import waypoint_db_path

#: Схема карт и точек - шаг потока миграций ядра (данные пакета ``rob_box_core.migrations``, ADR-0167 Ш1а).
#: Раньше файл искался в корне чекаута (``parents[4]``), а на роботе его там не было - шла копия DDL в коде.
_MIGRATION_NAME = "003_waypoints.sql"


def map_name_key(name: Optional[str]) -> str:
    """Ключ сравнения имён карт: без пробелов по краям и без регистра (``lower()`` SQLite кириллицу не понижает)."""
    return (name or "").strip().casefold()


#: Колонка кадра точки (#4390); NULL — точка записана до неё, кадр неизвестен.
_FRAME_ID_COLUMN = "frame_id"


class WaypointStore:
    """SQLite CRUD for maps and waypoints."""

    def __init__(self, db_path: Optional[str] = None) -> None:
        self._db_path = db_path or waypoint_db_path()
        self._lock = threading.Lock()

        # Ensure parent directory exists
        os.makedirs(os.path.dirname(self._db_path) or ".", exist_ok=True)

        self._conn = sqlite3.connect(self._db_path, check_same_thread=False)
        self._conn.execute("PRAGMA journal_mode=WAL")
        self._conn.execute("PRAGMA foreign_keys=ON")
        self._conn.row_factory = sqlite3.Row

        self._apply_migration()

    # ------------------------------------------------------------------
    # Migration
    # ------------------------------------------------------------------

    def _apply_migration(self) -> None:
        """Run 003_waypoints DDL (idempotent thanks to IF NOT EXISTS)."""
        ddl = core_migrations.script(_MIGRATION_NAME)

        with self._lock:
            cur = self._conn.cursor()
            cur.executescript(ddl)
            self._conn.commit()
        self._ensure_frame_id_column()

    def _ensure_frame_id_column(self) -> None:
        """Добавить ``waypoints.frame_id`` (TEXT NULL), если её нет; старые строки не трогает."""
        with self._lock:
            columns = {row["name"] for row in self._conn.execute("PRAGMA table_info(waypoints)")}
            if _FRAME_ID_COLUMN not in columns:
                self._conn.execute(f"ALTER TABLE waypoints ADD COLUMN {_FRAME_ID_COLUMN} TEXT")
                self._conn.commit()

    # ------------------------------------------------------------------
    # Map management
    # ------------------------------------------------------------------

    def create_map(self, name: Optional[str] = None) -> str:
        """Create a new map entry and make it the active map.

        Returns the generated ``map_id`` (UUID4 string).
        """
        map_id = str(uuid.uuid4())
        now = time.time()

        with self._lock:
            cur = self._conn.cursor()
            # Deactivate all previous maps
            cur.execute("UPDATE maps SET is_active = 0 WHERE is_active = 1")
            cur.execute(
                "INSERT INTO maps (map_id, name, created_at, is_active) VALUES (?, ?, ?, 1)",
                (map_id, name, now),
            )
            self._conn.commit()
        return map_id

    def get_active_map_id(self) -> Optional[str]:
        """Return the ``map_id`` of the current active map, or *None*."""
        with self._lock:
            row = self._conn.execute(
                "SELECT map_id FROM maps WHERE is_active = 1 LIMIT 1"
            ).fetchone()
        return row["map_id"] if row else None

    def get_active_map(self) -> Optional[Dict]:
        """Return full row for the active map, or *None*."""
        with self._lock:
            row = self._conn.execute(
                "SELECT map_id, name, created_at FROM maps WHERE is_active = 1 LIMIT 1"
            ).fetchone()
        if row is None:
            return None
        return {"map_id": row["map_id"], "name": row["name"], "created_at": row["created_at"]}

    def rename_map(self, map_id: str, name: str) -> bool:
        """Set a human-friendly name for a map.  Returns True if updated."""
        with self._lock:
            cur = self._conn.execute(
                "UPDATE maps SET name = ? WHERE map_id = ?", (name, map_id)
            )
            self._conn.commit()
        return cur.rowcount > 0

    def ensure_active_map(self) -> str:
        """Return active map_id, creating a default map if none exists."""
        map_id = self.get_active_map_id()
        if map_id is None:
            map_id = self.create_map(name="default")
        return map_id

    def delete_inactive_maps_by_name(self, name: str) -> int:
        """Удалить строки карт с именем ``name`` (без учёта регистра), кроме активной; их точки - каскадом.

        Активную карту так не удалить (#4392): её геометрия - единственная база rtabmap, строку-описание без
        базы оставить нельзя. Возвращает число удалённых строк (``0`` - таких неактивных карт нет).
        """
        wanted = map_name_key(name)
        if not wanted:
            return 0
        with self._lock:
            rows = self._conn.execute("SELECT map_id, name FROM maps WHERE is_active = 0").fetchall()
            doomed = [r["map_id"] for r in rows if map_name_key(r["name"]) == wanted]
            cur = self._conn.cursor()
            for map_id in doomed:
                cur.execute("DELETE FROM waypoints WHERE map_id = ?", (map_id,))
                cur.execute("DELETE FROM maps WHERE map_id = ?", (map_id,))
            self._conn.commit()
        return len(doomed)

    def list_maps(self) -> list:
        """Return all maps ordered by creation time (newest first)."""
        with self._lock:
            rows = self._conn.execute(
                "SELECT map_id, name, created_at, is_active FROM maps ORDER BY created_at DESC, id DESC"
            ).fetchall()
        return [
            {"map_id": r["map_id"], "name": r["name"], "created_at": r["created_at"], "is_active": bool(r["is_active"])}
            for r in rows
        ]

    # ------------------------------------------------------------------
    # Waypoint CRUD
    # ------------------------------------------------------------------

    def save_waypoint(
        self, name: str, x: float, y: float, theta: float = 0.0, frame_id: Optional[str] = None,
    ) -> bool:
        """Insert or update a waypoint on the active map.

        ``frame_id`` — кадр TF координат (#4390); ``None`` — неизвестен.
        Creates a default map if none is active.  Returns True on success.
        """
        map_id = self.ensure_active_map()
        now = time.time()
        name_lower = name.strip().lower()

        with self._lock:
            self._conn.execute(
                """
                INSERT INTO waypoints (map_id, name, x, y, theta, frame_id, created_at, updated_at)
                VALUES (?, ?, ?, ?, ?, ?, ?, ?)
                ON CONFLICT(map_id, name) DO UPDATE SET
                    x = excluded.x,
                    y = excluded.y,
                    theta = excluded.theta,
                    frame_id = excluded.frame_id,
                    updated_at = excluded.updated_at
                """,
                (map_id, name_lower, x, y, theta, frame_id, now, now),
            )
            self._conn.commit()
        return True

    def get_waypoint(self, name: str) -> Optional[Dict]:
        """Look up a single waypoint by name on the active map.

        ``frame_id`` в ответе — кадр координат, ``None`` у точек до #4390.
        """
        map_id = self.get_active_map_id()
        if map_id is None:
            return None

        name_lower = name.strip().lower()
        with self._lock:
            row = self._conn.execute(
                "SELECT name, x, y, theta, frame_id FROM waypoints WHERE map_id = ? AND name = ?",
                (map_id, name_lower),
            ).fetchone()

        if row is None:
            return None
        return {
            "name": row["name"], "x": row["x"], "y": row["y"], "theta": row["theta"], "frame_id": row["frame_id"],
        }

    def list_waypoints(self) -> List[Dict]:
        """Return all waypoints for the active map, ordered by name."""
        map_id = self.get_active_map_id()
        if map_id is None:
            return []

        with self._lock:
            rows = self._conn.execute(
                "SELECT name, x, y, theta FROM waypoints WHERE map_id = ? ORDER BY name",
                (map_id,),
            ).fetchall()

        return [{"name": r["name"], "x": r["x"], "y": r["y"], "theta": r["theta"]} for r in rows]

    def delete_waypoint(self, name: str) -> bool:
        """Delete a waypoint by name on the active map.  Returns True if deleted."""
        map_id = self.get_active_map_id()
        if map_id is None:
            return False

        name_lower = name.strip().lower()
        with self._lock:
            cur = self._conn.execute(
                "DELETE FROM waypoints WHERE map_id = ? AND name = ?",
                (map_id, name_lower),
            )
            self._conn.commit()
        return cur.rowcount > 0

    def clear_waypoints(self) -> int:
        """Delete ALL waypoints on the active map.  Returns count deleted."""
        map_id = self.get_active_map_id()
        if map_id is None:
            return 0

        with self._lock:
            cur = self._conn.execute(
                "DELETE FROM waypoints WHERE map_id = ?", (map_id,)
            )
            self._conn.commit()
        return cur.rowcount

    # ------------------------------------------------------------------
    # Convenience
    # ------------------------------------------------------------------

    def close(self) -> None:
        """Close the database connection."""
        with self._lock:
            self._conn.close()
