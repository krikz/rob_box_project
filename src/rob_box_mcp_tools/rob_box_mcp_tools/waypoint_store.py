#!/usr/bin/env python3
"""
waypoint_store.py — SQLite-backed CRUD for named waypoints, scoped per map.

Reuses the same database file as VoiceMemory (``VOICE_MEMORY_DB_PATH``).
Migration ``003_waypoints.sql`` is applied idempotently on first init.

Thread-safe: every public method acquires ``self._lock``.

Usage (inside MCPServer)::

    store = WaypointStore()                    # reads VOICE_MEMORY_DB_PATH
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
from pathlib import Path
from typing import Dict, List, Optional


# Path to 003_waypoints.sql relative to the package root.
# At runtime inside the Docker container the migrations folder may not exist,
# so we embed the DDL as a fallback.
_MIGRATION_DIR = Path(__file__).resolve().parents[4] / "migrations"
_MIGRATION_FILE = _MIGRATION_DIR / "003_waypoints.sql"

_INLINE_DDL = """\
CREATE TABLE IF NOT EXISTS maps (
    id         INTEGER PRIMARY KEY AUTOINCREMENT,
    map_id     TEXT    NOT NULL UNIQUE,
    name       TEXT,
    created_at REAL    NOT NULL,
    is_active  INTEGER NOT NULL DEFAULT 0
);
CREATE INDEX IF NOT EXISTS idx_maps_active ON maps(is_active);

CREATE TABLE IF NOT EXISTS waypoints (
    id         INTEGER PRIMARY KEY AUTOINCREMENT,
    map_id     TEXT    NOT NULL,
    name       TEXT    NOT NULL,
    x          REAL    NOT NULL,
    y          REAL    NOT NULL,
    theta      REAL    NOT NULL DEFAULT 0.0,
    created_at REAL    NOT NULL,
    updated_at REAL    NOT NULL,
    UNIQUE(map_id, name),
    FOREIGN KEY (map_id) REFERENCES maps(map_id) ON DELETE CASCADE
);
CREATE INDEX IF NOT EXISTS idx_wp_map ON waypoints(map_id);
"""


class WaypointStore:
    """SQLite CRUD for maps and waypoints."""

    def __init__(self, db_path: Optional[str] = None) -> None:
        self._db_path = db_path or os.getenv("VOICE_MEMORY_DB_PATH", "/data/voice_memory.db")
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
        ddl = _INLINE_DDL
        if _MIGRATION_FILE.exists():
            ddl = _MIGRATION_FILE.read_text(encoding="utf-8")

        with self._lock:
            cur = self._conn.cursor()
            cur.executescript(ddl)
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

    def set_active_map_by_name(self, name: str) -> bool:
        """Find a map by name (case-insensitive) and make it active.

        Returns True if found and activated, False if no map with that name.
        """
        name_lower = name.strip().lower()
        with self._lock:
            row = self._conn.execute(
                "SELECT map_id FROM maps WHERE lower(name) = ? ORDER BY created_at DESC LIMIT 1",
                (name_lower,),
            ).fetchone()
            if row is None:
                return False
            map_id = row["map_id"]
            cur = self._conn.cursor()
            cur.execute("UPDATE maps SET is_active = 0 WHERE is_active = 1")
            cur.execute("UPDATE maps SET is_active = 1 WHERE map_id = ?", (map_id,))
            self._conn.commit()
        return True

    def list_maps(self) -> list:
        """Return all maps ordered by creation time (newest first)."""
        with self._lock:
            rows = self._conn.execute(
                "SELECT map_id, name, created_at, is_active FROM maps ORDER BY created_at DESC"
            ).fetchall()
        return [
            {"map_id": r["map_id"], "name": r["name"], "created_at": r["created_at"], "is_active": bool(r["is_active"])}
            for r in rows
        ]

    # ------------------------------------------------------------------
    # Waypoint CRUD
    # ------------------------------------------------------------------

    def save_waypoint(self, name: str, x: float, y: float, theta: float = 0.0) -> bool:
        """Insert or update a waypoint on the active map.

        Creates a default map if none is active.  Returns True on success.
        """
        map_id = self.ensure_active_map()
        now = time.time()
        name_lower = name.strip().lower()

        with self._lock:
            self._conn.execute(
                """
                INSERT INTO waypoints (map_id, name, x, y, theta, created_at, updated_at)
                VALUES (?, ?, ?, ?, ?, ?, ?)
                ON CONFLICT(map_id, name) DO UPDATE SET
                    x = excluded.x,
                    y = excluded.y,
                    theta = excluded.theta,
                    updated_at = excluded.updated_at
                """,
                (map_id, name_lower, x, y, theta, now, now),
            )
            self._conn.commit()
        return True

    def get_waypoint(self, name: str) -> Optional[Dict]:
        """Look up a single waypoint by name on the active map."""
        map_id = self.get_active_map_id()
        if map_id is None:
            return None

        name_lower = name.strip().lower()
        with self._lock:
            row = self._conn.execute(
                "SELECT name, x, y, theta FROM waypoints WHERE map_id = ? AND name = ?",
                (map_id, name_lower),
            ).fetchone()

        if row is None:
            return None
        return {"name": row["name"], "x": row["x"], "y": row["y"], "theta": row["theta"]}

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
