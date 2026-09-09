#!/usr/bin/env python3
"""
generated_music_library.py — persistent storage for MiniMax-generated tracks.

Each track lives in its own directory under ``root_dir`` (default
``/data/music_library/``):

    /data/music_library/<track_id>/
        track.mp3     — raw mp3 bytes from MiniMax
        meta.json     — human-readable metadata

A single SQLite index at ``root_dir/index.db`` keeps the canonical record so
list/search/delete don't have to walk the filesystem. Schema:

    CREATE TABLE generated_tracks (
        id              TEXT PRIMARY KEY,        -- uuid4
        title           TEXT NOT NULL DEFAULT '',
        prompt          TEXT NOT NULL,
        lyrics          TEXT NOT NULL DEFAULT '',
        model           TEXT NOT NULL DEFAULT '',
        duration_ms     INTEGER NOT NULL DEFAULT 0,
        sample_rate     INTEGER NOT NULL DEFAULT 44100,
        bitrate         INTEGER NOT NULL DEFAULT 256000,
        file_size       INTEGER NOT NULL DEFAULT 0,
        tags            TEXT NOT NULL DEFAULT '[]',  -- JSON list
        mood            TEXT NOT NULL DEFAULT '',
        genre           TEXT NOT NULL DEFAULT '',
        lang            TEXT NOT NULL DEFAULT '',
        is_instrumental INTEGER NOT NULL DEFAULT 0,
        play_count      INTEGER NOT NULL DEFAULT 0,
        rating          INTEGER NOT NULL DEFAULT 0,
        notes           TEXT NOT NULL DEFAULT '',
        created_at      TEXT NOT NULL,          -- ISO 8601 UTC
        updated_at      TEXT NOT NULL
    );

The library is **thread-safe** (single ``threading.Lock``) and uses
``check_same_thread=False`` for the connection so it works from ROS executor
threads.

Designed to be ROS-free (no rclpy imports) so it can be unit-tested without a
running daemon — mirrors ``core/minimax_music_client.py``.
"""

from __future__ import annotations

import json
import os
import sqlite3
import threading
import uuid
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional


_DEFAULT_ROOT = os.getenv(
    "MUSIC_LIBRARY_PATH",
    "/data/music_library",
)


_SCHEMA_SQL = """
CREATE TABLE IF NOT EXISTS generated_tracks (
    id              TEXT PRIMARY KEY,
    title           TEXT NOT NULL DEFAULT '',
    prompt          TEXT NOT NULL,
    lyrics          TEXT NOT NULL DEFAULT '',
    model           TEXT NOT NULL DEFAULT '',
    duration_ms     INTEGER NOT NULL DEFAULT 0,
    sample_rate     INTEGER NOT NULL DEFAULT 44100,
    bitrate         INTEGER NOT NULL DEFAULT 256000,
    file_size       INTEGER NOT NULL DEFAULT 0,
    tags            TEXT NOT NULL DEFAULT '[]',
    mood            TEXT NOT NULL DEFAULT '',
    genre           TEXT NOT NULL DEFAULT '',
    lang            TEXT NOT NULL DEFAULT '',
    is_instrumental INTEGER NOT NULL DEFAULT 0,
    play_count      INTEGER NOT NULL DEFAULT 0,
    rating          INTEGER NOT NULL DEFAULT 0,
    notes           TEXT NOT NULL DEFAULT '',
    created_at      TEXT NOT NULL,
    updated_at      TEXT NOT NULL
);

CREATE INDEX IF NOT EXISTS idx_generated_tracks_created_at
    ON generated_tracks (created_at DESC);
CREATE INDEX IF NOT EXISTS idx_generated_tracks_rating
    ON generated_tracks (rating DESC);
"""


class GeneratedMusicLibrary:
    """Persistent library for MiniMax-generated tracks (mp3 + JSON metadata).

    Args:
        root_dir: Where to store ``<track_id>/`` folders + ``index.db``.
                  Defaults to ``$MUSIC_LIBRARY_PATH`` or ``/data/music_library``.
    """

    def __init__(self, root_dir: Optional[str] = None) -> None:
        self._root = Path(root_dir or _DEFAULT_ROOT)
        self._root.mkdir(parents=True, exist_ok=True)

        self._db_path = self._root / "index.db"
        self._lock = threading.Lock()
        self._conn = sqlite3.connect(str(self._db_path), check_same_thread=False)
        self._conn.row_factory = sqlite3.Row
        self._conn.executescript(_SCHEMA_SQL)
        self._conn.commit()

    # ── Internal helpers ──────────────────────────────────────────────────

    @staticmethod
    def _now_iso() -> str:
        return datetime.now(timezone.utc).isoformat()

    @staticmethod
    def _row_to_dict(row: sqlite3.Row) -> Dict[str, Any]:
        d = dict(row)
        # tags stored as JSON
        try:
            d["tags"] = json.loads(d.get("tags") or "[]")
        except json.JSONDecodeError:
            d["tags"] = []
        d["is_instrumental"] = bool(d.get("is_instrumental"))
        return d

    def _track_dir(self, track_id: str) -> Path:
        return self._root / track_id

    # ── Public API ────────────────────────────────────────────────────────

    def save_track(
        self,
        audio_bytes: bytes,
        *,
        prompt: str,
        title: str = "",
        lyrics: str = "",
        model: str = "",
        duration_ms: int = 0,
        sample_rate: int = 44100,
        bitrate: int = 256000,
        tags: Optional[List[str]] = None,
        mood: str = "",
        genre: str = "",
        lang: str = "",
        is_instrumental: bool = False,
        rating: int = 0,
        notes: str = "",
    ) -> Dict[str, Any]:
        """Persist a generated track to disk + SQLite index.

        Returns:
            dict ``{success, id, path, ...metadata}`` or ``{success: False, error}``.
        """
        if not audio_bytes:
            return {"success": False, "error": "audio_bytes is empty"}
        if not prompt or not prompt.strip():
            return {"success": False, "error": "prompt is required"}

        track_id = uuid.uuid4().hex
        track_dir = self._track_dir(track_id)
        track_dir.mkdir(parents=True, exist_ok=True)

        # Write mp3
        mp3_path = track_dir / "track.mp3"
        try:
            mp3_path.write_bytes(audio_bytes)
        except OSError as exc:
            return {"success": False, "error": f"Не удалось записать mp3: {exc}"}

        # Write meta.json (mirrors what's in SQLite — handy for offline tools)
        ts = self._now_iso()
        meta = {
            "id": track_id,
            "title": title,
            "prompt": prompt,
            "lyrics": lyrics,
            "model": model,
            "duration_ms": int(duration_ms),
            "sample_rate": int(sample_rate),
            "bitrate": int(bitrate),
            "file_size": len(audio_bytes),
            "tags": tags or [],
            "mood": mood,
            "genre": genre,
            "lang": lang,
            "is_instrumental": bool(is_instrumental),
            "rating": int(rating),
            "notes": notes,
            "created_at": ts,
            "updated_at": ts,
        }
        meta_path = track_dir / "meta.json"
        try:
            meta_path.write_text(json.dumps(meta, ensure_ascii=False, indent=2), encoding="utf-8")
        except OSError as exc:
            return {"success": False, "error": f"Не удалось записать meta.json: {exc}"}

        # Insert into SQLite
        tags_json = json.dumps(tags or [], ensure_ascii=False)
        try:
            with self._lock:
                self._conn.execute(
                    """
                    INSERT INTO generated_tracks
                        (id, title, prompt, lyrics, model, duration_ms, sample_rate,
                         bitrate, file_size, tags, mood, genre, lang,
                         is_instrumental, rating, notes, created_at, updated_at)
                    VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                    """,
                    (
                        track_id, title, prompt, lyrics, model,
                        int(duration_ms), int(sample_rate), int(bitrate),
                        len(audio_bytes), tags_json, mood, genre, lang,
                        1 if is_instrumental else 0, max(0, min(5, int(rating))),
                        notes, ts, ts,
                    ),
                )
                self._conn.commit()
        except sqlite3.Error as exc:
            return {"success": False, "error": f"SQLite insert failed: {exc}"}

        meta["path"] = str(mp3_path)
        meta["meta_json"] = str(meta_path)
        return {"success": True, "id": track_id, **meta}

    def list_tracks(
        self,
        *,
        limit: int = 20,
        sort_by: str = "recent",
        mood: Optional[str] = None,
        genre: Optional[str] = None,
        tag: Optional[str] = None,
    ) -> Dict[str, Any]:
        """List tracks with simple filters.

        Args:
            limit:   Maximum number of rows to return (default 20).
            sort_by: "recent" (created_at DESC, default) or "popular" (play_count DESC)
                     or "rating" (rating DESC).
            mood/genre/tag: Optional exact-match filters.

        Returns:
            dict ``{success, tracks: [...], total}``.
        """
        order = {
            "recent": "created_at DESC",
            "popular": "play_count DESC, created_at DESC",
            "rating": "rating DESC, created_at DESC",
        }.get(sort_by, "created_at DESC")

        where: List[str] = []
        params: List[Any] = []
        if mood:
            where.append("mood = ?")
            params.append(mood)
        if genre:
            where.append("genre = ?")
            params.append(genre)
        if tag:
            # tags stored as JSON array → LIKE match
            where.append("tags LIKE ?")
            params.append(f'%"{tag}"%')

        where_sql = (" WHERE " + " AND ".join(where)) if where else ""
        sql = f"SELECT * FROM generated_tracks{where_sql} ORDER BY {order} LIMIT ?"
        params.append(int(limit))

        try:
            with self._lock:
                rows = self._conn.execute(sql, params).fetchall()
        except sqlite3.Error as exc:
            return {"success": False, "error": f"SQLite query failed: {exc}"}

        tracks = [self._row_to_dict(r) for r in rows]
        return {"success": True, "tracks": tracks, "total": len(tracks)}

    def search_tracks(
        self,
        query: str,
        *,
        limit: int = 5,
    ) -> Dict[str, Any]:
        """Substring search across title/prompt/lyrics/genre/mood/notes.

        SQLite-native LIKE — no FTS5 (the 004_music_library migration is the
        canonical FTS5 source of truth and is owned by ``TrackLibrary``; this
        library keeps its schema self-contained to avoid coupling).

        Returns:
            dict ``{success, tracks, total, query}``.
        """
        q = (query or "").strip()
        if not q:
            return {"success": False, "error": "query is required", "tracks": [], "total": 0}

        like = f"%{q}%"
        sql = (
            "SELECT * FROM generated_tracks "
            "WHERE title LIKE ? OR prompt LIKE ? OR lyrics LIKE ? "
            "OR genre LIKE ? OR mood LIKE ? OR notes LIKE ? "
            "ORDER BY created_at DESC LIMIT ?"
        )
        params = [like, like, like, like, like, like, int(limit)]
        try:
            with self._lock:
                rows = self._conn.execute(sql, params).fetchall()
        except sqlite3.Error as exc:
            return {"success": False, "error": f"SQLite query failed: {exc}"}

        tracks = [self._row_to_dict(r) for r in rows]
        return {
            "success": True,
            "tracks": tracks,
            "total": len(tracks),
            "query": q,
        }

    def get_track_info(self, track_id: str) -> Dict[str, Any]:
        """Return metadata for a single track.

        Returns:
            dict ``{success, ...meta}`` or ``{success: False, error}``.
        """
        try:
            with self._lock:
                row = self._conn.execute(
                    "SELECT * FROM generated_tracks WHERE id = ?",
                    (track_id,),
                ).fetchone()
        except sqlite3.Error as exc:
            return {"success": False, "error": f"SQLite query failed: {exc}"}

        if not row:
            return {"success": False, "error": f"Трек '{track_id}' не найден"}

        meta = self._row_to_dict(row)
        track_dir = self._track_dir(track_id)
        meta["path"] = str(track_dir / "track.mp3")
        meta["exists_on_disk"] = (track_dir / "track.mp3").exists()
        return {"success": True, **meta}

    def delete_track(self, track_id: str) -> Dict[str, Any]:
        """Remove a track from SQLite + delete its directory.

        Returns:
            dict ``{success, message}`` or ``{success: False, error}``.
        """
        try:
            with self._lock:
                cur = self._conn.execute(
                    "DELETE FROM generated_tracks WHERE id = ? RETURNING id",
                    (track_id,),
                )
                deleted = cur.fetchone()
                self._conn.commit()
        except sqlite3.Error as exc:
            return {"success": False, "error": f"SQLite delete failed: {exc}"}

        if not deleted:
            return {"success": False, "error": f"Трек '{track_id}' не найден"}

        # Best-effort filesystem cleanup
        track_dir = self._track_dir(track_id)
        try:
            if track_dir.exists():
                for child in track_dir.iterdir():
                    try:
                        child.unlink()
                    except OSError:
                        pass
                try:
                    track_dir.rmdir()
                except OSError:
                    pass
        except OSError as exc:
            return {
                "success": True,
                "message": f"Трек '{track_id}' удалён из индекса (файлы на диске: {exc})",
            }

        return {"success": True, "message": f"Трек '{track_id}' удалён из библиотеки"}

    def increment_play_count(self, track_id: str) -> Dict[str, Any]:
        """Atomically bump play_count and return updated row."""
        try:
            with self._lock:
                cur = self._conn.execute(
                    "UPDATE generated_tracks SET play_count = play_count + 1 "
                    "WHERE id = ? RETURNING *",
                    (track_id,),
                )
                row = cur.fetchone()
                self._conn.commit()
        except sqlite3.Error as exc:
            return {"success": False, "error": f"SQLite update failed: {exc}"}

        if not row:
            return {"success": False, "error": f"Трек '{track_id}' не найден"}

        return {"success": True, **self._row_to_dict(row)}

    # ── Diagnostics ──────────────────────────────────────────────────────

    @property
    def root_dir(self) -> Path:
        return self._root

    @property
    def count(self) -> int:
        try:
            with self._lock:
                row = self._conn.execute("SELECT COUNT(*) FROM generated_tracks").fetchone()
            return int(row[0]) if row else 0
        except sqlite3.Error:
            return 0