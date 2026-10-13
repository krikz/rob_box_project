#!/usr/bin/env python3
"""
library.py — кэш сгенерированных треков на диске (ADR-0158 §2.5, §2.6).

Один трек — одна папка под ``root_dir`` (по умолчанию ``/data/music_library/``):

    /data/music_library/<track_id>/
        track.mp3     — аудио от провайдера (KIE удаляет у себя через 14 дней)
        meta.json     — те же метаданные, человекочитаемо

Один SQLite-индекс ``root_dir/index.db`` — единственная схема в коде
(миграция ``007_generated_music_library.sql`` удалена: её никто не применял, а
схема расходилась с этой). Таблицы:

* ``generated_tracks`` — треки. Колонки ADR-0158 §2.5: ``request_key``,
  ``pool_key``, ``gen_provider``, ``task_id``, ``variant``, ``credits``,
  ``tts_provider``, ``voice``, ``work_id``, ``instrument``, ``source_audio_sha``,
  ``bpm_nominal``, ``loudness_lufs``, ``last_played_at``, ``play_count``,
  ``rating``, ``meta_source`` (+ ``kind``, ``style_key``, ``file_sha256``).
* ``gen_ledger`` — траты кредитов (``ledger.GenLedger``, доступен как
  ``library.ledger``): ``creditsConsumed`` из ответа провайдера, не оценка.

**Миграция идемпотентна**: индекс прежней схемы (MiniMax, 3 трека на роботе)
открывается этим же классом — недостающие колонки добавляются
``ALTER TABLE ADD COLUMN``, старые строки сохраняются и помечаются
``gen_provider='minimax'``, ``meta_source='legacy'``.

Потокобезопасно (``threading.Lock``), соединение с ``check_same_thread=False``
для потоков ROS-executor'а. ROS-free.
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

from .keys import sha256_bytes, sha256_file
from .ledger import LEDGER_SQL, GenLedger

_DEFAULT_ROOT = os.getenv(
    "MUSIC_LIBRARY_PATH",
    "/data/music_library",
)

#: Версия схемы в ``PRAGMA user_version``. 0/1 — прежняя (MiniMax) схема.
SCHEMA_VERSION = 2

_BASE_SQL = """
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

#: Колонки, добавленные ADR-0158 поверх прежней схемы: (имя, DDL-тип с DEFAULT).
_NEW_COLUMNS = (
    ("kind", "TEXT NOT NULL DEFAULT ''"),            # instrumental / song / robot_song / cover
    ("style_key", "TEXT NOT NULL DEFAULT ''"),
    ("request_key", "TEXT"),
    ("pool_key", "TEXT"),
    ("gen_provider", "TEXT NOT NULL DEFAULT ''"),
    ("task_id", "TEXT"),
    ("variant", "INTEGER"),
    ("credits", "REAL NOT NULL DEFAULT 0"),
    ("tts_provider", "TEXT NOT NULL DEFAULT ''"),
    ("voice", "TEXT NOT NULL DEFAULT ''"),
    ("work_id", "TEXT NOT NULL DEFAULT ''"),
    ("instrument", "TEXT NOT NULL DEFAULT ''"),
    ("source_audio_sha", "TEXT NOT NULL DEFAULT ''"),
    ("bpm_nominal", "INTEGER"),
    ("loudness_lufs", "REAL"),
    ("last_played_at", "TEXT"),
    ("meta_source", "TEXT NOT NULL DEFAULT ''"),     # live / api_record / request_params / legacy
    ("file_sha256", "TEXT"),
)

_INDEX_SQL = """
CREATE UNIQUE INDEX IF NOT EXISTS uq_generated_tracks_task_variant
    ON generated_tracks (task_id, variant)
    WHERE task_id IS NOT NULL AND task_id <> '';
CREATE INDEX IF NOT EXISTS idx_generated_tracks_request_key
    ON generated_tracks (request_key);
CREATE INDEX IF NOT EXISTS idx_generated_tracks_pool_key
    ON generated_tracks (pool_key);
CREATE INDEX IF NOT EXISTS idx_generated_tracks_file_sha
    ON generated_tracks (file_sha256);
"""

#: Порядок колонок INSERT (совпадает с ключами ``meta`` в ``save_track``).
_INSERT_COLUMNS = (
    "id", "title", "prompt", "lyrics", "model", "duration_ms", "sample_rate",
    "bitrate", "file_size", "tags", "mood", "genre", "lang", "is_instrumental",
    "rating", "notes", "created_at", "updated_at", "kind", "style_key",
    "request_key", "pool_key", "gen_provider", "task_id", "variant", "credits",
    "tts_provider", "voice", "work_id", "instrument", "source_audio_sha",
    "bpm_nominal", "loudness_lufs", "meta_source", "file_sha256",
)


def utc_iso(dt: Optional[datetime] = None) -> str:
    """Метка времени в формате библиотеки: UTC, секунды, ``+00:00``."""
    return (dt or datetime.now(timezone.utc)).astimezone(timezone.utc).replace(microsecond=0).isoformat()


def migrate_schema(conn: sqlite3.Connection) -> None:
    """Добавить недостающие колонки/индексы, пометить строки прежней схемы. Идемпотентно."""
    have = {r["name"] for r in conn.execute("PRAGMA table_info(generated_tracks)")}
    for name, ddl in _NEW_COLUMNS:
        if name not in have:
            conn.execute(f"ALTER TABLE generated_tracks ADD COLUMN {name} {ddl}")
    conn.executescript(_INDEX_SQL)
    # Строки до ADR-0158 — только MiniMax-генерации.
    conn.execute(
        "UPDATE generated_tracks SET gen_provider = 'minimax' "
        "WHERE gen_provider = '' AND meta_source = ''"
    )
    conn.execute("UPDATE generated_tracks SET meta_source = 'legacy' WHERE meta_source = ''")
    conn.execute(f"PRAGMA user_version = {SCHEMA_VERSION}")
    conn.commit()


class GeneratedMusicLibrary:
    """Кэш сгенерированных треков (mp3 + JSON-метаданные + SQLite-индекс).

    Args:
        root_dir: Где хранить ``<track_id>/`` и ``index.db``.
                  По умолчанию ``$MUSIC_LIBRARY_PATH`` или ``/data/music_library``.
    """

    def __init__(self, root_dir: Optional[str] = None) -> None:
        self._root = Path(root_dir or _DEFAULT_ROOT)
        self._root.mkdir(parents=True, exist_ok=True)

        self._db_path = self._root / "index.db"
        self._lock = threading.Lock()
        self._conn = sqlite3.connect(str(self._db_path), check_same_thread=False)
        self._conn.row_factory = sqlite3.Row
        self._conn.executescript(_BASE_SQL)
        self._conn.executescript(LEDGER_SQL)
        #: Учёт кредитов в той же ``index.db`` (ADR-0158 §2.6).
        self.ledger = GenLedger(self._conn, self._lock)
        self._migrate()

    # ── Migration ─────────────────────────────────────────────────────────

    def _migrate(self) -> None:
        """Привести индекс к ``SCHEMA_VERSION``. Безопасно вызывать сколько угодно раз."""
        with self._lock:
            migrate_schema(self._conn)
            rows = self._conn.execute(
                "SELECT id FROM generated_tracks WHERE file_sha256 IS NULL"
            ).fetchall()
            for row in rows:  # хэш файла для строк без него (нужен импорту для дедупликации)
                mp3 = self._track_dir(row["id"]) / "track.mp3"
                if mp3.exists():
                    self._conn.execute(
                        "UPDATE generated_tracks SET file_sha256 = ? WHERE id = ?",
                        (sha256_file(mp3), row["id"]),
                    )
            self._conn.commit()

    # ── Internal helpers ──────────────────────────────────────────────────

    @staticmethod
    def _now_iso() -> str:
        return datetime.now(timezone.utc).isoformat()

    @staticmethod
    def _row_to_dict(row: sqlite3.Row) -> Dict[str, Any]:
        d = dict(row)
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
        kind: str = "",
        style_key: str = "",
        request_key: Optional[str] = None,
        pool_key: Optional[str] = None,
        gen_provider: str = "",
        task_id: Optional[str] = None,
        variant: Optional[int] = None,
        credits: float = 0.0,
        tts_provider: str = "",
        voice: str = "",
        work_id: str = "",
        instrument: str = "",
        source_audio_sha: str = "",
        bpm_nominal: Optional[int] = None,
        loudness_lufs: Optional[float] = None,
        meta_source: str = "live",
        created_at: Optional[str] = None,
        dedupe_sha: bool = False,
    ) -> Dict[str, Any]:
        """Сохранить трек на диск и в индекс.

        Идемпотентность: непустой ``task_id`` + ``variant`` уже в индексе →
        возвращается существующая запись (``duplicate=True``), файл не пишется.
        При ``dedupe_sha=True`` то же для совпавшего ``sha256`` файла
        (импорт, где ``task_id`` неизвестен).

        Returns:
            ``{success, id, path, ...metadata}`` или ``{success: False, error}``.
        """
        if not audio_bytes:
            return {"success": False, "error": "audio_bytes is empty"}
        if not prompt or not prompt.strip():
            return {"success": False, "error": "prompt is required"}

        task_id = task_id or None
        file_sha = sha256_bytes(audio_bytes)
        existing = self._find_existing(task_id, variant, file_sha if dedupe_sha else None)
        if existing is not None:
            return {**self.get_track_info(existing), "id": existing, "duplicate": True}

        track_id = uuid.uuid4().hex
        ts = created_at or self._now_iso()
        meta = {
            "id": track_id, "title": title, "prompt": prompt, "lyrics": lyrics, "model": model,
            "duration_ms": int(duration_ms), "sample_rate": int(sample_rate), "bitrate": int(bitrate),
            "file_size": len(audio_bytes), "tags": tags or [], "mood": mood, "genre": genre, "lang": lang,
            "is_instrumental": bool(is_instrumental), "rating": int(rating), "notes": notes,
            "created_at": ts, "updated_at": ts, "kind": kind, "style_key": style_key,
            "request_key": request_key, "pool_key": pool_key, "gen_provider": gen_provider,
            "task_id": task_id, "variant": variant, "credits": float(credits),
            "tts_provider": tts_provider, "voice": voice, "work_id": work_id, "instrument": instrument,
            "source_audio_sha": source_audio_sha, "bpm_nominal": bpm_nominal,
            "loudness_lufs": loudness_lufs, "meta_source": meta_source, "file_sha256": file_sha,
        }
        track_dir = self._track_dir(track_id)
        error = self._write_files(track_dir, audio_bytes, meta) or self._insert_row(meta)
        if error:
            return {"success": False, "error": error}

        mp3_path = track_dir / "track.mp3"
        meta_path = track_dir / "meta.json"
        meta["path"] = str(mp3_path)
        meta["meta_json"] = str(meta_path)
        return {"success": True, "duplicate": False, **meta}

    @staticmethod
    def _write_files(track_dir: Path, audio_bytes: bytes, meta: Dict[str, Any]) -> Optional[str]:
        """mp3 + meta.json на диск. Возвращает текст ошибки или ``None``."""
        track_dir.mkdir(parents=True, exist_ok=True)
        try:
            (track_dir / "track.mp3").write_bytes(audio_bytes)
        except OSError as exc:
            return f"Не удалось записать mp3: {exc}"
        try:
            (track_dir / "meta.json").write_text(
                json.dumps(meta, ensure_ascii=False, indent=2), encoding="utf-8")
        except OSError as exc:
            return f"Не удалось записать meta.json: {exc}"
        return None

    def _insert_row(self, meta: Dict[str, Any]) -> Optional[str]:
        """Строка в индекс. Возвращает текст ошибки или ``None``."""
        values = {
            **meta,
            "tags": json.dumps(meta["tags"], ensure_ascii=False),
            "is_instrumental": 1 if meta["is_instrumental"] else 0,
            "rating": max(-1, min(5, meta["rating"])),
        }
        sql = (
            f"INSERT INTO generated_tracks ({', '.join(_INSERT_COLUMNS)}) "
            f"VALUES ({', '.join('?' for _ in _INSERT_COLUMNS)})"
        )
        try:
            with self._lock:
                self._conn.execute(sql, [values[c] for c in _INSERT_COLUMNS])
                self._conn.commit()
        except sqlite3.Error as exc:
            return f"SQLite insert failed: {exc}"
        return None

    def _find_existing(
        self, task_id: Optional[str], variant: Optional[int], file_sha: Optional[str],
    ) -> Optional[str]:
        with self._lock:
            if task_id:
                row = self._conn.execute(
                    "SELECT id FROM generated_tracks WHERE task_id = ? AND variant IS ?",
                    (task_id, variant),
                ).fetchone()
                if row:
                    return row["id"]
            if file_sha:
                row = self._conn.execute(
                    "SELECT id FROM generated_tracks WHERE file_sha256 = ?", (file_sha,),
                ).fetchone()
                if row:
                    return row["id"]
        return None

    def find_by_request_key(self, request_key: str, *, limit: int = 10) -> Dict[str, Any]:
        """Треки кэша по ключу запроса: давно не игранные — первыми (ADR-0158 §2.5)."""
        if not request_key:
            return {"success": True, "tracks": [], "total": 0}
        sql = (
            "SELECT * FROM generated_tracks WHERE request_key = ? "
            "ORDER BY (last_played_at IS NOT NULL), last_played_at ASC, variant ASC LIMIT ?"
        )
        try:
            with self._lock:
                rows = self._conn.execute(sql, (request_key, int(limit))).fetchall()
        except sqlite3.Error as exc:
            return {"success": False, "error": f"SQLite query failed: {exc}"}
        tracks = [self._row_to_dict(r) for r in rows]
        return {"success": True, "tracks": tracks, "total": len(tracks)}

    def list_tracks(
        self,
        *,
        limit: int = 20,
        sort_by: str = "recent",
        mood: Optional[str] = None,
        genre: Optional[str] = None,
        tag: Optional[str] = None,
    ) -> Dict[str, Any]:
        """Список треков с простыми фильтрами.

        Args:
            limit:   Максимум строк (по умолчанию 20).
            sort_by: "recent" (created_at DESC), "popular" (play_count DESC)
                     или "rating" (rating DESC).
            mood/genre/tag: точные фильтры.

        Returns:
            ``{success, tracks: [...], total}``.
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
        """Поиск подстроки по title/prompt/lyrics/genre/mood/notes (SQLite ``LIKE``).

        Returns:
            ``{success, tracks, total, query}``.
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
        """Метаданные одного трека: ``{success, ...meta}`` или ``{success: False, error}``."""
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
        """Удалить трек из индекса и с диска."""
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
        """Атомарно увеличить ``play_count``, отметить ``last_played_at``, вернуть строку."""
        try:
            with self._lock:
                cur = self._conn.execute(
                    "UPDATE generated_tracks SET play_count = play_count + 1, "
                    "last_played_at = ? WHERE id = ? RETURNING *",
                    (utc_iso(), track_id),
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
