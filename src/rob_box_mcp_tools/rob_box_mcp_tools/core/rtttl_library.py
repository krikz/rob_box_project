"""SQLite-хранилище RTTTL-мелодий: архив в репо → миграция в БД на диске.

Мелодии лежат в ``data/rtttl_melodies.jsonl.gz`` (сжато, ~0.7 MB в репо).
Мигратор (``RtttlLibrary.__init__``) при первом запуске читает архив и
разворачивает его в таблицу ``rtttl_melodies`` (SQLite, на диске) одной
транзакцией. Дальше поиск идёт SQL-запросами — в ОЗУ держится только
результат запроса, а не все 10460 записей (на Raspberry Pi ОЗУ мало).

Формат строки JSONL в архиве:
    {"name": slug, "title": "...", "artist": "...", "source": "...",
     "tags": [...], "rtttl": "Name:d=4,o=5,b=...:ноты"}
"""

from __future__ import annotations

import gzip
import json
import os
import sqlite3
import threading
from datetime import datetime, timezone
from importlib.resources import as_file, files
from pathlib import Path
from typing import Any, Dict, Iterator, List, Optional, TextIO, Union

__all__ = ["RtttlLibrary"]

#: Имя архива внутри пакета ``rob_box_mcp_tools/data/``.
_ARCHIVE_NAME = "rtttl_melodies.jsonl.gz"

#: Размер батча INSERT при импорте архива — держит пик ОЗУ в узде.
_BATCH = 400

_SCHEMA = """
CREATE TABLE IF NOT EXISTS rtttl_melodies (
    id          INTEGER PRIMARY KEY AUTOINCREMENT,
    name        TEXT    NOT NULL,
    title       TEXT    NOT NULL DEFAULT '',
    artist      TEXT    NOT NULL DEFAULT '',
    source      TEXT    NOT NULL DEFAULT '',
    tags        TEXT    NOT NULL DEFAULT '[]',
    rtttl       TEXT    NOT NULL UNIQUE,
    created_at  TEXT    NOT NULL,
    updated_at  TEXT    NOT NULL
);
CREATE INDEX IF NOT EXISTS idx_rtttl_melodies_name  ON rtttl_melodies(name);
CREATE INDEX IF NOT EXISTS idx_rtttl_melodies_title ON rtttl_melodies(title);
CREATE INDEX IF NOT EXISTS idx_rtttl_melodies_artist ON rtttl_melodies(artist);
"""


def _default_archive() -> Union[Path, Any]:
    """Bundled ресурс (importlib.resources) → fallback на дерево исходников."""
    try:
        res = files("rob_box_mcp_tools.data").joinpath(_ARCHIVE_NAME)
        if res.is_file():
            return res
    except (ModuleNotFoundError, TypeError):
        pass
    return Path(__file__).resolve().parent.parent / "data" / _ARCHIVE_NAME


def _iter_archive_rows(archive: Union[Path, Any]) -> Iterator[Dict[str, Any]]:
    """Построчно отдавать записи из gzip-архива (не копируя всё в память)."""
    if isinstance(archive, Path):
        with gzip.open(archive, "rt", encoding="utf-8") as fh:
            yield from _iter_json_lines(fh)
    else:
        with as_file(archive) as p:
            with gzip.open(p, "rt", encoding="utf-8") as fh:
                yield from _iter_json_lines(fh)


def _iter_json_lines(fh: TextIO) -> Iterator[Dict[str, Any]]:
    for line in fh:
        line = line.strip()
        if line:
            yield json.loads(line)


class RtttlLibrary:
    """Мигратор архива RTTTL-мелодий + поиск через SQLite (без in-memory).

    При первом ``__init__`` читает ``data/rtttl_melodies.jsonl.gz`` и
    разворачивает его в таблицу ``rtttl_melodies`` той же БД, что и
    TrackLibrary (``VOICE_MEMORY_DB_PATH`` → ``/data/voice_memory.db``).
    Импорт идемпотентен: если таблица уже заполнена — архив не читается.
    """

    def __init__(
        self,
        db_path: Optional[str] = None,
        archive_path: Optional[str] = None,
    ) -> None:
        self._db_path = db_path or os.getenv("VOICE_MEMORY_DB_PATH", "/data/voice_memory.db")
        self._archive: Union[Path, Any] = (
            Path(archive_path) if archive_path else _default_archive()
        )
        self._lock = threading.Lock()

        os.makedirs(os.path.dirname(self._db_path) or ".", exist_ok=True)
        self._conn = sqlite3.connect(self._db_path, check_same_thread=False)
        self._conn.row_factory = sqlite3.Row
        with self._lock:
            self._conn.execute("PRAGMA journal_mode=WAL")
            self._conn.executescript(_SCHEMA)
            self._conn.commit()
            self._migrate_from_archive()

    # ------------------------------------------------------------------
    # Миграция: архив → SQLite (одна транзакция, идемпотентно)
    # ------------------------------------------------------------------

    def _migrate_from_archive(self) -> None:
        count = self._conn.execute("SELECT COUNT(*) FROM rtttl_melodies").fetchone()[0]
        if count > 0:
            return
        now = datetime.now(timezone.utc).isoformat()
        insert = (
            "INSERT OR IGNORE INTO rtttl_melodies "
            "(name, title, artist, source, tags, rtttl, created_at, updated_at) "
            "VALUES (?, ?, ?, ?, ?, ?, ?, ?)"
        )
        with self._conn:  # одна транзакция — частичный импорт невозможен
            batch: List[tuple] = []
            for rec in _iter_archive_rows(self._archive):
                batch.append((
                    rec.get("name", ""),
                    rec.get("title", ""),
                    rec.get("artist", ""),
                    rec.get("source", ""),
                    json.dumps(rec.get("tags") or [], ensure_ascii=False),
                    rec.get("rtttl", ""),
                    now,
                    now,
                ))
                if len(batch) >= _BATCH:
                    self._conn.executemany(insert, batch)
                    batch = []
            if batch:
                self._conn.executemany(insert, batch)

    # ------------------------------------------------------------------
    # Поиск (SQL, на диске)
    # ------------------------------------------------------------------

    @staticmethod
    def _to_dict(row: sqlite3.Row, include_rtttl: bool = False) -> Dict[str, Any]:
        d = dict(row)
        d["tags"] = json.loads(d.get("tags") or "[]")
        for key in ("id", "created_at", "updated_at"):
            d.pop(key, None)
        if not include_rtttl:
            d.pop("rtttl", None)
        return d

    def total(self) -> int:
        with self._lock:
            return self._conn.execute("SELECT COUNT(*) FROM rtttl_melodies").fetchone()[0]

    def get(self, name: str) -> Optional[Dict[str, Any]]:
        """Найти одну мелодию (точное имя → name/title → artist/tags)."""
        q = (name or "").strip().lower()
        if not q:
            return None
        with self._lock:
            row = self._conn.execute(
                """
                SELECT * FROM rtttl_melodies
                WHERE lower(name) LIKE :like OR lower(title) LIKE :like
                   OR lower(artist) LIKE :like OR lower(tags) LIKE :like
                ORDER BY CASE
                    WHEN lower(name) = :q THEN 0
                    WHEN lower(name) LIKE :prefix OR lower(title) LIKE :prefix THEN 1
                    ELSE 2
                END, title COLLATE NOCASE
                LIMIT 1
                """,
                {"q": q, "like": f"%{q}%", "prefix": f"{q}%"},
            ).fetchone()
        return self._to_dict(row, include_rtttl=True) if row is not None else None

    def search(self, query: str, limit: int = 20) -> List[Dict[str, Any]]:
        """Подстрочный поиск по name/title/artist/tags (SQL), top-N.

        Ранжирование делается в SQL (CASE в ORDER BY), поэтому в ОЗУ попадает
        только итоговый список (≤ limit), а не вся библиотека.
        """
        q = (query or "").strip().lower()
        if not q:
            return []
        limit = max(1, min(50, int(limit)))
        with self._lock:
            rows = self._conn.execute(
                """
                SELECT name, title, artist, source, tags FROM rtttl_melodies
                WHERE lower(name) LIKE :like OR lower(title) LIKE :like
                   OR lower(artist) LIKE :like OR lower(tags) LIKE :like
                ORDER BY CASE
                    WHEN lower(name) = :q THEN 0
                    WHEN lower(name) LIKE :prefix OR lower(title) LIKE :prefix THEN 1
                    WHEN lower(name) LIKE :like OR lower(title) LIKE :like THEN 2
                    ELSE 3
                END, title COLLATE NOCASE
                LIMIT :limit
                """,
                {"q": q, "like": f"%{q}%", "prefix": f"{q}%", "limit": limit},
            ).fetchall()
        return [self._to_dict(r) for r in rows]
