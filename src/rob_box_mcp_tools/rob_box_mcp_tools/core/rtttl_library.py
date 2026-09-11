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
import re
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
    rtttl_name  TEXT    NOT NULL DEFAULT '',
    created_at  TEXT    NOT NULL,
    updated_at  TEXT    NOT NULL
);
CREATE INDEX IF NOT EXISTS idx_rtttl_melodies_name  ON rtttl_melodies(name);
CREATE INDEX IF NOT EXISTS idx_rtttl_melodies_title ON rtttl_melodies(title);
CREATE INDEX IF NOT EXISTS idx_rtttl_melodies_artist ON rtttl_melodies(artist);
"""

#: Русские/жаргонные названия → канонический англ. запрос (архив англоязычный).
_ALIASES = {
    "гимн ссср": "soviet anthem",
    "гимн россии": "soviet anthem",
    "советский гимн": "soviet anthem",
    "гимн": "soviet anthem",
    "ссср": "soviet anthem",
    "ussr": "soviet anthem",
    "имперский марш": "imperial march",
    "дарт вейдер": "imperial march",
    "в пещере горного короля": "mountain king",
    "григ": "mountain king",
    "тетрис": "tetris",
    "коробейники": "tetris",
    "марио": "mario",
    "супер марио": "mario",
    "нокиа": "nokia",
    "к элизе": "fur elise",
    "ода к радости": "ode to joy",
    "с днём рождения": "happy birthday",
    "с днем рождения": "happy birthday",
    "джингл белс": "jingle bells",
    "звёздные войны": "star wars",
    "звездные войны": "star wars",
}

_ALIAS_SORTED = sorted(_ALIASES.items(), key=lambda kv: -len(kv[0]))

#: Английские стоп-слова, встречающиеся почти в каждом названии («of», «the»,
#: «and»). Без их отсева LIKE-кандидаты раздуваются до всего архива, LIMIT-окно
#: обрезает реальные совпадения — «National Anthem Of Soviet» находил «American
#: National Anthem» (токен «of» матчил всё, и нужная строка не влезала в окно).
_STOPWORDS = frozenset({
    "of", "the", "a", "an", "and", "in", "on", "for", "to", "with",
    "from", "at", "by", "it", "is", "are", "this", "that", "de", "la",
})


def _normalize(query: str) -> str:
    """Нижний регистр + замена русских/жаргонных имён на канонический англ."""
    q = (query or "").strip().lower()
    for key, value in _ALIAS_SORTED:
        if key in q:
            q = q.replace(key, value)
    return q


def _tokens(query: str) -> List[str]:
    """Разбить запрос на значимые токены (кириллица отбрасывается после алиасов)."""
    return [
        t for t in re.split(r"[^a-z0-9]+", query) if t and t not in _STOPWORDS
    ]


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
        self._ensure_rtttl_name_column()
        count = self._conn.execute("SELECT COUNT(*) FROM rtttl_melodies").fetchone()[0]
        if count == 0:
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
        # Для уже существующих и свежесозданных БД: добить rtttl_name и
        # восстановить потерянные имена (title='Unknown' → artist).
        self._repair_missing_names()

    def _ensure_rtttl_name_column(self) -> None:
        """Добавить колонку ``rtttl_name`` в уже существующую БД (ALTER)."""
        cols = {row[1] for row in self._conn.execute("PRAGMA table_info(rtttl_melodies)")}
        if "rtttl_name" not in cols:
            self._conn.execute(
                "ALTER TABLE rtttl_melodies ADD COLUMN rtttl_name TEXT NOT NULL DEFAULT ''"
            )
            self._conn.commit()

    def _repair_missing_names(self) -> None:
        """Восстановить имя из двух источников, которые миграция потеряла.

        Архив местами хранит ``name='unknown_NNN'``, ``title='Unknown'``, а
        реальное имя — в ``artist`` («Batman V1.0»). Дополнительно в самом
        формате RTTTL есть поле имени (префикс строки до первого ':'): его
        кладём в ``rtttl_name``, чтобы поиск матчил и по нему.
        """
        with self._conn:
            self._conn.execute(
                "UPDATE rtttl_melodies "
                "SET rtttl_name = substr(rtttl, 1, instr(rtttl, ':') - 1) "
                "WHERE rtttl_name = '' AND instr(rtttl, ':') > 0"
            )
            self._conn.execute(
                "UPDATE rtttl_melodies SET title = artist "
                "WHERE (title IS NULL OR trim(title) = '' OR lower(trim(title)) = 'unknown') "
                "AND artist IS NOT NULL AND trim(artist) != '' "
                "AND lower(trim(artist)) != 'unknown'"
            )

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

    @staticmethod
    def _score(row: sqlite3.Row, tokens: List[str]) -> int:
        """Скоринг: сколько токенов запроса попало в поля (name/rtttl_name весомее)."""
        name_l = (row["name"] or "").lower()
        title_l = (row["title"] or "").lower()
        artist_l = (row["artist"] or "").lower()
        tags_l = (row["tags"] or "").lower()
        rtttl_name_l = (row["rtttl_name"] or "").lower()
        score = 0
        for token in tokens:
            if token == name_l or token == rtttl_name_l:
                score += 4
            elif token in title_l:
                # title несёт полное имя («Happy Birthday To You») — ценнее,
                # чем подстрока slug'а. Иначе «happy birthday» выигрывает
                # трек Ashanti «Happy» (token == name) у правильного.
                score += 3
            elif token in name_l or token in rtttl_name_l:
                score += 2
            elif token in artist_l or token in tags_l:
                score += 1
        return score

    def _candidates(self, tokens: List[str], cap: int) -> List[sqlite3.Row]:
        """Строки, где хотя бы один токен встречается в полях (метаданные)."""
        clauses = []
        params: List[str] = []
        for token in tokens:
            like = f"%{token}%"
            clauses.append(
                "(lower(name) LIKE ? OR lower(title) LIKE ? "
                "OR lower(artist) LIKE ? OR lower(tags) LIKE ? "
                "OR lower(rtttl_name) LIKE ?)"
            )
            params += [like, like, like, like, like]
        sql = (
            "SELECT id, name, title, artist, source, tags, rtttl_name "
            "FROM rtttl_melodies WHERE " + " OR ".join(clauses) + " LIMIT ?"
        )
        return self._conn.execute(sql, params + [cap]).fetchall()

    def get(self, name: str) -> Optional[Dict[str, Any]]:
        """Найти одну мелодию (точное имя → лучший по токенам запроса)."""
        q = _normalize(name)
        tokens = _tokens(q)
        if not tokens:
            return None
        with self._lock:
            row = self._conn.execute(
                "SELECT * FROM rtttl_melodies WHERE lower(name) = ? LIMIT 1", (q,)
            ).fetchone()
            if row is not None:
                return self._to_dict(row, include_rtttl=True)
            rows = self._candidates(tokens, cap=2000)
        best: Optional[sqlite3.Row] = None
        best_score = 0
        for row in rows:
            score = self._score(row, tokens)
            if score > best_score:
                best_score = score
                best = row
        if best is None:
            return None
        with self._lock:
            full = self._conn.execute(
                "SELECT * FROM rtttl_melodies WHERE id = ?", (best["id"],)
            ).fetchone()
        return self._to_dict(full, include_rtttl=True) if full is not None else None

    def search(self, query: str, limit: int = 20) -> List[Dict[str, Any]]:
        """Поиск по токенам запроса (SQL кандидаты → скоринг в Python), top-N."""
        q = _normalize(query)
        tokens = _tokens(q)
        if not tokens:
            return []
        limit = max(1, min(50, int(limit)))
        with self._lock:
            rows = self._candidates(tokens, cap=limit * 10)
        scored = []
        for row in rows:
            score = self._score(row, tokens)
            if score > 0:
                scored.append((score, row))
        scored.sort(key=lambda item: (-item[0], (item[1]["title"] or "").lower()))
        return [self._to_dict(row) for _score, row in scored[:limit]]
