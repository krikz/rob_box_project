"""Persistent FAQ knowledge store for event mode."""

from __future__ import annotations

import os
import sqlite3
import struct
import threading
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional

from .voice_memory import _SQLITE_VEC_AVAILABLE, OllamaEmbedder

# Russian stopwords that carry no semantic meaning for FAQ lookup.
# Filtering these prevents high-frequency filler words (e.g. "про", "расскажи")
# from matching dozens of unrelated entries via the OR-based FTS query.
_RU_STOPWORDS: frozenset[str] = frozenset(
    {
        # Prepositions
        "в", "на", "за", "из", "от", "к", "с", "по", "до", "для", "через",
        "без", "о", "об", "обо", "при", "под", "над", "у", "про", "между",
        # Conjunctions / particles
        "и", "а", "но", "или", "если", "что", "как", "когда", "где", "потому",
        "хотя", "чтобы", "то", "же", "ли", "бы", "так", "даже", "ведь", "раз",
        "не", "ни", "вот", "вон", "ну", "уже", "ещё", "еще", "тоже", "только",
        "лишь", "вообще", "просто", "именно",
        # Pronouns
        "я", "ты", "он", "она", "оно", "мы", "вы", "они",
        "мой", "моя", "мое", "моё", "моих", "мои",
        "твой", "твоя", "твое", "твоё",
        "его", "её", "ее", "их", "наш", "ваш", "свой",
        "этот", "эта", "это", "эти", "тот", "та", "те",
        "себя", "себе", "собой", "сам", "сама", "само", "сами",
        # Spoken-query filler verbs
        "расскажи", "скажи", "поищи", "найди", "покажи", "объясни",
        "помоги", "хочу", "можешь", "можно", "нужно", "надо",
        # Misc high-frequency with no discriminative value
        "есть", "нет", "да", "там", "тут", "здесь", "сейчас",
        "все", "всё", "всех", "всем",
    }
)

# Minimum stem length kept when building a morphological prefix token.
_MIN_STEM_LEN: int = 4


def _tokenize_fts(query: str) -> List[str]:
    """Build FTS5 token list from *query* with two improvements over a plain split.

    1. Russian stopwords are removed so high-frequency filler words don't
       inflate result counts and crowd out vector search.
    2. For words longer than ``_MIN_STEM_LEN + 2`` characters a second, shorter
       prefix token is added (last 2 chars stripped).  This compensates for
       the lack of a Russian stemmer: e.g. "коррупцию" also emits "коррупци"*
       which matches the base form "коррупция" stored in the FAQ.
    """
    seen: set[str] = set()
    tokens: List[str] = []

    for raw in query.lower().split():
        word = raw.strip(".,!?;:\"'()[]{}—\u2014-")
        if not word or word in _RU_STOPWORDS:
            continue

        full = f'"{word}"*'
        if full not in seen:
            seen.add(full)
            tokens.append(full)

        # Morphological stem prefix (handles Russian declensions/conjugations)
        if len(word) > _MIN_STEM_LEN + 2:
            stem = f'"{word[:-2]}"*'
            if stem not in seen:
                seen.add(stem)
                tokens.append(stem)

    return tokens

try:
    import sqlite_vec
except ImportError:
    sqlite_vec = None


class FAQStore:
    """Stores FAQ rows in SQLite with FTS5 and optional vector search."""

    FTS_MIN_RESULTS = 3

    def __init__(
        self,
        db_path: str,
        migrations_dir: Optional[str] = None,
        ollama_base_url: Optional[str] = None,
        ollama_model: str = OllamaEmbedder.DEFAULT_MODEL,
        ollama_timeout: float = 5.0,
    ) -> None:
        self.db_path = db_path
        self.lock = threading.Lock()

        os.makedirs(os.path.dirname(os.path.abspath(db_path)), exist_ok=True)
        self.conn = sqlite3.connect(db_path, check_same_thread=False)
        self.conn.row_factory = sqlite3.Row

        with self.lock:
            self.conn.execute("PRAGMA foreign_keys = ON")
            self.conn.execute("PRAGMA journal_mode = WAL")
            self.conn.execute("PRAGMA synchronous = NORMAL")

        if migrations_dir:
            self.migrations_dir = migrations_dir
        else:
            self.migrations_dir = self._find_migrations_dir()

        self._run_migrations()
        self._vec_loaded = self._load_vec_extension()

        base_url = ollama_base_url or os.getenv(
            "OLLAMA_BASE_URL", "http://localhost:11434"
        )
        self.embedder = OllamaEmbedder(
            base_url=base_url,
            model=ollama_model,
            timeout=ollama_timeout,
        )

        stored_dim = self._get_meta_int("faq_embedding_dim")
        if stored_dim and self._vec_loaded:
            self._ensure_vec_table(stored_dim)

    @staticmethod
    def _find_migrations_dir() -> str:
        """Locate the project ``migrations/`` directory.

        Resolution order:
        1. ``/migrations`` — runtime mount in Docker.
        2. Walk up from this source file looking for a ``migrations/`` sibling
           of ``src/`` — works for both the development source tree and a
           colcon-installed package (because colcon copies only ``src/``,
           the relative layout is ``<root>/src/rob_box_voice/...``).
        3. The legacy 4-level-up path, preserved for backwards compatibility.
        """
        _docker_mount = "/migrations"
        if os.path.isdir(_docker_mount):
            return _docker_mount

        here = os.path.dirname(os.path.abspath(__file__))
        for parent in [here, *Path(here).parents]:
            candidate = os.path.join(parent, "migrations")
            if os.path.isdir(candidate):
                return candidate

        # Legacy fallback (4 parents up from src/rob_box_voice/core/).
        return os.path.normpath(
            os.path.join(here, "..", "..", "..", "..", "migrations")
        )

    def _run_migrations(self) -> None:
        """Apply SQL migrations from ``migrations_dir`` in numeric filename order.

        Each migration file is named ``NNN_*.sql`` and ``NNN`` is its version.
        We track the highest applied version via ``PRAGMA user_version``.

        SQLite has no ``ALTER TABLE ... ADD COLUMN IF NOT EXISTS``. Migration
        006 tolerates duplicate-column errors during its ALTER by relying on
        the ``TrackLibrary.__init__`` runtime guard as a fallback path; the
        migration runner itself propagates any other OperationalError so that
        genuine migration failures remain visible.
        """
        with self.lock:
            current = self.conn.execute("PRAGMA user_version").fetchone()[0]

        if not os.path.isdir(self.migrations_dir):
            return

        for filename in sorted(
            f for f in os.listdir(self.migrations_dir) if f.endswith(".sql")
        ):
            try:
                version = int(filename.split("_", 1)[0])
            except ValueError:
                continue
            if version <= current:
                continue
            sql_path = os.path.join(self.migrations_dir, filename)
            with open(sql_path, "r", encoding="utf-8") as handle:
                sql = handle.read()
            with self.lock, self.conn:
                self.conn.executescript(sql)
                self.conn.execute(f"PRAGMA user_version = {version}")

    def _load_vec_extension(self) -> bool:
        if not _SQLITE_VEC_AVAILABLE or sqlite_vec is None:
            return False
        try:
            with self.lock:
                self.conn.enable_load_extension(True)
                sqlite_vec.load(self.conn)
                self.conn.enable_load_extension(False)
            return True
        except Exception:
            return False

    def _ensure_vec_table(self, dim: int) -> bool:
        if not self._vec_loaded:
            return False
        stored = self._get_meta_int("faq_embedding_dim")
        if stored and stored != dim:
            with self.lock, self.conn:
                self.conn.execute("DROP TABLE IF EXISTS faq_items_vec")
            self._set_meta("faq_embedding_dim", str(dim))
        elif stored is None:
            self._set_meta("faq_embedding_dim", str(dim))

        try:
            with self.lock, self.conn:
                self.conn.execute(
                    f"CREATE VIRTUAL TABLE IF NOT EXISTS faq_items_vec "
                    f"USING vec0(rowid INTEGER PRIMARY KEY, embedding float[{dim}])"
                )
            return True
        except Exception:
            return False

    def _has_vec_table(self) -> bool:
        with self.lock:
            row = self.conn.execute(
                "SELECT name FROM sqlite_master WHERE type='table' AND name='faq_items_vec'"
            ).fetchone()
        return row is not None

    def _get_meta(self, key: str) -> Optional[str]:
        with self.lock:
            row = self.conn.execute(
                "SELECT value FROM voice_memory_meta WHERE key = ?",
                (key,),
            ).fetchone()
        return row["value"] if row else None

    def _get_meta_int(self, key: str) -> Optional[int]:
        value = self._get_meta(key)
        return int(value) if value is not None else None

    def _set_meta(self, key: str, value: str) -> None:
        with self.lock, self.conn:
            self.conn.execute(
                "INSERT OR REPLACE INTO voice_memory_meta (key, value) VALUES (?, ?)",
                (key, value),
            )

    def replace_items(self, event_id: str, items: List[Dict[str, str]]) -> int:
        """Replace all indexed FAQ items for one event."""
        if not event_id:
            raise ValueError("event_id is required")

        existing_ids: List[int] = []
        with self.lock:
            rows = self.conn.execute(
                "SELECT id FROM faq_items WHERE event_id = ?",
                (event_id,),
            ).fetchall()
            existing_ids = [int(row["id"]) for row in rows]

        if existing_ids and self._has_vec_table():
            placeholders = ",".join("?" for _ in existing_ids)
            with self.lock, self.conn:
                self.conn.execute(
                    f"DELETE FROM faq_items_vec WHERE rowid IN ({placeholders})",
                    existing_ids,
                )

        with self.lock, self.conn:
            self.conn.execute("DELETE FROM faq_items WHERE event_id = ?", (event_id,))

        inserted = 0
        indexed_at = datetime.utcnow().isoformat()
        for item in items:
            with self.lock, self.conn:
                cursor = self.conn.execute(
                    """
                    INSERT INTO faq_items (event_id, question, answer, category, source, indexed_at)
                    VALUES (?, ?, ?, ?, ?, ?)
                    """,
                    (
                        event_id,
                        item["question"],
                        item["answer"],
                        item.get("category", "general") or "general",
                        item.get("source", "") or "",
                        indexed_at,
                    ),
                )
                row_id = int(cursor.lastrowid)
            self._embed_and_store(row_id, item["question"], item["answer"])
            inserted += 1

        return inserted

    async def areplace_items(
        self, event_id: str, items: List[Dict[str, str]]
    ) -> int:
        """
        Async variant of :meth:`replace_items` -- never blocks the loop.

        Item rows are inserted synchronously (fast SQLite); the optional
        Ollama embedding for each item is awaited via ``aembed``.
        """
        if not event_id:
            raise ValueError("event_id is required")

        existing_ids: List[int] = []
        with self.lock:
            rows = self.conn.execute(
                "SELECT id FROM faq_items WHERE event_id = ?",
                (event_id,),
            ).fetchall()
            existing_ids = [int(row["id"]) for row in rows]

        if existing_ids and self._has_vec_table():
            placeholders = ",".join("?" for _ in existing_ids)
            with self.lock, self.conn:
                self.conn.execute(
                    f"DELETE FROM faq_items_vec WHERE rowid IN ({placeholders})",
                    existing_ids,
                )

        with self.lock, self.conn:
            self.conn.execute("DELETE FROM faq_items WHERE event_id = ?", (event_id,))

        inserted = 0
        indexed_at = datetime.utcnow().isoformat()
        for item in items:
            with self.lock, self.conn:
                cursor = self.conn.execute(
                    """
                    INSERT INTO faq_items (event_id, question, answer, category, source, indexed_at)
                    VALUES (?, ?, ?, ?, ?, ?)
                    """,
                    (
                        event_id,
                        item["question"],
                        item["answer"],
                        item.get("category", "general") or "general",
                        item.get("source", "") or "",
                        indexed_at,
                    ),
                )
                row_id = int(cursor.lastrowid)
            await self._aembed_and_store(row_id, item["question"], item["answer"])
            inserted += 1

        return inserted

    async def _aembed_and_store(
        self, row_id: int, question: str, answer: str
    ) -> None:
        """Async variant of :meth:`_embed_and_store` (awaits Ollama)."""
        vector = await self.embedder.aembed(f"{question}\n{answer}")
        if vector is None:
            return
        if not self._has_vec_table() and not self._ensure_vec_table(len(vector)):
            return
        stored_dim = self._get_meta_int("faq_embedding_dim")
        if stored_dim and len(vector) != stored_dim:
            return
        vector_blob = struct.pack(f"{len(vector)}f", *vector)
        try:
            with self.lock, self.conn:
                self.conn.execute(
                    "INSERT OR REPLACE INTO faq_items_vec (rowid, embedding) VALUES (?, ?)",
                    (row_id, vector_blob),
                )
        except Exception:
            return

    def _embed_and_store(self, row_id: int, question: str, answer: str) -> None:
        vector = self.embedder.embed(f"{question}\n{answer}")
        if vector is None:
            return
        if not self._has_vec_table() and not self._ensure_vec_table(len(vector)):
            return
        stored_dim = self._get_meta_int("faq_embedding_dim")
        if stored_dim and len(vector) != stored_dim:
            return
        vector_blob = struct.pack(f"{len(vector)}f", *vector)
        try:
            with self.lock, self.conn:
                self.conn.execute(
                    "INSERT OR REPLACE INTO faq_items_vec (rowid, embedding) VALUES (?, ?)",
                    (row_id, vector_blob),
                )
        except Exception:
            return

    def search(
        self, query: str, event_id: Optional[str] = None, limit: int = 3
    ) -> List[Dict]:
        """Search event FAQ by keyword first and vector similarity second."""
        if not query or not query.strip():
            return []

        fts_results = self._fts_search(query=query, event_id=event_id, limit=limit)
        if len(fts_results) < self.FTS_MIN_RESULTS and self.embedder.is_available():
            vec_results = self._vector_search(
                query=query, event_id=event_id, limit=limit
            )
            return self._merge_results(fts_results, vec_results, limit)
        return fts_results

    async def asearch(
        self, query: str, event_id: Optional[str] = None, limit: int = 3
    ) -> List[Dict]:
        """
        Async variant of :meth:`search` -- never blocks the event loop.

        FTS5 keyword search is fast and runs inline; the Ollama embedding
        (if any) is awaited so the ROS event loop is not stalled.
        """
        if not query or not query.strip():
            return []

        fts_results = self._fts_search(query=query, event_id=event_id, limit=limit)
        if len(fts_results) < self.FTS_MIN_RESULTS and self.embedder.is_available():
            vec_results = await self._avector_search(
                query=query, event_id=event_id, limit=limit
            )
            return self._merge_results(fts_results, vec_results, limit)
        return fts_results

    async def _avector_search(
        self, query: str, event_id: Optional[str], limit: int
    ) -> List[Dict]:
        """Async variant of :meth:`_vector_search` (awaits Ollama)."""
        if not self._has_vec_table():
            return []
        vector = await self.embedder.aembed(query)
        if vector is None:
            return []

        vector_blob = struct.pack(f"{len(vector)}f", *vector)
        try:
            with self.lock:
                if event_id:
                    rows = self.conn.execute(
                        """
                        SELECT fi.id, fi.event_id, fi.question, fi.answer, fi.category, fi.source,
                               (1.0 - fv.distance) AS score
                        FROM faq_items_vec fv
                        JOIN faq_items fi ON fi.id = fv.rowid
                        WHERE fv.embedding MATCH ? AND k = ? AND fi.event_id = ?
                        ORDER BY fv.distance
                        """,
                        (vector_blob, limit, event_id),
                    ).fetchall()
                else:
                    rows = self.conn.execute(
                        """
                        SELECT fi.id, fi.event_id, fi.question, fi.answer, fi.category, fi.source,
                               (1.0 - fv.distance) AS score
                        FROM faq_items_vec fv
                        JOIN faq_items fi ON fi.id = fv.rowid
                        WHERE fv.embedding MATCH ? AND k = ?
                        ORDER BY fv.distance
                        """,
                        (vector_blob, limit),
                    ).fetchall()
            return [{**dict(row), "source_type": "vec"} for row in rows]
        except sqlite3.OperationalError:
            return []

    def _fts_search(
        self, query: str, event_id: Optional[str], limit: int
    ) -> List[Dict]:
        tokens = _tokenize_fts(query)
        if not tokens:
            # All words were stopwords — fall back to plain split so we don't
            # return an empty result when the query is very short.
            tokens = [f'"{word}"*' for word in query.split() if word]
        if not tokens:
            return []
        fts_query = " OR ".join(tokens)
        try:
            with self.lock:
                if event_id:
                    rows = self.conn.execute(
                        """
                        SELECT fi.id, fi.event_id, fi.question, fi.answer, fi.category, fi.source,
                               (-bm25(faq_items_fts)) AS score
                        FROM faq_items_fts
                        JOIN faq_items fi ON fi.id = faq_items_fts.rowid
                        WHERE faq_items_fts MATCH ? AND fi.event_id = ?
                        ORDER BY score DESC
                        LIMIT ?
                        """,
                        (fts_query, event_id, limit),
                    ).fetchall()
                else:
                    rows = self.conn.execute(
                        """
                        SELECT fi.id, fi.event_id, fi.question, fi.answer, fi.category, fi.source,
                               (-bm25(faq_items_fts)) AS score
                        FROM faq_items_fts
                        JOIN faq_items fi ON fi.id = faq_items_fts.rowid
                        WHERE faq_items_fts MATCH ?
                        ORDER BY score DESC
                        LIMIT ?
                        """,
                        (fts_query, limit),
                    ).fetchall()
            return [{**dict(row), "source_type": "fts"} for row in rows]
        except sqlite3.OperationalError:
            return []

    def _vector_search(
        self, query: str, event_id: Optional[str], limit: int
    ) -> List[Dict]:
        if not self._has_vec_table():
            return []
        vector = self.embedder.embed(query)
        if vector is None:
            return []

        vector_blob = struct.pack(f"{len(vector)}f", *vector)
        try:
            with self.lock:
                if event_id:
                    rows = self.conn.execute(
                        """
                        SELECT fi.id, fi.event_id, fi.question, fi.answer, fi.category, fi.source,
                               (1.0 - fv.distance) AS score
                        FROM faq_items_vec fv
                        JOIN faq_items fi ON fi.id = fv.rowid
                        WHERE fv.embedding MATCH ? AND k = ? AND fi.event_id = ?
                        ORDER BY fv.distance
                        """,
                        (vector_blob, limit, event_id),
                    ).fetchall()
                else:
                    rows = self.conn.execute(
                        """
                        SELECT fi.id, fi.event_id, fi.question, fi.answer, fi.category, fi.source,
                               (1.0 - fv.distance) AS score
                        FROM faq_items_vec fv
                        JOIN faq_items fi ON fi.id = fv.rowid
                        WHERE fv.embedding MATCH ? AND k = ?
                        ORDER BY fv.distance
                        """,
                        (vector_blob, limit),
                    ).fetchall()
            return [{**dict(row), "source_type": "vec"} for row in rows]
        except sqlite3.OperationalError:
            return []

    def _merge_results(
        self, fts: List[Dict], vec: List[Dict], limit: int
    ) -> List[Dict]:
        seen: set[int] = set()
        merged: List[Dict] = []
        for item in fts:
            row = dict(item)
            row["source_type"] = "hybrid"
            seen.add(row["id"])
            merged.append(row)
        for item in vec:
            if item["id"] in seen:
                continue
            row = dict(item)
            row["source_type"] = "hybrid"
            seen.add(row["id"])
            merged.append(row)
        merged.sort(key=lambda value: value.get("score", 0), reverse=True)
        return merged[:limit]

    def get_stats(self, event_id: Optional[str] = None) -> Dict[str, int | bool]:
        """Return basic FAQ store statistics."""
        with self.lock:
            if event_id:
                item_count = self.conn.execute(
                    "SELECT COUNT(*) FROM faq_items WHERE event_id = ?",
                    (event_id,),
                ).fetchone()[0]
            else:
                item_count = self.conn.execute(
                    "SELECT COUNT(*) FROM faq_items"
                ).fetchone()[0]

        return {
            "item_count": int(item_count),
            "vec_enabled": self._has_vec_table(),
            "ollama_available": self.embedder.is_available(),
        }
