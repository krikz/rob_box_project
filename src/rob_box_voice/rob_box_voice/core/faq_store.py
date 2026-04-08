"""Persistent FAQ knowledge store for event mode."""

from __future__ import annotations

import os
import sqlite3
import struct
import threading
from datetime import datetime
from typing import Dict, List, Optional

from .voice_memory import _SQLITE_VEC_AVAILABLE, OllamaEmbedder

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
            self.migrations_dir = os.path.normpath(
                os.path.join(
                    os.path.dirname(__file__),
                    "..",
                    "..",
                    "..",
                    "..",
                    "migrations",
                )
            )

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

    def _run_migrations(self) -> None:
        with self.lock:
            current = self.conn.execute("PRAGMA user_version").fetchone()[0]

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

    def _fts_search(
        self, query: str, event_id: Optional[str], limit: int
    ) -> List[Dict]:
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
