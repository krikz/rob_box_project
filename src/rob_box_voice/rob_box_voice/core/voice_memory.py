#!/usr/bin/env python3
"""
voice_memory.py  Cross-session persistent memory for the voice assistant.

Architecture mirrors EchoVault (github.com/mraza007/echovault):
  - SQLite + FTS5 for keyword search (always available, no dependencies)
  - sqlite-vec + Ollama nomic-embed-text for semantic vector search (optional)
  - Hybrid tiered search: FTS5 first; if results sparse -> also vector search -> merge by score
  - Thread-safe (Lock + WAL mode)
  - Zero idle cost: Ollama embedder is lazy -- only called when a turn is saved,
    retried after backoff if temporarily unavailable

Two stores:
  voice_turns      -- every user/assistant exchange (+ FTS5 index + optional vec)
  voice_facts      -- user preferences / habits / names (persists across sessions)

Schema managed via migrations/002_voice_memory.sql (run at init).

Usage in DialogueNode:
    from rob_box_voice.core.voice_memory import VoiceMemory

    # Init once at node startup
    self.voice_memory = VoiceMemory(db_path="/data/voice_memory.db")

    # Reload context from previous sessions
    for turn in self.voice_memory.load_recent_turns(limit=15, exclude_current_session=True):
        if turn["role"] == "user":
            self.conversation_history.add_user_message(turn["content"])
        else:
            self.conversation_history.add_assistant_message(turn["content"])

    # After every user message:
    self.voice_memory.save_turn("user", text)

    # After every assistant reply:
    self.voice_memory.save_turn("assistant", reply)

    # Hybrid search (used by MemorySearchTool):
    results = self.voice_memory.search("где находится кухня", limit=5)
"""

import os
import sqlite3
import struct
import threading
import time
from datetime import datetime
from typing import Any, Dict, List, Optional, Tuple

# ---------------------------------------------------------------------------
# Optional: sqlite-vec extension for vector search
# ---------------------------------------------------------------------------
try:
    import sqlite_vec  # pip install sqlite-vec

    _SQLITE_VEC_AVAILABLE = True
except ImportError:
    _SQLITE_VEC_AVAILABLE = False


# ---------------------------------------------------------------------------
# Ollama embeddings (lazy, gracefully degrades)
# ---------------------------------------------------------------------------

class OllamaEmbedder:
    """
    Calls Ollama /api/embeddings endpoint for nomic-embed-text (768d).

    Lazy initialization -- does not fail at construction if Ollama is down.
    After a failed attempt, backs off for `retry_interval` seconds before
    the next try so we do not spam error logs.

    Default model: nomic-embed-text
      - 768 dimensions
      - Multilingual, including Russian
      - MIT license, runs on CPU (< 300 MB RAM)
      - Pull: `ollama pull nomic-embed-text`
    """

    DEFAULT_MODEL = "nomic-embed-text"
    DEFAULT_DIM = 768

    def __init__(
        self,
        base_url: str = "http://localhost:11434",
        model: str = DEFAULT_MODEL,
        timeout: float = 5.0,
        retry_interval: float = 60.0,
    ):
        self.base_url = base_url.rstrip("/")
        self.model = model
        self.timeout = timeout
        self.retry_interval = retry_interval
        self._dim: Optional[int] = None
        self._last_failure: float = 0.0
        self._available: Optional[bool] = None  # None = untried

    @property
    def dim(self) -> Optional[int]:
        return self._dim

    def _in_backoff(self) -> bool:
        return (
            self._available is False
            and (time.time() - self._last_failure) < self.retry_interval
        )

    def embed(self, text: str) -> Optional[List[float]]:
        """
        Return embedding vector for text, or None if Ollama unavailable.

        Never raises -- callers should treat None as "vector search disabled".
        """
        if self._in_backoff():
            return None

        try:
            import httpx  # already in requirements.txt (via openai)

            resp = httpx.post(
                f"{self.base_url}/api/embeddings",
                json={"model": self.model, "prompt": text},
                timeout=self.timeout,
            )
            resp.raise_for_status()
            vec = resp.json()["embedding"]
            if self._dim is None:
                self._dim = len(vec)
            self._available = True
            return vec
        except Exception:
            self._available = False
            self._last_failure = time.time()
            return None

    def is_available(self) -> bool:
        """Quick non-blocking availability check (uses cached state)."""
        return self._available is True


# ---------------------------------------------------------------------------
# VoiceMemory
# ---------------------------------------------------------------------------

class VoiceMemory:
    """
    Cross-session persistent memory for the ROBOX voice assistant.

    FTS5 keyword search always works.
    Vector search activates automatically once Ollama responds and
    sqlite-vec is installed.
    """

    # Hybrid search: if FTS returns fewer than this -> supplement with vector results
    FTS_MIN_RESULTS = 3

    def __init__(
        self,
        db_path: str,
        session_id: Optional[str] = None,
        migrations_dir: Optional[str] = None,
        ollama_base_url: Optional[str] = None,
        ollama_model: str = OllamaEmbedder.DEFAULT_MODEL,
        ollama_timeout: float = 5.0,
    ):
        """
        Args:
            db_path:          Absolute path to SQLite file (created if missing).
            session_id:       Current session tag. Defaults to "YYYYMMDD_HHMMSS".
            migrations_dir:   Override path to migrations/ directory.
            ollama_base_url:  Ollama HTTP base URL.
                              Default: env OLLAMA_BASE_URL or "http://localhost:11434".
            ollama_model:     Ollama embedding model. Default: "nomic-embed-text".
            ollama_timeout:   HTTP timeout per embedding request (seconds).
        """
        self.db_path = db_path
        self.session_id = session_id or datetime.now().strftime("%Y%m%d_%H%M%S")
        self.lock = threading.Lock()

        os.makedirs(os.path.dirname(os.path.abspath(db_path)), exist_ok=True)

        self.conn = sqlite3.connect(db_path, check_same_thread=False)
        self.conn.row_factory = sqlite3.Row

        with self.lock:
            self.conn.execute("PRAGMA foreign_keys = ON")
            self.conn.execute("PRAGMA journal_mode = WAL")
            self.conn.execute("PRAGMA synchronous = NORMAL")

        # Resolve migrations dir (4 levels up from core/ -> project root/migrations)
        if migrations_dir:
            self.migrations_dir = migrations_dir
        else:
            self.migrations_dir = os.path.normpath(
                os.path.join(
                    os.path.dirname(__file__),
                    "..", "..", "..", "..", "migrations",
                )
            )

        self._run_migrations()

        # Load sqlite-vec extension if available
        self._vec_loaded = self._load_vec_extension()

        # Embedder (lazy -- will try on first save_turn call)
        base_url = (
            ollama_base_url
            or os.getenv("OLLAMA_BASE_URL", "http://localhost:11434")
        )
        self.embedder = OllamaEmbedder(
            base_url=base_url,
            model=ollama_model,
            timeout=ollama_timeout,
        )

        # Restore stored embedding dim and ensure vec table exists
        stored_dim = self._get_meta_int("embedding_dim")
        if stored_dim and self._vec_loaded:
            self._ensure_vec_table(stored_dim)

    # ------------------------------------------------------------------
    # Schema management
    # ------------------------------------------------------------------

    def _run_migrations(self) -> None:
        """Apply pending SQL migrations (tracked by PRAGMA user_version)."""
        if not os.path.isdir(self.migrations_dir):
            self._create_schema_inline()
            return

        with self.lock:
            current = self.conn.execute("PRAGMA user_version").fetchone()[0]

        for fname in sorted(f for f in os.listdir(self.migrations_dir) if f.endswith(".sql")):
            try:
                ver = int(fname.split("_", 1)[0])
            except ValueError:
                continue
            if ver <= current:
                continue
            sql_path = os.path.join(self.migrations_dir, fname)
            with open(sql_path, "r", encoding="utf-8") as fh:
                sql = fh.read()
            with self.lock, self.conn:
                self.conn.executescript(sql)
                self.conn.execute(f"PRAGMA user_version = {ver}")

    def _create_schema_inline(self) -> None:
        """Fallback schema when migrations/ is absent."""
        schema = """
        CREATE TABLE IF NOT EXISTS voice_turns (
            id         INTEGER PRIMARY KEY AUTOINCREMENT,
            session_id TEXT NOT NULL,
            role       TEXT NOT NULL,
            content    TEXT NOT NULL,
            timestamp  REAL NOT NULL
        );
        CREATE INDEX IF NOT EXISTS idx_vt_session   ON voice_turns(session_id);
        CREATE INDEX IF NOT EXISTS idx_vt_timestamp ON voice_turns(timestamp);

        CREATE VIRTUAL TABLE IF NOT EXISTS voice_turns_fts USING fts5(
            content,
            content       = voice_turns,
            content_rowid = id,
            tokenize      = 'unicode61'
        );
        CREATE TRIGGER IF NOT EXISTS voice_turns_ai AFTER INSERT ON voice_turns BEGIN
            INSERT INTO voice_turns_fts(rowid, content) VALUES (new.id, new.content);
        END;
        CREATE TRIGGER IF NOT EXISTS voice_turns_au AFTER UPDATE OF content ON voice_turns BEGIN
            INSERT INTO voice_turns_fts(voice_turns_fts, rowid, content) VALUES ('delete', old.id, old.content);
            INSERT INTO voice_turns_fts(rowid, content) VALUES (new.id, new.content);
        END;
        CREATE TRIGGER IF NOT EXISTS voice_turns_ad AFTER DELETE ON voice_turns BEGIN
            INSERT INTO voice_turns_fts(voice_turns_fts, rowid, content) VALUES ('delete', old.id, old.content);
        END;

        CREATE TABLE IF NOT EXISTS voice_facts (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            fact TEXT NOT NULL,
            category TEXT NOT NULL DEFAULT 'general',
            created_at REAL NOT NULL,
            updated_at REAL NOT NULL
        );

        CREATE TABLE IF NOT EXISTS voice_memory_meta (
            key   TEXT PRIMARY KEY,
            value TEXT NOT NULL
        );
        """
        with self.lock, self.conn:
            self.conn.executescript(schema)

    # ------------------------------------------------------------------
    # sqlite-vec extension
    # ------------------------------------------------------------------

    def _load_vec_extension(self) -> bool:
        """Load sqlite-vec extension. Returns True if successful."""
        if not _SQLITE_VEC_AVAILABLE:
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
        """
        Create voice_turns_vec(dim) if it does not exist yet.
        On dimension mismatch: drops and recreates (reindex needed).
        Returns True if the table is ready.
        """
        if not self._vec_loaded:
            return False
        stored = self._get_meta_int("embedding_dim")
        if stored and stored != dim:
            # Dimension changed (different model) -- drop and recreate
            with self.lock, self.conn:
                self.conn.execute("DROP TABLE IF EXISTS voice_turns_vec")
            self._set_meta("embedding_dim", str(dim))
        elif stored is None:
            self._set_meta("embedding_dim", str(dim))

        try:
            with self.lock, self.conn:
                self.conn.execute(
                    f"CREATE VIRTUAL TABLE IF NOT EXISTS voice_turns_vec "
                    f"USING vec0(rowid INTEGER PRIMARY KEY, embedding float[{dim}])"
                )
            return True
        except Exception:
            return False

    def _has_vec_table(self) -> bool:
        with self.lock:
            row = self.conn.execute(
                "SELECT name FROM sqlite_master WHERE type='table' AND name='voice_turns_vec'"
            ).fetchone()
        return row is not None

    # ------------------------------------------------------------------
    # Meta helpers
    # ------------------------------------------------------------------

    def _get_meta(self, key: str) -> Optional[str]:
        try:
            with self.lock:
                row = self.conn.execute(
                    "SELECT value FROM voice_memory_meta WHERE key = ?", (key,)
                ).fetchone()
            return row["value"] if row else None
        except Exception:
            return None

    def _get_meta_int(self, key: str) -> Optional[int]:
        val = self._get_meta(key)
        return int(val) if val is not None else None

    def _set_meta(self, key: str, value: str) -> None:
        with self.lock, self.conn:
            self.conn.execute(
                "INSERT OR REPLACE INTO voice_memory_meta (key, value) VALUES (?, ?)",
                (key, value),
            )

    # ------------------------------------------------------------------
    # Conversation turns
    # ------------------------------------------------------------------

    def save_turn(
        self,
        role: str,
        content: str,
        session_id: Optional[str] = None,
    ) -> int:
        """
        Persist one dialogue turn and (optionally) its embedding vector.

        Args:
            role:       "user" or "assistant".
            content:    Message text.
            session_id: Override session; defaults to current session.

        Returns:
            Row ID, or -1 if content is empty.
        """
        if not content or not content.strip():
            return -1

        sid = session_id or self.session_id
        with self.lock, self.conn:
            cur = self.conn.execute(
                "INSERT INTO voice_turns (session_id, role, content, timestamp) VALUES (?, ?, ?, ?)",
                (sid, role, content.strip(), time.time()),
            )
            rowid = cur.lastrowid

        # Generate and store embedding (non-blocking: failure is silent)
        self._embed_and_store(rowid, content)
        return rowid

    def _embed_and_store(self, rowid: int, text: str) -> None:
        """Generate embedding for text and store in vec table (best-effort)."""
        vec = self.embedder.embed(text)
        if vec is None:
            return  # Ollama not available yet

        # Ensure vec table exists with correct dim
        if not self._has_vec_table():
            ok = self._ensure_vec_table(len(vec))
            if not ok:
                return

        stored_dim = self._get_meta_int("embedding_dim")
        if stored_dim and len(vec) != stored_dim:
            return  # Dimension mismatch -- skip silently

        try:
            vec_bytes = struct.pack(f"{len(vec)}f", *vec)
            with self.lock, self.conn:
                self.conn.execute(
                    "INSERT OR REPLACE INTO voice_turns_vec (rowid, embedding) VALUES (?, ?)",
                    (rowid, vec_bytes),
                )
        except Exception:
            pass

    def load_recent_turns(
        self,
        limit: int = 20,
        exclude_current_session: bool = False,
    ) -> List[Dict]:
        """
        Return recent turns in chronological order (oldest first).

        Intended for startup context restoration into ConversationHistory.

        Args:
            limit:                    Max number of turns.
            exclude_current_session:  Skip current session (avoid duplicates).
        """
        if exclude_current_session:
            q = (
                "SELECT id, session_id, role, content, timestamp "
                "FROM voice_turns WHERE session_id != ? "
                "ORDER BY timestamp DESC LIMIT ?"
            )
            params: Tuple = (self.session_id, limit)
        else:
            q = (
                "SELECT id, session_id, role, content, timestamp "
                "FROM voice_turns ORDER BY timestamp DESC LIMIT ?"
            )
            params = (limit,)

        with self.lock:
            rows = self.conn.execute(q, params).fetchall()
        return list(reversed([dict(r) for r in rows]))

    # ------------------------------------------------------------------
    # Hybrid search (FTS5 + optional vector, tiered like EchoVault)
    # ------------------------------------------------------------------

    def search(self, query: str, limit: int = 5) -> List[Dict]:
        """
        Hybrid tiered search over stored conversation turns.

        Strategy (mirrors EchoVault tiered_search):
          1. FTS5 BM25 keyword search -- fast, always available.
          2. If FTS returns < FTS_MIN_RESULTS AND Ollama is available:
             also run vector search, merge by score, deduplicate.

        Args:
            query: Natural language or keyword query (Russian / English).
            limit: Max number of results.

        Returns:
            List of dicts: {id, session_id, role, content, timestamp, score, source}
            source = "fts" | "vec" | "hybrid"
        """
        if not query or not query.strip():
            return []

        fts_results = self._fts_search(query, limit)

        # Supplement with vector search if results are sparse
        if len(fts_results) < self.FTS_MIN_RESULTS and self.embedder.is_available():
            vec_results = self._vector_search(query, limit)
            merged = self._merge_results(fts_results, vec_results, limit)
            return merged

        return fts_results

    def _fts_search(self, query: str, limit: int) -> List[Dict]:
        """FTS5 BM25 search using prefix-OR query."""
        tokens = [f'"{w}"*' for w in query.split() if w]
        if not tokens:
            return []
        fts_query = " OR ".join(tokens)
        try:
            with self.lock:
                rows = self.conn.execute(
                    """
                    SELECT vt.id, vt.session_id, vt.role, vt.content, vt.timestamp,
                           (-bm25(voice_turns_fts)) AS score
                    FROM voice_turns_fts
                    JOIN voice_turns vt ON vt.id = voice_turns_fts.rowid
                    WHERE voice_turns_fts MATCH ?
                    ORDER BY score DESC
                    LIMIT ?
                    """,
                    (fts_query, limit),
                ).fetchall()
            return [{**dict(r), "source": "fts"} for r in rows]
        except sqlite3.OperationalError:
            return []

    def _vector_search(self, query: str, limit: int) -> List[Dict]:
        """Semantic vector search via sqlite-vec KNN."""
        if not self._has_vec_table():
            return []

        vec = self.embedder.embed(query)
        if vec is None:
            return []

        vec_bytes = struct.pack(f"{len(vec)}f", *vec)
        try:
            with self.lock:
                rows = self.conn.execute(
                    """
                    SELECT vt.id, vt.session_id, vt.role, vt.content, vt.timestamp,
                           (1.0 - v.distance) AS score
                    FROM voice_turns_vec v
                    JOIN voice_turns vt ON vt.id = v.rowid
                    WHERE v.embedding MATCH ? AND k = ?
                    ORDER BY v.distance
                    """,
                    (vec_bytes, limit),
                ).fetchall()
            return [{**dict(r), "source": "vec"} for r in rows]
        except sqlite3.OperationalError:
            return []

    def _merge_results(
        self,
        fts: List[Dict],
        vec: List[Dict],
        limit: int,
    ) -> List[Dict]:
        """Merge FTS + vector results, deduplicate by id, sort by score."""
        seen: set = set()
        merged: List[Dict] = []
        for item in fts:
            item = dict(item)
            item["source"] = "hybrid"
            seen.add(item["id"])
            merged.append(item)
        for item in vec:
            if item["id"] not in seen:
                item = dict(item)
                item["source"] = "hybrid"
                seen.add(item["id"])
                merged.append(item)
        merged.sort(key=lambda x: x.get("score", 0), reverse=True)
        return merged[:limit]

    # ------------------------------------------------------------------
    # User facts / preferences
    # ------------------------------------------------------------------

    def save_fact(self, fact: str, category: str = "general") -> int:
        """
        Store a user fact or preference.

        Args:
            fact:     Human-readable text, e.g. "User prefers short answers".
            category: 'preference' | 'habit' | 'name' | 'general'.

        Returns:
            Row ID.
        """
        now = time.time()
        with self.lock, self.conn:
            cur = self.conn.execute(
                "INSERT INTO voice_facts (fact, category, created_at, updated_at) VALUES (?, ?, ?, ?)",
                (fact, category, now, now),
            )
            return cur.lastrowid

    def update_fact(self, fact_id: int, fact: str) -> bool:
        """Update an existing fact text. Returns True if updated."""
        with self.lock, self.conn:
            cur = self.conn.execute(
                "UPDATE voice_facts SET fact = ?, updated_at = ? WHERE id = ?",
                (fact, time.time(), fact_id),
            )
            return cur.rowcount > 0

    def delete_fact(self, fact_id: int) -> bool:
        """Delete a fact by ID. Returns True if deleted."""
        with self.lock, self.conn:
            cur = self.conn.execute("DELETE FROM voice_facts WHERE id = ?", (fact_id,))
            return cur.rowcount > 0

    def get_facts(
        self,
        category: Optional[str] = None,
        limit: int = 20,
    ) -> List[Dict]:
        """
        Retrieve stored user facts.

        Args:
            category: Optional filter ('preference', 'habit', 'name', 'general').
            limit:    Max results.

        Returns:
            List of {id, fact, category, created_at, updated_at}.
        """
        if category:
            q = (
                "SELECT id, fact, category, created_at, updated_at "
                "FROM voice_facts WHERE category = ? "
                "ORDER BY updated_at DESC LIMIT ?"
            )
            params: Any = (category, limit)
        else:
            q = (
                "SELECT id, fact, category, created_at, updated_at "
                "FROM voice_facts ORDER BY updated_at DESC LIMIT ?"
            )
            params = (limit,)

        with self.lock:
            rows = self.conn.execute(q, params).fetchall()
        return [dict(r) for r in rows]

    def format_facts_for_prompt(self) -> str:
        """
        Format stored facts as a string block for injection into system prompt.

        Returns:
            Multi-line string, or empty string if no facts.
        """
        facts = self.get_facts()
        if not facts:
            return ""
        lines = [f"- {f['fact']}" for f in facts]
        return "Known user facts:\n" + "\n".join(lines)

    # ------------------------------------------------------------------
    # Context for MCP memory_context tool
    # ------------------------------------------------------------------

    def get_context(self, limit: int = 10, query: Optional[str] = None) -> Dict:
        """
        Get memory context for injection into LLM (used by MemoryContextTool).

        Returns:
            Dict with keys: recent_turns, facts, total_turns, sessions, vec_enabled
        """
        if query:
            turns = self.search(query, limit=limit)
        else:
            turns = self.load_recent_turns(limit=limit, exclude_current_session=True)

        stats = self.get_stats()
        return {
            "recent_turns": turns,
            "facts": self.get_facts(),
            "total_turns": stats["turn_count"],
            "sessions": stats["session_count"],
            "vec_enabled": self._has_vec_table() and self.embedder.is_available(),
            "current_session": self.session_id,
        }

    # ------------------------------------------------------------------
    # Stats & maintenance
    # ------------------------------------------------------------------

    def get_stats(self) -> Dict:
        """Return memory statistics."""
        with self.lock:
            turn_count = self.conn.execute("SELECT COUNT(*) FROM voice_turns").fetchone()[0]
            session_count = self.conn.execute(
                "SELECT COUNT(DISTINCT session_id) FROM voice_turns"
            ).fetchone()[0]
            fact_count = self.conn.execute("SELECT COUNT(*) FROM voice_facts").fetchone()[0]

        vec_count = 0
        if self._has_vec_table():
            try:
                with self.lock:
                    vec_count = self.conn.execute(
                        "SELECT COUNT(*) FROM voice_turns_vec"
                    ).fetchone()[0]
            except Exception:
                pass

        try:
            db_size_kb = os.path.getsize(self.db_path) // 1024
        except OSError:
            db_size_kb = 0

        return {
            "turn_count": turn_count,
            "session_count": session_count,
            "fact_count": fact_count,
            "vec_count": vec_count,
            "db_size_kb": db_size_kb,
            "vec_enabled": self._has_vec_table(),
            "ollama_available": self.embedder.is_available(),
            "current_session": self.session_id,
        }

    def prune_older_than(self, days: int = 30) -> int:
        """Delete conversation turns older than N days. Returns deleted count."""
        cutoff = time.time() - days * 86400
        with self.lock, self.conn:
            cur = self.conn.execute(
                "DELETE FROM voice_turns WHERE timestamp < ?", (cutoff,)
            )
            return cur.rowcount

    def reindex_embeddings(self, progress_callback=None) -> Dict:
        """
        Re-embed all turns using current Ollama model.

        Call this after:
        - Changing Ollama model
        - First time Ollama becomes available after a long offline period

        Returns:
            Dict: {indexed, skipped, total, dim}
        """
        if not self.embedder.is_available():
            return {"indexed": 0, "skipped": 0, "total": 0, "dim": None, "error": "Ollama not available"}

        # Probe dimension
        probe = self.embedder.embed("probe")
        if probe is None:
            return {"indexed": 0, "skipped": 0, "total": 0, "dim": None, "error": "Failed to get embedding"}

        dim = len(probe)
        self._ensure_vec_table(dim)

        with self.lock:
            turns = self.conn.execute("SELECT id, content FROM voice_turns ORDER BY id").fetchall()

        total = len(turns)
        indexed = 0
        skipped = 0

        for i, row in enumerate(turns):
            vec = self.embedder.embed(row["content"])
            if vec is None:
                skipped += 1
                continue
            vec_bytes = struct.pack(f"{len(vec)}f", *vec)
            try:
                with self.lock, self.conn:
                    self.conn.execute(
                        "INSERT OR REPLACE INTO voice_turns_vec (rowid, embedding) VALUES (?, ?)",
                        (row["id"], vec_bytes),
                    )
                indexed += 1
            except Exception:
                skipped += 1

            if progress_callback:
                progress_callback(i + 1, total)

        return {"indexed": indexed, "skipped": skipped, "total": total, "dim": dim}

    def close(self) -> None:
        """Close the database connection gracefully."""
        try:
            with self.lock:
                self.conn.close()
        except Exception:
            pass
