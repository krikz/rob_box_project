"""
SQLiteLongTermMemory helper for rob_box_perception
Creates DB file if missing, runs SQL migrations from package migrations/ and provides API:
- save_event(event_dict) -> int
- save_summary(category, summary_text, event_ids) -> int
- mark_events_summarized(event_ids)
- get_recent_summaries(category=None, limit=5)
- prune_older_than(days)

Designed to be thread-safe for ROS2 node use.
"""

import os
import sqlite3
import json
import time
import threading
from typing import List, Optional, Dict

class SQLiteLongTermMemory:
    def __init__(self, db_path: str, migrations_dir: Optional[str] = None):
        self.db_path = db_path
        self.lock = threading.Lock()
        os.makedirs(os.path.dirname(self.db_path), exist_ok=True)

        # connect (allow multithread)
        self.conn = sqlite3.connect(self.db_path, check_same_thread=False)
        self.conn.row_factory = sqlite3.Row

        # pragmas for safety/perf
        with self.lock:
            self.conn.execute('PRAGMA foreign_keys = ON;')
            try:
                self.conn.execute('PRAGMA journal_mode = WAL;')
            except Exception:
                pass
            try:
                self.conn.execute('PRAGMA synchronous = NORMAL;')
            except Exception:
                pass

        # determine migrations dir
        if migrations_dir:
            self.migrations_dir = migrations_dir
        else:
            # default migrations path relative to project root (four levels up from utils/)
            self.migrations_dir = os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', 'migrations')
            self.migrations_dir = os.path.normpath(self.migrations_dir)

        # init DB (create tables or run migrations)
        self._ensure_db()

    def _ensure_db(self):
        # if migrations dir exists, run migrations, else attempt to create basic schema
        if os.path.isdir(self.migrations_dir):
            self._run_migrations()
        else:
            self._create_basic_schema()

    def _create_basic_schema(self):
        sql = """
        CREATE TABLE IF NOT EXISTS events (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            category TEXT NOT NULL,
            time REAL NOT NULL,
            content TEXT NOT NULL,
            important INTEGER DEFAULT 0,
            summarized INTEGER DEFAULT 0
        );
        CREATE INDEX IF NOT EXISTS idx_events_time ON events(time);
        CREATE INDEX IF NOT EXISTS idx_events_summarized ON events(summarized);

        CREATE TABLE IF NOT EXISTS summaries (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            category TEXT NOT NULL,
            time REAL NOT NULL,
            summary TEXT NOT NULL,
            event_count INTEGER NOT NULL
        );

        CREATE TABLE IF NOT EXISTS summary_events (
            summary_id INTEGER NOT NULL,
            event_id INTEGER NOT NULL,
            PRIMARY KEY (summary_id, event_id),
            FOREIGN KEY(summary_id) REFERENCES summaries(id) ON DELETE CASCADE,
            FOREIGN KEY(event_id) REFERENCES events(id) ON DELETE CASCADE
        );
        """
        with self.lock, self.conn:
            self.conn.executescript(sql)

    def _run_migrations(self):
        # Use PRAGMA user_version to track applied migrations
        with self.lock:
            cur = self.conn.execute('PRAGMA user_version')
            current = cur.fetchone()[0]

        # find migration files named NN_*.sql
        files = sorted([f for f in os.listdir(self.migrations_dir) if f.endswith('.sql')])
        to_apply = []
        for f in files:
            try:
                ver = int(f.split('_', 1)[0])
            except Exception:
                continue
            if ver > current:
                to_apply.append((ver, f))

        if not to_apply:
            return

        for ver, fname in to_apply:
            path = os.path.join(self.migrations_dir, fname)
            with open(path, 'r', encoding='utf-8') as fh:
                sql = fh.read()
            with self.lock, self.conn:
                self.conn.executescript(sql)
                # set user_version to ver
                self.conn.execute(f'PRAGMA user_version = {ver}')

    def save_event(self, event: Dict) -> int:
        """Save event dict with keys: type/category (prefer 'type' or 'category'), time, content, important"""
        category = event.get('type') or event.get('category') or 'unknown'
        t = float(event.get('time', time.time()))
        content = event.get('content')
        if not isinstance(content, str):
            try:
                content = json.dumps(content, ensure_ascii=False)
            except (TypeError, ValueError) as e:
                # Log the error and fall back to string conversion
                content = str(content)
        important = 1 if event.get('important') else 0

        with self.lock, self.conn:
            cur = self.conn.execute(
                'INSERT INTO events (category, time, content, important) VALUES (?, ?, ?, ?)',
                (category, t, content, important)
            )
            return cur.lastrowid

    def save_summary(self, category: str, summary_text: str, event_ids: List[int]) -> int:
        t = time.time()
        with self.lock, self.conn:
            cur = self.conn.execute(
                'INSERT INTO summaries (category, time, summary, event_count) VALUES (?, ?, ?, ?)',
                (category, t, summary_text, len(event_ids))
            )
            summary_id = cur.lastrowid
            if event_ids:
                self.conn.executemany(
                    'INSERT OR IGNORE INTO summary_events (summary_id, event_id) VALUES (?, ?)',
                    [(summary_id, int(eid)) for eid in event_ids]
                )
            return summary_id

    def mark_events_summarized(self, event_ids: List[int]):
        if not event_ids:
            return
        with self.lock, self.conn:
            self.conn.executemany('UPDATE events SET summarized = 1 WHERE id = ?', [(int(eid),) for eid in event_ids])

    def get_recent_summaries(self, category: Optional[str] = None, limit: int = 5) -> List[Dict]:
        q = 'SELECT id, category, time, summary, event_count FROM summaries '
        params = []
        if category:
            q += 'WHERE category = ? '
            params.append(category)
        q += 'ORDER BY time DESC LIMIT ?'
        params.append(limit)
        with self.lock:
            rows = self.conn.execute(q, params).fetchall()
        return [dict(r) for r in rows]

    def get_unsummarized_events(self, category: Optional[str] = None, older_than: Optional[float] = None, limit: Optional[int] = None) -> List[Dict]:
        q = 'SELECT id, category, time, content, important FROM events WHERE summarized = 0 '
        params = []
        if category:
            q += 'AND category = ? '
            params.append(category)
        if older_than:
            q += 'AND time <= ? '
            params.append(float(older_than))
        q += 'ORDER BY time ASC '
        if limit:
            q += 'LIMIT ?'
            params.append(int(limit))
        with self.lock:
            rows = self.conn.execute(q, params).fetchall()
        return [dict(r) for r in rows]

    def prune_older_than(self, days: int):
        cutoff = time.time() - float(days) * 24 * 3600
        with self.lock, self.conn:
            # delete old summaries (and cascade will remove summary_events if FK set)
            self.conn.execute('DELETE FROM summaries WHERE time <= ?', (cutoff,))
            self.conn.execute('DELETE FROM events WHERE time <= ? AND summarized = 1', (cutoff,))

    def close(self):
        """Close the database connection."""
        try:
            with self.lock:
                self.conn.close()
        except sqlite3.ProgrammingError:
            # Connection already closed, ignore
            pass
        except Exception as e:
            # Log unexpected errors during cleanup
            import logging
            logging.warning(f"Error closing LongTermMemory database: {e}")