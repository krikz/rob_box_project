"""Учёт кредитов: таблица ``gen_ledger`` в ``index.db`` библиотеки (ADR-0158 §2.6).

Пишется ``creditsConsumed`` из ответа провайдера (факт, не оценка). Одна трата на
``task_id`` — повтор записи (импорт, ретрай) безопасен.
"""

from __future__ import annotations

import sqlite3
import threading
from datetime import datetime, timezone
from typing import Optional

LEDGER_SQL = """
CREATE TABLE IF NOT EXISTS gen_ledger (
    task_id       TEXT PRIMARY KEY,
    ts            TEXT NOT NULL,
    credits       REAL NOT NULL,
    kind          TEXT NOT NULL DEFAULT '',
    request_key   TEXT NOT NULL DEFAULT '',
    model         TEXT NOT NULL DEFAULT '',
    gen_provider  TEXT NOT NULL DEFAULT ''
);
CREATE INDEX IF NOT EXISTS idx_gen_ledger_ts ON gen_ledger (ts);
CREATE TABLE IF NOT EXISTS gen_rejected (
    source_sha    TEXT PRIMARY KEY,
    ts            TEXT NOT NULL,
    task_id       TEXT NOT NULL DEFAULT '',
    work_id       TEXT NOT NULL DEFAULT '',
    detail        TEXT NOT NULL DEFAULT ''
);
"""


class GenLedger:
    def __init__(self, conn: sqlite3.Connection, lock: threading.Lock) -> None:
        self._conn = conn
        self._lock = lock

    def record_spend(
        self,
        task_id: str,
        credits: float,
        *,
        kind: str = "",
        request_key: str = "",
        model: str = "",
        gen_provider: str = "",
        ts: Optional[str] = None,
    ) -> bool:
        """Записать трату. Повтор того же ``task_id`` игнорируется (``False``)."""
        stamp = ts or datetime.now(timezone.utc).replace(microsecond=0).isoformat()
        with self._lock:
            cur = self._conn.execute(
                "INSERT OR IGNORE INTO gen_ledger "
                "(task_id, ts, credits, kind, request_key, model, gen_provider) "
                "VALUES (?, ?, ?, ?, ?, ?, ?)",
                (task_id, stamp, float(credits), kind, request_key, model, gen_provider),
            )
            self._conn.commit()
            return cur.rowcount > 0

    def settle(self, task_id: str, credits: float) -> None:
        """Факт трат по ``complete`` вместо оценки, записанной при раннем звуке (``pipeline``)."""
        with self._lock:
            self._conn.execute("UPDATE gen_ledger SET credits = ? WHERE task_id = ?", (float(credits), task_id))
            self._conn.commit()

    def record_rejected(self, source_sha: str, *, task_id: str = "", work_id: str = "", detail: str = "") -> None:
        """Исходник кавера (sha нашего рендера), который Suno отклонил (ADR-0158 В17): повторно не заказывается."""
        stamp = datetime.now(timezone.utc).replace(microsecond=0).isoformat()
        with self._lock:
            self._conn.execute("INSERT OR IGNORE INTO gen_rejected (source_sha, ts, task_id, work_id, detail) "
                               "VALUES (?, ?, ?, ?, ?)", (source_sha, stamp, task_id, work_id, detail))
            self._conn.commit()

    def rejected(self, source_sha: str) -> bool:
        with self._lock:
            row = self._conn.execute("SELECT 1 FROM gen_rejected WHERE source_sha = ?", (source_sha,)).fetchone()
        return row is not None

    def spent_since(self, ts_iso: str) -> float:
        """Сумма кредитов с момента ``ts_iso`` (UTC ISO)."""
        with self._lock:
            row = self._conn.execute(
                "SELECT COALESCE(SUM(credits), 0) FROM gen_ledger WHERE ts >= ?", (ts_iso,),
            ).fetchone()
        return float(row[0])
