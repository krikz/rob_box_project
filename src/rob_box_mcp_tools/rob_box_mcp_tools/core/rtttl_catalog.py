"""rtttl_catalog.py — построчный обход и пополнение :class:`RtttlLibrary` (issue #3225).

Вынесено из класса библиотеки (бюджет размера класса, ADR-0145): club-фрагменты
(:mod:`core.club_fragments`) один раз обходят архив, а карточки внешних
мелодий (г/д umbrella #3223) дописывают в него найденное.
"""

from __future__ import annotations

import json
from datetime import datetime, timezone
from typing import Any, Dict, Iterator, List, Optional

__all__ = ["add_melody", "iter_melodies", "melody_by_rowid"]


def iter_melodies(library: Any, batch: int = 500) -> Iterator[Dict[str, Any]]:
    """Все записи архива порциями (``rowid``, ``name``, ``rtttl``) — без загрузки в память.

    Порядок — по ``rowid`` (детерминирован).
    """
    last = 0
    while True:
        with library._lock:
            rows = library._conn.execute(
                "SELECT id, name, rtttl FROM rtttl_melodies WHERE id > ? ORDER BY id LIMIT ?",
                (last, int(batch)),
            ).fetchall()
        if not rows:
            return
        for row in rows:
            yield {"rowid": row["id"], "name": row["name"], "rtttl": row["rtttl"]}
        last = rows[-1]["id"]


def melody_by_rowid(library: Any, rowid: int) -> Optional[Dict[str, Any]]:
    """Полная запись (с ``rtttl``) по ``rowid`` из :func:`iter_melodies`; ``None`` — нет."""
    with library._lock:
        row = library._conn.execute("SELECT * FROM rtttl_melodies WHERE id = ?", (int(rowid),)).fetchone()
    return None if row is None else library._to_dict(row, include_rtttl=True)


def add_melody(
    library: Any, rtttl: str, name: str, title: str = "", artist: str = "", source: str = "",
    tags: Optional[List[str]] = None,
) -> bool:
    """Дописать мелодию в библиотеку (внешние ноты/мелодии карточек г и д).

    ``source`` и ``tags`` — метка происхождения (напр. ``source="dj-web"``,
    ``tags=["web", "dendy"]``). Дубликат по тексту RTTTL не пишется.

    Returns:
        ``True`` — добавлена; ``False`` — такая RTTTL-строка уже есть.
    """
    now = datetime.now(timezone.utc).isoformat()
    rtttl_name = rtttl.split(":", 1)[0].strip() if ":" in rtttl else ""
    with library._lock, library._conn:
        cur = library._conn.execute(
            "INSERT OR IGNORE INTO rtttl_melodies "
            "(name, title, artist, source, tags, rtttl, rtttl_name, created_at, updated_at) "
            "VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)",
            (name, title or name, artist, source, json.dumps(tags or [], ensure_ascii=False),
             rtttl.strip(), rtttl_name, now, now),
        )
    return cur.rowcount > 0
