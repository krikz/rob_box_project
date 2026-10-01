"""rtttl_catalog.py — построчный обход и пополнение :class:`RtttlLibrary` (issue #3225).

Вынесено из класса библиотеки (бюджет размера класса, ADR-0145): club-фрагменты
(:mod:`core.club_fragments`) один раз обходят архив, а карточки внешних
мелодий (г/д umbrella #3223) дописывают в него найденное.
"""

from __future__ import annotations

import json
from datetime import datetime, timezone
from typing import Any, Dict, Iterator, List, Optional

__all__ = ["add_melody", "iter_melodies", "melody_by_rowid", "melodies_by_tag", "purge_web_melodies"]


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


def melodies_by_tag(library: Any, tag: str, limit: int = 20) -> List[Dict[str, Any]]:
    """Записи (с ``rtttl``), у которых в ``tags`` точно есть ``tag`` (issue #3228).

    Поиск библиотеки токенизирует запрос по латинице (кириллица отбрасывается),
    поэтому мелодию русской темы из веба он не находит — её ищут по тегу.
    """
    if not tag:
        return []
    needle = "%" + json.dumps(tag, ensure_ascii=False) + "%"
    with library._lock:
        rows = library._conn.execute(
            "SELECT * FROM rtttl_melodies WHERE tags LIKE ? ORDER BY id LIMIT ?", (needle, int(limit)),
        ).fetchall()
    return [library._to_dict(row, include_rtttl=True) for row in rows]


def _age_s(created_at: str, now: datetime) -> float:
    try:
        return (now - datetime.fromisoformat(created_at)).total_seconds()
    except (TypeError, ValueError):
        return float("inf")  # нет/битая дата — считаем протухшей


def purge_web_melodies(library: Any, tag: str, ttl_s: float, keep_tag: str) -> int:
    """Удалить веб-мелодии темы (``source="web"``, тег ``tag``), которым нельзя верить (issue #3243).

    Протухшие (старше ``ttl_s``) и без отметки проверенной релевантности
    ``keep_tag`` (записаны до проверки, как «космос» ← демо rtttl.js). Вернуть
    число удалённых. Записи других источников (архив) не трогаются.
    """
    if not tag:
        return 0
    now = datetime.now(timezone.utc)
    needle = "%" + json.dumps(tag, ensure_ascii=False) + "%"
    with library._lock, library._conn:
        rows = library._conn.execute(
            "SELECT id, tags, created_at FROM rtttl_melodies WHERE source = 'web' AND tags LIKE ?", (needle,),
        ).fetchall()
        stale = [r["id"] for r in rows
                 if keep_tag not in json.loads(r["tags"] or "[]") or _age_s(r["created_at"], now) > ttl_s]
        for rowid in stale:
            library._conn.execute("DELETE FROM rtttl_melodies WHERE id = ?", (rowid,))
    return len(stale)
