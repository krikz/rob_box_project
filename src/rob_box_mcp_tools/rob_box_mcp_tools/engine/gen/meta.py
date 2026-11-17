"""Метаданные и оценка треков кэша (ADR-0158 §2.5, PR-9): правка полей, «нравится / не нравится», «что играло последним».

Живёт рядом с ``GeneratedMusicLibrary`` (``library.meta``) и пишет в тот же ``index.db`` и ``meta.json`` треков.
Оценка — число ``generated_tracks.rating``: ``kn.RATING_LIKE`` / ``kn.RATING_DISLIKE`` / 0 (нет). Путь музыки
(``rob_box_music.source``) не берёт в кандидаты треки с оценкой ниже нуля; квота диска (``quota``) вычищает их первыми.
"""

from __future__ import annotations

import json
import sqlite3
import threading
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, Optional


def row_to_dict(row: sqlite3.Row) -> Dict[str, Any]:
    """Строка ``generated_tracks`` → словарь: ``tags`` списком, ``is_instrumental`` булевым."""
    d = dict(row)
    try:
        d["tags"] = json.loads(d.get("tags") or "[]")
    except json.JSONDecodeError:
        d["tags"] = []
    d["is_instrumental"] = bool(d.get("is_instrumental"))
    return d


#: Поля, которые можно менять после генерации.
EDITABLE = ("title", "tags", "mood", "genre", "notes", "rating")


class TrackMeta:
    def __init__(self, conn: sqlite3.Connection, lock: threading.Lock, root: Path) -> None:
        self._conn = conn
        self._lock = lock
        self._root = root

    def update(self, track_id: str, **fields: Any) -> Dict[str, Any]:
        """Обновить поля трека (``EDITABLE``; ``tags`` — список) в индексе и в ``meta.json``."""
        unknown = set(fields) - set(EDITABLE)
        if unknown or not fields:
            return {"success": False, "error": f"нечего обновлять: {sorted(unknown) or 'пустой набор полей'}"}
        values = dict(fields)
        if "rating" in values:
            values["rating"] = max(-1, min(5, int(values["rating"])))
        if "tags" in values:
            values["tags"] = json.dumps(list(values["tags"]), ensure_ascii=False)
        sets = ", ".join(f"{name} = ?" for name in values)
        now = datetime.now(timezone.utc).isoformat()
        try:
            with self._lock:
                row = self._conn.execute(
                    f"UPDATE generated_tracks SET {sets}, updated_at = ? WHERE id = ? RETURNING *",
                    [*values.values(), now, track_id]).fetchone()
                self._conn.commit()
        except sqlite3.Error as exc:
            return {"success": False, "error": f"SQLite update failed: {exc}"}
        if not row:
            return {"success": False, "error": f"Трек '{track_id}' не найден"}
        info = row_to_dict(row)
        self._rewrite_json(track_id, info)
        return {"success": True, **info}

    def _rewrite_json(self, track_id: str, info: Dict[str, Any]) -> None:
        """``meta.json`` повторяет строку индекса; не записался — индекс главный, файл догонит при следующей записи."""
        path = self._root / track_id / "meta.json"
        try:
            disk = json.loads(path.read_text(encoding="utf-8")) if path.exists() else {}
            disk.update({k: v for k, v in info.items() if k in disk or k == "rating"})
            path.write_text(json.dumps(disk, ensure_ascii=False, indent=2), encoding="utf-8")
        except (OSError, ValueError):
            pass

    def set_rating(self, track_id: str, rating: int) -> Dict[str, Any]:
        """Оценка трека кодом: ``kn.RATING_LIKE`` / ``kn.RATING_DISLIKE``."""
        return self.update(track_id, rating=rating)

    def last_played(self) -> Optional[Dict[str, Any]]:
        """Трек кэша, игравший последним (по ``last_played_at``), или ``None``."""
        try:
            with self._lock:
                row = self._conn.execute(
                    "SELECT * FROM generated_tracks WHERE last_played_at IS NOT NULL "
                    "ORDER BY last_played_at DESC LIMIT 1").fetchone()
        except sqlite3.Error:
            return None
        return row_to_dict(row) if row else None


__all__ = ["EDITABLE", "TrackMeta", "row_to_dict"]
