"""Квота диска кэша генераций (ADR-0158 §2.5, PR-9): худшее и давно не игранное уходит первым.

Единица вычистки — заказ KIE целиком: оба варианта одного ``task_id`` (оба mp3 и оба ``track.16k.wav``), иначе
остался бы близнец без пары. Трек без ``task_id`` (импорт, MiniMax) — сам себе заказ. Порядок: сперва заказы с
худшей лучшей оценкой (не нравится, потом без оценки, потом нравится), внутри — давнее всего игранный
(не игранный — по ``created_at``). Только что сохранённые (``protect``) не трогаются.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, List, Tuple

from rob_box_music import knowledge as kn

MIB = 1024 * 1024
#: Сколько строк читать для расчёта (на роботе 14.10 их 19).
_ROWS_LIMIT = 100000


@dataclass(frozen=True)
class Order:
    """Заказ в кэше: треки одного ``task_id``, их вес на диске и порядок вычистки."""

    ids: Tuple[str, ...]
    bytes: int
    best_rating: int
    last_touch: str


def dir_bytes(path: Path) -> int:
    """Занято каталогом трека: mp3, wav сервера и ``meta.json``."""
    try:
        return sum(f.stat().st_size for f in path.iterdir() if f.is_file())
    except OSError:
        return 0


def orders(library: Any) -> List[Order]:
    """Заказы кэша с весом на диске, в порядке вычистки (первый уйдёт первым)."""
    rows = library.list_tracks(limit=_ROWS_LIMIT, sort_by="recent").get("tracks", ())
    groups: Dict[str, List[Dict[str, Any]]] = {}
    for row in rows:
        groups.setdefault(str(row.get("task_id") or f"solo:{row['id']}"), []).append(row)
    result = [Order(tuple(str(r["id"]) for r in group),
                    sum(dir_bytes(library.root_dir / str(r["id"])) for r in group),
                    max(int(r.get("rating") or 0) for r in group),
                    max(str(r.get("last_played_at") or r.get("created_at") or "") for r in group))
              for group in groups.values()]
    return sorted(result, key=lambda o: (o.best_rating, o.last_touch))


def used_bytes(library: Any) -> int:
    """Занято кэшем всего (каталоги треков, без индекса)."""
    return sum(o.bytes for o in orders(library))


def enforce_quota(library: Any, max_bytes: int = kn.GEN_LIBRARY_QUOTA_MB * MIB,
                  protect: Iterable[str] = ()) -> Dict[str, Any]:
    """Пока кэш больше квоты — удалять худший заказ (``library.delete_track`` на каждый его трек). Заказ с
    ``protect``-треком пропускается. ``{used, quota, freed, removed: [id…]}``; влезло — ничего не удаляется."""
    keep = set(protect)
    queue = orders(library)
    used = sum(o.bytes for o in queue)
    removed: List[str] = []
    freed = 0
    for order in queue:
        if used - freed <= max_bytes:
            break
        if keep.intersection(order.ids):
            continue
        for track_id in order.ids:
            if library.delete_track(track_id).get("success"):
                removed.append(track_id)
        freed += order.bytes
    return {"used": used, "quota": max_bytes, "freed": freed, "removed": removed}


__all__ = ["MIB", "Order", "dir_bytes", "enforce_quota", "orders", "used_bytes"]
