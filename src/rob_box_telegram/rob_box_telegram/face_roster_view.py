"""Текст ``/faces`` для оператора — из ответа владельца галереи (issue #3655).

Список лиц бот берёт тем же путём, что тул ТАРС ``list_faces``
(:mod:`rob_box_supervisor.face_roster_tool`): сервис ``/perception/people``
``OP_LIST`` на Vision Pi. Диск ``/data/faces`` на Sound Pi — копия-снимок,
её бот больше не читает (ADR-0159 В8, один владелец галереи).

Решения здесь нет: ``ok`` / ``unavailable`` / «база пуста» уже решил владелец
и :func:`listing_from_owner`. Этот модуль только складывает готовый словарь
в текст сообщения (ADR-0148: код решает, текст говорит).
"""

from __future__ import annotations

from typing import Any, Mapping

from rob_box_supervisor.face_roster_tool import listing_from_owner, unavailable_listing

UNAVAILABLE_TEXT = (
    "⚠️ Владелец лиц (Vision Pi) не ответил — список лиц получить не удалось. " "Галерея на этом узле не читается."
)


def format_owner_listing(listing: Mapping[str, Any]) -> str:
    """Текст ``/faces`` из словаря тула: честный отказ, пустая база или таблица."""
    if listing.get("status") != "ok":
        return UNAVAILABLE_TEXT
    faces = listing.get("faces")
    rows = faces if isinstance(faces, list) else []
    if not rows:
        return "Лицевая база пуста."
    total = listing.get("total_in_base")
    head = f"*Лица* ({total if isinstance(total, int) else len(rows)}):"
    lines = [head, ""]
    for row in rows:
        if not isinstance(row, Mapping):
            continue
        name = str(row.get("name") or "незнакомец")
        shown = "«незнакомец»" if name == "незнакомец" else name
        person_id = str(row.get("id") or "?")
        encounters = row.get("encounters")
        ago = row.get("last_seen_ago") or "—"
        count = encounters if isinstance(encounters, int) else 0
        lines.append(f"• {shown} `{person_id}` — встреч {count}, последняя {ago}")
    return "\n".join(lines)


__all__ = [
    "UNAVAILABLE_TEXT",
    "format_owner_listing",
    "listing_from_owner",
    "unavailable_listing",
]
