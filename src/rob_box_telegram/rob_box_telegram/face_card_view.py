"""Текст карточки ``/face`` — из ответа владельца галереи (issue #3655).

Коллаж и сводку бот берёт у ``vision-face`` (``OP_CARD`` сервиса
``/perception/people``). Диск ``/data/faces`` на Sound Pi не читается.
Решения здесь нет: ``ok`` / причина уже решил владелец. Этот модуль только
складывает готовый словарь в подпись или в строку отказа (ADR-0148).
"""

from __future__ import annotations

from typing import Any, Mapping

#: Причина отказа владельца -> строка в чат. Картинку к отказу не прикладываем.
REFUSALS = {
    "exhibition_mode": "В режиме выставки карточку лица не отдаю.",
    "no_snapshot": "У этого лица нет снимка — карточку собрать не из чего.",
    "no_card": "Коллаж карточки собрать не удалось.",
    "person_not_found": "Такого лица уже нет в базе. Список — /faces.",
    "ambiguous_id": "Такой id подходит нескольким лицам. Уточни по /faces.",
    "ambiguous_name": "Такое имя у нескольких лиц. Уточни id по /faces.",
    "bad_id": "Не разобрал, чью карточку просят.",
    "owner_unavailable": "Владелец лиц (Vision Pi) не ответил — карточку получить не удалось.",
}
REFUSAL_DEFAULT = "Карточку лица получить не удалось."


def refusal_text(reason: str, data: Any = None) -> str:
    """Честный отказ из причины владельца. Сводка в ``data`` отказ не смягчает и картинку не заменяет."""
    who = ""
    if isinstance(data, Mapping):
        name = data.get("name")
        short = data.get("id")
        if isinstance(name, str) and name.strip():
            who = name.strip()
        elif isinstance(short, str) and short:
            who = short
    base = REFUSALS.get(reason or "", REFUSAL_DEFAULT)
    if who and reason in ("no_snapshot", "no_card"):
        return f"{base} ({who})"
    return base


def caption_from_owner(data: Any) -> str:
    """Подпись ``reply_photo`` из словаря владельца. Пустой ответ — короткая строка, не выдуманные поля."""
    if not isinstance(data, Mapping):
        return "Карточка лица."
    name = data.get("name")
    shown = name.strip() if isinstance(name, str) and name.strip() else "«незнакомец»"
    shown = shown.replace("`", "'").replace("_", "\\_")
    short = str(data.get("id") or "?")
    encounters = data.get("encounters")
    count = encounters if isinstance(encounters, int) else 0
    last = data.get("last_encounter") or "—"
    epithet = data.get("epithet")
    lines = [f"🧑 *{shown}* (`{short}`)", "", f"встреч: {count}", f"последняя встреча по лицу: {last}"]
    if isinstance(epithet, str) and epithet.strip():
        lines.append(f"кличка: {epithet.strip()}")
    return "\n".join(lines)


__all__ = ["REFUSAL_DEFAULT", "REFUSALS", "caption_from_owner", "refusal_text"]
