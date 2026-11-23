"""Вайб лица как длительное состояние (#3780 B): стиль шара → слоты уже существующих черт.

Шар (``rob_box_dialog.vibe.VibeBall``) катится от фактов. Пока он не у нейтрали, лицо показывает
модуляцию слотов DSL — аксессуар, палитру, темп — а не сцену на ``FACE_SCENE_MAX_S``. Радуги-фона
и новых пикселей здесь нет (часть A). Числа кривой живут в ``knowledge``; эта таблица — в пакете
лица, ядро она не меняет.

Одна таблица, одно место. Стиль вне таблицы — не вайб: лицо остаётся обычным (нет выдуманного).
Явная просьба про аксессуар («надень» / «сними») — структурный вход, не разбор текста ответа.
"""

from __future__ import annotations

from typing import Mapping, Optional, Tuple

from rob_box_dialog.vibe import NEUTRAL, Vibe

#: Слоты, которыми вайб модулирует лицо. Фон и разовый импульс — не вайб (часть A / сцена LLM).
VIBE_FACE_SLOTS: Tuple[str, ...] = ("eyes", "tint", "tempo")
#: Аксессуар держится, пока шар у его стиля и не едет домой. Сняли — слот уходит сразу.
ACCESSORY = "eyes"
#: Явная просьба — сильный толчок. Музыка, человек, реплика, похвала — подсказка, не сцена.
ASK = 1.0

#: Стиль шара → слоты лица. Значения — только из ``FACE_SCENE_SLOTS``.
#: Очки — единственный аксессуар слота ``eyes``. Остальное — палитра и темп уже рисуемых черт:
#: разогрев тёплый и быстрый, думает холодный и медленный, сбой холодный и быстрый,
#: пробуждение тёплое, засыпание медленное, праздник — радужная палитра черт (не фон).
STYLE_SLOTS: Mapping[str, Mapping[str, str]] = {
    "shades": {"eyes": "shades"},
    "fire": {"tint": "warm", "tempo": "fast"},
    "matrix": {"tint": "cold", "tempo": "slow"},
    "glitch": {"tint": "cold", "tempo": "fast"},
    "boot": {"tint": "warm"},
    "power_down": {"tempo": "slow"},
    "rainbow": {"tint": "rainbow"},
    "plasma": {"tint": "purple", "tempo": "slow"},
    "rain": {"tint": "blue", "tempo": "slow"},
    "slash": {"tempo": "fast"},
}


def slots_of(style: Optional[str]) -> Mapping[str, str]:
    """Слоты стиля. Чужой стиль и нейтраль — пусто, лицо не выдумывает вайб."""
    return STYLE_SLOTS.get((style or "").strip().lower(), {})


def worn(vibe: Vibe) -> Mapping[str, str]:
    """Что лицо носит сейчас. Нейтраль и путь домой — ничего.

    Стиль на снимке после толчка «neutral» ещё тот, что снимают (``previous`` — он же):
    аксессуар с такого снимка не носится. Пока шар едет к стилю или просто держит его —
    слоты таблицы.
    """
    if vibe.neutral() or vibe.style == NEUTRAL:
        return {}
    if vibe.speed < 0.0 and vibe.previous == vibe.style:
        return {}
    return slots_of(vibe.style)


#: Глаза, которые носят, а не просто открывают. Сейчас в слоте ``eyes`` такой один — очки.
WORN_EYES = frozenset(style for style, slots in STYLE_SLOTS.items() if slots.get(ACCESSORY))


def accessory_of(slots: Mapping[str, str], wearing: bool = False) -> Optional[str]:
    """Аксессуар явной просьбы. ``shades`` — надеть; «open» и пусто — снять, и только если уже надето.

    Обычные глаза (``wide``, ``squint``, ``open`` без очков) и чужой слот — не просьба (``None``):
    шар от этого не толкается, сцена LLM держит их свои 30 с.
    """
    if ACCESSORY not in slots:
        return None
    kind = str(slots.get(ACCESSORY) or "").strip().lower()
    if kind in WORN_EYES:
        return kind
    if wearing and kind in ("", "open"):
        return NEUTRAL
    return None


__all__ = [
    "ACCESSORY",
    "ASK",
    "STYLE_SLOTS",
    "VIBE_FACE_SLOTS",
    "accessory_of",
    "slots_of",
    "worn",
]
