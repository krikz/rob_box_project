"""Эксперимент: чойсер вайба лица по короткому контексту (Шифу 17.11, #3780).

Не боевой путь. Боевой путь — код по событиям (шар, инерция, решение Шифу 05.11).
Здесь только проверка, что выбор по куску контекста осмыслен: код собирает промпт
и закрытый список имён, которые уже есть в :data:`.knowledge.FACE_SCENE_SLOTS`
(``bg`` и ``fx``) и рисуются в ``face/effects.py``. Новых эффектов нет.

Выбор делает переданная функция. Сеть и LLM этот модуль не вызывает. Ответ не из
списка код не применяет: наружу уходит ``neutral``.
"""

from __future__ import annotations

from typing import Callable, Mapping, Optional, Sequence, Tuple

from . import knowledge as kn

#: Нет значимого вайба. Не слот рендера и не имя эффекта.
NEUTRAL = "neutral"
#: Слоты, чьи значения — вайб полотна: фон под чертами и разовый импульс.
VIBE_SLOTS: Tuple[str, ...] = ("bg", "fx")
#: Что сказать выбирающей функции. Варианты дописывает :func:`prompt`.
PROMPT = (
    "По короткому контексту выбери один вайб лица из списка. "
    "Если ничего значимого нет — neutral. Ответь одним именем из списка."
)


def slot_names(slots: Optional[Mapping[str, Sequence[str]]] = None) -> Tuple[str, ...]:
    """Имена слотов ``bg`` и ``fx``, кроме ``none``. Порядок — как в таблице знания."""
    table = slots if slots is not None else kn.FACE_SCENE_SLOTS
    names = []
    for slot in VIBE_SLOTS:
        for name in table.get(slot, ()):
            if name and name != "none" and name not in names:
                names.append(name)
    return tuple(names)


def prompt(context: str, names: Sequence[str]) -> str:
    """Промпт выбора: кусок контекста и закрытый список. Пустой контекст — пустая строка контекста."""
    listed = ", ".join([*names, NEUTRAL])
    return f"{PROMPT}\nКонтекст: {context or ''}\nВарианты: {listed}"


def accept(answer: object, names: Sequence[str]) -> str:
    """Имя из списка, иначе ``neutral``. Регистр и крайние пробелы не считаются новым эффектом."""
    if not isinstance(answer, str):
        return NEUTRAL
    name = answer.strip().lower()
    allowed = {item.lower() for item in names}
    if name in allowed and name != NEUTRAL:
        return name
    return NEUTRAL


def _ask(picker: Callable[[str], object], text: str) -> object:
    """Сырой ответ выбирающей функции. Сбой функции — не выбор."""
    try:
        return picker(text)
    except Exception:
        return None


def choose(context: str, names: Sequence[str], picker: Callable[[str], object]) -> str:
    """Один вайб из ``names`` или ``neutral``.

    ``picker`` получает промпт и возвращает сырой ответ. Пустой список, пустой контекст
    и ответ вне списка не падают и не применяются.
    """
    allowed = tuple(dict.fromkeys(name for name in names if name and name != NEUTRAL))
    if not allowed:
        return NEUTRAL
    return accept(_ask(picker, prompt(context or "", allowed)), allowed)


__all__ = ["NEUTRAL", "PROMPT", "VIBE_SLOTS", "accept", "choose", "prompt", "slot_names"]
