"""Разбор RTTTL (Nokia ringtone) в ноты: ``parse_rtttl`` (ADR-0149 §3.3, §8.1).

Перенесено из ``rob_box_mcp_tools/core/rtttl.py`` без изменений (ADR-0149 PR-3): одна реализация,
старый модуль импортирует ``parse_rtttl`` отсюда.

Формат: ``Name:d=4,o=5,b=140:8c5,8c5,8g5,...``
  * ``d`` — длительность по умолчанию (4 = четверть, 8 = восьмая…);
  * ``o`` — октава по умолчанию (4-7);
  * ``b`` — темп;
  * нота — ``[длительность][a-g/p][#|_][октава][.]``; ``p`` — пауза, ``.`` —
    точка (увеличивает длительность в полтора раза).

Альтерации: ``#`` и ``_`` — оба обозначают диез (в коллекциях PICAXE ``_`` —
историческая замена ``#``; это доказано парой файлов одной песни в двух
тональностях — V1 с ``_`` и V2 с ``#``, ноты совпадают со сдвигом на полутон).
"""

from __future__ import annotations

import re
from collections import Counter
from typing import Any, Dict, List, Optional, Sequence, Tuple

__all__ = ["CONTOUR_NOTES", "THEME_HOOKS", "consensus_order", "contour", "parse_rtttl"]

#: Смещение буквы ноты в полутонах от C.
_NOTE_SEMITONE = {"c": 0, "d": 2, "e": 4, "f": 5, "g": 7, "a": 9, "b": 11}

#: Токен: [длительность][нота][#|_][октава][.], напр. ``8c5``, ``2e.``,
#: ``8g#5``, ``8a_``, ``16p``. Октава и длительность опциональны (берутся из
#: заголовка). Точка может стоять и ДО октавы (диалект PICAXE: ``8c.7``,
#: ``16d#.6``) — оба варианта понимаются.
_TOKEN_RE = re.compile(
    r"^(?P<dur>\d*)(?P<note>[a-gp])(?P<acc>[#_]?)"
    r"(?:(?P<oct>\d+)(?P<dot>\.?)|(?P<dot2>\.)(?P<oct2>\d*))?$",
    re.I,
)


def _to_midi(note: str, acc: str, octave: int) -> int:
    """Буква ноты + октава → MIDI (C4 = 60, как в научной нотации).

    ``acc`` — альтерация: ``#`` или ``_`` → +1 полутон (диез), пустая строка
    → 0. В диалекте PICAXE ``_`` — синоним ``#``.
    """
    shift = 1 if acc in ("#", "_") else 0
    return 12 * (octave + 1) + _NOTE_SEMITONE[note.lower()] + shift


def parse_rtttl(rtttl: str) -> Tuple[str, int, List[Tuple[Optional[int], float]]]:
    """Разобрать RTTTL-строку в ``(имя, bpm, [(midi|None, доли)])``.

    ``None`` вместо midi — пауза (в Renardo это rest в списке нот).
    Длительности — в долях такта (четверть = 1.0), точка = ×1.5.

    Raises:
        ValueError: строка не похожа на RTTTL (нет двух двоеточий или
            токен ноты не распознаётся).
    """
    try:
        name, settings, data = rtttl.strip().split(":", 2)
    except ValueError as exc:
        raise ValueError(
            f"RTTTL должен содержать 'name:d=…,o=…,b=…:ноты', получено {rtttl[:60]!r}"
        ) from exc

    defaults: dict = {}
    for kv in settings.split(","):
        key, _, value = kv.partition("=")
        key = key.strip().lower()
        if key:
            defaults[key] = value.strip()
    default_dur = int(defaults.get("d", "4"))
    default_oct = int(defaults.get("o", "5"))
    bpm = int(defaults.get("b", "120"))

    notes: List[Tuple[Optional[int], float]] = []
    for raw in data.split(","):
        token = raw.strip()
        if not token:
            continue
        # Arduino-вариант «4.f5» — точка сразу после длительности: переносим
        # её в конец токена, чтобы один regex понимал оба диалекта.
        if re.match(r"^\d+\.", token):
            token = token.replace(".", "", 1) + "."
        match = _TOKEN_RE.match(token)
        if not match:
            raise ValueError(f"Не удалось разобрать токен RTTTL: {token!r}")
        dur = int(match.group("dur") or default_dur)
        if dur <= 0:
            raise ValueError(f"Недопустимая длительность {dur} в токене RTTTL: {token!r}")
        note = match.group("note").lower()
        acc = match.group("acc")
        octave = int(match.group("oct") or match.group("oct2") or default_oct)
        dotted = bool(match.group("dot") or match.group("dot2"))
        beats = 4.0 / dur * (1.5 if dotted else 1.0)
        midi = None if note == "p" else _to_midi(note, acc, octave)
        notes.append((midi, beats))
    return name, bpm, notes


def contour(rtttl: str, notes: int) -> Optional[Tuple[int, ...]]:
    """Контур начала мелодии (#3427): интервалы между первыми ``notes`` нотами — без транспозиции, пауз и
    длительностей: у двух версий одной темы в разных тональностях и ритмической записи он один. Нот меньше
    ``notes`` или строка не RTTTL — ``None``."""
    try:
        _name, _bpm, events = parse_rtttl(rtttl)
    except ValueError:
        return None
    pitches = [midi for midi, _beats in events if midi is not None][:notes]
    if len(pitches) < notes:
        return None
    return tuple(b - a for a, b in zip(pitches, pitches[1:]))


#: Сколько найденных по теме мелодий получает профиль сета (кандидаты хука и LLM); у темы-перечисления — на часть.
THEME_HOOKS = 8
#: Нот в контуре начала для «консенсуса версий» (#3427): при 7 версии «Terminator» theme_177/theme_178 совпадают,
#: а повторные ноты «Space Quest» и «Exploration Of Space» уже различаются (при 6 — нет; при 8 расходятся 177/178).
CONTOUR_NOTES = 7


def consensus_order(hits: Sequence[Tuple[float, Dict[str, Any]]], limit: int = THEME_HOOKS) -> List[str]:
    """Первые ``limit`` найденных записей ``(confidence, запись с rtttl)`` — в порядке хуков темы (#3427, «консенсус
    версий»); набор тот же, меняется только порядок.

    Внутри одной доли слов: сначала мелодии, чей контур начала (:func:`rob_box_music.rtttl.contour`,
    :data:`CONTOUR_NOTES` нот) есть ещё хотя бы у одной найденной записи (считаются все ``hits``), — по одной версии
    на контур, затем их повторные версии, затем одиночные; при равенстве — порядок поиска (ближе к названию).
    Узнаваемая тема лежит в архиве в нескольких версиях («Terminator» theme_177/theme_178: d e f e c f), случайный
    рингтон с тем же словом в названии — в одной (terminat)."""
    contours = [contour(str(r.get("rtttl") or ""), CONTOUR_NOTES) for _c, r in hits]
    copies = Counter(c for c in contours if c is not None)
    versions: Counter = Counter()
    keys = []
    for i, ((confidence, _r), shape) in enumerate(zip(hits[:limit], contours)):
        single = shape is None or copies[shape] < 2
        keys.append((-confidence, single, 0 if single else versions[shape], i))
        versions[shape] += 1
    return [hits[i][1]["name"] for *_rank, i in sorted(keys)]
