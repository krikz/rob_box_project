"""RTTTL (Nokia ringtone) → Renardo code.

RTTTL — машинный формат мелодий ``длительность+нота+октава``, по которому в
интернете лежат готовые точные ноты (Nokia ringtones, BLHeli-тона переводимы
в него). Парсер конвертирует их в Renardo ``midinote + dur`` — детерминированно,
без ручного переписывания нот (корень ошибок #1810).

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
from typing import List, Optional, Tuple

__all__ = ["parse_rtttl", "rtttl_to_renardo"]

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


def rtttl_to_renardo(
    rtttl: str,
    synth: str = "pianovel",
    amp: float = 0.5,
) -> str:
    """Собрать Renardo-код (готовый для ``execute_music_code``) из RTTTL.

    Паузы рендерятся как ``None`` в списке ``midinote`` — Renardo играет
    ``None`` как rest. Код начинается с ``Clock.clear()``, чтобы стереть
    предыдущий паттерн.
    """
    _name, bpm, notes = parse_rtttl(rtttl)
    midi = [("None" if m is None else str(m)) for m, _ in notes]
    durs = [f"{d:g}" for _, d in notes]
    return (
        "Clock.clear()\n"
        f"Clock.bpm = {bpm}\n"
        f"p1 >> {synth}(midinote=[{', '.join(midi)}], "
        f"dur=[{', '.join(durs)}], amp={amp})"
    )
