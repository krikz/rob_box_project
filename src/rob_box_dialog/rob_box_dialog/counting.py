"""Счёт вслух до N строит код, если модель урезала его (#4314 К1, ADR-0148: решает код).

n802 марафона М9: «посчитай вслух от одного до сорока» - модель ответила «Считаю, Саша. Один… Десять.» и молча
урезала счёт до десяти. Сколько чисел просили, код знает из слов человека (:func:`asked_range`); до скольких модель
досчитала - из слов команды (:func:`reached`). Не досчитала - числа строит код (:func:`full_count`): весь счёт
влезает в речь (``SPEECH_MAX_CHARS``) - речью, иначе выступлением ``poem`` по ``COUNT_PER_SEGMENT`` чисел в сегменте
(в пределах схемы ``SEGMENTS_MAX`` × ``SEGMENT_MAX_CHARS``). Дальше ``COUNT_MAX`` код не считает и говорит об этом
сам (``COUNT_CAPPED_PHRASE``). Пределы и глаголы - в :mod:`.knowledge`, числительные - в :mod:`.plan.repeats`.

Модель отказалась считать или переспросила (в ответе меньше двух чисел) - её ответ остаётся: код не считает за неё
то, чего она не начинала. Правка идущего выступления (``edit_pending``) - не новый счёт. Разбор - по словам и
таблицам, без регексов. Модуль чистый: без ROS и I/O.
"""

from __future__ import annotations

from dataclasses import replace
from typing import Dict, List, Optional, Sequence, Tuple

from rob_box_dialog import knowledge as kn
from rob_box_dialog import phrases
from rob_box_dialog.command import Command, Performance
from rob_box_dialog.plan.repeats import NUMERAL_GENITIVE, NUMERAL_SAY, NUMERAL_WORDS, words

_VALUE: Dict[str, int] = {**NUMERAL_WORDS, **NUMERAL_GENITIVE}


def _number_at(tokens: Sequence[str], i: int) -> Tuple[Optional[int], int]:
    """Число, что начинается со слова ``i`` (цифры, слово, составное «двадцать один» / «тридцати пяти»), и сколько
    слов оно заняло; не число - ``(None, 0)``."""
    word = tokens[i] if i < len(tokens) else ""
    if word.isdigit():
        return int(word), 1
    value = _VALUE.get(word)
    if value is None:
        return None, 0
    unit = _VALUE.get(tokens[i + 1]) if i + 1 < len(tokens) else None
    if 20 <= value <= 90 and value % 10 == 0 and unit is not None and 1 <= unit <= 9:
        return value + unit, 2
    return value, 1


def asked_range(text: str) -> Optional[Tuple[int, int]]:
    """Счёт, о котором просит реплика человека: ``(от, до)`` - глагол счёта (``knowledge.COUNT_VERBS``) и «до N»
    («от M» - начало, без него с одного); не просьба о счёте - ``None``."""
    tokens = words(text)
    if not set(tokens) & set(kn.COUNT_VERBS):
        return None
    bounds: Dict[str, int] = {}
    for i, token in enumerate(tokens):
        value, _ = _number_at(tokens, i + 1) if token in ("от", "до") and token not in bounds else (None, 0)
        if value is not None:
            bounds[token] = value
    if "до" not in bounds:
        return None
    return bounds.get("от", 1), bounds["до"]


def said_numbers(text: str) -> Tuple[int, ...]:
    """Числа текста по порядку, составные - одним числом («Двадцать один.» - 21)."""
    tokens = words(text)
    out: List[int] = []
    i = 0
    while i < len(tokens):
        value, used = _number_at(tokens, i)
        if value is not None:
            out.append(value)
        i += used or 1
    return tuple(out)


def reached(cmd: Command) -> Tuple[int, ...]:
    """Числа, что звучат в команде: речь и сегменты выступления."""
    parts = [cmd.speech or "", *(cmd.perform.segments if cmd.perform is not None else ())]
    return tuple(n for part in parts for n in said_numbers(part))


def short_count(cmd: Command, asked: Tuple[int, int]) -> bool:
    """Модель начала счёт (не меньше двух чисел), но до конца просьбы не дошла."""
    nums = reached(cmd)
    if len(nums) < 2:
        return False
    start, end = asked
    return (max(nums) < end) if end >= start else (min(nums) > end)


def say(number: int) -> str:
    """Число 0…100 словами (именительный): «сорок два»."""
    if number in NUMERAL_SAY:
        return NUMERAL_SAY[number]
    return f"{NUMERAL_SAY[number - number % 10]} {NUMERAL_SAY[number % 10]}"


def count_text(numbers: Sequence[int]) -> str:
    """Числа словами, каждое - своим предложением: «Один. Два. Три.»."""
    return " ".join(f"{say(n).capitalize()}." for n in numbers)


def counted(asked: Tuple[int, int]) -> Tuple[Tuple[int, ...], bool]:
    """Числа счёта кодом (вверх или вниз) и урезан ли он до ``knowledge.COUNT_MAX``."""
    start, end = (min(n, kn.COUNT_MAX) for n in asked)
    step = 1 if end >= start else -1
    return tuple(range(start, end + step, step)), max(asked) > kn.COUNT_MAX


def full_count(cmd: Command, text: str) -> Optional[Command]:
    """Команда со счётом, построенным кодом, когда реплика ``text`` просила счёт до N, а модель досчитала меньше
    (:func:`short_count`); иначе ``None`` - команда модели остаётся."""
    asked = asked_range(text)
    if asked is None or cmd.edit_pending or not short_count(cmd, asked):
        return None
    numbers, capped = counted(asked)
    lead = phrases.text(kn.COUNT_CAPPED_PHRASE, top=_genitive(kn.COUNT_MAX)) if capped else ""
    whole = " ".join(part for part in (lead, count_text(numbers)) if part)
    if len(whole) <= kn.SPEECH_MAX_CHARS:
        return replace(cmd, speech=whole, perform=None, ask=None)
    step = kn.COUNT_PER_SEGMENT
    segments = [count_text(numbers[i:i + step]) for i in range(0, len(numbers), step)]
    if lead:
        segments[0] = f"{lead} {segments[0]}"
    return replace(cmd, speech=None, ask=None, perform=Performance(kind="poem", segments=tuple(segments)))


def _genitive(number: int) -> str:
    return next(word for word, value in NUMERAL_GENITIVE.items() if value == number)


__all__ = ["asked_range", "count_text", "counted", "full_count", "reached", "said_numbers", "say", "short_count"]
