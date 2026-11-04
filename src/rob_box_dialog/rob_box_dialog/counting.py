"""Счёт вслух до N строит код, если модель урезала его (#4314 К1, ADR-0148: решает код).

n802 марафона М9: «посчитай вслух от одного до сорока» - модель ответила «Считаю, Саша. Один… Десять.» и молча
урезала счёт до десяти. Сколько чисел просили, код знает из слов человека (:func:`asked_range`); до скольких модель
досчитала - из слов команды (:func:`reached`). Не досчитала - числа строит код (:func:`full_count`): весь счёт
влезает в речь (``SPEECH_MAX_CHARS``) - речью, иначе выступлением ``poem`` по ``COUNT_PER_SEGMENT`` чисел в сегменте
(в пределах схемы ``SEGMENTS_MAX`` × ``SEGMENT_MAX_CHARS``). Дальше ``COUNT_MAX`` код не считает и говорит об этом
сам (``COUNT_CAPPED_PHRASE``). Пределы и глаголы - в :mod:`.knowledge`, числительные - в :mod:`.plan.repeats`.

Модель отказалась считать или переспросила (в ответе меньше двух чисел) - её ответ остаётся: код не считает за неё
то, чего она не начинала. Правка идущего выступления (``edit_pending``) - не новый счёт.

Счёт речью - ряд элементов: число и слова его формы («Четыре шаг»), :func:`series`. Код пишет его по предложению на
число (:func:`sentenced`), чтобы владелец речи резал реплику на границе числа. Правка звучащего счёта («и после
каждого числа добавляй слово шаг», #4207) - правка формы элемента: от модели код берёт только форму, а числа - те, что
ещё не прозвучали по ``spoken_chars`` владельца речи (:func:`continued`). Разбор - по словам и таблицам, без регексов.
Модуль чистый: без ROS и I/O.
"""

from __future__ import annotations

from collections import Counter
from dataclasses import replace
from typing import Dict, List, NamedTuple, Optional, Sequence, Tuple

from rob_box_dialog import knowledge as kn
from rob_box_dialog import phrases
from rob_box_dialog.command import Command, Performance
from rob_box_dialog.plan.repeats import NUMERAL_GENITIVE, NUMERAL_SAY, NUMERAL_WORDS, SENTENCE_END, words

_VALUE: Dict[str, int] = {**NUMERAL_WORDS, **NUMERAL_GENITIVE}
#: Слова границ просьбы о счёте: «от M», «до N».
_BOUNDS = ("от", "до")
#: Знаки конца части фразы: элемент счёта - число и слова его формы до такого знака («Один шаг, два шага.»).
_CLAUSE_ENDS = SENTENCE_END + ",;:"


class Item(NamedTuple):
    """Элемент счёта: число, слова его формы после числа («шаг») и конец его части фразы в тексте (символ)."""

    number: int
    form: Tuple[str, ...]
    end: int


class Series(NamedTuple):
    """Счёт в реплике: текст до первого числа, элементы по порядку и текст после последнего."""

    lead: str
    items: Tuple[Item, ...]
    trailer: str


def _number_at(tokens: Sequence[str], i: int) -> Tuple[Optional[int], int]:
    """Число, что начинается со слова ``i`` (цифры, слово, составное «двадцать один» / «тридцати пяти»), и сколько
    слов оно заняло; не число - ``(None, 0)``. Составное - только в одном падеже: «до сорока. Один» - это 40 и 1, а не
    41 (#4320)."""
    word = tokens[i] if i < len(tokens) else ""
    if word.isdigit():
        return int(word), 1
    value = _VALUE.get(word)
    if value is None:
        return None, 0
    after = tokens[i + 1] if i + 1 < len(tokens) else ""
    unit = _VALUE.get(after) if (word in NUMERAL_GENITIVE) == (after in NUMERAL_GENITIVE) else None
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
        value, _ = _number_at(tokens, i + 1) if token in _BOUNDS and token not in bounds else (None, 0)
        if value is not None:
            bounds[token] = value
    if "до" not in bounds:
        return None
    return bounds.get("от", 1), bounds["до"]


def said_numbers(text: str, bounds: bool = True) -> Tuple[int, ...]:
    """Числа текста по порядку, составные - одним числом («Двадцать один.» - 21). ``bounds=False`` - без границ
    просьбы («до сорока», «от пяти»): это пересказ просьбы, а не счёт."""
    tokens = words(text)
    out: List[int] = []
    i = 0
    while i < len(tokens):
        value, used = _number_at(tokens, i)
        if value is not None and (bounds or not i or tokens[i - 1] not in _BOUNDS):
            out.append(value)
        i += used or 1
    return tuple(out)


def reached(cmd: Command) -> Tuple[int, ...]:
    """Числа, что звучат в команде: речь и сегменты выступления. Граница просьбы в словах модели - не досчитанное
    число (#4320: «Считаю до сорока. Один, …» читалось как 41, 2, … - счёт «полный», и код его не достраивал)."""
    parts = [cmd.speech or "", *(cmd.perform.segments if cmd.perform is not None else ())]
    return tuple(n for part in parts for n in said_numbers(part, bounds=False))


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


def _clauses(text: str) -> List[Tuple[int, int]]:
    """Части фразы ``(начало, конец)`` между знаками :data:`_CLAUSE_ENDS`."""
    out, start = [], 0
    for i, ch in enumerate(text + _CLAUSE_ENDS[0]):
        if ch in _CLAUSE_ENDS:
            out.append((start, i))
            start = i + 1
    return out


def _clause_items(clause: str, end: int) -> List[Item]:
    """Элементы одной части фразы: каждое число (не граница «до N») и слова после него до следующего числа."""
    tokens, out, i = words(clause), [], 0
    while i < len(tokens):
        value, used = _number_at(tokens, i)
        if value is not None and not (i and tokens[i - 1] in _BOUNDS):
            out.append(Item(value, (), end))
        elif value is None and out:
            out[-1] = out[-1]._replace(form=out[-1].form + (tokens[i],))
        i += used or 1
    return out


def series(text: str) -> Series:
    """Счёт реплики по элементам (:class:`Item`): «Считаю. Один шаг, два шага. Пришли.» - lead «Считаю.», (1, шаг),
    (2, шага), trailer «Пришли.». Слова части фразы перед её первым числом в форму не идут."""
    items: List[Item] = []
    first = last = 0
    for start, end in _clauses(text):
        found = _clause_items(text[start:end], end)
        first = first if items or not found else start
        last = end if found else last
        items += found
    if not items:
        return Series(text.strip(), (), "")
    return Series(text[:first].strip(), tuple(items), text[last:].lstrip(_CLAUSE_ENDS + " ").strip())


def _counts(items: Sequence[Item]) -> bool:
    """Ряд счёта: не меньше двух чисел подряд с шагом 1 (вверх или вниз), все - в пределах ``COUNT_MAX``."""
    steps = {b.number - a.number for a, b in zip(items, items[1:])}
    return len(items) >= 2 and len(steps) == 1 and steps <= {1, -1} and max(i.number for i in items) <= kn.COUNT_MAX


def is_count(text: str) -> bool:
    """Реплика - счёт (:func:`_counts`)."""
    return _counts(series(text).items)


def _sentence(number: int, form: Sequence[str]) -> str:
    return " ".join((say(number).capitalize(), *form)) + "."


def sentenced(cmd: Command, text: str) -> Optional[Command]:
    """Счёт речью, о котором просила реплика ``text``, - по предложению на число («Один, два» → «Один. Два.»): так
    владелец речи режет его на границе числа (``boundary=sentence``), а прогресс ``spoken_chars`` говорит, какое число
    прозвучало (#4207). Текст до и после счёта остаётся; не счёт с начала просьбы («Раз, два, три» - «раз» не число
    таблиц) или уже так - ``None``."""
    asked = asked_range(text)
    if asked is None or not cmd.speech or cmd.edit_pending:
        return None
    lead, items, trailer = series(cmd.speech)
    if not _counts(items) or items[0].number != asked[0]:
        return None
    speech = " ".join(part for part in (lead, *(_sentence(i.number, i.form) for i in items), trailer) if part)
    return None if speech == cmd.speech else replace(cmd, speech=speech)


def continued(sounding: str, spoken_chars: int, edit: str) -> str:
    """Продолжение звучащего счёта ``sounding`` после правки ``edit`` («Один шаг, два шага…»): числа, что не
    прозвучали (часть фразы элемента кончилась не дальше ``spoken_chars``), в форме из правки - у числа своя, иначе
    самая частая форма правки. Сказанное не повторяется, счёт не начинается заново; всё прозвучало - «»."""
    rest = [i.number for i in series(sounding).items if i.end > spoken_chars]
    told = series(edit)
    forms = {i.number: i.form for i in told.items}
    common = Counter(i.form for i in told.items).most_common(1)
    fallback = common[0][0] if common else ()
    if not rest:
        return ""
    return " ".join([*(_sentence(n, forms.get(n, fallback)) for n in rest), *([told.trailer] if told.trailer else [])])


def _genitive(number: int) -> str:
    return next(word for word, value in NUMERAL_GENITIVE.items() if value == number)


__all__ = ["Item", "Series", "asked_range", "continued", "count_text", "counted", "full_count", "is_count", "reached",
           "said_numbers", "say", "sentenced", "series", "short_count"]
