"""Длина DJ-сета из слов человека: «сет на 3 трека», «три десятка», «на 20 треков», «на полчаса» → число треков.

Решение Шифу 06.10: сет конечный — 10 треков, а названное человеком число решает КОД, не свободный текст LLM
(ADR-0148): грамматика роутера (:mod:`.media_command_grammar`) вырезает фразу длины из реплики и передаёт число
узким параметром ``dj_set(tracks)``. Время («на полчаса», «на 40 минут», «на час») переводится в треки по средней
длине трека ``set_plan.TRACK_SECONDS``; результат зажат в 1..``set_plan.MAX_TRACKS``.

Разбор — по словам и таблицам, без регексов (мораторий #3132). Модуль чистый: без ROS и I/O.
"""

from __future__ import annotations

from typing import Dict, List, Optional, Sequence, Tuple

from rob_box_music.set_plan import MAX_TRACKS, TRACK_SECONDS

_UNITS: Dict[str, int] = {
    "один": 1, "одного": 1, "одну": 1, "два": 2, "две": 2, "двух": 2, "пару": 2, "пара": 2, "парочку": 2,
    "три": 3, "трёх": 3, "трех": 3, "четыре": 4, "четырёх": 4, "четырех": 4, "пять": 5, "пяти": 5,
    "шесть": 6, "шести": 6, "семь": 7, "семи": 7, "восемь": 8, "восьми": 8, "девять": 9, "девяти": 9,
}
_TEENS: Dict[str, int] = {
    "десять": 10, "десяти": 10, "одиннадцать": 11, "одиннадцати": 11, "двенадцать": 12, "двенадцати": 12,
    "тринадцать": 13, "тринадцати": 13, "четырнадцать": 14, "четырнадцати": 14, "пятнадцать": 15,
    "пятнадцати": 15, "шестнадцать": 16, "шестнадцати": 16, "семнадцать": 17, "семнадцати": 17,
    "восемнадцать": 18, "восемнадцати": 18, "девятнадцать": 19, "девятнадцати": 19,
}
_TENS: Dict[str, int] = {
    "двадцать": 20, "двадцати": 20, "тридцать": 30, "тридцати": 30, "сорок": 40, "сорока": 40,
    "пятьдесят": 50, "пятидесяти": 50, "шестьдесят": 60, "шестидесяти": 60,
}
#: «три десятка», «десяток», «дюжина» — множитель числа перед ним (без числа — один).
_GROUPS: Dict[str, int] = {
    "десяток": 10, "десятка": 10, "десятков": 10, "десятку": 10, "дюжина": 12, "дюжину": 12, "дюжины": 12,
}
_TRACK_WORDS = frozenset({
    "трек", "трека", "треков", "треки", "трэк", "трэка", "трэков", "песен", "песни", "композиций", "композиции",
})
_MINUTE_WORDS = frozenset({"минут", "минуты", "минуту", "минутка", "минутку", "минуток"})
_HOUR_WORDS = frozenset({"час", "часа", "часов", "часик"})
#: Слова перед длиной: «на 3 трека», «из десяти треков» — вырезаются вместе с ней.
_LEADS = frozenset({"на", "из", "в"})

#: Слово реплики: (нижний регистр, начало, конец) в исходном тексте.
Token = Tuple[str, int, int]


def _kind(ch: str) -> str:
    low = ch.lower()
    return "d" if ch in "0123456789" else "a" if "a" <= low <= "z" or "а" <= low <= "я" or low == "ё" else ""


def tokens(text: str) -> List[Token]:
    """Слова и числа реплики с позициями: «на 3трека» → «на», «3», «трека» (цифры — отдельное слово)."""
    out: List[Token] = []
    start = 0
    for i in range(1, len(text) + 1):
        if i == len(text) or _kind(text[i]) != _kind(text[start]):
            if _kind(text[start]):
                out.append((text[start:i].lower(), start, i))
            start = i
    return out


def _number(words: Sequence[str], i: int) -> Tuple[Optional[float], int]:
    """Число со слова ``i``: цифры, «двадцать пять», «полтора»; ``(None, i)`` — не число."""
    word = words[i]
    if word.isdigit():
        return float(word), i + 1
    if word in _TENS:
        unit = _UNITS.get(words[i + 1], 0) if i + 1 < len(words) else 0
        return float(_TENS[word] + unit), i + (2 if unit else 1)
    value = _TEENS.get(word) or _UNITS.get(word) or (1.5 if word in ("полтора", "полторы") else None)
    return (None, i) if value is None else (float(value), i + 1)


def _from_minutes(minutes: float) -> int:
    return max(1, round(minutes * 60 / TRACK_SECONDS))


def _length_at(words: Sequence[str], i: int) -> Tuple[int, int]:
    """``(треков, конец)`` длины, начатой словом ``i``; ``(0, i)`` — длины там нет."""
    word = words[i]
    if word == "полчаса":
        return _from_minutes(30), i + 1
    if word in _HOUR_WORDS and i > 0 and words[i - 1] == "на":  # «сет на час»
        return _from_minutes(60), i + 1
    value, end = _number(words, i)
    if value is None and word in _GROUPS:  # «десяток треков»
        value, end = 1.0, i
    if value is None:
        return 0, i
    unit = words[end] if end < len(words) else ""
    if unit in _GROUPS:
        value, end = value * _GROUPS[unit], end + 1
        unit = words[end] if end < len(words) else ""
        return int(value), end + (1 if unit in _TRACK_WORDS else 0)
    if unit in _TRACK_WORDS:
        return int(value), end + 1
    if unit in _MINUTE_WORDS:
        return _from_minutes(value), end + 1
    if unit in _HOUR_WORDS:
        return _from_minutes(value * 60), end + 1
    return 0, i


def find_set_length(words: Sequence[str]) -> Tuple[int, int, int]:
    """``(треков, начало, конец)`` первой длины сета в словах (с «на»/«из» перед ней); ``(0, 0, 0)`` — числа нет.
    Число зажато в 1..``MAX_TRACKS``: «сет на два часа» — самый длинный сет, а не отказ."""
    for i in range(len(words)):
        tracks, end = _length_at(words, i)
        if tracks:
            start = i - 1 if i > 0 and words[i - 1] in _LEADS else i
            return min(max(1, tracks), MAX_TRACKS), start, end
    return 0, 0, 0


def split_set_length(text: str) -> Tuple[int, str]:
    """``(треков, реплика без фразы длины)``: «включи сет на 3 трека: Марио, Тетрис» → (3, «включи сет: Марио,
    Тетрис»). Числа нет — ``(0, text)``. Знаки препинания вне фразы длины остаются (перечисление темы)."""
    toks = tokens(text)
    tracks, start, end = find_set_length([t[0] for t in toks])
    if not tracks:
        return 0, text
    return tracks, (text[:toks[start][1]].rstrip() + " " + text[toks[end - 1][2]:].lstrip()).strip()


__all__ = ["Token", "find_set_length", "split_set_length", "tokens"]
