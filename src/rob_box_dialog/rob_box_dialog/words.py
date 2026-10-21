"""Сверка текста со словами :mod:`.knowledge` по границам слов (ADR-0150 §4.2, §6.3).

Граница слова здесь — не буква, не цифра и не дефис: «работает» не содержит вейка «бот», а «робота-диджея»
(речь о роботе, не к нему) не содержит вейка «робота». Подстрочного матча нет: «захватит» — не «хватит».
"""

from __future__ import annotations

import functools
import re
from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

from rob_box_dialog.knowledge import DEFAULT_WAKE_WORDS, STOP_LEAD_WORDS, STOP_QUESTION_PARTICLES, STOP_WORDS

_LEFT = r"(?<![\w-])"
_RIGHT = r"(?![\w-])"
_TOKEN = re.compile(r"[\w-]+")

#: Вердикты :func:`stop_verdict` (В19): ясная стоп-команда, спорная (стоп-слово внутри другой команды), нет стопа.
STOP_CLEAR = "clear"
STOP_DISPUTED = "disputed"
STOP_NONE = "none"


@functools.lru_cache(maxsize=64)
def _pattern(words: Tuple[str, ...], tail: str) -> "re.Pattern[str]":
    alternation = "|".join(re.escape(w) for w in words)
    return re.compile(f"{_LEFT}({alternation}){tail}", re.IGNORECASE)


def has_wake_word(text: str, wake_words: Optional[Sequence[str]] = None) -> bool:
    """Есть ли в ``text`` вейк-слово отдельным словом. Пустой список — режим «без вейка»: всё адресовано."""
    words = DEFAULT_WAKE_WORDS if wake_words is None else tuple(wake_words)
    if not words:
        return True
    return bool(_pattern(words, _RIGHT).search(text or ""))


def strip_wake_word(text: str, wake_words: Optional[Sequence[str]] = None) -> str:
    """Убрать ПЕРВОЕ вейк-слово (в любом месте фразы) вместе с пунктуацией и пробелами после него."""
    words = DEFAULT_WAKE_WORDS if wake_words is None else tuple(wake_words)
    if not words:
        return (text or "").strip()
    return _pattern(words, _RIGHT + r"[.,!?\s]*").sub("", text or "", count=1).strip()


def has_word(text: str, stems: Sequence[str]) -> bool:
    """Есть ли слово, НАЧИНАЮЩЕЕСЯ с одной из основ (``SILENCE_WORDS``: «помолч» → «помолчи», «помолчите»)."""
    if not stems:
        return False
    return bool(_pattern(tuple(stems), r"[\w-]*").search(text or ""))


@dataclass(frozen=True)
class StopCall:
    """Разбор стоп-команды (В19): вердикт, стоп-слово и текст ПОСЛЕ него («хватит, спой другую» → «спой другую»).

    ``word``/``rest`` заполнены только у ясного вердикта; у спорного ``word`` — первое стоп-слово фразы.
    """

    verdict: str
    word: str = ""
    rest: str = ""


def stop_verdict(text: str, wake_words: Optional[Sequence[str]] = None) -> str:
    """Стоп-команда ли фраза (В19, Шифу 20.10): :data:`STOP_CLEAR`, :data:`STOP_DISPUTED` или :data:`STOP_NONE`.

    Ясная — первое слово после вейка (ведущие «всё/ну/да…» из ``STOP_LEAD_WORDS`` и сами вейки не в счёт) —
    стоп-слово: «робот, стоп», «всё, хватит, спой другую». Спорная — стоп-слово дальше по фразе: «спой песню про
    стоп», «покажи анимацию стоп»; её решает Chooser (``grammar`` → хост, PR-6), речь до решения не рвётся. Одна
    функция на перебивание речи (stt_node → ``/dialog/interrupt_request``) и на Tier-1 грамматику.

    Команда — то, что ПОСЛЕ вейка: слова перед ним — хвост речи самого робота, который микрофон склеил с фразой
    человека (живой лог 21.10: «старой робот хватит» на сказке «…у корней старой сосны»). Вейк в конце фразы
    («хватит, робот») — команда перед ним.
    """
    return stop_command(text, wake_words).verdict


def stop_command(text: str, wake_words: Optional[Sequence[str]] = None) -> StopCall:
    """То же, что :func:`stop_verdict`, плюс стоп-слово и остаток команды после него (одна реализация разбора)."""
    words = DEFAULT_WAKE_WORDS if wake_words is None else tuple(wake_words)
    part = _command_part(text, words)
    spans = [(m.group(0).lower(), m.end()) for m in _TOKEN.finditer(part)]
    tokens = [t for t, _ in spans]
    skip = set(STOP_LEAD_WORDS) | {w.lower() for w in words}
    lead = next((i for i, t in enumerate(tokens) if t not in skip), None)
    if lead is not None and _clear_at(tokens, lead):
        return StopCall(STOP_CLEAR, tokens[lead], _rest(part, spans, lead, skip))
    first = next((t for t in tokens if _is_stop(t)), "")
    return StopCall(STOP_DISPUTED, first) if first else StopCall(STOP_NONE)


def _rest(part: str, spans: Sequence[Tuple[str, int]], lead: int, skip: set) -> str:
    """Текст после ясного стопа без повторных стоп-слов и вводных («стоп, стоп, ну давай …» → «давай …»)."""
    tail = next((i for i in range(lead + 1, len(spans)) if not (_is_stop(spans[i][0]) or spans[i][0] in skip)), None)
    return part[spans[tail - 1][1]:].strip(" ,.!?;:-–—") if tail is not None else ""


def _clear_at(tokens: Sequence[str], lead: int) -> bool:
    """Ведущее слово — стоп-слово, и за ним не вопросительная частица («хватит ли …»)."""
    after = tokens[lead + 1] if lead + 1 < len(tokens) else ""
    return _is_stop(tokens[lead]) and after not in STOP_QUESTION_PARTICLES


def _command_part(text: str, words: Tuple[str, ...]) -> str:
    """Текст после первого вейка; вейк в конце фразы или без вейка — вся фраза без него."""
    match = _pattern(words, _RIGHT).search(text or "") if words else None
    if match is None:
        return text or ""
    after = (text or "")[match.end():]
    return after if _TOKEN.search(after) else (text or "")[:match.start()]


def _is_stop(token: str) -> bool:
    """Стоп-слово целиком; «стоп-стоп» — тоже (все части через дефис — стоп-слова), «стоп-кран» — нет."""
    return all(part in STOP_WORDS for part in token.split("-")) if token else False
