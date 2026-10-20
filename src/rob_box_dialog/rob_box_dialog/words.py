"""Сверка текста со словами :mod:`.knowledge` по границам слов (ADR-0150 §4.2, §6.3).

Граница слова здесь — не буква, не цифра и не дефис: «работает» не содержит вейка «бот», а «робота-диджея»
(речь о роботе, не к нему) не содержит вейка «робота». Подстрочного матча нет: «захватит» — не «хватит».
"""

from __future__ import annotations

import functools
import re
from typing import Optional, Sequence, Tuple

from rob_box_dialog.knowledge import DEFAULT_WAKE_WORDS

_LEFT = r"(?<![\w-])"
_RIGHT = r"(?![\w-])"


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
