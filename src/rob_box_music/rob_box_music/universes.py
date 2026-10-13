"""Вселенная темы сета по словам человека (#3581, ADR-0157 §2.1): «анакен скайвокер», «звезде смерте» → ``starwars``.

Живой сет 12.10: «ты диджей анакен скайвокер и у нас сегодня веселье на звезде смерте погнали» — тема «веселье звезде
смерте» без персоны, хуки Celebration/Happy, Star Wars на 8–9 местах. Персонаж, место и предмет одного мира — одна
тема: алиасы — закрытая таблица знания ``knowledge.UNIVERSES``; формы слова — основа + окончание
``knowledge.UNIVERSE_ENDINGS``, опечатка STT — одна правка (``instruments.near``, тот же закрытый словарь, что у
слота инструмента). Без регексов по смыслу (мораторий #3132).
"""

from __future__ import annotations

import functools
import re
from typing import List, Optional, Sequence, Tuple

from . import knowledge as kn
from .instruments import near

_WORD_RE = re.compile(r"\w+")


def words_of(text: str) -> List[Tuple[str, int, int]]:
    """``(слово, начало, конец)`` — нижний регистр, ё → е; позиции — в исходном тексте."""
    low = (text or "").lower().replace("ё", "е")
    return [(m.group(), m.start(), m.end()) for m in _WORD_RE.finditer(low)]


@functools.lru_cache(maxsize=None)
def _forms(stem: str) -> Tuple[str, ...]:
    return tuple(stem + end for end in kn.UNIVERSE_ENDINGS)


def _is(word: str, stem: str) -> bool:
    return near(word, _forms(stem)) is not None


def _aliases() -> List[Tuple[Tuple[str, ...], str]]:
    """``(основы алиаса, ключ вселенной)``, длинные алиасы первыми: «звезда смерти» раньше одиночных слов."""
    pairs = [(alias, key) for key, uni in kn.UNIVERSES.items() for alias in uni.aliases]
    return sorted(pairs, key=lambda p: -len(p[0]))


def universe_spans(words: Sequence[str]) -> List[Tuple[int, int, str]]:
    """``(первое слово, за последним, ключ)`` алиасов вселенных в словах по порядку; алиасы не перекрываются."""
    aliases = _aliases()
    out: List[Tuple[int, int, str]] = []
    i = 0
    while i < len(words):
        hit = next(((len(a), key) for a, key in aliases
                    if all(i + k < len(words) and _is(words[i + k], s) for k, s in enumerate(a))), None)
        if hit:
            out.append((i, i + hit[0], hit[1]))
            i += hit[0]
        else:
            i += 1
    return out


def universe_of(text: Optional[str]) -> Optional[str]:
    """Ключ первой вселенной, названной в тексте («диджей анакен скайвокер» → ``starwars``), или ``None``."""
    spans = universe_spans([w for w, _s, _e in words_of(text or "")])
    return spans[0][2] if spans else None


def is_mood(word: str) -> bool:
    """Слово настроения, а не темы (``knowledge.THEME_MOOD_STEMS``): «веселье», «праздничный»."""
    return word.replace("ё", "е").startswith(kn.THEME_MOOD_STEMS)


__all__ = ["is_mood", "universe_of", "universe_spans", "words_of"]
