"""Тема сета → структурный профиль без LLM: ``seeded_profile(theme_text)`` (ADR-0149 §4.7).

Тема разбирается один раз при старте сета. Слова темы сверяются с закрытой таблицей
``knowledge.THEMES`` по основам (``космический`` ~ ``косм``); совпала строка — темп из её окна, лад
плана и хуки строки. Хуки темы — ещё и мелодии, которые по словам темы нашёл поиск по всей RTTTL-библиотеке
(``found``: ``engine.search.theme_hooks`` в процессе плеера, #3399): «терминатор» → ``terminat``. Ни строки, ни
находок — окно стиля и весь :data:`HOOK_POOL` (``source="pool"``, а не «тема») в порядке по хешу темы: у разных
тем разный первый хук. Какой хук откроет сет, решает история (``compose.opening_order``, #3399): пул из семи по хешу
одной фразы давал один и тот же набор и первый хук в каждом сете (popcorn/axelf, 07.10). Темп, тоника и порядок
пула — от sha256 текста темы: одна тема даёт один профиль на любом процессе (``hash()`` Python солёный).
"""

from __future__ import annotations

import hashlib
import random
import re
from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

from . import knowledge as kn

#: Мелодии строк ``pooled=False`` (праздник, дети): в пул чужих тем не идут, даже из общего пула.
_OCCASION_HOOKS = frozenset(h for row in kn.THEMES.values() if not row.pooled for h in row.hooks)
#: Широкий пул хуков темы без находок: мелодии строк ``pooled`` и общего пула без праздничных; сету достаётся
#: весь (:data:`POOL_HOOKS`) — свежие из него по истории идут первыми (#3399).
HOOK_POOL: Tuple[str, ...] = tuple(h for h in dict.fromkeys((
    *(h for row in kn.THEMES.values() if row.pooled for h in row.hooks), *kn.DEFAULT_HOOKS))
    if h not in _OCCASION_HOOKS)
POOL_HOOKS = len(HOOK_POOL)


@dataclass(frozen=True)
class ThemeProfile:
    theme: str
    style: str  # ключ ``knowledge.STYLES`` (ADR-0153)
    bpm: int
    root: int  # 0..11
    mode: str  # лад плана; лад трека с хуком берётся у хука (arrange.hook.track_key)
    hook_ids: Tuple[str, ...]
    row: Optional[str]  # ключ ``knowledge.THEMES`` или None — тема не из таблицы
    theme_hooks: Tuple[str, ...] = ()  # хуки, найденные по словам темы (строка таблицы, поиск) — A11 считает их
    #: хуки темы-перечисления по частям в порядке названного (``search.ThemeHits.parts``): трек N берёт часть
    #: (N-1) по кругу, в ней — наименее недавнюю версию (``compose.part_order``); пусто — тема одна
    theme_parts: Tuple[Tuple[str, ...], ...] = ()

    @property
    def source(self) -> str:
        """``theme`` — хуки по словам темы (A11), ``pool`` — пул без находок."""
        return "theme" if self.theme_hooks else "pool"


def _digest(text: str) -> int:
    return int(hashlib.sha256(text.encode("utf-8")).hexdigest()[:12], 16)


def _concept_rows(word: str) -> Tuple[str, ...]:
    """Слова запросов ``knowledge.THEME_CONCEPTS``, чей ключ начинает слово: «интерстеллар» → ``("space",)``."""
    return tuple(q for key, query in kn.THEME_CONCEPTS.items() if word.startswith(key) for q in query.split())


def match_row(theme_text: str) -> Optional[str]:
    """Строка таблицы с наибольшим числом слов темы, начинающихся с её основ или относящихся к ней через понятие
    (``knowledge.THEME_CONCEPTS``); ничья — порядок таблицы."""
    words = re.findall(r"\w+", theme_text.lower().replace("ё", "е"))
    best: Tuple[int, Optional[str]] = (0, None)
    for name, row in kn.THEMES.items():
        hits = sum(1 for w in words if any(w.startswith(stem) for stem in row.stems) or name in _concept_rows(w))
        if hits > best[0]:
            best = (hits, name)
    return best[1]


def match_style(words: Sequence[str]) -> Optional[str]:
    """Стиль по словам фразы (ADR-0153 §4.1, ADR-0148: решает код): первое слово, начинающееся с основы
    ``knowledge.STYLE_WORDS``; ни одного — ``None`` (вызывающий берёт ``knowledge.DEFAULT_STYLE``)."""
    for word in words:
        low = word.lower()
        style = next((st for stem, st in kn.STYLE_WORDS.items() if low.startswith(stem)), None)
        if style is not None:
            return style
    return None


def seeded_profile(theme_text: str, style: str = kn.DEFAULT_STYLE, found: Sequence[str] = (),
                   exact: bool = False, parts: Sequence[Sequence[str]] = ()) -> ThemeProfile:
    """Профиль темы за микросекунды, без сети и LLM; ``found`` — мелодии по словам темы (поиск, лучшие первыми).
    ``exact`` — тема и есть название найденной записи («Twinkle Twinkle Little Star»): строка таблицы тем («star» →
    ``space``) не применяется — ни её хуки, ни темп и лад (#3427). Пустая тема без находок — профиль стиля с пулом
    по хешу. ``parts`` — находки ``found`` по частям темы-перечисления (франшизы в порядке названного)."""
    text = " ".join(theme_text.lower().split())
    window = kn.STYLES[style]
    name = None if exact and found else match_row(text)
    digest = _digest(text)
    if name is None:
        lo, hi = window.bpm
        mode = window.modes[digest % len(window.modes)]
        row_hooks: Tuple[str, ...] = ()
    else:
        row = kn.THEMES[name]
        lo, hi = row.bpm
        # лад строки — если он есть у стиля (у клуба есть все), иначе лад стиля по хешу (рейв: минор/фригийский)
        mode = row.mode if row.mode in window.modes else window.modes[digest % len(window.modes)]
        row_hooks = row.hooks
    theme_hooks = tuple(dict.fromkeys((*found, *row_hooks)))
    hooks = theme_hooks or tuple(random.Random(digest).sample(HOOK_POOL, POOL_HOOKS))
    bpm = lo + (digest >> 8) % (hi - lo + 1)
    return ThemeProfile(text, style, bpm, (digest >> 16) % 12, mode, hooks, name, theme_hooks,
                        tuple(tuple(p) for p in parts if p))


__all__ = ["HOOK_POOL", "POOL_HOOKS", "ThemeProfile", "match_row", "match_style", "seeded_profile"]
