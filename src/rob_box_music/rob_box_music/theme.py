"""Тема сета → структурный профиль без LLM: ``seeded_profile(theme_text)`` (ADR-0149 §4.7).

Тема разбирается один раз при старте сета. Слова темы сверяются с закрытой таблицей
``knowledge.THEMES`` по основам (``космический`` ~ ``косм``); совпала строка — темп из её окна, лад
плана и хуки строки. Хуки темы — ещё и мелодии, которые по словам темы нашёл поиск по всей RTTTL-библиотеке
(``found``: ``engine.search.theme_hooks`` в процессе плеера, #3399): «терминатор» → ``terminat``. Ни строки, ни
находок — окно стиля и пул из :data:`HOOK_POOL` по хешу темы (``source="pool"``, а не «тема»): у разных тем
разные хуки, а не одни и те же семь. Темп, тоника и пул — от sha256 текста темы: одна тема даёт один профиль
на любом процессе (``hash()`` Python солёный).
"""

from __future__ import annotations

import hashlib
import random
import re
from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

from . import knowledge as kn

#: Широкий пул хуков темы без находок: все мелодии таблицы тем и общего пула; сету достаётся :data:`POOL_HOOKS`.
HOOK_POOL: Tuple[str, ...] = tuple(dict.fromkeys((*(h for row in kn.THEMES.values() for h in row.hooks),
                                                  *kn.DEFAULT_HOOKS)))
POOL_HOOKS = 7


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

    @property
    def source(self) -> str:
        """``theme`` — хуки по словам темы (A11), ``pool`` — пул по хешу темы."""
        return "theme" if self.theme_hooks else "pool"


def _digest(text: str) -> int:
    return int(hashlib.sha256(text.encode("utf-8")).hexdigest()[:12], 16)


def match_row(theme_text: str) -> Optional[str]:
    """Строка таблицы с наибольшим числом слов темы, начинающихся с её основ; ничья — порядок таблицы."""
    words = re.findall(r"\w+", theme_text.lower())
    best: Tuple[int, Optional[str]] = (0, None)
    for name, row in kn.THEMES.items():
        hits = sum(1 for w in words if any(w.startswith(stem) for stem in row.stems))
        if hits > best[0]:
            best = (hits, name)
    return best[1]


def seeded_profile(theme_text: str, style: str = kn.DEFAULT_STYLE, found: Sequence[str] = ()) -> ThemeProfile:
    """Профиль темы за микросекунды, без сети и LLM; ``found`` — мелодии по словам темы (поиск, лучшие первыми).
    Пустая тема без находок — профиль стиля с пулом по хешу."""
    text = " ".join(theme_text.lower().split())
    window = kn.STYLES[style]
    name = match_row(text)
    digest = _digest(text)
    if name is None:
        lo, hi = window.bpm
        mode = window.modes[digest % len(window.modes)]
        row_hooks: Tuple[str, ...] = ()
    else:
        row = kn.THEMES[name]
        lo, hi = row.bpm
        mode, row_hooks = row.mode, row.hooks
    theme_hooks = tuple(dict.fromkeys((*found, *row_hooks)))
    hooks = theme_hooks or tuple(random.Random(digest).sample(HOOK_POOL, POOL_HOOKS))
    bpm = lo + (digest >> 8) % (hi - lo + 1)
    return ThemeProfile(text, style, bpm, (digest >> 16) % 12, mode, hooks, name, theme_hooks)


__all__ = ["HOOK_POOL", "POOL_HOOKS", "ThemeProfile", "match_row", "seeded_profile"]
