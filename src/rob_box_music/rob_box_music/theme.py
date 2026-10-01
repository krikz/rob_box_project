"""Тема сета → структурный профиль без LLM: ``seeded_profile(theme_text)`` (ADR-0149 §4.7).

Тема разбирается один раз при старте сета. Слова темы сверяются с закрытой таблицей
``knowledge.THEMES`` по основам (``космический`` ~ ``косм``); совпала строка — темп из её окна, лад
плана и хуки темы из локальной RTTTL-библиотеки. Не совпала — окно жанра и общий пул хуков
``knowledge.DEFAULT_HOOKS`` (``source="pool"``, а не «тема»). Темп и тоника внутри окна — от
sha256 текста темы: одна тема даёт один профиль на любом процессе (``hash()`` Python солёный).
"""

from __future__ import annotations

import hashlib
import re
from dataclasses import dataclass
from typing import Optional, Tuple

from . import knowledge as kn


@dataclass(frozen=True)
class ThemeProfile:
    theme: str
    genre: str
    bpm: int
    root: int  # 0..11
    mode: str  # лад плана; лад трека с хуком берётся у хука (arrange.hook.track_key)
    hook_ids: Tuple[str, ...]
    row: Optional[str]  # ключ ``knowledge.THEMES`` или None — тема не из таблицы

    @property
    def source(self) -> str:
        """``theme`` — материал из таблицы темы (A11), ``pool`` — общий пул."""
        return "theme" if self.row else "pool"


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


def seeded_profile(theme_text: str, genre: str = "club") -> ThemeProfile:
    """Профиль темы за микросекунды, без сети и LLM. Пустая тема — профиль жанра с общим пулом."""
    text = " ".join(theme_text.lower().split())
    window = kn.GENRE_WINDOWS[genre]
    name = match_row(text)
    digest = _digest(text)
    if name is None:
        lo, hi = window.bpm
        mode = window.scales[digest % len(window.scales)]
        hooks = kn.DEFAULT_HOOKS
    else:
        row = kn.THEMES[name]
        lo, hi = row.bpm
        mode, hooks = row.mode, row.hooks
    bpm = lo + (digest >> 8) % (hi - lo + 1)
    return ThemeProfile(text, genre, bpm, (digest >> 16) % 12, mode, hooks, name)


__all__ = ["ThemeProfile", "match_row", "seeded_profile"]
