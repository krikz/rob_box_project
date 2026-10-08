"""Тема сета → структурный профиль без LLM: ``seeded_profile(theme_text)`` (ADR-0149 §4.7).

Тема разбирается один раз при старте сета. Слова темы сверяются с закрытой таблицей
``knowledge.THEMES`` по основам (``космический`` ~ ``косм``); совпала строка — темп из её окна, лад
плана и хуки строки. Хуки темы — ещё и мелодии, которые по словам темы нашёл поиск по всей RTTTL-библиотеке
(``found``: ``engine.search.theme_hooks`` в процессе плеера, #3399): «терминатор» → ``terminat``. Ни строки, ни
находок — окно стиля и весь :data:`HOOK_POOL` (``source="pool"``, а не «тема») в порядке по хешу темы: у разных
тем разный первый хук. Какой хук откроет сет, решает история (``diversity.opening_order``, #3399): пул из семи по хешу
одной фразы давал один и тот же набор и первый хук в каждом сете (popcorn/axelf, 07.10). Темп, тоника и порядок
пула — от sha256 текста темы: одна тема даёт один профиль на любом процессе (``hash()`` Python солёный).
"""

from __future__ import annotations

import hashlib
import random
import re
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

from . import knowledge as kn
from .works import concept_queries

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
    #: материалы партитур, найденные по названию в индексе партитур (``search.ThemeHits.materials``, ADR-0154 §3.5),
    #: лучшие первыми; план раздаёт их первым трекам (``set_plan.plan_materials``) — приоритет над RTTTL-хуками
    materials: Tuple[str, ...] = ()

    @property
    def source(self) -> str:
        """``theme`` — хуки или материал по словам темы (A11), ``pool`` — пул без находок."""
        return "theme" if self.theme_hooks or self.materials else "pool"

    def from_theme(self, hook: Optional[str]) -> bool:
        """Хук трека найден по словам темы (A11): RTTTL темы или материал партитуры."""
        return bool(hook) and (hook in self.theme_hooks or hook in self.materials)


def _digest(text: str) -> int:
    return int(hashlib.sha256(text.encode("utf-8")).hexdigest()[:12], 16)


def match_row(theme_text: str) -> Optional[str]:
    """Строка таблицы с наибольшим числом слов темы, начинающихся с её основ или относящихся к ней через понятие
    (семена-начала реестра, ``works.concept_queries``: «интерстеллар» → ``space``); ничья — порядок таблицы."""
    words = re.findall(r"\w+", theme_text.lower().replace("ё", "е"))
    best: Tuple[int, Optional[str]] = (0, None)
    for name, row in kn.THEMES.items():
        hits = sum(1 for w in words if any(w.startswith(stem) for stem in row.stems) or name in concept_queries(w))
        if hits > best[0]:
            best = (hits, name)
    return best[1]


def match_style(words: Sequence[str]) -> Optional[str]:
    """Стиль по словам фразы (ADR-0153 §4.1, ADR-0148: решает код): первое слово, начинающееся с основы
    ``knowledge.STYLE_WORDS``, или слово стиля с цифрой (``knowledge.STYLE_PATTERNS``: «8-битный» — и одним словом, и
    цифрой отдельно от «битный»); ни одного — ``None`` (вызывающий берёт ``knowledge.DEFAULT_STYLE``)."""
    marks = style_marks(words)
    return next((st for st in marks if st), None)


#: Слова стиля с цифрой (``knowledge.STYLE_PATTERNS``): «8-бит», «8битный», «16 bit» → ключ стиля.
_STYLE_PATTERNS: Tuple[Tuple["re.Pattern[str]", str], ...] = tuple(
    (re.compile(p, re.IGNORECASE), st) for p, st in kn.STYLE_PATTERNS.items())


def _pattern_style(text: str) -> Optional[str]:
    """Стиль, если ``text`` целиком — слово стиля с цифрой («8битный», «8 битный»), иначе ``None``."""
    return next((st for rx, st in _STYLE_PATTERNS if rx.fullmatch(text)), None)


def style_marks(words: Sequence[str]) -> List[Optional[str]]:
    """Стиль каждого слова фразы или ``None``: слово с основой ``knowledge.STYLE_WORDS``, слово стиля с цифрой
    («8битный») и пара «цифра + слово» («8», «битный» — так режут фразу грамматики роутера). Помеченные слова —
    стиль, а не тема сета (#3476): их вырезает тот, кто выделяет тему. Слово стиля из 2–3 слов
    («драм н бейс», «брейк бит» — ADR-0153 S3) помечается целиком."""
    low = [w.lower() for w in words]
    out: List[Optional[str]] = [None] * len(low)
    for i, word in enumerate(low):
        if out[i]:
            continue
        span = next(((n, st) for n in (3, 2) if i + n <= len(low)
                     for st in (_pattern_style(" ".join(low[i:i + n])),) if st), None)
        if span:
            out[i:i + span[0]] = [span[1]] * span[0]
            continue
        out[i] = _pattern_style(word) or next((st for stem, st in kn.STYLE_WORDS.items() if word.startswith(stem)),
                                              None)
    return out


def match_style_text(text: str) -> Optional[str]:
    """Стиль по тексту реплики: слова стиля с цифрой («8-битный») и слова ``knowledge.STYLE_WORDS``; нет — ``None``."""
    text = text or ""
    found = next((st for rx, st in _STYLE_PATTERNS if rx.search(text)), None)
    return found or match_style(re.findall(r"[^\W\d_]+", text))


def match_window_text(style: str, *texts: Optional[str]) -> Optional[str]:
    """Окно стиля ``style`` по словам текстов (ADR-0153 S5, ADR-0148: решает код): первое слово, начинающееся с основы
    ``knowledge.WINDOW_WORDS``, чьё окно есть у стиля («гранж» → ``grunge`` у ``rock``); нет — ``None`` (окно выбирает
    план по сиду, ``set_plan.pick_genre``)."""
    windows = kn.STYLES[style].genre_windows
    for text in texts:
        for word in re.findall(r"[^\W\d_]+", (text or "").lower()):
            found = next((w for stem, w in kn.WINDOW_WORDS.items() if word.startswith(stem) and w in windows), None)
            if found:
                return found
    return None


def style_for(*texts: Optional[str]) -> str:
    """Стиль сета, когда человек не назвал ключ (``dj_set(style=auto)``, ADR-0153 §4.2, ADR-0148): слова стиля в
    текстах по порядку (реплика человека, тема), затем стиль строки таблицы тем (``ThemeRow.style``: киберпанк →
    synthwave, детский праздник → chiptune), иначе ``knowledge.DEFAULT_STYLE``."""
    for text in texts:
        found = match_style_text(text or "")
        if found:
            return found
    for text in texts:
        row = match_row(" ".join((text or "").lower().split())) if text else None
        if row is not None and kn.THEMES[row].style:
            return kn.THEMES[row].style
    return kn.DEFAULT_STYLE


def seeded_profile(theme_text: str, style: str = kn.DEFAULT_STYLE, found: Sequence[str] = (),
                   exact: bool = False, parts: Sequence[Sequence[str]] = (),
                   materials: Sequence[str] = ()) -> ThemeProfile:
    """Профиль темы за микросекунды, без сети и LLM; ``found`` — мелодии по словам темы (поиск, лучшие первыми).
    ``exact`` — тема и есть название найденной записи («Twinkle Twinkle Little Star»): строка таблицы тем («star» →
    ``space``) не применяется — ни её хуки, ни темп и лад (#3427). Пустая тема без находок — профиль стиля с пулом
    по хешу. ``parts`` — находки ``found`` по частям темы-перечисления (франшизы в порядке названного). ``materials`` — партитуры по названию
    (ADR-0154 §3.5): строку таблицы тем не отменяют — темп, лад и строка («интерстеллар» → ``space``, #3463) остаются."""
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
                        tuple(tuple(p) for p in parts if p), tuple(dict.fromkeys(materials)))


__all__ = ["HOOK_POOL", "POOL_HOOKS", "ThemeProfile", "match_row", "match_style", "match_style_text", "match_window_text",
           "seeded_profile",
           "style_for", "style_marks"]
