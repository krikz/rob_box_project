"""Рэп в такт паузами MiniMax по сетке строк (ADR-0150 §5.6.3, решение О11 товарища Шифу 23.10, #3709 → #3901).

Слепое A/B спайка PR-S1 (``docs/dialog/rap_sync_spike_2026-10-23.md``): паузы ``<#x#>`` MiniMax между строками
куплета победили текущий код; остальные подходы — проиграли. Попадание меряется на слух, а не метрикой C16.

Строка куплета должна начинаться с такта подложки: строка занимает ``n`` тактов (наименьшее ``n`` ≤
``MAX_BARS_PER_LINE``, в которое она влезает с запасом ``BAR_FILL``), пауза после неё — ``n·такт − длина строки −
PAUSE_OVERHEAD_S``. Последняя строка сегмента — без паузы: следующий сегмент ставит на такт :mod:`.sync`.

Единственное место знания о паузах: формула, лимиты, формат метки, кто её понимает (``PAUSE_PROVIDERS``) и как её
снять для остальных (:func:`text_for` — tts_node на фолбеке Yandex/Silero). Скрипт спайка
(``scripts/dialog/rap_sync/approaches.py``) импортирует отсюда же. Чистый Python, без часов и ROS.
"""

from __future__ import annotations

import logging
import re
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

_LOG = logging.getLogger(__name__)

#: Провайдеры TTS, которые понимают метку паузы ``<#x#>`` (MiniMax T2A v2, platform.minimax.io/docs/api-reference/
#: speech-t2a-http). У Yandex (``sil<[мс]>``) и Silero (``<break>``) — своя разметка; им метка снимается.
PAUSE_PROVIDERS: Tuple[str, ...] = ("minimax",)
#: Строка занимает не больше этой доли своих тактов: запас на разброс темпа речи (спайк PR-S1, ``markup_text``).
BAR_FILL = 0.92
#: Строка длиннее ``BAR_FILL·MAX_BARS_PER_LINE`` тактов всё равно получает столько тактов — дальше не растягиваем.
MAX_BARS_PER_LINE = 2
#: Пауза MiniMax: документированный диапазон 0.01–99.99 с; короче ``MIN_PAUSE_S`` — не слышно, ставим минимум.
MIN_PAUSE_S = 0.05
MAX_PAUSE_S = 99.99
#: Темп речи MiniMax в рэп-строке, слогов (гласных) в секунду: медиана спайка PR-S1 по 48 строкам × 3 голоса
#: (male-qn-qingse 4.77, Russian_ReliableMan 5.03, Russian_BrightHeroine 5.08; p10 3.91, p90 6.01) — от первой до
#: последней буквы строки по меткам ``subtitle_type=word``. Меток синтеза у выступления нет — длина строки по нему.
SYLLABLES_PER_S = 4.9
#: Пауза MiniMax звучит дольше заказанной: шаг строки первого прохода спайка вышел длиннее расчёта на медиану
#: 360 мс (36 шагов, 3 голоса; естественная пауза на стыке строк). Вычитаем её — калибровка «4×2» одной
#: константой: второй синтез с поправкой по меткам у робота невозможен (меток нет) и в спайке не сошёлся.
PAUSE_OVERHEAD_S = 0.36
#: Гласные — слоги русского текста.
VOWELS = frozenset("аеёиоуыэюяАЕЁИОУЫЭЮЯ")

_PAUSE_RE = re.compile(r"([,.;:!?…]?)\s*<#\d+(?:\.\d+)?#>\s*")
_LINE_END = ",.;:!?"
#: Сегмент без переводов строк — строки по знакам препинания (запятая, конец предложения).
_PHRASE_SPLIT_RE = re.compile(r"(?<=[,.;:!?…])\s+")


@dataclass(frozen=True)
class Markup:
    """Текст сегмента с паузами, сколько тактов на строку и сами паузы (с)."""

    text: str
    bars_per_line: int
    pauses: Tuple[float, ...]


def syllables(text: str) -> int:
    """Слогов в русском тексте = гласных."""
    return sum(ch in VOWELS for ch in text)


def pause_tag(seconds: float) -> str:
    """Метка паузы MiniMax в пределах ``MIN_PAUSE_S``–``MAX_PAUSE_S``."""
    return f"<#{min(MAX_PAUSE_S, max(MIN_PAUSE_S, seconds)):.2f}#>"


def split_lines(text: str) -> List[str]:
    """Строки сегмента: по переводам строк; их нет — по знакам препинания (одна фраза — одна строка)."""
    lines = [ln.strip() for ln in text.split("\n") if ln.strip()]
    if len(lines) > 1:
        return lines
    return [p.strip() for p in _PHRASE_SPLIT_RE.split(text.strip()) if p.strip()]


def estimate_span(line: str) -> float:
    """Сколько звучит строка MiniMax (с) — по слогам (``SYLLABLES_PER_S``); без гласных — как один слог."""
    return max(1, syllables(line)) / SYLLABLES_PER_S


def layout(spans: Sequence[float], bar_s: float, max_speed: float = 1.0, fix: Sequence[float] = (),
           overhead_s: float = PAUSE_OVERHEAD_S) -> Tuple[List[float], float, int]:
    """Паузы после каждой строки, кроме последней, скорость и тактов на строку: каждая строка — с такта.

    ``spans`` — длины строк (с) при скорости 1; строка влезает в ``n`` тактов со скоростью ≤ ``max_speed``
    (у выступления скорости нет — 1); ``fix`` — поправка пауз по замеру прошлого синтеза (спайк, второй проход)."""
    longest = max(spans)
    n = next((k for k in range(1, MAX_BARS_PER_LINE + 1) if longest / max_speed <= BAR_FILL * k * bar_s),
             MAX_BARS_PER_LINE)
    speed = round(min(max_speed, max(1.0, longest / (BAR_FILL * n * bar_s))), 2)
    pauses = []
    for j, dur in enumerate(spans[:-1]):
        corr = fix[j] if j < len(fix) else 0.0
        pauses.append(n * bar_s - dur / speed - corr - overhead_s)
    return pauses, speed, n


def mark(lines: Sequence[str], pauses: Sequence[float]) -> str:
    """Строки через пробел, после каждой (кроме последней) — пауза; знак в конце строки уступает паузе."""
    parts = [f"{ln.rstrip(_LINE_END)} {pause_tag(p)}" for ln, p in zip(lines, pauses)]
    return " ".join([*parts, *lines[len(pauses):]])


def markup(text: str, bar_s: float) -> Optional[Markup]:
    """Сегмент выступления → текст с паузами MiniMax по сетке тактов ``bar_s``; одна строка — ``None`` (нечего
    расставлять, сегмент целиком встаёт на такт)."""
    lines = split_lines(text)
    if len(lines) < 2 or bar_s <= 0:
        return None
    pauses, _, n = layout([estimate_span(ln) for ln in lines], bar_s)
    return Markup(mark(lines, pauses), n, tuple(round(min(MAX_PAUSE_S, max(MIN_PAUSE_S, p)), 2) for p in pauses))


def strip_pauses(text: str) -> str:
    """Снять метки пауз: на их месте — знак конца строки (свой или запятая), чтобы фолбек не читал «решётку»."""
    return _PAUSE_RE.sub(lambda m: (m.group(1) or ",") + " ", text).strip()


def text_for(provider: str, text: str) -> str:
    """Текст для провайдера ``provider``: понимает паузы — как есть; нет — метки сняты (честно в логе)."""
    if provider in PAUSE_PROVIDERS or "<#" not in text:
        return text
    _LOG.info(f"разметка пауз снята: синтез {provider} не понимает <#x#>")
    return strip_pauses(text)


__all__ = ["BAR_FILL", "MAX_BARS_PER_LINE", "MAX_PAUSE_S", "MIN_PAUSE_S", "Markup", "PAUSE_OVERHEAD_S",
           "PAUSE_PROVIDERS", "SYLLABLES_PER_S", "VOWELS", "estimate_span", "layout", "mark", "markup", "pause_tag",
           "split_lines", "strip_pauses", "syllables", "text_for"]
