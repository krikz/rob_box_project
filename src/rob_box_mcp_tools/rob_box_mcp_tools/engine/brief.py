"""Подробный промт диджею: просьба и спецификация — разные части (#3845, ADR-0148).

Шифу 22.10 прислал в Telegram промт на 3 тысячи знаков: первая строка просит сет («Ты диджей Пауль Оакенфольд и у нас
вечеринка на тему Очень странные дела»), дальше — роль, BPM, тональность, ноты RTTTL, таблица MIDI. ``dj_set`` брал темой
ВСЕ слова промта (2154 знака), а «83/4 … 124.5 BPM» читал как длину сета (3 трека, 240 с).

Решает структура, а не регекс по ответу LLM: :func:`split_brief` делит реплику человека на ``head`` (просьба: первый
абзац до пустой строки / строки разметки, не длиннее :data:`HEAD_MAX_CHARS`) и ``spec`` (остальное). Тема, длина, стиль и
громкость берутся из ``head`` — как у обычной короткой просьбы. Из ``spec`` берётся то, что движок умеет: стили
(``set_genres``). Чего движок не умеет (темп, тональность, ноты/MIDI — сет играет по теме и стилю, темп решает трек), то
:func:`unsupported` называет, а :func:`unsupported_note` говорит об этом человеку фразой кода (без молчаливой подмены).
"""

from __future__ import annotations

import re
from dataclasses import dataclass
from typing import List, Optional, Tuple

#: Просьба не длиннее: обычная фраза («Ты диджей X и у нас вечеринка … замути сэт на 30 минут») — ~150 знаков.
HEAD_MAX_CHARS = 300
#: Строка с этим началом — разметка спецификации (заголовок, таблица, комментарий кода, список, ограда кода): просьба
#: кончилась. Первую строку реплики это не касается («[TG] Ты диджей …» — просьба).
_STRUCTURE_STARTS = ("#", "|", ";", "`", "- ", "* ", "> ", "---", "===")
_SENTENCE_END = (".", "!", "?", "…")

_BPM = re.compile(r"(?:\bbpm\b\W{0,3}(?P<a>\d+(?:[.,]\d+)?))|(?:(?P<b>\d+(?:[.,]\d+)?)\s*\bbpm\b)", re.IGNORECASE)
_TONALITY = re.compile(r"\b(?:тональность|key|major|minor|phrygian|dorian|lydian|мажор\w*|минор\w*)\b", re.IGNORECASE)
_NOTATION = re.compile(r"(?:\brtttl\b|\bmidi\b|^\s*[\w ]+:d=\d+,o=\d+,b=\d+:|^\s*\|.*\|\s*$)", re.IGNORECASE | re.MULTILINE)


@dataclass(frozen=True)
class Brief:
    """Реплика человека: ``head`` — просьба, ``spec`` — спецификация (пусто у обычной короткой просьбы)."""

    head: str
    spec: str = ""


def _is_structure(line: str) -> bool:
    return line.lstrip().startswith(_STRUCTURE_STARTS)


def _clip(text: str) -> Tuple[str, str]:
    """``(начало, остаток)``: текст длиннее :data:`HEAD_MAX_CHARS` режется по последнему концу предложения в пределе,
    иначе по границе слова."""
    if len(text) <= HEAD_MAX_CHARS:
        return text, ""
    window = text[:HEAD_MAX_CHARS]
    ends = [window.rfind(mark) for mark in _SENTENCE_END]
    cut = max(ends) + 1 if max(ends) > 0 else (window.rfind(" ") if " " in window else HEAD_MAX_CHARS)
    return text[:cut].strip(), text[cut:].strip()


def split_brief(text: Optional[str]) -> Brief:
    """Просьба и спецификация реплики: просьба — первая строка и следующие за ней до пустой строки или строки разметки
    (склеены пробелом), но не длиннее :data:`HEAD_MAX_CHARS`; остальное — ``spec``. Короткая реплика в одну строку —
    ``Brief(text)`` без изменений (поведение коротких просьб то же, что было)."""
    lines = (text or "").strip().splitlines()
    if not lines:
        return Brief("")
    head: List[str] = [lines[0].strip()]
    used = 1
    for line in lines[1:]:
        if not line.strip() or _is_structure(line):
            break
        head.append(line.strip())
        used += 1
    spec = "\n".join(lines[used:]).strip()
    first, over = _clip(" ".join(head))
    return Brief(first, "\n".join(p for p in (over, spec) if p))


def spec_bpm(spec: str) -> Optional[float]:
    """Темп из спецификации: число перед словом BPM («~124.5 BPM»; «BPM: 83/4 = ~124.5 BPM» - итог после «=»), иначе
    число после него («BPM: 124»); нет — ``None``."""
    found = list(_BPM.finditer(spec or ""))
    before = next((m.group("b") for m in found if m.group("b")), None)
    value = before or next((m.group("a") for m in found if m.group("a")), None)
    return float(value.replace(",", ".")) if value else None


def unsupported(spec: str) -> Tuple[str, ...]:
    """Что просит спецификация, чего движок не умеет задать: ``темп N BPM``, ``тональность``, ``ноты и MIDI``."""
    out: List[str] = []
    bpm = spec_bpm(spec)
    if bpm is not None:
        out.append(f"темп {bpm:g} BPM")
    if _TONALITY.search(spec or ""):
        out.append("тональность")
    if _NOTATION.search(spec or ""):
        out.append("ноты и MIDI")
    return tuple(out)


def unsupported_note(items: Tuple[str, ...]) -> str:
    """Фраза кода о том, что из промта не сыграет; нечего — ``""``."""
    if not items:
        return ""
    return f"Из промта не использую: {', '.join(items)} — сет играет по теме и стилю."


__all__ = ["HEAD_MAX_CHARS", "Brief", "spec_bpm", "split_brief", "unsupported", "unsupported_note"]
