"""Тема DJ-сета из СЛОВ ЧЕЛОВЕКА этого хода, а не из истории диалога (ADR-0148, живой лог 06.10 18:04–18:05).

Человек повторил фразу «Ты диджей 8битный и нас сегодня вечеринка любителей денди и классической музыки, замути сэт на
30 минут», а LLM вызвала ``dj_set(theme='Увертюра 1812, мегасет для геймеров: Марио, Аладдин, Тетрис …')`` — тему из
СТАРОЙ реплики истории: классика пропала. Решает код: текст реплики хода едет скрытым аргументом ``heard_text``
(``llm_adapter.TURN_CONTEXT_ARGS``; присланное LLM вырезается). Тема LLM остаётся, если делит со словами реплики
хоть одну значимую основу (:func:`search.content_stems`: перефраз, сокращение, жанр); иначе тема — слова реплики без
служебных («ты диджей X», «замути сэт», длина сета). Нет реплики или в ней нет значимых слов («да», «давай») —
тема LLM как есть.

:func:`heard_theme` — ЕДИНСТВЕННОЕ место, где тема выделяется из свободных слов реплики: путь LLM и путь команды
роутера (``rob_box_voice.core.media_router`` шлёт ``dj_set`` без темы, «у нас сегодня …» грамматика не режет) дают
одну тему (07.10: команда «… и у нас сегодня …» везла тему «… классической музыки замути сэт»). Слова просьбы —
таблица грамматики ``media_command_grammar.SET_REQUEST_WORDS``, имя диджея — её ``dj_persona_span``, стиль с цифрой
(«8битный», #3476) — ``search.blank_style``.
"""

from __future__ import annotations

from typing import List, Optional, Tuple

from rob_box_music import knowledge as kn
from rob_box_music.theme import match_style

from .search import blank_style, content_stems, genre_of

_AND = frozenset({"и", "and"})


def _theme_word(words: List[str], i: int, request: frozenset) -> bool:
    """Слово ``i`` реплики называет тему: не просьба, не стиль, значимое; «музыки» — при слове жанра."""
    word = words[i].replace("ё", "е")
    if genre_of(word):
        return True
    if word in kn.GENRE_FILLER:
        return i > 0 and bool(genre_of(words[i - 1]))  # «классической музыки» — жанр словами человека
    if word in request or match_style([word]) is not None:
        return False
    return word.isdigit() or bool(content_stems(word))  # цифра темы — слово («mambo nr 5»), длину уже вырезали


def heard_theme(text: str) -> str:
    """Тема из реплики человека: слова без просьбы, «ты диджей X», стиля и длины сета; «и» между словами темы и
    запятые остаются (границы частей темы-перечисления, ``search.theme_parts``). Значимых слов нет — ``""``."""
    from rob_box_voice.core.media_command_grammar import SET_REQUEST_WORDS, dj_persona_span
    from rob_box_voice.core.set_length_words import split_set_length, tokens  # длину сета разбирает одна грамматика

    _tracks, rest = split_set_length(text or "")
    persona = dj_persona_span(rest)
    if persona:
        rest = rest[:persona[0]] + " " * (persona[1] - persona[0]) + rest[persona[1]:]
    rest = blank_style(rest)
    toks = tokens(rest)
    words = [t[0] for t in toks]
    keep = [w not in _AND and _theme_word(words, i, SET_REQUEST_WORDS) for i, w in enumerate(words)]
    for i, word in enumerate(words):  # «и» — только между словами темы
        if word in _AND:
            keep[i] = any(keep[:i]) and any(keep[i + 1:])
    out = ""
    last_end = 0
    for (word, start, end), kept in zip(toks, keep):
        if not kept:
            continue
        if out:
            out += (", " if any(c in ",;:" for c in rest[last_end:start]) else " ")
        out += word
        last_end = end
    return out if content_stems(out) else ""


def grounded_theme(theme: Optional[str], heard_text: Optional[str]) -> Tuple[str, bool]:
    """``(тема, заменена ли)``: тема LLM, если она делит значимую основу с репликой хода, иначе тема реплики."""
    theme = theme or ""
    heard = heard_theme(heard_text or "")
    if not heard or content_stems(theme) & content_stems(heard):
        return theme, False
    return heard, True


__all__ = ["grounded_theme", "heard_theme"]
