"""Тема DJ-сета из СЛОВ ЧЕЛОВЕКА этого хода, а не из истории диалога (ADR-0148, живой лог 06.10 18:04–18:05).

Человек повторил фразу «Ты диджей 8битный и нас сегодня вечеринка любителей денди и классической музыки, замути сэт на
30 минут», а LLM вызвала ``dj_set(theme='Увертюра 1812, мегасет для геймеров: Марио, Аладдин, Тетрис …')`` — тему из
СТАРОЙ реплики истории: классика пропала. Решает код: текст реплики хода едет скрытым аргументом ``heard_text``
(``llm_adapter.TURN_CONTEXT_ARGS``; присланное LLM вырезается). Тема LLM остаётся, если делит со словами реплики
хоть одну значимую основу (:func:`search.content_stems`: перефраз, сокращение, жанр); иначе тема — слова реплики без
служебных («ты диджей X», «замути сэт», длина сета). Нет реплики или в ней нет значимых слов («да», «давай») —
тема LLM как есть.
"""

from __future__ import annotations

from typing import List, Optional, Tuple

from .search import content_stems, genre_of

#: Слова просьбы, которых нет в ``knowledge.SEARCH_STOPWORDS`` (там слова темы): глаголы запуска, обращение, длина.
_REQUEST_WORDS = frozenset({
    "ты", "робот", "замути", "замутим", "замутить", "замутишь", "сэт", "сэта", "сэту", "сеты", "сета", "сету",
    "сделай", "сделаешь", "организуй", "устрой", "забабахай", "врубай", "минут", "минуты", "минуту", "час", "часа",
    "часов", "трека", "треков", "будь", "стань", "дай", "дальше", "пусть", "можешь", "сейчас", "ещё", "еще", "мы",
    "вы", "я", "наш", "наша", "наше", "нашу", "нашего", "диджей", "dj", "диджеем", "диджея",
})
_PERSONA_LEADS = frozenset({"ты", "будь", "стань"})
_DJ_WORDS = frozenset({"диджей", "dj", "диджеем"})
_AND = frozenset({"и", "and"})


def _drop(words: List[str], i: int) -> bool:
    """Слово ``i`` реплики не называет тему: просьба, обращение, имя диджея («ты диджей X»), число."""
    word = words[i].replace("ё", "е")
    if genre_of(word):
        return False
    if word in _REQUEST_WORDS or word.isdigit() or word in _AND:
        return word not in _AND
    if i >= 2 and words[i - 1] in _DJ_WORDS and words[i - 2] in _PERSONA_LEADS:
        return True  # «ты диджей Снупдог» — имя диджея, не тема
    return not content_stems(word)


def heard_theme(text: str) -> str:
    """Тема из реплики человека: слова без просьбы, «ты диджей X» и длины сета; «и» между словами темы и запятые
    остаются (границы частей темы-перечисления, ``search.theme_parts``). Значимых слов нет — ``""``."""
    from rob_box_voice.core.set_length_words import split_set_length, tokens  # длину сета разбирает одна грамматика

    _tracks, rest = split_set_length(text or "")
    toks = tokens(rest)
    words = [t[0] for t in toks]
    keep = [not _drop(words, i) for i in range(len(words))]
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
