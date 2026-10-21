"""Похожа ли фраза на ОТВЕТ на вопрос робота — по грамматике, кодом (#3695, решение Шифу 21.10; ADR-0148).

:func:`classify_reply` — единственное место решения «фраза без вейка в открытом окне ответа — ответ или чужой
разговор». Виды: ``name`` («меня зовут X», «я X», «зови меня X», голое имя из 1–2 слов), ``yes`` / ``no`` («да», «это
я», «не я»), ``refusal`` («не скажу», «секрет», «потом»). :func:`classify_same` — ответ на вопрос склейки после
регистрации («это я» / «мы разные», #2828). Похоже на ответ — фраза берётся как ход; нет — бэклог.
Слова — из :mod:`.knowledge`; DOA и голос здесь не участвуют (DOA в комнате прыгает, решение Шифу).
"""

from __future__ import annotations

import re
from dataclasses import dataclass
from typing import List, Optional

from rob_box_dialog import knowledge as kn
from rob_box_dialog.words import strip_wake_word

NAME, YES, NO, REFUSAL = "name", "yes", "no", "refusal"
_TOKEN = re.compile(r"[\w-]+")
#: Голое имя из 1–2 слов; «Имя Фамилия» берётся по первому слову.
_BARE_MAX_WORDS = 2


@dataclass(frozen=True)
class Reply:
    """Ответ на вопрос: ``kind`` ∈ {name, yes, no, refusal}; ``name`` — имя (с заглавной) для ``name``."""

    kind: str
    name: str = ""


def _norm(word: str) -> str:
    return word.lower().replace("ё", "е").strip("-")


def _words(text: str) -> List[str]:
    return [w for w in (_norm(t) for t in _TOKEN.findall(text)) if w]


def _display(word: str) -> str:
    return word[:1].upper() + word[1:]


def _refusal(words: List[str]) -> bool:
    if not 0 < len(words) <= kn.REPLY_REFUSAL_MAX_WORDS:
        return False
    joined = " ".join(words)
    return any(joined == phrase or joined.startswith(phrase + " ") for phrase in kn.REPLY_REFUSALS)


def _introduced(words: List[str]) -> Optional[str]:
    """«меня зовут X» (X — любое слово), «я X» (X — имя из словаря) в любом месте фразы."""
    for phrase in kn.REPLY_INTRO_PHRASES:
        head = phrase.split()
        for i in range(len(words) - len(head)):
            if words[i:i + len(head)] == head and len(words[i + len(head)]) >= 2:
                return words[i + len(head)]
    for i in range(len(words) - 1):
        if words[i] == "я" and words[i + 1] in kn.KNOWN_FIRST_NAMES and (i == 0 or words[i - 1] != "не"):
            return words[i + 1]
    return None


def _bare_name(words: List[str]) -> Optional[str]:
    """Голое имя: 1–2 слова, не служебные и не глаголы; словарное имя — всегда."""
    if not 0 < len(words) <= _BARE_MAX_WORDS or not all(w.replace("-", "").isalpha() for w in words):
        return None
    known = words[0] in kn.KNOWN_FIRST_NAMES
    rest = words[1:] if known else words  # словарное имя — всегда; остальные слова (фамилия) — не глаголы и не служебные
    if any(w in kn.REPLY_NOT_A_NAME or len(w) < 3 or w.endswith(kn.REPLY_VERB_ENDINGS) for w in rest):
        return None
    return words[0]


def classify_reply(text: str, *, asks_name: bool = True) -> Optional[Reply]:
    """Вид ответа во фразе или ``None`` (не похоже на ответ). ``asks_name`` — вопрос «как тебя зовут?» (иначе голое имя
    ответом не считается: на «Саша, это ты?» отвечают да/нет)."""
    words = _words(strip_wake_word(text or ""))
    if not words:
        return None
    joined = " ".join(words)
    if _refusal(words):
        return Reply(REFUSAL)
    if joined in kn.REPLY_NO:
        return Reply(NO)
    if joined in kn.REPLY_YES:
        return Reply(YES)
    named = _introduced(words) or (_bare_name(words) if asks_name else None)
    return Reply(NAME, _display(named)) if named else None


def classify_same(text: str, *, max_words: Optional[int] = kn.REPLY_SAME_MAX_WORDS) -> Optional[bool]:
    """Ответ на «вы разные люди или это ты?» (#2828): ``True`` — один человек, ``False`` — разные, ``None`` — не понять
    (оба сигнала сразу — «не тот самый» — тоже ``None``: склеить потом можно, разлепить нечем). Фраза длиннее
    ``max_words`` — не ответ (без вейка чужой разговор с «это я» внутри); ``None`` — без предела (фраза адресована)."""
    words = _words(strip_wake_word(text or ""))
    if not words or (max_words is not None and len(words) > max_words):
        return None
    padded = f" {' '.join(words)} "
    different = bool(set(words) & kn.REPLY_DIFFERENT_WORDS) or any(f" {p} " in padded for p in kn.REPLY_DIFFERENT_PHRASES)
    same = " ".join(words) in kn.REPLY_SAME_WHOLE or any(f" {p} " in padded for p in kn.REPLY_SAME_PHRASES)
    return same if different != same else None


__all__ = ["NAME", "NO", "REFUSAL", "Reply", "YES", "classify_reply", "classify_same"]
