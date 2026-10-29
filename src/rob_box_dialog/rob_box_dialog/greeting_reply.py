"""Похожа ли фраза на ответ на приветствие робота — по таблице ``knowledge.GREETING_REPLIES`` (ADR-0148)."""

from __future__ import annotations

import re

from rob_box_dialog import knowledge as kn
from rob_box_dialog.words import strip_wake_word

_TOKEN = re.compile(r"[\w-]+")


def is_greeting_reply(text: str) -> bool:
    """Фраза — ответ на приветствие («привет», «и тебе», «как дела», «спасибо»): начинается с фразы таблицы и коротка
    (длинный рассказ с «привет» в начале — не ответ)."""
    words = [w.lower().replace("ё", "е") for w in _TOKEN.findall(strip_wake_word(text or ""))]
    if not 0 < len(words) <= kn.GREETING_REPLY_MAX_WORDS:
        return False
    joined = " ".join(words)
    return any(joined == p or joined.startswith(p + " ") for p in kn.GREETING_REPLIES)
