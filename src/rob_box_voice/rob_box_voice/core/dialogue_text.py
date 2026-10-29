"""Вейк-слова и команды тишины для старого пути v1 — импорт из одной таблицы знания (ADR-0150 PR-1).

Списки живут в :mod:`rob_box_dialog.knowledge`, сверка по границам слов — в :mod:`rob_box_dialog.words`.
Модуль оставлен ради прежних имён у ``stt_node``, ``dialogue_node``, ``stt_admission`` и тестов; своих списков
и регексов в нём нет. Уходит вместе со старым путём (PR-15).
"""

from __future__ import annotations

from typing import Sequence

from rob_box_dialog.knowledge import (
    DEFAULT_OPERATOR_WAKE_WORDS,
    DEFAULT_WAKE_WORDS,
    SILENCE_WORDS,
    UNSILENCE_WORDS,
    load_wake_word_namespaces,
    resolve_wake_word_namespaces,
)
from rob_box_dialog.words import has_wake_word, has_word, strip_wake_word

#: Прежние имена списков тишины (основы слов, матч по границе слова).
DEFAULT_SILENCE_COMMANDS = SILENCE_WORDS
DEFAULT_UNSILENCE_COMMANDS = UNSILENCE_WORDS


def is_silence_command(text_lower: str, commands: Sequence[str] | None = None) -> bool:
    """Есть ли в фразе команда тишины — слово с основой «помолч»/«замолч» («хватит» — стоп звуков, не тишина, #3700)."""
    return has_word(text_lower, SILENCE_WORDS if commands is None else commands)


def is_unsilence_command(text_lower: str, commands: Sequence[str] | None = None) -> bool:
    """Есть ли в фразе команда снять тишину («говори», «отвечай», …) — по границе слова."""
    return has_word(text_lower, UNSILENCE_WORDS if commands is None else commands)


__all__ = [
    "DEFAULT_WAKE_WORDS",
    "DEFAULT_OPERATOR_WAKE_WORDS",
    "DEFAULT_SILENCE_COMMANDS",
    "DEFAULT_UNSILENCE_COMMANDS",
    "has_wake_word",
    "strip_wake_word",
    "is_silence_command",
    "is_unsilence_command",
    "load_wake_word_namespaces",
    "resolve_wake_word_namespaces",
]
