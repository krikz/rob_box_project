"""Счётчик вопроса об имени (ADR-0150 §7.5, A29): решает код, не промпт.

«Как тебя зовут?» / «это ты?» — не больше одного на собеседника за сессию и ноль в ходе с действием.
Число 1 — из ADR. Когда спросить, этот модуль не планирует.
"""

from __future__ import annotations

import logging
import re
from dataclasses import replace
from typing import Any, Mapping, MutableMapping, Optional

from rob_box_dialog.command import Ask, Command

#: A29 / §7.5: не чаще одного раза на ``person_ref|track_ref`` за сессию.
MAX_PER_INTERLOCUTOR = 1
#: Собеседник без ``person_ref`` и ``track_ref`` — один ключ на сессию.
_UNKNOWN = ""
#: Те же маркеры, что приёмка A29 и аудит ``scripts/dialog/turn_metrics.py``.
_QUESTION = re.compile(r"как тебя зовут|это ты\?", re.IGNORECASE)
_SENTENCE = re.compile(r"[^.!?…]+[.!?…]*")
_LOG = logging.getLogger(__name__)


def interlocutor_key(speaker: Any) -> str:
    """Ключ счётчика: ``person_ref``, иначе ``track_ref``, иначе пустой (незнакомец без трека — один на сессию)."""
    if speaker is None:
        return _UNKNOWN
    return str(getattr(speaker, "person_ref", None) or getattr(speaker, "track_ref", None) or _UNKNOWN)


def in_text(text: Optional[str]) -> bool:
    """В ``text`` есть «как тебя зовут» или «это ты?»."""
    return bool(text and _QUESTION.search(text))


def asks(cmd: Command) -> bool:
    """Команда задаёт вопрос об имени: ``ask.expects=name`` или маркеры A29 в ``speech`` / ``ask.text``."""
    ask = cmd.ask
    if ask is not None and (ask.expects == "name" or in_text(ask.text)):
        return True
    return in_text(cmd.speech)


def allowed(cmd: Command, asked: Mapping[str, int], speaker: Any) -> bool:
    """Можно отдать вопрос: в ходе нет действий и лимит собеседника ещё не исчерпан."""
    if cmd.actions:
        return False
    return asked.get(interlocutor_key(speaker), 0) < MAX_PER_INTERLOCUTOR


def without_question(text: Optional[str]) -> Optional[str]:
    """``text`` без предложений с вопросом об имени; ничего не осталось — ``None``."""
    if not text:
        return text
    kept = "".join(sentence for sentence in _SENTENCE.findall(text) if not _QUESTION.search(sentence)).strip()
    return kept or None


def stripped(cmd: Command) -> Command:
    """Команда без вопроса об имени; остальные поля не тронуты."""
    speech = without_question(cmd.speech)
    ask = cmd.ask
    if ask is not None and (ask.expects == "name" or in_text(ask.text)):
        text = without_question(ask.text)
        ask = None if ask.expects == "name" or not text else Ask(ask.expects, text, ask.key)
    return replace(cmd, speech=speech, ask=ask)


def limited(cmd: Command, asked: Mapping[str, int], speaker: Any) -> Command:
    """Лишний вопрос об имени снят (действие в ходе или лимит); разрешённый — как был, счётчик не трогает."""
    if not asks(cmd) or allowed(cmd, asked, speaker):
        return cmd
    why = "действие" if cmd.actions else "уже спрашивали"
    _LOG.warning("identity_ask: вопрос об имени снят (%s)", why)
    return stripped(cmd)


def note(cmd: Command, asked: MutableMapping[str, int], speaker: Any) -> None:
    """Засчитать отданный вопрос об имени. Вызывать после :func:`limited`, когда вопрос ещё в команде."""
    if not asks(cmd) or cmd.actions:
        return
    key = interlocutor_key(speaker)
    asked[key] = asked.get(key, 0) + 1


__all__ = [
    "MAX_PER_INTERLOCUTOR",
    "allowed",
    "asks",
    "in_text",
    "interlocutor_key",
    "limited",
    "note",
    "stripped",
    "without_question",
]
