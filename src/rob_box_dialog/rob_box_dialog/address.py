"""Адресовано ли роботу — решает код по одной таблице (ADR-0150 §4.2, §7.3 ревизия 5; решения В2, В12, В18).

:func:`decide` — единственное место этого решения: вейк по границам слов из таблицы ``knowledge`` (рискованные
вейки В18 выключены по умолчанию), Telegram и оператор адресованы всегда, кроме обращения к координатору
(«Клод …» из Telegram, #3518). Голое «робот» после неуслышанной просьбы (В12) берёт свежую фразу бэклога того же
направления/человека как текст хода (``pickup``); без неё — только рефлекс «слушаю», 0 вызовов LLM. Окна
адресации без вейка нет (§17 Ж); единственная фраза без вейка, которую робот берёт, — ответ на его собственный
вопрос об имени ≤ 15 с с той же стороны (В17(б), ``speaker_adapter.answers_question``).

Чистый модуль: сообщения ROS читаются утиной типизацией (:func:`heard_from`).
"""

from __future__ import annotations

import re
from dataclasses import dataclass
from typing import Any, List, Optional, Sequence, Tuple, Union

from rob_box_dialog.backlog import Backlog, Heard
from rob_box_dialog.session import SceneFrame
from rob_box_dialog.speaker_adapter import answers_question
from rob_box_dialog.knowledge import (
    ADDRESS_FILLER_WORDS,
    COORDINATOR_NAMES,
    DEFAULT_OPERATOR_WAKE_WORDS,
    DEFAULT_WAKE_WORDS,
    resolve_wake_word_namespaces,
)
from rob_box_dialog.words import has_wake_word, strip_wake_word

#: Источники реплики (значения ``Utterance.SOURCE_*``; тест сверяет с ``.msg``).
SOURCE_RESPEAKER, SOURCE_TELEGRAM, SOURCE_OPERATOR = "respeaker", "telegram", "operator"
#: ``Addressed.source``: откуда текст хода.
VOICE, BACKLOG_PICKUP, BARE_WAKE, IDENTITY_ANSWER = "voice", "backlog_pickup", "bare_wake", "identity_answer"
#: ``NotAddressed.reason``.
NO_WAKE, COORDINATOR, EMPTY = "no_wake", "coordinator", "empty"
#: Запас окна фразы на хвост VAD для направления (тот же, что у stt_node).
VAD_TAIL_S = 1.5

_COORDINATOR_RE = re.compile(r"^\s*(?:" + "|".join(COORDINATOR_NAMES) + r")(?![\w])", re.IGNORECASE)
_WORD = re.compile(r"[\w-]+")


@dataclass(frozen=True)
class Addressed:
    """Реплика адресована: ``text`` — текст хода; ``pickup`` — он взят из бэклога (В12), ``channels`` — почему
    это тот же собеседник (``speaker_adapter.same_interlocutor``)."""

    text: str
    source: str
    pickup: bool = False
    channels: Tuple[str, ...] = ()


@dataclass(frozen=True)
class NotAddressed:
    reason: str


Decision = Union[Addressed, NotAddressed]


def is_coordinator_call(text: str) -> bool:
    """Реплика начинается с обращения к координатору («Клод», «claude», не «Клодия»)."""
    return isinstance(text, str) and _COORDINATOR_RE.match(text) is not None


def is_bare(rest: str) -> bool:
    """После вейка нет слов, кроме заполнителей («робот», «робот, ну?») — фразы нет."""
    return all(w in ADDRESS_FILLER_WORDS for w in _WORD.findall((rest or "").lower()))


def decide(heard: Heard, backlog: Optional[Backlog], wake_words: Sequence[str],
           question: Optional[SceneFrame] = None) -> Decision:
    """Адресована ли реплика роботу и какой у хода текст (§4.2, В12). ``question`` — живой кадр ``Identity``
    (робот спросил имя): фраза без вейка с той же стороны — ответ на него (В17(б))."""
    text = (heard.text or "").strip()
    if heard.source in (SOURCE_TELEGRAM, SOURCE_OPERATOR):
        if heard.source == SOURCE_TELEGRAM and is_coordinator_call(text):
            return NotAddressed(COORDINATOR)
        rest = strip_wake_word(text, wake_words)
        return Addressed(rest, heard.source) if rest else NotAddressed(EMPTY)
    if not has_wake_word(text, wake_words):
        return Addressed(text, IDENTITY_ANSWER) if text and answers_question(question, heard) else NotAddressed(NO_WAKE)
    rest = strip_wake_word(text, wake_words)
    if not is_bare(rest):
        return Addressed(rest, VOICE)
    entry, channels = backlog.pickup(heard) if backlog is not None else (None, ())
    return Addressed(entry.text, BACKLOG_PICKUP, True, channels) if entry else Addressed("", BARE_WAKE)


def _seconds(stamp: Any) -> float:
    return float(getattr(stamp, "sec", 0) or 0) + float(getattr(stamp, "nanosec", 0) or 0) / 1e9


def heard_from(msg: Any, *, now: float, doa: Any = None, person_ref: str = "", faces: Any = None) -> Heard:
    """``Utterance`` → :class:`Heard`: время — конец фразы; направление (DOA) и лица в кадре — по окну её звучания."""
    started, ended = _seconds(getattr(msg, "started_at", None)), _seconds(getattr(msg, "ended_at", None))
    t = ended or now
    window = (started - VAD_TAIL_S, t)
    doa_deg = doa.during(*window) if (doa is not None and started) else None
    seen = faces.during(*window) if (faces is not None and started) else frozenset()
    return Heard(text=(getattr(msg, "text", "") or "").strip(), t=t, source=getattr(msg, "source", "") or "",
                 doa_deg=doa_deg, person_ref=person_ref or "", voice_s=max(0.0, ended - started) if started else 0.0,
                 faces=seen, utterance_id=str(getattr(msg, "utterance_id", "") or ""))


def person_ref_from(registry: Any, utterance_id: Optional[str]) -> str:
    """``speaker_id`` узнанного диктора этой фразы из реестра биометрии (``peek`` по id); нет — ``""``."""
    result = registry.peek(utterance_id) if (registry is not None and utterance_id) else None
    return str(result.get("speaker_id") or "") if isinstance(result, dict) and result.get("is_known") else ""


def log_label(msg: Any) -> str:
    """Текст реплики для лога ``🎤 STT: …``: у Telegram — ``[TG:<id>] текст``.

    Формат — контракт с координатором: он читает «Клод …» из этой строки лога voice-assistant.
    """
    text = (getattr(msg, "text", "") or "").strip()
    chat_id = getattr(msg, "chat_id", 0)
    if text and chat_id and getattr(msg, "source", "") == SOURCE_TELEGRAM:
        return f"[TG:{chat_id}] {text}"
    return text


def wake_words(path: str = "", personality: Sequence[str] = ()) -> Tuple[List[str], List[str]]:
    """Таблица вейков ``(personality, operator)``: из ``wake_words.yaml``, иначе — переданные, иначе — кода."""
    return resolve_wake_word_namespaces(
        path, personality_fallback=list(personality) or list(DEFAULT_WAKE_WORDS),
        operator_fallback=DEFAULT_OPERATOR_WAKE_WORDS)
