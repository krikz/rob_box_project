"""Адресовано ли роботу — решает код по одной таблице (ADR-0150 §4.2, §7.3 ревизия 5; решения В2, В12, В18).

:func:`decide` — единственное место этого решения: вейк по границам слов из таблицы ``knowledge`` (рискованные
вейки В18 выключены по умолчанию), Telegram и оператор адресованы всегда, кроме обращения к координатору
(«Клод …» из Telegram, #3518). Голое «робот» после неуслышанной просьбы (В12) берёт все свежие фразы бэклога
(и куски того же рассказа) как текст хода (``pickup``, #3767 — от кого бы они ни были, кто сказал — метка каждой
фразы); бэклог пуст — только рефлекс «слушаю», 0 вызовов LLM. Окна
адресации без вейка нет (§17 Ж); фразы без вейка, которые робот берёт, — ответ на его собственный вопрос
об имени ≤ 15 с, если она по грамматике похожа на ответ — имя, да/нет, отказ (В17(б), решение Шифу 21.10,
``speaker_adapter.answer_to_question``; DOA не условие), ответ приветствованному в окне 15 с после
приветствия при встрече (#3776, ``speaker_adapter.answer_to_greeting``) и «да»/«нет» в окне 15 с после вопроса
выступления «Сейчас играет …, выключить и зачитать?» (О9′, #3700).

Чистый модуль: сообщения ROS читаются утиной типизацией (:func:`heard_from`).
"""

from __future__ import annotations

import re
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Sequence, Tuple, Union

from rob_box_dialog.backlog import Backlog, Heard
from rob_box_dialog.reply_grammar import NO, YES, classify_reply
from rob_box_dialog.session import SceneFrame
from rob_box_dialog.speaker_adapter import answer_to_greeting, answer_to_question
from rob_box_dialog.knowledge import (
    ADDRESS_FILLER_WORDS,
    COORDINATOR_NAMES,
    DEFAULT_OPERATOR_WAKE_WORDS,
    DEFAULT_WAKE_WORDS,
    REPLY_WINDOW_S,
    resolve_wake_word_namespaces,
)
from rob_box_dialog.words import has_wake_word, strip_wake_word

#: Источники реплики (значения ``Utterance.SOURCE_*``; тест сверяет с ``.msg``).
SOURCE_RESPEAKER, SOURCE_TELEGRAM, SOURCE_OPERATOR = "respeaker", "telegram", "operator"
#: ``Addressed.source``: откуда текст хода.
VOICE, BACKLOG_PICKUP, BARE_WAKE, IDENTITY_ANSWER = "voice", "backlog_pickup", "bare_wake", "identity_answer"
#: Фраза без вейка в окне после приветствия при встрече (#3776).
GREETING_REPLY = "greeting_reply"
#: «Да»/«нет» без вейка в окне вопроса выступления о смене музыки (О9′, #3700).
PERFORM_ANSWER = "perform_answer"
#: ``NotAddressed.reason``.
NO_WAKE, COORDINATOR, EMPTY = "no_wake", "coordinator", "empty"
#: Запас окна фразы на хвост VAD для направления (тот же, что у stt_node).
VAD_TAIL_S = 1.5

_COORDINATOR_RE = re.compile(r"^\s*(?:" + "|".join(COORDINATOR_NAMES) + r")(?![\w])", re.IGNORECASE)
_WORD = re.compile(r"[\w-]+")


@dataclass(frozen=True)
class Addressed:
    """Реплика адресована: ``text`` — текст хода; ``pickup`` — он взят из бэклога (В12), ``picked`` — подхваченные
    фразы с метками (``Backlog.label``: кто сказал, когда, откуда) по порядку."""

    text: str
    source: str
    pickup: bool = False
    picked: Tuple[Dict[str, Any], ...] = ()


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


def _yes_no_in_window(frame: Optional[SceneFrame], text: str, t: float) -> bool:
    """Окно вопроса да/нет открыто (``asked_at`` — после речи и грейса микрофона) и фраза — «да» или «нет»."""
    asked_at = frame.payload.get("asked_at") if frame is not None else None
    if asked_at is None or not 0.0 <= float(t or 0.0) - float(asked_at) <= REPLY_WINDOW_S:
        return False
    reply = classify_reply(text or "", asks_name=False)
    return reply is not None and reply.kind in (YES, NO)


def _wakeless_reply(text: str, heard: Heard, question: Optional[SceneFrame],
                    greeting: Optional[SceneFrame], ask: Optional[SceneFrame] = None) -> Decision:
    """Фраза без вейка: ответ на вопрос об имени (В17(б)), ответ на приветствие (#3776), «да»/«нет» на вопрос
    выступления (О9′) или не адресована."""
    if text and answer_to_question(question, heard) is not None:
        return Addressed(text, IDENTITY_ANSWER)
    if text and answer_to_greeting(greeting, heard):
        return Addressed(text, GREETING_REPLY)
    if text and _yes_no_in_window(ask, text, heard.t):
        return Addressed(text, PERFORM_ANSWER)
    return NotAddressed(NO_WAKE)


def decide(heard: Heard, backlog: Optional[Backlog], wake_words: Sequence[str],
           question: Optional[SceneFrame] = None, greeting: Optional[SceneFrame] = None,
           ask: Optional[SceneFrame] = None) -> Decision:
    """Адресована ли реплика роботу и какой у хода текст (§4.2, В12). ``question`` — живой кадр ``Identity``
    (робот спросил имя), `greeting` — окно после приветствия (#3776), ``ask`` — окно вопроса выступления
    ``PerformAsk`` (О9′): фраза без вейка, похожая на ответ, — ход."""
    text = (heard.text or "").strip()
    if heard.source in (SOURCE_TELEGRAM, SOURCE_OPERATOR):
        if heard.source == SOURCE_TELEGRAM and is_coordinator_call(text):
            return NotAddressed(COORDINATOR)
        rest = strip_wake_word(text, wake_words)
        return Addressed(rest, heard.source) if rest else NotAddressed(EMPTY)
    if not has_wake_word(text, wake_words):
        return _wakeless_reply(text, heard, question, greeting, ask)
    rest = strip_wake_word(text, wake_words)
    if is_bare(rest) and _yes_no_in_window(ask, rest, heard.t):  # «Робот, да» — ответ на вопрос, не голый вейк
        return Addressed(rest, PERFORM_ANSWER)
    if not is_bare(rest):
        return Addressed(rest, VOICE)
    taken = backlog.pickup(heard) if backlog is not None else []
    if not taken:
        return Addressed("", BARE_WAKE)
    return Addressed(" ".join(e.text for e in taken), BACKLOG_PICKUP, True,
                     tuple(Backlog.label(e, heard) for e in taken))


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
