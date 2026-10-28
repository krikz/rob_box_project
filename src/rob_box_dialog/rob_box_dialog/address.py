"""Адресовано ли роботу — решает код по одной таблице (ADR-0150 §4.2, §7.3 ревизия 5; решения В2, В12, В18).

:func:`decide` — единственное место этого решения: вейк по границам слов из таблицы ``knowledge`` (рискованные
вейки В18 выключены по умолчанию; искажённое STT первое слово — ``words.slipped_wake_rest``, #3975), Telegram
и оператор адресованы всегда, кроме обращения к координатору
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
from dataclasses import dataclass, replace
from typing import Any, Dict, List, Optional, Sequence, Tuple, Union

from rob_box_dialog.backlog import Backlog, Heard
from rob_box_dialog.heard import repair
from rob_box_dialog.reply_grammar import NO, YES, classify_reply
from rob_box_dialog.session import SceneFrame
from rob_box_dialog.speaker_adapter import answer_to_greeting, answer_to_question, attributed_person, in_reply_window
from rob_box_dialog.knowledge import (
    ADDRESS_FILLER_WORDS,
    COORDINATOR_NAMES,
    DEFAULT_OPERATOR_WAKE_WORDS,
    DEFAULT_WAKE_WORDS,
    resolve_wake_word_namespaces,
)
from rob_box_dialog.words import has_wake_word, slipped_wake_rest, strip_wake_word, wake_command

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
#: Оператор держит личность (``OperatorHold``, ТАРС работает): голос не слушаем и в бэклог не копим (§6.3).
HELD = "operator_hold"
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
    #: Адресованный ход с фразой (не подхват): ≤ 1 свежая фоновая фраза того же собеседника (ADR-0150 §7.3, #3894).
    recent: Tuple[Dict[str, Any], ...] = ()
    #: Адресованный ход с фразой: все фоновые фразы всех авторов за последние ``BACKGROUND_WINDOW_S`` (В23, 23.10, #3892).
    background: Tuple[Dict[str, Any], ...] = ()
    #: Запасной разбор Tier-1 (``turn.tier1_of``): слова после вейка (``words.wake_command``), если они короче ``text``.
    #: Tier-1 разбирает сначала ``text`` целиком (команда до вейка — «Стоп, Робот, пожалуйста»), а это — только когда
    #: целое не Tier-1 или спорно: перед вейком эхо речи робота (n811 27.10: «четыре недели робот хватит читать…» —
    #: спорный стоп). Модель видит ``text`` целиком. «» — команда и есть ``text``.
    command: str = ""
    #: Починки STT по закрытой таблице (``heard.repair``, #4017): «что было» → «что стало» для строки лога хода;
    #: ``text`` и ``command`` уже починены.
    stt_fixes: Tuple[str, ...] = ()
    #: id фразы STT (``Utterance.utterance_id``, тот же хеш звука, что у SpeakerResult), из которой взят текст хода: по
    #: нему ход берёт голос ЭТОЙ фразы (ответ на «ты меня узнал?», #4141). У подхвата бэклога и голого вейка - «»: текст
    #: хода не из этой фразы.
    utterance_id: str = ""


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
    """Окно вопроса да/нет открыто и фраза — «да» или «нет». Окно — с конца вопроса, грейс микрофона включён
    (``in_grace``, #3849): целое «да»/«нет» эхом «Сейчас играет …, выключить и зачитать?» не бывает, а кадр
    открыт только после ``SpeechEvent finished`` вопроса."""
    if frame is None or not in_reply_window(frame.payload.get("asked_at"), t, in_grace=True):
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


def _held(heard: Heard, hold: Optional[SceneFrame]) -> bool:
    """Оператор держит личность, а реплика — голос с микрофона (Telegram и оператор — другие агенты)."""
    return hold is not None and heard.source not in (SOURCE_TELEGRAM, SOURCE_OPERATOR)


def decide(heard: Heard, backlog: Optional[Backlog], wake_words: Sequence[str],
           question: Optional[SceneFrame] = None, greeting: Optional[SceneFrame] = None,
           ask: Optional[SceneFrame] = None, hold: Optional[SceneFrame] = None, *,
           music_playing: bool = False) -> Decision:
    """Адресована ли реплика роботу и какой у хода текст (§4.2, В12). ``question`` — живой кадр ``Identity``
    (робот спросил имя), `greeting` — окно после приветствия (#3776), ``ask`` — окно вопроса выступления
    ``PerformAsk`` (О9′): фраза без вейка, похожая на ответ, — ход. ``hold`` — кадр ``OperatorHold``: пока он
    есть, голос с микрофона не адресован никому (Telegram и оператор — другие агенты, их кадр не касается).
    ``music_playing`` — играет ли музыка: под ней искажённый вейк пропускает и правку музыки («роботы ускорь темп»,
    #3975; ворота ``under_music`` в ``knowledge.WAKE_SLIP_GATES``). Ход из самой фразы помнит её id
    (``Addressed.utterance_id``, :func:`_of_phrase`)."""
    return _of_phrase(_decide(heard, backlog, wake_words, question, greeting, ask, hold, music_playing), heard)


def _of_phrase(decision: Decision, heard: Heard) -> Decision:
    """Ход, текст которого — сама фраза ``heard`` (не подхват бэклога и не голый вейк), — с её ``utterance_id``: по нему
    ход находит SpeakerResult этой фразы (#4141)."""
    if isinstance(decision, Addressed) and not decision.pickup and decision.source != BARE_WAKE:
        return replace(decision, utterance_id=heard.utterance_id)
    return decision


def _decide(heard: Heard, backlog: Optional[Backlog], wake_words: Sequence[str], question: Optional[SceneFrame],
            greeting: Optional[SceneFrame], ask: Optional[SceneFrame], hold: Optional[SceneFrame],
            music_playing: bool) -> Decision:
    """Тело :func:`decide`: адресована ли реплика и какой у хода текст."""
    text = (heard.text or "").strip()
    if _held(heard, hold):
        return NotAddressed(HELD)
    if heard.source in (SOURCE_TELEGRAM, SOURCE_OPERATOR):
        if heard.source == SOURCE_TELEGRAM and is_coordinator_call(text):
            return NotAddressed(COORDINATOR)
        rest = strip_wake_word(text, wake_words)
        return Addressed(rest, heard.source) if rest else NotAddressed(EMPTY)
    woken = has_wake_word(text, wake_words)
    rest = (strip_wake_word(text, wake_words) if woken
            else slipped_wake_rest(text, wake_words, music_playing=music_playing))
    if rest is None:
        return _wakeless_reply(text, heard, question, greeting, ask)
    if is_bare(rest) and _yes_no_in_window(ask, rest, heard.t):  # «Робот, да» — ответ на вопрос, не голый вейк
        return Addressed(rest, PERFORM_ANSWER)
    return _woken(rest, heard, backlog, _command(text, rest, wake_words) if woken else "")


def _command(text: str, rest: str, wake_words: Sequence[str]) -> str:
    """Запасной разбор Tier-1 (``Addressed.command``): слова после вейка, если перед ним было что-то ещё; иначе «»."""
    command = wake_command(text, wake_words)
    return command if command and command != rest.strip(" ,.!?;:-–—") else ""


def decide_in_session(heard: Heard, backlog: Optional[Backlog], wake_words: Sequence[str], snap: Any,
                      greeting: Optional[SceneFrame] = None) -> Decision:
    """:func:`decide` для сессии Личности: окна ответа (``Identity``, ``PerformAsk``) и удержание оператором
    (``OperatorHold``) и «музыка играет» (ворота правки музыки искажённого вейка, #3975) — из снимка; ``greeting`` —
    окно после приветствия (живёт у хоста, не в снимке).
    Единственная точка, где ROS-хост ``dialog_node`` и сухой стенд (``scripts/dialog/dry_marathon.py``, #3906)
    решают «адресована ли реплика» одинаково."""
    return decide(heard, backlog, wake_words, question=snap.top("Identity"), greeting=greeting,
                  ask=snap.top("PerformAsk"), hold=snap.top("OperatorHold"), music_playing=snap.music_playing)


def _woken(rest: str, heard: Heard, backlog: Optional[Backlog], command: str = "") -> Addressed:
    """Вейк со словами — ход с фразой и фоновой фразой того же собеседника (§7.3, #3894); голый — подхват (В12).
    ``command`` — команда после вейка для Tier-1 (``Addressed.command``). Начало фразы и команды чинится по таблице
    известных искажений STT (``heard.repair``, #4017) — до Tier-1 и до модели."""
    if not is_bare(rest):
        said, after_wake = repair(rest), repair(command)
        recent = backlog.recent_unaddressed(heard) if backlog is not None else None
        since = backlog.background_since_reply(heard) if backlog is not None else []
        if backlog is not None:
            backlog.note_addressed(heard.t)  # фоновое до этой фразы модель видит в снимке хода: издалека не подхватывать
        return Addressed(said.text, VOICE, recent=(recent,) if recent else (), background=tuple(since),
                         command=after_wake.text, stt_fixes=said.fixes + after_wake.fixes)
    taken = backlog.pickup(heard) if backlog is not None else []
    if not taken:
        return Addressed("", BARE_WAKE)
    # что не вошло в рассказ, остаётся в бэклоге и идёт в снимок с автором и возрастом (#3975): «не сходится по смыслу -
    # часть могла уйти в бэклог» модель видит данными, не правилом
    return Addressed(" ".join(e.text for e in taken), BACKLOG_PICKUP, True,
                     tuple(Backlog.label(e, heard) for e in taken),
                     background=tuple(backlog.background_since_reply(heard)))


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
    """``speaker_id`` диктора этой фразы из реестра биометрии (``peek`` по id), если голос приписан профилю
    (``speaker_adapter.attributed_person``: спорный голос — никому, #3993); нет — ``""``."""
    result = registry.peek(utterance_id) if (registry is not None and utterance_id) else None
    return attributed_person(result) if isinstance(result, dict) else ""


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
