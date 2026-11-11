"""Ход диалога v2 без ROS: адресованная реплика → Tier-1 или команда LLM → исполнение → фраза кода (ADR-0150 §3, §4.1,
§7.3, §8; PR-8 ч.8d).

:class:`TurnEngine` — весь ход в одном месте и только вызовами модулей пакета: грамматика Tier-1 (``grammar.match``,
0 вызовов LLM), снимок хода (``context.build_context``) и окно истории (``turn_log``: озвученное, исходы действий и
метки кода), срез тулов
(``tools_view.available_tools``), команда модели, исполнение действий владельцам, фраза об исходе из события
(``respond``), дедлайн хода (``deadline``), e2e-события (``e2e_events``). Транспорт — :class:`Ports` снаружи (ROS-хост
``dialog_node``, ч.8e): вызов LLM с разбором и повтором по нарушениям, исполнитель действий (тот же MCP-провайдер
тулов, что у v1, — одна дорога исполнения), речь, стадии хода (``TurnEvent`` + рефлексы), отмена речи, интент.

Правила хода (ADR-0148: решает код):

* голое «робот» без бэклога — только рефлекс ``addressed``, 0 вызовов LLM, 0 речи (§7.3 п.1); с подхватом бэклога
  (В12″) — модель получает подхваченные фразы каждую со своим автором (``[говорит: …]`` на момент фразы, не
  того, кто сказал «робот»), и они остаются в окне прошлыми репликами;
* Tier-1 — исполняет код (стоп, тишина, громкость, голос, новая сессия, очередь показа лица «покажи по очереди
  радость, грусть и удивление» — ``play_animation`` на сцену, лицо итога и его очередь; «повтори, я не
  расслышал» — последняя озвученная реплика робота дословно, ``TurnLog.last_spoken``: модель не зовётся и
  идущее выступление не правит), спорный стоп (В19) идёт в LLM;
* вопрос тула («Удалить «X»? Скажи «да, удаляй».», ``knowledge.TOOL_QUESTIONS``) — окно ответа в сессии
  (``tool_ask``): ответ «да, удаляй» исполняет код подтверждённым вызовом, фраза — из результата тула; «нет» —
  фраза кода; другие слова — к LLM с тулом вопроса в срезе. Вопрос — не провал: фраза и лицо не грустят
  (``respond.failed``);
* невалидная команда (после повтора моста) — допустимые действия исполняются, итог (или честное «не понял») говорит
  код, ход ``done`` (Шифу 23.10); спасённый ``query`` возвращает данные модели, пока есть итерации, иначе код
  говорит по данным; ``speech`` рядом с тихими действиями (``knowledge.QUIET_WITH_SPEECH_TOOLS``) законен;
  ``passed`` — тихо;
* форма команды хода (:func:`command_form`): ``passed`` есть в схеме только при подхвате бэклога (адресованную
  реплику модель молчанием не закрывает — нарушение и повтор моста), голос реплики — только по просьбе о голосе,
  подложка сказки, анекдота и стиха — только по просьбе о музыке в этой реплике (``Form.backing``);
  путь пения решает хост до модели (:func:`sing_route`: ``live:song`` — поёт сам, ``perform`` обязателен,
  генерация и трек вне среза; ``generate:…`` — ``generate_music`` обязателен, песня выступлением — нарушение);
* голый неслышимый ответ (``play_animation`` без слов) на адресованную реплику, где анимацию не просили
  (``tools_view.needs_words``), не исполняется: модель получает нарушение ``VIOLATION_SILENT_ANSWER``
  следующей итерацией (≤ ``COMMAND_RETRIES_MAX`` повторов); повтор кончился — анимация и фраза кода, не молчание;
  команда из одной ``expression`` — тот же класс: по прямой просьбе («покажи, как тебе грустно») лицо молча,
  иначе повтор; человек сам просил без слов («молча», «без слов», ``tools_view.silent_asked``) — лицом и
  анимацией одинаково без речи и без нарушения;
* агент чата: человек просит сделать роботом («покекай роботом», «через робота», «вслух» —
  ``knowledge.CHAT_ROBOT_ASK_FORMS``; вопрос с теми же словами — справка, ``words.asks_question``), а команда — только
  текст в чат (:func:`robot_owed`): тот же путь — нарушение ``VIOLATION_REMOTE_OWED`` и повтор, повтор
  кончился — слова модели уходят в чат, а рядом код говорит факт «На роботе ничего не сделал.» (ответ не подменяется);
* выступление рядом с музыкой, запущенной той же командой («агитируй толпу под Калинку-Малинку»), идёт поверх неё
  (:func:`over_ordered_music`, ``knowledge.PERFORM_OVER_TOOLS``); голос, о котором просил человек («спой голосом
  Алисы», поле ``voice`` открыто формой), — голос всех его сегментов один раз (:func:`asked_voice`);
* экспрессия команды (эмоция, жест, сцена лица LLM) красит рефлекс итога ``done`` (:func:`done_expression` →
  ``Ports.stage(..., expression=)`` → ``expression.merge``), отдельного запроса лицу нет; исполненный ``play_animation``
  красит его своей сценой, сценой лица и временем (:func:`shown_face`: тул лицо не трогает); в тишине и у
  отменённого хода — рефлекс как есть; показ по просьбе держится ``expression.show_seconds`` (≥
  ``FACE_SHOW_MIN_S``, а не 3 с рефлекса), пачка ``play_animation`` — очередью (``Shown.then``); лица команды
  нет, а код сказал грустную фразу (провал, отказ чтения памяти) — итог грустный, а не рефлекс ``happy``;
* действия — владельцам; ``speech`` при действиях не звучит (§4.1), звучит фраза кода об исходе и ``perform``;
* ``query`` — данные обратно модели, не больше ``knowledge.LLM_QUERY_ITERATIONS_MAX`` итераций, и рядом с действиями
  тоже (``[play_animation, music_library list]`` — действия исполнены, об их исходе говорит код, модели уходят
  данные ``query``); действие, уже исполненное в этом ходе, следующая итерация не повторяет (:func:`unrepeated`,
  ``music_library play`` не перезапускает трек), сцены ``play_animation`` прошлой итерации лицо модели не
  перекрывает (``command_face(earlier=)``); последний вызов модели идёт без query-тулов в схеме (читать нечем —
  остаётся сказать итог), а если она всё же просит query (или только повторяет сделанное) и её слов не слышно — ход
  ``done`` с фразой кода по данным (``query_fallback``: у списка библиотеки — число треков и названия, п.3), а не
  молчащий ``failed`` (В21); чтение памяти,
  отклонённое кодом без собеседника, на вопрос о памяти о самом человеке — фраза кода (``memory.voiced_refusals``), на остальные реплики — данные модели, и она отвечает на вопрос;
* ход ведётся до конца, пока провайдер жив, но не дольше ``TURN_CEILING_S`` (В31): ~``PROGRESS_AFTER_S`` тишины —
  одна промежуточная фраза кода (``understand.progress``/``media.progress``); потолок в ожидании LLM — фраза с именем
  провайдера, в ожидании действия — фраза его класса о таймауте (не «не понял»); цепочка провайдеров отказала —
  фраза сразу, без ожидания потолка; строка ``provider_latency``;
* ясный стоп — перебивание (``interrupt.py``, PR-9): речь и выступление гаснут по слову («стой» — сразу, «хватит» —
  на границе предложения), эпоха + 1; отменённый ход старой эпохи — ``cancelled``, а доживший до ответа модели —
  молчит и ничего не исполняет (``stale_epoch``).
"""

from __future__ import annotations

import functools
import inspect
import json
import logging
import random
import time
import uuid
from dataclasses import dataclass, field, fields, replace
from types import MappingProxyType
from typing import (Any, Awaitable, Callable, Collection, Dict, FrozenSet, Iterable, List, Mapping, Optional, Protocol,
                    Sequence, Tuple)

from rob_box_core.tool_catalog import get_tool
from rob_box_core.utterance import Sink, Utterance
from rob_box_dialog import counting
from rob_box_dialog import face_scene
from rob_box_dialog import foreign_name
from rob_box_dialog import knowledge as kn
from rob_box_dialog import memory
from rob_box_dialog import pickup_parties
from rob_box_dialog.voice_warmth import no_warm_voices
from rob_box_dialog import state_claim
from rob_box_dialog import theme_switch
from rob_box_dialog import phrases
from rob_box_dialog import tool_ask
from rob_box_dialog import track_ref
from rob_box_dialog import action_chain
from rob_box_dialog.address import GREETING_WAKE, WORDLESS, Addressed
from rob_box_dialog.agents import agent_spec, allows_tier1, tools_for
from rob_box_dialog.command import (Command, Expression, Form, Invalid, Performance, Result, ToolCall, current_form,
                                    explain, fit_speech, loud_actions, turn_form)
from rob_box_dialog.context import build_context
from rob_box_dialog.deadline import TIMEOUT_PHRASE, DeadlineExpired, ProviderFailed, TurnDeadline, latency_line
from rob_box_dialog.execute import deadline_s
from rob_box_dialog.expression import face_label, show_seconds, shown_scene, shows
from rob_box_dialog.grammar import Tier1Command, Voices, match
from rob_box_dialog.heard import HEARD_MARK
from rob_box_dialog.interrupt import Interrupt, interrupt_for, stop_music_calls, stops_music
from rob_box_dialog.music_state import after_stop
from rob_box_dialog.respond import (ActionResult, asks, audible, classed, failed, join, outcome_emotion,
                                    phrase_from_result, reconcile_stops)
from rob_box_dialog.session import Session, SessionSnapshot, SpeakerRef
from rob_box_dialog.speaker_adapter import asks_recognized, known_voice_registers, recognition_reply, speaker_block
from rob_box_dialog.window_reset import WindowGuard, reset_if_idle, reset_window
from rob_box_dialog.words import asks_question, asks_to_listen, has_form, music_stop_claimed, same_stop_rest
from rob_box_dialog.tools_view import (available_tools, catalogs_asked, music_asked, needs_words, robot_asked,
                                       show_asked, voice_asked, voice_named)
from rob_box_dialog.turn_log import (PERFORM_STARTED, TOOL, USER, TurnAggregate, TurnLog, phrase_label, results_of,
                                     speaker_label)

_LOG = logging.getLogger(__name__)
#: Строка лога хода v2 (вход, решение, итог).
LOG_MARK = "🧭 [turn-v2]"
_QUERY = "query"
_FAILED = ("rejected", "timeout")
#: Строка лога подхвата бэклога голым «робот».
PICKUP_MARK = "🗒️ [backlog] pickup"
#: Метка строк лога окна вопроса тула: «открыт» после вопроса, «ответ» на реплику в окне.
ASK_MARK = "❓ [tool-ask]"
#: Раздел снимка с исходами действий прошлых ходов (окно ``turn_log``: ``tool`` — структурой, не в тексте реплик).
RECENT_ACTIONS = "recent_actions"
#: Раздел снимка хода с фразой: фоновая фраза того же собеседника (``Backlog.recent_unaddressed``, §7.3).
RECENT_UNADDRESSED = "recent_unaddressed"
#: Раздел снимка хода с фразой: все фоновые фразы всех авторов за последние ``BACKGROUND_WINDOW_S`` (В23, 23.10).
BACKGROUND_SINCE_REPLY = "background_since_reply"
#: Раздел снимка: сводка фона кодом - число голосов, кто узнан, сколько безымянных (``backlog.voices_summary``, #4212).
BACKGROUND_SUMMARY = "background_summary"


def host_sections(snap: SessionSnapshot, music: Any, providers: Sequence[Any] = (),
                  noted: Tuple[Optional[str], Optional[Mapping[str, Any]]] = (None, None)) -> Mapping[str, Any]:
    """Разделы снимка хода Личности из владельцев: собеседник (``speaker_block``), провайдеры, музыка
    (``MusicStateMemory.section``) — данные, не правила. Один сборщик для ROS-хоста ``dialog_node`` и сухого стенда
    (``scripts/dialog/dry_marathon.py``). ``noted`` — ``(person_ref, speaker_memory)``, взятые на начале хода
: раздел показывается, только если собеседник снимка тот же (сменился — чужого не показываем)."""
    person, block = noted
    again = bool(person) and person == memory.profiled_person(snap.speaker)
    sections = {"speaker": speaker_block(snap.speaker), "providers": list(providers), "music_state": music.section()}
    return {**sections, "speaker_memory": block} if again and block else sections


def recent_results(past: Sequence[Any], own: Sequence[Any], owner: Optional[str]) -> List[Mapping[str, Any]]:
    """Раздел ``recent_actions`` снимка хода: исходы действий прошлых ходов окна (``past``), затем исходы самого хода
    (``own``: ``TurnLog.results_of``) с меткой ``knowledge.THIS_TURN_FIELD``. Свои исходы нужны остатку фразы после
    ясного стопа: окно исключает ход, что спрашивает модель, и стоп музыки, исполненный кодом ДО её вызова в этом же
    ходе, иначе в снимке не было бы (n709 28.10: «сделаю потише» по музыке, которую код только что остановил).
    Исход тула памяти другого владельца памяти не идёт (``memory.foreign_result``)."""
    def mine(entry: Any) -> bool:
        return not memory.foreign_result(entry.content, entry.owner, owner)

    return [*(dict(e.content) for e in past if e.role == TOOL and mine(e)),
            *({**e.content, kn.THIS_TURN_FIELD: True} for e in own if mine(e))]


def _follow_as_user(data: str) -> Sequence[Any]:
    return ({"role": "user", "content": data},)


@dataclass(frozen=True)
class Answer:
    """Ответ LLM-порта: команда после разбора, проверки и повтора по нарушениям; провайдер; сколько вызовов LLM ушло
    (с повтором); токены промпта; ``follow(data)`` — продолжение запроса данными ``query`` (форму знает транспорт)."""

    result: Result
    provider: str = ""
    calls: int = 1
    tokens: int = 0
    follow: Callable[[str], Sequence[Any]] = _follow_as_user


class Ports(Protocol):
    """Транспорт хода (реализует ROS-хост). Необязательный ``provider() -> str`` — провайдер LLM, к которому идёт
    вызов сейчас (у цепочки — ``answered_by``): его имя называет фраза потолка (В31); без него — без имени.
    ``ask`` при отказе провайдера (цепочка исчерпана, транспорт оборвал вызов) бросает ``deadline.ProviderFailed`` —
    ход закрывается сразу, не ждёт потолка; прочие исключения — ошибка кода, ход падает громко."""

    def ask(self, messages: Sequence[Any], tools: Sequence[Any], timeout_s: float) -> Awaitable[Answer]: ...

    def execute(self, actions: Sequence[ToolCall], snapshot: SessionSnapshot,
                turn_id: str) -> Awaitable[List[ActionResult]]: ...

    def say(self, turn_id: str, epoch: int, text: str, voice: Optional[str] = None,
            emotion: Optional[str] = None, boundary: str = "none") -> str:
        """Реплика владельцу речи; ``boundary="sentence"`` - отмена режет её на конце предложения (счёт, #4207).
        Порт без ``boundary`` зовётся прежним вызовом (:func:`send_speech`)."""
        ...

    def stage(self, turn_id: str, epoch: int, stage: str, status: str, reason: str = "",
              cls: Optional[str] = None, llm_calls: int = 0, provider: str = "", source: str = "",
              expression: Optional[Expression] = None) -> Optional[bool]:
        """Событие стадии и рефлекс. ``expression`` (необязательный) — экспрессия команды на итоге ``done``
        (:func:`done_expression`): красит рефлекс. Порт без этого параметра (шлем оператора — лица у него нет)
        зовётся прежним вызовом (:func:`finish_stage`); ``False`` в ответ — лицо порт не показал (пульт чата)."""
        ...

    def cancel_speech(self, epoch: int, reason: str, boundary: str = "as_requested", speech_id: str = "") -> None:
        """Отмена речи эпохи хода; ``speech_id`` - ровно этой реплики (правка счёта, :func:`count_edit`). Порт без
        ``speech_id`` правку звучащего счёта не поддерживает."""
        ...

    def stop_perform(self, how: str, keep_backing: bool = False) -> bool:
        """Выступление: ``halt`` — сразу, ``stop`` — дочитать предложение. ``True`` — играющая музыка — его бит
        (её снимает само выступление, общий стоп музыки не нужен). ``keep_backing`` (необязательный: «а
        музыку оставь») — бит выступления доигрывает сам; порт без этого параметра зовётся прежним вызовом
        (:func:`stop_performance`)."""
        ...

    def intent(self, turn_id: str, epoch: int, kind: str) -> None: ...

    def perform(self, turn_id: str, epoch: int, perf: Performance, edit: bool) -> Optional[Mapping[str, Any]]:
        """Выступление — его владельцу (хост ``PerformBlock``: сегменты на тактах, подложка, паузы по сетке строк);
        исход владельца (``pending``/``done``/``rejected``). ``None`` — владельца выступлений у агента нет
        (оператор): сегменты звучат репликами хода. У чата выступление исполняет робот (пульт)."""
        ...


@dataclass
class _Turn:
    turn_id: str
    epoch: int
    text: str
    source: str
    deadline: TurnDeadline
    llm_calls: int = 0
    provider: str = ""
    tokens: int = 0
    stage: str = "addressed"
    status: str = "finished"
    reason: str = ""
    pickup: bool = False
    heard: Tuple[Tuple[str, str], ...] = ()  # подхват бэклога: (кто сказал, фраза) по порядку
    recent: Tuple[Mapping[str, Any], ...] = ()  # фоновая фраза того же собеседника для снимка
    background: Tuple[Mapping[str, Any], ...] = ()  # фоновые фразы всех с последнего ответа робота (В23)
    background_summary: Mapping[str, Any] = field(default_factory=dict)  # сводка голосов фона кодом (#4212)
    said: List[str] = field(default_factory=list)
    queried: Tuple[ActionResult, ...] = ()  # исходы выполненного в этом ходе query (для страховки от молчания)
    replacing: bool = False  # ход заказал новую музыку, пока играла прежняя (#4313): фразы ожидания - по этому факту
    waiting: str = "llm"  # чего ждёт ход: ответа LLM или события владельца (класс действия) — фразы В31
    retried: int = 0  # повторы модели за ответ не тот (``VIOLATION_SILENT_ANSWER``, ``…_REMOTE_OWED``)
    pushed: int = 0  # повторы за словами на последней итерации (``SPEECH_PUSHES_MAX``)
    expression: Optional[Expression] = None  # экспрессия исполненной команды — красит рефлекс ``done``
    command: str = ""  # запасной разбор Tier-1 (:func:`tier1_of`): слова после вейка (``Addressed.command``, n811)
    acted: Tuple[ToolCall, ...] = ()  # исполненные в ходе вызовы (не ``query``): повтор не исполняется
    sad: bool = False  # прозвучала грустная фраза (провал, отказ): лицо итога без лица команды - грусть
    utterance_id: str = ""  # фраза STT хода (``Addressed.utterance_id``): по ней - её SpeakerResult
    music_stopped: bool = False  # ход уже выключил музыку (стоп-тул ``done``): второй стоп не «ничего не играет»
    stop_said: bool = False  # Tier-1 уже сказал о стопе музыки (:func:`music_stop_phrase`): второй стоп хода молчит
    unvoiced: Tuple[ActionResult, ...] = ()  # исходы действий хода, о которых код ещё не сказал (Tier-1 стоп молчит)
    seen: Tuple[Mapping[str, Any], ...] = ()  # ``recent_actions`` снимка хода: данные, что модель видела до хода
    read: Tuple[ToolCall, ...] = ()  # исполненные в ходе вызовы ``query``: повтор не исполняется (:func:`unreread`)
    called: Tuple[str, ...] = ()  # тулы, вызванные в ходе: ``faces`` переносится на следующий ход (#4505)
    #: Подхват: собеседник хода по правам — автор фразы (``pickup_parties.pickup_speaker``); ``None`` — сессии.
    speaker: Optional[SpeakerRef] = None
    pickup_view: Mapping[str, Any] = field(default_factory=dict)  # раздел ``backlog_pickup``: кто и что сказал
    #: Таблица вейков личности (``SessionSnapshot.wake_words``, профиль продукта): ей снимают вейк с реплики хода.
    wake_words: Optional[Tuple[str, ...]] = None


def picked_heard(addressed: Addressed) -> Tuple[Tuple[str, str], ...]:
    """Подхваченные фразы бэклога ``(кто, фраза)``: автор — метка самой фразы (``Backlog.label``), не собеседник
    хода."""
    return tuple((phrase_label(p.get("speaker") or {}), str(p.get("text") or "")) for p in addressed.picked)


def pickup_parts(addressed: Addressed, snap: SessionSnapshot, warm: Sequence[str] = ()
                 ) -> Tuple[Optional[SpeakerRef], SessionSnapshot, Mapping[str, Any]]:
    """Подхват (решение Шифу 05.11, ``pickup_parties``): собеседник хода по правам (автор фразы, а не короткий вейк),
    снимок хода с ним и раздел ``backlog_pickup`` — авторы фраз (метки :func:`picked_heard`), инициатор (собеседник
    сессии ``snap``) и чья память у хода (``memory.memory_owner`` снимка хода); ``warm`` — прогретые голоса без имени
    (``voice_warmth``): автор не узнан — просьба назваться (``pickup_parties.ask_intro``). Не подхват —
    ``(None, snap, {})``."""
    speaker = pickup_parties.pickup_speaker(addressed, snap.speaker)
    own = pickup_parties.own_snapshot(snap, speaker)
    owner = speaker if speaker is not None and memory.memory_owner(own) else None
    authors = [who for who, _ in picked_heard(addressed)]
    intro = pickup_parties.ask_intro(addressed.picked, snap.speaker, speaker, warm)
    return speaker, own, pickup_parties.pickup_section(addressed.picked, authors, snap.speaker, owner, intro)


def recent_section(addressed: Addressed) -> Tuple[Mapping[str, Any], ...]:
    """Раздел ``recent_unaddressed`` адресованного хода: метка фразы бэклога + ``who`` — автор той же схемой, что
    тег ``kn.SPEAKER_TAG`` подхвата (:func:`phrase_label`)."""
    return tuple({**p, "who": phrase_label(p.get("speaker") or {})} for p in addressed.recent)


def background_section(addressed: Addressed) -> Tuple[Mapping[str, Any], ...]:
    """Раздел ``background_since_reply``: кто (:func:`phrase_label` и ``speaker_id`` — профиль голоса или ``unknown``,
    два безымянных голоса так различимы), что, сколько секунд назад — и только это."""
    return tuple({"who": phrase_label(p.get("speaker") or {}), "speaker_id": p.get("speaker_id"),
                  "text": p.get("text", ""), "age_s": p.get("age_s")} for p in addressed.background)


def user_text(turn: "_Turn") -> str:
    """Реплика человека в запросе хода: подхват — фразы с тегом авторства, иначе — текст хода."""
    if not turn.heard:
        return turn.text
    return "\n".join(kn.SPEAKER_TAG.format(who=who, text=text) for who, text in turn.heard)


def log_pickup(log: Any, addressed: Addressed, turn: "_Turn") -> None:
    """Голое «робот»: строка подхвата — сколько фраз, чьи, в каком ходе (``n=0`` — бэклог пуст или фразы старше
    окна, ход — только рефлекс)."""
    if addressed.pickup or addressed.source in WORDLESS:
        speakers = ",".join(who for who, _ in turn.heard) or "-"
        log.info(f"{PICKUP_MARK} n={len(turn.heard)} speakers={speakers} turn={turn.turn_id}")


def log_stt_fixes(log: Any, addressed: Addressed, turn: "_Turn") -> None:
    """Реплика починена по таблице искажений STT (``heard.repair``): строка «что было → что стало» — корпус
    искажений и живая проверка; ход идёт уже по починенному тексту."""
    if addressed.stt_fixes:
        log.info(f"{HEARD_MARK} {'; '.join(addressed.stt_fixes)} turn={turn.turn_id}")


def tier1_for(text: str, snapshot: SessionSnapshot, voices: Voices = ()) -> Optional[Tier1Command]:
    """Команда Tier-1 реплики, если она доступна агенту сессии (``AgentSpec.grammar``): «замолчи» в чате — не
    тишина Личности (решение О2), а слова для LLM телеграм-агента."""
    command = match(text, snapshot, voices)
    return command if command is not None and allows_tier1(agent_spec(snapshot.agent), command.kind) else None


def tier1_of(text: str, after_wake: str, snapshot: SessionSnapshot,
             voices: Voices = ()) -> Tuple[Optional[Tier1Command], str]:
    """Tier-1 хода и текст, по которому он сработал (его же читает политика перебивания ``interrupt_for``).

    Первым — фраза целиком (``text``): команда до вейка остаётся рефлексом («Стоп, Робот, пожалуйста» — ``stop_all``;
    «Сделай музыку, Робот, потише» — громкость музыки, а не голоса). Слова после вейка (``after_wake``,
    ``Addressed.command``) — только запасной разбор, когда целое не Tier-1 или стоп в нём спорный, а после вейка —
    ясная команда: эхо речи робота, склеенное микрофоном перед вейком (n811 27.10: «четыре недели робот хватит читать
    а музыку оставь» — целое спорное, после вейка ясный ``stop_speech``)."""
    whole = tier1_for(text, snapshot, voices)
    if after_wake and (whole is None or whole.disputed):
        command = tier1_for(after_wake, snapshot, voices)
        if command is not None and not command.disputed:
            return command, after_wake
    return whole, text


def tier1_actions(command: Tier1Command, snapshot: SessionSnapshot) -> Tuple[ToolCall, ...]:
    """Действия Tier-1 владельцам: громкость — тул канала, голос — ``set_voice`` (обычный голос снимает и режим тона:
    ``knowledge.VOICE_DEFAULT_RESETS``), «говори нормально» — сброс скорости и тона (``set_speed``/``set_pitch``
    ``normal``), шаг голоса — ``set_speed``/``set_pitch`` шагом, очередь показа — ``play_animation``
    на каждую сцену по порядку реплики (та же дорога исполнения, что у вызовов модели; окно LLM видит исход),
    стоп всего — музыка, если играет (:func:`~rob_box_dialog.interrupt.stops_music`); стоп речи (``stop_speech``) и
    оговорка «музыку оставь» музыку не трогают."""
    if command.kind == "volume":
        tool = kn.VOLUME_CHANNELS[command.args["channel"]].tool
        return (ToolCall(tool, {"action": command.args["action"]}),) if tool else ()
    if command.kind == "set_voice":
        voice = command.args["voice"]
        resets = kn.VOICE_DEFAULT_RESETS if voice == kn.VOICE_DEFAULT_ARG else ()
        return (ToolCall("set_voice", {"voice": voice}),) + _resets(resets)
    if command.kind == kn.PROSODY_NORMAL_KIND:
        return _resets(kn.PROSODY_RESET_TOOLS)
    if command.kind == kn.VOICE_STEP_KIND:
        return tuple(ToolCall(tool, {"action": action}) for tool, action in command.args["steps"])
    if command.kind == kn.SHOW_QUEUE_KIND:  # сцены по порядку реплики, время - по умолчанию показа
        return tuple(ToolCall(kn.SHOW_TOOL, {"animation": name}) for name in command.args["scenes"])
    return stop_music_calls(snapshot) if stops_music(command) else ()


def _resets(tools: Sequence[str]) -> Tuple[ToolCall, ...]:
    return tuple(ToolCall(tool, {"action": kn.PROSODY_NORMAL_ARG}) for tool in tools)


def stop_performance(ports: Any, how: str, keep_backing: bool = False) -> bool:
    """Перебивание выступления через ``Ports.stop_perform``. ``keep_backing`` («а музыку оставь») уходит в
    порт, только если его ``stop_perform`` объявил такой параметр: реализации порта без него (чат, оператор —
    выступлений у них нет) зовутся прежним вызовом, их не ломает новый вид стопа."""
    stop = getattr(ports, "stop_perform")
    if keep_backing and _accepts(stop, "keep_backing"):
        return bool(stop(how, keep_backing=True))
    return bool(stop(how))


def _accepts(fn: Callable[..., Any], name: str) -> bool:
    try:
        return name in inspect.signature(fn).parameters
    except (TypeError, ValueError):
        return False


def voices_passed(pickup: bool, snapshot: SessionSnapshot) -> bool:
    """«Промолчать» модели на адресованную реплику озвучивает код (Шифу 22.10); из бэклога и в тишине — молчание."""
    return not pickup and snapshot.top("Silence") is None


def silence_insisted(result: Result) -> Result:
    """Модель и после повтора моста с нарушением ``VIOLATION_PASSED_ADDRESSED`` «молчит» на адресованную
    реплику — исход хода ``passed``: код мычит фразой ``PASSED_ADDRESSED_PHRASE``, ход ``finished/passed`` (решение
    Шифу 22.10), как и до формы хода; не «не понял» невалидной команды. Это только ГОЛЫЙ ``passed``: рядом с речью или
    действиями поле ``passed`` на адресованной реплике отбрасывает валидатор (``command._drop_closed_passed``) - ответ исполняется и звучит, сюда такая команда не доходит и мычанием не теряется."""
    if isinstance(result, Invalid) and result.reason == kn.VIOLATION_PASSED_ADDRESSED:
        return Command(passed=kn.PASSED_INSISTED)
    return result


def command_form(turn: "_Turn", snapshot: SessionSnapshot, voices: Voices = (), sing: str = "") -> Form:
    """Форма команды хода (ADR-0148 п.2): ``passed`` — только при подхвате бэклога (адресованную реплику
    молчанием не закрыть: схема его не содержит, валидатор моста шлёт нарушение); голос реплики — только когда человек
    в этой реплике просил о голосе (``tools_view.voice_asked``: группа голоса или названный голос реестра ``voices`` —
    ``(id, название, *алиасы)``, как у Tier-1; id, название и каждый алиас - отдельные имена: слово ищется целиком
    по каждому, а не по склейке); ``sing`` — путь пения, решённый кодом до модели (:func:`sing_route`);
    живое пение при играющей музыке (``SessionSnapshot.music_playing``) — поверх неё (``Form.sing_over``, n810);
    подложка сказки, анекдота и стиха — только когда человек в этой реплике назвал музыку
    (``tools_view.music_asked``); песня и рэп ставят её по смыслу (``knowledge.PERFORM_BACKING_BY_KIND``)."""
    names = tuple(name for entry in voices for name in entry if name)
    return Form(passed=turn.pickup, voice=voice_asked(snapshot, turn.text, names), sing=sing,
                sing_over=bool(sing) and snapshot.music_playing, backing=music_asked(turn.text))


#: Путь пения хода от хоста: ``(реплика, прошлые реплики человека в окне, играет ли музыка) -> id кандидата``
#: (``live:song``, ``generate:robot_song:V6``; «» — не пение). Строит ``rob_box_music.source.choose_sing``; не пение, а
#: правка играющей музыки — ``edit:<вид>+…`` (``rob_box_music.track_edit.edit_route``, ``knowledge.EDIT_ROUTE``).
#: Хост с параметром ``voiced`` получает и то, назван ли в реплике голос реестра (``tools_view.voice_named``: «спой
#: голосом Алисы» — только живое).
SingRouter = Callable[..., str]
#: Голос фразы от хоста: ``(utterance_id, сколько ждать, с) -> SpeakerResult этой фразы`` (словарь
#: ``rob_box_core.speaker_wire``) или ``None`` - не пришёл за это время. Хост ``dialog_node`` - реестр биометрии по id
#: фразы (``UtteranceSpeakerRegistry.resolve``).
PhraseVoice = Callable[[str, float], Awaitable[Optional[Mapping[str, Any]]]]


async def no_phrase_voice(_utterance_id: str, _wait_s: float) -> Optional[Mapping[str, Any]]:
    """Хост без биометрии фраз: голоса фразы нет - на «ты меня узнал?» код честно «не уверен»."""
    return None


async def sing_route(engine: "TurnEngine", turn: "_Turn", snap: SessionSnapshot) -> str:
    """Путь пения до основной модели (решение Шифу 27.10): решает хост (маркеры слов, чойсер с контекстом), ход
    сужает по нему форму команды. У чата Telegram тот же хост, что у Личности (решение Шифу 28.10: «спой» из
    чата — по контексту, как голосом), музыка в его снимке — музыка робота. Хоста пути пения нет (оператор, тесты) или
    подхват бэклога — «»: форма не меняется.
    Чойсер может спросить модель (Jev/LLM при неясном разрыве) — вызов в пуле, петля хода не стоит. Исключение хоста
    ход не роняет (N5): строка WARNING и ход без формы пения, как без хоста."""
    import asyncio  # noqa: PLC0415 — asyncio тянет socket: импорт пакета чистый (test_purity)

    if engine._sing is None or turn.pickup:
        return ""
    recent = tuple(str(e.content) for e in engine.log.window(exclude=turn.turn_id) if e.role == USER)
    names = tuple(name for entry in engine._voices() for name in entry if name)
    route = functools.partial(engine._sing, turn.text, recent, snap.music_playing)
    if _accepts(engine._sing, "voiced"):  # голос реестра назван — путь только живой
        route = functools.partial(route, voiced=voice_named(turn.text, names))
    try:
        return await asyncio.get_running_loop().run_in_executor(None, route)
    except Exception as exc:  # noqa: BLE001 — путь пения не держит ход: без него модель решает как раньше
        engine._log.warning(f"{LOG_MARK} путь пения не решён ({type(exc).__name__}: {exc}) — ход без формы пения "
                            f"turn={turn.turn_id}")
        return ""


def form_tools(tools: Sequence[Any], form: Form, agent: str, owners: Optional[Sequence[str]] = None) -> Tuple[Any, ...]:
    """Тулы хода в форме пути пения: закрытые путём убраны, тул, который путь требует (``generate_music``), добавлен,
    если его срезала обрезка по ``TOOLS_VISIBLE_MAX`` (а агенту он доступен при владельцах продукта ``owners``)."""
    sing = form.sing_form
    if sing is None:
        return tuple(tools)
    kept = [t for t in tools if t.name not in sing.closed_tools]
    need = sing.needs_tool
    if need and need not in {t.name for t in kept} and need in tools_for(agent_spec(agent), owners):
        kept.append(get_tool(need))
    return tuple(kept)


def speaks_aloud(snapshot: SessionSnapshot) -> bool:
    """Кто ждёт слов от хода: агент говорит в динамики (Личность, не шлем оператора и не чат) и не
    тишина.
    Подхват бэклога - тоже: он бывает только по голому «робот», человек обратился к роботу (живое 28.10 17:55 и n301:
    «Робот.» → подхват → ответ одним лицом, тишина)."""
    return agent_spec(snapshot.agent).sink == Sink.SPEAKERS and snapshot.top("Silence") is None


def words_owed(snapshot: SessionSnapshot, text: str, cmd: Command) -> bool:
    """Команда модели не слышна, а человек ждёт слов: :func:`speaks_aloud` (и при подхвате бэклога), и
    сама команда без слов (``tools_view.needs_words``: голая анимация, которую словами не просили)."""
    return speaks_aloud(snapshot) and needs_words(text, cmd)


def read_owed(snapshot: SessionSnapshot, turn: "_Turn", cmd: Command) -> bool:
    """Ход прочитал данные (``_Turn.queried``), а ответ - одно лицо без слов, и лица человек не просил (#4295, класс
    К2 #4157; живой 03.11: ТАРС на «статус контейнеров» прочитал ``container_status`` и показал ``thinking`` - в Telegram
    ничего). Слов после чтения ждут от КАЖДОГО агента: у Личности это :func:`words_owed` (динамики), у оператора (шлем,
    чат) ``speaks_aloud`` ложно - без этого ход закрывался молча. Решает код по структуре хода и реплике человека
    (``tools_view.needs_words``), а не по тексту ответа."""
    return bool(turn.queried) and not speaks_aloud(snapshot) and needs_words(turn.text, cmd)


def sound_owed(snapshot: SessionSnapshot, turn: "_Turn", cmd: Command) -> bool:
    """Ход прочитал данные (``_Turn.queried``), человек спросил (``words.asks_question``), а ответ - одни звуки без
    слов (М11 n109, #4236: «какие звуки ты умеешь издавать?» → ``get_sound_info`` → ``play_sound robot_happy``; робот
    пискнул и не ответил). Успех звука молчит (``knowledge.SILENT_DONE_TOOLS``), так что вопрос остаётся без ответа.
    Звук на ПРОСЬБУ («включи какой-нибудь звук») - законный ответ без слов: здесь ``asks_question`` ложно. Слов ждут
    там же, где :func:`words_owed` (:func:`speaks_aloud`); лицо по просьбе (``tools_view.show_asked``) - не трогаем.
    Решает код по структуре команды и реплике человека, а не по тексту ответа модели (ADR-0148)."""
    asked = asks_question(turn.text, turn.wake_words)
    if not turn.queried or not speaks_aloud(snapshot) or show_asked(turn.text) or not asked:
        return False
    if cmd.speech or cmd.ask is not None or cmd.perform is not None or cmd.passed:
        return False
    return bool(cmd.actions) and all(a.tool in kn.SILENT_DONE_TOOLS for a in cmd.actions)


def catalog_owed(snapshot: SessionSnapshot, turn: "_Turn", cmd: Command) -> str:
    """Нарушение ``Catalog.violation`` (``knowledge.CATALOGS``) или «»: человек спросил о каталоге, который знает только
    тул робота (``tools_view.catalogs_asked``: «какие звуки ты умеешь?», «какими голосами ты умеешь говорить?»), агенту
    тул каталога доступен, а ответ - без его данных: в ходе тул не читали и в команде его нет (М11 n109, 01:12:19:
    «Умею подтверждать… глючить…» - звуки по памяти; М12 n401, 05:22:14: «надёжный мужчина… русоволосый друг детства»
    - голоса по памяти). Встречный вопрос, перформанс и пропуск хода - не ответ о каталоге. Решает код по реплике
    человека и структуре команды, не по тексту ответа (ADR-0148)."""
    if cmd.ask is not None or cmd.perform is not None or cmd.passed:
        return ""
    available = tools_for(agent_spec(snapshot.agent), snapshot.owners)
    for key in catalogs_asked(turn.text):
        catalog = kn.CATALOGS[key]
        read = any(r.tool == catalog.tool for r in turn.queried) or any(a.tool == catalog.tool for a in cmd.actions)
        if catalog.tool in available and not read:
            return catalog.violation
    return ""


def done_expression(turn: "_Turn", snapshot: SessionSnapshot) -> Optional[Expression]:
    """Экспрессия команды для рефлекса итога (ADR-0148: решает код): лицо команды красит ``done``. ``None`` —
    рефлекс как есть: ход не ``done`` или отменён (перебивание, ``stale_epoch``), в кадре ``Silence`` (лицо там —
    заклеенный рот), у команды нечего показать: ни сцены (``expression.shows``), ни очереди пачки за ней
    (``Shown.then``: пачка ``[talking, happy]`` - первой сцене нечего показать без речи, ``done`` - обычный
    рефлекс, а очередь всё равно идёт в свой срок, а не теряется вместе с первой сценой). Нейтраль показа (``idle``) -
    сцена: лицо покоя на своё время (``expression.shown_scene``), а не рефлекс ``happy``.

    Лица команды нет, а в ходе прозвучала грустная фраза (``_Turn.sad``: провал действия, отказ чтения памяти «Не узнаю
    твой голос…», «не понял») - итог грустный (:data:`SAD_FACE`), а не рефлекс ``happy`` (§5.5.1: радостно о
    провале не бывает)."""
    if turn.stage != "done" or turn.status == "cancelled" or snapshot.top("Silence") is not None:
        return None
    face = turn.expression
    queue = face.then if isinstance(face, Shown) else ()
    if face is not None and (shows(face.emotion, face.gesture, face.face) or queue):
        return face
    return SAD_FACE if turn.sad else None


@dataclass(frozen=True)
class Shown(Expression):
    """Лицо итога после исполненных ``play_animation``: сцена первого вызова (``emotion``), её сцена лица
    LLM и время ``duration_s`` (``expression.show_seconds``); ``then`` - сцены остальных вызовов той же команды, по
    очереди после неё, каждая своё время (``expression.queued``): пачка не перезаписывает друг друга. Не команда
    модели: схема команды - поля :class:`~rob_box_dialog.command.Expression`."""

    then: Tuple[Expression, ...] = ()


#: Лицо итога после грустной фразы кода, когда лица команды нет: эмоция фразы, время рефлекса ``done``.
SAD_FACE = Expression(emotion="sad")


def _asked_seconds(duration: Any, face: Optional[face_scene.FaceScene]) -> Any:
    """Заказанное время сцены: ``duration`` вызова (или ``expression.duration_s``), иначе ``duration_s`` её сцены лица,
    иначе не задано (``None``) — тогда ``expression.show_seconds`` берёт время по умолчанию."""
    if duration is not None:
        return duration
    return face.duration_s if face is not None else None


def shown_face(expression: Optional[Expression], data: Mapping[str, Any]) -> Optional[Expression]:
    """Сцена одного исполненного ``play_animation`` (ADR-0150 PR-3b: тул лицо не трогает, показывает
    диалог). ``animation`` (тул уже свёл его к имени сцены лица) - сцена вместо эмоции и жеста команды (жест
    ``face_node`` показал бы поверх сцены) со своей сценой лица LLM (:class:`Shown`). Только ``face`` - сцена лица
    поверх экспрессии команды. Время - ``duration`` вызова, иначе ``duration_s`` его сцены лица, по правилу показа
    ``expression.show_seconds`` (не меньше ``FACE_SHOW_MIN_S``; сцена лица гаснет вместе со сценой). Ни того ни
    другого - экспрессия команды как есть."""
    animation, duration, raw = (data.get(key) for key in kn.SHOW_FIELDS)
    face = face_scene.validate(raw)[0] if raw is not None else None
    if not animation and face is None:
        return expression
    seconds = show_seconds(_asked_seconds(duration, face))
    face = replace(face, duration_s=seconds) if face is not None else None
    if animation:  # нейтраль - лицо покоя на своё время, а не рефлекс done
        return Shown(emotion=shown_scene(str(animation)), face=face, duration_s=seconds)
    return replace(expression or Expression(), face=face, duration_s=seconds)


def _in_order(scenes: Sequence[Expression]) -> Expression:
    """Сцены исполненных ``play_animation`` по порядку вызовов: первая - лицо итога, остальные - его очередь
    (``Shown.then``), а не перезапись последней."""
    first = scenes[0]
    return Shown(**{f.name: getattr(first, f.name) for f in fields(Expression)}, then=tuple(scenes[1:]))


def held_expression(expression: Optional[Expression], text: str) -> Optional[Expression]:
    """Лицо команды без ``play_animation``: время - заказанное моделью (``expression.duration_s``) или, когда
    человек просил показать лицо (``tools_view.show_asked``: «покажи, как тебе грустно», «покажи молча, что ты
    задумался»), по правилу показа ``expression.show_seconds`` (не меньше ``FACE_SHOW_MIN_S``, а не 3 с рефлекса
    ``done``); сцена лица - то же время. Обычный ответ (показ не просили, время не задано) - время рефлекса, как
    было."""
    if expression is None or not shows(expression.emotion, expression.gesture, expression.face):
        return expression
    if expression.duration_s is None and not show_asked(text):
        return expression
    seconds = show_seconds(_asked_seconds(expression.duration_s, expression.face))
    face = replace(expression.face, duration_s=seconds) if expression.face is not None else None
    return replace(expression, face=face, duration_s=seconds)


def _scenes_of(face: Optional[Expression]) -> List[Expression]:
    """Сцены исполненных ``play_animation``, которые уже стоят в лице хода (:class:`Shown`: первая и очередь за ней),
    по порядку вызовов; иное лицо или его нет - пусто."""
    if not isinstance(face, Shown):
        return []
    return [replace(face, then=()), *face.then]


def command_face(expression: Optional[Expression], results: Sequence[ActionResult], text: str = "",
                 earlier: Optional[Expression] = None) -> Optional[Expression]:
    """Экспрессия исполненной команды для итога хода; исполненные ``play_animation`` красят итог своими
    сценами по очереди (:func:`shown_face`: «покажи, как тебе грустно» → ``sad``, а не рефлекс ``happy``;
    пачка — первая сцена на итоге, остальные в ``Shown.then``), иначе лицо команды со временем показа
    (:func:`held_expression`, ``text`` — реплика человека). Провал действия её не показывает (``None`` — рефлекс как
    есть): «радостно» о провале не бывает, как и в ``expression.merge``. Вопрос тула («Удалить «X»?») её тоже
    не показывает: это фраза кода, настроение модели её не красит. Одно правило для исполненной команды
    (``TurnEngine._speak``) и для действий, исполненных после нарушения (:func:`_close_invalid`).

    ``earlier`` - лицо хода после прошлых итераций (``turn.expression``): сцены ``play_animation``, исполненные в них,
    остаются на итоге, а сцены этой итерации встают очередью за ними (n905: ``[play_animation victory,
    get_current_time]`` → данные модели → речь; вторая итерация красила итог только своим результатом и затирала
    ``victory``, на лице оставался рефлекс ``done`` на 3 с). Та же сцена, повторённая моделью на следующей итерации
    (``[play_animation sad, list]`` до потолка итераций), в очередь второй раз не встаёт."""
    if any(failed(r) or asks(r) for r in results):
        return None
    shown = [shown_face(expression, r.data) for r in results if r.tool == kn.SHOW_TOOL and r.status == "done"]
    kept, fresh = _scenes_of(earlier), [s for s in shown if s is not None and s is not expression]
    scenes = kept + _without_repeat(kept, fresh)
    return _in_order(scenes) if scenes else held_expression(expression, text)


def _scene_key(scene: Expression) -> Tuple[Any, ...]:
    """Сцена как значение: поля команды ``Expression`` без очереди ``then`` и без вида (``Shown`` или нет)."""
    return tuple(getattr(scene, f.name) for f in fields(Expression))


def _without_repeat(kept: Sequence[Expression], fresh: Sequence[Expression]) -> List[Expression]:
    """Сцены этой итерации без повтора того, что уже стоит на итоге: самое длинное начало ``fresh``, совпадающее с хвостом
    ``kept``, отбрасывается. Модель, которая зовёт одно и то же на каждой итерации (``[play_animation sad, list]`` до
    потолка итераций), и одну сцену, и список из двух-трёх сцен, не множит показ (сравнивалась
    только ``fresh[0]`` с ``kept[-1]``, и список ``[victory, sad]`` встал бы второй раз целиком). Сцены в одной команде
    («покажи победу два раза») не схлопываются: сравниваются итерации, а не соседи внутри ``fresh``."""
    have, new = [_scene_key(s) for s in kept], [_scene_key(s) for s in fresh]
    for size in range(min(len(have), len(new)), 0, -1):
        if have[-size:] == new[:size]:
            return list(fresh[size:])
    return list(fresh)


def finish_stage(ports: Any, turn: "_Turn", snapshot: SessionSnapshot) -> Optional[Expression]:
    """Итог хода в ``Ports.stage``. Экспрессия команды (:func:`done_expression`) уходит, только если порт объявил
    ``expression``: лицо есть у Личности и у пульта чата (лицо робота), а шлем оператора зовётся прежним вызовом
    (как ``stop_performance``). Возвращает лицо, которое порт показал (``None`` - рефлекс как есть, порт лица не
    принимает или ответил ``False``: чат без пульта, лицо - настроение текста в чат): ход записывает его в окно
    (``TurnLog.faced``), и запись «показано» не опережает показ."""
    face = done_expression(turn, snapshot)
    shown = face if face is not None and _accepts(ports.stage, "expression") else None
    extra = {"expression": shown} if shown is not None else {}
    took = ports.stage(turn.turn_id, turn.epoch, turn.stage, turn.status, turn.reason, llm_calls=turn.llm_calls,
                       provider=turn.provider, **extra)
    return None if took is False else shown


def prosody_phrase(results: Sequence[ActionResult]) -> str:
    """Фраза «говори нормально» из РЕЗУЛЬТАТОВ ``set_speed``/``set_pitch`` (ADR-0148): оба сброса удались -
    одна фраза; хоть один нет - фразы исходов тулов (``respond.join``): что встало на норму и что не вышло
    (провайдер без регулировки - ``local.rejected.*_unsupported``). Результатов нет (ход отброшен) - молчание."""
    if results and all(r.status == "done" for r in results):
        return phrases.text(kn.PROSODY_NORMAL_PHRASE)
    return outcome_phrase(results) or ""


def show_queue_phrase(results: Sequence[ActionResult]) -> Optional[str]:
    """Фраза очереди показа: все сцены приняты - слов нет, ответ - лицо (прямая просьба показать, решение Шифу
    26.10; речь рисовала бы рот поверх первой сцены). Хоть одна не принята - фраза исхода по РЕЗУЛЬТАТУ тула
    (``respond.join``), а не молчание."""
    if all(r.status == "done" for r in results):
        return None
    return outcome_phrase(results) or phrases.text("local.rejected")


def tier1_face(command: Tier1Command, results: Sequence[ActionResult], text: str) -> Optional[Expression]:
    """Лицо итога хода Tier-1: у очереди показа - сцены исполненных ``play_animation`` по порядку реплики
    (:func:`command_face`: первая на итоге ``done``, остальные - ``Shown.then``, каждая своё время
    ``expression.show_seconds``); провал - рефлекс как есть. У остальных видов лица команды нет."""
    return command_face(None, results, text) if command.kind == kn.SHOW_QUEUE_KIND else None


def repeat_phrase(last_spoken: str) -> str:
    """«Повтори» (ADR-0148: решает код): последняя озвученная реплика робота дословно; робот в этой сессии ещё
    ничего не сказал - честная фраза кода, а не тишина и не выдумка модели."""
    return last_spoken or phrases.text(kn.REPEAT_NOTHING_PHRASE)


def voice_phrase(results: Sequence[ActionResult]) -> Optional[str]:
    """Фраза смены голоса Tier-1 (``set_voice``): ОДНА реплика по результатам тулов команды (ADR-0148), тул голоса
    сам не говорит. Обычный голос (``set_voice`` default) снимает и режим тона: голос применён (или уже обычный) и тон
    правда менялся — «Говорю своим обычным голосом.» целиком (``knowledge.VOICE_DEFAULT_PHRASE``): «Я и так говорю…»
    было бы неправдой, а отдельное «Тон голоса обычный.» — второй репликой об одном. Ось уже на норме (причина
    ``VOICE_STEP_AT_LIMIT``) молчит; не вышло что-то — фразы исходов (``outcome_phrase``), и сброс тона, и голос."""
    own = [r for r in results if r.reason != kn.VOICE_STEP_AT_LIMIT]
    lifted = any(r.tool in kn.VOICE_DEFAULT_RESETS and r.status == "done" for r in own)
    voice_ok = any(r.tool == "set_voice" and r.status == "done" for r in own)
    if lifted and voice_ok:
        return phrases.text(kn.VOICE_DEFAULT_PHRASE)
    return outcome_phrase(own)


def music_stop_phrase(command: Tier1Command, results: Sequence[ActionResult],
                      pick: Callable[[str], str]) -> Optional[str]:
    """Фраза Tier-1 стопа музыки (решение Шифу 05.11: «всегда что-то говорить, разные вариации»; М12 n511/n712 -
    «глуши музыку» молчало). Вариант выбирает код без повтора подряд (``pick``), исход - по факту: тул стопа отказал -
    его фраза (``music.stop_fail``); музыку погасили (``was_playing`` не ``False`` или погасил раньше в ходе) -
    :data:`~rob_box_dialog.knowledge.MUSIC_STOP_SAID`; играть было нечему - ``MUSIC_STOP_QUIET`` («И так тихо»), но у
    ``stop_all`` без тулов музыки (стоп одной речи, выступление погасил хост) слов нет: о музыке его не просили."""
    own = [r for r in results if r.cls != _QUERY]
    if any(r.status != "done" for r in own):
        return outcome_phrase(own)
    if not own and command.kind != kn.MUSIC_OFF_KIND:
        return None
    played = any(r.data.get("was_playing") is not False or r.data.get(kn.STOPPED_EARLIER_FIELD) for r in own)
    return pick(kn.MUSIC_STOP_SAID if played else kn.MUSIC_STOP_QUIET)


def note_stop_said(turn: "_Turn", command: Tier1Command) -> None:
    """Tier-1 сказал фразу о погашенной музыке (:func:`music_stop_phrase`): стоп модели в том же ходе её не повторит
    (:func:`unsaid_stops`), заглушка по данным рядом не нужна (:func:`data_close`)."""
    if stops_music(command) and turn.music_stopped:
        turn.stop_said = True


def unsaid_stops(turn: "_Turn", results: Sequence[ActionResult]) -> Sequence[ActionResult]:
    """Исходы без стопа музыки, о котором ход уже сказал (``_Turn.stop_said``): «хватит, диджей выключай» - Tier-1
    сказал «Выключил музыку», а стоп модели в том же ходе (``STOPPED_EARLIER_FIELD``) второй раз этого не говорит."""
    if not turn.stop_said:
        return results
    return [r for r in results if not (r.status == "done" and r.data.get(kn.STOPPED_EARLIER_FIELD))]


def tier1_phrase(command: Tier1Command, results: Sequence[ActionResult],
                 pick: Callable[[str], str] = lambda key: "", last_spoken: str = "") -> Optional[str]:
    """Фраза Tier-1: громкость — по исходу тула; сброс скорости и тона — по исходам тулов (:func:`prosody_phrase`);
    шаг голоса — по исходам тулов («Говорю ниже.», ось на границе — «Ниже уже некуда…», отказ — его фраза);
    новая сессия — вариант из каталога; очередь показа — лицом, слова только о провале (:func:`show_queue_phrase`);
    «повтори» — ``last_spoken`` дословно (последняя озвученная реплика робота, ``TurnLog.last_spoken``), а
    повторять нечего — фраза кода ``REPEAT_NOTHING_PHRASE``; стоп музыки — вариант кода по исходу
    (:func:`music_stop_phrase`), стоп одной речи и тишина молчат; смена голоса — одна фраза кода по результатам тулов
    (:func:`voice_phrase`), тул голоса не говорит."""
    if command.kind == kn.REPEAT_KIND:
        return repeat_phrase(last_spoken)
    if command.kind == "new_session":
        return pick(kn.NEW_SESSION_PHRASE)
    if command.kind == kn.PROSODY_NORMAL_KIND:
        return prosody_phrase(results)
    if command.kind == kn.SHOW_QUEUE_KIND:
        return show_queue_phrase(results)
    if stops_music(command):  # «выключи музыку», «хватит» под музыкой, стоп без вейка (решение Шифу 05.11)
        return music_stop_phrase(command, results, pick)
    if command.kind == kn.VOICE_STEP_KIND:
        return outcome_phrase(results)
    if command.kind == "set_voice":
        return voice_phrase(results)
    if command.kind != "volume":
        return None
    prefix = "music" if command.args["channel"] == "music" else "voice"
    ok = bool(results) and all(r.status == "done" for r in results)
    return phrases.text(f"{prefix}.volume.{command.args['action']}" if ok else f"{prefix}.volume_fail")


def hold_silence(engine: "TurnEngine", turn: "_Turn") -> bool:
    """Тишина (кадр ``Silence``): после Tier-1 ход не идёт к LLM и ничего не говорит (решение Шифу 23.10: робот
    молчит, снимает только «Робот, говори»; вейк тишину не снимает). Реплика уже в журнале хода (``TurnLog.begin``), здесь
    - строка лога с причиной, метка «рот заклеен» в окне LLM после реплики (``TurnLog.mark``: без неё реплика
    висит без ответа) и итог хода ``silenced``/``finished``: не провал и не «не понял»."""
    if engine.session.snapshot.top("Silence") is None:
        return False
    engine.log.mark(turn.turn_id, kn.WINDOW_MUTED_MARK)
    engine._log.info(f"{LOG_MARK} {kn.SILENCED_TURN_REASON}: LLM не зовётся, речи нет text={turn.text[:80]!r} "
                     f"turn={turn.turn_id}")
    turn.stage, turn.status, turn.reason = kn.SILENCED_TURN_STAGE, "finished", kn.SILENCED_TURN_REASON
    return True


async def answer_recognized(engine: "TurnEngine", turn: "_Turn", snap: SessionSnapshot) -> bool:
    """«Ты меня узнал?» (решение Шифу 26.10): ответ строит код (``speaker_adapter.recognition_reply``), LLM не
    зовётся - модель не выдумает «узнал» спорному голосу и «голос не запомнил» узнанному. Голос - ЭТОЙ фразы:
    её SpeakerResult по ``utterance_id`` от хоста, не дольше ``RECOGNIZED_VOICE_WAIT_S`` (биометрия и STT не
    упорядочены); не пришёл - «не уверен», а не собеседник прошлой фразы, иначе опоздавшая биометрия дала бы новому
    голосу чужое имя. Собеседник снимка нужен только короткой фразе без оценки (``inconclusive``). Только у
    агента с голосом (``PERSON_AGENTS``) и не в подхвате бэклога: там вопрос мог задать не тот, кто сказал «робот».
    Реплика - не этот вопрос - ``False``, ход идёт модели."""
    if turn.pickup or snap.agent not in kn.PERSON_AGENTS or not asks_recognized(turn.text):
        return False
    result = await engine._phrase_voice(turn.utterance_id, kn.RECOGNIZED_VOICE_WAIT_S)
    state, said = recognition_reply(result, snap.speaker, now=engine.session.now())
    turn.stage, turn.reason = "done", f"{kn.RECOGNIZED_TURN_REASON}:{state}"
    voice = "нет за окно" if result is None else "есть"
    engine._log.info(f"{LOG_MARK} узнал? {state}: ответ кодом по голосу фразы utterance={turn.utterance_id or '-'} "
                     f"({voice}), LLM не зовётся turn={turn.turn_id}")
    engine._say(turn, said)
    return True


def answer_listen_only(engine: "TurnEngine", turn: "_Turn") -> bool:
    """Голое «робот» подхватило из бэклога просьбу слушать и не встревать (марафон-2 n301: «ты просто слушай и не
    влезай»; ``words.asks_to_listen``): подхват адресован, молчать нельзя, но и отвечать по существу не о чем - код
    говорит короткое ``LISTEN_ONLY_PHRASE`` («Слушаю»), LLM не зовётся. Другой подхват или не подхват - ``False``."""
    if not turn.pickup or not asks_to_listen(turn.text):
        return False
    turn.stage, turn.reason = "done", kn.LISTEN_ONLY_PHRASE
    engine._log.info(f"{LOG_MARK} подхват бэклога - просьба слушать {turn.text[:80]!r} → «слушаю» кодом, LLM не "
                     f"зовётся turn={turn.turn_id}")
    engine._say(turn, phrases.text(kn.LISTEN_ONLY_PHRASE))
    return True


def wordless(engine: "TurnEngine", turn: "_Turn") -> bool:
    """Ход без слов человека — модели нечего отвечать, ход кончается без LLM. Голое «робот» — рефлекс (лицо, кольцо);
    голое «робот» в окне ответа на приветствие (``GREETING_WAKE``, #4509) — ещё и «Слушаю.» кодом: человека позвал
    робот, молча закрытый ход оставлял его без ответа. ``False`` — у хода есть текст, он идёт дальше."""
    if turn.source == GREETING_WAKE:
        turn.stage, turn.reason = "done", kn.LISTEN_ONLY_PHRASE
        engine._log.info(f"{LOG_MARK} голое «робот» в окне ответа на приветствие → «слушаю» кодом, LLM не зовётся "
                         f"turn={turn.turn_id}")
        engine._say(turn, phrases.text(kn.LISTEN_ONLY_PHRASE))
        return True
    return turn.source in WORDLESS or not turn.text


async def understand(engine: "TurnEngine", turn: "_Turn", snap: SessionSnapshot) -> None:
    """Ход к LLM (``TurnEngine._understand``), если его не проглотила тишина (:func:`hold_silence`) и на него не ответил
    код (:func:`answer_listen_only`, :func:`answer_recognized`), — в форме команды хода (:func:`command_form`):
    схема и валидатор моста видят её на время вызовов LLM этого хода."""
    if hold_silence(engine, turn) or answer_listen_only(engine, turn) or await answer_recognized(engine, turn, snap):
        return
    form = command_form(turn, snap, engine._voices(), await sing_route(engine, turn, snap))
    over = " поверх играющего" if form.sing_over else ""
    kinds = "" if form.backing else " подложка=песня,рэп"  # сказка, анекдот, стих - без бита, музыку не просили
    engine._log.info(f"{LOG_MARK} форма команды: закрыты={list(form.closed()) or '-'} пение={form.sing or '-'}{over}"
                     f"{kinds} turn={turn.turn_id}")
    with turn_form(form):
        await engine._understand(turn, snap)


def outcome_phrase(results: Sequence[ActionResult]) -> Optional[str]:
    """Фраза кода об исходе действий хода (``respond.join``): без ``query`` (его данные уходят модели). Голос тоже
    здесь: тулы ``set_voice``/``set_tts_provider`` молчат, фраза — по их результату (``knowledge.VOICE_DONE``). Слота нет
    в данных события — общая фраза ``local.*``."""
    own = [r for r in results if r.cls != _QUERY]
    try:
        said = join(own)
    except KeyError:
        lost = [r for r in own if r.status in _FAILED]
        return phrases.text(f"local.{lost[0].status}") if lost else None
    return said.text if said is not None else None


def _spoken_outcome(cmd: Command, results: Sequence[ActionResult], agent: str,
                    performing: bool = False) -> Optional[str]:
    """Фраза кода об исходе хода; речь модели рядом с тихими действиями (в чат телеграма она не звучит) молчит об их
    простом успехе (``respond.audible``), а при идущем выступлении (``performing``) молчит общий успех запуска
    музыки — только у агента, чьи фразы звучат в динамиках поверх выступления: чату о запуске музыки на роботе
    код говорит всегда (пульт). Исходов ``query`` здесь нет: пока итерации есть, они уходят модели
    (:func:`query_more`, там же отказ чтения памяти), на последней ``query`` не исполняется (:func:`_runnable`)."""
    speaks = bool(cmd.speech) and agent != kn.TEXT_CHANNEL_AGENT
    over_performance = performing and agent_spec(agent).sink == Sink.SPEAKERS
    return outcome_phrase(audible(results, speaks, over_performance)) if results else None


def _content_lines(cmd: Command, agent: str = kn.AGENTS[0], segments: bool = True) -> List[Tuple[str, str]]:
    """Что звучит из слов модели: реплика или вопрос — только без действий (§4.1), выступление — всегда. У агента с
    текстовым каналом (телеграм, §9.2) реплика при действиях — текст в чат, он не звучит, и правило §4.1 не нужно.
    ``segments=False`` — выступление взял его владелец (``Ports.perform``): сегменты звучат там, не репликами."""
    lines: List[Tuple[str, str]] = []
    said = cmd.speech or (cmd.ask.text if cmd.ask is not None else None)
    if said and (not loud_actions(cmd) or agent == kn.TEXT_CHANNEL_AGENT):
        lines.append((said, cmd.voice or ""))
    elif said:
        _LOG.info(f"{LOG_MARK} речь при действиях не звучит (§4.1): {said[:120]!r}")
    if cmd.perform is not None and segments:
        voices = cmd.perform.voice_per_segment or ("",) * len(cmd.perform.segments)
        lines.extend(zip(cmd.perform.segments, voices))
    return lines


def hand_perform(ports: Ports, turn_id: str, epoch: int, cmd: Command, log: Any = _LOG) -> Tuple[bool, str]:
    """Выступление команды — его владельцу (``Ports.perform``: такты, подложка, паузы по сетке строк). Раньше v2 читал
    сегменты рэпа репликами хода — без бита и без пауз. ``(взял ли владелец, фраза кода об отказе или "")``; не взял
    (выступления нет, у агента нет владельца) — сегменты звучат репликами хода. Фраза отказа — по его причине
    (``perform.rejected.<причина>``: пульт чата при ``operator_hold``), причины без ключа — «Не вышло.»."""
    if cmd.perform is None:
        return False, ""
    outcome = ports.perform(turn_id, epoch, cmd.perform, cmd.edit_pending)
    if outcome is None:
        return False, ""
    log.info(f"{LOG_MARK} выступление {'правка' if cmd.edit_pending else 'старт'} kind={cmd.perform.kind} "
             f"backing={cmd.perform.backing or '-'} segments={len(cmd.perform.segments)} → {dict(outcome)} "
             f"turn={turn_id}")
    if outcome.get("status") != "rejected" or outcome.get("phrased"):  # отказ уже озвучило выступление — одна фраза
        return True, ""
    reason = outcome.get("reason")
    key = phrases.first_key(f"{kn.PERFORM_REJECTED_PHRASE}.{reason}" if reason else None, "local.rejected")
    return True, phrases.text(str(key))


def asked_voice(cmd: Command) -> Command:
    """Голос, которым человек просил выступить («спой голосом Алисы», решение Шифу 28.10): поле ``voice``
    команды форма хода открывает только по просьбе о голосе в этой реплике (``Form.voice``), валидатор сверяет его с
    каталогом звучащего провайдера — им звучат все сегменты выступления этой команды, один раз, без ``set_voice``
    (голос робота не меняется). Голоса ролей сказки уже решил код (``voice_per_segment``) — их это не трогает."""
    perf = cmd.perform
    if perf is None or not cmd.voice or perf.voice_per_segment:
        return cmd
    return replace(cmd, perform=replace(perf, voice_per_segment=(cmd.voice,) * len(perf.segments)))


def over_ordered_music(cmd: Command, results: Sequence[ActionResult]) -> Command:
    """Выступление рядом с музыкой, которую запустила та же команда (решение Шифу 28.10: «агитируй толпу под
    Калинку-Малинку» → ``request_music`` пьесы + выступление): заказанное и есть подложка — ``backing=none``, поверх
    него, без второго бита и без вопроса «выключить и зачитать?» (``knowledge.PERFORM_OVER_TOOLS``). Решает код по
    исходам команды: музыка не пошла (отказ, таймаут) — подложка как просила модель; правка идущего — не трогается."""
    perf = cmd.perform
    if perf is None or cmd.edit_pending or perf.backing == "none":
        return cmd
    if not any(r.tool in kn.PERFORM_OVER_TOOLS and r.status in ("done", "pending") for r in results):
        return cmd
    _LOG.info(f"{LOG_MARK} выступление поверх музыки этой команды: backing {perf.backing or '-'} → none")
    return replace(cmd, perform=replace(perf, backing="none"))


def note_perform_start(log: TurnLog, turn_id: str, cmd: Command, taken: bool) -> None:
    """Владелец взял выступление (``taken``): в окне LLM просьба закрыта записью о старте, дальше её обновляет конец
    выступления (``TurnLog.performance_ended``). Сегменты звучат у владельца мимо реплик хода - без записи
    просьба висела бы неотвеченной и исполнялась заново на чужую реплику."""
    if taken and cmd.perform is not None:
        log.performed(turn_id, cmd.perform.kind, PERFORM_STARTED)


def _runnable(actions: Sequence[ToolCall], last: bool) -> Tuple[ToolCall, ...]:
    """Действия к исполнению: на последней итерации ``query`` уже не исполняется (данные некому вернуть). Класс -
    вызова (``knowledge.call_class``): ``music_library`` list/search/info - ``query``."""
    return tuple(a for a in actions if not last or kn.call_class(a.tool, a.args) != _QUERY)


def acted_calls(actions: Sequence[ToolCall], results: Sequence[ActionResult]) -> Tuple[ToolCall, ...]:
    """Вызовы, исполненные действием (не ``query``): их исход сказан кодом, повтор в том же ходе не исполняется
    (:func:`unrepeated`). Результаты исполнителя - в порядке вызовов (:func:`execute_for_speaker`)."""
    return tuple(call for call, result in zip(actions, results) if result.cls != _QUERY)


def unrepeated(log: Any, turn: "_Turn", cmd: Command) -> Command:
    """Команда без действий, уже исполненных в этом ходе (ADR-0148: решает код). После смешанной команды
    ``[действие, query]`` модели уходят только данные ``query`` (:func:`query_more`; ADR-0150 §4.1: ``action.*``
    терминальны), и во втором ответе она может повторить действие: ``music_library play`` перезапустил бы трек,
    ``play_animation`` перезаписал бы сцену, провал прозвучал бы дважды. Тот же вызов (тул и аргументы) второй раз за
    ход не исполняется - строка лога называет отброшенное; что модель не дала ничего сверх сделанного, решает
    :func:`unanswered`. Повтор чтения (``query``) исполняется: данные те же, итераций ограничено."""
    repeats = tuple(a for a in cmd.actions if any(a.tool == b.tool and dict(a.args) == dict(b.args)
                                                  for b in turn.acted))
    if not repeats:
        return cmd
    log.info(f"{LOG_MARK} повтор исполненного в этом ходе не исполняется: {[a.tool for a in repeats]} "
             f"turn={turn.turn_id}")
    return replace(cmd, actions=tuple(a for a in cmd.actions if a not in repeats))


def _reread(turn: "_Turn", call: ToolCall) -> bool:
    return any(call.tool == b.tool and dict(call.args) == dict(b.args) for b in turn.read)


def unreread(log: Any, turn: "_Turn", cmd: Command) -> Command:
    """Команда без чтений, уже исполненных в этом ходе (#4286; живой 03.11 №1 #4253: ``get_robot_status`` дважды
    подряд): тот же ``query`` (тул и аргументы) второй раз не исполняется - его данные уже у модели в окне хода. Что
    сказать модели или закрыть ход кодом, решает :func:`reread_hint`."""
    repeats = tuple(a for a in cmd.actions if _reread(turn, a))
    if not repeats:
        return cmd
    log.info(f"{LOG_MARK} {kn.VIOLATION_QUERY_REPEAT}: повтор чтения в этом ходе не исполняется: "
             f"{[a.tool for a in repeats]} turn={turn.turn_id}")
    return replace(cmd, actions=tuple(a for a in cmd.actions if a not in repeats))


def reread_reason(turn: "_Turn", cmd: Command, fresh: Command, last: bool) -> str:
    """Модель только повторила исполненное, среди него - прочитанное в ходе (:func:`unreread`), и слов не дала:
    ``VIOLATION_QUERY_REPEAT`` - модели подсказка, что данные уже выше (``QUERY_REPEAT_HINT``, :func:`_answer_owed`),
    пока повтор есть; повтора нет - «», ход закрывает код по данным (:func:`settled` → :func:`_close_unread`)."""
    if not retry_left(turn, last) or not settled(cmd, fresh) or not any(_reread(turn, a) for a in cmd.actions):
        return ""
    return kn.VIOLATION_QUERY_REPEAT


def counted_in_full(log: Any, turn: "_Turn", cmd: Command) -> Command:
    """Счёт, урезанный моделью, - числами кода (``counting.full_count``, #4314 n802: «до сорока» → «Один… Десять.»)."""
    full = counting.full_count(cmd, turn.text)
    if full is None:
        return counting.sentenced(cmd, turn.text) or cmd  # счёт речью - по числу в предложении (#4207)
    reached = counting.reached(cmd)
    upto = f"до {max(reached)}" if reached else "обещан без чисел"
    log.warning(f"{LOG_MARK} счёт урезан моделью ({upto}): числа строит код {counting.asked_range(turn.text)} "
                f"turn={turn.turn_id}")
    return full


def send_speech(ports: Any, turn_id: str, epoch: int, text: str, voice: Optional[str],
                emotion: Optional[str]) -> str:
    """Реплика владельцу речи. Счёт («Один. Два. …», ``counting.is_count``) - с ``boundary=sentence``: отмена (правка
    счёта, «хватит») дочитывает звучащее число, а не режет его посередине (#4207)."""
    if counting.is_count(text) and _accepts(ports.say, "boundary"):
        return ports.say(turn_id, epoch, text, voice, emotion, boundary="sentence")
    return ports.say(turn_id, epoch, text, voice, emotion)


def _sounding_count(engine: "TurnEngine", turn: "_Turn") -> Optional[str]:
    """``speech_id`` последнего звучащего счёта прошлого хода (``TurnLog.sounding``); нет - ``None``."""
    counts = [sid for sid, owner, text in engine.log.sounding() if owner != turn.turn_id and counting.is_count(text)]
    return counts[-1] if counts else None


def count_edit(engine: "TurnEngine", turn: "_Turn", cmd: Command) -> Command:
    """Правка звучащего счёта (#4207, n807: «и после каждого числа добавляй слово шаг»). Модель на неё присылает счёт
    с начала («Один шаг, два шага…»), и он звучал повтором после старого. Решает код (ADR-0148): старый счёт режется
    на границе звучащего числа (``cancel_speech`` по его ``speech_id``, реплика счёта отдана с
    ``boundary=sentence``), а продолжение строится, когда владелец речи скажет, сколько прозвучало
    (:func:`continue_count`, ``counting.continued``): числа - старого ряда после сказанного, форма - из правки.
    Новая просьба о счёте («посчитай до тридцати») - не правка; речь при громких действиях не звучит (§4.1) - и не
    правит. Не правка - команда как есть."""
    sid = _sounding_count(engine, turn)
    if (sid is None or not cmd.speech or loud_actions(cmd) or not counting.is_count(cmd.speech)
            or counting.asked_range(turn.text) is not None or not _accepts(engine._ports.cancel_speech, "speech_id")):
        return cmd
    engine._count_edits[sid] = (turn, cmd.speech, cmd.voice, cmd.expression.emotion)
    engine._ports.cancel_speech(turn.epoch, "count_edit", "as_requested", speech_id=sid)
    engine._log.info(f"{LOG_MARK} правка счёта: звучащий {sid[:8]} режу на границе числа, продолжение - по "
                     f"spoken_chars, форма из правки turn={turn.turn_id}")
    return replace(cmd, speech=None)


def continue_count(engine: "TurnEngine", speech_id: str, said: Optional[Tuple[str, str]], spoken_chars: int) -> None:
    """Счёт, который правили (:func:`count_edit`), отзвучал (``cancelled`` на границе числа или ``finished``):
    продолжение - числа, что не прозвучали по ``spoken_chars``, в новой форме. Всё прозвучало - молчим: счёт не
    повторяется."""
    edit = engine._count_edits.pop(speech_id, None)
    if edit is None or said is None:
        return
    turn, told, voice, emotion = edit
    text = counting.continued(said[1], spoken_chars, told)
    engine._log.info(f"{LOG_MARK} правка счёта: прозвучало {spoken_chars} из {len(said[1])} знаков, продолжение "
                     f"{text[:80]!r} turn={turn.turn_id}")
    engine._say(turn, text, voice, emotion)


def considered(engine: "TurnEngine", turn: "_Turn", cmd: Command, last: bool) -> Tuple[Command, FrozenSet[str]]:
    """Команда к исполнению и имена, которыми нельзя звать собеседника (:func:`foreign_keys`): без повторов
    исполненного (:func:`unrepeated`) и прочитанного (:func:`unreread`) в ходе; счёт, урезанный моделью, - числами
    кода (:func:`counted_in_full`); без чужого имени и без утверждения о
    состоянии без данных, когда вернуть команду модели уже нельзя (:func:`unnamed_without_retry`,
    :func:`unclaimed_without_retry`)."""
    log, snapshot = engine._log, pickup_parties.own_snapshot(engine.session.snapshot, turn.speaker)
    cmd = unconfirmed_by_model(log, turn, cmd)
    cmd = unasked_aloud(log, turn, snapshot.agent, cmd)
    fresh = counted_in_full(log, turn, unreread(log, turn, unrepeated(log, turn, cmd)))
    fresh = switched_theme(log, snapshot, turn, fresh)  # просьба сменить тему сета - действием кода (#4347)
    names = foreign_keys(engine.log.window(exclude=turn.turn_id), turn, fresh, snapshot)
    fresh = unnamed_without_retry(log, turn, fresh, names, last)
    return unclaimed_without_retry(log, turn, fresh, snapshot.agent, last), names


def unconfirmed_by_model(log: Any, turn: "_Turn", cmd: Command) -> Command:
    """Вызовы МОДЕЛИ тулов с вопросом (``knowledge.TOOL_QUESTIONS``) идут без ключей подтверждения
    (``ToolQuestion.confirm``): «да» человека не заменить словом модели, тул обязан спросить. Подтверждённый вызов
    приходит только через ``tool_ask`` (``Asked.confirmed()``, код) и сюда не попадает. Решает таблица, не имена."""
    confirm_keys: dict = {}
    for (tool, _), question in kn.TOOL_QUESTIONS.items():
        confirm_keys.setdefault(tool, set()).update(question.confirm)
    fixed, changed = [], False
    for call in cmd.actions:
        keys = confirm_keys.get(call.tool, set()) & set(call.args)
        if keys:
            log.warning(f"{LOG_MARK} confirm от модели снят: {call.tool} {sorted(keys)} turn={turn.turn_id}")
            call = replace(call, args=MappingProxyType({k: v for k, v in call.args.items() if k not in keys}))
            changed = True
        fixed.append(call)
    return replace(cmd, actions=tuple(fixed)) if changed else cmd


def aloud_asked(text: str, wake_words: Optional[Sequence[str]] = None) -> bool:
    """Человек явно просил озвучить («скажи вслух», «озвучь», «произнеси», «через робота»): ``tools_view.robot_asked``
    плюс :data:`knowledge.CHAT_ALOUD_EXTRA_FORMS`. Решает слова человека, не модель."""
    return robot_asked(text, wake_words) or has_form(text or "", kn.CHAT_ALOUD_EXTRA_FORMS)


def unasked_aloud(log: Any, turn: "_Turn", agent: str, cmd: Command) -> Command:
    """Чат Telegram: ``say_aloud`` исполняется только при явной просьбе озвучить (:func:`aloud_asked`, #4505): ответ
    в чат - текстом, динамики робота молчат. Иначе код отбрасывает действие и пишет причину в лог. Подхват бэклога и
    другие агенты не затрагиваются."""
    if agent != kn.TEXT_CHANNEL_AGENT or turn.pickup or aloud_asked(turn.text, turn.wake_words):
        return cmd
    dropped = tuple(a for a in cmd.actions if a.tool == kn.ALOUD_TOOL)
    if not dropped:
        return cmd
    log.info(f"{LOG_MARK} {kn.ALOUD_TOOL} отброшен: человек не просил озвучить (ответ - текстом в чат) "
             f"turn={turn.turn_id}")
    return replace(cmd, actions=tuple(a for a in cmd.actions if a not in dropped))


def carried_groups(called: Sequence[str]) -> Tuple[str, ...]:
    """Группы тулов, что переходят на следующий ход: ``faces`` - если в этом ходе звали её тулы. «Ну пришли их», «а
    ещё?» слов группы не содержат, а продолжают разговор про лица (#4505); решает код по вызванным тулам, не слова."""
    return (kn.FACES_GROUP,) if set(called) & set(kn.TOOL_GROUPS[kn.FACES_GROUP].tools) else ()


def switched_theme(log: Any, snapshot: SessionSnapshot, turn: "_Turn", cmd: Command) -> Command:
    """Просьба сменить тему играющего сета, которую команда не исполнила, - действием кода (``theme_switch``, #4347:
    «заявку принял: рок будет» без ``dj_set``)."""
    fresh, done = theme_switch.with_switch(snapshot.top("DJ") is not None, turn.text, cmd, turn.acted)
    if done:
        log.warning(f"{LOG_MARK} смена темы сета без тула ({done}): слова модели не звучат, "
                    f"{[a.tool for a in fresh.actions]} turn={turn.turn_id}")
    return fresh


def owed_now(snapshot: SessionSnapshot, turn: "_Turn", cmd: Command, fresh: Command, names: Collection[str],
             last: bool) -> str:
    """Причина вернуть ответ модели (:data:`_OWED`) или «»: повтор прочитанного (:func:`reread_reason`), иначе - ответ
    не тот, что ждут (:func:`owed_reason`); модель только повторила исполненное (:func:`settled`) - говорит код."""
    reread = reread_reason(turn, cmd, fresh, last)
    if reread or settled(cmd, fresh):
        return reread
    return owed_reason(snapshot, turn, fresh, names)


def read_calls(actions: Sequence[ToolCall], results: Sequence[ActionResult]) -> Tuple[ToolCall, ...]:
    """Вызовы, исполненные чтением (``query``): повтор в том же ходе не исполняется (:func:`unreread`)."""
    return tuple(call for call, result in zip(actions, results) if result.cls == _QUERY)


def settled(cmd: Command, fresh: Command) -> bool:
    """Модель повторила только исполненное в этом ходе (:func:`unrepeated` убрал все её действия) и слов не дала: ответа
    сверх сделанного нет. Ход закрывается сразу фразой кода по исходу (:func:`_close_unread`), модель не зовётся снова
    и нарушения ``silent_answer`` ей не возвращают: повтор с лицом остаётся «одним лицом без слов» (``bare_answer``), и
    ход крутил повтор по кругу, пока не звучала заглушка «Ага, я на связи.» (n608 марафон-6, #4173)."""
    return bool(cmd.actions) and not fresh.actions and not fresh.speech and fresh.ask is None and fresh.perform is None


def unanswered(cmd: Command, fresh: Command, runnable: Sequence[ToolCall]) -> bool:
    """Модель не дала ответа сверх уже сделанного: на последней итерации ещё просит ``query`` (он не исполняется,
    :func:`_runnable`) или все её действия - повтор исполненного в этом ходе (:func:`unrepeated`). Слов её при этом не
    слышно - говорит код по данным хода (:func:`_close_unread`)."""
    return len(runnable) < len(fresh.actions) or (bool(cmd.actions) and not fresh.actions)


def _offered(tools: Sequence[Any], last: bool) -> Tuple[Any, ...]:
    """Тулы вызова LLM хода. На последнем ``query`` убраны из схемы: данные ему уже вернули (живой прогон
    23.10: модель искала память трижды, потолок ``LLM_QUERY_ITERATIONS_MAX`` съел последний вызов, и ход молчал).
    Выбор сужает структура — читать больше нечем, остаётся сказать итог или сделать действие."""
    return tuple(t for t in tools if not last or kn.TOOL_CLASSES.get(t.name) != _QUERY)


def answer_form(form: Form, turn: "_Turn", last: bool, snapshot: SessionSnapshot) -> Form:
    """Форма вызова модели одной итерации (ADR-0148 п.2: валидатор с понятной ошибкой, а не правило в промпте).
    Последняя итерация после выполненного запроса: данные у модели, читать больше нечем (:func:`_offered` убрал
    query-тулы) - ответ без слов нарушает ``VIOLATION_SPEECH_REQUIRED``, и ход даёт модели повторы за словами
    (:func:`_push_for_words`; живой 28.10 13:40: «скажи обо мне пять фактов» - два поиска по памяти, а ответ
    последнего вызова - одно лицо; марафон-2 n109, n908).
    Слов ждут те же, кому их должен :func:`words_owed` (:func:`speaks_aloud`: динамики, не тишина; подхват бэклога
    голым «робот» - тоже), лицо без слов человек не просил (``tools_view.show_asked``). Путь пения решён кодом
    (``perform`` или тул пути обязательны) - не трогаем."""
    owed = (last and bool(turn.queried) and not form.sing and speaks_aloud(snapshot) and not show_asked(turn.text))
    return replace(form, speech_required=owed)


async def _ask_iteration(engine: "TurnEngine", turn: _Turn, messages: Sequence[Any], tools: Sequence[Any],
                         last: bool) -> "Answer":
    """Один вызов модели хода в форме ЭТОЙ итерации (:func:`answer_form`): её видят схема и валидатор моста;
    после вызова снова форма хода."""
    turn.waiting = "llm"
    with turn_form(answer_form(current_form(), turn, last, engine.session.snapshot)):
        return await turn.deadline.wait(engine._ports.ask(messages, _offered(tools, last), turn.deadline.left()), "llm")


def _push_for_words(engine: "TurnEngine", turn: _Turn) -> Optional[str]:
    """Последняя итерация после запроса, а модель без слов (``VIOLATION_SPEECH_REQUIRED``; решение Шифу 28.10 по PR
    «модель не должна молчать… пнуть модель»): данные хода уже в её окне - ещё один вызов с подсказкой нарушения
    (``command.HINTS``: ответь словами по этим данным, коротко), не больше ``SPEECH_PUSHES_MAX`` на ход и только если до
    потолка хода осталось ``SPEECH_PUSH_MIN_LEFT_S``. Повторов нет (лимит, время, форма итерации слов не ждёт) -
    ``None``: тогда говорит код по данным (:func:`query_fallback`) как последняя страховка."""
    owed = answer_form(current_form(), turn, True, engine.session.snapshot).speech_required
    if not owed or turn.pushed >= kn.SPEECH_PUSHES_MAX or turn.deadline.left() < kn.SPEECH_PUSH_MIN_LEFT_S:
        return None
    turn.pushed += 1
    engine._log.warning(f"{LOG_MARK} {kn.VIOLATION_SPEECH_REQUIRED}: последняя итерация без слов → повтор за словами "
                        f"{turn.pushed}/{kn.SPEECH_PUSHES_MAX} (осталось {turn.deadline.left():.1f} с) "
                        f"turn={turn.turn_id}")
    return explain(Invalid(kn.VIOLATION_SPEECH_REQUIRED), after_query=True)


#: Нарушения последней итерации «ответа нет, слов нет»: за словами ход повторяет вызов (:func:`_push_for_words`) -
#: от валидатора моста (форма итерации) и от хода (голый ответ, когда мост команду не судит).
_WORDS_MISSED = (kn.VIOLATION_SPEECH_REQUIRED, kn.VIOLATION_SILENT_ANSWER, kn.VIOLATION_SOUND_ANSWER)


def _missed_words(engine: "TurnEngine", turn: _Turn, reason: str) -> Optional[str]:
    """Последняя итерация, а ответ без слов (:data:`_WORDS_MISSED`): подсказка повтора за словами
    (:func:`_push_for_words`) или ``None`` - повторов больше нет. Модель молчала и после повторов - строка нарушения
    в лог (страховка кода по данным видна в марафоне)."""
    if reason not in _WORDS_MISSED:
        return None
    more = _push_for_words(engine, turn)
    if more is None and turn.pushed:
        engine._log.warning(f"{LOG_MARK} {kn.VIOLATION_SPEECH_REQUIRED}: без слов и после {turn.pushed} повторов → "
                            f"страховка: фраза кода по данным turn={turn.turn_id}")
    return more


def unvoiced(said: Optional[str], results: Sequence[ActionResult]) -> Tuple[ActionResult, ...]:
    """Исходы действий Tier-1, о которых код не сказал (стоп и тишина молчат, :func:`tier1_phrase`): их скажет
    :func:`_close_unread`, если модель на остаток фразы только повторит исполненное (марафон-4 n608)."""
    return () if said else tuple(r for r in results if r.cls != _QUERY)


def _close_unread(turn: _Turn, results: Sequence[ActionResult]) -> Tuple[str, Optional[str]]:
    """Модель на последней итерации ещё просит ``query`` (его уже не исполняют, :func:`_runnable`) или только
    повторяет исполненное в ходе (:func:`unrepeated`), а её слов не слышно: ответа она не дала
    (:func:`unanswered`). ``(фраза кода, эмоция)``. Запрос в ходе был - ход ``done``, говорит код по данным запроса
    (В21: на адресованное не молчим: ``[play_animation, music_library list]`` - анимация исполнена, фраза кода по
    данным). Запроса не было: действия хода исполнены (``results`` этой команды или ``_Turn.acted`` раньше в ходе) -
    ход сделал, что просили; об исходе, который ещё никто не сказал (``_Turn.unvoiced``: Tier-1 стоп молчит, а модель
    на остаток фразы повторила тот же стоп - марафон-4 n608), говорит код по исходу (ADR-0148), иначе он уже сказан.
    Исполнять было нечего - провал без речи."""
    if turn.queried:
        turn.stage, turn.status, turn.reason = "done", "done", "query_fallback:ceiling"
        return data_close(turn, "sad")
    if not results and not turn.acted:
        turn.stage, turn.status, turn.reason = "failed", "failed", "nothing_to_do"
        return "", None
    unvoiced, turn.unvoiced = turn.unvoiced, ()
    said = join(unvoiced)
    return (said.text, said.emotion) if said is not None else ("", None)


def data_close(turn: _Turn, emotion: Optional[str]) -> Tuple[str, Optional[str]]:
    """Фраза кода, которой закрывается ход с выполненным ``query``, когда слов модели нет (#4157, класс К2):
    ``(фраза, эмоция)``. Сначала исход действий хода, о котором ещё никто не сказал (``_Turn.unvoiced``: Tier-1 стоп
    молчит; М7 n608 - стоп сета, потом ``get_music_state``, потом повтор стопа: звучала заглушка «ответ не сложился»
    вместо «Выключил музыку.»), затем ответ по данным запроса (:func:`query_fallback`). Исход сказан, а в данных
    прочитанного слов нет - звучит только исход: общая заглушка рядом с ним не нужна. ``emotion`` - эмоция, если
    нет ни исхода, ни ответа по данным."""
    unvoiced, turn.unvoiced = turn.unvoiced, ()
    said = join(unvoiced)
    read = any(r.status == "done" for r in turn.queried)
    voiced = said is not None or turn.stop_said  # исход стопа уже сказал Tier-1 (``_Turn.stop_said``)
    tail = "" if voiced and read and query_answer(turn.queried) is None else query_fallback(turn.queried)
    text = " ".join(part for part in (said.text if said is not None else "", tail) if part)
    return text, said.emotion if said is not None else query_emotion(turn.queried, emotion)


def read_refusal(refused: Sequence[ActionResult]) -> Optional[Utterance]:
    """Фраза кода об отказах чтения памяти (:func:`memory.voiced_refusals`): одна на причину (``memory_context`` и
    ``memory_search`` спорным голосом - одна «не буду гадать, что помню»); отказов нет - ``None``."""
    return join(list({r.reason: r for r in refused}.values()))


def query_more(engine: "TurnEngine", turn: _Turn, results: Sequence[ActionResult],
               emotion: Optional[str] = None) -> Optional[str]:
    """Исходы команды с ``query`` (итерации ещё есть): данные ``query`` - модели, строкой для ``Answer.follow``, она
    отвечает по ним (ADR-0150 §4.1 query-цикл, §5.1). Действия той же команды (``[play_animation,
    music_library list]``: спасённые) уже исполнены: об их исходе говорит код сразу (:func:`outcome_phrase`,
    провал - грустно, иначе ``emotion``), модели они не уходят - ``action.*`` терминальны, слов о них она не получает.

    Отказ чтения памяти до владельца (собеседника нет или голос не подтверждён) на вопрос о памяти о самом человеке
    (:func:`memory.voiced_refusals`) - не данные: о нём говорит код фразой ``query.rejected.<причина>`` первой в той
    же реплике («что ты обо мне помнишь?» спорным голосом - «не буду гадать, что помню», а не «запоминать
    не буду»: и рядом с ``play_sound``); читать модели больше нечего - ``None``, ход закрыт фразой кода.
    На любую другую реплику отказ - данные модели, как раньше, и ход продолжается («кто такой
    Гагарин?» неузнанным голосом - модель отвечает про Гагарина)."""
    refused = memory.voiced_refusals(results, turn.text)
    refusal = read_refusal(refused)
    acted = [r for r in results if r.cls != _QUERY]
    if refusal is not None:
        engine._log.info(f"{LOG_MARK} отказ чтения памяти {[(r.tool, r.reason) for r in refused]} на вопрос о памяти "
                         f"о собеседнике → фраза кода turn={turn.turn_id}")
    said, sad = _before_data(refusal, acted)
    engine._say(turn, said, None, "sad" if sad else emotion)
    readable = tuple(r for r in results if r.cls == _QUERY and r not in refused)
    if not readable:
        return None
    if acted:
        engine._log.info(f"{LOG_MARK} query {[r.tool for r in readable]} рядом с действиями "
                         f"{[(r.tool, r.status) for r in acted]} → данные модели, итог действий сказал код "
                         f"turn={turn.turn_id}")
    turn.queried += readable  # всё прочитанное в ходе: фраза кода по данным видит и прошлые итерации
    return _query_data(readable)


def _before_data(refusal: Optional[Utterance], acted: Sequence[ActionResult]) -> Tuple[str, bool]:
    """Реплика кода, пока данные ``query`` идут модели (:func:`query_more`): отказ чтения памяти первым, затем исход
    исполненных действий команды (:func:`outcome_phrase`); ``True`` - грустно (отказ или провал действия)."""
    parts = (refusal.text if refusal is not None else None, outcome_phrase(acted))
    return " ".join(part for part in parts if part), refusal is not None or any(failed(r) for r in acted)


def _query_data(results: Sequence[ActionResult]) -> str:
    rows = [{"tool": r.tool, "status": r.status, "reason": r.reason, "data": dict(r.data)} for r in results]
    return json.dumps({kn.QUERY_RESULTS_FIELD: rows}, ensure_ascii=False, default=str)


def _voices_summary(data: Mapping[str, Any]) -> Optional[str]:
    """«Яндекс — 13, MiniMax — 10» из данных ``list_tts_voices`` (обзор по провайдерам или голоса одного)."""
    overview = data.get("providers")
    if isinstance(overview, list):
        pairs = [(str(p.get("label") or p.get("provider")), p.get("voices_count")) for p in overview
                 if isinstance(p, Mapping)]
    elif isinstance(data.get("voices"), list):
        pairs = [(str(data.get("provider") or ""), len(data["voices"]))]
    else:
        return None
    return ", ".join(f"{name} — {count}" for name, count in pairs if name and isinstance(count, int)) or None


def _voices_slots(data: Mapping[str, Any]) -> Optional[Mapping[str, Any]]:
    summary = _voices_summary(data)
    return {"summary": summary} if summary else None


def _quoted_names(rows: Any, field: str = "title") -> List[str]:
    """Первые ``LIBRARY_SUMMARY_NAMES`` разных значений ``field`` строк в кавычках (варианты одного заказа - одно
    имя)."""
    values = (" ".join(str(row.get(field) or "").split()) for row in rows or () if isinstance(row, Mapping))
    return [f"«{value}»" for value in dict.fromkeys(values) if value][:kn.LIBRARY_SUMMARY_NAMES]


def _library_slots(data: Mapping[str, Any]) -> Optional[Mapping[str, Any]]:
    """Слоты фразы ``music_library`` по действию вызова (``knowledge.QUERY_FALLBACK_PHRASES``): ``list``/``search`` -
    сколько треков (``total`` тула: у ``list`` - вся библиотека, а не выборка) и первые ``LIBRARY_SUMMARY_NAMES``
    названий; пусто - слоты есть (фраза - о пустом); ``info`` - название и известные поля карточки
    (:func:`_info_slots`). Нет числа или треки без названий - ``None``: фразы по данным нет."""
    action, total = data.get("action"), data.get("total")
    if action == "info":
        return _info_slots(data)
    if action not in ("list", "search") or not _is_count(total):
        return None
    names = _quoted_names(data.get("tracks"))
    if not names and total > 0:
        return None
    return {"total": total, "names": ", ".join(names), "query": str(data.get("query") or "").strip()}


def _info_slots(data: Mapping[str, Any]) -> Optional[Mapping[str, Any]]:
    """Карточка трека (``music_library info``, n707): название и непустые поля ``LIBRARY_INFO_FIELDS`` (теги - списком
    или строкой). Без названия или без полей - ``None``: говорит общий разбор данных (:func:`_names_slots`)."""
    title = " ".join(str(data.get("title") or "").split())
    parts: List[str] = []
    for name in kn.LIBRARY_INFO_FIELDS:
        value = data.get(name)
        items = value if isinstance(value, (list, tuple)) else [value]
        parts += [text for text in (" ".join(str(v or "").split()) for v in items) if text and text not in parts]
    return {"title": title, "details": ", ".join(parts)} if title and parts else None


def _names_slots(data: Mapping[str, Any]) -> Optional[Mapping[str, Any]]:
    """Общий разбор данных любого ``query`` без своей фразы и без фразы тула: названия (``QUERY_NAME_FIELDS``) из
    первого списка строк данных, иначе название самого результата. Названий нет - ``None``."""
    for name_field in kn.QUERY_NAME_FIELDS:
        for value in data.values():
            names = _quoted_names(value, name_field) if isinstance(value, (list, tuple)) else []
            if names:
                return {"names": ", ".join(names)}
    names = _quoted_names([data], next((f for f in kn.QUERY_NAME_FIELDS if data.get(f)), ""))
    return {"names": names[0]} if names else None


def _is_count(value: Any) -> bool:
    return isinstance(value, int) and not isinstance(value, bool)


def _facts_slots(texts: Iterable[Any]) -> Optional[Mapping[str, Any]]:
    """Слоты фразы по данным памяти: первые ``MEMORY_SUMMARY_FACTS`` разных фактов как есть, через «;» (точка в
    конце факта во фразе лишняя). Фактов нет - ``None``: фразы по данным нет."""
    cleaned = (" ".join(str(text or "").split()).rstrip(".;").strip() for text in texts)
    facts = list(dict.fromkeys(fact for fact in cleaned if fact))[:kn.MEMORY_SUMMARY_FACTS]
    return {"facts": "; ".join(facts)} if facts else None


def _score(row: Mapping[str, Any]) -> float:
    score = row.get("score")
    return float(score) if isinstance(score, (int, float)) and not isinstance(score, bool) else 0.0


def _search_slots(data: Mapping[str, Any]) -> Optional[Mapping[str, Any]]:
    """``memory_search``: записи ``kind=fact`` по убыванию ``score``; реплики прошлых разговоров (``kind=turn``) - не
    факты о человеке, их фраза не называет."""
    rows = [r for r in data.get("results") or () if isinstance(r, Mapping) and r.get("kind") == "fact"]
    return _facts_slots(r.get("content") for r in sorted(rows, key=_score, reverse=True))


def _context_slots(data: Mapping[str, Any]) -> Optional[Mapping[str, Any]]:
    """``memory_context``: ``facts`` - факты о собеседнике (``{fact, category, …}``) в порядке владельца памяти."""
    return _facts_slots(f.get("fact") for f in data.get("facts") or () if isinstance(f, Mapping))


def _sounds_slots(data: Mapping[str, Any]) -> Optional[Mapping[str, Any]]:
    """Слоты фразы каталога звуков (``get_sound_info`` без аргументов): сколько звуков всего и по каким
    категориям (``knowledge.SOUND_CATEGORY_NAMES``; звуки прочих категорий - «другие»). Список одной категории или один
    звук - слов по данным нет (``None``)."""
    total, counts = data.get("total_sounds"), data.get("categories")
    if not _is_count(total) or total <= 0 or not isinstance(counts, Mapping):
        return None
    named = [(label, counts.get(key)) for key, label in kn.SOUND_CATEGORY_NAMES.items()]
    named.append((kn.SOUND_OTHER_NAME, total - sum(n for _, n in named if _is_count(n))))
    names = ", ".join(f"{label} — {n}" for label, n in named if _is_count(n) and n > 0)
    return {"total": total, "names": names} if names else None


def _web_slots(data: Mapping[str, Any]) -> Optional[Mapping[str, Any]]:
    """``search_web`` (n908): текст (``body``) первой строки выдачи с текстом, без хвостового многоточия обрезки
    тула. Выдача пуста - ``None``: фразы по данным нет."""
    bodies = (" ".join(str(r.get("body") or "").split()) for r in data.get("results") or () if isinstance(r, Mapping))
    text = next((body.rstrip(".…;: ") for body in bodies if body.rstrip(".…;: ")), "")
    return {"text": text} if text else None


def _uptime(seconds: float) -> str:
    """Время работы словами: «45 с», «17 мин», «3 ч»."""
    if seconds < 60:
        return f"{round(seconds)} с"
    if seconds < 3600:
        return f"{round(seconds / 60)} мин"
    return f"{round(seconds / 3600)} ч"


def _containers_slots(data: Mapping[str, Any]) -> Optional[Mapping[str, Any]]:
    """``container_status`` (#4295): сколько контейнеров в отчёте и разброс ``uptime_sec`` (рестарты и нагрузка
    бывают пустыми - не называются). Контейнеров или времени работы в данных нет - ``None``: говорит общий разбор."""
    rows = [r for r in data.get("containers") or () if isinstance(r, Mapping)]
    ups = [float(r["uptime_sec"]) for r in rows
           if isinstance(r.get("uptime_sec"), (int, float)) and not isinstance(r.get("uptime_sec"), bool)]
    if not rows or not ups:
        return None
    return {"total": len(rows), "shortest": _uptime(min(ups)), "longest": _uptime(max(ups))}


#: Слоты фразы кода по данным ``query`` - по тулу таблицы ``knowledge.QUERY_FALLBACK_PHRASES`` (ключи совпадают).
_QUERY_SLOTS: Mapping[str, Callable[[Mapping[str, Any]], Optional[Mapping[str, Any]]]] = {
    "list_tts_voices": _voices_slots, kn.LIBRARY_TOOL: _library_slots, "memory_search": _search_slots,
    "memory_context": _context_slots, "get_sound_info": _sounds_slots, "search_web": _web_slots,
    "container_status": _containers_slots}


def _answer_sources(results: Sequence[ActionResult]) -> List[ActionResult]:
    """Выполненные ``query`` хода, по данным которых код может ответить: память о собеседнике
    (``knowledge.MEMORY_FACTS_TOOLS``) - только когда ход больше ничего не читал (n908: память рядом с ``search_web`` -
    контекст, а не ответ на «рецепт борща»)."""
    done = [r for r in results if r.status == "done"]
    others = [r for r in done if r.tool not in kn.MEMORY_FACTS_TOOLS]
    return others or done


def _data_phrase(r: ActionResult) -> Optional[str]:
    """Фраза кода по данным одного результата: своя фраза тула из таблицы (``knowledge.QUERY_FALLBACK_PHRASES``,
    ключ - по данным вызова), иначе фраза, которую построил сам тул (``knowledge.TOOL_MESSAGE_FIELD``), иначе
    названия из данных (``knowledge.QUERY_NAMES_PHRASE``). Слов в данных нет - ``None``."""
    entry = kn.QUERY_FALLBACK_PHRASES.get(r.tool)
    key = (entry.key(r.data) if isinstance(entry, (kn.ByData, kn.ByAction)) else entry) if entry is not None else None
    slots = _QUERY_SLOTS[r.tool](r.data) if key is not None else None
    if slots is not None:
        return phrases.text(key, **slots)
    said = " ".join(str(r.data.get(kn.TOOL_MESSAGE_FIELD) or "").split())
    if said:
        return said
    names = _names_slots(r.data)
    return phrases.text(kn.QUERY_NAMES_PHRASE, **names) if names is not None else None


def query_answer(results: Sequence[ActionResult]) -> Optional[str]:
    """Ответ кода по данным выполненного ``query`` (:func:`_data_phrase`), начиная с ПОСЛЕДНЕГО результата хода
    (решение Шифу 28.10, #4157: «код отвечает по данным последнего результата»; n707 - три поиска подряд, ответ - по
    последнему): голоса - сводка по провайдерам, библиотека - число треков и 2–3 названия или карточка трека, память -
    2–3 факта (только если ход читал одну её, :func:`_answer_sources`), каталог звуков - сколько и каких категорий,
    поиск в сети - текст первой строки выдачи; у тула без своей фразы - фраза самого тула, иначе названия из данных.
    Одно место слотов: что модель прочла и не сказала, называет код. Ответ не длиннее речи, которую слышат
    (``command.fit_speech``). Слов в данных нет - ``None``."""
    for r in reversed(_answer_sources(results)):
        text = _data_phrase(r)
        if text:
            return fit_speech(Command(speech=text)).speech
    return None


def query_emotion(results: Sequence[ActionResult], emotion: Optional[str]) -> Optional[str]:
    """Эмоция фразы кода по данным хода (:func:`query_fallback`): ответ по данным (:func:`query_answer`) - без грусти
    (это ответ, а не провал: лицо итога по фразе); иначе ``emotion`` (общая «не смог сложить», отказ)."""
    return None if query_answer(results) else emotion


async def _close_invalid(engine: "TurnEngine", turn: _Turn, invalid: Invalid, snap: SessionSnapshot,
                         expression: Optional[Expression] = None, last: bool = True) -> Optional[str]:
    """Невалидная команда после повтора моста: на адресованную реплику не молчим (В21, решение Шифу 23.10).
    Запрос в ходе выполнен - говорит код по его данным. Иначе допустимые действия команды (``Invalid.salvage``)
    исполняются, об исходе говорит код (:func:`_salvaged_phrase`); исполнять нечего - честная фраза
    ``INVALID_COMMAND_PHRASE``. Ход ``done``. Среди спасённых есть ``query`` (класс ВЫЗОВА: ``music_library list``) и итерации ещё есть (не ``last``) - его данные возвращаются модели тем же путём, что у исполненной команды
    (:func:`query_more`), и она отвечает словами; итераций нет - фраза кода по данным (:func:`query_fallback`). Все
    спасённые ``query`` - озвученные отказы чтения памяти (``memory.voiced_refusals``): фразу сказал :func:`query_more`,
    читать модели нечего - ход ``done`` здесь же (итог ``addressed`` давал рефлекс «услышал»).
    ``expression`` - лицо команды, исполняемой после нарушения ``silent_answer``: красит итог вместе с её
    действиями, если они не провалились (:func:`command_face`)."""
    log, why = engine._log, invalid.reason
    pushed = _missed_words(engine, turn, why) if last else None
    if pushed is not None:
        return pushed
    if turn.queried:
        log.warning(f"{LOG_MARK} невалидная команда {why} после query → фраза кода по данным запроса")
        turn.stage, turn.status, turn.reason = "done", "done", f"query_fallback:{why}"[:60]
        said, emotion = data_close(turn, invalid_emotion(invalid, ()))
        engine._say(turn, said, None, emotion)
        return None
    invalid = replace(invalid, salvage=theme_switch.salvage_with_switch(
        engine.session.snapshot.top("DJ") is not None, turn.text, invalid.salvage, turn.acted))
    log.warning(f"{LOG_MARK} невалидная команда {why} → допустимых действий {len(invalid.salvage)}, итог скажет код")
    if action_chain.runs_as_chain(invalid.salvage):  # повтор не помог - вереница с планом кода (ADR-0165 §3)
        start_chain(engine, turn, Command(expression=expression or Expression()), invalid.salvage, snap)
        turn.reason = f"chain_salvaged:{why}"[:60]
        return None
    results = await engine._execute(turn, invalid.salvage, snap)
    if engine._stale(turn, "results"):
        return None
    turn.expression = command_face(expression, results, turn.text, earlier=turn.expression)
    read = not last and any(r.cls == _QUERY for r in results)
    more = query_more(engine, turn, results) if read else None
    if more is not None:
        log.warning(f"{LOG_MARK} невалидная команда {why}: спасённый query {[r.tool for r in turn.queried]} → данные "
                    f"модели, ещё итерация turn={turn.turn_id}")
        return more
    turn.stage, turn.status, turn.reason = "done", "done", f"invalid_salvaged:{why}"[:60]
    if not read:  # прочитанное, но не отданное модели - озвученный отказ памяти: его фразу уже сказал query_more
        engine._say(turn, _salvaged_phrase(engine, turn, invalid, results), None, invalid_emotion(invalid, results))
    return None


def invalid_emotion(invalid: Invalid, results: Sequence[ActionResult]) -> Optional[str]:
    """Эмоция фразы кода после нарушения: провал исполненного — грустно; исполнять было нечего — грустно («не понял»),
    кроме нарушений, где ничего не провалилось (``knowledge.CALM_VIOLATIONS``: ответ лицом, анимацией или earcon без
    слов — «Угу, я тут.» звучало с ``sad``)."""
    emotion = outcome_emotion(results)  # провал - sad, вопрос тула - эмоция вопроса
    if emotion is not None:
        return emotion
    return None if results or invalid.reason in kn.CALM_VIOLATIONS else "sad"


def _salvaged_phrase(engine: "TurnEngine", turn: _Turn, invalid: Invalid, results: Sequence[ActionResult]) -> str:
    """Фраза кода после невалидной команды: об исходе исполненного; успех звука молчит (звук сам себе доказательство) -
    тишина честна. Спасённый ``query`` на последней итерации - его данные читать некому, говорит код по ним
    (:func:`query_fallback`: «что у тебя в библиотеке?» не молчит). Исполнять было нечего или исполненного не
    слышно, а человек ждёт слов (:func:`words_owed`: голая анимация) - фраза причины
    (``INVALID_COMMAND_PHRASES``), не молчание."""
    said = outcome_phrase(results) if results else None
    queried = [r for r in results if r.cls == _QUERY]
    if queried:
        return " ".join(text for text in (said, query_fallback(queried)) if text)
    snap = engine.session.snapshot
    if said or (results and not words_owed(snap, turn.text, Command(actions=tuple(invalid.salvage)))):
        return said or ""
    if invalid.reason == kn.VIOLATION_MUSIC_PROMISE:
        return engine._pick(music_promise_key(turn.music_stopped, snap.music_playing))
    if invalid.reason == kn.VIOLATION_VOICE_PROMISE:
        return engine._pick(kn.VOICE_PROMISE_PHRASE)
    if invalid.reason == kn.VIOLATION_SEARCH_PROMISE:
        return search_promise_phrase(engine, turn)
    return engine._pick(kn.INVALID_COMMAND_PHRASES.get(invalid.reason, kn.INVALID_COMMAND_PHRASE))


def search_promise_phrase(engine: "TurnEngine", turn: _Turn) -> str:
    """Фраза кода после обещания поискать без тула и после повтора (#4314, n907): поиск в этом ходе был - ответ по его
    данным (:func:`query_fallback`), а не «поищу ещё»; не было - честное «поиск я не запускал»."""
    searched = [r for r in turn.queried if r.tool in kn.SEARCH_PROMISE_TOOLS]
    return query_fallback(searched) if searched else engine._pick(kn.SEARCH_PROMISE_PHRASE)


def music_promise_key(stopped: bool, playing: bool) -> str:
    """Ключ фразы кода после обещания действия с музыкой без тула и после повтора (ADR-0148: об исходе говорит
    код по событию владельца, а не слова модели): ход сам выключил музыку (``_Turn.music_stopped``, исход ``done`` стоп-тула)
    - «Музыку я уже выключил.»; музыка не играет (снимок сессии) - «Сейчас музыка не играет.»; играет, а обещание не
    исполнено - «Сделать с музыкой ничего не получилось…». Стоп хода сильнее снимка: снимок плеера приходит позже стопа."""
    state = "stopped" if stopped else "playing" if playing else "silent"
    return kn.MUSIC_PROMISE_PHRASES[state]


def robot_owed(snapshot: SessionSnapshot, turn: "_Turn", cmd: Command) -> bool:
    """Человек в чате просит сделать это РОБОТОМ (``tools_view.robot_asked``: «покекай роботом», «через робота»,
    «вслух» — и реплика не справка: «Как управлять роботом?», «найди, что пишет народ», «расскажи про народ» — ответ в
    чат, решение Шифу 28.10), а команда на роботе ничего не
    делает — только текст в чат («Покекал: глазами поводил, повертелся» без единого
    тула; «через робота» текстом в чат вместо ``say_aloud``). Решает код по словам ЧЕЛОВЕКА и по структуре команды, а
    не по тексту ответа модели: в команде нет действий, выступления и лица, и прошлые итерации хода на роботе тоже
    ничего не исполнили (``_Turn.acted``; прочитанный ``query`` — не действие: «прочитай вслух, который час» после
    ``get_current_time`` ждёт ``say_aloud``). Только агент с текстовым каналом (чат): у Личности речь и есть голос
    робота."""
    if snapshot.agent != kn.TEXT_CHANNEL_AGENT or turn.pickup or turn.acted:
        return False
    face = cmd.expression
    if cmd.actions or cmd.perform is not None or cmd.passed or shows(face.emotion, face.gesture, face.face):
        return False
    return robot_asked(turn.text, turn.wake_words)


def owed_reason(snapshot: SessionSnapshot, turn: "_Turn", cmd: Command, names: Collection[str] = ()) -> str:
    """Ответ модели не тот, что ждёт человек, — причина нарушения (:data:`_OWED`), иначе «»: голый неслышимый ответ на
    адресованную реплику (:func:`words_owed`; и подхват бэклога), звук без слов на вопрос после чтения
    (:func:`sound_owed`), ответ о каталоге звуков/голосов без его данных (:func:`catalog_owed`), только текст в чат на
    просьбу сделать роботом (:func:`robot_owed`), обращение к неподтверждённому собеседнику чужим именем (``names`` - ключи
    из :func:`foreign_keys`), «тишина» под играющей музыкой без стопа (:func:`stop_claim_false`) или утверждение о
    состоянии подсистемы робота без данных о ней (:func:`unbacked_claims`)."""
    if words_owed(snapshot, turn.text, cmd) or read_owed(snapshot, turn, cmd):
        return kn.VIOLATION_SILENT_ANSWER
    if known_voice_registers(turn_speaker(snapshot, turn), turn.text, cmd.actions, wake_words=turn.wake_words):
        return kn.VIOLATION_KNOWN_VOICE_REGISTER
    if sound_owed(snapshot, turn, cmd):
        return kn.VIOLATION_SOUND_ANSWER
    catalog = catalog_owed(snapshot, turn, cmd)
    if catalog:
        return catalog
    if robot_owed(snapshot, turn, cmd):
        return kn.VIOLATION_REMOTE_OWED
    if foreign_name.offending(cmd, names):
        return kn.VIOLATION_FOREIGN_NAME
    if stop_claim_false(snapshot, turn, cmd):
        return kn.VIOLATION_MUSIC_PROMISE
    return kn.VIOLATION_STATE_CLAIM if unbacked_claims(snapshot.agent, turn, cmd) else ""


def turn_speaker(snapshot: SessionSnapshot, turn: "_Turn") -> Optional[SpeakerRef]:
    """Собеседник хода: при подхвате - автор фразы (``_Turn.speaker``), иначе собеседник сессии."""
    return turn.speaker if turn.speaker is not None else snapshot.speaker


def stop_claim_false(snapshot: SessionSnapshot, turn: "_Turn", cmd: Command) -> bool:
    """Речь команды утверждает, что музыку выключили или стало тихо (``words.music_stop_claimed``), а музыка играет
    (М11 n712: «Тишина. Борису больше не мешаем.» без стоп-тула, музыка играла дальше). Решает код по СОСТОЯНИЮ
    (ADR-0148): снимок плеера говорит «играет» (``SessionSnapshot.music_playing``), ход музыку не выключал
    (``_Turn.music_stopped``, стоп-тул среди исполненных ``_Turn.acted``), и в самой команде стоп-тула нет. После
    стопа и без музыки те же слова - правда, нарушения нет."""
    if not snapshot.music_playing or turn.music_stopped or not cmd.speech:
        return False
    if any(kn.stops_music(a.tool, a.args) for a in (*cmd.actions, *turn.acted)):
        return False
    return music_stop_claimed(cmd.speech)


def unbacked_claims(agent: str, turn: "_Turn", cmd: Command) -> Tuple[str, ...]:
    """Подсистемы, о состоянии которых речь команды утверждает без данных (``state_claim``): опора - то, что модель
    видела, - ``recent_actions`` снимка хода (``_Turn.seen``) и исходы ``query`` этого хода (``_Turn.queried``)."""
    return state_claim.offending(agent, cmd, state_claim.evidence((*turn.seen, *turn.queried)))


def foreign_keys(window: Sequence[Any], turn: "_Turn", cmd: Command, snapshot: SessionSnapshot) -> FrozenSet[str]:
    """Имена, которыми нельзя обращаться к собеседнику хода: собеседник неподтверждён
    (:func:`foreign_name.unconfirmed`), а имена, которые модель видела в разговоре, - авторы реплик окна, подхвата и
    фона, лица в кадре из прочитанного в ходе (:func:`foreign_name.face_names`, #4388); ключи -
    :func:`foreign_name.name_keys`. У подтверждённого собеседника чужие имена лиц тоже запрещены: его имя даёт профиль,
    не камера.
    Не считаются названные самим человеком в этой реплике и им же в ``register_speaker`` команды: «меня зовут Гриша» -
    ответ «Понял, Гриша» честен.
    Лица в кадре берутся и без снимка собеседника (:func:`foreign_name.named`: фраза без биометрии - инъекция, первая
    после рестарта): лицо в кадре не говорящий, пока голос не подтвердил имя (#4388, живой 05.11)."""
    speaker = snapshot.speaker
    seen = list(foreign_name.face_names((*turn.seen, *turn.queried)))
    if foreign_name.named(speaker):
        return foreign_name.name_keys(seen) - foreign_name.name_keys([speaker.name])
    if foreign_name.unconfirmed(speaker):
        seen += [e.speaker for e in window if e.role == USER] + [who for who, _ in turn.heard]
        seen += [str(p.get("who") or "") for p in (*turn.recent, *turn.background)]
    said = [turn.text, *(text for _who, text in turn.heard)]
    said += [str(a.args.get("name") or "") for a in cmd.actions if a.tool == "register_speaker"]
    return foreign_name.name_keys(seen) - foreign_name.name_keys(said)


@dataclass(frozen=True)
class _Owed:
    """Нарушение «ответ не тот, что ждут» (ADR-0148 п.2: валидатор с понятной ошибкой): ``hint`` — подсказка модели,
    ``what`` — что в логе, ``keeps_words`` — после повтора слова модели уходят как есть, а код говорит рядом факт
    (фраза ``INVALID_COMMAND_PHRASES``), а не вместо них; ``reads`` — после повтора код сам читает этот тул вместо
    действий команды и говорит по его данным (``_salvaged_phrase`` → ``query_fallback``)."""

    hint: str
    what: str
    keeps_words: bool = False
    reads: str = ""


#: Голый неслышимый ответ на адресованную реплику: слов модели нет — говорит код. Текст в чат на просьбу
#: сделать роботом: слова модели — её ответ, их не подменяем (промах признака на справке не теряет ответ), а факт «на роботе ничего не сделал» говорит код рядом.
_OWED: Mapping[str, _Owed] = {
    kn.VIOLATION_SILENT_ANSWER: _Owed(kn.SILENT_ANSWER_HINT, "без слов на адресованную реплику"),
    kn.VIOLATION_REMOTE_OWED: _Owed(kn.REMOTE_OWED_HINT, "только текст в чат на просьбу сделать роботом",
                                    keeps_words=True),
    kn.VIOLATION_FOREIGN_NAME: _Owed(kn.FOREIGN_NAME_HINT, "обращение к неподтверждённому собеседнику чужим именем"),
    kn.VIOLATION_KNOWN_VOICE_REGISTER: _Owed(kn.KNOWN_VOICE_REGISTER_HINT,
                                             "регистрация голоса, уже записанного под этим именем"),
    kn.VIOLATION_STATE_CLAIM: _Owed(kn.STATE_CLAIM_HINT, "утверждение о состоянии подсистемы без данных о ней"),
    kn.VIOLATION_QUERY_REPEAT: _Owed(kn.QUERY_REPEAT_HINT, "повтор чтения, уже исполненного в этом ходе"),
    kn.VIOLATION_SOUND_ANSWER: _Owed(kn.SOUND_ANSWER_HINT, "звук без слов на вопрос после чтения"),
    **{c.violation: _Owed(c.hint, c.what, reads=c.tool) for c in kn.CATALOGS.values()},
    # после повтора говорит код по состоянию (``_salvaged_phrase`` → ``music_promise_key``: «играет» - «Сделать с
    # музыкой ничего не получилось…»), ложная «Тишина» не звучит
    kn.VIOLATION_MUSIC_PROMISE: _Owed(kn.MUSIC_STOP_CLAIM_HINT, "заявлена тишина без стоп-тула, а музыка играет"),
}


def unnamed_without_retry(log: Any, turn: _Turn, cmd: Command, names: Collection[str], last: bool) -> Command:
    """Обращение по чужому имени, а вернуть команду модели нельзя (последняя итерация, повтор исчерпан): имя из речи
    вычищает код, фраза остаётся грамотной (ADR-0148). Повтор есть - команда как есть: нарушение
    :data:`kn.VIOLATION_FOREIGN_NAME` вернёт её модели (:func:`_answer_owed`)."""
    called = foreign_name.offending(cmd, names)
    if not called or retry_left(turn, last):
        return cmd
    log.warning(f"{LOG_MARK} {kn.VIOLATION_FOREIGN_NAME}: обращение {list(called)} к неподтверждённому собеседнику и "
                f"после повтора → имя вычищено из речи turn={turn.turn_id}")
    return foreign_name.unnamed(cmd, names)


def unclaimed_without_retry(log: Any, turn: _Turn, cmd: Command, agent: str, last: bool) -> Command:
    """Утверждение о состоянии подсистемы без данных (``VIOLATION_STATE_CLAIM``), а вернуть команду модели нельзя
    (последняя итерация, повтор исчерпан): код вырезает утверждение из речи и говорит рядом, о чём данных нет
    (``state_claim.unclaimed``, ADR-0148: фразу о состоянии строит код). Повтор есть - команда как есть: нарушение
    вернёт её модели (:func:`_answer_owed`)."""
    kinds = unbacked_claims(agent, turn, cmd)
    if not kinds or retry_left(turn, last):
        return cmd
    log.warning(f"{LOG_MARK} {kn.VIOLATION_STATE_CLAIM}: {list(kinds)} без данных и после повтора → утверждение "
                f"вырезано, говорит код turn={turn.turn_id}")
    return state_claim.unclaimed(cmd, kinds)


def retry_left(turn: _Turn, last: bool) -> bool:
    """Модели можно вернуть нарушение: итерация не последняя и повторов за ответ ещё не исчерпано."""
    return not last and turn.retried < kn.COMMAND_RETRIES_MAX


async def _answer_owed(engine: "TurnEngine", turn: _Turn, cmd: Command, snap: SessionSnapshot, last: bool,
                       reason: str = kn.VIOLATION_SILENT_ANSWER) -> Optional[str]:
    """Ответ не тот, что ждёт человек (ADR-0148: решает код): голый неслышимый ответ на адресованную реплику
    (``VIOLATION_SILENT_ANSWER``) или только текст в чат на просьбу сделать роботом (``VIOLATION_REMOTE_OWED``,
    :func:`robot_owed`). Повтор есть (не последняя итерация, ≤ ``COMMAND_RETRIES_MAX``) - команда не исполняется,
    модели уходит нарушение в форме ``command.explain`` (следующая итерация ``_understand``). Повтора нет: голый ответ -
    действия и лицо команды исполняются, а говорит код (:func:`_close_invalid` с этой причиной, фраза
    ``INVALID_COMMAND_PHRASES``), не молчание; текст в чат (``_Owed.keeps_words``) - слова модели уходят в чат, а рядом
    код говорит факт по исходу (на роботе ничего не исполнено), не подменяя её ответ."""
    owed = _OWED[reason]
    tools = [a.tool for a in cmd.actions] or [f"expression:{cmd.expression.emotion}"]
    if retry_left(turn, last):
        turn.retried += 1
        engine._log.warning(f"{LOG_MARK} {reason}: {tools} {owed.what} → повтор с нарушением turn={turn.turn_id}")
        return explain(Invalid(reason, owed.hint), after_query=bool(turn.queried))
    if owed.keeps_words:
        engine._log.warning(f"{LOG_MARK} {reason}: {owed.what} и после повтора → слова модели в чат, рядом факт кода "
                            f"turn={turn.turn_id}")
        turn.reason = reason
        engine._speak(turn, cmd, ())
        engine._say(turn, engine._pick(kn.INVALID_COMMAND_PHRASES[reason]))
        return None
    engine._log.warning(f"{LOG_MARK} {reason}: {tools} {owed.what} и после повтора → исполняем, говорит код "
                        f"turn={turn.turn_id}")
    # лицо команды, как и её анимация, показывается — говорит код; последняя итерация после запроса - сначала
    # повтор за словами (:func:`_push_for_words`); ответ без данных тула (``_Owed.reads``) - тул читает код сам
    salvage = (ToolCall(owed.reads),) if owed.reads else cmd.actions
    needless = known_voice_registers(turn_speaker(engine.session.snapshot, turn), turn.text, salvage,
                                     wake_words=turn.wake_words)
    salvage = tuple(a for a in salvage if all(a is not n for n in needless))  # регистрировать нечего (М13 n211)
    return await _close_invalid(engine, turn, Invalid(reason, salvage=salvage), snap, cmd.expression)


def query_fallback(results: Sequence[ActionResult]) -> str:
    """Фраза кода, когда запрос хода выполнен, а ответ модели не прошёл проверку и после повтора (решение Шифу
    23.10): по данным результата тула (:func:`query_answer`: голоса, список библиотеки), иначе общая честная
    фраза. «Нашёл ответ»
    - только когда хоть один запрос ``done``: ни одного (``rejected``/``timeout``: спасённый ``music_library list``
    отклонён) - фраза его исхода по результату (``respond.join``: «Не нашёл ответа.», «Не дождался ответа…»), а не
    «нашёл»."""
    if not any(r.status == "done" for r in results):
        failed = join(results)  # у класса query фраза есть на каждый не-done исход (rejected, timeout)
        return failed.text if failed is not None else phrases.text("query.rejected")
    return query_answer(results) or phrases.text(kn.QUERY_FALLBACK_GENERIC)


async def execute_for_speaker(ports: Ports, actions: Sequence[ToolCall], snap: SessionSnapshot,
                              now: SessionSnapshot, turn_id: str, log: Any = _LOG) -> List[ActionResult]:
    """Действия владельцам; исход ``memory_save``, который решает код, — без запроса: при смене собеседника с начала
    хода ``rejected{speaker_changed}`` (чей это факт); факт, лишь повторяющий имя ``register_speaker`` этой же
    команды (``memory.name_fact``: имя живёт в профиле голоса), - исход этой регистрации с причиной
    ``name_in_profile`` (``memory.name_outcome``): ``done`` только по ``done`` владельца голосов. Результаты — в порядке
    команды, с классом ВЫЗОВА (``respond.classed``): исполнитель клеит класс тула, а ``music_library``
    list/search/info - ``query``."""
    names = memory.name_facts(actions, snap)
    changed = {i for i, a in enumerate(actions) if i not in names and memory.speaker_changed(a, snap, now)}
    sent = [a for i, a in enumerate(actions) if i not in names and i not in changed]
    done = iter(await ports.execute(sent, snap, turn_id) if sent else [])
    results = [_by_code(turn_id, i, a, snap, "rejected", kn.SPEAKER_CHANGED_REASON) if i in changed
               else None if i in names else classed(next(done), a.args) for i, a in enumerate(actions)]
    return _settle_names(results, names, actions, snap, turn_id, log)


def _by_code(turn_id: str, i: int, call: ToolCall, snap: SessionSnapshot, status: str, reason: str) -> ActionResult:
    """Результат действия, исход которого решил код, а не владелец (:func:`execute_for_speaker`)."""
    return ActionResult(f"{turn_id}/{i}", snap.epoch, snap.agent, call.tool, kn.call_class(call.tool, call.args),
                        status, reason)


def _settle_names(results: List[Optional[ActionResult]], names: Mapping[int, int], actions: Sequence[ToolCall],
                  snap: SessionSnapshot, turn_id: str, log: Any) -> List[ActionResult]:
    """Факты-имена (``memory.name_facts``: факт → регистрация той же команды) получают исход своей регистрации с
    причиной ``name_in_profile`` (``memory.name_outcome``), когда исход регистрации известен."""
    for i, r in names.items():
        reg = results[r]
        results[i] = _by_code(turn_id, i, actions[i], snap, *memory.name_outcome(reg))
        log.info(f"{LOG_MARK} {actions[i].tool} {reg.status}{{{kn.NAME_IN_PROFILE_REASON}}}: факт повторяет имя "
                 f"{actions[r].tool}({actions[r].args.get('name')!r}) этой команды, исход - её "
                 f"({reg.status}{{{reg.reason or '-'}}}), владельцу памяти не отправлен: {dict(actions[i].args)} "
                 f"turn={turn_id}")
    return results  # type: ignore[return-value]  # все None заполнены: индексы фактов-имён - ключи ``names``


def action_term(actions: Sequence[ToolCall]) -> float:
    """Срок действий хода: дедлайн самого долгого класса + ``ACTION_GRACE_S`` - исполнитель закроет ожидание честным
    ``timeout`` раньше, чем дедлайн хода (``TurnDeadline.grant``) оборвёт его «долго думаю»."""
    return max(deadline_s(kn.call_class(a.tool, a.args)) for a in actions) + kn.ACTION_GRACE_S


def close_passed(engine: "TurnEngine", turn: _Turn, cmd: Command) -> None:
    """Модель выбрала промолчать (``passed``). Решение Шифу 22.10: на адресованную реплику (вейк, окно ответа,
    Telegram) молчать нельзя — код мычит; ``passed`` из бэклога (подхват накопленного) — тихо."""
    turn.reason = "passed"
    engine._log.info(f"{LOG_MARK} passed={cmd.passed} addressed={not turn.pickup}")
    if voices_passed(turn.pickup, engine.session.snapshot):
        engine._say(turn, engine._pick(kn.PASSED_ADDRESSED_PHRASE))


def request_ref(turn: _Turn, ref: Optional[str]) -> str:
    """``turn_id`` запросов к владельцам: свой у шага вереницы (``<ход>/c<k>``), иначе - ход."""
    return ref or turn.turn_id


async def settle(engine: "TurnEngine") -> None:
    """Дождаться всех идущих верениц движка (тесты, сухой стенд): ход закрыт после фразы плана, шаги идут дальше."""
    import asyncio  # noqa: PLC0415 — asyncio тянет socket: импорт пакета чистый (test_purity)

    while engine.chains:
        await asyncio.gather(*tuple(engine.chains), return_exceptions=True)


def start_chain(engine: "TurnEngine", turn: _Turn, cmd: Command, actions: Sequence[ToolCall],
                snap: SessionSnapshot) -> None:
    """Вереница (ADR-0165 §3): фраза плана - речь модели, если она есть и ничего не утверждает сделанным, иначе план
    кода; ход на этом закрыт (новые реплики принимаются), шаги идут задачей :func:`run_chain`. Вереница за лимитом
    своего типа задачи не начинается - фраза кода (§5)."""
    import asyncio  # noqa: PLC0415 — asyncio тянет socket: импорт пакета чистый (test_purity)

    chain = action_chain.chain_of(actions)
    turn.stage, turn.status, turn.reason = "done", "done", turn.reason or "chain"
    turn.expression = command_face(cmd.expression, (), turn.text, earlier=turn.expression)
    if action_chain.over_limit(chain):
        engine._log.warning(f"{LOG_MARK} вереница за лимитом {chain.task}: шагов {len(chain.steps)}, поездок "
                            f"{chain.trips} - не начинается turn={turn.turn_id}")
        engine._say(turn, phrases.text(kn.CHAIN_TOO_LONG_PHRASE), None, "sad")
        return
    source, text = action_chain.plan(chain, cmd.speech)
    engine._log.info(f"{LOG_MARK} вереница {chain.task}: шагов {len(chain.steps)}, оценка {chain.estimate_s:.0f} с, "
                     f"план {source} {[s.call.tool for s in chain.steps]} turn={turn.turn_id}")
    engine._say(turn, text, cmd.voice, cmd.expression.emotion)
    chain_run = ChainRun(turn)
    task = asyncio.ensure_future(run_chain(engine, chain_run, chain, snap))
    engine.chains[task] = chain_run
    task.add_done_callback(lambda done: engine.chains.pop(done, None))


@dataclass(eq=False)
class ChainRun:
    """Идущая вереница движка (ADR-0165 §6): ход её плана (его эпоха - эпоха вереницы) и эпоха стопа Tier-1, что её
    погасил (``None`` - не гасили). Эпоха сессии ушла от эпохи хода без стопа (новая сессия, сброс окна) - вереница
    тоже погашена, но итог не звучит."""

    turn: _Turn
    stop_epoch: Optional[int] = None


def steer_chains(engine: "TurnEngine", kind: str, old: int, new: int) -> None:
    """Перебивание Tier-1 (``TurnEngine._interrupt``: эпоха ``old`` → ``new``) и идущие вереницы эпохи ``old``: стоп
    (``CHAIN_STOP_KINDS``: «стой», «хватит») гасит вереницу - эпоха остаётся старой, остаток ``skipped``, итог
    звучит в эпохе стопа; «замолчи» и «хватит говорить» - нет: вереница идёт дальше в новой эпохе (решение Шифу В4,
    объезд идёт молча), её шаги и итог - в ней."""
    for chain_run in tuple(engine.chains.values()):
        if chain_run.stop_epoch is not None or chain_run.turn.epoch != old:
            continue
        if kind in kn.CHAIN_STOP_KINDS:
            chain_run.stop_epoch = new
        else:
            chain_run.turn.epoch = new
        engine._log.info(f"{LOG_MARK} вереница {'гаснет' if chain_run.stop_epoch is not None else 'идёт дальше'}: "
                         f"tier1 {kind} epoch {old}→{new} turn={chain_run.turn.turn_id}")


def _halted(engine: "TurnEngine", chain_run: ChainRun) -> bool:
    """Вереница погашена: эпоха сессии ушла от её эпохи (стоп Tier-1, новая сессия)."""
    return engine.session.snapshot.epoch != chain_run.turn.epoch


async def run_chain(engine: "TurnEngine", chain_run: ChainRun, chain: action_chain.Chain,
                    snap: SessionSnapshot) -> None:
    """Шаги вереницы по одному в порядке модели (ADR-0165 §3, §6, §10): следующий уходит только после исхода
    предыдущего; вереница погашена (:func:`_halted`) - новых шагов нет, остаток ``skipped``; провал поездки гасит
    остаток. Шаг, что шёл при стопе, засчитан по исходу владельца (``action_chain.step_status``, ``StepOutcome.cut``).
    Шаги уходят в эпохе вереницы: после «замолчи» - в новой (:func:`steer_chains`). Итог - :func:`close_chain`."""
    turn = chain_run.turn
    outcomes: List[action_chain.StepOutcome] = []
    gone = False
    for k, step in enumerate(chain.steps):
        if gone or _halted(engine, chain_run):
            outcomes.append(action_chain.StepOutcome(step, kn.CHAIN_STEP_SKIPPED))
            continue
        outcome = await _chain_step(engine, turn, step, replace(snap, epoch=turn.epoch), f"{turn.turn_id}/c{k}")
        outcomes.append(replace(outcome, cut=_halted(engine, chain_run)))
        gone = outcome.status not in ("done", "pending") and kn.CHAIN_CLASSES[step.cls].stops_rest
    close_chain(engine, chain_run, outcomes)


def close_chain(engine: "TurnEngine", chain_run: ChainRun, outcomes: Sequence[action_chain.StepOutcome]) -> None:
    """Итог вереницы - одна фраза кода (``action_chain.summary``, ADR-0165 §7). После стопа Tier-1 - «Остановился. …»
    в эпохе стопа (этап 4: ход вереницы переходит в неё, иначе владелец речи отбросил бы фразу старой эпохи); в
    тишине (В4) или после новой сессии не звучит: строка журнала и метка ``CHAIN_UNVOICED_MARK`` в окне хода (после
    новой сессии хода в окне нет - только журнал)."""
    turn, now = chain_run.turn, engine.session.snapshot.epoch
    said = action_chain.summary(outcomes, stopped=chain_run.stop_epoch is not None)
    if chain_run.stop_epoch == now:
        turn.epoch = now
    quiet = turn.epoch != now or engine.session.snapshot.top("Silence") is not None
    engine._log.info(f"{LOG_MARK} вереница итог{' (не звучит)' if quiet else ''}: "
                     f"{[(o.step.call.tool, o.status) for o in outcomes]} «{said}» turn={turn.turn_id}")
    if quiet:
        engine.log.mark(turn.turn_id, kn.CHAIN_UNVOICED_MARK.format(summary=said))
        return
    failed_any = any(o.status not in ("done", "pending") for o in outcomes)
    engine._say(turn, said, None, "sad" if failed_any else None)
    finish_stage(engine._ports, turn, engine.session.snapshot)


async def _chain_step(engine: "TurnEngine", turn: _Turn, step: "action_chain.Step", snap: SessionSnapshot,
                      ref: str) -> "action_chain.StepOutcome":
    """Один шаг вереницы через исполнитель хода (``TurnEngine._execute``: окно, события, вопрос тула) со своим ``ref``
    запроса. Срок хода шагу не мешает: шаг получает срок своего класса (``TurnDeadline.grant``)."""
    try:
        [result] = await engine._execute(turn, (step.call,), snap, ref=ref)
    except DeadlineExpired:
        return action_chain.StepOutcome(step, "timeout")
    status = action_chain.step_status(result.status, result.reason, result.data)
    told = phrase_from_result(result) if status not in ("done", "pending") else None
    return action_chain.StepOutcome(step, status, told.text if told is not None else "")


def say_late(engine: "TurnEngine", result: ActionResult) -> None:
    """Итог действия после закрытия его хода (``Executor.on_late``): фраза кода по событию владельца, если класс из
    ``LATE_SPOKEN_CLASSES`` (музыка заиграла после ``timeout`` - «Включаю музыку.»); тихие успехи и чужие классы
    молчат."""
    result = classed(result, result.data)  # аргументы вызова лежат в данных результата поверх ответа тула
    late = result.cls in kn.LATE_SPOKEN_CLASSES
    text = outcome_phrase(audible([result], False, engine.log.performing())) if late else None  # рэп не рвём
    if not text:
        return
    turn_id = result.turn_id.split("/")[0]
    engine._log.info(f"{LOG_MARK} поздний итог {result.tool} {result.status} {result.reason or '-'} turn={turn_id}")
    speech_id = engine._ports.say(turn_id, result.epoch, text, None, outcome_emotion((result,)))
    engine.log.said(turn_id, speech_id, text)
    engine._event_raw("speech_requested", turn_id, result.epoch, text, {"speech_id": speech_id, "voice": ""})


async def _tier1_then_rest(engine: "TurnEngine", turn: _Turn, command: Tier1Command, snap: SessionSnapshot,
                           parsed: str) -> None:
    """Ясный стоп с продолжением («хватит, спой другую», ``Tier1Command.rest``, §6.2): стоп — кодом, остаток фразы —
    новая реплика того же хода (Tier-1, если он ею покрыт, иначе LLM), а не потерянные слова (живой прогон 23.10,
    акт 4 n410: «ладно, хватит тянуть, теперь наоборот» — стоп, 0 вызовов LLM, тишина). ``parsed`` — текст, по
    которому сработал ``command`` (:func:`tier1_of`)."""
    await engine._tier1(turn, command, snap, parsed)
    if not command.rest:
        return
    if stops_music(command) and same_stop_rest(command.rest):  # «хватит, диджей выключай»: тот же стоп (М11 n608)
        _close_same_stop(engine, turn, command.rest)
        return
    turn.text, turn.command = command.rest, ""
    snap = engine.session.snapshot
    engine._log.info(f"{LOG_MARK} остаток после стопа: {turn.text[:80]!r} turn={turn.turn_id}")
    follow, parsed = tier1_of(turn.text, turn.command, snap, engine._voices())
    if follow is not None and follow.kind == kn.MUSIC_OFF_KIND and stops_music(command):
        follow = None  # «хватит, выключи музыку и спой»: музыку уже гасит стоп, остаток - модели, как до #4349
    if follow is not None and not follow.disputed:
        await _tier1_then_rest(engine, turn, follow, snap, parsed)
    else:
        await understand(engine, turn, snap)


def _close_same_stop(engine: "TurnEngine", turn: _Turn, rest: str) -> None:
    """Остаток после Tier-1 стопа музыки - тот же стоп (``words.same_stop_rest``: «диджей выключай», М11 n608): LLM
    не зовётся, ход закрыт кодом. Человек назвал музыку словами - об исходе стопа, что Tier-1 не сказал (стоп молчит,
    ``_Turn.unvoiced``), говорит код по результату («Выключил музыку.»), а не модель после ``get_music_state``."""
    said = outcome_phrase(turn.unvoiced) if turn.unvoiced else None
    engine._log.info(f"{LOG_MARK} остаток после стопа - тот же стоп: {rest[:80]!r}, ход закрыт кодом "
                     f"said={said!r} turn={turn.turn_id}")
    if said and engine.session.snapshot.top("Silence") is None:
        engine._say(turn, said)
        turn.unvoiced = ()


def turn_work(engine: "TurnEngine", turn: _Turn, snap: SessionSnapshot) -> Awaitable[None]:
    """Чем ответить на адресованную реплику: ответ на открытый вопрос тула (``tool_ask.reply``) и Tier-1 -
    кодом, без LLM; иначе - ход к LLM. На вопрос тула отвечает только реплика из ``tool_ask.ANSWER_SOURCES`` (вейк,
    Telegram, оператор): «да» без вейка на другой вопрос робота и подхват бэклога необратимое не подтверждают; новый
    вопрос робота или модели, тишина и стоп окно снимают (``Session``, ``knowledge.TOOL_ASK_CLOSED_BY``)."""
    reply = tool_ask.reply(engine.session, turn.text, turn.source)
    if reply is not None:
        return answer_tool(engine, turn, reply, snap)
    tier1, parsed = tier1_of(turn.text, turn.command, snap, engine._voices())
    if tier1 is not None and not tier1.disputed:
        return _tier1_then_rest(engine, turn, tier1, snap, parsed)
    return understand(engine, turn, snap)


async def answer_tool(engine: "TurnEngine", turn: _Turn, reply: tool_ask.Reply, snap: SessionSnapshot) -> None:
    """Ответ человека на вопрос тула (ADR-0148: решает код, 0 вызовов LLM). Окно снимается сразу. «Да» -
    подтверждённый вызов (``Asked.confirmed``: аргументы вопроса + ``confirm``) исполняет код, фраза об исходе - из
    РЕЗУЛЬТАТА тула (``respond.join``: «Удалил «X» из библиотеки.», отказ - своей фразой; тул спросил снова - вопрос и
    новое окно, ``TurnEngine._execute``). «Нет» - фраза ``ToolQuestion.declined`` («Не удаляю.»), тул не зовётся."""
    asked = reply.asked
    tool_ask.close(engine.session)
    turn.stage = "done"
    engine._log.info(f"{LOG_MARK} {ASK_MARK} ответ {'да' if reply.yes else 'нет'}: {tool_ask.describe(asked)} "
                     f"text={turn.text[:60]!r} turn={turn.turn_id}")
    if not reply.yes:
        turn.reason = "tool_ask:declined"
        engine._say(turn, phrases.text(asked.question.declined))
        return
    turn.reason = "tool_ask:confirmed"
    results = await engine._execute(turn, (asked.confirmed(),), snap)
    if engine._stale(turn, "results"):
        return
    emotion = outcome_emotion(results)  # тул спросил снова - эмоция вопроса, не грусть
    said = outcome_phrase(results) or phrases.text("local.rejected" if emotion == "sad" else "local.done")
    engine._say(turn, said, None, emotion)  # на ответ не молчим


def provider_in_flight(ports: Any) -> str:
    """Провайдер, к которому идёт вызов LLM сейчас (необязательный ``Ports.provider``); порт не знает — «»."""
    current = getattr(ports, "provider", None)
    return str(current() or "") if callable(current) else ""


def orders_over_music(snap: SessionSnapshot, actions: Sequence[ToolCall]) -> bool:
    """Ход заказывает новую музыку (``MUSIC_STARTING_TOOLS``, не стоп), пока музыка играет (#4313)."""
    return snap.music_playing and any(a.tool in kn.MUSIC_STARTING_TOOLS and a.args.get("action") != "stop"
                                      for a in actions)


def replacing_music(engine: "TurnEngine", turn: Any) -> bool:
    """Ход ждёт новую музыку, а прежняя ещё звучит (#4313): заказ сделан под играющую музыку и она играет сейчас.
    Факт - из снимка плеера (``SessionSnapshot.music_playing``), не из слов модели; прежняя замолчала, а новая не
    встала - честно «не заиграла»."""
    return (getattr(turn, "replacing", False) and turn.waiting == "action.media"
            and engine.session.snapshot.music_playing)


def replace_timeout_outcome(engine: "TurnEngine", turn: Any, results: Sequence[ActionResult],
                            outcome: Optional[str]) -> Optional[str]:
    """Срок класса вышел (исполнитель закрыл ``action.media`` статусом ``timeout``), а ход заказал смену под играющую
    музыку (#4479, М14 n604: «Музыка пока не заиграла.» при играющем сете): факт - из снимка плеера, не из слов
    модели; прежний сет звучит - фраза смены (``MUSIC_REPLACE_PHRASES``), иначе исход как есть."""
    timed_out = any(r.cls == "action.media" and r.status == "timeout" for r in results)
    if timed_out and replacing_music(engine, turn):
        return phrases.text(kn.MUSIC_REPLACE_PHRASES["timeout"])
    return outcome


def progress_due(engine: "TurnEngine", turn: _Turn) -> Optional[str]:
    """Промежуточная фраза хода (В31) или ``None``: ход ещё молчит, речь идёт в динамики, не тишина. Ключ — по тому,
    чего ход ждёт (``knowledge.PROGRESS_PHRASES``: ответ LLM, старт музыки)."""
    snap = engine.session.snapshot
    if turn.said or agent_spec(snap.agent).sink != Sink.SPEAKERS or snap.top("Silence") is not None:
        return None
    if replacing_music(engine, turn):
        return engine._pick(kn.MUSIC_REPLACE_PHRASES["progress"])
    return engine._pick(kn.PROGRESS_PHRASES.get(turn.waiting, kn.PROGRESS_LLM_PHRASE))


async def _progress(engine: "TurnEngine", turn: _Turn) -> None:
    import asyncio  # noqa: PLC0415

    await asyncio.sleep(kn.PROGRESS_AFTER_S)
    text = progress_due(engine, turn)
    if text:
        engine._log.info(f"{LOG_MARK} промежуточная фраза: ждём {turn.waiting} {turn.deadline.elapsed():.1f} с "
                         f"turn={turn.turn_id}")
        engine._say(turn, text, filler=True)  # заглушка: в окне есть, «повтори» её не повторяет (n808)


def ceiling_phrase(turn: _Turn, provider: str, replacing: bool = False) -> str:
    """Фраза потолка (В31): ждали действие — фраза его класса о таймауте («Музыка пока не заиграла.»), а не «не
    понял»; ждали LLM — с именем провайдера; провайдер неизвестен — ``understand.timeout``."""
    action = kn.ACTION_CLASSES.get(turn.waiting)
    if replacing:
        return phrases.text(kn.MUSIC_REPLACE_PHRASES["timeout"])
    if action is not None:
        return phrases.text(action.phrases.get("timeout") or TIMEOUT_PHRASE)
    return phrases.text(kn.CEILING_PHRASE, provider=provider) if provider else phrases.text(TIMEOUT_PHRASE)


def _close_ceiling(engine: "TurnEngine", turn: _Turn, expired: DeadlineExpired) -> None:
    provider = turn.provider or provider_in_flight(engine._ports)
    engine._log.warning(latency_line(provider, expired.what.split(":")[0], expired.waited_s * 1000, "timeout"))
    engine._say(turn, ceiling_phrase(turn, provider, replacing_music(engine, turn)))
    turn.stage, turn.status, turn.reason = "timeout", "failed", "deadline"


def _close_failed(engine: "TurnEngine", turn: _Turn, failed: ProviderFailed) -> None:
    engine._log.warning(latency_line(failed.provider, "llm", turn.deadline.elapsed() * 1000, "error"))
    engine._log.warning(f"{LOG_MARK} провайдер не ответил: {failed} turn={turn.turn_id}")
    text = phrases.text(kn.PROVIDER_FAILED_PHRASE, provider=failed.provider) if failed.provider else \
        phrases.text(TIMEOUT_PHRASE)
    engine._say(turn, text, None, "sad")
    turn.stage, turn.status, turn.reason = "timeout", "failed", "provider"


async def _paced(engine: "TurnEngine", turn: _Turn, work: Awaitable[None]) -> None:
    """Ход под потолком (В31): промежуточная фраза по таймеру, потолок и отказ провайдера закрывают ход фразой кода.
    Отмена (стоп человека) проходит насквозь — прерывает всегда."""
    import asyncio  # noqa: PLC0415

    progress = asyncio.ensure_future(_progress(engine, turn))
    try:
        await work
    except DeadlineExpired as expired:
        _close_ceiling(engine, turn, expired)
    except ProviderFailed as failed:
        _close_failed(engine, turn, failed)
    finally:
        progress.cancel()


class TurnEngine:
    """Ходы одного агента (личность): владелец сессии и журнала ходов — этот объект, транспорт — :class:`Ports`."""

    def __init__(self, session: Session, ports: Ports, *, system_prompt: str = "",
                 sections: Optional[Callable[[SessionSnapshot, str], Mapping[str, Any]]] = None,
                 voices: Optional[Callable[[], Voices]] = None, turn_log: Optional[TurnLog] = None,
                 events: Any = None, new_id: Callable[[], str] = lambda: uuid.uuid4().hex[:12],
                 clock: Callable[[], float] = time.monotonic, deadline_s: float = kn.TURN_CEILING_S,
                 log: Optional[logging.Logger] = None, rng: Optional[random.Random] = None,
                 sing: Optional[SingRouter] = None, phrase_voice: PhraseVoice = no_phrase_voice,
                 warm_voices: Callable[[], Tuple[str, ...]] = no_warm_voices) -> None:
        self.session = session
        self._warm_voices = warm_voices  # прогретые голоса без имени от хоста (``voice_warmth``, Шифу 05.11)
        self._sing = sing  # путь пения от хоста; нет — форма хода без пути пения
        self._phrase_voice = phrase_voice  # голос фразы по её id от хоста
        self._rng = rng
        self._pick = phrases.Picker(rng)
        self.log = turn_log or TurnLog()
        self._ports = ports
        self._system = system_prompt
        self._sections = sections or (lambda _snap, _text: {})
        self._voices = voices or (lambda: ())
        self._events = events
        self._new_id = new_id
        self._clock = clock
        self._deadline_s = deadline_s
        self._log = log or _LOG
        self._window = WindowGuard()
        self._generated = track_ref.GeneratedTopic(clock)  # тема последнего заказа генерации: «песня про енота» (n707)
        #: Правки звучащего счёта (:func:`count_edit`): ``speech_id`` старого счёта → (ход, правка, голос, эмоция).
        self._count_edits: Dict[str, Tuple[Any, str, Optional[str], Optional[str]]] = {}
        #: Источник прошлого хода и группы, что переносятся на следующий ход того же источника (:func:`carried_groups`).
        self._carry: Tuple[str, Tuple[str, ...]] = ("", ())
        #: Идущие вереницы (:func:`run_chain`): задача живёт после закрытия своего хода, ссылка держит её от сборщика.
        self.chains: Dict[Any, ChainRun] = {}

    def reset_window(self, reason: str) -> None:
        """Сброс окна по смене режима e2e (#4258); хост зовёт в потоке цикла ходов."""
        reset_window(self.session, self.log, self._window, reason, self._clock(), self._log)

    async def run(self, addressed: Addressed) -> TurnAggregate:
        """Один адресованный ход до конца; итог — агрегат без текста (строка лога ``📊 [turn]``). Отмена задачи хода
        (перебивание стопом) — итог ``cancelled`` и ``CancelledError`` дальше, как положено задаче asyncio."""
        import asyncio  # noqa: PLC0415 — asyncio тянет socket: импорт пакета чистый (test_purity)

        reset_if_idle(self.session, self.log, self._window, self._clock(), self._log)  # окно без срока давности (#4258)
        self._window.touch(self._clock())
        speaker, snap, view = pickup_parts(addressed, self.session.snapshot, self._warm_voices())  # права - у автора
        turn = _Turn(self._new_id(), snap.epoch, addressed.text.strip(), addressed.source,
                     TurnDeadline(self._deadline_s, clock=self._clock), pickup=addressed.pickup,
                     heard=picked_heard(addressed), recent=recent_section(addressed),
                     background=background_section(addressed), background_summary=addressed.background_summary,
                     command=addressed.command, speaker=speaker, pickup_view=view, wake_words=snap.wake_words,
                     utterance_id=addressed.utterance_id)
        self.log.begin(turn.turn_id, turn.epoch, turn.text, speaker_label(snap.speaker), turn.heard)
        log_pickup(self._log, addressed, turn)
        log_stt_fixes(self._log, addressed, turn)
        self._event("user", turn, text=turn.text, data={"source": turn.source})
        self._ports.stage(turn.turn_id, turn.epoch, "addressed", "started", source=turn.source)
        if wordless(self, turn):
            return self._finish(turn)
        work = turn_work(self, turn, snap)
        try:
            await _paced(self, turn, work)
        except asyncio.CancelledError:
            turn.stage, turn.status, turn.reason = "done", "cancelled", "interrupt"
            self._log.info(f"{LOG_MARK} ход отменён перебиванием turn={turn.turn_id} epoch={turn.epoch}")
            self._finish(turn)
            raise
        return self._finish(turn)

    def speech_started(self, speech_id: str) -> None:
        """Владелец речи начал реплику хода (``SpeechEvent started``) — e2e-событие с её текстом."""
        said = self.log.speech(speech_id)
        if said is not None:
            self._event_raw("speech_started", said[0], text=said[1])

    def speech_ended(self, speech_id: str, spoken_chars: int) -> None:
        """Реплика отзвучала (``finished``/``cancelled``/``rejected``): в окно — только озвученная часть."""
        said = self.log.speech(speech_id)
        part = self.log.spoken(speech_id, spoken_chars)
        if said is not None and part:
            self._event_raw("speech_spoken", said[0], text=part, data={"spoken_chars": int(spoken_chars)})
        continue_count(self, speech_id, said, spoken_chars)  # правленый счёт - дальше в новой форме (#4207)

    async def _tier1(self, turn: _Turn, command: Tier1Command, snap: SessionSnapshot, parsed: str) -> None:
        """Tier-1 кодом: сессия — здесь, действия — владельцам, речь и движение — через порты. 0 вызовов LLM.
        Политика перебивания читает тот же текст, по которому сработал ``command`` (``parsed``, :func:`tier1_of`)."""
        self._log.info(f"{LOG_MARK} tier1 kind={command.kind} args={dict(command.args)} turn={turn.turn_id}")
        turn.stage = "done"
        actions = tier1_actions(command, snap)
        stop = interrupt_for(command, parsed, snap)
        if stop.interrupts:
            snap = self._interrupt(turn, stop)
            actions = () if stop_performance(self._ports, str(stop.perform), stop.keep_backing) else stop.music
        if command.kind == "silence" and snap.top("Silence") is None:
            self.session.push_scene("Silence", "user")
        elif command.kind == "unsilence":
            self.session.pop_scene("Silence", "user")
        elif command.kind == "new_session":
            self.session.reset()
            self.log.reset()
            turn.epoch = self.session.snapshot.epoch  # сам ход — в новой сессии: его подтверждение не «устарело»
        self.log.mark(turn.turn_id, kn.WINDOW_SILENCE_MARKS.get(command.kind, ""))  # тишина вкл/выкл без слов
        results = await self._execute(turn, actions, snap)
        turn.expression = tier1_face(command, results, turn.text)  # очередь показа красит итог done
        said = tier1_phrase(command, results, self._pick, self.log.last_spoken(exclude=turn.turn_id))  # повтор
        if said and self.session.snapshot.top("Silence") is None:
            self._say(turn, said)
            note_stop_said(turn, command)
        turn.unvoiced += unvoiced(said, results)  # стоп молчит: исход скажет код, если модель повторит то же (n608)

    def _interrupt(self, turn: _Turn, stop: Interrupt) -> SessionSnapshot:
        """REPLACE (§6.2): речь эпохи хода — отмена (граница по слову), движение — ``stop_motion``, эпоха + 1.
        Ход сам переходит в новую эпоху: его стоп музыки — действие новой эпохи, а не отброшенное."""
        self._ports.cancel_speech(turn.epoch, f"tier1 {stop.kind}", stop.speech_boundary)
        if stop.stop_motion:
            self._ports.intent(turn.turn_id, turn.epoch, "stop_motion")
        old, turn.epoch = turn.epoch, self.session.replace_epoch()
        steer_chains(self, stop.kind, old, turn.epoch)  # стоп гасит вереницу, «замолчи» - нет (ADR-0165 §6)
        self._log.info(f"{LOG_MARK} interrupt policy={stop.policy} kind={stop.kind} boundary={stop.speech_boundary} "
                       f"epoch {old}→{turn.epoch} keep={sorted(stop.keep)} turn={turn.turn_id}")
        return self.session.snapshot

    def _stale(self, turn: _Turn, what: str) -> bool:
        """Эпоха сменилась, пока ход ждал (перебили стопом, новая сессия): его ``what`` не исполняется."""
        now = self.session.snapshot.epoch
        if now == turn.epoch:
            return False
        self._log.warning(f"{LOG_MARK} stale_epoch {what} turn={turn.turn_id} epoch={turn.epoch} now={now} — отброшено")
        turn.stage, turn.status, turn.reason = "done", "cancelled", "stale_epoch"
        return True

    async def _understand(self, turn: _Turn, snap: SessionSnapshot) -> None:
        """Команда модели: ≤ ``LLM_QUERY_ITERATIONS_MAX`` итераций ``query``, затем действия и/или речь; последняя без
        слов после запроса - ещё ≤ ``SPEECH_PUSHES_MAX`` вызовов за словами (:func:`_push_for_words`)."""
        self._ports.stage(turn.turn_id, turn.epoch, "thinking", "started")
        carry = self._carry[1] if self._carry[0] == turn.source else ()
        offered = available_tools(snap, None, snap.agent, turn.text, carry=carry,
                                  asked=tool_ask.asked_tools(self.session))
        tools = form_tools(offered, current_form(), snap.agent, snap.owners)
        messages: List[Any] = self._messages(turn, snap)
        for iteration in range(kn.LLM_QUERY_ITERATIONS_MAX + 1 + kn.SPEECH_PUSHES_MAX):  # + повторы за словами
            last = iteration >= kn.LLM_QUERY_ITERATIONS_MAX
            answer = await _ask_iteration(self, turn, messages, tools, last)
            turn.llm_calls += answer.calls
            turn.provider, turn.tokens = answer.provider or turn.provider, turn.tokens + answer.tokens
            if self._stale(turn, "answer"):
                return
            data = answer.result.reason if isinstance(answer.result, Invalid) else None
            self._event("command", turn, data={"provider": answer.provider, "valid": data is None, "reason": data})
            result = silence_insisted(answer.result)
            if isinstance(result, Command):
                more = await self._act(turn, result, snap, last=last)
            else:
                more = await _close_invalid(self, turn, result, snap, last=last)
            if more is None:
                return
            messages.extend(answer.follow(more))

    async def _act(self, turn: _Turn, cmd: Command, snap: SessionSnapshot, last: bool) -> Optional[str]:
        """Исполнить команду. В исходах есть ``query`` и итерации ещё есть — вернуть его данные модели
        (:func:`query_more`), и рядом с действиями тоже (``[play_animation, music_library list]`` без речи
        молчал); слова команды, написанные до данных, не звучат — модель ответит по данным, и экспрессия такой команды
        итог не красит: красит исполненное (сцена ``play_animation``, :func:`command_face`), голый ``query`` лицо
        итога не трогает. Голый неслышимый ответ на адресованную реплику — нарушение для модели
        вместо исполнения. Действие, уже исполненное в этом ходе, второй раз не исполняется
        (:func:`unrepeated`)."""
        turn.stage = "done"
        if cmd.passed:
            close_passed(self, turn, cmd)
            return None
        fresh, names = considered(self, turn, cmd, last)
        owed = owed_now(self.session.snapshot, turn, cmd, fresh, names, last)
        if owed:
            return await _answer_owed(self, turn, fresh, snap, last, owed)
        runnable = _runnable(fresh.actions, last)
        if action_chain.runs_as_chain(runnable):  # два и больше шагов: план, ход закрыт, шаги - вереницей (ADR-0165)
            start_chain(self, turn, fresh, runnable, snap)
            return None
        results = await self._execute(turn, runnable, snap)
        if self._stale(turn, "results"):
            return None
        if not last and any(r.cls == _QUERY for r in results):
            turn.expression = command_face(turn.expression, results, turn.text, earlier=turn.expression)
            return query_more(self, turn, results, cmd.expression.emotion)
        self._speak(turn, fresh, results, unread=unanswered(cmd, fresh, runnable))
        return None

    def _speak(self, turn: _Turn, cmd: Command, results: Sequence[ActionResult], unread: bool = False) -> None:
        """Сначала фраза кода об исходе (провал — грустно), затем слова модели, что звучат (§4.1). Экспрессия команды
        запоминается для рефлекса ``done`` (:func:`command_face`; сцены прошлых итераций - ``earlier``).
        ``unread`` — модель не дала ответа сверх сделанного (:func:`unanswered`: на последней итерации ещё просила
        ``query`` или только повторила исполненное): слов её не слышно — говорит код по данным хода
        (:func:`_close_unread`), без грусти, если это ответ по данным (:func:`query_emotion`)."""
        turn.expression = command_face(cmd.expression, results, turn.text, earlier=turn.expression)
        outcome = _spoken_outcome(cmd, unsaid_stops(turn, results), self.session.snapshot.agent, self.log.performing())
        outcome = replace_timeout_outcome(self, turn, results, outcome)  # прежний сет звучит - не «не заиграла» (#4479)
        if outcome:  # провал - грустно, вопрос тула («Удалить «X»?») - эмоцией вопроса, иначе лицом модели
            self._say(turn, outcome, None, outcome_emotion(results, cmd.expression.emotion))
        cmd = count_edit(self, turn, cmd)  # правка звучащего счёта: продолжение, а не счёт с начала (#4207)
        asked = over_ordered_music(asked_voice(cmd), results)  # голос и подложка, о которых просил человек
        owned, refusal = hand_perform(self._ports, turn.turn_id, turn.epoch, asked, self._log)
        note_perform_start(self.log, turn.turn_id, cmd, owned and not refusal)
        self._say(turn, refusal, None, "sad")  # отказ владельца выступления — фраза кода, а не тишина ("" — нет)
        lines = _content_lines(cmd, self.session.snapshot.agent, segments=not owned)
        for text, voice in lines:
            self._say(turn, text, voice or None, cmd.expression.emotion)
        if cmd.ask is not None and not cmd.actions:
            self.session.set_pending_ask(cmd.ask)
        if unread and not lines and not owned:  # модель ещё читала (query сверх лимита) или повторила исполненное
            said, emotion = _close_unread(turn, results)
            self._say(turn, said, None, emotion)

    async def _execute(self, turn: _Turn, actions: Sequence[ToolCall], snap: SessionSnapshot,
                       ref: Optional[str] = None) -> List[ActionResult]:
        """Действия владельцам разом; ``ref`` - свой ``turn_id`` запросов (шаг вереницы: ``<ход>/c<k>``), иначе ход."""
        if not actions:
            return []
        # «эта песня» / «песня про <тему заказа>» - сочинённая только что (#4216, М11 n707): ссылку ставит код
        actions = track_ref.bind_last_generated(actions, turn.text, self._generated.stems(), wake_words=turn.wake_words)
        cls = kn.call_class(actions[0].tool, actions[0].args)
        turn.waiting = cls
        turn.called += tuple(a.tool for a in actions)
        turn.replacing = orders_over_music(self.session.snapshot, actions)
        self._ports.stage(turn.turn_id, turn.epoch, "executing", "started", cls=cls)
        # свой срок класса целиком (исполнитель закроет честным timeout), а не остаток хода после LLM и его повтора
        turn.deadline.grant(action_term(actions))
        now = pickup_parties.own_snapshot(self.session.snapshot, turn.speaker)  # подхват: собеседник - автор фразы
        run = execute_for_speaker(self._ports, actions, snap, now, request_ref(turn, ref), self._log)
        results = pickup_parties.intro_reasons(await turn.deadline.wait(run, "actions"), turn.pickup_view)
        self._generated.note(actions, results)
        turn.music_stopped, results = reconcile_stops(turn.music_stopped, results)  # фраза стопа - правда о ходе
        turn.acted += acted_calls(actions, results)  # повтор в этом ходе не исполняется
        turn.read += read_calls(actions, results)  # повтор чтения в этом ходе не исполняется
        owner = memory.memory_owner(snap)  # чья память у хода при исполнении: исход - только ей в окне
        for result in results:
            self.log.result(turn.turn_id, result, owner)
            self._event("action", turn, data={"tool": result.tool, "status": result.status, "reason": result.reason,
                                              "cls": result.cls})
        self._note_asked(turn, actions, results)
        return results

    def _note_asked(self, turn: _Turn, actions: Sequence[ToolCall], results: Sequence[ActionResult]) -> None:
        """Окно вопроса тула по исходам хода (``tool_ask.note``). Эпоха сменилась, пока тул отвечал (стоп, новая
        сессия): вопрос не прозвучит (``_stale``) - и окна под него нет."""
        if self.session.snapshot.epoch != turn.epoch:
            return
        asked = tool_ask.note(self.session, actions, results)
        if asked is not None:
            self._log.info(f"{LOG_MARK} {ASK_MARK} открыт: {tool_ask.describe(asked)} ttl={kn.TOOL_ASK_TTL_S:.0f}с "
                           f"turn={turn.turn_id}")

    def _messages(self, turn: _Turn, snap: SessionSnapshot) -> List[Any]:
        """Запрос: системный промпт, окно истории (``user``/``assistant``: озвученное и метки кода «[ответ действием:
        …]», «[ответ лицом: …]», «[рот заклеен — молчу]» вместо ответа без слов), снимок хода + реплика человека.
        Исходы прошлых действий — разделом снимка :data:`RECENT_ACTIONS`, а не репликами (§7.3); исход тула памяти
        другого владельца памяти (спорный голос, другой человек) туда не идёт (``memory.foreign_result``)."""
        past = self.log.window(exclude=turn.turn_id)  # реплика этого хода — последним сообщением, со снимком
        history = [e for e in past if e.role != TOOL]
        sections = dict(self._sections(snap, turn.text))
        if turn.music_stopped:  # стоп этого хода: снимок плеера придёт позже, а модель не должна читать «играет»
            sections["music_state"] = after_stop(sections.get("music_state"))
        recent = recent_results(past, results_of(self.log, turn.turn_id), memory.memory_owner(snap))
        turn.seen = tuple(recent[-kn.TURN_WINDOW_MAX_TURNS:])  # опора слов о состоянии (``state_claim``)
        context = build_context({**sections, RECENT_UNADDRESSED: list(turn.recent),
                                 BACKGROUND_SINCE_REPLY: list(turn.background),
                                 BACKGROUND_SUMMARY: dict(turn.background_summary),
                                 pickup_parties.PICKUP_SECTION: dict(turn.pickup_view),
                                 RECENT_ACTIONS: recent[-kn.TURN_WINDOW_MAX_TURNS:]})
        messages: List[Any] = [{"role": "system", "content": self._system}] if self._system else []
        messages += [{"role": e.role, "content": e.for_llm()} for e in history]
        messages.append({"role": "user", "content": f"{context}\n\n{user_text(turn)}"})
        return messages

    def _say(self, turn: _Turn, text: str, voice: Optional[str] = None, emotion: Optional[str] = None,
             filler: bool = False) -> None:
        if not text or self._stale(turn, "speech"):
            return
        turn.sad |= emotion == "sad"  # лицо итога без лица команды - по грустной фразе
        speech_id = send_speech(self._ports, turn.turn_id, turn.epoch, text, voice, emotion)
        # реплика не в динамики (чат, шлем оператора) — SpeechEvent о ней не придёт: в окне она сразу, как отдана
        delivered = agent_spec(self.session.snapshot.agent).sink != Sink.SPEAKERS
        self.log.said(turn.turn_id, speech_id, text, delivered=delivered, filler=filler)
        turn.said.append(speech_id)
        self._event("speech_requested", turn, text=text, data={"speech_id": speech_id, "voice": voice or ""})

    def _finish(self, turn: _Turn) -> TurnAggregate:
        latency = int(turn.deadline.elapsed() * 1000)
        face = finish_stage(self._ports, turn, self.session.snapshot)
        if face is not None:  # ответ одним лицом: без записи в окне реплика человека висит без ответа
            self.log.faced(turn.turn_id, face_label(face.emotion, face.gesture, face.face))
        agg = TurnAggregate(turn.turn_id, turn.epoch, self.session.snapshot.agent, turn.stage, turn.status,
                            latency, turn.llm_calls, turn.provider, turn.tokens, turn.reason)
        self._log.info(agg.log_line())
        self._event("turn", turn, data={**{k: v for k, v in vars(agg).items() if k != "turn_id"},
                                        "source": turn.source})
        self._window.touch(self._clock())  # долгий ход (выступление) пауза не считается
        self._carry = (turn.source, carried_groups(turn.called))
        return agg

    def _event(self, kind: str, turn: _Turn, text: str = "", data: Optional[Mapping[str, Any]] = None) -> None:
        self._event_raw(kind, turn.turn_id, turn.epoch, text, data)

    def _event_raw(self, kind: str, turn_id: str, epoch: Optional[int] = None, text: str = "",
                   data: Optional[Mapping[str, Any]] = None) -> None:
        """Событие в e2e-БД (прод: писателя нет — ничего не пишется)."""
        if self._events is None:
            return
        try:
            self._events.write(kind, turn_id=turn_id, epoch=self.session.snapshot.epoch if epoch is None else epoch,
                               agent=self.session.snapshot.agent, text=text, data=dict(data or {}))
        except Exception as exc:  # noqa: BLE001 — сбой e2e-БД не роняет ход робота
            self._log.warning(f"{LOG_MARK} e2e-событие {kind} не записано: {exc}")


__all__ = ["ASK_MARK", "Answer", "ChainRun", "close_chain", "settle", "start_chain", "steer_chains", "run_chain", "LOG_MARK", "Ports", "RECENT_ACTIONS", "RECENT_UNADDRESSED", "BACKGROUND_SINCE_REPLY",
           "BACKGROUND_SUMMARY", "SAD_FACE", "Shown", "TurnEngine", "answer_recognized", "command_face", "command_form", "done_expression",
           "asked_voice", "finish_stage", "hand_perform", "held_expression", "host_sections", "shown_face",
           "foreign_keys", "invalid_emotion", "outcome_phrase", "over_ordered_music", "owed_reason", "query_answer",
           "music_promise_key", "query_fallback", "recent_results", "considered", "owed_now", "read_calls", "reread_reason", "unbacked_claims", "unclaimed_without_retry",
           "unnamed_without_retry", "unreread", "robot_owed", "show_queue_phrase", "stop_performance", "tier1_actions", "tier1_face", "tier1_for",
           "tier1_phrase", "turn_work", "answer_tool", "unrepeated", "voices_passed", "words_owed"]
