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
from typing import (Any, Awaitable, Callable, Collection, FrozenSet, List, Mapping, Optional, Protocol, Sequence,
                    Tuple)

from rob_box_core.tool_catalog import get_tool
from rob_box_core.utterance import Sink, Utterance
from rob_box_dialog import face_scene
from rob_box_dialog import foreign_name
from rob_box_dialog import knowledge as kn
from rob_box_dialog import memory
from rob_box_dialog import phrases
from rob_box_dialog import tool_ask
from rob_box_dialog.address import BARE_WAKE, Addressed
from rob_box_dialog.agents import agent_spec, allows_tier1, tools_for
from rob_box_dialog.command import (Command, Expression, Form, Invalid, Performance, Result, ToolCall, current_form,
                                    explain, loud_actions, turn_form)
from rob_box_dialog.context import build_context
from rob_box_dialog.deadline import TIMEOUT_PHRASE, DeadlineExpired, ProviderFailed, TurnDeadline, latency_line
from rob_box_dialog.execute import deadline_s
from rob_box_dialog.expression import face_label, show_seconds, shown_scene, shows
from rob_box_dialog.grammar import Tier1Command, Voices, match
from rob_box_dialog.heard import HEARD_MARK
from rob_box_dialog.interrupt import Interrupt, interrupt_for, stop_music_calls, stops_music
from rob_box_dialog.respond import (ActionResult, asks, audible, classed, failed, join, outcome_emotion,
                                    reconcile_stops)
from rob_box_dialog.session import Session, SessionSnapshot
from rob_box_dialog.speaker_adapter import asks_recognized, recognition_reply, speaker_block
from rob_box_dialog.tools_view import (available_tools, music_asked, needs_words, robot_asked, show_asked,
                                       voice_asked, voice_named)
from rob_box_dialog.turn_log import PERFORM_STARTED, TOOL, USER, TurnAggregate, TurnLog, phrase_label, speaker_label

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
            emotion: Optional[str] = None) -> str: ...

    def stage(self, turn_id: str, epoch: int, stage: str, status: str, reason: str = "",
              cls: Optional[str] = None, llm_calls: int = 0, provider: str = "", source: str = "",
              expression: Optional[Expression] = None) -> Optional[bool]:
        """Событие стадии и рефлекс. ``expression`` (необязательный) — экспрессия команды на итоге ``done``
        (:func:`done_expression`): красит рефлекс. Порт без этого параметра (шлем оператора — лица у него нет)
        зовётся прежним вызовом (:func:`finish_stage`); ``False`` в ответ — лицо порт не показал (пульт чата)."""
        ...

    def cancel_speech(self, epoch: int, reason: str, boundary: str = "as_requested") -> None: ...

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
    said: List[str] = field(default_factory=list)
    queried: Tuple[ActionResult, ...] = ()  # исходы выполненного в этом ходе query (для страховки от молчания)
    waiting: str = "llm"  # чего ждёт ход: ответа LLM или события владельца (класс действия) — фразы В31
    retried: int = 0  # повторы модели за ответ не тот (``VIOLATION_SILENT_ANSWER``, ``…_REMOTE_OWED``)
    expression: Optional[Expression] = None  # экспрессия исполненной команды — красит рефлекс ``done``
    command: str = ""  # запасной разбор Tier-1 (:func:`tier1_of`): слова после вейка (``Addressed.command``, n811)
    acted: Tuple[ToolCall, ...] = ()  # исполненные в ходе вызовы (не ``query``): повтор не исполняется
    sad: bool = False  # прозвучала грустная фраза (провал, отказ): лицо итога без лица команды - грусть
    utterance_id: str = ""  # фраза STT хода (``Addressed.utterance_id``): по ней - её SpeakerResult
    music_stopped: bool = False  # ход уже выключил музыку (стоп-тул ``done``): второй стоп не «ничего не играет»


def picked_heard(addressed: Addressed) -> Tuple[Tuple[str, str], ...]:
    """Подхваченные фразы бэклога ``(кто, фраза)``: автор — метка самой фразы (``Backlog.label``), не собеседник
    хода."""
    return tuple((phrase_label(p.get("speaker") or {}), str(p.get("text") or "")) for p in addressed.picked)


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
    if addressed.pickup or addressed.source == BARE_WAKE:
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


def form_tools(tools: Sequence[Any], form: Form, agent: str) -> Tuple[Any, ...]:
    """Тулы хода в форме пути пения: закрытые путём убраны, тул, который путь требует (``generate_music``), добавлен,
    если его срезала обрезка по ``TOOLS_VISIBLE_MAX`` (а агенту он доступен)."""
    sing = form.sing_form
    if sing is None:
        return tuple(tools)
    kept = [t for t in tools if t.name not in sing.closed_tools]
    need = sing.needs_tool
    if need and need not in {t.name for t in kept} and need in tools_for(agent_spec(agent)):
        kept.append(get_tool(need))
    return tuple(kept)


def words_owed(snapshot: SessionSnapshot, pickup: bool, text: str, cmd: Command) -> bool:
    """Команда модели не слышна, а человек ждёт слов: агент говорит в динамики (Личность, не шлем оператора и
    не чат), реплика адресована - не подхват бэклога и не тишина (:func:`voices_passed`), и сама команда без слов
    (``tools_view.needs_words``: голая анимация, которую словами не просили)."""
    return (agent_spec(snapshot.agent).sink == Sink.SPEAKERS and voices_passed(pickup, snapshot)
            and needs_words(text, cmd))


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


def tier1_phrase(command: Tier1Command, results: Sequence[ActionResult],
                 pick: Callable[[str], str] = lambda key: "", last_spoken: str = "") -> Optional[str]:
    """Фраза Tier-1: громкость — по исходу тула; сброс скорости и тона — по исходам тулов (:func:`prosody_phrase`);
    шаг голоса — по исходам тулов («Говорю ниже.», ось на границе — «Ниже уже некуда…», отказ — его фраза);
    новая сессия — вариант из каталога; очередь показа — лицом, слова только о провале (:func:`show_queue_phrase`);
    «повтори» — ``last_spoken`` дословно (последняя озвученная реплика робота, ``TurnLog.last_spoken``), а
    повторять нечего — фраза кода ``REPEAT_NOTHING_PHRASE``; стоп и тишина молчат; смена голоса — одна фраза кода по
    результатам тулов (:func:`voice_phrase`), тул голоса не говорит."""
    if command.kind == kn.REPEAT_KIND:
        return repeat_phrase(last_spoken)
    if command.kind == "new_session":
        return pick(kn.NEW_SESSION_PHRASE)
    if command.kind == kn.PROSODY_NORMAL_KIND:
        return prosody_phrase(results)
    if command.kind == kn.SHOW_QUEUE_KIND:
        return show_queue_phrase(results)
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


async def understand(engine: "TurnEngine", turn: "_Turn", snap: SessionSnapshot) -> None:
    """Ход к LLM (``TurnEngine._understand``), если его не проглотила тишина (:func:`hold_silence`) и на него не ответил
    код (:func:`answer_recognized`), — в форме команды хода (:func:`command_form`): схема и валидатор моста видят
    её на время вызовов LLM этого хода."""
    if hold_silence(engine, turn) or await answer_recognized(engine, turn, snap):
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
    if outcome.get("status") != "rejected":
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


def _close_unread(turn: _Turn, acted: bool) -> str:
    """Модель на последней итерации ещё просит ``query`` (его уже не исполняют, :func:`_runnable`) или только
    повторяет исполненное в ходе (:func:`unrepeated`), а её слов не слышно: ответа она не дала
    (:func:`unanswered`). Запрос в ходе был - ход ``done``, говорит код по данным запроса (В21: на
    адресованное не молчим: ``[play_animation, music_library list]`` - анимация исполнена, фраза кода по данным).
    Запроса не было: действия исполнены (``acted``) - их итог уже сказан, исполнять было нечего - провал без речи."""
    if turn.queried:
        turn.stage, turn.status, turn.reason = "done", "done", "query_fallback:ceiling"
        return query_fallback(turn.queried)
    if not acted:
        turn.stage, turn.status, turn.reason = "failed", "failed", "nothing_to_do"
    return ""


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
    turn.queried = readable
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


def _library_slots(data: Mapping[str, Any]) -> Optional[Mapping[str, Any]]:
    """Слоты фразы списка библиотеки из данных ``music_library list``: всего треков (``total`` тула - вся
    библиотека, а не выборка) и первые ``LIBRARY_SUMMARY_NAMES`` названий в кавычках (варианты одного заказа - одно
    имя). Пустая библиотека - слоты есть (её фраза - о пустой, ``QUERY_FALLBACK_PHRASES``). Не ``list``, нет числа
    или треки без названий - ``None``: фразы по данным нет."""
    total, tracks = data.get("total"), data.get("tracks")
    if data.get("action") != "list" or isinstance(total, bool) or not isinstance(total, int):
        return None
    titles = dict.fromkeys(str(t.get("title") or "").strip() for t in tracks or () if isinstance(t, Mapping))
    names = [f"«{title}»" for title in titles if title][:kn.LIBRARY_SUMMARY_NAMES]
    return {"total": total, "names": ", ".join(names)} if names or total <= 0 else None


#: Слоты фразы кода по данным ``query`` - по тулу таблицы ``knowledge.QUERY_FALLBACK_PHRASES`` (ключи совпадают).
_QUERY_SLOTS: Mapping[str, Callable[[Mapping[str, Any]], Optional[Mapping[str, Any]]]] = {
    "list_tts_voices": _voices_slots, kn.LIBRARY_TOOL: _library_slots}


def query_answer(results: Sequence[ActionResult]) -> Optional[str]:
    """Ответ кода по данным выполненного ``query``: фраза тула (``knowledge.QUERY_FALLBACK_PHRASES``,
    ключ - по данным вызова) со слотами из данных результата: голоса - сводка по провайдерам, список библиотеки - число
    треков и 2–3 названия. Фразы по данным нет - ``None``."""
    for r in results:
        entry = kn.QUERY_FALLBACK_PHRASES.get(r.tool) if r.status == "done" else None
        slots = _QUERY_SLOTS[r.tool](r.data) if entry is not None else None
        if slots is not None:
            return phrases.text(entry.key(r.data) if isinstance(entry, kn.ByData) else entry, **slots)
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
    if turn.queried:
        log.warning(f"{LOG_MARK} невалидная команда {why} после query → фраза кода по данным запроса")
        turn.stage, turn.status, turn.reason = "done", "done", f"query_fallback:{why}"[:60]
        engine._say(turn, query_fallback(turn.queried), None, query_emotion(turn.queried, invalid_emotion(invalid, ())))
        return None
    log.warning(f"{LOG_MARK} невалидная команда {why} → допустимых действий {len(invalid.salvage)}, итог скажет код")
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
    if said or (results and not words_owed(snap, turn.pickup, turn.text, Command(actions=tuple(invalid.salvage)))):
        return said or ""
    return engine._pick(kn.INVALID_COMMAND_PHRASES.get(invalid.reason, kn.INVALID_COMMAND_PHRASE))


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
    return robot_asked(turn.text)


def owed_reason(snapshot: SessionSnapshot, turn: "_Turn", cmd: Command, names: Collection[str] = ()) -> str:
    """Ответ модели не тот, что ждёт человек, — причина нарушения (:data:`_OWED`), иначе «»: голый неслышимый ответ на
    адресованную реплику (:func:`words_owed`), только текст в чат на просьбу сделать роботом
    (:func:`robot_owed`) или обращение к неподтверждённому собеседнику чужим именем (``names`` - ключи из
    :func:`foreign_keys`)."""
    if words_owed(snapshot, turn.pickup, turn.text, cmd):
        return kn.VIOLATION_SILENT_ANSWER
    if robot_owed(snapshot, turn, cmd):
        return kn.VIOLATION_REMOTE_OWED
    return kn.VIOLATION_FOREIGN_NAME if foreign_name.offending(cmd, names) else ""


def foreign_keys(window: Sequence[Any], turn: "_Turn", cmd: Command, snapshot: SessionSnapshot) -> FrozenSet[str]:
    """Имена, которыми нельзя обращаться к собеседнику хода: собеседник неподтверждён
    (:func:`foreign_name.unconfirmed`), а имена, которые модель видела в разговоре, - авторы реплик окна, подхвата и
    фона (:func:`foreign_name.name_keys`). Не считаются названные самим человеком в этой реплике и им же в
    ``register_speaker`` команды: «меня зовут Гриша» - ответ «Понял, Гриша» честен."""
    if not foreign_name.unconfirmed(snapshot.speaker):
        return frozenset()
    seen = [e.speaker for e in window if e.role == USER] + [who for who, _ in turn.heard]
    seen += [str(p.get("who") or "") for p in (*turn.recent, *turn.background)]
    said = [turn.text, *(text for _who, text in turn.heard)]
    said += [str(a.args.get("name") or "") for a in cmd.actions if a.tool == "register_speaker"]
    return foreign_name.name_keys(seen) - foreign_name.name_keys(said)


@dataclass(frozen=True)
class _Owed:
    """Нарушение «ответ не тот, что ждут» (ADR-0148 п.2: валидатор с понятной ошибкой): ``hint`` — подсказка модели,
    ``what`` — что в логе, ``keeps_words`` — после повтора слова модели уходят как есть, а код говорит рядом факт
    (фраза ``INVALID_COMMAND_PHRASES``), а не вместо них."""

    hint: str
    what: str
    keeps_words: bool = False


#: Голый неслышимый ответ на адресованную реплику: слов модели нет — говорит код. Текст в чат на просьбу
#: сделать роботом: слова модели — её ответ, их не подменяем (промах признака на справке не теряет ответ), а факт «на роботе ничего не сделал» говорит код рядом.
_OWED: Mapping[str, _Owed] = {
    kn.VIOLATION_SILENT_ANSWER: _Owed(kn.SILENT_ANSWER_HINT, "без слов на адресованную реплику"),
    kn.VIOLATION_REMOTE_OWED: _Owed(kn.REMOTE_OWED_HINT, "только текст в чат на просьбу сделать роботом",
                                    keeps_words=True),
    kn.VIOLATION_FOREIGN_NAME: _Owed(kn.FOREIGN_NAME_HINT, "обращение к неподтверждённому собеседнику чужим именем"),
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
    # лицо команды, как и её анимация, показывается — говорит код
    await _close_invalid(engine, turn, Invalid(reason, salvage=cmd.actions), snap, cmd.expression)
    return None


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
    turn.text, turn.command = command.rest, ""
    snap = engine.session.snapshot
    engine._log.info(f"{LOG_MARK} остаток после стопа: {turn.text[:80]!r} turn={turn.turn_id}")
    follow, parsed = tier1_of(turn.text, turn.command, snap, engine._voices())
    if follow is not None and not follow.disputed:
        await _tier1_then_rest(engine, turn, follow, snap, parsed)
    else:
        await understand(engine, turn, snap)


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


def progress_due(engine: "TurnEngine", turn: _Turn) -> Optional[str]:
    """Промежуточная фраза хода (В31) или ``None``: ход ещё молчит, речь идёт в динамики, не тишина. Ключ — по тому,
    чего ход ждёт (``knowledge.PROGRESS_PHRASES``: ответ LLM, старт музыки)."""
    snap = engine.session.snapshot
    if turn.said or agent_spec(snap.agent).sink != Sink.SPEAKERS or snap.top("Silence") is not None:
        return None
    return engine._pick(kn.PROGRESS_PHRASES.get(turn.waiting, kn.PROGRESS_LLM_PHRASE))


async def _progress(engine: "TurnEngine", turn: _Turn) -> None:
    import asyncio  # noqa: PLC0415

    await asyncio.sleep(kn.PROGRESS_AFTER_S)
    text = progress_due(engine, turn)
    if text:
        engine._log.info(f"{LOG_MARK} промежуточная фраза: ждём {turn.waiting} {turn.deadline.elapsed():.1f} с "
                         f"turn={turn.turn_id}")
        engine._say(turn, text, filler=True)  # заглушка: в окне есть, «повтори» её не повторяет (n808)


def ceiling_phrase(turn: _Turn, provider: str) -> str:
    """Фраза потолка (В31): ждали действие — фраза его класса о таймауте («Музыка пока не заиграла.»), а не «не
    понял»; ждали LLM — с именем провайдера; провайдер неизвестен — ``understand.timeout``."""
    action = kn.ACTION_CLASSES.get(turn.waiting)
    if action is not None:
        return phrases.text(action.phrases.get("timeout") or TIMEOUT_PHRASE)
    return phrases.text(kn.CEILING_PHRASE, provider=provider) if provider else phrases.text(TIMEOUT_PHRASE)


def _close_ceiling(engine: "TurnEngine", turn: _Turn, expired: DeadlineExpired) -> None:
    provider = turn.provider or provider_in_flight(engine._ports)
    engine._log.warning(latency_line(provider, expired.what.split(":")[0], expired.waited_s * 1000, "timeout"))
    engine._say(turn, ceiling_phrase(turn, provider))
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
                 sing: Optional[SingRouter] = None, phrase_voice: PhraseVoice = no_phrase_voice) -> None:
        self.session = session
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

    async def run(self, addressed: Addressed) -> TurnAggregate:
        """Один адресованный ход до конца; итог — агрегат без текста (строка лога ``📊 [turn]``). Отмена задачи хода
        (перебивание стопом) — итог ``cancelled`` и ``CancelledError`` дальше, как положено задаче asyncio."""
        import asyncio  # noqa: PLC0415 — asyncio тянет socket: импорт пакета чистый (test_purity)

        snap = self.session.snapshot
        turn = _Turn(self._new_id(), snap.epoch, addressed.text.strip(), addressed.source,
                     TurnDeadline(self._deadline_s, clock=self._clock), pickup=addressed.pickup,
                     heard=picked_heard(addressed), recent=recent_section(addressed),
                     background=background_section(addressed), command=addressed.command,
                     utterance_id=addressed.utterance_id)
        self.log.begin(turn.turn_id, turn.epoch, turn.text, speaker_label(snap.speaker), turn.heard)
        log_pickup(self._log, addressed, turn)
        log_stt_fixes(self._log, addressed, turn)
        self._event("user", turn, text=turn.text, data={"source": turn.source})
        self._ports.stage(turn.turn_id, turn.epoch, "addressed", "started", source=turn.source)
        if addressed.source == BARE_WAKE or not turn.text:
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

    def _interrupt(self, turn: _Turn, stop: Interrupt) -> SessionSnapshot:
        """REPLACE (§6.2): речь эпохи хода — отмена (граница по слову), движение — ``stop_motion``, эпоха + 1.
        Ход сам переходит в новую эпоху: его стоп музыки — действие новой эпохи, а не отброшенное."""
        self._ports.cancel_speech(turn.epoch, f"tier1 {stop.kind}", stop.speech_boundary)
        if stop.stop_motion:
            self._ports.intent(turn.turn_id, turn.epoch, "stop_motion")
        old, turn.epoch = turn.epoch, self.session.replace_epoch()
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
        """Команда модели: ≤ ``LLM_QUERY_ITERATIONS_MAX`` итераций ``query``, затем действия и/или речь."""
        self._ports.stage(turn.turn_id, turn.epoch, "thinking", "started")
        offered = available_tools(snap, None, snap.agent, turn.text, asked=tool_ask.asked_tools(self.session))
        tools = form_tools(offered, current_form(), snap.agent)
        messages: List[Any] = self._messages(turn, snap)
        for iteration in range(kn.LLM_QUERY_ITERATIONS_MAX + 1):
            last = iteration >= kn.LLM_QUERY_ITERATIONS_MAX
            turn.waiting = "llm"
            answer = await turn.deadline.wait(self._ports.ask(messages, _offered(tools, last), turn.deadline.left()),
                                              "llm")
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
            turn.reason = "passed"
            self._log.info(f"{LOG_MARK} passed={cmd.passed} addressed={not turn.pickup}")
            # Решение Шифу 22.10: на адресованную реплику (вейк, окно ответа, Telegram) молчать нельзя — код мычит;
            # passed из бэклога (подхват накопленного) — тихо.
            if voices_passed(turn.pickup, self.session.snapshot):
                self._say(turn, self._pick(kn.PASSED_ADDRESSED_PHRASE))
            return None
        fresh = unrepeated(self._log, turn, cmd)
        names = foreign_keys(self.log.window(exclude=turn.turn_id), turn, fresh, self.session.snapshot)
        fresh = unnamed_without_retry(self._log, turn, fresh, names, last)
        owed = owed_reason(self.session.snapshot, turn, fresh, names)
        if owed:
            return await _answer_owed(self, turn, fresh, snap, last, owed)
        runnable = _runnable(fresh.actions, last)
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
        outcome = _spoken_outcome(cmd, results, self.session.snapshot.agent, self.log.performing())
        if outcome:  # провал - грустно, вопрос тула («Удалить «X»?») - эмоцией вопроса, иначе лицом модели
            self._say(turn, outcome, None, outcome_emotion(results, cmd.expression.emotion))
        asked = over_ordered_music(asked_voice(cmd), results)  # голос и подложка, о которых просил человек
        owned, refusal = hand_perform(self._ports, turn.turn_id, turn.epoch, asked, self._log)
        note_perform_start(self.log, turn.turn_id, cmd, owned and not refusal)
        self._say(turn, refusal, None, "sad")  # отказ владельца выступления — фраза кода, а не тишина ("" — нет)
        lines = _content_lines(cmd, self.session.snapshot.agent, segments=not owned)
        for text, voice in lines:
            self._say(turn, text, voice or None, cmd.expression.emotion)
        if cmd.ask is not None and not cmd.actions:
            self.session.set_pending_ask(cmd.ask)
        if unread and not lines and not owned:  # модель ещё читала (query сверх лимита), а её слов не слышно
            self._say(turn, _close_unread(turn, acted=bool(results)), None, query_emotion(turn.queried, "sad"))

    async def _execute(self, turn: _Turn, actions: Sequence[ToolCall], snap: SessionSnapshot) -> List[ActionResult]:
        if not actions:
            return []
        cls = kn.call_class(actions[0].tool, actions[0].args)
        turn.waiting = cls
        self._ports.stage(turn.turn_id, turn.epoch, "executing", "started", cls=cls)
        # свой срок класса целиком (исполнитель закроет честным timeout), а не остаток хода после LLM и его повтора
        turn.deadline.grant(action_term(actions))
        run = execute_for_speaker(self._ports, actions, snap, self.session.snapshot, turn.turn_id, self._log)
        results = await turn.deadline.wait(run, "actions")
        turn.music_stopped, results = reconcile_stops(turn.music_stopped, results)  # фраза стопа - правда о ходе
        turn.acted += acted_calls(actions, results)  # повтор в этом ходе не исполняется
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
        owner = memory.memory_owner(snap)
        recent = [dict(e.content) for e in past
                  if e.role == TOOL and not memory.foreign_result(e.content, e.owner, owner)]
        context = build_context({**sections, RECENT_UNADDRESSED: list(turn.recent),
                                 BACKGROUND_SINCE_REPLY: list(turn.background),
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
        speech_id = self._ports.say(turn.turn_id, turn.epoch, text, voice, emotion)
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
                            latency, turn.llm_calls, turn.provider, turn.tokens)
        self._log.info(agg.log_line())
        self._event("turn", turn, data={**{k: v for k, v in vars(agg).items() if k != "turn_id"},
                                        "reason": turn.reason, "source": turn.source})
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


__all__ = ["ASK_MARK", "Answer", "LOG_MARK", "Ports", "RECENT_ACTIONS", "RECENT_UNADDRESSED", "BACKGROUND_SINCE_REPLY",
           "SAD_FACE", "Shown", "TurnEngine", "answer_recognized", "command_face", "command_form", "done_expression",
           "asked_voice", "finish_stage", "hand_perform", "held_expression", "host_sections", "shown_face",
           "foreign_keys", "invalid_emotion", "outcome_phrase", "over_ordered_music", "owed_reason", "query_answer",
           "query_fallback", "unnamed_without_retry", "robot_owed", "show_queue_phrase", "stop_performance", "tier1_actions", "tier1_face", "tier1_for",
           "tier1_phrase", "turn_work", "answer_tool", "unrepeated", "voices_passed", "words_owed"]
