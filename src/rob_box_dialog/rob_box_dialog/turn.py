"""Ход диалога v2 без ROS: адресованная реплика → Tier-1 или команда LLM → исполнение → фраза кода (ADR-0150 §3, §4.1,
§7.3, §8; PR-8 ч.8d #3702).

:class:`TurnEngine` — весь ход в одном месте и только вызовами модулей пакета: грамматика Tier-1 (``grammar.match``,
0 вызовов LLM), снимок хода (``context.build_context``) и окно озвученного (``turn_log``), срез тулов
(``tools_view.available_tools``), команда модели, исполнение действий владельцам, фраза об исходе из события
(``respond``), дедлайн хода (``deadline``), e2e-события (``e2e_events``). Транспорт — :class:`Ports` снаружи (ROS-хост
``dialog_node``, ч.8e): вызов LLM с разбором и повтором по нарушениям, исполнитель действий (тот же MCP-провайдер
тулов, что у v1, — одна дорога исполнения), речь, стадии хода (``TurnEvent`` + рефлексы), отмена речи, интент.

Правила хода (ADR-0148: решает код):

* голое «робот» без бэклога — только рефлекс ``addressed``, 0 вызовов LLM, 0 речи (§7.3 п.1); с подхватом бэклога
  (В12″) — модель получает подхваченные фразы каждую со своим автором (``[говорит: …]`` на момент фразы, не
  того, кто сказал «робот»), и они остаются в окне прошлыми репликами (#3892);
* Tier-1 — исполняет код (стоп, тишина, громкость, голос, новая сессия, очередь показа лица «покажи по очереди
  радость, грусть и удивление» — ``play_animation`` на сцену, лицо итога и его очередь, #4103), спорный стоп (В19)
  идёт в LLM;
* вопрос тула («Удалить «X»? Скажи «да, удаляй».», ``knowledge.TOOL_QUESTIONS``) — окно ответа в сессии
  (``tool_ask``, #4062): ответ «да, удаляй» исполняет код подтверждённым вызовом, фраза — из результата тула; «нет» —
  фраза кода; другие слова — к LLM с тулом вопроса в срезе. Вопрос — не провал: фраза и лицо не грустят
  (``respond.failed``);
* невалидная команда (после повтора моста) — допустимые действия исполняются, итог (или честное «не понял») говорит
  код, ход ``done`` (#3953, Шифу 23.10); спасённый ``query`` возвращает данные модели, пока есть итерации, иначе код
  говорит по данным (#4050); ``speech`` рядом с тихими действиями (``knowledge.QUIET_WITH_SPEECH_TOOLS``) законен;
  ``passed`` — тихо;
* форма команды хода (:func:`command_form`, #4021): ``passed`` есть в схеме только при подхвате бэклога (адресованную
  реплику модель молчанием не закрывает — нарушение и повтор моста), голос реплики — только по просьбе о голосе;
  путь пения (#4079) решает хост до модели (:func:`sing_route`: ``live:song`` — поёт сам, ``perform`` обязателен,
  генерация и трек вне среза; ``generate:…`` — ``generate_music`` обязателен, песня выступлением — нарушение);
* голый неслышимый ответ (``play_animation`` без слов) на адресованную реплику, где анимацию не просили
  (``tools_view.needs_words``, #3995), не исполняется: модель получает нарушение ``VIOLATION_SILENT_ANSWER``
  следующей итерацией (≤ ``COMMAND_RETRIES_MAX`` повторов); повтор кончился — анимация и фраза кода, не молчание;
  команда из одной ``expression`` — тот же класс (#3997): по прямой просьбе («покажи, как тебе грустно») лицо молча,
  иначе повтор; человек сам просил без слов («молча», «без слов», ``tools_view.silent_asked``, #4083) — лицом и
  анимацией одинаково без речи и без нарушения;
* экспрессия команды (эмоция, жест, сцена лица LLM) красит рефлекс итога ``done`` (:func:`done_expression` →
  ``Ports.stage(..., expression=)`` → ``expression.merge``), отдельного запроса лицу нет; исполненный ``play_animation``
  красит его своей сценой, сценой лица и временем (:func:`shown_face`, #4049: тул лицо не трогает); в тишине и у
  отменённого хода — рефлекс как есть (#3997); показ по просьбе держится ``expression.show_seconds`` (≥
  ``FACE_SHOW_MIN_S``, а не 3 с рефлекса), пачка ``play_animation`` — очередью (``Shown.then``, #4083);
* действия — владельцам; ``speech`` при действиях не звучит (§4.1), звучит фраза кода об исходе и ``perform``;
* ``query`` — данные обратно модели, не больше ``knowledge.LLM_QUERY_ITERATIONS_MAX`` итераций, и рядом с действиями
  тоже (#4065: ``[play_animation, music_library list]`` — действия исполнены, об их исходе говорит код, модели уходят
  данные ``query``); последний вызов модели идёт без query-тулов в схеме (читать больше нечем — остаётся сказать итог),
  а если она всё же просит query и её слов не слышно — ход ``done`` с фразой кода по данным (``query_fallback``), а не
  молчащий ``failed`` (#3953, В21); чтение памяти,
  отклонённое кодом без собеседника, на вопрос о памяти о самом человеке — фраза кода (``memory.voiced_refusals``,
  ревью #4053), на остальные реплики — данные модели, и она отвечает на вопрос (ревью PR #4055);
* ход ведётся до конца, пока провайдер жив, но не дольше ``TURN_CEILING_S`` (В31): ~``PROGRESS_AFTER_S`` тишины —
  одна промежуточная фраза кода (``understand.progress``/``media.progress``); потолок в ожидании LLM — фраза с именем
  провайдера, в ожидании действия — фраза его класса о таймауте (не «не понял»); цепочка провайдеров отказала —
  фраза сразу, без ожидания потолка; строка ``provider_latency``;
* ясный стоп — перебивание (``interrupt.py``, PR-9): речь и выступление гаснут по слову («стой» — сразу, «хватит» —
  на границе предложения), эпоха + 1; отменённый ход старой эпохи — ``cancelled``, а доживший до ответа модели —
  молчит и ничего не исполняет (``stale_epoch``).
"""

from __future__ import annotations

import inspect
import json
import logging
import random
import time
import uuid
from dataclasses import dataclass, field, fields, replace
from typing import Any, Awaitable, Callable, List, Mapping, Optional, Protocol, Sequence, Tuple

from rob_box_core.tool_catalog import get_tool
from rob_box_core.utterance import Sink, Utterance
from rob_box_dialog import face_scene
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
from rob_box_dialog.grammar import Tier1Command, match
from rob_box_dialog.interrupt import Interrupt, interrupt_for, stop_music_calls, stops_music
from rob_box_dialog.respond import ActionResult, asks, audible, classed, failed, join, outcome_emotion
from rob_box_dialog.session import Session, SessionSnapshot
from rob_box_dialog.speaker_adapter import speaker_block
from rob_box_dialog.tools_view import available_tools, needs_words, show_asked, voice_asked
from rob_box_dialog.turn_log import PERFORM_STARTED, TOOL, USER, TurnAggregate, TurnLog, phrase_label, speaker_label

_LOG = logging.getLogger(__name__)
#: Строка лога хода v2 (вход, решение, итог).
LOG_MARK = "🧭 [turn-v2]"
_QUERY = "query"
_FAILED = ("rejected", "timeout")
#: Строка лога подхвата бэклога голым «робот» (#3767, #3892).
PICKUP_MARK = "🗒️ [backlog] pickup"
#: Метка строк лога окна вопроса тула (#4062): «открыт» после вопроса, «ответ» на реплику в окне.
ASK_MARK = "❓ [tool-ask]"
#: Раздел снимка с исходами действий прошлых ходов (окно ``turn_log``: ``tool`` — структурой, не в тексте реплик).
RECENT_ACTIONS = "recent_actions"
#: Раздел снимка хода с фразой: фоновая фраза того же собеседника (``Backlog.recent_unaddressed``, §7.3, #3894).
RECENT_UNADDRESSED = "recent_unaddressed"
#: Раздел снимка хода с фразой: все фоновые фразы всех авторов за последние ``BACKGROUND_WINDOW_S`` (В23, 23.10).
BACKGROUND_SINCE_REPLY = "background_since_reply"


def host_sections(snap: SessionSnapshot, music: Any, providers: Sequence[Any] = (),
                  noted: Tuple[Optional[str], Optional[Mapping[str, Any]]] = (None, None)) -> Mapping[str, Any]:
    """Разделы снимка хода Личности из владельцев: собеседник (``speaker_block``), провайдеры, музыка
    (``MusicStateMemory.section``) — данные, не правила. Один сборщик для ROS-хоста ``dialog_node`` и сухого стенда
    (``scripts/dialog/dry_marathon.py``, #3906). ``noted`` — ``(person_ref, speaker_memory)``, взятые на начале хода
    (#3919): раздел показывается, только если собеседник снимка тот же (сменился — чужого не показываем)."""
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
              expression: Optional[Expression] = None) -> None:
        """Событие стадии и рефлекс. ``expression`` (необязательный, #3997) — экспрессия команды на итоге ``done``
        (:func:`done_expression`): красит рефлекс. Порт без этого параметра (чат, шлем оператора — лица у них нет)
        зовётся прежним вызовом (:func:`finish_stage`)."""
        ...

    def cancel_speech(self, epoch: int, reason: str, boundary: str = "as_requested") -> None: ...

    def stop_perform(self, how: str, keep_backing: bool = False) -> bool:
        """Выступление: ``halt`` — сразу, ``stop`` — дочитать предложение. ``True`` — играющая музыка — его бит
        (её снимает само выступление, общий стоп музыки не нужен). ``keep_backing`` (необязательный, #3994: «а
        музыку оставь») — бит выступления доигрывает сам; порт без этого параметра зовётся прежним вызовом
        (:func:`stop_performance`)."""
        ...

    def intent(self, turn_id: str, epoch: int, kind: str) -> None: ...

    def perform(self, turn_id: str, epoch: int, perf: Performance, edit: bool) -> Optional[Mapping[str, Any]]:
        """Выступление — его владельцу (хост ``PerformBlock``: сегменты на тактах, подложка, паузы по сетке строк);
        исход владельца (``pending``/``done``/``rejected``). ``None`` — владельца выступлений у агента нет (чат,
        оператор): сегменты звучат репликами хода."""
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
    heard: Tuple[Tuple[str, str], ...] = ()  # подхват бэклога: (кто сказал, фраза) по порядку (#3892)
    recent: Tuple[Mapping[str, Any], ...] = ()  # фоновая фраза того же собеседника для снимка (#3894)
    background: Tuple[Mapping[str, Any], ...] = ()  # фоновые фразы всех с последнего ответа робота (В23)
    said: List[str] = field(default_factory=list)
    queried: Tuple[ActionResult, ...] = ()  # исходы выполненного в этом ходе query (для страховки от молчания, #3883)
    waiting: str = "llm"  # чего ждёт ход: ответа LLM или события владельца (класс действия) — фразы В31
    retried: int = 0  # повторы модели за голый неслышимый ответ (``VIOLATION_SILENT_ANSWER``, #3995)
    expression: Optional[Expression] = None  # экспрессия исполненной команды — красит рефлекс ``done`` (#3997)
    command: str = ""  # запасной разбор Tier-1 (:func:`tier1_of`): слова после вейка (``Addressed.command``, n811)


def picked_heard(addressed: Addressed) -> Tuple[Tuple[str, str], ...]:
    """Подхваченные фразы бэклога ``(кто, фраза)``: автор — метка самой фразы (``Backlog.label``), не собеседник
    хода."""
    return tuple((phrase_label(p.get("speaker") or {}), str(p.get("text") or "")) for p in addressed.picked)


def recent_section(addressed: Addressed) -> Tuple[Mapping[str, Any], ...]:
    """Раздел ``recent_unaddressed`` адресованного хода: метка фразы бэклога + ``who`` — автор той же схемой, что
    тег ``kn.SPEAKER_TAG`` подхвата (:func:`phrase_label`, #3892)."""
    return tuple({**p, "who": phrase_label(p.get("speaker") or {})} for p in addressed.recent)


def background_section(addressed: Addressed) -> Tuple[Mapping[str, Any], ...]:
    """Раздел ``background_since_reply``: кто (:func:`phrase_label`), что, сколько секунд назад — и только это."""
    return tuple({"who": phrase_label(p.get("speaker") or {}), "text": p.get("text", ""), "age_s": p.get("age_s")}
                 for p in addressed.background)


def user_text(turn: "_Turn") -> str:
    """Реплика человека в запросе хода: подхват — фразы с тегом авторства (#3892), иначе — текст хода."""
    if not turn.heard:
        return turn.text
    return "\n".join(kn.SPEAKER_TAG.format(who=who, text=text) for who, text in turn.heard)


def log_pickup(log: Any, addressed: Addressed, turn: "_Turn") -> None:
    """Голое «робот»: строка подхвата — сколько фраз, чьи, в каком ходе (``n=0`` — бэклог пуст или фразы старше
    окна, ход — только рефлекс)."""
    if addressed.pickup or addressed.source == BARE_WAKE:
        speakers = ",".join(who for who, _ in turn.heard) or "-"
        log.info(f"{PICKUP_MARK} n={len(turn.heard)} speakers={speakers} turn={turn.turn_id}")


def tier1_for(text: str, snapshot: SessionSnapshot, voices: Sequence[Tuple[str, str]] = ()) -> Optional[Tier1Command]:
    """Команда Tier-1 реплики, если она доступна агенту сессии (``AgentSpec.grammar``): «замолчи» в чате — не
    тишина Личности (решение О2), а слова для LLM телеграм-агента."""
    command = match(text, snapshot, voices)
    return command if command is not None and allows_tier1(agent_spec(snapshot.agent), command.kind) else None


def tier1_of(text: str, after_wake: str, snapshot: SessionSnapshot,
             voices: Sequence[Tuple[str, str]] = ()) -> Tuple[Optional[Tier1Command], str]:
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
    """Действия Tier-1 владельцам: громкость — тул канала, голос — ``set_voice``, «говори нормально» — сброс скорости и
    тона (``set_speed``/``set_pitch`` ``normal``, #4022), очередь показа — ``play_animation`` на каждую сцену по
    порядку реплики (#4103: та же дорога исполнения, что у вызовов модели; окно LLM видит исход), стоп всего — музыка,
    если играет (:func:`~rob_box_dialog.interrupt.stops_music`); стоп речи (``stop_speech``) и оговорка «музыку
    оставь» (#3994) музыку не трогают."""
    if command.kind == "volume":
        tool = kn.VOLUME_CHANNELS[command.args["channel"]].tool
        return (ToolCall(tool, {"action": command.args["action"]}),) if tool else ()
    if command.kind == "set_voice":
        return (ToolCall("set_voice", {"voice": command.args["voice"]}),)
    if command.kind == kn.PROSODY_NORMAL_KIND:
        return tuple(ToolCall(tool, {"action": kn.PROSODY_NORMAL_ARG}) for tool in kn.PROSODY_RESET_TOOLS)
    if command.kind == kn.SHOW_QUEUE_KIND:  # #4103: сцены по порядку реплики, время - по умолчанию показа
        return tuple(ToolCall(kn.SHOW_TOOL, {"animation": name}) for name in command.args["scenes"])
    return stop_music_calls(snapshot) if stops_music(command) else ()


def stop_performance(ports: Any, how: str, keep_backing: bool = False) -> bool:
    """Перебивание выступления через ``Ports.stop_perform``. ``keep_backing`` (#3994, «а музыку оставь») уходит в
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
    """Модель и после повтора моста с нарушением ``VIOLATION_PASSED_ADDRESSED`` (#4021) «молчит» на адресованную
    реплику — исход хода ``passed``: код мычит фразой ``PASSED_ADDRESSED_PHRASE``, ход ``finished/passed`` (решение
    Шифу 22.10), как и до формы хода; не «не понял» невалидной команды. Это только ГОЛЫЙ ``passed``: рядом с речью или
    действиями поле ``passed`` на адресованной реплике отбрасывает валидатор (``command._drop_closed_passed``, ревью
    #4053) - ответ исполняется и звучит, сюда такая команда не доходит и мычанием не теряется."""
    if isinstance(result, Invalid) and result.reason == kn.VIOLATION_PASSED_ADDRESSED:
        return Command(passed=kn.PASSED_INSISTED)
    return result


def command_form(turn: "_Turn", snapshot: SessionSnapshot, voices: Sequence[Tuple[str, str]] = (),
                 sing: str = "") -> Form:
    """Форма команды хода (#4021, ADR-0148 п.2): ``passed`` — только при подхвате бэклога (адресованную реплику
    молчанием не закрыть: схема его не содержит, валидатор моста шлёт нарушение); голос реплики — только когда человек
    в этой реплике просил о голосе (``tools_view.voice_asked``: группа голоса или названный голос реестра ``voices`` —
    пары ``(id, название)``, как у Tier-1); ``sing`` — путь пения, решённый кодом до модели (:func:`sing_route`);
    живое пение при играющей музыке (:func:`music_playing`) — поверх неё (``Form.sing_over``, n810)."""
    names = tuple(name for pair in voices for name in pair if name)
    return Form(passed=turn.pickup, voice=voice_asked(snapshot, turn.text, names), sing=sing,
                sing_over=bool(sing) and music_playing(snapshot))


def music_playing(snap: SessionSnapshot) -> bool:
    """Играет музыка (сет DJ или трек плеера) — по снимку хода: контекст чойсера пути пения и подложка живого пения."""
    return snap.top("DJ") is not None or snap.music.state == "playing"


#: Путь пения хода от хоста (#4079): ``(реплика, прошлые реплики человека в окне, играет ли музыка) -> id кандидата``
#: (``live:song``, ``generate:robot_song:V6``; «» — не пение). Строит ``rob_box_music.source.choose_sing``.
SingRouter = Callable[[str, Sequence[str], bool], str]


async def sing_route(engine: "TurnEngine", turn: "_Turn", snap: SessionSnapshot) -> str:
    """Путь пения до основной модели (#4079, решение Шифу 27.10): решает хост (маркеры слов, чойсер с контекстом), ход
    сужает по нему форму команды. Хоста пути пения нет (чат, оператор) или подхват бэклога — «»: форма не меняется.
    Чойсер может спросить модель (Jev/LLM при неясном разрыве) — вызов в пуле, петля хода не стоит. Исключение хоста
    ход не роняет (ревью #4071 N5): строка WARNING и ход без формы пения, как без хоста."""
    import asyncio  # noqa: PLC0415 — asyncio тянет socket: импорт пакета чистый (test_purity)

    if engine._sing is None or turn.pickup:
        return ""
    recent = tuple(str(e.content) for e in engine.log.window(exclude=turn.turn_id) if e.role == USER)
    try:
        return await asyncio.get_running_loop().run_in_executor(None, engine._sing, turn.text, recent,
                                                                music_playing(snap))
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
    """Команда модели не слышна, а человек ждёт слов (#3995): агент говорит в динамики (Личность, не шлем оператора и
    не чат), реплика адресована - не подхват бэклога и не тишина (:func:`voices_passed`), и сама команда без слов
    (``tools_view.needs_words``: голая анимация, которую словами не просили)."""
    return (agent_spec(snapshot.agent).sink == Sink.SPEAKERS and voices_passed(pickup, snapshot)
            and needs_words(text, cmd))


def done_expression(turn: "_Turn", snapshot: SessionSnapshot) -> Optional[Expression]:
    """Экспрессия команды для рефлекса итога (#3997, ADR-0148: решает код): лицо команды красит ``done``. ``None`` —
    рефлекс как есть: ход не ``done`` или отменён (перебивание, ``stale_epoch``), в кадре ``Silence`` (лицо там —
    заклеенный рот, #3984), у команды нечего показать: ни сцены (``expression.shows``), ни очереди пачки за ней
    (``Shown.then``, #4083: пачка ``[talking, happy]`` - первой сцене нечего показать без речи, ``done`` - обычный
    рефлекс, а очередь всё равно идёт в свой срок, а не теряется вместе с первой сценой). Нейтраль показа (``idle``) -
    сцена: лицо покоя на своё время (``expression.shown_scene``, #4103), а не рефлекс ``happy``."""
    face = turn.expression
    if face is None or turn.stage != "done" or turn.status == "cancelled" or snapshot.top("Silence") is not None:
        return None
    queue = face.then if isinstance(face, Shown) else ()
    return face if shows(face.emotion, face.gesture, face.face) or queue else None


@dataclass(frozen=True)
class Shown(Expression):
    """Лицо итога после исполненных ``play_animation`` (#4049, #4083): сцена первого вызова (``emotion``), её сцена лица
    LLM и время ``duration_s`` (``expression.show_seconds``); ``then`` - сцены остальных вызовов той же команды, по
    очереди после неё, каждая своё время (``expression.queued``): пачка не перезаписывает друг друга. Не команда
    модели: схема команды - поля :class:`~rob_box_dialog.command.Expression`."""

    then: Tuple[Expression, ...] = ()


def _asked_seconds(duration: Any, face: Optional[face_scene.FaceScene]) -> Any:
    """Заказанное время сцены: ``duration`` вызова (или ``expression.duration_s``), иначе ``duration_s`` её сцены лица,
    иначе не задано (``None``) — тогда ``expression.show_seconds`` берёт время по умолчанию."""
    if duration is not None:
        return duration
    return face.duration_s if face is not None else None


def shown_face(expression: Optional[Expression], data: Mapping[str, Any]) -> Optional[Expression]:
    """Сцена одного исполненного ``play_animation`` (#4049; ADR-0150 PR-3b: тул лицо не трогает, показывает
    диалог). ``animation`` (тул уже свёл его к имени сцены лица) - сцена вместо эмоции и жеста команды (жест
    ``face_node`` показал бы поверх сцены) со своей сценой лица LLM (:class:`Shown`). Только ``face`` - сцена лица
    поверх экспрессии команды. Время - ``duration`` вызова, иначе ``duration_s`` его сцены лица, по правилу показа
    ``expression.show_seconds`` (#4083: не меньше ``FACE_SHOW_MIN_S``; сцена лица гаснет вместе со сценой). Ни того ни
    другого - экспрессия команды как есть."""
    animation, duration, raw = (data.get(key) for key in kn.SHOW_FIELDS)
    face = face_scene.validate(raw)[0] if raw is not None else None
    if not animation and face is None:
        return expression
    seconds = show_seconds(_asked_seconds(duration, face))
    face = replace(face, duration_s=seconds) if face is not None else None
    if animation:  # нейтраль - лицо покоя на своё время, а не рефлекс done (#4103)
        return Shown(emotion=shown_scene(str(animation)), face=face, duration_s=seconds)
    return replace(expression or Expression(), face=face, duration_s=seconds)


def _in_order(scenes: Sequence[Expression]) -> Expression:
    """Сцены исполненных ``play_animation`` по порядку вызовов: первая - лицо итога, остальные - его очередь
    (``Shown.then``), а не перезапись последней (#4083)."""
    first = scenes[0]
    return Shown(**{f.name: getattr(first, f.name) for f in fields(Expression)}, then=tuple(scenes[1:]))


def held_expression(expression: Optional[Expression], text: str) -> Optional[Expression]:
    """Лицо команды без ``play_animation`` (#4083): время - заказанное моделью (``expression.duration_s``) или, когда
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
    """Экспрессия исполненной команды для итога хода (#3997); исполненные ``play_animation`` красят итог своими
    сценами по очереди (:func:`shown_face`, #4049: «покажи, как тебе грустно» → ``sad``, а не рефлекс ``happy``;
    #4083: пачка — первая сцена на итоге, остальные в ``Shown.then``), иначе лицо команды со временем показа
    (:func:`held_expression`, ``text`` — реплика человека). Провал действия её не показывает (``None`` — рефлекс как
    есть): «радостно» о провале не бывает, как и в ``expression.merge``. Вопрос тула («Удалить «X»?», #4062) её тоже
    не показывает: это фраза кода, настроение модели её не красит. Одно правило для исполненной команды
    (``TurnEngine._speak``) и для действий, исполненных после нарушения (:func:`_close_invalid`).

    ``earlier`` - лицо хода после прошлых итераций (``turn.expression``): сцены ``play_animation``, исполненные в них,
    остаются на итоге, а сцены этой итерации встают очередью за ними (#4103, n905: ``[play_animation victory,
    get_current_time]`` → данные модели → речь; вторая итерация красила итог только своим результатом и затирала
    ``victory``, на лице оставался рефлекс ``done`` на 3 с). Та же сцена, повторённая моделью на следующей итерации
    (``[play_animation sad, list]`` до потолка итераций, #4065), в очередь второй раз не встаёт."""
    if any(failed(r) or asks(r) for r in results):
        return None
    shown = [shown_face(expression, r.data) for r in results if r.tool == kn.SHOW_TOOL and r.status == "done"]
    kept, fresh = _scenes_of(earlier), [s for s in shown if s is not None and s is not expression]
    if kept and fresh and fresh[0] == kept[-1]:  # та же сцена на следующей итерации - дважды не показываем
        fresh = fresh[1:]
    scenes = kept + fresh
    return _in_order(scenes) if scenes else held_expression(expression, text)


def finish_stage(ports: Any, turn: "_Turn", snapshot: SessionSnapshot) -> Optional[Expression]:
    """Итог хода в ``Ports.stage``. Экспрессия команды (:func:`done_expression`) уходит, только если порт объявил
    ``expression``: лицо есть у Личности, а чат и шлем оператора зовутся прежним вызовом (как ``stop_performance``).
    Возвращает лицо, которое порт получил (``None`` - рефлекс как есть или порт лица не принимает): ход записывает его в
    окно (``TurnLog.faced``, #4073), и запись «показано» не опережает показ - у чата и шлема оператора лица нет."""
    face = done_expression(turn, snapshot)
    shown = face if face is not None and _accepts(ports.stage, "expression") else None
    extra = {"expression": shown} if shown is not None else {}
    ports.stage(turn.turn_id, turn.epoch, turn.stage, turn.status, turn.reason, llm_calls=turn.llm_calls,
                provider=turn.provider, **extra)
    return shown


def prosody_phrase(results: Sequence[ActionResult]) -> str:
    """Фраза «говори нормально» (#4022) из РЕЗУЛЬТАТОВ ``set_speed``/``set_pitch`` (ADR-0148): оба сброса удались -
    одна фраза; хоть один нет - фразы исходов тулов (``respond.join``): что встало на норму и что не вышло
    (провайдер без регулировки - ``local.rejected.*_unsupported``). Результатов нет (ход отброшен) - молчание."""
    if results and all(r.status == "done" for r in results):
        return phrases.text(kn.PROSODY_NORMAL_PHRASE)
    return outcome_phrase(results) or ""


def show_queue_phrase(results: Sequence[ActionResult]) -> Optional[str]:
    """Фраза очереди показа (#4103): все сцены приняты - слов нет, ответ - лицо (прямая просьба показать, решение Шифу
    26.10; речь рисовала бы рот поверх первой сцены). Хоть одна не принята - фраза исхода по РЕЗУЛЬТАТУ тула
    (``respond.join``), а не молчание."""
    if all(r.status == "done" for r in results):
        return None
    return outcome_phrase(results) or phrases.text("local.rejected")


def tier1_face(command: Tier1Command, results: Sequence[ActionResult], text: str) -> Optional[Expression]:
    """Лицо итога хода Tier-1: у очереди показа (#4103) - сцены исполненных ``play_animation`` по порядку реплики
    (:func:`command_face`: первая на итоге ``done``, остальные - ``Shown.then``, каждая своё время
    ``expression.show_seconds``); провал - рефлекс как есть. У остальных видов лица команды нет."""
    return command_face(None, results, text) if command.kind == kn.SHOW_QUEUE_KIND else None


def tier1_phrase(command: Tier1Command, results: Sequence[ActionResult],
                 pick: Callable[[str], str] = lambda key: "") -> Optional[str]:
    """Фраза Tier-1: громкость — по исходу тула; сброс скорости и тона — по исходам тулов (:func:`prosody_phrase`);
    новая сессия — вариант из каталога; очередь показа — лицом, слова только о провале (:func:`show_queue_phrase`);
    стоп и тишина молчат; голос говорит сам тул (SpeechEvent)."""
    if command.kind == "new_session":
        return pick(kn.NEW_SESSION_PHRASE)
    if command.kind == kn.PROSODY_NORMAL_KIND:
        return prosody_phrase(results)
    if command.kind == kn.SHOW_QUEUE_KIND:
        return show_queue_phrase(results)
    if command.kind != "volume":
        return None
    prefix = "music" if command.args["channel"] == "music" else "voice"
    ok = bool(results) and all(r.status == "done" for r in results)
    return phrases.text(f"{prefix}.volume.{command.args['action']}" if ok else f"{prefix}.volume_fail")


def hold_silence(engine: "TurnEngine", turn: "_Turn") -> bool:
    """Тишина (кадр ``Silence``): после Tier-1 ход не идёт к LLM и ничего не говорит (#3982, решение Шифу 23.10: робот
    молчит, снимает только «Робот, говори»; вейк тишину не снимает). Реплика уже в журнале хода (``TurnLog.begin``), здесь
    - строка лога с причиной, метка «рот заклеен» в окне LLM после реплики (``TurnLog.mark``, #4106: без неё реплика
    висит без ответа) и итог хода ``silenced``/``finished``: не провал и не «не понял»."""
    if engine.session.snapshot.top("Silence") is None:
        return False
    engine.log.mark(turn.turn_id, kn.WINDOW_MUTED_MARK)
    engine._log.info(f"{LOG_MARK} {kn.SILENCED_TURN_REASON}: LLM не зовётся, речи нет text={turn.text[:80]!r} "
                     f"turn={turn.turn_id}")
    turn.stage, turn.status, turn.reason = kn.SILENCED_TURN_STAGE, "finished", kn.SILENCED_TURN_REASON
    return True


async def understand(engine: "TurnEngine", turn: "_Turn", snap: SessionSnapshot) -> None:
    """Ход к LLM (``TurnEngine._understand``), если его не проглотила тишина (:func:`hold_silence`), — в форме команды
    хода (:func:`command_form`, #4021): схема и валидатор моста видят её на время вызовов LLM этого хода."""
    if hold_silence(engine, turn):
        return
    form = command_form(turn, snap, engine._voices(), await sing_route(engine, turn, snap))
    over = " поверх играющего" if form.sing_over else ""
    engine._log.info(f"{LOG_MARK} форма команды: закрыты={list(form.closed()) or '-'} пение={form.sing or '-'}{over} "
                     f"turn={turn.turn_id}")
    with turn_form(form):
        await engine._understand(turn, snap)


def outcome_phrase(results: Sequence[ActionResult]) -> Optional[str]:
    """Фраза кода об исходе действий хода (``respond.join``): без ``query`` и без тулов, которые говорят сами
    (``knowledge.OUTCOME_SPOKEN_ELSEWHERE == "tool"``). Слота нет в данных события — общая фраза ``local.*``."""
    own = [r for r in results if r.cls != _QUERY and kn.OUTCOME_SPOKEN_ELSEWHERE.get(r.tool) != "tool"]
    try:
        said = join(own)
    except KeyError:
        lost = [r for r in own if r.status in _FAILED]
        return phrases.text(f"local.{lost[0].status}") if lost else None
    return said.text if said is not None else None


def _spoken_outcome(cmd: Command, results: Sequence[ActionResult], agent: str,
                    performing: bool = False) -> Optional[str]:
    """Фраза кода об исходе хода; речь модели рядом с тихими действиями (в чат телеграма она не звучит) молчит об их
    простом успехе (``respond.audible``, #3953), а при идущем выступлении (``performing``) молчит общий успех запуска
    музыки (#4075). Исходов ``query`` здесь нет: пока итерации есть, они уходят модели (:func:`query_more`, там же отказ
    чтения памяти), на последней ``query`` не исполняется (:func:`_runnable`)."""
    speaks = bool(cmd.speech) and agent != kn.TEXT_CHANNEL_AGENT
    return outcome_phrase(audible(results, speaks, performing)) if results else None


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
    """Выступление команды — его владельцу (``Ports.perform``: такты, подложка, паузы по сетке строк). #3901: v2 читал
    сегменты рэпа репликами хода — без бита и без пауз. ``(взял ли владелец, фраза кода об отказе или "")``; не взял
    (выступления нет, у агента нет владельца) — сегменты звучат репликами хода."""
    if cmd.perform is None:
        return False, ""
    outcome = ports.perform(turn_id, epoch, cmd.perform, cmd.edit_pending)
    if outcome is None:
        return False, ""
    log.info(f"{LOG_MARK} выступление {'правка' if cmd.edit_pending else 'старт'} kind={cmd.perform.kind} "
             f"backing={cmd.perform.backing or '-'} segments={len(cmd.perform.segments)} → {dict(outcome)} "
             f"turn={turn_id}")
    return True, phrases.text("local.rejected") if outcome.get("status") == "rejected" else ""


def note_perform_start(log: TurnLog, turn_id: str, cmd: Command, taken: bool) -> None:
    """Владелец взял выступление (``taken``): в окне LLM просьба закрыта записью о старте, дальше её обновляет конец
    выступления (``TurnLog.performance_ended``, #3973). Сегменты звучат у владельца мимо реплик хода - без записи
    просьба висела бы неотвеченной и исполнялась заново на чужую реплику."""
    if taken and cmd.perform is not None:
        log.performed(turn_id, cmd.perform.kind, PERFORM_STARTED)


def _runnable(actions: Sequence[ToolCall], last: bool) -> Tuple[ToolCall, ...]:
    """Действия к исполнению: на последней итерации ``query`` уже не исполняется (данные некому вернуть). Класс -
    вызова (``knowledge.call_class``): ``music_library`` list/search/info - ``query``."""
    return tuple(a for a in actions if not last or kn.call_class(a.tool, a.args) != _QUERY)


def _offered(tools: Sequence[Any], last: bool) -> Tuple[Any, ...]:
    """Тулы вызова LLM хода. На последнем ``query`` убраны из схемы (#3953): данные ему уже вернули (живой прогон
    23.10: модель искала память трижды, потолок ``LLM_QUERY_ITERATIONS_MAX`` съел последний вызов, и ход молчал).
    Выбор сужает структура — читать больше нечем, остаётся сказать итог или сделать действие."""
    return tuple(t for t in tools if not last or kn.TOOL_CLASSES.get(t.name) != _QUERY)


def _close_unread(turn: _Turn, acted: bool) -> str:
    """Модель на последней итерации ещё просит ``query`` (его уже не исполняют, :func:`_runnable`), а её слов не
    слышно: ответа она не дала. Запрос в ходе был - ход ``done``, говорит код по данным запроса (#3953, В21: на
    адресованное не молчим; #4065: ``[play_animation, music_library list]`` - анимация исполнена, фраза кода по данным).
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
    отвечает по ним (ADR-0150 §4.1 query-цикл, §5.1). Действия той же команды (#4065: ``[play_animation,
    music_library list]``; #4050: спасённые) уже исполнены: об их исходе говорит код сразу (:func:`outcome_phrase`,
    провал - грустно, иначе ``emotion``), модели они не уходят - ``action.*`` терминальны, слов о них она не получает.

    Отказ чтения памяти до владельца (собеседника нет или голос не подтверждён) на вопрос о памяти о самом человеке
    (:func:`memory.voiced_refusals`) - не данные: о нём говорит код фразой ``query.rejected.<причина>`` первой в той
    же реплике (ревью #4053: «что ты обо мне помнишь?» спорным голосом - «не буду гадать, что помню», а не «запоминать
    не буду»; ревью PR #4055: и рядом с ``play_sound``); читать модели больше нечего - ``None``, ход закрыт фразой кода.
    На любую другую реплику отказ - данные модели, как до #4053, и ход продолжается (ревью PR #4055: «кто такой
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


async def _close_invalid(engine: "TurnEngine", turn: _Turn, invalid: Invalid, snap: SessionSnapshot,
                         expression: Optional[Expression] = None, last: bool = True) -> Optional[str]:
    """Невалидная команда после повтора моста: на адресованную реплику не молчим (В21, решение Шифу 23.10, #3953).
    Запрос в ходе выполнен - говорит код по его данным (#3883). Иначе допустимые действия команды (``Invalid.salvage``)
    исполняются, об исходе говорит код (:func:`_salvaged_phrase`); исполнять нечего - честная фраза
    ``INVALID_COMMAND_PHRASE``. Ход ``done``. Среди спасённых есть ``query`` (класс ВЫЗОВА: ``music_library list``,
    #4050) и итерации ещё есть (не ``last``) - его данные возвращаются модели тем же путём, что у исполненной команды
    (:func:`query_more`), и она отвечает словами; итераций нет - фраза кода по данным (:func:`query_fallback`). Все
    спасённые ``query`` - озвученные отказы чтения памяти (``memory.voiced_refusals``): фразу сказал :func:`query_more`,
    читать модели нечего - ход ``done`` здесь же (ревью PR #4067: итог ``addressed`` давал рефлекс «услышал»).
    ``expression`` - лицо команды, исполняемой после нарушения ``silent_answer`` (#3997): красит итог вместе с её
    действиями, если они не провалились (:func:`command_face`)."""
    log, why = engine._log, invalid.reason
    if turn.queried:
        log.warning(f"{LOG_MARK} невалидная команда {why} после query → фраза кода по данным запроса")
        turn.stage, turn.status, turn.reason = "done", "done", f"query_fallback:{why}"[:60]
        engine._say(turn, query_fallback(turn.queried), None, invalid_emotion(invalid, ()))
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
    слов; ревью #4041 — «Угу, я тут.» звучало с ``sad``)."""
    emotion = outcome_emotion(results)  # провал - sad, вопрос тула - эмоция вопроса (#4062)
    if emotion is not None:
        return emotion
    return None if results or invalid.reason in kn.CALM_VIOLATIONS else "sad"


def _salvaged_phrase(engine: "TurnEngine", turn: _Turn, invalid: Invalid, results: Sequence[ActionResult]) -> str:
    """Фраза кода после невалидной команды: об исходе исполненного; успех звука молчит (звук сам себе доказательство) -
    тишина честна. Спасённый ``query`` на последней итерации - его данные читать некому, говорит код по ним
    (:func:`query_fallback`, #4050: «что у тебя в библиотеке?» не молчит). Исполнять было нечего или исполненного не
    слышно, а человек ждёт слов (:func:`words_owed`, #3995: голая анимация) - фраза причины
    (``INVALID_COMMAND_PHRASES``), не молчание."""
    said = outcome_phrase(results) if results else None
    queried = [r for r in results if r.cls == _QUERY]
    if queried:
        return " ".join(text for text in (said, query_fallback(queried)) if text)
    snap = engine.session.snapshot
    if said or (results and not words_owed(snap, turn.pickup, turn.text, Command(actions=tuple(invalid.salvage)))):
        return said or ""
    return engine._pick(kn.INVALID_COMMAND_PHRASES.get(invalid.reason, kn.INVALID_COMMAND_PHRASE))


async def _answer_in_words(engine: "TurnEngine", turn: _Turn, cmd: Command, snap: SessionSnapshot,
                           last: bool) -> Optional[str]:
    """Голый неслышимый ответ на адресованную реплику (#3995, ADR-0148: решает код). Повтор есть (не последняя
    итерация, ≤ ``COMMAND_RETRIES_MAX``) - команда не исполняется, модели уходит нарушение ``VIOLATION_SILENT_ANSWER``
    в форме ``command.explain`` (следующая итерация ``_understand``). Повтора нет - анимация исполняется, а говорит код
    (:func:`_close_invalid` с этой причиной), не молчание."""
    tools = [a.tool for a in cmd.actions] or [f"expression:{cmd.expression.emotion}"]
    if not last and turn.retried < kn.COMMAND_RETRIES_MAX:
        turn.retried += 1
        engine._log.warning(f"{LOG_MARK} {kn.VIOLATION_SILENT_ANSWER}: {tools} без слов на адресованную реплику → "
                            f"повтор с нарушением turn={turn.turn_id}")
        return explain(Invalid(kn.VIOLATION_SILENT_ANSWER, kn.SILENT_ANSWER_HINT), after_query=bool(turn.queried))
    engine._log.warning(f"{LOG_MARK} {kn.VIOLATION_SILENT_ANSWER}: {tools} без слов и после повтора → анимация и "
                        f"фраза кода turn={turn.turn_id}")
    # лицо команды, как и её анимация, показывается — говорит код (#3997)
    await _close_invalid(engine, turn, Invalid(kn.VIOLATION_SILENT_ANSWER, salvage=cmd.actions), snap, cmd.expression)
    return None


def query_fallback(results: Sequence[ActionResult]) -> str:
    """Фраза кода, когда запрос хода выполнен, а ответ модели не прошёл проверку и после повтора (#3883, решение Шифу
    23.10): по данным результата тула (``knowledge.QUERY_FALLBACK_PHRASES``), иначе общая честная фраза. «Нашёл ответ»
    - только когда хоть один запрос ``done``: ни одного (``rejected``/``timeout``: спасённый ``music_library list``
    отклонён) - фраза его исхода по результату (``respond.join``: «Не нашёл ответа.», «Не дождался ответа…»), а не
    «нашёл» (ревью PR #4055)."""
    if not any(r.status == "done" for r in results):
        failed = join(results)  # у класса query фраза есть на каждый не-done исход (rejected, timeout)
        return failed.text if failed is not None else phrases.text("query.rejected")
    for r in results:
        key = kn.QUERY_FALLBACK_PHRASES.get(r.tool)
        summary = _voices_summary(r.data) if key and r.status == "done" else None
        if key and summary:
            return phrases.text(key, summary=summary)
    return phrases.text(kn.QUERY_FALLBACK_GENERIC)


async def execute_for_speaker(ports: Ports, actions: Sequence[ToolCall], snap: SessionSnapshot,
                              now: SessionSnapshot, turn_id: str, log: Any = _LOG) -> List[ActionResult]:
    """Действия владельцам; исход ``memory_save``, который решает код, — без запроса: при смене собеседника с начала
    хода ``rejected{speaker_changed}`` (чей это факт, #3881); факт, лишь повторяющий имя ``register_speaker`` этой же
    команды (``memory.name_fact``, #4054: имя живёт в профиле голоса), - исход этой регистрации с причиной
    ``name_in_profile`` (``memory.name_outcome``): ``done`` только по ``done`` владельца голосов. Результаты — в порядке
    команды, с классом ВЫЗОВА (``respond.classed``): исполнитель клеит класс тула, а ``music_library``
    list/search/info - ``query`` (#4020)."""
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
    ``timeout`` раньше, чем дедлайн хода (``TurnDeadline.grant``) оборвёт его «долго думаю» (#3960)."""
    return max(deadline_s(kn.call_class(a.tool, a.args)) for a in actions) + kn.ACTION_GRACE_S


def say_late(engine: "TurnEngine", result: ActionResult) -> None:
    """Итог действия после закрытия его хода (``Executor.on_late``): фраза кода по событию владельца, если класс из
    ``LATE_SPOKEN_CLASSES`` (музыка заиграла после ``timeout`` - «Включаю музыку.»); тихие успехи и чужие классы
    молчат (#3960)."""
    result = classed(result, result.data)  # аргументы вызова лежат в данных результата поверх ответа тула
    late = result.cls in kn.LATE_SPOKEN_CLASSES
    text = outcome_phrase(audible([result], False, engine.log.performing())) if late else None  # #4075: рэп не рвём
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
    """Чем ответить на адресованную реплику: ответ на открытый вопрос тула (``tool_ask.reply``, #4062) и Tier-1 -
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
    """Ответ человека на вопрос тула (#4062, ADR-0148: решает код, 0 вызовов LLM). Окно снимается сразу. «Да» -
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
        engine._say(turn, text)


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
                 voices: Optional[Callable[[], Sequence[Tuple[str, str]]]] = None, turn_log: Optional[TurnLog] = None,
                 events: Any = None, new_id: Callable[[], str] = lambda: uuid.uuid4().hex[:12],
                 clock: Callable[[], float] = time.monotonic, deadline_s: float = kn.TURN_CEILING_S,
                 log: Optional[logging.Logger] = None, rng: Optional[random.Random] = None,
                 sing: Optional[SingRouter] = None) -> None:
        self.session = session
        self._sing = sing  # путь пения от хоста (#4079); нет — форма хода без пути пения
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
                     background=background_section(addressed), command=addressed.command)
        self.log.begin(turn.turn_id, turn.epoch, turn.text, speaker_label(snap.speaker), turn.heard)
        log_pickup(self._log, addressed, turn)
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
        self.log.mark(turn.turn_id, kn.WINDOW_SILENCE_MARKS.get(command.kind, ""))  # тишина вкл/выкл без слов (#4106)
        results = await self._execute(turn, actions, snap)
        turn.expression = tier1_face(command, results, turn.text)  # очередь показа красит итог done (#4103)
        said = tier1_phrase(command, results, self._pick)
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
        (:func:`query_more`), и рядом с действиями тоже (#4065: ``[play_animation, music_library list]`` без речи
        молчал); слова команды, написанные до данных, не звучат — модель ответит по данным, и экспрессия такой команды
        итог не красит: красит исполненное (сцена ``play_animation``, :func:`command_face`), голый ``query`` лицо
        итога не трогает (ревью PR #4067). Голый неслышимый ответ на адресованную реплику — нарушение для модели
        вместо исполнения (#3995)."""
        turn.stage = "done"
        if cmd.passed:
            turn.reason = "passed"
            self._log.info(f"{LOG_MARK} passed={cmd.passed} addressed={not turn.pickup}")
            # Решение Шифу 22.10: на адресованную реплику (вейк, окно ответа, Telegram) молчать нельзя — код мычит;
            # passed из бэклога (подхват накопленного) — тихо.
            if voices_passed(turn.pickup, self.session.snapshot):
                self._say(turn, self._pick(kn.PASSED_ADDRESSED_PHRASE))
            return None
        if words_owed(self.session.snapshot, turn.pickup, turn.text, cmd):
            return await _answer_in_words(self, turn, cmd, snap, last)
        runnable = _runnable(cmd.actions, last)
        results = await self._execute(turn, runnable, snap)
        if self._stale(turn, "results"):
            return None
        if not last and any(r.cls == _QUERY for r in results):
            turn.expression = command_face(turn.expression, results, turn.text, earlier=turn.expression)
            return query_more(self, turn, results, cmd.expression.emotion)
        self._speak(turn, cmd, results, unread=len(runnable) < len(cmd.actions))
        return None

    def _speak(self, turn: _Turn, cmd: Command, results: Sequence[ActionResult], unread: bool = False) -> None:
        """Сначала фраза кода об исходе (провал — грустно), затем слова модели, что звучат (§4.1). Экспрессия команды
        запоминается для рефлекса ``done`` (:func:`command_face`, #3997). ``unread`` — модель на последней итерации
        ещё просила ``query`` (не исполнен): слов её не слышно — говорит код по данным хода (:func:`_close_unread`)."""
        turn.expression = command_face(cmd.expression, results, turn.text, earlier=turn.expression)
        outcome = _spoken_outcome(cmd, results, self.session.snapshot.agent, self.log.performing())
        if outcome:  # провал - грустно, вопрос тула («Удалить «X»?», #4062) - эмоцией вопроса, иначе лицом модели
            self._say(turn, outcome, None, outcome_emotion(results, cmd.expression.emotion))
        owned, refusal = hand_perform(self._ports, turn.turn_id, turn.epoch, cmd, self._log)
        note_perform_start(self.log, turn.turn_id, cmd, owned and not refusal)
        self._say(turn, refusal, None, "sad")  # отказ владельца выступления — фраза кода, а не тишина ("" — нет)
        lines = _content_lines(cmd, self.session.snapshot.agent, segments=not owned)
        for text, voice in lines:
            self._say(turn, text, voice or None, cmd.expression.emotion)
        if cmd.ask is not None and not cmd.actions:
            self.session.set_pending_ask(cmd.ask)
        if unread and not lines and not owned:  # модель ещё читала (query сверх лимита), а её слов не слышно
            self._say(turn, _close_unread(turn, acted=bool(results)), None, "sad")

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
        for result in results:
            self.log.result(turn.turn_id, result)
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
        """Запрос: системный промпт, окно озвученного (``user``/``assistant``), снимок хода + реплика человека.
        Исходы прошлых действий — разделом снимка :data:`RECENT_ACTIONS`, а не репликами (§7.3)."""
        past = self.log.window(exclude=turn.turn_id)  # реплика этого хода — последним сообщением, со снимком
        history = [e for e in past if e.role != TOOL]
        sections = dict(self._sections(snap, turn.text))
        recent = [dict(e.content) for e in past if e.role == TOOL]
        context = build_context({**sections, RECENT_UNADDRESSED: list(turn.recent),
                                 BACKGROUND_SINCE_REPLY: list(turn.background),
                                 RECENT_ACTIONS: recent[-kn.TURN_WINDOW_MAX_TURNS:]})
        messages: List[Any] = [{"role": "system", "content": self._system}] if self._system else []
        messages += [{"role": e.role, "content": e.for_llm()} for e in history]
        messages.append({"role": "user", "content": f"{context}\n\n{user_text(turn)}"})
        return messages

    def _say(self, turn: _Turn, text: str, voice: Optional[str] = None, emotion: Optional[str] = None) -> None:
        if not text or self._stale(turn, "speech"):
            return
        speech_id = self._ports.say(turn.turn_id, turn.epoch, text, voice, emotion)
        # реплика не в динамики (чат, шлем оператора) — SpeechEvent о ней не придёт: в окне она сразу, как отдана
        delivered = agent_spec(self.session.snapshot.agent).sink != Sink.SPEAKERS
        self.log.said(turn.turn_id, speech_id, text, delivered=delivered)
        turn.said.append(speech_id)
        self._event("speech_requested", turn, text=text, data={"speech_id": speech_id, "voice": voice or ""})

    def _finish(self, turn: _Turn) -> TurnAggregate:
        latency = int(turn.deadline.elapsed() * 1000)
        face = finish_stage(self._ports, turn, self.session.snapshot)
        if face is not None:  # ответ одним лицом (#4073): без записи в окне реплика человека висит без ответа
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
           "Shown", "TurnEngine", "command_face", "command_form", "done_expression", "finish_stage", "hand_perform",
           "held_expression", "host_sections", "shown_face",
           "invalid_emotion", "outcome_phrase", "query_fallback", "show_queue_phrase", "stop_performance",
           "tier1_actions", "tier1_face", "tier1_for", "tier1_phrase", "turn_work", "answer_tool", "voices_passed",
           "words_owed"]
