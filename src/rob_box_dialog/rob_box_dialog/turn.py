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
* Tier-1 — исполняет код (стоп, тишина, громкость, голос, новая сессия), спорный стоп (В19) идёт в LLM;
* невалидная команда (после повтора моста) — ход ``failed`` без речи; ``passed`` — тихо;
* действия — владельцам; ``speech`` при действиях не звучит (§4.1), звучит фраза кода об исходе и ``perform``;
* ``query`` — данные обратно модели, не больше ``knowledge.LLM_QUERY_ITERATIONS_MAX`` итераций;
* весь ход — в бюджете ``TURN_DEADLINE_S``: истёк — ``understand.timeout``, строка ``provider_latency``;
* ясный стоп — перебивание (``interrupt.py``, PR-9): речь и выступление гаснут по слову («стой» — сразу, «хватит» —
  на границе предложения), эпоха + 1; отменённый ход старой эпохи — ``cancelled``, а доживший до ответа модели —
  молчит и ничего не исполняет (``stale_epoch``).
"""

from __future__ import annotations

import json
import logging
import random
import time
import uuid
from dataclasses import dataclass, field
from typing import Any, Awaitable, Callable, List, Mapping, Optional, Protocol, Sequence, Tuple

from rob_box_core.utterance import Sink
from rob_box_dialog import knowledge as kn
from rob_box_dialog import memory
from rob_box_dialog import phrases
from rob_box_dialog.address import BARE_WAKE, Addressed
from rob_box_dialog.agents import agent_spec, allows_tier1
from rob_box_dialog.command import Command, Invalid, Performance, Result, ToolCall
from rob_box_dialog.context import build_context
from rob_box_dialog.deadline import TIMEOUT_PHRASE, DeadlineExpired, TurnDeadline, latency_line
from rob_box_dialog.grammar import Tier1Command, match
from rob_box_dialog.interrupt import Interrupt, interrupt_for, stop_music_calls
from rob_box_dialog.respond import ActionResult, join
from rob_box_dialog.session import Session, SessionSnapshot
from rob_box_dialog.tools_view import available_tools
from rob_box_dialog.turn_log import TOOL, TurnAggregate, TurnLog, phrase_label, speaker_label

_LOG = logging.getLogger(__name__)
#: Строка лога хода v2 (вход, решение, итог).
LOG_MARK = "🧭 [turn-v2]"
_QUERY = "query"
_FAILED = ("rejected", "timeout")
#: Строка лога подхвата бэклога голым «робот» (#3767, #3892).
PICKUP_MARK = "🗒️ [backlog] pickup"
#: Раздел снимка с исходами действий прошлых ходов (окно ``turn_log``: ``tool`` — структурой, не в тексте реплик).
RECENT_ACTIONS = "recent_actions"
#: Раздел снимка хода с фразой: фоновая фраза того же собеседника (``Backlog.recent_unaddressed``, §7.3, #3894).
RECENT_UNADDRESSED = "recent_unaddressed"
#: Раздел снимка хода с фразой: все фоновые фразы всех авторов за последние ``BACKGROUND_WINDOW_S`` (В23, 23.10).
BACKGROUND_SINCE_REPLY = "background_since_reply"


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
    """Транспорт хода (реализует ROS-хост)."""

    def ask(self, messages: Sequence[Any], tools: Sequence[Any], timeout_s: float) -> Awaitable[Answer]: ...

    def execute(self, actions: Sequence[ToolCall], snapshot: SessionSnapshot,
                turn_id: str) -> Awaitable[List[ActionResult]]: ...

    def say(self, turn_id: str, epoch: int, text: str, voice: Optional[str] = None,
            emotion: Optional[str] = None) -> str: ...

    def stage(self, turn_id: str, epoch: int, stage: str, status: str, reason: str = "",
              cls: Optional[str] = None, llm_calls: int = 0, provider: str = "", source: str = "") -> None: ...

    def cancel_speech(self, epoch: int, reason: str, boundary: str = "as_requested") -> None: ...

    def stop_perform(self, how: str) -> bool:
        """Выступление: ``halt`` — сразу, ``stop`` — дочитать предложение. ``True`` — играющая музыка — его бит
        (её снимает само выступление, общий стоп музыки не нужен)."""
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


def tier1_actions(command: Tier1Command, snapshot: SessionSnapshot) -> Tuple[ToolCall, ...]:
    """Действия Tier-1 владельцам: громкость — тул канала, голос — ``set_voice``, стоп — музыка, если играет."""
    if command.kind == "volume":
        tool = kn.VOLUME_CHANNELS[command.args["channel"]].tool
        return (ToolCall(tool, {"action": command.args["action"]}),) if tool else ()
    if command.kind == "set_voice":
        return (ToolCall("set_voice", {"voice": command.args["voice"]}),)
    if command.kind == "stop_all":
        return stop_music_calls(snapshot)
    return ()


def voices_passed(pickup: bool, snapshot: SessionSnapshot) -> bool:
    """«Промолчать» модели на адресованную реплику озвучивает код (Шифу 22.10); из бэклога и в тишине — молчание."""
    return not pickup and snapshot.top("Silence") is None


def tier1_phrase(command: Tier1Command, results: Sequence[ActionResult],
                 pick: Callable[[str], str] = lambda key: "") -> Optional[str]:
    """Фраза Tier-1: громкость — по исходу тула; новая сессия — вариант из каталога; стоп и тишина молчат; голос говорит
    сам тул (SpeechEvent)."""
    if command.kind == "new_session":
        return pick(kn.NEW_SESSION_PHRASE)
    if command.kind != "volume":
        return None
    prefix = "music" if command.args["channel"] == "music" else "voice"
    ok = bool(results) and all(r.status == "done" for r in results)
    return phrases.text(f"{prefix}.volume.{command.args['action']}" if ok else f"{prefix}.volume_fail")


def outcome_phrase(results: Sequence[ActionResult]) -> Optional[str]:
    """Фраза кода об исходе действий хода (``respond.join``): без ``query`` и без тулов, которые говорят сами
    (``knowledge.OUTCOME_SPOKEN_ELSEWHERE == "tool"``). Слота нет в данных события — общая фраза ``local.*``."""
    own = [r for r in results if r.cls != _QUERY and kn.OUTCOME_SPOKEN_ELSEWHERE.get(r.tool) != "tool"]
    try:
        said = join(own)
    except KeyError:
        failed = [r for r in own if r.status in _FAILED]
        return phrases.text(f"local.{failed[0].status}") if failed else None
    return said.text if said is not None else None


def _content_lines(cmd: Command, agent: str = kn.AGENTS[0], segments: bool = True) -> List[Tuple[str, str]]:
    """Что звучит из слов модели: реплика или вопрос — только без действий (§4.1), выступление — всегда. У агента с
    текстовым каналом (телеграм, §9.2) реплика при действиях — текст в чат, он не звучит, и правило §4.1 не нужно.
    ``segments=False`` — выступление взял его владелец (``Ports.perform``): сегменты звучат там, не репликами."""
    lines: List[Tuple[str, str]] = []
    said = cmd.speech or (cmd.ask.text if cmd.ask is not None else None)
    if said and (not cmd.actions or agent == kn.TEXT_CHANNEL_AGENT):
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


def _runnable(actions: Sequence[ToolCall], last: bool) -> Tuple[ToolCall, ...]:
    """Действия к исполнению: на последней итерации ``query`` уже не исполняется (данные некому вернуть)."""
    return tuple(a for a in actions if not last or kn.TOOL_CLASSES[a.tool] != _QUERY)


def _query_data(results: Sequence[ActionResult]) -> str:
    rows = [{"tool": r.tool, "status": r.status, "reason": r.reason, "data": dict(r.data)} for r in results]
    return json.dumps({"query_results": rows}, ensure_ascii=False, default=str)


async def execute_for_speaker(ports: Ports, actions: Sequence[ToolCall], snap: SessionSnapshot,
                              now: SessionSnapshot, turn_id: str) -> List[ActionResult]:
    """Действия владельцам; факт ``memory_save`` при смене собеседника с начала хода — ``rejected{speaker_changed}``
    без запроса (код решает, чей это факт, #3881). Результаты — в порядке команды."""
    changed = {i for i, a in enumerate(actions) if memory.speaker_changed(a, snap, now)}
    sent = [a for i, a in enumerate(actions) if i not in changed]
    done = iter(await ports.execute(sent, snap, turn_id) if sent else [])
    return [ActionResult(f"{turn_id}/{i}", snap.epoch, snap.agent, a.tool, kn.TOOL_CLASSES[a.tool], "rejected",
                         kn.SPEAKER_CHANGED_REASON) if i in changed else next(done) for i, a in enumerate(actions)]


async def _tier1_then_rest(engine: "TurnEngine", turn: _Turn, command: Tier1Command, snap: SessionSnapshot) -> None:
    """Ясный стоп с продолжением («хватит, спой другую», ``Tier1Command.rest``, §6.2): стоп — кодом, остаток фразы —
    новая реплика того же хода (Tier-1, если он ею покрыт, иначе LLM), а не потерянные слова (живой прогон 23.10,
    акт 4 n410: «ладно, хватит тянуть, теперь наоборот» — стоп, 0 вызовов LLM, тишина)."""
    await engine._tier1(turn, command, snap)
    if not command.rest:
        return
    turn.text = command.rest
    snap = engine.session.snapshot
    engine._log.info(f"{LOG_MARK} остаток после стопа: {turn.text[:80]!r} turn={turn.turn_id}")
    follow = tier1_for(turn.text, snap, engine._voices())
    if follow is not None and not follow.disputed:
        await _tier1_then_rest(engine, turn, follow, snap)
    else:
        await engine._understand(turn, snap)


class TurnEngine:
    """Ходы одного агента (личность): владелец сессии и журнала ходов — этот объект, транспорт — :class:`Ports`."""

    def __init__(self, session: Session, ports: Ports, *, system_prompt: str = "",
                 sections: Optional[Callable[[SessionSnapshot, str], Mapping[str, Any]]] = None,
                 voices: Optional[Callable[[], Sequence[Tuple[str, str]]]] = None, turn_log: Optional[TurnLog] = None,
                 events: Any = None, new_id: Callable[[], str] = lambda: uuid.uuid4().hex[:12],
                 clock: Callable[[], float] = time.monotonic, deadline_s: float = kn.TURN_DEADLINE_S,
                 log: Optional[logging.Logger] = None, rng: Optional[random.Random] = None) -> None:
        self.session = session
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
                     background=background_section(addressed))
        self.log.begin(turn.turn_id, turn.epoch, turn.text, speaker_label(snap.speaker), turn.heard)
        log_pickup(self._log, addressed, turn)
        self._event("user", turn, text=turn.text, data={"source": turn.source})
        self._ports.stage(turn.turn_id, turn.epoch, "addressed", "started", source=turn.source)
        if addressed.source == BARE_WAKE or not turn.text:
            return self._finish(turn)
        tier1 = tier1_for(turn.text, snap, self._voices())
        try:
            if tier1 is not None and not tier1.disputed:
                await _tier1_then_rest(self, turn, tier1, snap)
            else:
                await self._understand(turn, snap)
        except asyncio.CancelledError:
            turn.stage, turn.status, turn.reason = "done", "cancelled", "interrupt"
            self._log.info(f"{LOG_MARK} ход отменён перебиванием turn={turn.turn_id} epoch={turn.epoch}")
            self._finish(turn)
            raise
        except DeadlineExpired as expired:
            self._log.warning(latency_line(turn.provider, expired.what.split(":")[0], expired.waited_s * 1000,
                                           "timeout"))
            self._say(turn, phrases.text(TIMEOUT_PHRASE))
            turn.stage, turn.status, turn.reason = "timeout", "failed", "deadline"
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

    async def _tier1(self, turn: _Turn, command: Tier1Command, snap: SessionSnapshot) -> None:
        """Tier-1 кодом: сессия — здесь, действия — владельцам, речь и движение — через порты. 0 вызовов LLM."""
        self._log.info(f"{LOG_MARK} tier1 kind={command.kind} args={dict(command.args)} turn={turn.turn_id}")
        turn.stage = "done"
        actions = tier1_actions(command, snap)
        stop = interrupt_for(command, turn.text, snap)
        if stop.interrupts:
            snap = self._interrupt(turn, stop)
            actions = () if self._ports.stop_perform(str(stop.perform)) else stop.music
        if command.kind == "silence" and snap.top("Silence") is None:
            self.session.push_scene("Silence", "user")
        elif command.kind == "unsilence":
            self.session.pop_scene("Silence", "user")
        elif command.kind == "new_session":
            self.session.reset()
            self.log.reset()
            turn.epoch = self.session.snapshot.epoch  # сам ход — в новой сессии: его подтверждение не «устарело»
        results = await self._execute(turn, actions, snap)
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
        tools = available_tools(snap, None, snap.agent, turn.text)
        messages: List[Any] = self._messages(turn, snap)
        for iteration in range(kn.LLM_QUERY_ITERATIONS_MAX + 1):
            answer = await turn.deadline.wait(self._ports.ask(messages, tools, turn.deadline.left()), "llm")
            turn.llm_calls += answer.calls
            turn.provider, turn.tokens = answer.provider or turn.provider, turn.tokens + answer.tokens
            if self._stale(turn, "answer"):
                return
            data = answer.result.reason if isinstance(answer.result, Invalid) else None
            self._event("command", turn, data={"provider": answer.provider, "valid": data is None, "reason": data})
            if not isinstance(answer.result, Command):
                turn.stage, turn.status, turn.reason = "failed", "failed", f"invalid_command:{data}"[:60]
                self._log.warning(f"{LOG_MARK} невалидная команда {data} → ход провален, без речи")
                return
            more = await self._act(turn, answer.result, snap, last=iteration >= kn.LLM_QUERY_ITERATIONS_MAX)
            if more is None:
                return
            messages.extend(answer.follow(more))

    async def _act(self, turn: _Turn, cmd: Command, snap: SessionSnapshot, last: bool) -> Optional[str]:
        """Исполнить команду. Все действия — ``query`` и итерации ещё есть: вернуть данные для модели."""
        turn.stage = "done"
        if cmd.passed:
            turn.reason = "passed"
            self._log.info(f"{LOG_MARK} passed={cmd.passed} addressed={not turn.pickup}")
            # Решение Шифу 22.10: на адресованную реплику (вейк, окно ответа, Telegram) молчать нельзя — код мычит;
            # passed из бэклога (подхват накопленного) — тихо.
            if voices_passed(turn.pickup, self.session.snapshot):
                self._say(turn, self._pick(kn.PASSED_ADDRESSED_PHRASE))
            return None
        results = await self._execute(turn, _runnable(cmd.actions, last), snap)
        if self._stale(turn, "results"):
            return None
        if not last and results and all(r.cls == _QUERY for r in results):
            return _query_data(results)
        self._speak(turn, cmd, results)
        return None

    def _speak(self, turn: _Turn, cmd: Command, results: Sequence[ActionResult]) -> None:
        """Сначала фраза кода об исходе (провал — грустно), затем слова модели, что звучат (§4.1)."""
        outcome = outcome_phrase(results) if results else None
        if outcome:
            sad = any(r.status in _FAILED for r in results)
            self._say(turn, outcome, None, "sad" if sad else cmd.expression.emotion)
        owned, refusal = hand_perform(self._ports, turn.turn_id, turn.epoch, cmd, self._log)
        self._say(turn, refusal, None, "sad")  # отказ владельца выступления — фраза кода, а не тишина ("" — нет)
        lines = _content_lines(cmd, self.session.snapshot.agent, segments=not owned)
        for text, voice in lines:
            self._say(turn, text, voice or None, cmd.expression.emotion)
        if cmd.ask is not None and not cmd.actions:
            self.session.set_pending_ask(cmd.ask)
        if cmd.actions and not results and not lines:  # query сверх лимита и сказать нечего: провал, а не «ок»
            turn.stage, turn.status, turn.reason = "failed", "failed", "nothing_to_do"

    async def _execute(self, turn: _Turn, actions: Sequence[ToolCall], snap: SessionSnapshot) -> List[ActionResult]:
        if not actions:
            return []
        cls = kn.TOOL_CLASSES[actions[0].tool]
        self._ports.stage(turn.turn_id, turn.epoch, "executing", "started", cls=cls)
        run = execute_for_speaker(self._ports, actions, snap, self.session.snapshot, turn.turn_id)
        results = await turn.deadline.wait(run, "actions")
        for result in results:
            self.log.result(turn.turn_id, result)
            self._event("action", turn, data={"tool": result.tool, "status": result.status, "reason": result.reason,
                                              "cls": result.cls})
        return results

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
        self._ports.stage(turn.turn_id, turn.epoch, turn.stage, turn.status, turn.reason,
                          llm_calls=turn.llm_calls, provider=turn.provider)
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


__all__ = ["Answer", "LOG_MARK", "Ports", "RECENT_ACTIONS", "RECENT_UNADDRESSED", "BACKGROUND_SINCE_REPLY",
           "TurnEngine", "hand_perform", "outcome_phrase", "tier1_actions",
           "tier1_for", "tier1_phrase", "voices_passed"]
