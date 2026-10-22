"""Ход диалога v2 без ROS: адресованная реплика → Tier-1 или команда LLM → исполнение → фраза кода (ADR-0150 §3, §4.1,
§7.3, §8; PR-8 ч.8d #3702).

:class:`TurnEngine` — весь ход в одном месте и только вызовами модулей пакета: грамматика Tier-1 (``grammar.match``,
0 вызовов LLM), снимок хода (``context.build_context``) и окно озвученного (``turn_log``), срез тулов
(``tools_view.available_tools``), команда модели, исполнение действий владельцам, фраза об исходе из события
(``respond``), дедлайн хода (``deadline``), e2e-события (``e2e_events``). Транспорт — :class:`Ports` снаружи (ROS-хост
``dialog_node``, ч.8e): вызов LLM с разбором и повтором по нарушениям, исполнитель действий (тот же MCP-провайдер
тулов, что у v1, — одна дорога исполнения), речь, стадии хода (``TurnEvent`` + рефлексы), отмена речи, интент.

Правила хода (ADR-0148: решает код):

* голое «робот» без бэклога — только рефлекс ``addressed``, 0 вызовов LLM, 0 речи (§7.3 п.1);
* Tier-1 — исполняет код (стоп, тишина, громкость, голос, новая сессия), спорный стоп (В19) идёт в LLM;
* невалидная команда (после повтора моста) — ход ``failed`` без речи; ``passed`` — тихо;
* действия — владельцам; ``speech`` при действиях не звучит (§4.1), звучит фраза кода об исходе и ``perform``;
* ``query`` — данные обратно модели, не больше ``knowledge.LLM_QUERY_ITERATIONS_MAX`` итераций;
* весь ход — в бюджете ``TURN_DEADLINE_S``: истёк — ``understand.timeout``, строка ``provider_latency``.
"""

from __future__ import annotations

import json
import logging
import time
import uuid
from dataclasses import dataclass, field
from typing import Any, Awaitable, Callable, List, Mapping, Optional, Protocol, Sequence, Tuple

from rob_box_dialog import knowledge as kn
from rob_box_dialog import phrases
from rob_box_dialog.address import BARE_WAKE, Addressed
from rob_box_dialog.command import Command, Invalid, Result, ToolCall
from rob_box_dialog.context import build_context
from rob_box_dialog.deadline import TIMEOUT_PHRASE, DeadlineExpired, TurnDeadline, latency_line
from rob_box_dialog.grammar import Tier1Command, match
from rob_box_dialog.respond import ActionResult, join
from rob_box_dialog.session import Session, SessionSnapshot
from rob_box_dialog.tools_view import available_tools
from rob_box_dialog.turn_log import TOOL, TurnAggregate, TurnLog

_LOG = logging.getLogger(__name__)
#: Строка лога хода v2 (вход, решение, итог).
LOG_MARK = "🧭 [turn-v2]"
_QUERY = "query"
_FAILED = ("rejected", "timeout")
#: Раздел снимка с исходами действий прошлых ходов (окно ``turn_log``: ``tool`` — структурой, не в тексте реплик).
RECENT_ACTIONS = "recent_actions"


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

    def cancel_speech(self, epoch: int, reason: str) -> None: ...

    def intent(self, turn_id: str, epoch: int, kind: str) -> None: ...


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
    said: List[str] = field(default_factory=list)


def tier1_actions(command: Tier1Command, snapshot: SessionSnapshot) -> Tuple[ToolCall, ...]:
    """Действия Tier-1 владельцам: громкость — тул канала, голос — ``set_voice``, стоп — музыка, если играет."""
    if command.kind == "volume":
        tool = kn.VOLUME_CHANNELS[command.args["channel"]].tool
        return (ToolCall(tool, {"action": command.args["action"]}),) if tool else ()
    if command.kind == "set_voice":
        return (ToolCall("set_voice", {"voice": command.args["voice"]}),)
    if command.kind == "stop_all" and (snapshot.music.state == "playing" or snapshot.music.dj):
        return (ToolCall("dj_set", {"action": "stop"}), ToolCall("stop_music", {})) if snapshot.music.dj \
            else (ToolCall("stop_music", {}),)
    return ()


def tier1_phrase(command: Tier1Command, results: Sequence[ActionResult]) -> Optional[str]:
    """Фраза Tier-1: громкость — по исходу тула; стоп и тишина молчат; голос говорит сам тул (SpeechEvent)."""
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


def _content_lines(cmd: Command) -> List[Tuple[str, str]]:
    """Что звучит из слов модели: реплика или вопрос — только без действий (§4.1), выступление — всегда."""
    lines: List[Tuple[str, str]] = []
    said = cmd.speech or (cmd.ask.text if cmd.ask is not None else None)
    if said and not cmd.actions:
        lines.append((said, cmd.voice or ""))
    elif said:
        _LOG.info(f"{LOG_MARK} речь при действиях не звучит (§4.1): {said[:120]!r}")
    if cmd.perform is not None:
        voices = cmd.perform.voice_per_segment or ("",) * len(cmd.perform.segments)
        lines.extend(zip(cmd.perform.segments, voices))
    return lines


def _runnable(actions: Sequence[ToolCall], last: bool) -> Tuple[ToolCall, ...]:
    """Действия к исполнению: на последней итерации ``query`` уже не исполняется (данные некому вернуть)."""
    return tuple(a for a in actions if not last or kn.TOOL_CLASSES[a.tool] != _QUERY)


def _query_data(results: Sequence[ActionResult]) -> str:
    rows = [{"tool": r.tool, "status": r.status, "reason": r.reason, "data": dict(r.data)} for r in results]
    return json.dumps({"query_results": rows}, ensure_ascii=False, default=str)


class TurnEngine:
    """Ходы одного агента (личность): владелец сессии и журнала ходов — этот объект, транспорт — :class:`Ports`."""

    def __init__(self, session: Session, ports: Ports, *, system_prompt: str = "",
                 sections: Optional[Callable[[SessionSnapshot, str], Mapping[str, Any]]] = None,
                 voices: Optional[Callable[[], Sequence[Tuple[str, str]]]] = None, turn_log: Optional[TurnLog] = None,
                 events: Any = None, new_id: Callable[[], str] = lambda: uuid.uuid4().hex[:12],
                 clock: Callable[[], float] = time.monotonic, deadline_s: float = kn.TURN_DEADLINE_S,
                 log: Optional[logging.Logger] = None) -> None:
        self.session = session
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
        """Один адресованный ход до конца; итог — агрегат без текста (строка лога ``📊 [turn]``)."""
        snap = self.session.snapshot
        turn = _Turn(self._new_id(), snap.epoch, addressed.text.strip(), addressed.source,
                     TurnDeadline(self._deadline_s, clock=self._clock))
        self.log.begin(turn.turn_id, turn.epoch, turn.text)
        self._event("user", turn, text=turn.text, data={"source": turn.source})
        self._ports.stage(turn.turn_id, turn.epoch, "addressed", "started", source=turn.source)
        if addressed.source == BARE_WAKE or not turn.text:
            return self._finish(turn)
        tier1 = match(turn.text, snap, self._voices())
        try:
            if tier1 is not None and not tier1.disputed:
                await self._tier1(turn, tier1, snap)
            else:
                await self._understand(turn, snap)
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
        if command.kind in ("stop_all", "silence"):
            self._ports.cancel_speech(turn.epoch, f"tier1 {command.kind}")
        if command.kind == "stop_all":
            self._ports.intent(turn.turn_id, turn.epoch, "stop_motion")
        if command.kind == "silence" and snap.top("Silence") is None:
            self.session.push_scene("Silence", "user")
        elif command.kind == "unsilence":
            self.session.pop_scene("Silence", "user")
        elif command.kind == "new_session":
            self.session.reset()
            self.log.reset()
        results = await self._execute(turn, tier1_actions(command, snap), snap)
        said = tier1_phrase(command, results)
        if said and self.session.snapshot.top("Silence") is None:
            self._say(turn, said)

    async def _understand(self, turn: _Turn, snap: SessionSnapshot) -> None:
        """Команда модели: ≤ ``LLM_QUERY_ITERATIONS_MAX`` итераций ``query``, затем действия и/или речь."""
        self._ports.stage(turn.turn_id, turn.epoch, "thinking", "started")
        tools = available_tools(snap, None, snap.agent, turn.text)
        messages: List[Any] = self._messages(turn, snap)
        for iteration in range(kn.LLM_QUERY_ITERATIONS_MAX + 1):
            answer = await turn.deadline.wait(self._ports.ask(messages, tools, turn.deadline.left()), "llm")
            turn.llm_calls += answer.calls
            turn.provider, turn.tokens = answer.provider or turn.provider, turn.tokens + answer.tokens
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
            self._log.info(f"{LOG_MARK} passed={cmd.passed}")
            return None
        results = await self._execute(turn, _runnable(cmd.actions, last), snap)
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
        lines = _content_lines(cmd)
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
        results = await turn.deadline.wait(self._ports.execute(actions, snap, turn.turn_id), "actions")
        for result in results:
            self.log.result(turn.turn_id, result)
            self._event("action", turn, data={"tool": result.tool, "status": result.status, "reason": result.reason,
                                              "cls": result.cls})
        return results

    def _messages(self, turn: _Turn, snap: SessionSnapshot) -> List[Any]:
        """Запрос: системный промпт, окно озвученного (``user``/``assistant``), снимок хода + реплика человека.
        Исходы прошлых действий — разделом снимка :data:`RECENT_ACTIONS`, а не репликами (§7.3)."""
        past = self.log.window()[:-1]  # последняя запись — реплика этого хода (журнал начат в run)
        history = [e for e in past if e.role != TOOL]
        sections = dict(self._sections(snap, turn.text))
        recent = [dict(e.content) for e in past if e.role == TOOL]
        context = build_context({**sections, RECENT_ACTIONS: recent[-kn.TURN_WINDOW_MAX_TURNS:]})
        messages: List[Any] = [{"role": "system", "content": self._system}] if self._system else []
        messages += [{"role": e.role, "content": e.content} for e in history]
        messages.append({"role": "user", "content": f"{context}\n\n{turn.text}"})
        return messages

    def _say(self, turn: _Turn, text: str, voice: Optional[str] = None, emotion: Optional[str] = None) -> None:
        if not text:
            return
        speech_id = self._ports.say(turn.turn_id, turn.epoch, text, voice, emotion)
        self.log.said(turn.turn_id, speech_id, text)
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


__all__ = ["Answer", "LOG_MARK", "Ports", "RECENT_ACTIONS", "TurnEngine", "outcome_phrase", "tier1_actions",
           "tier1_phrase"]
