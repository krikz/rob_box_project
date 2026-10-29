"""Режим команд в v1 терминален: действия исполняет код, об исходе говорит код (ADR-0150 §4.1, §5.2–5.4; ADR-0148).

PR-7 ч.7c (#3698). Мост ``rob_box_voice.core.command_bridge`` отдаёт команду LLM, где есть действия, выступление или
голос реплики, одним вызовом ``command`` (``knowledge.COMMAND_TOOL``). Цикл тулов ``AgentCore`` не исполняет его как
тул, а отдаёт :class:`CommandTurn`:

* команда проверяется :func:`rob_box_dialog.command.validate` по срезу тулов хода (``set_voice`` виден только по явной
  голосовой просьбе, #3744); невалидна (или мост уже признал её такой: ``COMMAND_INVALID_FIELD``) — речи нет, ход закрывается провалом:
  :attr:`CommandTurn.failure` → рефлекс ``failed`` (earcon fail, красное кольцо), следующий ход идёт обычно (повтор по
  нарушениям уже сделал мост ``command_bridge`` по тому же срезу, #3698; второго вызова LLM нет);
* действия исполняет :class:`rob_box_dialog.execute.Executor` разом; транспорт — исполнитель тулов v1 (MCP под
  планировщиком ресурсов ``PlanToolExecutor``, PR-7b #3699): ждать ли занятое — решает план, исход — событие владельца,
  которое тул кладёт в данные результата (``status``/``reason``), или ``timeout``/``rejected`` плана;
* «расскажи анекдот, а потом включи музыку» (Tier-1 «потом», §5.6.4): действия, которые план ставит после речи,
  ждут её — сначала звучат фраза кода «Поставлю после …» и выступление, потом действие;
* фразу об исходе строит код (:func:`rob_box_dialog.respond.join`); ``speech`` модели при действиях не звучит
  (строка лога) — LLM не может сказать «Вот радостный звук!», когда звук не сыграл (Шифу 21.10). После действий
  ``action.*`` ход окончен: LLM слова не получает;
* ``perform`` — ответ по сути (сказка, анекдот), а не отчёт: звучит и при действиях, после фразы кода, сегментами;
  голос сегмента — ``voice_per_segment``, голос реплики — ``voice`` (голос робота они не меняют);
* ``query`` — данные обратно LLM, не больше ``knowledge.LLM_QUERY_ITERATIONS_MAX`` раз за ход.

Речь идёт тем же ``speak_text`` исполнителя тулов (порядок держит владелец речи, голос на реплику): счётчики ``speak_text`` и честная
история хода у ``AgentCore`` прежние. Модуль временный — уходит с v1-циклом в PR-8.
"""

from __future__ import annotations

import logging
import uuid
from dataclasses import dataclass
from types import SimpleNamespace
from typing import Any, Awaitable, Callable, Iterable, List, Mapping, Optional, Sequence, Tuple

from rob_box_dialog import knowledge as kn
from rob_box_dialog import phrases
from rob_box_dialog.command import Busy, Command, Invalid, loud_actions, parse, validate
from rob_box_dialog.execute import ActionRequest, Executor, OwnerEvent
from rob_box_dialog.plan.composition import compose, kind_of
from rob_box_dialog.plan.plan_events import PlanEvent, queued_phrase
from rob_box_dialog.respond import ActionResult, audible, join
from rob_box_llm.provider import LLMMessage, LLMResponse, ToolCall, ToolResult

_LOG = logging.getLogger(__name__)
#: Строка лога каждой исполненной команды (читается вместе с ``🧾 [command]`` моста).
LOG_MARK = "🧾 [command]"
SPEAK_TOOL = kn.SPEECH_CHANNEL_TOOLS[0]
_QUERY = "query"
_FAILED = ("rejected", "timeout")
#: Мост режима команд стоит только у личности (оператор и телеграм команду не получают); эпохи в v1 нет.
_SNAPSHOT = SimpleNamespace(epoch=0, agent=kn.AGENTS[0])

Line = Tuple[str, str]  # (текст, голос; "" — текущий голос робота)
Next = Callable[[], Awaitable[LLMResponse]]


@dataclass(frozen=True)
class CommandStep:
    """Итог одной команды: исполненные вызовы (действия и речь) с результатами — для счётчиков хода, и следующий
    ответ: пустой (ход окончен) или ответ LLM на данные ``query``."""

    calls: Tuple[ToolCall, ...]
    results: Mapping[str, ToolResult]
    response: LLMResponse


def owner_event(request: ActionRequest, result: ToolResult) -> OwnerEvent:
    """Исход тула как событие владельца. Статус и причина — из данных результата (их кладёт тул по событию владельца:
    ``play_sound`` — ``SoundEvent``, ``register_speaker`` — ``pending``; план — ``timeout``/``rejected``); статуса
    нет — ошибка тула ``rejected``, иначе ``done``. Слоты фразы — аргументы вызова, данные результата поверх них."""
    data = dict(result.data or {})
    status = data.get("status")
    if status not in kn.ACTION_STATUSES:
        status = "rejected" if result.is_error else "done"
    reason = data.get("reason") if isinstance(data.get("reason"), str) else None
    if reason is None and result.is_error:  # отказ MCP до тула (подпись, срез): причина — в тексте ошибки (#3853)
        reason = result.content[:200] or None
    ref = data.get("event_ref")
    return OwnerEvent(request.turn_id, status, reason, {**dict(request.args), **data},
                      event_ref=ref if isinstance(ref, str) and ref else None)


def outcome_phrase(results: Sequence[ActionResult]) -> Optional[str]:
    """Фраза кода об исходе действий хода. ``query`` и тулы, чей исход говорит другой код
    (``knowledge.OUTCOME_SPOKEN_ELSEWHERE``), не входят — кроме отказа плана (``PLAN_WAIT_REASON``): до владельца
    запрос не дошёл, и другой код о нём не знает (живой робот 21.10: музыка «потом» не дождалась — робот молчал).
    Слота фразы нет в данных события — общая фраза о неудаче."""
    own = [r for r in results if r.cls != _QUERY
           and (r.tool not in kn.OUTCOME_SPOKEN_ELSEWHERE or r.reason == kn.PLAN_WAIT_REASON)]
    try:
        said = join(own)
    except KeyError as missing:
        _LOG.warning(f"{LOG_MARK} у фразы исхода нет слота {missing}: {[(r.tool, r.status) for r in own]}")
        failed = [r for r in own if r.status in _FAILED]
        return phrases.text(f"local.{failed[0].status}") if failed else None
    return said.text if said is not None else None


def spoken_lines(cmd: Command, results: Sequence[ActionResult], content: bool = True) -> List[Line]:
    """Что прозвучит после исполнения: фраза кода об исходе, затем реплика (только без действий) и сегменты
    выступления — ответ по сути звучит и при действиях; ``content=False`` — выступление уже прозвучало до действий."""
    lines: List[Line] = []
    phrase = outcome_phrase(audible(results, bool(cmd.speech)))
    if phrase:
        lines.append((phrase, ""))
    said = cmd.speech or (cmd.ask.text if cmd.ask is not None else None)
    if said and not loud_actions(cmd):
        lines.append((said, cmd.voice or ""))
    elif said:
        _LOG.info(f"{LOG_MARK} речь при действиях не звучит (§4.1): {said[:120]!r}")
    perf = cmd.perform
    if perf is not None and content:
        voices = perf.voice_per_segment or ("",) * len(perf.segments)
        lines.extend(zip(perf.segments, voices))
    return lines


class CommandTurn:
    """Команды одного хода v1-цикла: проверка, исполнение через :class:`Executor`, речь, счёт query-итераций."""

    def __init__(self, tools: Any, offered: Iterable[Mapping[str, Any]]) -> None:
        self._tools = tools
        self._available = frozenset(str((t.get("function") or {}).get("name", "")) for t in offered)
        self._turn = uuid.uuid4().hex[:12]
        self._queries = 0
        self._n = 0
        #: Причина провала хода (``""`` — не провалился): невалидная команда → ход провален (``turn_failure``).
        self.failure = ""

    @staticmethod
    def handles(response: LLMResponse) -> bool:
        """Ответ — команда моста: ровно один вызов ``command``."""
        calls = response.tool_calls or ()
        return len(calls) == 1 and calls[0].name == kn.COMMAND_TOOL

    async def run(self, response: LLMResponse, messages: List[LLMMessage], next_response: Next) -> CommandStep:
        """Исполнить команду; ``messages`` дополняется только на ``query`` (данные — обратно LLM)."""
        call = response.tool_calls[0]
        marked = call.arguments.get(kn.COMMAND_INVALID_FIELD)
        cmd = Invalid(str(marked)) if marked else validate(parse(dict(call.arguments)), self._available,
                                                           busy=self._busy())
        done: List[Tuple[ToolCall, ToolResult]] = []
        if isinstance(cmd, Invalid):
            self.failure = f"invalid_command:{cmd.reason}"[:60]
            _LOG.warning(f"{LOG_MARK} невалидна: {cmd.reason} {cmd.detail[:120]!r} → ход провален, без речи")
            # Провалившийся вызов виден в ``tools_called``/``tool_error_occurred``: пустой ответ после него — не «тихий
            # первый ответ», v1-ретрай #1217 (второй вызов LLM) не нужен.
            done.append((call, ToolResult(tool_call_id=call.id, content=self.failure, is_error=True)))
            return self._step(done, LLMResponse(content=""))
        if cmd.passed:  # честное молчание (#3859): не провал и не «нечего исполнить»; ход закрывается без речи
            _LOG.info(f"{LOG_MARK} passed={cmd.passed}")
            return self._step(done, LLMResponse(content=""))
        performed = self._perform(cmd, call, done)
        first = self._before_actions(cmd, with_segments=not performed)
        if first:
            await self._say(done, first)
        results = await self._execute(self._allowed(cmd), done)
        if results and all(r.cls == _QUERY for r in results):
            self._queries += 1
            data = ToolResult(tool_call_id=call.id, content="\n".join(f"{c.name}: {r.content}" for c, r in done))
            messages.append(LLMMessage(role="assistant", content="", tool_calls=(call,)))
            messages.append(LLMMessage(role="tool", content=data.content, tool_call_id=call.id, tool_result=data))
            return self._step(done, await next_response())
        lines = spoken_lines(cmd, results, content=not (first or performed))
        if not lines and not done:  # нечего исполнить и нечего сказать (query сверх лимита): провал, а не молчаливое «ок»
            self.failure = "nothing_to_do"
        _LOG.info(f"{LOG_MARK} ход окончен: {[(r.tool, r.status, r.reason) for r in results]} "
                  f"скажу {[text[:60] for text, _ in lines]}")
        await self._say(done, lines)
        return self._step(done, LLMResponse(content=""))

    def _busy(self) -> Busy:
        """Что занято для валидатора: ACTIVE-сегмент идущего выступления (``edit_pending``, PR-7c)."""
        performer = getattr(self._tools, "performer", None)
        return Busy(active_segment=getattr(performer, "active_segment", None))

    def _perform(self, cmd: Command, call: ToolCall, done: List[Tuple[ToolCall, ToolResult]]) -> bool:
        """Выступление — владельцу (``PerformBlock`` хоста, ADR-0150 PR-7c): сегменты на тактах, подложка, правка
        ``edit_pending``. Хоста нет (агент без выступлений) — сегменты звучат репликами, как раньше. Отданное
        владельцу — исполненный вызов хода (не «тихий ответ», второго вызова LLM нет)."""
        performer = getattr(self._tools, "performer", None)
        if performer is None or cmd.perform is None:
            return False
        outcome = performer.edit(cmd.perform) if cmd.edit_pending else performer.start(cmd.perform, self._turn)
        _LOG.info(f"{LOG_MARK} выступление {'правка' if cmd.edit_pending else 'старт'}: {outcome}")
        done.append((call, ToolResult(tool_call_id=call.id, content=f"perform: {outcome.get('status')}",
                                      is_error=outcome.get("status") == "rejected", data=outcome)))
        return True

    def _before_actions(self, cmd: Command, with_segments: bool = True) -> List[Line]:
        """Действия, которые план ставит после речи этой команды (Tier-1 «потом»): до них звучит фраза кода
        «Поставлю после того, как договорю» (и выступление, если его не взял владелец); их речь план видит занятой и
        ждёт её конца."""
        planner = getattr(self._tools, "planner", None)
        if planner is None or cmd.perform is None:
            return []
        later = [a for a in cmd.actions if "speech" in compose(kind_of(a.tool), ["speech"], planner.hint).waits_for]
        if not later:
            return []
        chars = sum(len(s) for s in cmd.perform.segments)
        eta_ms = int(chars / planner.state.rate.chars_per_s * 1000)
        phrase = queued_phrase(PlanEvent("queued", later[0].tool, after_resource="speech", eta_ms=eta_ms),
                               kind_of(later[0].tool))
        if not with_segments:
            return [(phrase, "")]
        voices = cmd.perform.voice_per_segment or ("",) * len(cmd.perform.segments)
        return [(phrase, ""), *zip(cmd.perform.segments, voices)]

    def _allowed(self, cmd: Command) -> Tuple[Any, ...]:
        """Действия к исполнению: ``query`` сверх ``LLM_QUERY_ITERATIONS_MAX`` итераций хода не исполняется."""
        if self._queries < kn.LLM_QUERY_ITERATIONS_MAX:
            return cmd.actions
        kept = tuple(a for a in cmd.actions if kn.TOOL_CLASSES[a.tool] != _QUERY)
        if len(kept) < len(cmd.actions):
            _LOG.warning(f"{LOG_MARK} query сверх {kn.LLM_QUERY_ITERATIONS_MAX} итераций не исполняю")
        return kept

    async def _execute(self, actions: Sequence[Any], done: List[Tuple[ToolCall, ToolResult]]) -> List[ActionResult]:
        if not actions:
            return []

        async def send(request: ActionRequest) -> None:
            call, result = await self._call(request.tool, dict(request.args))
            done.append((call, result))
            executor.on_event(owner_event(request, result))

        executor = Executor(send, log=_LOG)
        return await executor.run(actions, _SNAPSHOT, f"{self._turn}-{self._n}")

    async def _say(self, done: List[Tuple[ToolCall, ToolResult]], lines: Sequence[Line]) -> None:
        for text, voice in lines:
            done.append(await self._call(SPEAK_TOOL, {"text": text, **({"voice": voice} if voice else {})}))

    async def _call(self, name: str, args: Mapping[str, Any]) -> Tuple[ToolCall, ToolResult]:
        self._n += 1
        call = ToolCall(id=f"cmd-{self._turn}-{self._n}", name=name, arguments=dict(args))
        try:
            return call, await self._tools.execute(call)
        except Exception as exc:  # noqa: BLE001 — сбой транспорта: честный отказ, а не падение хода
            return call, ToolResult(tool_call_id=call.id, content=f"{type(exc).__name__}: {exc}", is_error=True)

    @staticmethod
    def _step(done: Sequence[Tuple[ToolCall, ToolResult]], response: LLMResponse) -> CommandStep:
        return CommandStep(tuple(c for c, _ in done), {c.id: r for c, r in done}, response)


__all__ = ["CommandStep", "CommandTurn", "LOG_MARK", "outcome_phrase", "owner_event", "spoken_lines"]
