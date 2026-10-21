"""Режим команд в v1 терминален: действия исполняет код, об исходе говорит код (ADR-0150 §4.1, §5.2–5.4; ADR-0148).

PR-7 ч.7c (#3698). Мост ``rob_box_voice.core.command_bridge`` отдаёт команду LLM, где есть действия, выступление или
голос реплики, одним вызовом ``command`` (``knowledge.COMMAND_TOOL``). Цикл тулов ``AgentCore`` не исполняет его как
тул, а отдаёт :class:`CommandTurn`:

* команда проверяется :func:`rob_box_dialog.command.validate` по срезу тулов хода (``set_voice`` виден только по явной
  голосовой просьбе, #3744); невалидна — вопрос-шаблон ``understand.invalid`` (повтор по нарушениям уже сделал мост
  ``command_bridge`` по тому же срезу, #3698; здесь — страховка, второго вызова LLM нет);
* действия исполняет :class:`rob_box_dialog.execute.Executor`; транспорт — исполнитель тулов v1 (MCP, каналы
  планировщика), исход — событие владельца, которое тул кладёт в данные результата (``status``/``reason``, PR-7b);
* фразу об исходе строит код (:func:`rob_box_dialog.respond.join`); ``speech`` модели при действиях не звучит
  (строка лога) — LLM не может сказать «Вот радостный звук!», когда звук не сыграл (Шифу 21.10). После действий
  ``action.*`` ход окончен: LLM слова не получает;
* ``perform`` — ответ по сути (сказка, анекдот), а не отчёт: звучит и при действиях, после фразы кода, сегментами;
  голос сегмента — ``voice_per_segment``, голос реплики — ``voice`` (голос робота они не меняют);
* ``query`` — данные обратно LLM, не больше ``knowledge.LLM_QUERY_ITERATIONS_MAX`` раз за ход.

Речь идёт тем же ``speak_text`` исполнителя тулов (очередь VOICE, голос на реплику): счётчики ``speak_text`` и честная
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
from rob_box_dialog.command import INVALID_ASK_KEY, Command, Invalid, parse, validate
from rob_box_dialog.execute import ActionRequest, Executor, OwnerEvent
from rob_box_dialog.respond import ActionResult, join
from rob_box_llm.provider import LLMMessage, LLMResponse, ToolCall, ToolResult

_LOG = logging.getLogger(__name__)
#: Строка лога каждой исполненной команды (читается вместе с ``🧾 [command]`` моста).
LOG_MARK = "🧾 [command]"
SPEAK_TOOL = kn.SPEECH_CHANNEL_TOOLS[0]
_QUERY = "query"
#: Исход, который планировщик каналов v1 отдаёт сразу (анимация, стоп музыки после речи): обещание без события.
_QUEUED = "queued"
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
    ``play_sound`` — ``SoundEvent``, ``register_speaker`` — ``pending``); «в очереди» — обещание ``pending``; статуса
    нет — ошибка тула ``rejected``, иначе ``done``. Слоты фразы — аргументы вызова, данные результата поверх них."""
    data = dict(result.data or {})
    status = "pending" if data.get("status") == _QUEUED else data.get("status")
    if status not in kn.ACTION_STATUSES:
        status = "rejected" if result.is_error else "done"
    reason = data.get("reason") if isinstance(data.get("reason"), str) else None
    return OwnerEvent(request.turn_id, status, reason, {**dict(request.args), **data})


def outcome_phrase(results: Sequence[ActionResult]) -> Optional[str]:
    """Фраза кода об исходе действий хода. ``query`` и тулы, чей исход говорит другой код
    (``knowledge.OUTCOME_SPOKEN_ELSEWHERE``), не входят. Слота фразы нет в данных события — общая фраза о неудаче."""
    own = [r for r in results if r.cls != _QUERY and r.tool not in kn.OUTCOME_SPOKEN_ELSEWHERE]
    try:
        said = join(own)
    except KeyError as missing:
        _LOG.warning(f"{LOG_MARK} у фразы исхода нет слота {missing}: {[(r.tool, r.status) for r in own]}")
        failed = [r for r in own if r.status in _FAILED]
        return phrases.text(f"local.{failed[0].status}") if failed else None
    return said.text if said is not None else None


def spoken_lines(cmd: Command, results: Sequence[ActionResult]) -> List[Line]:
    """Что прозвучит после исполнения: фраза кода об исходе, затем реплика (только без действий) и сегменты
    выступления — ответ по сути звучит и при действиях."""
    lines: List[Line] = []
    phrase = outcome_phrase(results)
    if phrase:
        lines.append((phrase, ""))
    said = cmd.speech or (cmd.ask.text if cmd.ask is not None else None)
    if said and not cmd.actions:
        lines.append((said, cmd.voice or ""))
    elif said:
        _LOG.info(f"{LOG_MARK} речь при действиях не звучит (§4.1): {said[:120]!r}")
    perf = cmd.perform
    if perf is not None:
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

    @staticmethod
    def handles(response: LLMResponse) -> bool:
        """Ответ — команда моста: ровно один вызов ``command``."""
        calls = response.tool_calls or ()
        return len(calls) == 1 and calls[0].name == kn.COMMAND_TOOL

    async def run(self, response: LLMResponse, messages: List[LLMMessage], next_response: Next) -> CommandStep:
        """Исполнить команду; ``messages`` дополняется только на ``query`` (данные — обратно LLM)."""
        call = response.tool_calls[0]
        cmd = validate(parse(dict(call.arguments)), self._available)
        done: List[Tuple[ToolCall, ToolResult]] = []
        if isinstance(cmd, Invalid):
            _LOG.warning(f"{LOG_MARK} невалидна: {cmd.reason} {cmd.detail[:120]!r}")
            await self._say(done, [(phrases.text(cmd.ask.key or INVALID_ASK_KEY), "")])
            return self._step(done, LLMResponse(content=""))
        results = await self._execute(self._allowed(cmd), done)
        if results and all(r.cls == _QUERY for r in results):
            self._queries += 1
            data = ToolResult(tool_call_id=call.id, content="\n".join(f"{c.name}: {r.content}" for c, r in done))
            messages.append(LLMMessage(role="assistant", content="", tool_calls=(call,)))
            messages.append(LLMMessage(role="tool", content=data.content, tool_call_id=call.id, tool_result=data))
            return self._step(done, await next_response())
        lines = spoken_lines(cmd, results) or ([] if done else [(phrases.text(INVALID_ASK_KEY), "")])
        _LOG.info(f"{LOG_MARK} ход окончен: {[(r.tool, r.status, r.reason) for r in results]} "
                  f"скажу {[text[:60] for text, _ in lines]}")
        await self._say(done, lines)
        return self._step(done, LLMResponse(content=""))

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
