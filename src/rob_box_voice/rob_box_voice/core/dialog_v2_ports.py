"""Транспорт хода v2 (``rob_box_dialog.turn.Ports``) поверх того, что уже есть у v1 (ADR-0150 §3; PR-8 ч.8e #3702).

Одна дорога на оба движка (решение координатора 22.10 по #3702):

* **LLM** — цепочка ``rob_box_harness.core.assembly.build_llm_chain`` (тот же ``HealthAwareFallbackLLM``), каждый
  провайдер — в режиме команд моста ``command_bridge`` (форма запроса по ``degrade.select_mode``, один парсер, повтор по
  нарушениям ``COMMAND_RETRIES_MAX``). Ответ моста в форме v1 (текст речи или вызов ``command``) переводится обратно в
  ``command.Result`` (:func:`to_result`); цикла ``AgentCore`` нет — окно и итерации ``query`` ведёт ``turn.py``;
* **действия** — :class:`rob_box_dialog.execute.Executor` поверх исполнителя тулов v1 (``plan_host.make_tool_executor``:
  MCP под планировщиком ресурсов), исход — событие владельца из данных результата (``command_turn.owner_event``).
  Топик ``ActionRequest`` не публикуется: подписчика у него нет (mcp_server слушает ``/mcp/execute``);
* **речь / стадии / интент** — ``speech_client``, ``expression_client`` (+ рефлекс ``expression.reflex``),
  ``/dialog/intent``.

Модуль без класса ноды: ROS-хост ``dialog_node`` (ч.8e2) отдаёт сюда себя и собранные объекты.
"""

from __future__ import annotations

import uuid
from typing import Any, AsyncIterator, Callable, Iterable, List, Mapping, Optional, Sequence

from rob_box_core import expression_client, speech_client
from rob_box_core.utterance import Sink, Utterance
from rob_box_dialog import knowledge as kn
from rob_box_dialog.command import Command, Invalid, Result, ToolCall as CommandCall, parse
from rob_box_dialog.context import measure
from rob_box_dialog.execute import ActionRequest, Executor
from rob_box_dialog.expression import reflex
from rob_box_dialog.respond import ActionResult
from rob_box_dialog.session import SessionSnapshot
from rob_box_dialog.turn import Answer
from rob_box_harness.core.agent_core import answering_provider
from rob_box_harness.core.tool_loop.command_turn import owner_event
from rob_box_llm.provider import LLMMessage, LLMResponse, LLMSettings, ToolCall, ToolResult

from .command_intent import fill_intent
from .command_parser import Command as MotionCommand, IntentType

#: Ответ модели пуст (ни речи, ни команды) — невалиден (A14: пустой ответ по провайдеру).
EMPTY = "empty"
_STATUS = {"done": "finished", "failed": "failed", "timeout": "failed"}


class CallCounter:
    """Сколько раз провайдеры цепочки реально вызывались за ход (повтор моста — тоже вызов LLM, A1)."""

    def __init__(self) -> None:
        self.calls = 0


class Counted:
    """Провайдер цепочки, который считает свои вызовы; всё остальное — у обёрнутого (имя кэша здоровья то же)."""

    def __init__(self, inner: Any, counter: CallCounter) -> None:
        self._inner = inner
        self._counter = counter
        self.name = getattr(inner, "name", type(inner).__name__)

    def __getattr__(self, item: str) -> Any:
        inner = self.__dict__.get("_inner")
        if inner is None:
            raise AttributeError(item)
        return getattr(inner, item)

    async def complete(self, messages: Iterable[LLMMessage], **kwargs: Any) -> LLMResponse:
        self._counter.calls += 1
        return await self._inner.complete(messages, **kwargs)

    def stream(self, messages: Iterable[LLMMessage], **kwargs: Any) -> AsyncIterator[Any]:
        self._counter.calls += 1
        return self._inner.stream(messages, **kwargs)


def counted_wrapper(bridge_wrap: Callable[[Any], Any], counter: CallCounter) -> Callable[[Any], Any]:
    """``AgentSpec.wrap_provider`` для v2: счётчик вызовов внутри моста команд (повтор считается)."""
    return lambda provider: bridge_wrap(Counted(provider, counter))


def to_result(response: LLMResponse) -> Result:
    """Ответ моста (форма v1) → команда: вызов ``command`` — его аргументы (отметка невалидности моста — ``Invalid``),
    иначе текст — реплика; ни того ни другого — ``Invalid(empty)``."""
    for call in response.tool_calls or ():
        if call.name == kn.COMMAND_TOOL:
            marked = call.arguments.get(kn.COMMAND_INVALID_FIELD)
            return Invalid(str(marked)) if marked else parse(dict(call.arguments))
    text = (response.content or "").strip()
    return Command(speech=text) if text else Invalid(EMPTY, "ни речи, ни команды")


def to_message(message: Any) -> LLMMessage:
    """Сообщение хода (словарь ``turn.py`` или готовое ``LLMMessage`` продолжения) → ``LLMMessage``."""
    if isinstance(message, LLMMessage):
        return message
    return LLMMessage(role=str(message["role"]), content=str(message.get("content") or ""))


def tool_schema(entry: Any) -> Mapping[str, Any]:
    """Запись каталога (``ToolCatalogEntry``) → OpenAI-схема тула для провайдера."""
    return {"type": "function", "function": {"name": entry.name, "description": entry.description,
                                             "parameters": dict(entry.parameters)}}


def follow_for(response: LLMResponse) -> Callable[[str], Sequence[Any]]:
    """Продолжение запроса данными ``query``: вызов ``command`` модели и tool-результат с данными на него."""
    calls = tuple(c for c in response.tool_calls or () if c.name == kn.COMMAND_TOOL)
    if not calls:
        return lambda data: ({"role": "user", "content": data},)
    call = calls[0]

    def follow(data: str) -> Sequence[Any]:
        return (LLMMessage(role="assistant", content="", tool_calls=(call,)),
                LLMMessage(role="tool", content=data, tool_call_id=call.id,
                           tool_result=ToolResult(tool_call_id=call.id, content=data)))

    return follow


class V2Ports:
    """Порты хода v2 одной ноды: LLM-цепочка, исполнитель тулов, речь, стадии, интент."""

    def __init__(self, node: Any, llm: Any, counter: CallCounter, tools: Any, *, agent: str = "personality",
                 settings: Optional[LLMSettings] = None, intent_pub: Any = None,
                 log: Callable[[str], None] = lambda line: None, epoch: Optional[Callable[[], int]] = None,
                 performer: Callable[[], Any] = lambda: None) -> None:
        self._node = node
        self._llm = llm
        self._counter = counter
        self._tools = tools
        self._agent = agent
        self._settings = settings
        self._intent_pub = intent_pub
        self._log = log
        self._turn = ""
        self._n = 0
        self._performer = performer
        self.executor = Executor(self._send, on_late=self._late, epoch=epoch)

    async def ask(self, messages: Sequence[Any], tools: Sequence[Any], timeout_s: float) -> Answer:
        """Один ответ LLM (с повтором моста по нарушениям); тайм-аут ставит дедлайн хода снаружи."""
        msgs = [to_message(m) for m in messages]
        schemas = [tool_schema(t) for t in tools]
        before = self._counter.calls
        response = await self._llm.complete(msgs, tools=schemas, settings=self._settings)
        provider = answering_provider(self._llm)
        usage = dict(response.usage or {})
        tokens = int(usage.get("prompt_tokens") or measure(msgs, schemas).total)
        self._log(f"🧮 [prompt] tokens={tokens} estimated={'prompt_tokens' not in usage} provider={provider} "
                  f"view=v2 tools={len(schemas)}")
        return Answer(to_result(response), provider, max(1, self._counter.calls - before), tokens,
                      follow_for(response))

    async def execute(self, actions: Sequence[CommandCall], snapshot: SessionSnapshot,
                      turn_id: str) -> List[ActionResult]:
        """Действия команды — владельцам через исполнитель тулов v1; исход — событие владельца."""
        if turn_id != self._turn:
            self._turn = turn_id
            begin = getattr(self._tools, "begin_turn", None)
            if callable(begin):
                begin()
        return await self.executor.run(actions, snapshot, turn_id)

    async def _send(self, request: ActionRequest) -> None:
        self._n += 1
        call = ToolCall(id=f"v2-{request.turn_id}-{self._n}", name=request.tool, arguments=dict(request.args))
        try:
            result = await self._tools.execute(call)
        except Exception as exc:  # noqa: BLE001 — сбой транспорта: честный отказ владельца, а не падение хода
            result = ToolResult(tool_call_id=call.id, content=f"{type(exc).__name__}: {exc}", is_error=True)
        self.executor.on_event(owner_event(request, result))

    def _late(self, result: ActionResult) -> None:
        self._log(f"🧭 [turn-v2] поздний исход {result.tool} {result.turn_id}: {result.status} {result.reason}")

    def say(self, turn_id: str, epoch: int, text: str, voice: Optional[str] = None,
            emotion: Optional[str] = None) -> str:
        utterance = Utterance(text=text, sink=Sink.SPEAKERS, voice=voice or None, emotion=emotion or None,
                              speech_id=str(uuid.uuid4()))
        return speech_client.say(self._node, utterance, agent=self._agent, epoch=epoch, turn_id=turn_id)

    def stage(self, turn_id: str, epoch: int, stage: str, status: str, reason: str = "", cls: Optional[str] = None,
              llm_calls: int = 0, provider: str = "", source: str = "") -> None:
        """Событие стадии (``/dialog/turn_event``) и рефлекс кода (лицо/кольцо/earcon) без LLM."""
        status = _STATUS.get(status, status)
        expression_client.turn_event(self._node, turn_id=turn_id, epoch=epoch, agent=self._agent, stage=stage,
                                     status=status, reason=reason, llm_calls=llm_calls, llm_provider=provider,
                                     source=source)
        plan = reflex(stage, cls)
        if plan is not None:
            expression_client.express(self._node, turn_id=turn_id, epoch=epoch, agent=self._agent,
                                      emotion=plan.emotion, earcon=plan.earcon, ttl_ms=plan.ttl_ms)

    def cancel_speech(self, epoch: int, reason: str, boundary: str = "as_requested") -> None:
        speech_client.cancel(self._node, agent=self._agent, epoch=epoch, reason=reason, boundary=boundary)

    def stop_perform(self, how: str) -> bool:
        """Перебивание выступления (``PerformHost.halt``/``stop``); ``True`` — играющая музыка — его бит."""
        host = self._performer()
        if host is None:
            return False
        owns = bool(host.owns_backing)
        stopped = host.halt() if how == "halt" else host.stop()
        return bool(stopped) and owns

    def intent(self, turn_id: str, epoch: int, kind: str) -> None:
        """Интент движения для ``command_node`` (сейчас — только ``stop_motion`` Tier-1 стопа)."""
        if self._intent_pub is None or kind != "stop_motion":
            return
        from rob_box_dialog_msgs.msg import Intent  # noqa: PLC0415 — сообщения ROS только в рантайме ноды

        stop = MotionCommand(intent=IntentType.STOP, text="", entities={}, confidence=1.0)
        self._intent_pub.publish(fill_intent(Intent(), stop, epoch=epoch, turn_id=turn_id))


def command_providers(chain: Sequence[str]) -> str:
    """Все провайдеры цепочки — в режиме команд (v2 не знает ``speak_text``): строка для ``command_bridge``."""
    return ",".join(n for n in chain if n)


__all__ = ["CallCounter", "Counted", "EMPTY", "V2Ports", "command_providers", "counted_wrapper", "follow_for",
           "to_message", "to_result", "tool_schema"]
