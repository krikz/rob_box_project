"""Транспорт хода v2 (``rob_box_dialog.turn.Ports``) поверх того, что уже есть у v1 (ADR-0150 §3; PR-8 ч.8e #3702).

Одна дорога на оба движка (решение координатора 22.10 по #3702):

* **LLM** — цепочка ``rob_box_harness.core.assembly.build_llm_chain`` (тот же ``HealthAwareFallbackLLM``), каждый
  провайдер — в режиме команд моста ``command_bridge`` (форма запроса по ``degrade.select_mode``, один парсер, повтор по
  нарушениям ``COMMAND_RETRIES_MAX``). Ответ моста в форме v1 (текст речи или вызов ``command``) переводится обратно в
  ``command.Result`` (:func:`to_result`); цикла ``AgentCore`` нет — окно и итерации ``query`` ведёт ``turn.py``;
* **действия** — :class:`rob_box_dialog.execute.Executor` поверх исполнителя тулов v1 (``plan_host.make_tool_executor``:
  MCP под планировщиком ресурсов), исход — событие владельца из данных результата (``command_turn.owner_event``).
  Топик ``ActionRequest`` не публикуется: подписчика у него нет (mcp_server слушает ``/mcp/execute``);
* **речь / стадии / интент** — ``speech_client``, ``expression_client`` (+ рефлекс ``expression.reflex``, на итоге
  ``done`` окрашенный экспрессией команды через ``expression.merge``, #3997; пачка ``play_animation`` — очередью
  сцен за итогом, ``expression.queued``, #4083), ``/dialog/intent``.

Модуль без класса ноды: ROS-хост ``dialog_node`` (ч.8e2) отдаёт сюда себя и собранные объекты.
"""

from __future__ import annotations

import asyncio
import threading
import uuid
from typing import Any, AsyncIterator, Callable, Iterable, List, Mapping, Optional, Sequence, Tuple

from rob_box_core import expression_client, speech_client
from rob_box_core.utterance import Sink, Utterance
from rob_box_dialog import knowledge as kn
from rob_box_dialog.command import Busy, Command, Expression, Invalid, Result, ToolCall as CommandCall, parse
from rob_box_dialog.context import measure
from rob_box_dialog.deadline import ProviderFailed
from rob_box_dialog.execute import ActionRequest, Executor
from rob_box_dialog.expression import ExpressionPlan, held_face, hold_ms, merge, queued, reflex
from rob_box_dialog.respond import ActionResult
from rob_box_dialog.session import SessionSnapshot
from rob_box_dialog.turn import Answer
from rob_box_harness.core.agent_core import answering_provider
from rob_box_harness.core.tool_loop.command_turn import owner_event
from rob_box_llm.errors import ProviderError
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
            if not marked:
                return parse(dict(call.arguments))
            rest = parse({k: v for k, v in call.arguments.items() if k != kn.COMMAND_INVALID_FIELD})
            return Invalid(str(marked), salvage=rest.actions if isinstance(rest, Command) else ())
    text = (response.content or "").strip()
    return Command(speech=text) if text else Invalid(EMPTY, "ни речи, ни команды")


def stage_plan(stage: str, cls: Optional[str] = None,
               expression: Optional[Expression] = None) -> Optional[ExpressionPlan]:
    """Рефлекс стадии (``expression.reflex``), окрашенный экспрессией команды (#3997): эмоция, жест и сцена лица LLM
    (``FaceScene.to_dict()``) через ``expression.merge`` - красит только ``speaking``/``done``, провал побеждает.
    Отдельного запроса лицу нет: команда из одной ``expression`` и исполненный ``play_animation`` (``turn.Shown``,
    #4049) доходят до лица рефлексом ``done``. Время сцены - ``expression.duration_s``, решённое ходом
    (``turn.command_face``, #4083: показ по просьбе ≥ ``FACE_SHOW_MIN_S``, а не 3 с рефлекса); не задано - время
    рефлекса."""
    plan = reflex(stage, cls)
    if expression is None:
        return plan
    seconds = expression.duration_s
    return merge(plan, stage, expression.emotion, expression.gesture, held_face(expression.face, seconds),
                 hold_ms(seconds))


def show_queue(stage: str, expression: Optional[Expression]) -> Tuple[Tuple[float, ExpressionPlan], ...]:
    """Сцены пачки ``play_animation`` после первой (``turn.Shown.then``, #4083): ``(через сколько секунд, план)`` —
    по очереди за сценой итога ``done`` (``expression.queued``). На другой стадии и без пачки — пусто."""
    then = getattr(expression, "then", ()) if stage == "done" else ()
    return queued(expression.duration_s, then) if expression is not None and then else ()


def _call_later(delay_s: float, fire: Callable[[], None]) -> Any:
    """Таймер очереди лица: в asyncio-loop хода (``dialog_node``: loop ноды крутится в своём потоке, ход — его задача),
    вне loop — поток-таймер. У обоих есть ``cancel()``."""
    try:
        return asyncio.get_running_loop().call_later(delay_s, fire)
    except RuntimeError:
        timer = threading.Timer(delay_s, fire)
        timer.daemon = True
        timer.start()
        return timer


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
    """Продолжение запроса данными ``query``: вызов ``command`` модели и tool-результат с данными на него. Невалидную
    команду мост отдаёт вызовом с отметкой ``COMMAND_INVALID_FIELD`` и спасёнными действиями (#4050: спасённый
    ``query`` возвращает данные модели) - модель видит свой вызов без служебной отметки: только исполненные действия."""
    calls = tuple(c for c in response.tool_calls or () if c.name == kn.COMMAND_TOOL)
    if not calls:
        return lambda data: ({"role": "user", "content": data},)
    call = calls[0]
    if kn.COMMAND_INVALID_FIELD in call.arguments:
        call = ToolCall(id=call.id, name=call.name,
                        arguments={k: v for k, v in call.arguments.items() if k != kn.COMMAND_INVALID_FIELD})

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
                 performer: Callable[[], Any] = lambda: None,
                 later: Callable[[float, Callable[[], None]], Any] = _call_later) -> None:
        self._node = node
        #: Таймер очереди лица (#4083): ``later(через_с, fire) -> handle.cancel()``; тест подменяет его своим.
        self._later = later
        self._queue: List[Any] = []
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
        #: Хост агента (чат Telegram) берёт поздний итог ``pending``-действия: «Озвучил (12 с)» по SpeechEvent.
        self.on_late: Optional[Callable[[ActionResult], None]] = None
        self.executor = Executor(self._send, on_late=self._late, epoch=epoch)

    def provider(self) -> str:
        """Провайдер, к которому идёт вызов сейчас: цепочка ставит ``answered_by`` ДО вызова (фразы потолка, В31).
        Без цепочки — «» (имя обёртки ``health-aware-fallback`` человеку ничего не говорит)."""
        return str(getattr(self._llm, "answered_by", None) or "")

    async def ask(self, messages: Sequence[Any], tools: Sequence[Any], timeout_s: float) -> Answer:
        """Один ответ LLM (с повтором моста по нарушениям); потолок ставит ход снаружи. Провайдер не жив (цепочка
        исчерпана, транспорт оборвал вызов своим таймаутом) — :class:`ProviderFailed`: ход не ждёт потолка (В31)."""
        msgs = [to_message(m) for m in messages]
        schemas = [tool_schema(t) for t in tools]
        before = self._counter.calls
        try:
            response = await self._llm.complete(msgs, tools=schemas, settings=self._settings)
        except (ProviderError, OSError, asyncio.TimeoutError) as exc:
            raise ProviderFailed(self.provider(), exc) from exc
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
        if self.on_late is not None:
            self.on_late(result)

    def say(self, turn_id: str, epoch: int, text: str, voice: Optional[str] = None,
            emotion: Optional[str] = None) -> str:
        utterance = Utterance(text=text, sink=Sink.SPEAKERS, voice=voice or None, emotion=emotion or None,
                              speech_id=str(uuid.uuid4()))
        return speech_client.say(self._node, utterance, agent=self._agent, epoch=epoch, turn_id=turn_id)

    def stage(self, turn_id: str, epoch: int, stage: str, status: str, reason: str = "", cls: Optional[str] = None,
              llm_calls: int = 0, provider: str = "", source: str = "",
              expression: Optional[Expression] = None) -> None:
        """Событие стадии (``/dialog/turn_event``) и рефлекс кода (лицо/кольцо/earcon) без LLM. ``expression`` -
        экспрессия команды на итоге хода (``turn.done_expression``: не в тишине, не у отменённого): красит рефлекс
        (:func:`stage_plan`), жест и сцена лица LLM уходят в том же ``ExpressionRequest``; строка ``🎭 [turn-v2]``.
        Пачка ``play_animation`` (#4083) — остальные сцены по очереди за итогом (:func:`show_queue`), каждая своим
        запросом в свой срок; любая следующая стадия (новый ход, стоп) недопоказанную очередь снимает."""
        status = _STATUS.get(status, status)
        self._drop_queue(stage, turn_id)
        expression_client.turn_event(self._node, turn_id=turn_id, epoch=epoch, agent=self._agent, stage=stage,
                                     status=status, reason=reason, llm_calls=llm_calls, llm_provider=provider,
                                     source=source)
        plan = stage_plan(stage, cls, expression)
        if plan is not None and expression is not None:
            held = f" ttl_ms={plan.ttl_ms}" if expression.duration_s is not None else ""
            self._log(f"🎭 [turn-v2] лицо команды на {stage}: emotion={plan.emotion} gesture={plan.gesture or '-'} "
                      f"face={'llm' if plan.face else '-'}{held} turn={turn_id}")
        if plan is not None:
            expression_client.express(self._node, turn_id=turn_id, epoch=epoch, agent=self._agent,
                                      emotion=plan.emotion, gesture=plan.gesture, earcon=plan.earcon, face=plan.face,
                                      ttl_ms=plan.ttl_ms)
        self._queue_shows(turn_id, epoch, show_queue(stage, expression))

    def _queue_shows(self, turn_id: str, epoch: int, shows: Sequence[Tuple[float, ExpressionPlan]]) -> None:
        """Сцены пачки — таймерами в свой срок (строка ``🎭 [turn-v2] очередь лица`` для живой проверки)."""
        if not shows:
            return
        steps = ", ".join(f"{p.emotion or 'face'} через {at:g} с на {p.ttl_ms / 1000:g} с" for at, p in shows)
        self._log(f"🎭 [turn-v2] очередь лица: {steps} turn={turn_id}")
        for at, plan in shows:
            self._queue.append(self._later(at, lambda p=plan: self._show_queued(turn_id, epoch, p)))

    def _show_queued(self, turn_id: str, epoch: int, plan: ExpressionPlan) -> None:
        """Сцена очереди пришла в свой срок: ``ExpressionRequest`` без earcon и жеста — только сцена и её время."""
        if self._queue:
            self._queue.pop(0)
        self._log(f"🎭 [turn-v2] лицо из очереди: emotion={plan.emotion or '-'} face={'llm' if plan.face else '-'} "
                  f"ttl_ms={plan.ttl_ms} turn={turn_id}")
        expression_client.express(self._node, turn_id=turn_id, epoch=epoch, agent=self._agent, emotion=plan.emotion,
                                  face=plan.face, ttl_ms=plan.ttl_ms)

    def _drop_queue(self, stage: str, turn_id: str) -> None:
        """Новая стадия (следующий ход, стоп): недопоказанные сцены пачки снимаются — лицо отдаёт новому ходу."""
        if not self._queue:
            return
        pending, self._queue = self._queue, []
        for handle in pending:
            handle.cancel()
        self._log(f"🎭 [turn-v2] очередь лица снята: {len(pending)} сцен (стадия {stage} turn={turn_id})")

    def cancel_speech(self, epoch: int, reason: str, boundary: str = "as_requested") -> None:
        speech_client.cancel(self._node, agent=self._agent, epoch=epoch, reason=reason, boundary=boundary)

    def stop_perform(self, how: str, keep_backing: bool = False) -> bool:
        """Перебивание выступления (``PerformHost.halt``/``stop``); ``True`` — играющая музыка — его бит, и его
        снимает само выступление. ``keep_backing`` («а музыку оставь», #3994) — бит доигрывает: выступление его не
        снимает (``False``), общий стоп музыки при оговорке тоже не зовётся (``interrupt.stops_music``)."""
        host = self._performer()
        if host is None:
            return False
        owns = bool(host.owns_backing) and not keep_backing
        stopped = host.halt(keep_backing=keep_backing) if how == "halt" else host.stop(keep_backing=keep_backing)
        return bool(stopped) and owns

    def perform(self, turn_id: str, epoch: int, perf: Any, edit: bool) -> Optional[Mapping[str, Any]]:
        """Выступление — хосту ``PerformHost`` (``PerformBlock``: такты, подложка, паузы MiniMax по сетке строк), как
        у v1 (``command_turn.CommandTurn._perform``); ``edit`` — ``edit_pending``. Хоста нет — ``None``."""
        host = self._performer()
        if host is None:
            return None
        return host.edit(perf) if edit else host.start(perf, turn_id, epoch)

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


def assemble_llm(chain: Sequence[str], counter: CallCounter, *, prompt_dir: Any, system_prompt_file: str,
                 settings: Optional[LLMSettings] = None, agent: str = kn.AGENTS[0],
                 log: Callable[[str], None] = lambda line: None, busy: Optional[Callable[[], Busy]] = None) -> tuple:
    """LLM-цепочка v2 агента и его системный промпт: ``build_llm_chain`` с каждым провайдером в режиме команд моста
    (валидатор знает агента), вызовы считаются в ``counter``. Одна сборка для ROS-хоста ``dialog_node`` и сухого
    стенда (``scripts/dialog/dry_marathon.py``, #3906): ответ модели строит та же цепочка, что на роботе."""
    from rob_box_harness.core.assembly import AgentSpec, build_llm_chain, load_system_prompt  # noqa: PLC0415

    from . import command_bridge  # noqa: PLC0415

    names = tuple(p.strip().lower() for p in chain if p.strip())
    bridge = command_bridge.wrapper_for(command_providers(names), log=log, busy=busy or Busy, agent=agent)
    spec = AgentSpec(name=agent, prompt_dir=prompt_dir, system_prompt_file=system_prompt_file, provider_chain=names,
                     settings=settings, wrap_provider=counted_wrapper(bridge, counter))
    return build_llm_chain(spec), load_system_prompt(spec)


__all__ = ["CallCounter", "Counted", "EMPTY", "V2Ports", "assemble_llm", "command_providers", "counted_wrapper",
           "follow_for", "show_queue", "stage_plan", "to_message", "to_result", "tool_schema"]
