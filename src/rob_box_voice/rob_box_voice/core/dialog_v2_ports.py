"""Транспорт хода v2 (``rob_box_dialog.turn.Ports``) поверх того, что уже есть у v1 (ADR-0150 §3; PR-8 ч.8e #3702).

Одна дорога на оба движка (решение координатора 22.10 по #3702):

* **LLM** — цепочка ``rob_box_harness.core.assembly.build_llm_chain`` (тот же ``HealthAwareFallbackLLM``), каждый
  провайдер — в режиме команд ``command_bridge`` (форма запроса по ``degrade.select_mode``, один парсер, повтор по
  нарушениям ``COMMAND_RETRIES_MAX``); его ответ несёт ``command.Result`` (``CommandResponse.command``), окно и
  итерации ``query`` ведёт ``turn.py``;
* **действия** — :class:`rob_box_dialog.execute.Executor` поверх исполнителя тулов v1 (``plan_host.make_tool_executor``:
  MCP под планировщиком ресурсов), исход — событие владельца из данных результата (``execute.owner_event``).
  Топик ``ActionRequest`` не публикуется: подписчика у него нет (mcp_server слушает ``/mcp/execute``);
* **речь / стадии / интент** — ``speech_client``, ``expression_client`` (+ рефлекс ``expression.reflex``, на итоге
  ``done`` окрашенный экспрессией команды через ``expression.merge``, #3997; пачка ``play_animation`` — очередью
  сцен за итогом, ``expression.queued``, #4083), ``/dialog/intent``; лицо пульта агента чата — :meth:`V2Ports.show`
  (#4040: та же сцена без стадии хода и earcon'а).

Модуль без класса ноды: ROS-хост ``dialog_node`` (ч.8e2) отдаёт сюда себя и собранные объекты.
"""

from __future__ import annotations

import asyncio
import threading
import time
import uuid
from dataclasses import replace
from typing import Any, Callable, Iterable, List, Mapping, Optional, Sequence, Tuple

from rob_box_core import expression_client, speech_client
from rob_box_core.utterance import Sink, Utterance
from rob_box_dialog import knowledge as kn
from rob_box_dialog.command import Busy, Expression, Result, ToolCall as CommandCall
from rob_box_dialog.context import measure
from rob_box_dialog.deadline import ProviderFailed
from rob_box_dialog.execute import ActionRequest, Executor, owner_event
from rob_box_dialog.expression import ExpressionPlan, held_face, hold_ms, merge, queued, reflex
from rob_box_dialog.respond import ActionResult
from rob_box_dialog.session import SessionSnapshot
from rob_box_dialog.turn import Answer
from rob_box_llm.errors import ProviderError
from rob_box_llm.provider import LLMMessage, LLMResponse, LLMSettings, ToolCall, ToolResult

from .command_bridge import EMPTY, model_call, wrapper_for
from .command_intent import fill_intent
from .command_parser import Command as MotionCommand, IntentType

_STATUS = {"done": "finished", "failed": "failed", "timeout": "failed"}


def answering_provider(llm: Any) -> str:
    """Имя провайдера, ответившего на последний вызов: у цепочки с фолбеком — ``answered_by``, иначе ``name`` LLM.

    ``name`` обёртки (``health-aware-fallback``) один на все провайдеры, и метрика A5 по провайдерам по нему не
    собирается (живая проверка #3696 ч.4, 21.10). Перенесено из ``agent_core`` v1 (#4354 A3b).
    """
    return getattr(llm, "answered_by", None) or getattr(llm, "name", "unknown")


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


def counted_wrapper(bridge_wrap: Callable[[Any], Any], counter: CallCounter) -> Callable[[Any], Any]:
    """``AgentSpec.wrap_provider`` для v2: счётчик вызовов внутри моста команд (повтор считается)."""
    return lambda provider: bridge_wrap(Counted(provider, counter))


def result_of(response: LLMResponse) -> Result:
    """Команда хода из ответа цепочки (``CommandResponse.command``). Ответа без команды v2 не ждёт: провайдер цепочки
    без режима команд — ошибка сборки, ход падает громко, а не молчит."""
    result = getattr(response, "command", None)
    if result is None:
        raise TypeError(f"ответ LLM без команды ({type(response).__name__}): провайдер цепочки не в режиме команд")
    return result


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


def show_plan(expression: Optional[Expression]) -> Optional[ExpressionPlan]:
    """Лицо команды вне хода робота (пульт агента чата, #4040): тот же план, что у итога ``done`` (:func:`stage_plan`:
    сцена, жест, сцена лица LLM, время показа), но без earcon'а «готово» - хода у робота нет, звенеть нечему."""
    plan = stage_plan("done", None, expression)
    return replace(plan, earcon="") if plan is not None else None


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


def follow_for(result: Result) -> Callable[[str], Sequence[Any]]:
    """Продолжение запроса данными ``query``: вызов ``command`` модели (``command_bridge.model_call``) и
    tool-результат с данными на него. У невалидной команды модель видит в своём вызове только спасённые действия
    (#4050: спасённый ``query`` возвращает данные модели). Вызова нет (голая реплика) — данные репликой пользователя."""
    call = model_call(result)
    if call is None:
        return lambda data: ({"role": "user", "content": data},)

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
                 later: Callable[[float, Callable[[], None]], Any] = _call_later,
                 on_ask: Callable[..., None] = lambda provider, *, ok, duration_s: None) -> None:
        self._node = node
        #: Таймер очереди лица (#4083): ``later(через_с, fire) -> handle.cancel()``; тест подменяет его своим.
        self._later = later
        #: Итог вызова цепочки LLM (провайдер, ok, задержка) - хост ставит метрики (#4328); ядро без prometheus.
        self._on_ask = on_ask
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
        started = time.monotonic()
        try:
            response = await self._llm.complete(msgs, tools=schemas, settings=self._settings)
        except (ProviderError, OSError, asyncio.TimeoutError) as exc:
            self._on_ask(self.provider(), ok=False, duration_s=time.monotonic() - started)
            raise ProviderFailed(self.provider(), exc) from exc
        provider = answering_provider(self._llm)
        self._on_ask(provider, ok=True, duration_s=time.monotonic() - started)
        usage = dict(response.usage or {})
        tokens = int(usage.get("prompt_tokens") or measure(msgs, schemas).total)
        self._log(f"🧮 [prompt] tokens={tokens} estimated={'prompt_tokens' not in usage} provider={provider} "
                  f"view=v2 tools={len(schemas)}")
        result = result_of(response)
        return Answer(result, provider, max(1, self._counter.calls - before), tokens, follow_for(result))

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
            emotion: Optional[str] = None, boundary: str = "none") -> str:
        """Реплика хода; ``boundary="sentence"`` - счёт (#4207): отмена дочитывает звучащее предложение-число."""
        utterance = Utterance(text=text, sink=Sink.SPEAKERS, voice=voice or None, role=voice or "", emotion=emotion or None,
                              speech_id=str(uuid.uuid4()))
        return speech_client.say(self._node, utterance, agent=self._agent, epoch=epoch, turn_id=turn_id,
                                 boundary=boundary)

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
        self._express(turn_id, epoch, plan, show_queue(stage, expression))

    def show(self, turn_id: str, epoch: int, expression: Expression) -> None:
        """Лицо команды без стадии хода (пульт агента чата, #4040, ``dialog_node.RobotRemote``): сцена и очередь пачки
        ``play_animation`` — как на итоге ``done`` (:func:`show_plan`), но ``TurnEvent`` нет: хода у робота нет, кольцо
        и метрики хода не трогаются. Недопоказанную очередь прошлого хода снимает, как любая новая стадия. Строка лога
        своя (``лицо пульта``): разбор марафона считает «лицо команды на done» ответом хода Личности."""
        self._drop_queue("show", turn_id)
        plan = show_plan(expression)
        if plan is not None:
            self._log(f"🎭 [turn-v2] лицо пульта чата: emotion={plan.emotion} gesture={plan.gesture or '-'} "
                      f"face={'llm' if plan.face else '-'} ttl_ms={plan.ttl_ms} turn={turn_id}")
        self._express(turn_id, epoch, plan, show_queue("done", expression))

    def _express(self, turn_id: str, epoch: int, plan: Optional[ExpressionPlan],
                 shows: Sequence[Tuple[float, ExpressionPlan]]) -> None:
        """План лица — ``ExpressionRequest``, сцены пачки — в очередь за ним."""
        if plan is not None:
            expression_client.express(self._node, turn_id=turn_id, epoch=epoch, agent=self._agent,
                                      emotion=plan.emotion, gesture=plan.gesture, earcon=plan.earcon, face=plan.face,
                                      ttl_ms=plan.ttl_ms)
        self._queue_shows(turn_id, epoch, shows)

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

    def cancel_speech(self, epoch: int, reason: str, boundary: str = "as_requested", speech_id: str = "") -> None:
        """Отмена речи эпохи; ``speech_id`` - ровно этой реплики (правка звучащего счёта, #4207)."""
        speech_client.cancel(self._node, agent=self._agent, speech_id=speech_id or None, epoch=epoch, reason=reason,
                             boundary=boundary)

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
        return host.edit(perf, turn_id, epoch) if edit else host.start(perf, turn_id, epoch)

    def intent(self, turn_id: str, epoch: int, kind: str) -> None:
        """Интент движения для ``command_node`` (сейчас — только ``stop_motion`` Tier-1 стопа)."""
        if self._intent_pub is None or kind != "stop_motion":
            return
        from rob_box_dialog_msgs.msg import Intent  # noqa: PLC0415 — сообщения ROS только в рантайме ноды

        stop = MotionCommand(intent=IntentType.STOP, text="", entities={}, confidence=1.0)
        self._intent_pub.publish(fill_intent(Intent(), stop, epoch=epoch, turn_id=turn_id))


def assemble_llm(chain: Sequence[str], counter: CallCounter, *, prompt: str,
                 settings: Optional[LLMSettings] = None, agent: str = kn.AGENTS[0],
                 log: Callable[[str], None] = lambda line: None, busy: Optional[Callable[[], Busy]] = None) -> tuple:
    """LLM-цепочка v2 агента и его системный промпт: ``build_llm_chain`` с каждым провайдером в режиме команд моста
    (валидатор знает агента), вызовы считаются в ``counter``. Одна сборка для ROS-хоста ``dialog_node`` и сухого
    стенда (``scripts/dialog/dry_marathon.py``, #3906): ответ модели строит та же цепочка, что на роботе. ``prompt`` -
    основа промпта агента, её даёт хост (Личность - ``rob_box_dialog.prompts`` с персоной профиля, ADR-0167 Ш1б-3)."""
    from rob_box_harness.core.assembly import AgentSpec, build_llm_chain  # noqa: PLC0415

    names = tuple(p.strip().lower() for p in chain if p.strip())
    bridge = wrapper_for(log=log, busy=busy or Busy, agent=agent)
    spec = AgentSpec(name=agent, provider_chain=names, settings=settings,
                     wrap_provider=counted_wrapper(bridge, counter))
    return build_llm_chain(spec), system_prompt(prompt, agent)


def system_prompt(prompt: str, agent: str) -> str:
    """Системный промпт агента: основа (персона и протокол хода) + правила из ``knowledge.PROMPT_RULES`` (PR-15 15-rule,
    ADR-0148: правила модели живут в таблице знания, а не в тексте основы). Основы нет — промпта нет (агент не
    соберётся)."""
    base = prompt.strip()
    rules = kn.prompt_rules(agent)
    return f"{base}\n\n{rules}" if base and rules else base


__all__ = ["CallCounter", "Counted", "EMPTY", "V2Ports", "assemble_llm", "counted_wrapper", "follow_for", "result_of",
           "show_plan", "show_queue", "stage_plan", "system_prompt", "to_message", "tool_schema"]
