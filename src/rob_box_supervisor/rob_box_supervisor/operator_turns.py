"""ТАРС на общем движке диалога (ADR-0150 §9.3, решение В7; PR-13 #3708) — путь супервизора при ``dialog_engine=v2``.

Оператор говорит с тем же движком, что Личность (:class:`rob_box_dialog.turn.TurnEngine`): своя сессия
``agent="operator"`` со своим окном ходов, грамматика Tier-1 и срез тулов — из ``AgentSpec(operator)``, те же
``Command`` / ``Executor`` / ``phrase_from_result``. Ответ оператору (``/avatar/command_result``: панель шлема, чат) строит
код из хода: слова модели, что звучат по правилу §4.1, и фраза кода об исходе по событию владельца; ``tool_calls`` —
исполненные действия с их исходом, а не имена из ответа модели (пустой ``tool_calls`` у «не вышло» больше невозможен:
отказ без действия не исполнен — фразы о нём нет).

``say`` — действие ``action.speak_aloud``: тул ждёт только начала реплики (``pending`` с ``event_ref=speech_id``), итог
«Озвучил (N с)» приходит позже событием ``SpeechEvent finished`` (:meth:`OperatorHost.on_speech`) и уходит вторым
``command_result`` того же запроса (:attr:`OperatorPorts.on_late`). Сроки обещаний — те же, что у чата
(:class:`rob_box_dialog.chats.ChatTurns`: одна реализация). Путь v1 (``AgentCore``) живёт до PR-15.

Модуль без ROS: LLM-цепочку, MCP-тулы и публикации даёт ``supervisor_node``.
"""

from __future__ import annotations

import asyncio
import logging
import threading
import uuid
from dataclasses import dataclass
from typing import Any, Callable, Dict, List, Mapping, Optional, Sequence, Tuple

from rob_box_dialog.address import Addressed
from rob_box_dialog.chats import ChatTurns
from rob_box_dialog.respond import ActionResult, phrase_from_result

_LOG = logging.getLogger(__name__)
#: Агент ТАРС в ``knowledge.AGENTS``.
OPERATOR = "operator"
#: Строка лога хода ТАРС v2.
LOG_MARK = "🛰 [tars-v2]"
#: ``summary`` хода без слов (Tier-1 стоп, громкость без фразы): пусто — в шлем не звучит, консоль и чат молчат.
NO_REPLY = ""
#: Сколько поток ROS ждёт ход сверх его потолка (``TURN_CEILING_S``, В31): потолок хода ставит сам движок.
RUN_SLACK_S = 10.0
#: Поздний итог действия: ``(request_id команды, результат, фраза кода)``.
OnLate = Callable[[str, ActionResult, str], None]


@dataclass(frozen=True)
class OperatorReply:
    """Итог хода ТАРС для ``/avatar/command_result``: ``ok`` — ход дошёл до конца, ``summary`` — что сказано,
    ``tool_calls`` — исполненные действия ``{name, status, reason}``."""

    ok: bool
    summary: str
    tool_calls: Tuple[Dict[str, str], ...]


def tool_call(result: ActionResult) -> Dict[str, str]:
    return {"name": result.tool, "status": result.status, "reason": result.reason or ""}


def reply_of(status: str, said: Sequence[str], results: Sequence[ActionResult]) -> OperatorReply:
    """Итог хода из хода, а не из слов модели: ``status`` агрегата хода, сказанное (модель и код), исходы действий."""
    text = " ".join(t.strip() for t in said if t and t.strip())
    return OperatorReply(status == "finished", text or NO_REPLY, tuple(tool_call(r) for r in results))


class OperatorPorts:
    """Порты хода ТАРС: LLM, исполнение и отмена речи — общие порты v2 (``base``); реплика — текст ответа оператору
    (голос в шлем публикует супервизор по источнику команды). Лицо и кольцо робота стадии ТАРС не трогают: оператор
    говорит не с роботом вслух; выступления у ТАРС нет."""

    def __init__(self, base: Any, log: Callable[[str], None] = _LOG.info) -> None:
        self._base = base
        self._log = log
        self.request_id = ""
        self._said: List[str] = []
        self._results: List[ActionResult] = []
        self._owners: Dict[str, str] = {}  # turn_id запроса обещания -> request_id команды оператора
        self.on_late: Optional[OnLate] = None

    def begin(self, request_id: str) -> None:
        self.request_id, self._said, self._results = request_id, [], []

    def reply(self, status: str) -> OperatorReply:
        return reply_of(status, self._said, self._results)

    def ask(self, messages: Sequence[Any], tools: Sequence[Any], timeout_s: float) -> Any:
        return self._base.ask(messages, tools, timeout_s)

    async def execute(self, actions: Sequence[Any], snapshot: Any, turn_id: str) -> List[ActionResult]:
        results = await self._base.execute(actions, snapshot, turn_id)
        self._results.extend(results)
        self._owners.update({r.turn_id: self.request_id for r in results if r.status == "pending"})
        return results

    def late(self, result: ActionResult) -> None:
        """Поздний итог (``Executor.on_late``): фраза кода по событию владельца — оператору тем же запросом."""
        said = phrase_from_result(result)
        request_id = self._owners.pop(result.turn_id, "")
        self._log(f"{LOG_MARK} итог {result.tool} {result.status} {result.reason or '-'} request={request_id or '-'}")
        if self.on_late is not None:
            self.on_late(request_id, result, said.text if said is not None else "")

    def say(self, turn_id: str, epoch: int, text: str, voice: Optional[str] = None,
            emotion: Optional[str] = None) -> str:
        self._said.append(text)
        return uuid.uuid4().hex

    def stage(self, turn_id: str, epoch: int, stage: str, status: str, reason: str = "", cls: Optional[str] = None,
              llm_calls: int = 0, provider: str = "", source: str = "") -> None:
        if status != "started":
            self._log(f"{LOG_MARK} turn={turn_id} stage={stage} status={status} reason={reason or '-'} "
                      f"llm_calls={llm_calls} provider={provider or '-'}")

    def cancel_speech(self, epoch: int, reason: str, boundary: str = "as_requested") -> None:
        self._base.cancel_speech(epoch, reason, boundary)

    def stop_perform(self, how: str) -> bool:
        return False

    def perform(self, turn_id: str, epoch: int, perf: Any, edit: bool) -> None:
        """Выступлений у ТАРС нет: сегменты звучат репликами хода."""
        return None

    def intent(self, turn_id: str, epoch: int, kind: str) -> None:
        self._base.intent(turn_id, epoch, kind)


class OperatorHost:
    """Ходы ТАРС по очереди в своём asyncio-цикле (поток): обещания ``say`` ждут ``SpeechEvent`` и после команды."""

    def __init__(self, engine: Any, ports: OperatorPorts, executor: Any, log: Callable[[str], None] = _LOG.info,
                 loop: Optional[asyncio.AbstractEventLoop] = None) -> None:
        self.engine = engine
        self.ports = ports
        self._log = log
        self._promises = ChatTurns(lambda _channel: engine, (), log)  # только сроки и итоги обещаний речи
        self._promises.adopt(executor)
        self._loop = loop or asyncio.new_event_loop()
        self._lock: Optional[asyncio.Lock] = None
        self._thread = threading.Thread(target=self._loop.run_forever, name="tars_v2_loop", daemon=True)
        self._thread.start()

    def run(self, request_id: str, text: str, source: str, timeout_s: float) -> OperatorReply:
        """Один ход оператора до конца (блокирует вызывающий поток ROS не дольше ``timeout_s``)."""
        future = asyncio.run_coroutine_threadsafe(self._turn(request_id, text, source), self._loop)
        return future.result(timeout_s)

    async def _turn(self, request_id: str, text: str, source: str) -> OperatorReply:
        self._lock = self._lock or asyncio.Lock()
        async with self._lock:
            self.ports.begin(request_id)
            agg = await self.engine.run(Addressed(text=text, source=source or "operator"))
            reply = self.ports.reply(agg.status)
            self._log(f"{LOG_MARK} request={request_id} status={agg.status} latency_ms={agg.latency_ms} "
                      f"llm_calls={agg.llm_calls} tools={[c['name'] for c in reply.tool_calls]}")
            return reply

    def run_payload(self, payload: Mapping[str, Any]) -> Dict[str, Any]:
        """Ход команды ``/avatar/command`` в форме v1-пути супервизора (``{ok, summary, tool_calls, …}``): отказ —
        честный ``ok=false`` с кодом, а не исключение в колбэке ROS."""
        from rob_box_dialog import knowledge as kn  # noqa: PLC0415

        text = str(payload.get("text", "") or "").strip()
        meta = {"source": str(payload.get("source", "") or ""), "client_id": str(payload.get("client_id", "") or "")}
        if not text:
            return {"ok": False, "summary": "empty_input", "tool_calls": [], **meta}
        try:
            reply = self.run(str(payload.get("request_id", "") or ""), text, meta["source"],
                             kn.TURN_CEILING_S + RUN_SLACK_S)
        except Exception as exc:  # noqa: BLE001 — упавший ход не роняет супервизор, но громко
            self._log(f"{LOG_MARK} ход упал: {type(exc).__name__}: {exc}")
            return {"ok": False, "summary": f"llm_error: {type(exc).__name__}", "tool_calls": [], "error": str(exc),
                    **meta}
        return {"ok": reply.ok, "summary": reply.summary, "tool_calls": list(reply.tool_calls), **meta}

    def on_speech(self, event: Any) -> bool:
        """``SpeechEvent`` реплики агента ``operator``: итог ``say`` тому обещанию, что его ждёт."""
        return self._promises.on_speech(event)

    def tick(self) -> None:
        self._promises.tick()

    def shutdown(self) -> None:
        self._loop.call_soon_threadsafe(self._loop.stop)
        self._thread.join(timeout=5.0)


def build_host(node: Any) -> Tuple[Any, Any]:
    """ТАРС v2 у супервизора — та же дорога, что у ``dialog_node``: цепочка ``build_llm_chain`` в режиме команд моста
    (валидатор знает агента ``operator``), MCP-тулы оператора, ``V2Ports`` с исполнителем. ``(хост, None)``; сбой
    сборки — ``(None, None)`` (как у v1: ``agent_unavailable``, а не падение ноды)."""
    import time  # noqa: PLC0415
    from pathlib import Path  # noqa: PLC0415

    from rob_box_core import dialog_engine  # noqa: PLC0415
    from rob_box_dialog import knowledge as kn  # noqa: PLC0415
    from rob_box_dialog.session import Session  # noqa: PLC0415
    from rob_box_dialog.turn import TurnEngine  # noqa: PLC0415
    from rob_box_dialog_msgs.msg import Intent, SpeechEvent  # noqa: PLC0415
    from rob_box_harness.core.assembly import AgentSpec, build_llm_chain, load_system_prompt  # noqa: PLC0415
    from rob_box_voice.core import command_bridge  # noqa: PLC0415
    from rob_box_voice.core.dialog_v2_ports import CallCounter, V2Ports, command_providers, counted_wrapper  # noqa

    from rob_box_supervisor.operator_prompt import operator_prompt  # noqa: PLC0415

    tools = node._build_operator_tools()
    if tools is None:
        return (None, None)
    log = node.get_logger().info
    chain = node._parse_provider_chain(node._param_str("llm_providers", kn.COMMAND_PROVIDERS_DEFAULT))
    counter = CallCounter()
    bridge = command_bridge.wrapper_for(command_providers(chain), log=log, agent=OPERATOR)
    spec = AgentSpec(name=OPERATOR, prompt_dir=node._resolve_prompts_dir() or Path(""),
                     system_prompt_file=node._system_prompt_file, provider_chain=chain,
                     settings=node._build_operator_llm_settings(), wrap_provider=counted_wrapper(bridge, counter))
    session = Session(OPERATOR, clock=time.time)
    # «стой» оператора (Tier-1) гасит и движение: интент ``stop_motion`` владельцу движения (command_node)
    intent_pub = node.create_publisher(Intent, "/dialog/intent", 10)
    base = V2Ports(node, build_llm_chain(spec), counter, tools, agent=OPERATOR, intent_pub=intent_pub, log=log,
                   epoch=lambda: session.snapshot.epoch)
    ports = OperatorPorts(base, log)
    base.on_late = ports.late
    node._operator_journal = node._build_operator_journal()
    engine = TurnEngine(session, ports, system_prompt=operator_prompt(load_system_prompt(spec), dialog_engine.V2),
                        log=node.get_logger(),
                        sections=lambda _snap, _text: {"operator_journal": node._render_journal_context()})
    host = OperatorHost(engine, ports, base.executor, log)
    ports.on_late = lambda rid, result, text: node._publish_command_result(
        rid or uuid.uuid4().hex, {"ok": result.status == "done", "summary": text, "tool_calls": [tool_call(result)]})
    node.create_subscription(SpeechEvent, "/voice/speech/event",
                             lambda msg: msg.agent == OPERATOR and host.on_speech(msg), 10)
    node.create_timer(5.0, host.tick)
    log(f"{LOG_MARK} ТАРС на общем движке: providers={chain} prompt={node._system_prompt_file}")
    return (host, None)


def agent_path(node: Any, engine: str) -> Tuple[Callable[[], Tuple[Any, Any]], Callable[[Any, Mapping[str, Any]], Any]]:
    """Сборка и ход агента ТАРС по флагу движка: ``v1`` — ``AgentCore`` супервизора, ``v2`` — общий движок."""
    if engine == "v2":
        return (lambda: _safe_build(node)), (lambda host, payload: host.run_payload(payload))
    # v1 — позднее связывание: тесты и PR-15 подменяют методы ноды
    return (lambda: node._build_agent_core_sync()), (lambda core, payload: node._run_agent_sync(core, payload))


def _safe_build(node: Any) -> Tuple[Any, Any]:
    try:
        return build_host(node)
    except Exception as exc:  # noqa: BLE001 — нет движка: agent_unavailable в ответе, нода живёт (ADR-0018)
        node.get_logger().warning(f"{LOG_MARK} сборка ТАРС v2 не удалась: {type(exc).__name__}: {exc}")
        return (None, None)


__all__ = ["LOG_MARK", "NO_REPLY", "OPERATOR", "OperatorHost", "OperatorPorts", "OperatorReply", "agent_path",
           "build_host", "reply_of", "tool_call"]
