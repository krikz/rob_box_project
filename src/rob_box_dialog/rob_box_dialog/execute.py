"""Исполнение команды: действие → запрос владельцу → событие владельца → :class:`ActionResult` (ADR-0150 §5.2-5.3).

Успех — это событие владельца (плеер, владелец речи, исполнители экспрессии, звук, память, ``speaker_id_node``,
владелец голоса), а не публикация запроса и не слова LLM (ADR-0148). :class:`Executor` шлёт на каждое действие
команды :class:`ActionRequest` и ждёт :class:`OwnerEvent` с тем же ``turn_id`` запроса (или с ``event_ref``,
который владелец выдал раньше: ``track_id``, ``speech_id``, ``goal_id``). Нет события к дедлайну класса
(``knowledge.ACTION_CLASSES``) — честный ``timeout``; событие, которого никто не ждёт, — ``WARNING stale_event``.

Действия хода уходят владельцам разом (§5.2: совместимые — параллельно): ждать ли занятый ресурс и что после чего —
решает планировщик ресурсов у транспорта (``plan.runner``, PR-7b) по ``knowledge.COMPOSITION``. Событие
``pending`` (обещание: «Еду к…») закрывает ожидание статусом ``pending``; итог по его ``event_ref`` придёт позже —
в ``on_late``. Транспорт (ROS-топик, вызов MCP) — снаружи: модуль чистый, без ROS.

Эпоха (PR-9 #3703, §6.2): исполнитель с ``epoch`` (текущая эпоха сессии) отбрасывает исход запроса прошлой эпохи —
``WARNING stale_epoch``, в ход он приходит ``cancelled{stale_epoch}``, в ``on_late`` не попадает. Отмена хода
(перебивание) запросы непрерываемых классов (``interrupt.survives``: память, голос, знакомство, движение) не рвёт:
их отправка доживает до события владельца; остальные отменяются вместе с ходом.
"""

from __future__ import annotations

import inspect
import logging
from dataclasses import dataclass, field
from types import MappingProxyType
from typing import TYPE_CHECKING, Any, Callable, Dict, List, Mapping, Optional, Sequence

from rob_box_dialog import knowledge as kn
from rob_box_dialog.interrupt import survives
from rob_box_dialog.respond import ActionResult

if TYPE_CHECKING:  # asyncio тянет socket: импорт пакета остаётся чистым (test_purity), цикл — при исполнении
    import asyncio

_LOG = logging.getLogger(__name__)
#: Итоговые статусы события владельца; ``pending`` — промежуточный (обещание), ``timeout`` ставит только исполнитель.
_FINAL = ("done", "rejected", "cancelled")


@dataclass(frozen=True)
class ActionRequest:
    """Запрос владельцу ресурса (§10.1 ``ActionRequest.msg``). ``turn_id`` — id ЭТОГО вызова (``<ход>/<n>``):
    по нему владелец отвечает событием и отбрасывает повтор (идемпотентность, ADR-0149 I6)."""

    turn_id: str
    epoch: int
    agent: str
    tool: str
    args: Mapping[str, Any]
    cls: str
    timeout_s: float


@dataclass(frozen=True)
class OwnerEvent:
    """Ответ владельца на запрос: ``ref`` — ``turn_id`` запроса или выданный ранее ``event_ref``."""

    ref: str
    status: str
    reason: Optional[str] = None
    data: Mapping[str, Any] = field(default_factory=lambda: MappingProxyType({}))
    event_ref: Optional[str] = None


Send = Callable[[ActionRequest], Any]
OnLate = Callable[[ActionResult], Any]
Epoch = Callable[[], int]


def deadline_s(cls: str) -> float:
    """Дедлайн события класса; у классов, где его задаёт владелец (речь, выступление), — бюджет хода."""
    timeout = kn.ACTION_CLASSES[cls].timeout_s
    return float(timeout if timeout is not None else kn.TURN_DEADLINE_S)


class Executor:
    """Исполнитель одного агента: запросы владельцам и сопоставление их событий с ожиданиями."""

    def __init__(self, send: Send, on_late: Optional[OnLate] = None, log: Optional[logging.Logger] = None,
                 epoch: Optional[Epoch] = None) -> None:
        self._send = send
        self._on_late = on_late
        self._epoch = epoch
        self._log = log or _LOG
        self._loop: Optional["asyncio.AbstractEventLoop"] = None
        self._waiting: Dict[str, "asyncio.Future[OwnerEvent]"] = {}
        self._requests: Dict[str, ActionRequest] = {}  # ref ожидания или обещания → запрос

    async def run(self, actions: Sequence[Any], snapshot: Any, turn_id: str) -> List[ActionResult]:
        """Исполнить действия команды (``ToolCall`` с ``tool``/``args``) разом; результаты — в порядке команды."""
        import asyncio  # noqa: PLC0415

        self._loop = asyncio.get_running_loop()
        requests = []
        for n, call in enumerate(actions):
            cls = kn.TOOL_CLASSES[call.tool]
            requests.append(ActionRequest(f"{turn_id}/{n}", snapshot.epoch, snapshot.agent, call.tool,
                                          MappingProxyType(dict(call.args)), cls, deadline_s(cls)))
        return list(await asyncio.gather(*(self._one(r) for r in requests)))

    def on_event(self, event: OwnerEvent) -> None:
        """Событие владельца из любого потока (колбэк ROS); сопоставление — в цикле исполнителя."""
        loop = self._loop
        if loop is None or loop.is_closed():
            self._stale(event)
            return
        import asyncio  # noqa: PLC0415

        try:
            running = asyncio.get_running_loop()
        except RuntimeError:
            running = None
        if running is loop:
            self._deliver(event)
        else:
            loop.call_soon_threadsafe(self._deliver, event)

    async def _one(self, request: ActionRequest) -> ActionResult:
        import asyncio  # noqa: PLC0415

        assert self._loop is not None
        future: "asyncio.Future[OwnerEvent]" = self._loop.create_future()
        self._waiting[request.turn_id] = future
        self._requests[request.turn_id] = request
        survivor = False
        try:
            sent = self._send(request)
            if inspect.isawaitable(sent):
                await (asyncio.shield(sent) if survives(request.cls) else sent)
            event = await asyncio.wait_for(asyncio.shield(future), request.timeout_s)
        except asyncio.CancelledError:
            survivor = survives(request.cls)  # исход начатого непрерываемого дойдёт до _deliver (stale_epoch/on_late)
            raise
        except asyncio.TimeoutError:
            self._log.warning(f"⏱ [execute] {request.tool} {request.turn_id}: нет события за {request.timeout_s:.0f} с")
            return _result(request, OwnerEvent(request.turn_id, "timeout"))
        except Exception as exc:  # noqa: BLE001 — запрос не ушёл: честный отказ, а не падение хода
            self._log.warning(f"⚠️ [execute] {request.tool} {request.turn_id}: запрос не отправлен: {exc}")
            return _result(request, OwnerEvent(request.turn_id, "rejected", "error"))
        finally:
            self._waiting.pop(request.turn_id, None)
            if not future.done() and not survivor:
                self._requests.pop(request.turn_id, None)
        if event.status == "pending" and event.event_ref:
            self._requests[event.event_ref] = request
        self._requests.pop(request.turn_id, None)
        if self._stale_epoch(request, event):
            return _result(request, OwnerEvent(request.turn_id, "cancelled", "stale_epoch"))
        return _result(request, event)

    def _deliver(self, event: OwnerEvent) -> None:
        future = self._waiting.get(event.ref)
        if future is not None and not future.done():
            future.set_result(event)
            return
        request = self._requests.get(event.ref)
        if request is None or event.status not in _FINAL:
            self._stale(event)
            return
        self._requests.pop(event.ref, None)
        if not self._stale_epoch(request, event) and self._on_late is not None:
            self._on_late(_result(request, event))

    def _stale_epoch(self, request: ActionRequest, event: OwnerEvent) -> bool:
        """Исход запроса прошлой эпохи (перебили стопом, новая сессия): в ход не идёт, только строка лога."""
        current = None if self._epoch is None else self._epoch()
        if current is None or request.epoch >= current:
            return False
        self._log.warning(f"⚠️ [execute] stale_epoch {request.tool} {request.turn_id} epoch={request.epoch} "
                          f"now={current} status={event.status}")
        return True

    def _stale(self, event: OwnerEvent) -> None:
        self._log.warning(f"⚠️ [execute] stale_event ref={event.ref} status={event.status} reason={event.reason}")


def _result(request: ActionRequest, event: OwnerEvent) -> ActionResult:
    known = event.status in kn.ACTION_STATUSES
    return ActionResult(
        turn_id=request.turn_id, epoch=request.epoch, agent=request.agent, tool=request.tool, cls=request.cls,
        status=event.status if known else "rejected", reason=(event.reason if known else "bad_event") or None,
        data=MappingProxyType(dict(event.data)),
        event_ref=event.event_ref,
    )


__all__ = ["ActionRequest", "Executor", "OwnerEvent", "deadline_s"]
