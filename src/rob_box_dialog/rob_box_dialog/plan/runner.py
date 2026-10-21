"""Допуск действия по плану (ADR-0150 §5.6.4): :func:`compose` против занятых ресурсов → идти, ждать или отказ.

Один исполнитель плана на оба пути хода v1 (свободный цикл тулов и режим команд): хост зовёт :meth:`PlanRunner.admit`
перед каждым действием. Ожидание — до события конца занятого блока от владельца (хост кормит
:attr:`PlanRunner.state` событиями и зовёт :meth:`PlanRunner.changed`), не дольше потолка
``knowledge.plan_wait_ceiling_s``; не дождался — честный ``timeout``, действие не исполняется. Очередь, которую ведёт
сам владелец (``knowledge.OWNER_QUEUES``: реплика за репликой у tts_node, звук за звуком у sound_node), здесь не
ждётся — запрос уходит владельцу сразу.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import TYPE_CHECKING, Callable, Optional, Sequence

from rob_box_dialog import knowledge as kn
from rob_box_dialog.plan.composition import Decision, compose, kind_of
from rob_box_dialog.plan.plan_events import PlanEvent, queued, rejected
from rob_box_dialog.plan.resources import ResourceState
from rob_box_dialog.words import policy_word

if TYPE_CHECKING:  # asyncio тянет socket: импорт пакета чистый (test_purity), цикл — при исполнении
    import asyncio

_LOG = logging.getLogger(__name__)
LOG_MARK = "🗓️ [plan]"


@dataclass(frozen=True)
class Admission:
    """Итог допуска: ``go`` — исполнять; ``rejected`` — клетка REJECT; ``timeout`` — занятое не освободилось к
    потолку. ``event`` — ``queued``/``rejected`` плана (если был), ``waited_s`` — сколько ждали."""

    decision: Decision
    status: str
    event: Optional[PlanEvent] = None
    waited_s: float = 0.0


class PlanRunner:
    """Планировщик ресурсов одного агента: снимок владельцев, Tier-1 хода, допуск действий."""

    def __init__(self, state: Optional[ResourceState] = None, publish: Optional[Callable[[PlanEvent], None]] = None,
                 log: Optional[Callable[[str], None]] = None, agent: str = kn.AGENTS[0]) -> None:
        self.state = state or ResourceState()
        self._publish = publish
        self._log = log or _LOG.info
        self._agent = agent
        self._hint: Optional[str] = None
        self._loop: Optional["asyncio.AbstractEventLoop"] = None
        self._event: Optional["asyncio.Event"] = None

    @property
    def hint(self) -> Optional[str]:
        """Tier-1 переопределение политики хода (ключ ``TIER1_POLICY``) или ``None``."""
        return self._hint

    def begin_turn(self, utterance: str = "") -> None:
        """Граница хода: Tier-1 по словам человека этого хода («а потом …» → APPEND)."""
        self._hint = policy_word(utterance) if utterance else None

    def changed(self) -> None:
        """Снимок владельцев обновился (из любого потока): разбудить ожидающих."""
        loop, event = self._loop, self._event
        if loop is None or event is None or loop.is_closed():
            return
        loop.call_soon_threadsafe(event.set)

    def emit(self, event: PlanEvent) -> None:
        """Опубликовать событие плана и записать строку лога (её ищет живая проверка)."""
        self._log(f"{LOG_MARK} PlanEvent {event.kind} block={event.block_id[:12]} after={event.after_resource or '-'}"
                  f"/{event.after_block[:8] or '-'} eta_ms={event.eta_ms} reason={event.reason or '-'}")
        if self._publish is not None:
            try:
                self._publish(event)
            except Exception as exc:  # noqa: BLE001 — публикация наблюдателю не ломает действие
                _LOG.warning(f"{LOG_MARK} PlanEvent не опубликован: {exc}")

    def decide(self, tool: str, when: str = "after_current") -> Decision:
        """Решение по таблице для тула против занятого сейчас (без ожидания)."""
        return compose(kind_of(tool), self.state.busy(), self._hint, when)

    async def admit(self, tool: str, block_id: str, *, turn_id: str = "", epoch: int = 0,
                    when: str = "after_current") -> Admission:
        """Допуск действия ``tool``: отказ по таблице, ожидание конца занятого (с потолком) или «иди сразу»."""
        import asyncio  # noqa: PLC0415

        kind = kind_of(tool)
        decision = self.decide(tool, when)
        ids = {"turn_id": turn_id, "epoch": epoch, "agent": self._agent}
        if decision.policy == "REJECT":
            event = rejected(block_id, decision, **ids)
            self.emit(event)
            return Admission(decision, "rejected", event)
        waits = Decision(kind, tuple(c for c in decision.cells if c.waits and (kind, c.busy) not in kn.OWNER_QUEUES))
        event = queued(block_id, waits, self.state, **ids)
        if event is None:
            return Admission(decision, "go")
        self.emit(event)
        loop = asyncio.get_running_loop()
        started = loop.time()
        ceiling = kn.plan_wait_ceiling_s(event.eta_ms / 1000 if event.eta_ms else None)
        free = await self._wait_free(waits.waits_for, started + ceiling)
        waited = loop.time() - started
        if not free:
            self._log(f"{LOG_MARK} {tool} {block_id[:12]}: {event.after_resource} не освободился за {waited:.1f} с"
                      " — не исполняю (timeout)")
            return Admission(decision, "timeout", event, waited)
        self._log(f"{LOG_MARK} {tool} {block_id[:12]}: дождался {event.after_resource} за {waited:.1f} с")
        return Admission(decision, "go", event, waited)

    async def _wait_free(self, resources: Sequence[str], deadline: float) -> bool:
        import asyncio  # noqa: PLC0415

        loop = asyncio.get_running_loop()
        if self._loop is not loop or self._event is None:
            self._loop, self._event = loop, asyncio.Event()
        while any(r in self.state.busy() for r in resources):
            left = deadline - loop.time()
            if left <= 0:
                return False
            self._event.clear()
            try:
                await asyncio.wait_for(self._event.wait(), left)
            except asyncio.TimeoutError:
                return not any(r in self.state.busy() for r in resources)
        return True


__all__ = ["Admission", "LOG_MARK", "PlanRunner"]
