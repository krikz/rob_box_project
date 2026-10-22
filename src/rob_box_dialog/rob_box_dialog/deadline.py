"""Бюджет времени хода (ADR-0150 §8, решение В5; PR-8 #3702).

У хода один дедлайн ``knowledge.TURN_DEADLINE_S`` от начала. Каждое ожидание хода (вызов LLM, событие владельца)
ограничено остатком: таймаут провайдера = ``min(provider_timeout, deadline_left)`` (:meth:`TurnDeadline.cap`).
Истёк — ожидание отменяется (:meth:`TurnDeadline.wait` бросает :class:`DeadlineExpired`), ход закрывается фразой
``understand.timeout``, а в лог идёт строка :data:`LATENCY_MARK` — кто и сколько ждал (``provider_latency``, §14 A14).
"""

from __future__ import annotations

import time
from typing import Any, Awaitable, Callable, Optional, TypeVar

from rob_box_dialog import knowledge as kn

#: Строка лога задержки провайдера на ходе (читает ``scripts/dialog``).
LATENCY_MARK = "⏱ [provider_latency]"
#: Ключ фразы, которой закрывается ход с истёкшим дедлайном.
TIMEOUT_PHRASE = "understand.timeout"

T = TypeVar("T")


class DeadlineExpired(Exception):
    """Бюджет хода кончился во время ожидания ``what`` (провайдер/событие)."""

    def __init__(self, what: str, waited_s: float) -> None:
        super().__init__(f"{what}: дедлайн хода истёк через {waited_s:.1f} с")
        self.what = what
        self.waited_s = waited_s


def latency_line(provider: str, kind: str, latency_ms: int, status: str) -> str:
    """Строка лога ``provider_latency``: провайдер, вид (llm/stt/tts/event), мс, исход (ok/timeout/error)."""
    return f"{LATENCY_MARK} provider={provider or '-'} kind={kind} latency_ms={int(latency_ms)} status={status}"


class TurnDeadline:
    """Дедлайн одного хода по монотонным часам."""

    def __init__(self, budget_s: float = kn.TURN_DEADLINE_S, clock: Callable[[], float] = time.monotonic) -> None:
        self._clock = clock
        self.started = clock()
        self.budget_s = float(budget_s)

    def elapsed(self) -> float:
        return max(0.0, self._clock() - self.started)

    def left(self) -> float:
        """Сколько осталось (не меньше нуля)."""
        return max(0.0, self.budget_s - self.elapsed())

    def expired(self) -> bool:
        return self.left() <= 0.0

    def cap(self, provider_timeout_s: Optional[float] = None) -> float:
        """Таймаут ожидания: ``min(provider_timeout, deadline_left)``; у провайдера нет своего — остаток хода."""
        left = self.left()
        return left if provider_timeout_s is None else max(0.0, min(float(provider_timeout_s), left))

    async def wait(self, awaitable: Awaitable[T], what: str, provider_timeout_s: Optional[float] = None) -> T:
        """Дождаться ``awaitable`` не дольше :meth:`cap`; не успели — отмена и :class:`DeadlineExpired` (если
        кончился бюджет хода) или ``asyncio.TimeoutError`` (если раньше кончился таймаут провайдера)."""
        import asyncio  # noqa: PLC0415 — asyncio тянет socket: пакет при импорте чистый (test_purity)

        left = self.left()
        limit = self.cap(provider_timeout_s)
        try:
            return await asyncio.wait_for(_as_coro(awaitable), timeout=limit)
        except asyncio.TimeoutError:
            # Чей предел сработал, решает то, какой был меньше на входе, а не часы после (дрожание таймера ±мс).
            if provider_timeout_s is None or float(provider_timeout_s) >= left:
                raise DeadlineExpired(what, self.elapsed()) from None
            raise


async def _as_coro(awaitable: Awaitable[Any]) -> Any:
    return await awaitable


__all__ = ["DeadlineExpired", "LATENCY_MARK", "TIMEOUT_PHRASE", "TurnDeadline", "latency_line"]
