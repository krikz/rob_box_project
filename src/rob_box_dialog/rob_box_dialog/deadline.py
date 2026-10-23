"""Потолок времени хода (ADR-0150 §8; решения В5 и В31; PR-8 #3702, #3960).

У хода один потолок ``knowledge.TURN_CEILING_S`` от начала (В31: пока провайдер жив, ход ведётся до конца, но не
дольше потолка). Каждое ожидание хода (вызов LLM, событие владельца) ограничено остатком: таймаут провайдера =
``min(provider_timeout, deadline_left)`` (:meth:`TurnDeadline.cap`). Истёк — ожидание отменяется
(:meth:`TurnDeadline.wait` бросает :class:`DeadlineExpired`), ход закрывается фразой кода, а в лог идёт строка
:data:`LATENCY_MARK` — кто и сколько ждал (``provider_latency``, §14 A14). Провайдер не жив (цепочка отказала) —
:class:`ProviderFailed` сразу, потолок не ждётся.
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


class ProviderFailed(Exception):
    """Вызов LLM упал (цепочка провайдеров исчерпана, транспорт оборвал вызов): провайдер не жив — ход не ждёт
    потолка (В31). ``provider`` — тот, к кому шёл вызов («» — порт его не знает)."""

    def __init__(self, provider: str, error: BaseException) -> None:
        super().__init__(f"{provider or '-'}: {type(error).__name__}: {error}")
        self.provider = provider
        self.error = error


def latency_line(provider: str, kind: str, latency_ms: int, status: str) -> str:
    """Строка лога ``provider_latency``: провайдер, вид (llm/stt/tts/event), мс, исход (ok/timeout/error)."""
    return f"{LATENCY_MARK} provider={provider or '-'} kind={kind} latency_ms={int(latency_ms)} status={status}"


class TurnDeadline:
    """Дедлайн одного хода по монотонным часам."""

    def __init__(self, budget_s: float = kn.TURN_CEILING_S, clock: Callable[[], float] = time.monotonic) -> None:
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

    def grant(self, own_s: float) -> None:
        """Действию владельца - его срок целиком, а не остаток хода: LLM (повтор после невалидной команды) съела бюджет,
        а музыка честно ещё стартует. Бюджет хода растёт до «сейчас + ``own_s``», не уменьшается."""
        self.budget_s = max(self.budget_s, self.elapsed() + float(own_s))

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


__all__ = ["DeadlineExpired", "LATENCY_MARK", "ProviderFailed", "TIMEOUT_PHRASE", "TurnDeadline", "latency_line"]
