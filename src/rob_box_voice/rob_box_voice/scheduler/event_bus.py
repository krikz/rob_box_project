"""Bounded in-process event bus for scheduler component coordination."""
from __future__ import annotations

import asyncio
import fnmatch
import time
import uuid
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, AsyncIterator


class BackpressurePolicy(str, Enum):
    """Action to take when a subscriber queue reaches its bound."""

    BLOCK = "block"
    DROP_OLDEST = "drop_oldest"
    RAISE = "raise"


class EventBusError(RuntimeError):
    """Base error for EventBus operations."""


class EventBusClosedError(EventBusError):
    """The bus or subscription is closed."""


class EventQueueFullError(EventBusError):
    """A RAISE-policy subscriber cannot accept another event."""


@dataclass(frozen=True)
class EventEnvelope:
    """Immutable message exchanged by scheduler components."""

    topic: str
    payload: Any
    event_id: str = field(default_factory=lambda: uuid.uuid4().hex)
    created_at: float = field(default_factory=time.monotonic)
    correlation_id: str | None = None

    def __post_init__(self) -> None:
        if not self.topic:
            raise ValueError("event topic must not be empty")


_CLOSE = object()


class EventSubscription(AsyncIterator[EventEnvelope]):
    """A bounded, independently consumable event stream."""

    def __init__(
        self,
        bus: "EventBus",
        topic_pattern: str,
        max_queue_size: int,
        policy: BackpressurePolicy,
    ) -> None:
        self._bus = bus
        self.topic_pattern = topic_pattern
        self.policy = policy
        self._queue: asyncio.Queue[EventEnvelope | object] = asyncio.Queue(max_queue_size)
        self._closed = False
        self.dropped_count = 0

    def matches(self, topic: str) -> bool:
        return fnmatch.fnmatchcase(topic, self.topic_pattern)

    async def get(self) -> EventEnvelope:
        item = await self._queue.get()
        if item is _CLOSE:
            self._queue.task_done()
            raise EventBusClosedError("subscription is closed")
        self._queue.task_done()
        return item  # type: ignore[return-value]

    async def __anext__(self) -> EventEnvelope:
        try:
            return await self.get()
        except EventBusClosedError as exc:
            raise StopAsyncIteration from exc

    def __aiter__(self) -> "EventSubscription":
        return self

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self._bus._unsubscribe(self)
        self._signal_closed()

    def _signal_closed(self) -> None:
        while self._queue.full():
            try:
                self._queue.get_nowait()
                self._queue.task_done()
            except asyncio.QueueEmpty:
                break
        try:
            self._queue.put_nowait(_CLOSE)
        except asyncio.QueueFull:  # defensive; queue was just drained
            pass

    async def _deliver(self, event: EventEnvelope) -> None:
        if self._closed:
            return
        if self.policy is BackpressurePolicy.BLOCK:
            await self._queue.put(event)
            return
        if self.policy is BackpressurePolicy.RAISE:
            try:
                self._queue.put_nowait(event)
            except asyncio.QueueFull as exc:
                raise EventQueueFullError(
                    f"subscriber queue full for {self.topic_pattern!r}"
                ) from exc
            return
        if self._queue.full():
            self._queue.get_nowait()
            self._queue.task_done()
            self.dropped_count += 1
        self._queue.put_nowait(event)


class EventBus:
    """Async publish/subscribe bus with explicit bounded backpressure."""

    def __init__(self) -> None:
        self._subscriptions: set[EventSubscription] = set()
        self._closed = False
        self._publish_lock = asyncio.Lock()

    def subscribe(
        self,
        topic_pattern: str,
        *,
        max_queue_size: int = 64,
        policy: BackpressurePolicy = BackpressurePolicy.BLOCK,
    ) -> EventSubscription:
        if self._closed:
            raise EventBusClosedError("event bus is closed")
        if not topic_pattern:
            raise ValueError("topic must not be empty")
        if max_queue_size <= 0:
            raise ValueError("max_queue_size must be greater than zero")
        subscription = EventSubscription(self, topic_pattern, max_queue_size, policy)
        self._subscriptions.add(subscription)
        return subscription

    async def publish(self, event: EventEnvelope) -> int:
        if self._closed:
            raise EventBusClosedError("event bus is closed")
        async with self._publish_lock:
            matching = [s for s in tuple(self._subscriptions) if s.matches(event.topic)]
            # Validate strict subscribers before delivering to avoid partial fan-out.
            for subscription in matching:
                if (
                    subscription.policy is BackpressurePolicy.RAISE
                    and subscription._queue.full()
                ):
                    raise EventQueueFullError(
                        f"subscriber queue full for {subscription.topic_pattern!r}"
                    )
            await asyncio.gather(*(s._deliver(event) for s in matching))
            return len(matching)

    async def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        for subscription in tuple(self._subscriptions):
            subscription._closed = True
            subscription._signal_closed()
        self._subscriptions.clear()

    def _unsubscribe(self, subscription: EventSubscription) -> None:
        self._subscriptions.discard(subscription)
