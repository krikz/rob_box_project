"""Phase 2 unit tests: bounded publish/subscribe EventBus."""
from __future__ import annotations

import asyncio

import pytest

from rob_box_voice.scheduler import (
    BackpressurePolicy,
    EventBus,
    EventBusClosedError,
    EventEnvelope,
    EventQueueFullError,
)


@pytest.mark.asyncio
async def test_publish_delivers_ordered_events_to_matching_subscribers():
    bus = EventBus()
    voice = bus.subscribe("task.*", max_queue_size=2)
    all_events = bus.subscribe("*", max_queue_size=2)

    first = EventEnvelope(topic="task.started", payload={"id": "1"})
    second = EventEnvelope(topic="task.completed", payload={"id": "1"})
    await bus.publish(first)
    await bus.publish(second)

    assert await voice.get() == first
    assert await voice.get() == second
    assert await all_events.get() == first
    assert await all_events.get() == second
    voice.close()
    all_events.close()
    await bus.close()


@pytest.mark.asyncio
async def test_block_policy_applies_backpressure_until_consumer_reads():
    bus = EventBus()
    subscription = bus.subscribe("task.*", max_queue_size=1)
    await bus.publish(EventEnvelope(topic="task.started", payload=1))

    blocked_publish = asyncio.create_task(
        bus.publish(EventEnvelope(topic="task.completed", payload=2))
    )
    await asyncio.sleep(0)
    assert not blocked_publish.done()

    assert (await subscription.get()).payload == 1
    await asyncio.wait_for(blocked_publish, timeout=0.1)
    assert (await subscription.get()).payload == 2
    subscription.close()
    await bus.close()


@pytest.mark.asyncio
async def test_drop_oldest_policy_keeps_latest_event_and_counts_drop():
    bus = EventBus()
    subscription = bus.subscribe(
        "task.*", max_queue_size=1, policy=BackpressurePolicy.DROP_OLDEST
    )
    await bus.publish(EventEnvelope(topic="task.started", payload=1))
    await bus.publish(EventEnvelope(topic="task.completed", payload=2))

    assert (await subscription.get()).payload == 2
    assert subscription.dropped_count == 1
    subscription.close()
    await bus.close()


@pytest.mark.asyncio
async def test_raise_policy_reports_slow_subscriber_without_partial_delivery():
    bus = EventBus()
    subscription = bus.subscribe(
        "task.*", max_queue_size=1, policy=BackpressurePolicy.RAISE
    )
    await bus.publish(EventEnvelope(topic="task.started", payload=1))

    with pytest.raises(EventQueueFullError):
        await bus.publish(EventEnvelope(topic="task.completed", payload=2))

    assert (await subscription.get()).payload == 1
    subscription.close()
    await bus.close()


@pytest.mark.asyncio
async def test_close_unblocks_pending_get_and_rejects_publish():
    bus = EventBus()
    subscription = bus.subscribe("*", max_queue_size=1)
    pending_get = asyncio.create_task(subscription.get())
    await asyncio.sleep(0)

    await bus.close()

    with pytest.raises(EventBusClosedError):
        await pending_get
    with pytest.raises(EventBusClosedError):
        await bus.publish(EventEnvelope(topic="x", payload=None))


def test_subscribe_rejects_unbounded_or_empty_topic():
    bus = EventBus()
    with pytest.raises(ValueError, match="max_queue_size"):
        bus.subscribe("*", max_queue_size=0)
    with pytest.raises(ValueError, match="topic"):
        bus.subscribe("", max_queue_size=1)
