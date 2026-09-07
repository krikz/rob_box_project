"""test_attach_consumes_parsed_command.py — DoD #2 of issue #1997.

Wires the bridge end-to-end on the in-process :class:`EventBus` and
asserts that publishing a parsed NAVIGATE / STOP command via
``CommandNode.build_parsed_envelope`` produces a
:class:`ReflexEvent` on ``/reflex/events`` with the right kind.

This is the *only* acceptance criterion that requires actually
running the asyncio consumer; the other two DoDs (envelope shape,
disabled=no-op) are covered by sibling test files in this directory.

Architecture context (ADR-0054 + SCHEDULER_DESIGN §8.10.4):

    STT → CommandParser → CommandNode.build_parsed_envelope
        → EventBus.publish(Envelope(topic='/command/parsed', ...))
        → ReflexLayer.attach() consumer
        → EventBus.publish(Envelope(topic='/reflex/events', ...))

The test stands up a real ``EventBus`` and ``ReflexLayer`` (no rclpy,
no thread, no scheduler daemon) — the bus is a tiny in-process
asyncio primitive that has no I/O surface to mock.
"""

from __future__ import annotations

import asyncio
from dataclasses import dataclass
from typing import Any, Dict, List

import pytest

from rob_box_voice.command_node import CommandNode
from rob_box_voice.scheduler import (
    EventBus,
    EventEnvelope,
    ReflexKind,
    ReflexLayer,
    TaskScheduler,
)

pytestmark = pytest.mark.asyncio


@dataclass
class _FakeIntent:
    value: str


@dataclass
class _FakeCommand:
    """Minimal command stand-in matching what
    ``CommandNode.build_parsed_envelope`` expects.

    The production code only reads ``.intent.value`` / ``.text`` /
    ``.entities`` / ``.confidence`` — see ADR-0054 wire contract.
    Using ``# type: ignore`` at the call site because the type
    annotation is the real ``Command`` dataclass but we duck-type it
    intentionally to keep the test independent of the parser.
    """

    intent_value: str
    text: str = ""
    entities: Dict[str, Any] | None = None
    confidence: float = 0.9

    @property
    def intent(self) -> _FakeIntent:
        return _FakeIntent(self.intent_value)


def _make_scheduler() -> TaskScheduler:
    """Build + start a scheduler inside the running event loop.

    Mirrors the helper in ``test_reflex_layer.py`` — the channel
    ``asyncio.Lock`` must be bound to a live loop at construction.
    """

    scheduler = TaskScheduler()
    scheduler.start()
    return scheduler


async def _wait_for_envelope(sub, timeout: float = 1.0) -> EventEnvelope:
    """Block until one envelope arrives on ``sub`` (or time out)."""

    return await asyncio.wait_for(sub.get(), timeout=timeout)


async def test_attach_consumes_navigate_command_and_emits_reflex_event() -> None:
    """The happy path: NAVIGATE → ReflexEvent(kind=MOVE_DIRECTION) on /reflex/events."""

    bus = EventBus()
    scheduler = _make_scheduler()
    layer = ReflexLayer(scheduler, bus)
    layer.attach(bus)
    try:
        # Subscribe to the output topic BEFORE publishing, otherwise the
        # publish may complete before the subscriber attaches and the
        # event is dropped (EventBus has no replay buffer).
        out_sub = bus.subscribe(ReflexLayer.TOPIC, max_queue_size=8)

        envelope = CommandNode.build_parsed_envelope(  # type: ignore[arg-type]
            _FakeCommand(
                intent_value="navigate",
                text="поехали на кухню",
                entities={"destination": "kitchen"},
                confidence=0.92,
            )
        )
        await bus.publish(envelope)

        out_envelope = await _wait_for_envelope(out_sub)
    finally:
        layer.detach()
        await layer.aclose()
        await bus.close()

    assert isinstance(out_envelope, EventEnvelope)
    assert out_envelope.topic == ReflexLayer.TOPIC, (
        f"expected topic={ReflexLayer.TOPIC!r}, got {out_envelope.topic!r}"
    )
    payload = out_envelope.payload
    assert payload["kind"] == ReflexKind.MOVE_DIRECTION.value, (
        f"expected kind={ReflexKind.MOVE_DIRECTION.value!r}, "
        f"got {payload['kind']!r}"
    )
    assert payload["source"] == "reflex"
    assert payload["text"] == "поехали на кухню"
    assert payload["entities"] == {"destination": "kitchen"}


async def test_attach_consumes_stop_command_and_emits_reflex_event() -> None:
    """STOP → ReflexEvent(kind=STOP). Side effects on the scheduler
    (cancel-all) are covered by ``test_reflex_layer.py``; here we
    only care about the published event on the bus (DoD #2)."""

    bus = EventBus()
    scheduler = _make_scheduler()
    layer = ReflexLayer(scheduler, bus)
    layer.attach(bus)
    try:
        out_sub = bus.subscribe(ReflexLayer.TOPIC, max_queue_size=8)
        envelope = CommandNode.build_parsed_envelope(  # type: ignore[arg-type]
            _FakeCommand(intent_value="stop", text="стоп", confidence=0.99)
        )
        await bus.publish(envelope)

        out_envelope = await _wait_for_envelope(out_sub)
    finally:
        layer.detach()
        await layer.aclose()
        await bus.close()

    payload = out_envelope.payload
    assert payload["kind"] == ReflexKind.STOP.value


async def test_attach_emits_event_with_correct_priority_and_source() -> None:
    """The published envelope must carry ``source='reflex'`` plus a
    sensible priority. STATUS is non-reflex per the classification
    matrix (§8.10.3) so its priority must be normal-or-low."""

    bus = EventBus()
    scheduler = _make_scheduler()
    layer = ReflexLayer(scheduler, bus)
    layer.attach(bus)
    try:
        out_sub = bus.subscribe(ReflexLayer.TOPIC, max_queue_size=8)
        await bus.publish(
            CommandNode.build_parsed_envelope(  # type: ignore[arg-type]
                _FakeCommand(intent_value="status", text="статус")
            )
        )
        envelope = await _wait_for_envelope(out_sub)
    finally:
        layer.detach()
        await layer.aclose()
        await bus.close()

    payload = envelope.payload
    assert payload["kind"] == ReflexKind.STATUS.value
    assert payload["source"] == "reflex"
    assert "priority" in payload
    assert payload["priority"] in {"normal", "low"}, (
        f"unexpected priority for STATUS reflex: {payload['priority']!r}"
    )


async def test_attach_round_trips_intent_text_entities_confidence() -> None:
    """The ReflexEvent payload must carry the original text/entities/
    confidence so downstream consumers (LLM feedback, monitoring)
    see what the user actually said — not just the kind.

    Regression guard: a previous refactor dropped ``text`` from the
    payload and the LLM feedback loop lost context. (See ADR-0054
    §\"wire contract\".)
    """

    bus = EventBus()
    scheduler = _make_scheduler()
    layer = ReflexLayer(scheduler, bus)
    layer.attach(bus)
    try:
        out_sub = bus.subscribe(ReflexLayer.TOPIC, max_queue_size=8)
        await bus.publish(
            CommandNode.build_parsed_envelope(  # type: ignore[arg-type]
                _FakeCommand(
                    intent_value="follow",
                    text="иди за мной",
                    entities={"speaker": "owner"},
                    confidence=0.81,
                )
            )
        )
        envelope = await _wait_for_envelope(out_sub)
    finally:
        layer.detach()
        await layer.aclose()
        await bus.close()

    payload = envelope.payload
    assert payload["text"] == "иди за мной"
    assert payload["entities"] == {"speaker": "owner"}
    assert abs(payload["confidence"] - 0.81) < 1e-6
