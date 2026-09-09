"""Phase 2.5 unit tests: ReflexLayer routing into EventBus + TaskScheduler.

Covers the acceptance criteria from ``docs/design/CHILD_TASKS_PROPOSAL.md``
(card #5 — Phase 2.5 Reflex layer) and ``SCHEDULER_DESIGN.md`` §8.10 /
§11.2 / §11.3. The tests run against the in-process
:class:`TaskScheduler` + :class:`EventBus` (no ROS, no LLM, no audio).

Test convention: every test builds its own scheduler with
``_make_scheduler()`` inside the running event loop, mirroring the
pattern from ``test_task_scheduler.py`` (the channel ``asyncio.Lock``
must be bound to a live loop at construction time).
"""
from __future__ import annotations

import asyncio
import time
from dataclasses import dataclass, field
from typing import Any, Dict

import pytest

from rob_box_voice.scheduler import (
    ChannelKind,
    EventBus,
    EventEnvelope,
    SchedulerTask,
    TaskResult,
    TaskScheduler,
    TaskStatus,
)
from rob_box_voice.scheduler.reflex import (
    DEFAULT_DEBOUNCE_MS,
    DEFAULT_HISTORY_SIZE,
    ReflexEvent,
    ReflexKind,
    ReflexLayer,
    ReflexMetrics,
    ReflexPriority,
    command_to_view,
)


# ---------------------------------------------------------------------------
# Fixtures + helpers
# ---------------------------------------------------------------------------


@dataclass
class FakeCommand:
    """Duck-typed stand-in for ``command_parser.Command``.

    The reflex layer only reads ``.intent.value``, ``.text``,
    ``.entities`` and ``.confidence`` — :func:`command_to_view`
    does not care that this isn't the real parser dataclass.
    """

    intent_value: str
    text: str
    entities: Dict[str, Any] = field(default_factory=dict)
    confidence: float = 0.9

    @property
    def intent(self) -> "_FakeIntent":
        return _FakeIntent(self.intent_value)


@dataclass
class _FakeIntent:
    value: str


def stop_cmd(text: str = "стоп") -> FakeCommand:
    return FakeCommand("stop", text)


def direction_cmd(direction: str, text: str = "направо") -> FakeCommand:
    return FakeCommand("navigate", text, entities={"direction": direction})


def waypoint_cmd(waypoint: str, text: str = "кухня") -> FakeCommand:
    return FakeCommand(
        "navigate", text, entities={"waypoint": waypoint}
    )


def follow_cmd(text: str = "следуй за мной") -> FakeCommand:
    return FakeCommand("follow", text)


def status_cmd(text: str = "где ты") -> FakeCommand:
    return FakeCommand("status", text)


def map_cmd(text: str = "покажи карту") -> FakeCommand:
    return FakeCommand("map", text)


def vision_cmd(text: str = "что видишь") -> FakeCommand:
    return FakeCommand("vision", text)


def unknown_cmd(text: str = "привет") -> FakeCommand:
    return FakeCommand("unknown", text)


@pytest.fixture
def event_bus() -> EventBus:
    return EventBus()


def _make_scheduler(
    *, channels: tuple[ChannelKind, ...] = TaskScheduler.DEFAULT_CHANNELS,
) -> TaskScheduler:
    """Build + start a scheduler inside the running event loop.

    Mirrors the helper from ``test_task_scheduler.py`` — the
    scheduler must be constructed inside a running loop so the
    per-channel ``asyncio.Lock`` is bound to the right loop.
    """

    sched = TaskScheduler(channels=channels)
    sched.start()
    return sched


async def _noop_exec(task: SchedulerTask) -> TaskResult:
    return TaskResult(payload={"task_id": task.task_id})


async def _collect(subscription, n: int) -> list[EventEnvelope]:
    out: list[EventEnvelope] = []
    for _ in range(n):
        out.append(await subscription.get())
    return out


# ---------------------------------------------------------------------------
# Classification
# ---------------------------------------------------------------------------


def test_command_to_view_extracts_fields():
    cmd = FakeCommand("stop", "стоп", confidence=0.85)
    view = command_to_view(cmd)
    assert view.intent == "stop"
    assert view.text == "стоп"
    assert view.confidence == 0.85
    assert view.entities == {}


def test_command_to_view_rejects_missing_intent():
    class Broken:
        text = "x"
        entities: Dict[str, Any] = {}
        confidence = 0.0

    with pytest.raises(TypeError):
        command_to_view(Broken())


def test_command_to_view_rejects_non_mapping_entities():
    class Broken:
        intent_value = "stop"
        text = "x"
        entities = [1, 2, 3]
        confidence = 0.0

        @property
        def intent(self):
            return _FakeIntent(self.intent_value)

    with pytest.raises(TypeError):
        command_to_view(Broken())


@pytest.mark.parametrize(
    "cmd, expected_kind, expected_priority",
    [
        (stop_cmd(), ReflexKind.STOP, ReflexPriority.CRITICAL),
        (direction_cmd("right"), ReflexKind.MOVE_DIRECTION, ReflexPriority.HIGH),
        (waypoint_cmd("кухня"), ReflexKind.MOVE_DIRECTION, ReflexPriority.NORMAL),
        (follow_cmd(), ReflexKind.FOLLOW, ReflexPriority.NORMAL),
        (status_cmd(), ReflexKind.STATUS, ReflexPriority.NORMAL),
        (map_cmd(), ReflexKind.MAP, ReflexPriority.NORMAL),
        (vision_cmd(), ReflexKind.VISION, ReflexPriority.NORMAL),
        (unknown_cmd(), ReflexKind.UNKNOWN, ReflexPriority.NORMAL),
    ],
)
def test_classification_matrix(cmd, expected_kind, expected_priority):
    """Pure-Python classification check — no asyncio needed."""

    view = command_to_view(cmd)
    kind, priority = ReflexLayer._classify(view)
    assert kind is expected_kind
    assert priority is expected_priority


# ---------------------------------------------------------------------------
# EventBus integration
# ---------------------------------------------------------------------------


@pytest.mark.asyncio
async def test_reflex_event_is_published_on_topic_with_payload(event_bus):
    sched = _make_scheduler()
    try:
        sub = event_bus.subscribe(ReflexLayer.TOPIC)
        layer = ReflexLayer(sched, event_bus)

        decision = await layer.handle(stop_cmd("хватит"))

        env = await sub.get()
        assert env.topic == ReflexLayer.TOPIC
        assert env.correlation_id == decision.event.event_id
        payload = env.payload
        assert payload["kind"] == "stop"
        assert payload["priority"] == "critical"
        assert payload["source"] == "reflex"
        assert payload["text"] == "хватит"
        assert payload["confidence"] == pytest.approx(0.9)
    finally:
        sched.shutdown()


@pytest.mark.asyncio
async def test_priority_is_propagated_into_envelope(event_bus):
    sched = _make_scheduler()
    try:
        sub = event_bus.subscribe(ReflexLayer.TOPIC)
        layer = ReflexLayer(sched, event_bus)

        await layer.handle(direction_cmd("left"))
        env = await sub.get()
        assert env.payload["priority"] == "high"
    finally:
        sched.shutdown()


@pytest.mark.asyncio
async def test_correlation_id_provider_is_used_when_supplied(event_bus):
    sched = _make_scheduler()
    try:
        sub = event_bus.subscribe(ReflexLayer.TOPIC)

        calls: list[ReflexEvent] = []

        def provider(ev: ReflexEvent) -> str:
            calls.append(ev)
            return f"corr-{ev.kind.value}"

        layer = ReflexLayer(sched, event_bus, correlation_id_provider=provider)
        await layer.handle(stop_cmd())

        env = await sub.get()
        assert env.correlation_id == "corr-stop"
        assert calls and calls[0].kind is ReflexKind.STOP
    finally:
        sched.shutdown()


@pytest.mark.asyncio
async def test_correlation_id_provider_exception_is_swallowed(event_bus):
    sched = _make_scheduler()
    try:
        sub = event_bus.subscribe(ReflexLayer.TOPIC)

        def boom(ev: ReflexEvent) -> str:
            raise RuntimeError("provider down")

        layer = ReflexLayer(sched, event_bus, correlation_id_provider=boom)
        decision = await layer.handle(stop_cmd())

        env = await sub.get()
        # Falls back to the per-event uuid so downstream tracing still works.
        assert env.correlation_id == decision.event.event_id
    finally:
        sched.shutdown()


@pytest.mark.asyncio
async def test_publish_only_emits_envelope_without_side_effects(event_bus):
    sched = _make_scheduler()
    try:
        sub = event_bus.subscribe(ReflexLayer.TOPIC)
        layer = ReflexLayer(sched, event_bus)

        event = ReflexLayer.make_event(
            kind=ReflexKind.STATUS, text="где ты", priority=ReflexPriority.NORMAL
        )
        ok = await layer.publish_only(event)
        assert ok is True

        env = await sub.get()
        assert env.payload["kind"] == "status"
        assert env.payload["text"] == "где ты"
    finally:
        sched.shutdown()


@pytest.mark.asyncio
async def test_publish_only_returns_false_when_bus_closes(event_bus):
    sched = _make_scheduler()
    try:
        layer = ReflexLayer(sched, event_bus)
        await event_bus.close()

        event = ReflexLayer.make_event(kind=ReflexKind.STATUS)
        ok = await layer.publish_only(event)
        assert ok is False
    finally:
        sched.shutdown()


# ---------------------------------------------------------------------------
# STOP behaviour
# ---------------------------------------------------------------------------


@pytest.mark.asyncio
async def test_stop_cancels_all_active_tasks_sync(event_bus):
    """§8.10.6 scenario 1: «спой песню» → «стой!» — задачи в очереди → CANCELLED.

    Phase 1 MVP semantics: ``TaskScheduler.cancel`` removes tasks
    from the channel queue but does NOT preempt a running executor
    coroutine — that lands in Phase 2 with the EventBus-driven
    preemption. We submit *more* tasks than the channel can run in
    parallel so at least one task is still in the queue when STOP
    arrives; that queued task MUST flip to CANCELLED. The running
    task is left running (documented MVP behaviour).
    """

    sched = _make_scheduler()
    try:
        async def _exec(task: SchedulerTask) -> TaskResult:
            try:
                await asyncio.sleep(5.0)
            finally:
                pass
            return TaskResult(payload={"i": task.task_id})

        # Submit three tasks on the same FIFO channel so the channel
        # can only run one at a time. Two stay QUEUED, one is RUNNING.
        voice_tasks = []
        for i in range(3):
            t = SchedulerTask(
                task_id=f"v-{i}",
                tool="speak_text",
                channel=ChannelKind.VOICE,
                executor=_exec,
            )
            sched.submit(t)
            voice_tasks.append(t)

        # Let the pump SCHEDULE + start the first task.
        await asyncio.sleep(0.05)
        running = [t for t in voice_tasks if t.status is TaskStatus.RUNNING]
        queued = [t for t in voice_tasks if t.status is TaskStatus.QUEUED]
        assert running, "expected at least one task to be RUNNING"
        assert queued, "expected at least one task to be QUEUED"

        layer = ReflexLayer(sched, event_bus)
        started = time.monotonic()
        decision = await layer.handle(stop_cmd())
        elapsed = time.monotonic() - started

        # All queued tasks MUST be cancelled by the reflex layer.
        assert decision.cancelled == len(queued)
        for t in queued:
            assert t.status is TaskStatus.CANCELLED
        # Running task is untouched (Phase 2 preemption not implemented).
        for t in running:
            assert t.status is TaskStatus.RUNNING
        # Latency: cancel_all is local + asyncio.Lock, well under the §8.10
        # 500 ms budget.
        assert elapsed < 0.5, f"cancel took {elapsed * 1000:.1f} ms"
    finally:
        sched.shutdown()


@pytest.mark.asyncio
async def test_stop_skips_terminal_tasks(event_bus):
    """Already-completed tasks must NOT be touched by reflex cancel."""

    sched = _make_scheduler()
    try:
        async def _exec(task: SchedulerTask) -> TaskResult:
            return TaskResult(payload={"ok": True})

        finished = sched.submit(SchedulerTask(
            task_id="done", tool="speak_text", channel=ChannelKind.VOICE, executor=_exec,
        ))
        await sched.wait_all()
        assert finished.status is TaskStatus.COMPLETED

        layer = ReflexLayer(sched, event_bus)
        decision = await layer.handle(stop_cmd())

        assert decision.cancelled == 0
        assert finished.status is TaskStatus.COMPLETED
    finally:
        sched.shutdown()


@pytest.mark.asyncio
async def test_stop_with_no_active_tasks_returns_zero(event_bus):
    sched = _make_scheduler()
    try:
        layer = ReflexLayer(sched, event_bus)
        decision = await layer.handle(stop_cmd())
        assert decision.cancelled == 0
        assert decision.published is True
    finally:
        sched.shutdown()


# ---------------------------------------------------------------------------
# Debounce (scenario 4)
# ---------------------------------------------------------------------------


@pytest.mark.asyncio
async def test_repeated_stop_within_window_is_debounced(event_bus):
    sched = _make_scheduler()
    try:
        sub = event_bus.subscribe(ReflexLayer.TOPIC)
        layer = ReflexLayer(sched, event_bus)

        d1 = await layer.handle(stop_cmd("стоп"))
        d2 = await layer.handle(stop_cmd("стоп"))
        d3 = await layer.handle(stop_cmd("стоп"))

        assert d1.published is True and d1.reason == "routed"
        assert d2.published is False and d2.reason == "debounced"
        assert d3.published is False and d3.reason == "debounced"

        # Only the first envelope reached the bus.
        env = await sub.get()
        with pytest.raises(asyncio.TimeoutError):
            await asyncio.wait_for(sub.get(), timeout=0.05)
        assert env.payload["text"] == "стоп"
    finally:
        sched.shutdown()


@pytest.mark.asyncio
async def test_debounce_can_be_disabled(event_bus):
    sched = _make_scheduler()
    try:
        sub = event_bus.subscribe(ReflexLayer.TOPIC)
        layer = ReflexLayer(sched, event_bus, debounce_window_ms=0)

        await layer.handle(stop_cmd())
        await layer.handle(stop_cmd())

        envs = await _collect(sub, 2)
        assert [e.payload["text"] for e in envs] == ["стоп", "стоп"]
    finally:
        sched.shutdown()


@pytest.mark.asyncio
async def test_non_stop_kinds_do_not_reset_debounce_window(event_bus):
    sched = _make_scheduler()
    try:
        layer = ReflexLayer(sched, event_bus)

        await layer.handle(stop_cmd())
        # Direction event in between does NOT touch the STOP timestamp.
        d_dir = await layer.handle(direction_cmd("left"))
        assert d_dir.published is True
        d_stop = await layer.handle(stop_cmd())
        assert d_stop.published is False  # still inside the 500 ms window
    finally:
        sched.shutdown()


@pytest.mark.asyncio
async def test_debounce_window_expires(event_bus):
    sched = _make_scheduler()
    try:
        sub = event_bus.subscribe(ReflexLayer.TOPIC)
        layer = ReflexLayer(sched, event_bus, debounce_window_ms=20)

        await layer.handle(stop_cmd())
        await asyncio.sleep(0.03)
        d2 = await layer.handle(stop_cmd())
        assert d2.published is True

        envs = await _collect(sub, 2)
        assert len(envs) == 2
    finally:
        sched.shutdown()


# ---------------------------------------------------------------------------
# MOVE_DIRECTION routing — parallel, channels independent (§8.10.6 scenario 3)
# ---------------------------------------------------------------------------


@pytest.mark.asyncio
async def test_direction_submits_to_factory_task_on_voice_channel(event_bus):
    """The nav_factory contract: produces a SchedulerTask that the
    layer hands to the scheduler. The test uses the Phase 1 ``voice``
    channel as a stand-in because ``nav`` lands in Phase 2 — the
    routing logic is channel-agnostic."""

    sched = _make_scheduler()
    try:
        seen: list[SchedulerTask] = []

        def factory(event: ReflexEvent) -> SchedulerTask:
            assert event.kind is ReflexKind.MOVE_DIRECTION
            assert event.entities["direction"] == "right"
            t = SchedulerTask(
                task_id="nav-1",
                tool="move_direction",
                channel=ChannelKind.VOICE,  # stand-in until nav channel lands
                executor=_noop_exec,
                args={"direction": event.entities["direction"]},
            )
            seen.append(t)
            return t

        layer = ReflexLayer(sched, event_bus, nav_factory=factory)
        decision = await layer.handle(direction_cmd("right"))

        assert decision.submitted is not None
        assert decision.submitted.channel is ChannelKind.VOICE
        assert len(seen) == 1
        # scheduler stores the task under that id
        assert sched.get_task("nav-1") is decision.submitted
    finally:
        sched.shutdown()


@pytest.mark.asyncio
async def test_direction_does_not_cancel_voice_channel(event_bus):
    """§8.10.6 scenario 3: voice keeps playing while nav runs in parallel.

    Both the voice-playing task and the direction task land on the
    ``voice`` channel for the test (Phase 2 will introduce ``nav``);
    the reflex layer must still NOT cancel the running voice task.
    """

    sched = _make_scheduler()
    try:
        voice_started = asyncio.Event()
        voice_done = asyncio.Event()

        async def _voice_exec(task: SchedulerTask) -> TaskResult:
            voice_started.set()
            try:
                await asyncio.sleep(0.3)
            finally:
                voice_done.set()
            return TaskResult(payload="voice ok")

        async def _direction_exec(task: SchedulerTask) -> TaskResult:
            return TaskResult(payload="direction ok")

        voice_task = SchedulerTask(
            task_id="v-1", tool="speak_text", channel=ChannelKind.VOICE, executor=_voice_exec
        )
        sched.submit(voice_task)

        # Wait for the voice executor to actually start.
        await voice_started.wait()

        def factory(_event: ReflexEvent) -> SchedulerTask:
            return SchedulerTask(
                task_id="dir-1", tool="move_direction",
                channel=ChannelKind.VOICE, executor=_direction_exec,
            )

        layer = ReflexLayer(sched, event_bus, nav_factory=factory)
        decision = await layer.handle(direction_cmd("left"))

        assert decision.submitted is not None
        # Voice is still in flight — it was NOT cancelled by the direction event.
        assert voice_task.status is TaskStatus.RUNNING

        # Let things settle.
        await asyncio.wait_for(voice_done.wait(), timeout=1.0)
        assert voice_task.status is TaskStatus.COMPLETED
    finally:
        sched.shutdown()


@pytest.mark.asyncio
async def test_direction_without_factory_still_publishes(event_bus):
    """No nav_factory → decision reason reflects that, but the bus
    envelope still goes out so the LLM feedback block sees it."""

    sched = _make_scheduler()
    try:
        sub = event_bus.subscribe(ReflexLayer.TOPIC)
        layer = ReflexLayer(sched, event_bus)  # no nav_factory
        decision = await layer.handle(direction_cmd("right"))

        assert decision.submitted is None
        assert decision.published is True
        assert decision.reason == "no_nav_factory"

        env = await sub.get()
        assert env.payload["kind"] == "move_direction"
    finally:
        sched.shutdown()


@pytest.mark.asyncio
async def test_factory_returning_wrong_type_is_a_programmer_error(event_bus):
    sched = _make_scheduler()
    try:
        def factory(_event: ReflexEvent) -> int:
            return 42  # not a SchedulerTask

        layer = ReflexLayer(sched, event_bus, nav_factory=factory)
        with pytest.raises(TypeError):
            await layer.handle(direction_cmd("right"))
    finally:
        sched.shutdown()


@pytest.mark.asyncio
async def test_factory_exception_does_not_publish_and_does_not_raise(event_bus):
    sched = _make_scheduler()
    try:
        sub = event_bus.subscribe(ReflexLayer.TOPIC)
        layer = ReflexLayer(
            sched, event_bus, nav_factory=lambda ev: (_ for _ in ()).throw(
                RuntimeError("nav down")
            )
        )
        decision = await layer.handle(direction_cmd("right"))

        # Factory failed → no submitted task; envelope still publishes so
        # observability is preserved.
        assert decision.submitted is None
        assert decision.published is True
        env = await asyncio.wait_for(sub.get(), timeout=0.5)
        assert env.payload["kind"] == "move_direction"
    finally:
        sched.shutdown()


# ---------------------------------------------------------------------------
# Other kinds — observable only
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    "cmd", [follow_cmd(), status_cmd(), map_cmd(), vision_cmd(), unknown_cmd()]
)
@pytest.mark.asyncio
async def test_non_reflex_kinds_publish_without_side_effects(event_bus, cmd):
    sched = _make_scheduler()
    try:
        sub = event_bus.subscribe(ReflexLayer.TOPIC)
        layer = ReflexLayer(sched, event_bus)
        decision = await layer.handle(cmd)

        assert decision.published is True
        assert decision.cancelled == 0
        assert decision.submitted is None
        assert decision.reason == "routed"

        env = await asyncio.wait_for(sub.get(), timeout=0.5)
        assert env.payload["kind"] == cmd.intent_value
    finally:
        sched.shutdown()


# ---------------------------------------------------------------------------
# Metrics / feedback block (§7 / §8.10.2)
# ---------------------------------------------------------------------------


@pytest.mark.asyncio
async def test_metrics_count_per_kind(event_bus):
    sched = _make_scheduler()
    try:
        layer = ReflexLayer(sched, event_bus)

        await layer.handle(stop_cmd())
        await layer.handle(stop_cmd())  # debounced — does NOT increment
        await layer.handle(direction_cmd("right"))
        await layer.handle(follow_cmd())

        snap = layer.metrics.snapshot()
        assert snap["counters"]["stop"] == 1
        assert snap["counters"]["move_direction"] == 1
        assert snap["counters"]["follow"] == 1
        assert "status" not in snap["counters"]
    finally:
        sched.shutdown()


@pytest.mark.asyncio
async def test_metrics_rolling_window_respects_size(event_bus):
    sched = _make_scheduler()
    try:
        layer = ReflexLayer(sched, event_bus, history_size=2)

        await layer.handle(follow_cmd())
        await layer.handle(follow_cmd())
        await layer.handle(follow_cmd())

        snap = layer.metrics.snapshot()
        assert len(snap["last_events"]) == 2
    finally:
        sched.shutdown()


@pytest.mark.asyncio
async def test_history_returns_frozen_view(event_bus):
    sched = _make_scheduler()
    try:
        layer = ReflexLayer(sched, event_bus)
        await layer.handle(stop_cmd())

        hist = layer.history()
        assert len(hist) == 1
        assert hist[0].kind is ReflexKind.STOP
        # Tuple → immutable from the caller's POV.
        assert isinstance(hist, tuple)
    finally:
        sched.shutdown()


# ---------------------------------------------------------------------------
# Decision / Event dataclasses
# ---------------------------------------------------------------------------


def test_reflex_event_is_frozen_and_serialisable():
    event = ReflexLayer.make_event(
        kind=ReflexKind.MOVE_DIRECTION,
        priority=ReflexPriority.HIGH,
        text="налево",
        entities={"direction": "left"},
        confidence=0.91,
        correlation_id="corr-1",
    )
    with pytest.raises(Exception):
        event.kind = ReflexKind.STOP  # type: ignore[misc]

    payload = event.to_payload()
    assert payload == {
        "event_id": event.event_id,
        "kind": "move_direction",
        "priority": "high",
        "source": "reflex",
        "text": "налево",
        "entities": {"direction": "left"},
        "confidence": 0.91,
        "correlation_id": "corr-1",
    }


def test_metrics_default_factories_are_independent():
    """Two metrics instances must not share their counters/last_events."""

    a = ReflexMetrics()
    b = ReflexMetrics()
    a.counters[ReflexKind.STOP] = 1
    a.last_events.append(
        ReflexLayer.make_event(kind=ReflexKind.STOP)
    )
    assert b.counters == {}
    assert list(b.last_events) == []


# ---------------------------------------------------------------------------
# Constructor validation
# ---------------------------------------------------------------------------


@pytest.mark.asyncio
async def test_constructor_rejects_invalid_arguments(event_bus):
    sched = _make_scheduler()
    try:
        with pytest.raises(ValueError):
            ReflexLayer(sched, event_bus, debounce_window_ms=-1)
        with pytest.raises(ValueError):
            ReflexLayer(sched, event_bus, history_size=0)
    finally:
        sched.shutdown()


def test_default_constants_match_design():
    """§8.10.6 scenario 4 calls for a 500 ms STOP debounce window."""

    assert DEFAULT_DEBOUNCE_MS == 500
    assert DEFAULT_HISTORY_SIZE >= 8


# ---------------------------------------------------------------------------
# Concurrency
# ---------------------------------------------------------------------------


@pytest.mark.asyncio
async def test_concurrent_stop_calls_are_serialised_by_lock(event_bus):
    """Two concurrent STOPs inside the debounce window → only one
    actually publishes (the second is suppressed)."""

    sched = _make_scheduler()
    try:
        sub = event_bus.subscribe(ReflexLayer.TOPIC)
        layer = ReflexLayer(sched, event_bus)

        results = await asyncio.gather(
            layer.handle(stop_cmd()),
            layer.handle(stop_cmd()),
            layer.handle(stop_cmd()),
        )
        published = [d for d in results if d.published]
        debounced = [d for d in results if not d.published]
        assert len(published) == 1
        assert len(debounced) == 2

        # Exactly one envelope arrived.
        env = await asyncio.wait_for(sub.get(), timeout=0.5)
        with pytest.raises(asyncio.TimeoutError):
            await asyncio.wait_for(sub.get(), timeout=0.05)
        assert env.payload["kind"] == "stop"
    finally:
        sched.shutdown()


# ---------------------------------------------------------------------------
# Integration with the EventBus subscribing to wildcard patterns
# ---------------------------------------------------------------------------


@pytest.mark.asyncio
async def test_multiple_subscribers_receive_reflex_event(event_bus):
    sched = _make_scheduler()
    try:
        sub_a = event_bus.subscribe(ReflexLayer.TOPIC)
        sub_b = event_bus.subscribe("/reflex/*")
        layer = ReflexLayer(sched, event_bus)

        await layer.handle(stop_cmd())

        env_a = await asyncio.wait_for(sub_a.get(), timeout=0.5)
        env_b = await asyncio.wait_for(sub_b.get(), timeout=0.5)
        assert env_a.payload["event_id"] == env_b.payload["event_id"]
    finally:
        sched.shutdown()
