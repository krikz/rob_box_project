"""reflex.py — Phase 2.5 Reflex layer (issue #968 §8.10).

Isolated reflex-path layer that consumes parsed voice commands (the
output of :class:`~rob_box_voice.core.command_parser.CommandParser`)
and routes them straight to the scheduler / scheduler event bus,
**without** going through the LLM. See ``docs/design/SCHEDULER_DESIGN.md``
§8.10 and §11.3 (card #5 in ``docs/design/CHILD_TASKS_PROPOSAL.md``)
for the full design.

Why isolated
------------

The reflex path already exists in ``command_node.py`` (regex match →
Nav2 goal, ≤ 500 ms, no LLM). The scheduler MUST coexist with that
path, not replace it — this module is the glue that:

1. publishes a structured :class:`ReflexEvent` to the
   :class:`EventBus` on topic ``/reflex/events`` with
   ``source="reflex"`` and the correct priority, so downstream
   consumers (LLM feedback, monitoring) see reflex activity;
2. short-circuits the planner for **safety** commands (STOP) by
   cancelling every active task on every channel synchronously;
3. forwards **direction** commands as new tasks on the navigation
   channel so they run in parallel with voice/music (channels
   are independent — see §3.1);
4. debounces repeated STOPs inside a configurable window (matches
   the §8.10.6 scenario-4 debounce already in ``command_node``);
5. records per-kind counters and a rolling window of the last
   :attr:`ReflexLayer.history_size` events for metrics/feedback.

Non-goals
---------

* No rewrite of ``command_node.py``. The publish to the bus is
  *additive* — the existing Nav2 path keeps working.
* No state machine. The layer is a stateless classifier + router;
  state lives in the underlying :class:`TaskScheduler` /
  :class:`EventBus`.
* No ROS dependency. The module is pure Python + ``asyncio`` and is
  unit-tested without rclpy.
"""

from __future__ import annotations

import asyncio
import logging
import time
import uuid
from collections import deque
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Callable, Deque, Mapping, Optional, Sequence

from .event_bus import EventBus, EventEnvelope
from .task_scheduler import (
    SchedulerTask,
    TaskScheduler,
    TaskStatus,
)

# Set of statuses for which :meth:`TaskScheduler.cancel` is a no-op
# (already terminal). Pulled out so :meth:`ReflexLayer._cancel_all`
# does not poke finished tasks unnecessarily.
_TERMINAL_STATUSES = frozenset(
    {TaskStatus.COMPLETED, TaskStatus.FAILED, TaskStatus.CANCELLED}
)

_LOG = logging.getLogger(__name__)


#: Default debounce window for duplicate STOP reflex events (ms).
#: Matches §8.10.6 scenario 4 — repeated «стоп» within 500 ms is
#: suppressed after the first one.
DEFAULT_DEBOUNCE_MS: int = 500

#: Default size of the rolling window kept in
#: :attr:`ReflexMetrics.last_events`. Surfaced via the §7
#: ``[REFLEX EVENTS]`` feedback block.
DEFAULT_HISTORY_SIZE: int = 32


# ---------------------------------------------------------------------------
# Public enums & dataclasses
# ---------------------------------------------------------------------------


class ReflexKind(str, Enum):
    """Classification of a parsed voice command for the reflex path.

    Mirrors :class:`rob_box_voice.core.command_parser.IntentType` so
    that the parser → reflex bridge is a straight enum mapping, but
    stays decoupled (the reflex layer does not import
    ``command_parser``).
    """

    STOP = "stop"
    MOVE_DIRECTION = "move_direction"
    FOLLOW = "follow"
    STATUS = "status"
    MAP = "map"
    VISION = "vision"
    UNKNOWN = "unknown"


class ReflexPriority(str, Enum):
    """Priority class for a :class:`ReflexEvent`.

    Critical wins over everything (STOP — reflex always beats the
    planner per §8.10.3). High wins over normal tasks but loses to
    critical (direction while driving — REPLACE on the nav channel).
    Normal is informational — the event is published for observability
    but does not cancel anything.
    """

    CRITICAL = "critical"
    HIGH = "high"
    NORMAL = "normal"


@dataclass(frozen=True)
class ReflexEvent:
    """Structured representation of a single reflex trigger.

    Built by :meth:`ReflexLayer.handle` from the underlying
    :class:`~rob_box_voice.core.command_parser.Command`. Frozen so
    downstream subscribers cannot accidentally mutate the event
    before it is observed.

    Attributes:
        event_id: uuid4 hex — stable across the bus + the metrics
            history so consumers can correlate.
        kind: Which :class:`ReflexKind` was recognised.
        priority: Routing priority.
        source: Always ``"reflex"`` for events emitted by this
            layer (kept as a field so test doubles can override).
        text: Original command text, lowercased and wake-word
            stripped — what the regex actually matched.
        entities: Entities extracted by the parser (direction,
            waypoint, …). Empty for STOP.
        confidence: Parser confidence score in ``[0.0, 1.0]``.
        created_at: ``time.monotonic()`` at construction.
        correlation_id: Optional caller-supplied correlation id;
            propagated into the published :class:`EventEnvelope`.
    """

    event_id: str = field(default_factory=lambda: uuid.uuid4().hex)
    kind: ReflexKind = ReflexKind.UNKNOWN
    priority: ReflexPriority = ReflexPriority.NORMAL
    source: str = "reflex"
    text: str = ""
    entities: Mapping[str, Any] = field(default_factory=dict)
    confidence: float = 0.0
    created_at: float = field(default_factory=time.monotonic)
    correlation_id: Optional[str] = None

    def to_payload(self) -> dict[str, Any]:
        """Return a JSON-friendly dict for the :class:`EventEnvelope`."""

        return {
            "event_id": self.event_id,
            "kind": self.kind.value,
            "priority": self.priority.value,
            "source": self.source,
            "text": self.text,
            "entities": dict(self.entities),
            "confidence": self.confidence,
            "correlation_id": self.correlation_id,
        }


@dataclass(frozen=True)
class ReflexDecision:
    """Outcome returned to the caller of :meth:`ReflexLayer.handle`.

    Captures what the layer actually did so callers (typically
    ``command_node``) can act on the result without inspecting the
    scheduler directly:

    * ``event`` — the :class:`ReflexEvent` that was emitted (or, if
      the command was suppressed by debounce, the suppressed event).
    * ``cancelled`` — how many tasks the scheduler cancelled
      synchronously (``STOP`` path). Zero for non-STOP kinds.
    * ``submitted`` — the :class:`SchedulerTask` submitted to the
      navigation channel (``MOVE_DIRECTION`` path), or ``None`` for
      other kinds.
    * ``published`` — ``True`` when the event was forwarded to the
      :class:`EventBus`. ``False`` only for debounced duplicates.
    * ``reason`` — short, log-friendly tag (``"routed"``,
      ``"debounced"``, ``"no_nav_channel"``).
    """

    event: ReflexEvent
    cancelled: int = 0
    submitted: Optional[SchedulerTask] = None
    published: bool = False
    reason: str = "routed"


@dataclass
class ReflexMetrics:
    """Per-kind counters + a rolling window of recent events.

    Exposed via :attr:`ReflexLayer.metrics` for the §7 LLM feedback
    block (``[REFLEX EVENTS]``) and for unit-test assertions. The
    counters never decrement; ``last_events`` is the rolling
    window that lets the LLM see *which* reflex events fired
    recently.
    """

    counters: dict[ReflexKind, int] = field(default_factory=dict)
    last_events: Deque[ReflexEvent] = field(default_factory=deque)

    def record(self, event: ReflexEvent, *, window: int) -> None:
        """Increment the counter for ``event.kind`` and append to the
        rolling window (oldest entries are dropped past ``window``).
        """

        self.counters[event.kind] = self.counters.get(event.kind, 0) + 1
        self.last_events.append(event)
        while len(self.last_events) > window:
            self.last_events.popleft()

    def snapshot(self) -> dict[str, Any]:
        """JSON-friendly snapshot for logging / feedback."""

        return {
            "counters": {k.value: v for k, v in self.counters.items()},
            "last_events": [event.to_payload() for event in self.last_events],
        }


# ---------------------------------------------------------------------------
# Command bridge
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class _ReflexCommandView:
    """Minimal projection of a parsed command for the reflex layer.

    The reflex layer does NOT import
    :class:`rob_box_voice.core.command_parser.CommandParser` —
    callers convert their parsed command into this lightweight view
    so the layer stays independently testable.
    """

    intent: str
    text: str
    entities: Mapping[str, Any]
    confidence: float = 0.0


def command_to_view(command: Any) -> _ReflexCommandView:
    """Convert a parsed command (duck-typed) into a reflex view.

    Accepts either a
    :class:`rob_box_voice.core.command_parser.Command` or any object
    with ``intent.value`` (an enum-like), ``text``, ``entities`` and
    ``confidence`` attributes. Raises :class:`TypeError` if the
    object does not expose the required surface.
    """

    intent = getattr(command, "intent", None)
    intent_value = getattr(intent, "value", intent)
    if intent_value is None:
        raise TypeError("command must expose an .intent with a value")
    text = getattr(command, "text", "") or ""
    entities = getattr(command, "entities", {}) or {}
    confidence = getattr(command, "confidence", 0.0) or 0.0
    if not isinstance(entities, Mapping):
        raise TypeError("command.entities must be a mapping")
    return _ReflexCommandView(
        intent=str(intent_value),
        text=str(text),
        entities=dict(entities),
        confidence=float(confidence),
    )


# ---------------------------------------------------------------------------
# RefexLayer
# ---------------------------------------------------------------------------


class ReflexLayer:
    """Route parsed reflex commands into the scheduler / event bus.

    Parameters
    ----------
    scheduler:
        :class:`TaskScheduler` whose channels the layer drives. The
        ``nav`` channel is **not** added by the layer — callers that
        want direction events submitted to the scheduler must
        construct the scheduler with the navigation channel included
        (Phase 2 work tracks when ``nav`` becomes a default) and
        supply a :paramref:`nav_factory` that returns tasks on that
        channel. Without a factory, direction events are still
        published on the bus but the scheduler side-effect is
        skipped (decision ``reason="no_nav_factory"``).
    event_bus:
        :class:`EventBus` the layer publishes to. Every accepted
        reflex event becomes an :class:`EventEnvelope` on topic
        ``/reflex/events``.
    debounce_window_ms:
        Window in milliseconds during which a duplicate STOP is
        suppressed (matches the §8.10.6 scenario-4 contract).
        ``0`` disables debouncing.
    history_size:
        How many recent events to keep in
        :attr:`ReflexLayer.metrics` for the ``[REFLEX EVENTS]``
        feedback block.
    nav_factory:
        Optional callable producing the :class:`SchedulerTask` for a
        MOVE_DIRECTION event. Receives the :class:`ReflexEvent` and
        must return a :class:`SchedulerTask`. If omitted, the layer
        publishes the event and emits a ``no_nav_factory`` decision
        (useful for tests and for code paths that handle nav
        elsewhere — e.g. ``command_node``).
    correlation_id_provider:
        Optional callable that returns a correlation id for a
        given :class:`ReflexEvent` (e.g. ASR utterance id). When
        ``None`` the layer generates per-event uuids but does not
        propagate an external correlation.

    Threading
    ---------
    All public methods are coroutines and must be awaited on the
    asyncio loop that owns ``scheduler`` and ``event_bus``. The
    debounce bookkeeping is guarded by an ``asyncio.Lock`` so
    concurrent :meth:`handle` calls cannot race on the timestamp
    of the last accepted STOP.
    """

    TOPIC: str = "/reflex/events"
    """Topic the layer publishes on. Matches §8.10.2 / §11.3."""

    def __init__(
        self,
        scheduler: TaskScheduler,
        event_bus: EventBus,
        *,
        debounce_window_ms: int = DEFAULT_DEBOUNCE_MS,
        history_size: int = DEFAULT_HISTORY_SIZE,
        nav_factory: Optional[Callable[[ReflexEvent], SchedulerTask]] = None,
        correlation_id_provider: Optional[Callable[[ReflexEvent], Optional[str]]] = None,
    ) -> None:
        if debounce_window_ms < 0:
            raise ValueError("debounce_window_ms must be >= 0")
        if history_size <= 0:
            raise ValueError("history_size must be > 0")

        self._scheduler = scheduler
        self._event_bus = event_bus
        self._debounce_window_ms = debounce_window_ms
        self._history_size = history_size
        self._nav_factory = nav_factory
        self._correlation_id_provider = correlation_id_provider

        self._last_stop_at: Optional[float] = None
        self._lock = asyncio.Lock()
        self.metrics = ReflexMetrics()

    # ----- public surface ------------------------------------------------

    @property
    def scheduler(self) -> TaskScheduler:
        """Underlying scheduler (read-only handle)."""
        return self._scheduler

    @property
    def event_bus(self) -> EventBus:
        """Underlying event bus (read-only handle)."""
        return self._event_bus

    async def handle(self, command: Any) -> ReflexDecision:
        """Route a parsed command through the reflex layer.

        Steps (matches §8.10.2):

        1. Convert the command to a :class:`_ReflexCommandView`.
        2. Build a :class:`ReflexEvent` (kind/priority from intent).
        3. Debounce duplicate STOPs (returns ``debounced`` without
           publishing or cancelling).
        4. Publish on ``/reflex/events`` and run the kind-specific
           side-effect (cancel-all for STOP, submit for
           MOVE_DIRECTION, no-op for the rest).
        5. Record metrics, return :class:`ReflexDecision`.
        """

        view = command_to_view(command)
        event = self._build_event(view)
        async with self._lock:
            if event.kind is ReflexKind.STOP and self._is_debounced():
                _LOG.info(
                    "reflex: STOP debounced (window=%dms, text=%r)",
                    self._debounce_window_ms,
                    event.text,
                )
                return ReflexDecision(event=event, reason="debounced")
            self._maybe_record_stop_timestamp(event)
            decision = await self._dispatch(event)
        if decision.published:
            self.metrics.record(event, window=self._history_size)
        return decision

    async def publish_only(self, event: ReflexEvent) -> bool:
        """Publish a pre-built :class:`ReflexEvent` without side effects.

        Useful when callers construct the event themselves (e.g.
        integration with ``command_node`` that already ran the Nav2
        side-effect) but still want the bus-level observability +
        metrics. Returns ``True`` when the envelope reached the bus
        (regardless of subscriber count), ``False`` when the bus
        rejected it (e.g. closed).
        """

        try:
            delivered = await self._event_bus.publish(self._envelope(event))
        except Exception as exc:  # noqa: BLE001 — bus boundary
            _LOG.warning("reflex: publish failed: %s: %s", type(exc).__name__, exc)
            return False
        self.metrics.record(event, window=self._history_size)
        _LOG.info(
            "reflex: published event_id=%s kind=%s priority=%s delivered=%d",
            event.event_id,
            event.kind.value,
            event.priority.value,
            delivered,
        )
        return True

    # ----- internals -----------------------------------------------------

    def _build_event(self, view: _ReflexCommandView) -> ReflexEvent:
        kind, priority = self._classify(view)
        correlation: Optional[str] = None
        if self._correlation_id_provider is not None:
            # The provider needs the *kind* to make its decision (e.g.
            # an ASR utterance id may differ between STOP and direction);
            # hand it a provisional event with the right kind and the
            # payload that is already known at this stage.
            provisional = ReflexEvent(
                kind=kind,
                priority=priority,
                text=view.text,
                entities=view.entities,
                confidence=view.confidence,
            )
            try:
                correlation = self._correlation_id_provider(provisional)
            except Exception as exc:  # noqa: BLE001 — provider boundary
                _LOG.warning(
                    "reflex: correlation_id_provider raised %s: %s",
                    type(exc).__name__,
                    exc,
                )
        return ReflexEvent(
            kind=kind,
            priority=priority,
            text=view.text,
            entities=view.entities,
            confidence=view.confidence,
            correlation_id=correlation,
        )

    @staticmethod
    def _classify(view: _ReflexCommandView) -> tuple[ReflexKind, ReflexPriority]:
        """Map a parsed command to ``(kind, priority)``.

        Mapping is intentionally narrow: any intent not explicitly
        handled collapses to ``(UNKNOWN, NORMAL)`` so the layer
        never invents behaviour the parser did not endorse.
        """

        intent = view.intent.lower()
        if intent == "stop":
            return ReflexKind.STOP, ReflexPriority.CRITICAL
        if intent == "navigate":
            # NAVIGATE contains two reflex cases:
            # * direction (entities["direction"]) — parallel, HIGH
            # * absolute waypoint (entities["waypoint"]) — not a
            #   reflex correction; PUBLISHED but not routed here.
            if "direction" in view.entities:
                return ReflexKind.MOVE_DIRECTION, ReflexPriority.HIGH
            return ReflexKind.MOVE_DIRECTION, ReflexPriority.NORMAL
        if intent == "follow":
            return ReflexKind.FOLLOW, ReflexPriority.NORMAL
        if intent == "status":
            return ReflexKind.STATUS, ReflexPriority.NORMAL
        if intent == "map":
            return ReflexKind.MAP, ReflexPriority.NORMAL
        if intent == "vision":
            return ReflexKind.VISION, ReflexPriority.NORMAL
        return ReflexKind.UNKNOWN, ReflexPriority.NORMAL

    def _is_debounced(self) -> bool:
        if self._debounce_window_ms == 0 or self._last_stop_at is None:
            return False
        window_s = self._debounce_window_ms / 1000.0
        return (time.monotonic() - self._last_stop_at) < window_s

    def _maybe_record_stop_timestamp(self, event: ReflexEvent) -> None:
        if event.kind is ReflexKind.STOP and self._debounce_window_ms > 0:
            self._last_stop_at = time.monotonic()

    async def _dispatch(self, event: ReflexEvent) -> ReflexDecision:
        envelope = self._envelope(event)
        cancelled = 0
        submitted: Optional[SchedulerTask] = None
        reason = "routed"

        if event.kind is ReflexKind.STOP:
            cancelled = self._cancel_all()
            _LOG.info(
                "reflex: STOP cancelled %d task(s) (text=%r, confidence=%.2f)",
                cancelled,
                event.text,
                event.confidence,
            )
        elif event.kind is ReflexKind.MOVE_DIRECTION and event.priority is ReflexPriority.HIGH:
            submitted = self._route_direction(event)
            if submitted is None:
                reason = "no_nav_factory"

        try:
            delivered = await self._event_bus.publish(envelope)
        except Exception as exc:  # noqa: BLE001 — bus boundary
            _LOG.warning("reflex: publish failed: %s: %s", type(exc).__name__, exc)
            return ReflexDecision(
                event=event,
                cancelled=cancelled,
                submitted=submitted,
                published=False,
                reason=f"publish_failed:{type(exc).__name__}",
            )

        _LOG.info(
            "reflex: published event_id=%s kind=%s priority=%s delivered=%d",
            event.event_id,
            event.kind.value,
            event.priority.value,
            delivered,
        )
        return ReflexDecision(
            event=event,
            cancelled=cancelled,
            submitted=submitted,
            published=True,
            reason=reason,
        )

    def _cancel_all(self) -> int:
        """Cancel every non-terminal task in the scheduler.

        Implemented locally instead of on the scheduler to keep the
        reflex layer fully decoupled — Phase 2's :class:`EventBus`
        cancel preemption lands separately on the scheduler, and
        the reflex layer should not depend on a method that is not
        yet public. Returns the number of tasks that were either
        removed from a queue or reported as already-running (a
        best-effort log-only signal in the Phase 1 MVP).
        """

        cancelled = 0
        snapshot = self._scheduler.tasks_snapshot()
        for task_id, payload in snapshot.items():
            status = payload.get("status")
            if status in {s.value for s in _TERMINAL_STATUSES}:
                continue
            if self._scheduler.cancel(task_id):
                cancelled += 1
        return cancelled

    def _route_direction(self, event: ReflexEvent) -> Optional[SchedulerTask]:
        if self._nav_factory is None:
            _LOG.debug(
                "reflex: no nav_factory configured; direction event published "
                "without scheduler submission (event_id=%s)",
                event.event_id,
            )
            return None
        try:
            task = self._nav_factory(event)
        except Exception as exc:  # noqa: BLE001 — factory boundary
            _LOG.error(
                "reflex: nav_factory raised %s: %s (event_id=%s)",
                type(exc).__name__,
                exc,
                event.event_id,
            )
            return None
        if not isinstance(task, SchedulerTask):
            raise TypeError(
                "nav_factory must return a SchedulerTask instance, got "
                f"{type(task).__name__}"
            )
        # Channels are independent (§3.1): direction does NOT cancel
        # voice/music, it just enqueues on nav. The nav channel must
        # already exist; if it does not, the scheduler raises
        # TaskSubmitError which we surface via the decision's
        # `reason` (see _dispatch → _route_direction caller).
        self._scheduler.submit(task)
        return task

    def _envelope(self, event: ReflexEvent) -> EventEnvelope:
        return EventEnvelope(
            topic=self.TOPIC,
            payload=event.to_payload(),
            correlation_id=event.correlation_id or event.event_id,
        )

    # ----- helpers for tests --------------------------------------------

    @staticmethod
    def make_event(
        *,
        kind: ReflexKind,
        priority: ReflexPriority = ReflexPriority.NORMAL,
        text: str = "",
        entities: Optional[Mapping[str, Any]] = None,
        confidence: float = 0.0,
        correlation_id: Optional[str] = None,
    ) -> ReflexEvent:
        """Construct a :class:`ReflexEvent` for tests / fixtures.

        Lives on the class (instead of being a free function) so it
        is discoverable from :class:`ReflexLayer` and mirrors the
        layer's vocabulary.
        """

        return ReflexEvent(
            kind=kind,
            priority=priority,
            text=text,
            entities=dict(entities or {}),
            confidence=confidence,
            correlation_id=correlation_id,
        )

    def history(self) -> Sequence[ReflexEvent]:
        """Read-only snapshot of the metrics rolling window."""

        return tuple(self.metrics.last_events)


__all__ = [
    "DEFAULT_DEBOUNCE_MS",
    "DEFAULT_HISTORY_SIZE",
    "ReflexDecision",
    "ReflexEvent",
    "ReflexKind",
    "ReflexLayer",
    "ReflexMetrics",
    "ReflexPriority",
    "command_to_view",
]
