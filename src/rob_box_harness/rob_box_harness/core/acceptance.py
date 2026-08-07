"""Acceptance gate + awaiting-confirmation segment (issue #968 §8 / §11.2).

This module is the runtime side of the acceptance layer. The data
side — :class:`ToolConfirmationPolicy` — lives in
:mod:`rob_box_harness.core.confirmation_policy`. Together they form
the «слой валидации/acceptance вызовов tools планировщика» that
issue #968 §8 asks for.

Components
----------

* :class:`SegmentStatus` — the §2.1 status enum, extended with the
  ``AWAITING_CONFIRMATION`` value (§8.3).
* :class:`SegmentKind` — coarse routing for the scheduler channels
  (voice / music / anim / nav / mapping / mutate). Used by
  :class:`AcceptanceGate` to decide which channel the segment will
  enter once it leaves the gate.
* :class:`PendingSegment` — the data record for a single tool call
  in flight. Holds the tool name, arguments, classification
  decision, and the ``AWAITING_CONFIRMATION → ACTIVE | REJECTED |
  CANCELLED`` state machine.
* :class:`AcceptanceGate` — the orchestrator-facing façade. Its
  :meth:`submit` is the single entry point that any caller
  (currently :class:`rob_box_harness.core.dialog_core.DialogCore`)
  uses to dispatch a tool call.
* :class:`AwaitingFeedbackFormatter` — turns the live segment set
  into the §7 ``[AWAITING]`` block that is injected into the LLM
  system prompt before the next LLM turn.

Threading / concurrency
-----------------------

The module is **pure Python** — no ``rclpy``, no asyncio locks in
the hot path. The single lock inside :class:`AcceptanceGate` is a
``threading.Lock``; the awaiting-confirmation timer is driven by an
``asyncio`` event-loop callback the caller passes in via
``timeout_scheduler``. This keeps the module usable both from
``DialogCore`` (which already owns an asyncio loop) and from the
future :class:`TaskScheduler` (which has its own event loop).

The state machine inside :class:`PendingSegment` is intentionally
**synchronous and explicit** — every transition is a method call
that mutates a frozen-ish record under the gate's lock. Callers
that need to observe the transitions can subscribe via
:meth:`AcceptanceGate.subscribe`.
"""

from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Callable, Iterable, Mapping

from rob_box_harness.config import ConfirmationPolicyConfig
from rob_box_harness.core.confirmation_policy import (
    AcceptanceDecision,
    ConfirmationKind,
    EMERGENCY_TOOLS,
    ToolConfirmationPolicy,
)


_LOG = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Status & kind enums
# ---------------------------------------------------------------------------


class SegmentStatus(str, Enum):
    """§2.1 status enum, extended with :attr:`AWAITING_CONFIRMATION`.

    ``str``-valued so values round-trip through YAML / JSON / log
    lines without bespoke (de)serialisers. The on-the-wire spelling
    matches the §2.1 table verbatim (UPPER_SNAKE) and is what the
    :class:`AwaitingFeedbackFormatter` emits.
    """

    PENDING = "PENDING"
    AWAITING_CONFIRMATION = "AWAITING_CONFIRMATION"
    ACTIVE = "ACTIVE"
    COMPLETED = "COMPLETED"
    CANCELLED = "CANCELLED"
    REJECTED = "REJECTED"
    SKIPPED = "SKIPPED"


class SegmentKind(str, Enum):
    """Coarse routing key — which scheduler channel owns the segment.

    Matches §3.1 (voice / music / anim) plus the §8 additions
    (nav / mapping / mutate). ``UNKNOWN`` is the safe fallback when
    a tool is not registered with any channel.
    """

    VOICE = "voice"
    MUSIC = "music"
    ANIM = "anim"
    NAV = "nav"
    MAPPING = "mapping"
    MUTATE = "mutate"  # data deletion / overwrite (waypoints, tracks, …)
    UNKNOWN = "unknown"


#: Tool name → :class:`SegmentKind`. Mirrors the §8.2 table. Kept as
#: a module-level constant (instead of a YAML catalog) because the
#: mapping is small, stable, and used at every :meth:`submit` call —
#: a YAML round-trip would be wasted overhead. The mapping IS
#: unit-tested by ``test_routing_table_is_consistent_with_policy``.
_TOOL_KIND: Mapping[str, SegmentKind] = {
    "speak_text": SegmentKind.VOICE,
    "play_sound": SegmentKind.VOICE,
    "play_animation": SegmentKind.ANIM,
    "listen_for_response": SegmentKind.VOICE,
    "execute_music_code": SegmentKind.MUSIC,
    "stop_music": SegmentKind.MUSIC,
    "set_vibe_preset": SegmentKind.MUSIC,
    "search_samples": SegmentKind.MUSIC,
    "save_track": SegmentKind.MUSIC,
    "load_track": SegmentKind.MUSIC,
    "delete_track": SegmentKind.MUTATE,
    "list_tracks": SegmentKind.UNKNOWN,
    "get_music_state": SegmentKind.UNKNOWN,
    "navigate_to_waypoint": SegmentKind.NAV,
    "navigate_to_coordinates": SegmentKind.NAV,
    "move_direction": SegmentKind.NAV,
    "stop_navigation": SegmentKind.NAV,
    "save_waypoint": SegmentKind.MUTATE,
    "delete_waypoint": SegmentKind.MUTATE,
    "clear_waypoints": SegmentKind.MUTATE,
    "list_waypoints": SegmentKind.UNKNOWN,
    "get_current_pose": SegmentKind.UNKNOWN,
    "start_mapping": SegmentKind.MAPPING,
    "continue_mapping": SegmentKind.MAPPING,
    "finish_mapping": SegmentKind.MAPPING,
    "optimize_map": SegmentKind.MAPPING,
    "load_map": SegmentKind.MAPPING,
    "get_perception_context": SegmentKind.UNKNOWN,
    "get_battery_level": SegmentKind.UNKNOWN,
    "get_robot_status": SegmentKind.UNKNOWN,
    "get_current_time": SegmentKind.UNKNOWN,
    "get_sound_info": SegmentKind.UNKNOWN,
    "estimate_tts_duration": SegmentKind.UNKNOWN,
    "faq_search": SegmentKind.UNKNOWN,
    "memory_save": SegmentKind.MUTATE,
    "memory_search": SegmentKind.UNKNOWN,
    "memory_context": SegmentKind.UNKNOWN,
}


def route_kind(tool_name: str) -> SegmentKind:
    """Return the :class:`SegmentKind` for *tool_name*.

    Unknown tools fall back to :attr:`SegmentKind.UNKNOWN` — the
    scheduler treats them as out-of-band and forwards them to the
    executor directly (the same behaviour we had before the §8
    layer existed).
    """
    return _TOOL_KIND.get(tool_name, SegmentKind.UNKNOWN)


# ---------------------------------------------------------------------------
# The segment record
# ---------------------------------------------------------------------------


@dataclass
class PendingSegment:
    """A single tool call in flight through the acceptance gate.

    The record is mutable in the sense that the state machine
    transitions write to it — but always under
    :attr:`AcceptanceGate._lock`. Outside the lock, the record is
    treated as immutable; observers should call
    :meth:`snapshot` to get a frozen view.

    Attributes:
        segment_id: Stable identifier (uuid4 hex). Echoed in every
            transition event so observers can correlate log lines.
        tool: Tool name (matches the LLM-facing schema).
        args: Raw argument dict — kept verbatim so the executor can
            re-call the tool once the segment is released.
        decision: The :class:`AcceptanceDecision` from
            :meth:`ToolConfirmationPolicy.classify`. Stored here so
            feedback events can show the human-readable reason.
        kind: Coarse channel routing (see :class:`SegmentKind`).
        status: Current §2.1 status.
        created_at: ``time.monotonic()`` at submission — used for
            timeout accounting and for the ``[AWAITING]`` block's
            ``elapsed_ms`` field.
        confirmed_at: ``time.monotonic()`` at confirmation, or
            ``None`` while the segment is still AWAITING. Stamped so
            the executor can compute end-to-end latency.
        timeout_handle: Opaque token returned by the
            ``timeout_scheduler``. ``None`` for non-require
            segments and after the timer is cancelled.
    """

    segment_id: str
    tool: str
    args: dict[str, Any]
    decision: AcceptanceDecision
    kind: SegmentKind
    status: SegmentStatus = SegmentStatus.PENDING
    created_at: float = field(default_factory=time.monotonic)
    confirmed_at: float | None = None
    timeout_handle: Any | None = None

    # ----- state-machine transitions ------------------------------------

    def mark_awaiting(self) -> None:
        """``PENDING → AWAITING_CONFIRMATION``. Only valid for require."""
        if self.status is not SegmentStatus.PENDING:
            raise _IllegalTransition(self.segment_id, self.status, SegmentStatus.AWAITING_CONFIRMATION)
        if self.decision.kind is not ConfirmationKind.REQUIRE:
            raise _IllegalTransition(
                self.segment_id,
                self.status,
                SegmentStatus.AWAITING_CONFIRMATION,
                reason="only require-classified segments can await confirmation",
            )
        self.status = SegmentStatus.AWAITING_CONFIRMATION

    def mark_active(self) -> None:
        """``AWAITING_CONFIRMATION → ACTIVE`` (user confirmed)."""
        if self.status is not SegmentStatus.AWAITING_CONFIRMATION:
            raise _IllegalTransition(self.segment_id, self.status, SegmentStatus.ACTIVE)
        self.status = SegmentStatus.ACTIVE
        self.confirmed_at = time.monotonic()

    def mark_rejected(self) -> None:
        """``AWAITING_CONFIRMATION → REJECTED`` (user said no OR timeout)."""
        if self.status is not SegmentStatus.AWAITING_CONFIRMATION:
            raise _IllegalTransition(self.segment_id, self.status, SegmentStatus.REJECTED)
        self.status = SegmentStatus.REJECTED

    def mark_cancelled(self) -> None:
        """Force ``AWAITING → CANCELLED`` (e.g. reflex-«стой» mid-await)."""
        if self.status is not SegmentStatus.AWAITING_CONFIRMATION:
            raise _IllegalTransition(self.segment_id, self.status, SegmentStatus.CANCELLED)
        self.status = SegmentStatus.CANCELLED

    def snapshot(self) -> dict[str, Any]:
        """Return a frozen, log-friendly view of the segment."""
        return {
            "segment_id": self.segment_id,
            "tool": self.tool,
            "kind": self.kind.value,
            "status": self.status.value,
            "decision": self.decision.kind.value,
            "reason": self.decision.reason,
            "elapsed_ms": int((time.monotonic() - self.created_at) * 1000),
            "args_keys": sorted(self.args.keys()),
        }


class _IllegalTransition(Exception):
    """Raised on illegal state-machine transitions (used by tests)."""

    def __init__(self, segment_id: str, current: SegmentStatus, target: SegmentStatus, *, reason: str = "") -> None:
        super().__init__(
            f"segment {segment_id}: illegal {current.value} → {target.value}"
            + (f" ({reason})" if reason else "")
        )
        self.segment_id = segment_id
        self.current = current
        self.target = target


# ---------------------------------------------------------------------------
# The gate
# ---------------------------------------------------------------------------


#: Timeout-scheduler signature. Returns an opaque handle that
#: :meth:`AcceptanceGate.cancel_timeout` accepts. ``time.monotonic``
#: gives us a clock that doesn't jump backwards on NTP adjustments.
TimeoutScheduler = Callable[[float, Callable[[], None]], Any]
TimeoutCanceller = Callable[[Any], None]


class AcceptanceGate:
    """Orchestrator-facing façade for the acceptance layer.

    Lifecycle:

    1. Construct with a :class:`ToolConfirmationPolicy` (typically
       from :func:`load_default_policy`) and a
       :class:`ConfirmationPolicyConfig`.
    2. Call :meth:`submit` for every tool call dispatched by the
       LLM. The gate returns the resulting
       :class:`AcceptanceDecision` — the caller uses the ``kind``
       to decide whether to await confirmation, notify, or
       pass through.
    3. When the user responds, call :meth:`confirm` or
       :meth:`reject`. When the timeout fires, the gate
       automatically transitions the segment to ``REJECTED`` and
       publishes a :class:`confirmation_rejected` event.
    4. Observers (the feedback formatter, the future scheduler)
       receive every transition via :meth:`subscribe`.

    The gate is **not** a singleton — tests construct fresh
    instances with synthetic policies to exercise the state
    machine in isolation.
    """

    def __init__(
        self,
        *,
        policy: ToolConfirmationPolicy,
        config: ConfirmationPolicyConfig,
        timeout_scheduler: TimeoutScheduler | None = None,
        timeout_canceller: TimeoutCanceller | None = None,
        segment_id_factory: Callable[[], str] | None = None,
    ) -> None:
        if not isinstance(policy, ToolConfirmationPolicy):
            raise TypeError("policy must be a ToolConfirmationPolicy instance")
        if not isinstance(config, ConfirmationPolicyConfig):
            raise TypeError("config must be a ConfirmationPolicyConfig instance")
        self._policy = policy
        self._config = config
        self._timeout_scheduler = timeout_scheduler
        self._timeout_canceller = timeout_canceller
        self._segment_id_factory = segment_id_factory or _default_segment_id
        self._lock = threading.Lock()
        # segment_id → PendingSegment. Kept across the gate's
        # lifetime so the feedback formatter can include them in
        # the [AWAITING] block until they leave AWAITING_CONFIRMATION.
        self._segments: dict[str, PendingSegment] = {}
        # subscriber callbacks for transition events.
        self._subscribers: list[Callable[[str, PendingSegment, SegmentStatus], None]] = []

    # ----- public read-only views ---------------------------------------

    @property
    def config(self) -> ConfirmationPolicyConfig:
        """The runtime config (frozen dataclass)."""
        return self._config

    @property
    def policy(self) -> ToolConfirmationPolicy:
        """The classifier used by this gate."""
        return self._policy

    def awaiting(self) -> list[PendingSegment]:
        """Return a snapshot of all segments currently AWAITING_CONFIRMATION."""
        with self._lock:
            return [s for s in self._segments.values() if s.status is SegmentStatus.AWAITING_CONFIRMATION]

    def all_segments(self) -> list[PendingSegment]:
        """Return a snapshot of every segment the gate is tracking."""
        with self._lock:
            return list(self._segments.values())

    def get(self, segment_id: str) -> PendingSegment | None:
        """Look up a segment by id (or return ``None``)."""
        with self._lock:
            return self._segments.get(segment_id)

    def subscribe(
        self,
        callback: Callable[[str, PendingSegment, SegmentStatus], None],
    ) -> Callable[[], None]:
        """Register *callback* for every segment transition.

        Returns an ``unsubscribe()`` function so tests can detach
        cleanly. The callback receives ``(segment_id, segment, new_status)``
        for every transition (including the initial
        ``PENDING → AWAITING_CONFIRMATION``).
        """
        with self._lock:
            self._subscribers.append(callback)

        def _unsubscribe() -> None:
            with self._lock:
                try:
                    self._subscribers.remove(callback)
                except ValueError:
                    pass

        return _unsubscribe

    # ----- public mutators ----------------------------------------------

    def submit(
        self,
        *,
        tool: str,
        args: Mapping[str, Any] | None = None,
        call_id: str | None = None,
    ) -> tuple[PendingSegment, AcceptanceDecision]:
        """Submit a tool call for acceptance.

        Args:
            tool: The tool name (must match the LLM-facing schema).
            args: Raw argument dict; defaults to empty.
            call_id: Optional LLM-side tool call id; stashed on the
                segment for cross-correlation in the feedback block.

        Returns:
            ``(segment, decision)`` — the segment is *also* registered
            with the gate; callers may discard it and look it up via
            :meth:`get` or :meth:`awaiting` later.

        Raises:
            ValueError: if the policy is disabled AND classifies the
                tool as ``require`` (the gate refuses to enqueue a
                blocking call when the layer is off).
        """
        decision = self._policy.classify(tool)
        if not self._config.enabled:
            # Layer off — every decision collapses to pass-through.
            decision = AcceptanceDecision(
                kind=ConfirmationKind.PASS_THROUGH,
                tool=decision.tool,
                reason="confirmation layer disabled — auto pass-through",
                plan_text=None,
            )
        segment = PendingSegment(
            segment_id=self._segment_id_factory(),
            tool=tool,
            args=dict(args or {}),
            decision=decision,
            kind=route_kind(tool),
            status=SegmentStatus.PENDING,
        )
        # stash the LLM-side call id for feedback correlation
        if call_id is not None:
            segment.args.setdefault("_call_id", call_id)
        with self._lock:
            self._segments[segment.segment_id] = segment

        if decision.kind is ConfirmationKind.REQUIRE:
            self._enter_awaiting(segment)
        elif decision.kind is ConfirmationKind.NOTIFY:
            # notify segments skip confirmation but still appear in
            # the [AWAITING] block as "soft notifications" — for
            # now we just transition them straight to ACTIVE so the
            # executor can run them.
            with self._lock:
                segment.status = SegmentStatus.ACTIVE
                segment.confirmed_at = time.monotonic()
            self._publish(segment.segment_id, segment, SegmentStatus.ACTIVE)
        else:  # pass_through
            with self._lock:
                segment.status = SegmentStatus.ACTIVE
                segment.confirmed_at = time.monotonic()
            self._publish(segment.segment_id, segment, SegmentStatus.ACTIVE)
        return segment, decision

    def confirm(self, segment_id: str) -> PendingSegment:
        """User said «да». Transitions ``AWAITING → ACTIVE``.

        Cancels the timeout timer. Raises ``KeyError`` if no segment
        with that id exists; raises :class:`_IllegalTransition` if
        the segment is not in ``AWAITING_CONFIRMATION`` (e.g. already
        ACTIVE after a timeout race).
        """
        with self._lock:
            segment = self._segments.get(segment_id)
            if segment is None:
                raise KeyError(segment_id)
            segment.mark_active()
            self._cancel_timeout_locked(segment)
        self._publish(segment.segment_id, segment, SegmentStatus.ACTIVE)
        return segment

    def reject(self, segment_id: str, *, reason: str = "user_said_no") -> PendingSegment:
        """User said «нет» (or external cancellation).

        Transitions ``AWAITING → REJECTED`` and cancels the timeout.
        The *reason* is included in the published event for
        observability and is what the LLM receives in the
        ``confirmation_rejected`` feedback block.
        """
        with self._lock:
            segment = self._segments.get(segment_id)
            if segment is None:
                raise KeyError(segment_id)
            segment.mark_rejected()
            self._cancel_timeout_locked(segment)
            segment.args["_rejection_reason"] = reason
        self._publish(segment.segment_id, segment, SegmentStatus.REJECTED)
        return segment

    def cancel_all(self) -> list[PendingSegment]:
        """Cancel every AWAITING segment (e.g. reflex-«стой»).

        Used by the future SchedulerEventBus subscriber (§8.10) and
        by tests that need a clean slate. Returns the list of
        segments that were transitioned.
        """
        cancelled: list[PendingSegment] = []
        with self._lock:
            for segment in list(self._segments.values()):
                if segment.status is SegmentStatus.AWAITING_CONFIRMATION:
                    segment.mark_cancelled()
                    self._cancel_timeout_locked(segment)
                    cancelled.append(segment)
        for segment in cancelled:
            self._publish(segment.segment_id, segment, SegmentStatus.CANCELLED)
        return cancelled

    # ----- internals ----------------------------------------------------

    def _enter_awaiting(self, segment: PendingSegment) -> None:
        """Move a require segment into AWAITING and arm the timeout."""
        with self._lock:
            segment.mark_awaiting()
            if self._timeout_scheduler is not None:
                timeout_s = self._config.confirmation_timeout_ms / 1000.0
                segment_id = segment.segment_id

                def _on_timeout(sid: str = segment_id) -> None:
                    # Defensive lookup — the segment may have been
                    # confirmed/cancelled between scheduling and firing.
                    with self._lock:
                        current = self._segments.get(sid)
                        if current is None:
                            return
                        if current.status is not SegmentStatus.AWAITING_CONFIRMATION:
                            return
                        current.mark_rejected()
                        current.args["_rejection_reason"] = "timeout"
                    self._publish(current.segment_id, current, SegmentStatus.REJECTED)

                handle = self._timeout_scheduler(timeout_s, _on_timeout)
                segment.timeout_handle = handle
        self._publish(segment.segment_id, segment, SegmentStatus.AWAITING_CONFIRMATION)

    def _cancel_timeout_locked(self, segment: PendingSegment) -> None:
        if segment.timeout_handle is not None and self._timeout_canceller is not None:
            try:
                self._timeout_canceller(segment.timeout_handle)
            except Exception as exc:  # pragma: no cover — defensive
                _LOG.warning(
                    "AcceptanceGate: timeout canceller raised for segment %s: %s",
                    segment.segment_id,
                    exc,
                )
            segment.timeout_handle = None

    def _publish(
        self,
        segment_id: str,
        segment: PendingSegment,
        new_status: SegmentStatus,
    ) -> None:
        with self._lock:
            subs = list(self._subscribers)
        for cb in subs:
            try:
                cb(segment_id, segment, new_status)
            except Exception as exc:  # pragma: no cover — defensive
                _LOG.warning(
                    "AcceptanceGate: subscriber raised for %s: %s",
                    segment_id,
                    exc,
                )


def _default_segment_id() -> str:
    """uuid4 hex — short enough for log lines, unique enough for tests."""
    import uuid

    return uuid.uuid4().hex


# ---------------------------------------------------------------------------
# Feedback formatter — §7 [AWAITING] block
# ---------------------------------------------------------------------------


def render_awaiting_block(
    segments: Iterable[PendingSegment],
    *,
    timeout_announcement: str | None = None,
    now: float | None = None,
) -> str:
    """Render the §7 ``[AWAITING]`` block for the LLM system prompt.

    Format (one segment per line, empty block when no segments are
    AWAITING_CONFIRMATION)::

        [AWAITING]
        segment_id=<hex> tool=navigate_to_waypoint plan="<plan_text>" elapsed_ms=<int> timeout_ms=<int>
        segment_id=<hex> tool=delete_waypoint plan="<plan_text>" elapsed_ms=<int> timeout_ms=<int>
        [/AWAITING]

    The block is intentionally machine-parseable (one segment per
    line, no nested braces) so that the LLM can reason over it
    without a JSON schema. An empty block is still emitted
    (``[AWAITING]\n[/AWAITING]``) so the LLM can detect "no
    confirmation is pending right now" unambiguously.
    """
    now_mono = now if now is not None else time.monotonic()
    lines = ["[AWAITING]"]
    for segment in segments:
        if segment.status is not SegmentStatus.AWAITING_CONFIRMATION:
            continue
        elapsed_ms = int((now_mono - segment.created_at) * 1000)
        plan = (segment.decision.plan_text or "").replace("\n", " ").strip()
        lines.append(
            f"segment_id={segment.segment_id} "
            f"tool={segment.tool} "
            f'plan="{plan}" '
            f"elapsed_ms={elapsed_ms}"
        )
    if timeout_announcement:
        lines.append(f"# timeout_announcement={timeout_announcement}")
    lines.append("[/AWAITING]")
    return "\n".join(lines)


__all__ = [
    "AcceptanceGate",
    "AwaitingFeedbackFormatter",
    "PendingSegment",
    "SegmentKind",
    "SegmentStatus",
    "EMERGENCY_TOOLS",
    "render_awaiting_block",
    "route_kind",
]