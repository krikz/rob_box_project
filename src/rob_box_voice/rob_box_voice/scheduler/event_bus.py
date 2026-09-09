"""Event envelope value type shared by scheduler components.

ADR-0086 (2026-09-09): the bounded publish/subscribe ``EventBus`` +
``EventSubscription`` pair that used to live in this module is removed —
its only subscriber was ``ReflexLayer`` (deleted the same day, issue
#2259), and ``TaskScheduler``'s cancel-preemption publish had zero other
subscribers. See ADR-0086 §1.1(а)/(б) for the raw findings.

``EventEnvelope`` itself stays: it is also the value type for the
unrelated S10 ``llm_continue_hook`` auto-trigger mechanism (scheduler-
segments-merge, issue #968 §4.5) — ``TaskScheduler.notify_event`` /
``_maybe_trigger_continue`` / ``LlmContinueContext.events`` construct
and consume plain ``EventEnvelope`` values directly, never through the
now-removed pub/sub bus.
"""
from __future__ import annotations

import time
import uuid
from dataclasses import dataclass, field
from typing import Any


@dataclass(frozen=True)
class EventEnvelope:
    """Immutable message exchanged by scheduler components.

    ``priority`` — S10 (scheduler-segments-merge, issue #968, §4.5):
    used by :meth:`TaskScheduler.notify_event`/``_maybe_trigger_continue``
    to decide whether an event counts toward the auto-trigger's
    "unapplied event" condition. ``"low"`` events (noise/IGNORE-level
    signals) never count; anything else (default ``"normal"``) does.
    Defaults to ``"normal"`` so every existing caller is unaffected.
    """

    topic: str
    payload: Any
    event_id: str = field(default_factory=lambda: uuid.uuid4().hex)
    created_at: float = field(default_factory=time.monotonic)
    correlation_id: str | None = None
    priority: str = "normal"

    def __post_init__(self) -> None:
        if not self.topic:
            raise ValueError("event topic must not be empty")
