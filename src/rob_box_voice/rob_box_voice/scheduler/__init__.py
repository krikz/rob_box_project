"""Task scheduler (Phase 1 MVP) — voice / music / anim FIFO channels.

This package implements the minimal viable scheduler described in
``docs/design/SCHEDULER_DESIGN.md`` §11.1 (issue #968, Phase 1 — MVP).

Scope (MVP only):

* Three FIFO channels — :class:`~rob_box_harness.core.acceptance.SegmentKind`
  values ``VOICE`` / ``MUSIC`` / ``ANIM``. Each channel owns its own
  asyncio queue and runs its tasks strictly sequentially so two
  TTS requests never collide on the audio device.
* A :class:`TaskScheduler` façade with ``submit`` / ``cancel`` /
  ``wait_all`` / ``channel_status`` methods. No two-tier decision
  logic, no Reflex channel, no SegmentEstimator, no
  :class:`SchedulerEventBus` — those land in later phases.
* Pure data + asyncio, no rclpy. The scheduler is unit-tested
  with synthetic executors so the W7 LLM integration can wire it
  up via a thin adapter.

Out of scope (deliberately):

* Phase 1.5 AcceptanceGate/AWAITING_CONFIRMATION — see
  ``rob_box_harness.core.acceptance``.
* Phase 2 quick-decide L2 and SchedulerEventBus.
* Phase 3 speculative pre-generation.

See :class:`TaskScheduler` for the public entry point.
"""

from __future__ import annotations

from .decision import (
    DecisionCoordinator,
    DecisionPlan,
    HighLevelPlanner,
    LowLevelExecutor,
    PlanExecution,
    PlanStep,
    SchedulerStepExecutor,
    StepExecution,
    StepStatus,
)
from .event_bus import (
    BackpressurePolicy,
    EventBus,
    EventBusClosedError,
    EventBusError,
    EventEnvelope,
    EventQueueFullError,
    EventSubscription,
)
from .reflex import (
    DEFAULT_DEBOUNCE_MS,
    DEFAULT_HISTORY_SIZE,
    ReflexDecision,
    ReflexEvent,
    ReflexKind,
    ReflexLayer,
    ReflexMetrics,
    ReflexPriority,
    command_to_view,
)
from .task_scheduler import (
    ChannelKind,
    ChannelStatus,
    TaskOutcome,
    TaskScheduler,
    TaskStatus,
    SchedulerTask,
    TaskExecutor,
    TaskResult,
    TaskSubmitError,
    TaskNotFoundError,
    ChannelBusyError,
)

__all__ = [
    "DecisionCoordinator",
    "DecisionPlan",
    "HighLevelPlanner",
    "LowLevelExecutor",
    "PlanExecution",
    "PlanStep",
    "SchedulerStepExecutor",
    "StepExecution",
    "StepStatus",
    "BackpressurePolicy",
    "EventBus",
    "EventBusClosedError",
    "EventBusError",
    "EventEnvelope",
    "EventQueueFullError",
    "EventSubscription",
    "ChannelKind",
    "ChannelStatus",
    "TaskOutcome",
    "TaskScheduler",
    "TaskStatus",
    "SchedulerTask",
    "TaskExecutor",
    "TaskResult",
    "TaskSubmitError",
    "TaskNotFoundError",
    "ChannelBusyError",
    # Phase 2.5 — Reflex layer (issue #968 §8.10)
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
