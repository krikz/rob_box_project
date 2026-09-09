"""Task scheduler (Phase 1 + 2 + 3) — voice / music / anim FIFO channels.

This package implements the scheduler described in
``docs/design/SCHEDULER_DESIGN.md`` §11.1 (issue #968). Each
phase lands as a separate PR; the package surface grows without
breaking the MVP contract.

Phase 1 MVP (``TaskScheduler``):

* Three FIFO channels — ``VOICE`` / ``MUSIC`` / ``ANIM``.
  Each channel owns its own asyncio queue and runs its tasks
  strictly sequentially so two TTS requests never collide on
  the audio device.
* A :class:`TaskScheduler` façade with ``submit`` / ``cancel`` /
  ``wait_all`` / ``channel_status`` methods.

Phase 2 (quick-decide + EventBus):

* :class:`EventBus` — bounded publish/subscribe bus with explicit
  backpressure.

Speculative TTS pre-generation (ADR-0056) lives in the dedicated
:mod:`rob_box_voice.scheduler.pregen` sub-package and is the only
speculation path wired into :mod:`rob_box_voice.tts_node`. The
older ``scheduler.{pre_gen,speculative_executor,decision,
estimator,quality,reflex}`` modules were removed because they were
never wired to a live caller and were covered only by their own
tests — see ADR-0080 §2.8 and ADR-0086.

Pure data + asyncio, no rclpy. Unit tests build synthetic
executors so the LLM integration can wire the package via a
thin adapter.

See :class:`TaskScheduler` for the public entry point.
"""

from __future__ import annotations

from .delta import (
    DeltaOp,
    DeltaOpKind,
    TaskDelta,
    append,
    drop,
    replace,
    rewrite,
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
from .quick_decide import (
    CONFIDENCE_FLOOR,
    DEDUP_WINDOW_S,
    QuickVerdict,
    quick_decide,
)
# ADR-0086 (2026-09-09): the reflex layer and its EventBus cancel bridge
# were removed — the module subscribed to a ``TaskScheduler`` instance
# that never received tasks, so the bridge had no observable effect.
# ``EventBus`` and the ``scheduler.cancel`` envelope stay inside the
# scheduler package (``task_scheduler.py``, ``event_bus.py``) as the
# internal observability channel for cancel-preemption.
from .task_scheduler import (
    ChannelKind,
    ChannelStatus,
    LlmContinueContext,
    LlmContinueHook,
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
    "DeltaOp",
    "DeltaOpKind",
    "TaskDelta",
    "append",
    "drop",
    "replace",
    "rewrite",
    "BackpressurePolicy",
    "EventBus",
    "EventBusClosedError",
    "EventBusError",
    "EventEnvelope",
    "EventQueueFullError",
    "EventSubscription",
    "CONFIDENCE_FLOOR",
    "DEDUP_WINDOW_S",
    "QuickVerdict",
    "quick_decide",
    "ChannelKind",
    "ChannelStatus",
    "LlmContinueContext",
    "LlmContinueHook",
    "TaskOutcome",
    "TaskScheduler",
    "TaskStatus",
    "SchedulerTask",
    "TaskExecutor",
    "TaskResult",
    "TaskSubmitError",
    "TaskNotFoundError",
    "ChannelBusyError",
]
