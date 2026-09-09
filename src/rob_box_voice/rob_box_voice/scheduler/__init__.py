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

Speculative TTS pre-generation (ADR-0056) lives in the dedicated
:mod:`rob_box_voice.scheduler.pregen` sub-package and is the only
speculation path wired into :mod:`rob_box_voice.tts_node`. The
older ``scheduler.{pre_gen,speculative_executor,decision,
estimator,quality,reflex}`` modules were removed because they were
never wired to a live caller and were covered only by their own
tests — see ADR-0080 §2.8 and ADR-0086. The bounded publish/
subscribe ``EventBus`` (Phase 2, issue #968 §11.6) was removed the
same day for the same reason: its only subscriber was ``reflex``.
``EventEnvelope`` stays — it is also the value type for the
unrelated S10 ``llm_continue_hook`` mechanism (issue #968 §4.5).

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
from .event_bus import EventEnvelope
from .quick_decide import (
    CONFIDENCE_FLOOR,
    DEDUP_WINDOW_S,
    QuickVerdict,
    quick_decide,
)
# ADR-0086 (2026-09-09): the reflex layer, its EventBus cancel bridge,
# and the EventBus pub/sub class itself were removed — the reflex
# module subscribed to a ``TaskScheduler`` instance that never received
# tasks, and once it was gone the bus had zero subscribers left.
# ``EventEnvelope`` stays as a plain value type (see module docstring).
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
    "EventEnvelope",
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
