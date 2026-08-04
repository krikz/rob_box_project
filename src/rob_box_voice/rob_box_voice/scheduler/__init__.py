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
* :class:`DecisionCoordinator` / :class:`DecisionPlan` /
  :class:`PlanStep` / :class:`SchedulerStepExecutor` — two-tier
  planner/executor contract that wires through the MVP scheduler.

Phase 3 (estimators + speculative pre-generation):

* :class:`SegmentEstimator` Protocol + :class:`BaselineEstimator`
  — pluggable three-axis prediction (duration / cost / confidence).
* :class:`EstimatorQualityTracker` — EMA error, MAPE, calibration
  bins; consumed by the LLM feedback loop (§7.1).
* :class:`SpeculativePreGenerator` — runtime-budget pre-gen with
  cancel semantics tied to :class:`SchedulerTask` lifecycle.

Pure data + asyncio, no rclpy. Unit tests build synthetic
executors so the LLM integration can wire the package via a
thin adapter.

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
from .estimator import (
    BaselineEstimator,
    EstimatorContext,
    SegmentEstimate,
    SegmentEstimator,
    estimate_total_duration_ms,
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
from .pre_gen import (
    PreGenCandidate,
    PreGenCancelledError,
    PreGenFactory,
    PreGenPlan,
    SpeculativePreGenerator,
)
from .quality import (
    CalibrationBin,
    EstimatorQualityTracker,
    EstimatorSample,
    PredictionOutcome,
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
    "BaselineEstimator",
    "EstimatorContext",
    "SegmentEstimate",
    "SegmentEstimator",
    "estimate_total_duration_ms",
    "BackpressurePolicy",
    "EventBus",
    "EventBusClosedError",
    "EventBusError",
    "EventEnvelope",
    "EventQueueFullError",
    "EventSubscription",
    "PreGenCandidate",
    "PreGenCancelledError",
    "PreGenFactory",
    "PreGenPlan",
    "SpeculativePreGenerator",
    "CalibrationBin",
    "EstimatorQualityTracker",
    "EstimatorSample",
    "PredictionOutcome",
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
]