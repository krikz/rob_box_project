"""Speculative TTS pre-generation for the audio-chunk layer (ADR-0056, issue #2003).

This subpackage implements the **chunk-level** speculative generation
contract — distinct from the existing scheduler-level
``scheduler.pre_gen.SpeculativePreGenerator`` (which deals with
``PreGenCandidate`` / freeze-boundary / ``MERGE`` segments).

Why a separate package
-----------------------

| Concern | ``scheduler.pre_gen`` (existing) | ``scheduler.pregen`` (this pkg) |
|---|---|---|
| Unit of speculation | scheduler task / segment | one audio chunk |
| Trigger | scheduler hint about next segment | LLM-bundled ``pregenerate`` field in /voice/dialogue/response |
| Cancellation contract | keep cached results on cancel | **clear** cache on cancel (dialogue_id change) |
| Quality gate | EstimatorQualityTracker (EMA/MAPE) | Audio heuristics (rms_low / duration_ratio / silence_ratio) |
| Decides whether to fire | freeze-boundary policy | CONFIDENCE_FLOOR (default 0.6) |

Trying to shoehorn the chunk-level workflow into
``SpeculativePreGenerator`` would force a "thin adapter" which
mutates one interface to look like another — a known bug-magnet
(see the failed attempt in commit ``a1b823ce2`` and the explicit
rejection in ADR-0056 §4).

Public surface
--------------

* :class:`PreGenTask`, :class:`PreGenResult`, :class:`QualityVerdict`,
  :class:`Decision`, :class:`SegmentEstimate` — the data contract
  (re-exported from sub-modules for convenience).
* :func:`pre_gen.build_pregen_task` — produces a :class:`PreGenTask`
  from the current chunk + dialogue context, or ``None`` when
  speculation is impossible.
* :func:`estimator.estimate_confidence` — gates whether to even
  start the speculative synthesis.
* :func:`quality.check_audio_quality` — three audio heuristics
  (``rms_low`` / ``duration_ratio`` / ``silence_ratio``).
* :func:`decision.decide` — final ACCEPT/REJECT rule.
* :class:`speculative_executor.SpeculativeExecutor` — orchestrator
  that owns the asyncio lifecycle of in-flight speculative tasks.
* :class:`speculative_executor.PreGenMetrics` — counters for the
  ``/voice/tts/metrics`` topic.

All five modules are intentionally pure / no-rclpy / no-async-loop
(except ``speculative_executor`` which uses ``asyncio`` to schedule
in-flight tasks). They are unit-testable in isolation.
"""

from __future__ import annotations

from .pre_gen import (
    DEFAULT_NEXT_PRIORITY,
    PreGenTask,
    build_pregen_task,
)
from .estimator import (
    CONFIDENCE_FLOOR,
    SegmentEstimate,
    estimate_confidence,
)
from .quality import (
    DURATION_RATIO_FLOOR,
    RMS_DB_FLOOR,
    SILENCE_RATIO_FLOOR,
    QualityVerdict,
    check_audio_quality,
)
from .decision import (
    ACCEPT_BASIS_DEFAULT,
    Decision,
    decide,
)
from .speculative_executor import (
    PreGenMetrics,
    PreGenResult,
    SpeculativeExecutor,
)

__all__ = [
    # data classes
    "SegmentEstimate",
    "PreGenTask",
    "PreGenResult",
    "QualityVerdict",
    "Decision",
    # pure functions
    "build_pregen_task",
    "estimate_confidence",
    "check_audio_quality",
    "decide",
    # orchestrator
    "SpeculativeExecutor",
    "PreGenMetrics",
    # constants
    "CONFIDENCE_FLOOR",
    "DURATION_RATIO_FLOOR",
    "RMS_DB_FLOOR",
    "SILENCE_RATIO_FLOOR",
    "ACCEPT_BASIS_DEFAULT",
    "DEFAULT_NEXT_PRIORITY",
]