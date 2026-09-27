"""Unit tests for :mod:`rob_box_voice.scheduler.pregen` (ADR-0056, issue #2003).

These tests cover the *pure* modules of the chunk-level speculative
pipeline (no rclpy, no asyncio orchestration) plus the
:class:`SpeculativeExecutor` orchestrator (async-aware).

The legacy scheduler-level :mod:`rob_box_voice.scheduler.pre_gen`
has its own tests under :mod:`rob_box_voice.test.test_speculative_pre_gen`
— they are intentionally separate because the two modules work on
different abstraction layers (chunk vs segment).

Test matrix (cross-references ADR-0056 §6.4):

* ``test_build_pregen_task_none_when_no_next`` — §6.4 #1
* ``test_build_pregen_task_returns_when_next_available`` — §6.4 #2
* ``test_confidence_floor_filter`` — §6.4 #3
* ``test_rms_low_rejects`` — §6.4 #4
* ``test_duration_ratio_rejects_clipped`` — §6.4 #5
* ``test_silence_ratio_rejects_trimmed`` — §6.4 #6
* ``test_decide_accept_only_on_pass_and_floor`` — §6.4 #7
* ``test_kickoff_claim_roundtrip`` — §6.4 #8
* ``test_cancel_clears_results`` — §6.4 #9
* ``test_prebaked_audio_kwarg_skips_synth_chain`` — §6.4 #10
"""

from __future__ import annotations

import asyncio
import time

import numpy as np
import pytest

from rob_box_voice.scheduler.pregen import (
    ACCEPT_BASIS_DEFAULT,
    CONFIDENCE_FLOOR,
    DURATION_RATIO_FLOOR,
    Decision,
    PreGenResult,
    PreGenTask,
    QualityVerdict,
    RMS_DB_FLOOR,
    SILENCE_RATIO_FLOOR,
    SegmentEstimate,
    SpeculativeExecutor,
    build_pregen_task,
    check_audio_quality,
    decide,
    estimate_confidence,
)

# Bring in the constants that are not in the public __all__.
from rob_box_voice.scheduler.pregen.quality import (
    _rms_dbfs,
    _silence_ratio_head_tail,
)
from rob_box_voice.scheduler.pregen.estimator import (
    _HISTORY_WINDOW,
    _CALIBRATION_GOOD_CV,
    _CALIBRATION_BAD_CV,
)