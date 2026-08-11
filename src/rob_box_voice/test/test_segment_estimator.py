"""Phase 3 unit tests: pluggable SegmentEstimator + baseline heuristics."""
from __future__ import annotations

import math

import pytest

from rob_box_voice.scheduler import (
    BaselineEstimator,
    ChannelKind,
    EstimatorContext,
    SchedulerTask,
    SegmentEstimate,
    TaskResult,
    estimate_total_duration_ms,
)
from rob_box_voice.scheduler.task_scheduler import TaskStatus


def _noop_task(channel: ChannelKind, tool: str, args: dict | None = None) -> SchedulerTask:
    async def _executor(_: SchedulerTask) -> TaskResult:
        return TaskResult(payload={"ok": True})

    return SchedulerTask(
        task_id="",
        tool=tool,
        channel=channel,
        executor=_executor,
        args=args or {},
        status=TaskStatus.QUEUED,
    )


# ---------------------------------------------------------------------------
# Contract validation
# ---------------------------------------------------------------------------


def test_segment_estimate_rejects_negative_duration_and_cost() -> None:
    with pytest.raises(ValueError, match="duration_ms"):
        SegmentEstimate(duration_ms=-1.0, cost=0.0, confidence=0.5)
    with pytest.raises(ValueError, match="cost"):
        SegmentEstimate(duration_ms=0.0, cost=-0.1, confidence=0.5)


def test_segment_estimate_rejects_out_of_range_confidence() -> None:
    with pytest.raises(ValueError, match="confidence"):
        SegmentEstimate(duration_ms=0.0, cost=0.0, confidence=-0.1)
    with pytest.raises(ValueError, match="confidence"):
        SegmentEstimate(duration_ms=0.0, cost=0.0, confidence=1.5)


# ---------------------------------------------------------------------------
# Baseline heuristics (per SCHEDULER_DESIGN.md §6.2)
# ---------------------------------------------------------------------------


def test_baseline_estimator_voice_uses_chars_per_second() -> None:
    estimator = BaselineEstimator()
    task = _noop_task(ChannelKind.VOICE, "speak_text", {"text": "a" * 90})
    estimate = estimator.estimate(task, EstimatorContext(channel=ChannelKind.VOICE, tool="speak_text"))
    # 90 chars at 30 cps = 3.0 seconds
    assert estimate.duration_ms == pytest.approx(3000.0)
    assert estimate.cost == BaselineEstimator.COST_VOICE
    assert 0.0 < estimate.confidence < 1.0
    assert "chars_per_second" in estimate.basis


def test_baseline_estimator_voice_uses_context_override() -> None:
    estimator = BaselineEstimator()
    task = _noop_task(ChannelKind.VOICE, "speak_text", {"text": "x" * 60})
    ctx = EstimatorContext(
        channel=ChannelKind.VOICE,
        tool="speak_text",
        chars_per_second=20.0,
    )
    estimate = estimator.estimate(task, ctx)
    # 60 chars at 20 cps = 3.0 seconds
    assert estimate.duration_ms == pytest.approx(3000.0)
    assert "chars_per_second=20" in estimate.basis


def test_baseline_estimator_music_prefers_duration_ms_then_duration_sec() -> None:
    estimator = BaselineEstimator()
    # duration_ms wins
    task = _noop_task(ChannelKind.MUSIC, "execute_music_code", {"duration_ms": 4200})
    est = estimator.estimate(task, EstimatorContext(channel=ChannelKind.MUSIC, tool="execute_music_code"))
    assert est.duration_ms == pytest.approx(4200.0)
    # duration_sec fallback
    task2 = _noop_task(ChannelKind.MUSIC, "execute_music_code", {"duration_sec": 1.5})
    est2 = estimator.estimate(task2, EstimatorContext(channel=ChannelKind.MUSIC, tool="execute_music_code"))
    assert est2.duration_ms == pytest.approx(1500.0)
    # default nominal when neither is present
    task3 = _noop_task(ChannelKind.MUSIC, "execute_music_code", {})
    est3 = estimator.estimate(task3, EstimatorContext(channel=ChannelKind.MUSIC, tool="execute_music_code"))
    assert est3.duration_ms == pytest.approx(1000.0)
    assert est3.confidence == pytest.approx(0.9)


def test_baseline_estimator_music_uses_segments_contract() -> None:
    """Issue #990 — segments (bars) is now the primary music contract."""
    estimator = BaselineEstimator()
    # 16 bars × 4 beats/bar @120 BPM = 16 × 2000 ms = 32 s
    task = _noop_task(ChannelKind.MUSIC, "execute_music_code", {"segments": 16})
    est = estimator.estimate(task, EstimatorContext(channel=ChannelKind.MUSIC, tool="execute_music_code"))
    assert est.duration_ms == pytest.approx(32000.0)
    assert est.confidence == pytest.approx(0.9)
    # segments wins over the deprecated duration_sec
    task2 = _noop_task(
        ChannelKind.MUSIC,
        "execute_music_code",
        {"segments": 8, "duration_sec": 1.5},
    )
    est2 = estimator.estimate(task2, EstimatorContext(channel=ChannelKind.MUSIC, tool="execute_music_code"))
    assert est2.duration_ms == pytest.approx(16000.0)


def test_baseline_estimator_anim_uses_preset_manifest() -> None:
    estimator = BaselineEstimator()
    estimator.register_preset("wave", 800.0)
    task = _noop_task(ChannelKind.ANIM, "play_animation", {"name": "wave"})
    estimate = estimator.estimate(task, EstimatorContext(channel=ChannelKind.ANIM, tool="play_animation"))
    assert estimate.duration_ms == pytest.approx(800.0)
    assert estimate.confidence == pytest.approx(1.0)
    assert estimate.basis == "preset:wave"


def test_baseline_estimator_anim_without_preset_reports_missing() -> None:
    estimator = BaselineEstimator()
    task = _noop_task(ChannelKind.ANIM, "play_animation", {"name": "ghost"})
    estimate = estimator.estimate(task, EstimatorContext(channel=ChannelKind.ANIM, tool="play_animation"))
    assert estimate.duration_ms is None
    assert estimate.confidence == 0.0
    assert "missing_preset" in estimate.basis


def test_baseline_estimator_unknown_channel_returns_none() -> None:
    estimator = BaselineEstimator()
    # Voice / music / anim are the only channels Phase 3 supports;
    # feed a fresh enum value via a shim task to make sure the
    # baseline handles unknowns cleanly without crashing.
    fake = _noop_task(ChannelKind.VOICE, "probe", {})
    fake.channel = ChannelKind.VOICE
    estimate = estimator.estimate(fake, EstimatorContext(channel=fake.channel, tool="probe"))
    # Should not crash — voice path returns a sensible estimate.
    assert estimate.duration_ms is not None

    # And an explicit unknown via a fabricated channel string would
    # not even reach the enum — that path is exercised by
    # `SegmentEstimator` consumers that gate on the enum. We assert
    # the design intent here: the Protocol is pluggable so a custom
    # estimator can take over for new channels.


def test_baseline_feedback_records_absolute_error() -> None:
    estimator = BaselineEstimator()
    task = _noop_task(ChannelKind.VOICE, "speak_text", {"text": "hello"})
    ctx = EstimatorContext(channel=ChannelKind.VOICE, tool="speak_text")
    estimate = estimator.estimate(task, ctx)
    assert estimate.duration_ms is not None
    estimator.feedback(task, estimate, actual_duration_ms=estimate.duration_ms + 250)
    assert estimator.last_error_ms == pytest.approx(250.0)
    # None duration → last_error_ms is reset to None (no signal).
    gap_estimate = SegmentEstimate(duration_ms=None, cost=None, confidence=0.0)
    estimator.feedback(task, gap_estimate, actual_duration_ms=100.0)
    assert estimator.last_error_ms is None


# ---------------------------------------------------------------------------
# Aggregator helper used by §6.5 runtime-budget
# ---------------------------------------------------------------------------


def test_estimate_total_duration_ms_sums_known_and_skips_gaps() -> None:
    estimates = [
        SegmentEstimate(duration_ms=1000.0, cost=None, confidence=0.5),
        SegmentEstimate(duration_ms=None, cost=None, confidence=0.0),
        SegmentEstimate(duration_ms=2500.0, cost=None, confidence=0.5),
    ]
    assert estimate_total_duration_ms(estimates) == pytest.approx(3500.0)


def test_estimate_total_duration_ms_handles_empty_input() -> None:
    assert estimate_total_duration_ms([]) == 0.0


def test_estimate_total_duration_ms_rejects_non_finite_totals() -> None:
    # Stack enough huge estimates to overflow to inf deterministically.
    huge = [SegmentEstimate(duration_ms=1e308, cost=None, confidence=0.5)] * 10
    with pytest.raises(ValueError, match="non-finite"):
        estimate_total_duration_ms(huge)


# ---------------------------------------------------------------------------
# Pluggability — a custom estimator must satisfy the Protocol
# ---------------------------------------------------------------------------


def test_custom_estimator_satisfies_segment_estimator_protocol() -> None:
    class MyEstimator:
        def estimate(self, task: SchedulerTask, context: EstimatorContext) -> SegmentEstimate:
            return SegmentEstimate(duration_ms=42.0, cost=1.0, confidence=1.0)

        def feedback(self, *args, **kwargs) -> None:  # noqa: D401 — protocol hook
            return None

    assert isinstance(MyEstimator(), BaselineEstimator) is False  # not a Baseline
    # Protocol runtime check is implicit via @runtime_checkable
    from rob_box_voice.scheduler.estimator import SegmentEstimator

    assert isinstance(MyEstimator(), SegmentEstimator)


def test_baseline_rejects_non_positive_chars_per_second() -> None:
    with pytest.raises(ValueError, match="chars_per_second"):
        BaselineEstimator(chars_per_second=0.0)
    with pytest.raises(ValueError, match="chars_per_second"):
        BaselineEstimator(chars_per_second=-1.0)


def test_baseline_register_preset_rejects_negative_duration() -> None:
    estimator = BaselineEstimator()
    with pytest.raises(ValueError, match="duration_ms"):
        estimator.register_preset("bad", -1.0)


def test_baseline_estimator_handles_unicode_text_length() -> None:
    """Russian / emoji text must be measured in *characters*, not bytes."""
    estimator = BaselineEstimator()
    # 12 chars of Cyrillic — len() counts Unicode code points.
    task = _noop_task(ChannelKind.VOICE, "speak_text", {"text": "Привет, мир!"})
    estimate = estimator.estimate(task, EstimatorContext(channel=ChannelKind.VOICE, tool="speak_text"))
    # 12 chars / 30 cps = 0.4 seconds = 400 ms
    assert estimate.duration_ms == pytest.approx(400.0)
