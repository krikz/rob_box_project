"""Tests for :class:`SpeculativeExecutor` (ADR-0056 §6.4 #8-9)."""
from __future__ import annotations

import asyncio

import numpy as np
import pytest

from rob_box_voice.scheduler.pregen import (
    CONFIDENCE_FLOOR,
    Decision,
    PreGenResult,
    SpeculativeExecutor,
)


def _make_synth(audio: np.ndarray, sample_rate: int = 16000):
    """Build a sync synth callable that returns ``audio`` after a tiny delay."""
    def _synth(ssml, text, ssml_attributes, voice, language):
        return {"audio_np": audio.copy(), "sample_rate": sample_rate}
    return _synth


def _sine(duration_s: float = 1.0, sr: int = 16000) -> np.ndarray:
    t = np.arange(int(duration_s * sr))
    return (0.5 * np.sin(2 * np.pi * 440 * t / sr)).astype(np.float32)


def _run(coro):
    """Drive an async coroutine from a sync test."""
    return asyncio.run(coro)


def _warm_history(executor: SpeculativeExecutor, n: int = 5) -> None:
    """Feed the executor's calibration window with stable samples."""
    for _ in range(n):
        executor.record_synthesis_actual(
            actual_duration_ms=1000.0, estimated_duration_ms=1000.0
        )


def _chunk_with_pregen(next_speech_id: str = "next") -> dict:
    return {
        "speech_id": "cur",
        "ssml": "<speak>x</speak>",
        "dialogue_id": "d",
        "batch_index": 1,
        "batch_total": 3,
        "pregenerate": {
            "next_speech_id": next_speech_id,
            "next_ssml": "<speak>y</speak>",
        },
    }


def test_executor_kickoff_then_claim_roundtrip():
    """§6.4 #8: kickoff → wait → claim returns audio."""
    # 60 ms of audio at 16 kHz = 960 samples. Estimated = 60 ms / char * 1
    # char ("y") = 60 ms → ratio = 1.0 → PASS on duration_ratio.
    audio = _sine(duration_s=0.06)
    executor = SpeculativeExecutor(synth_callable=_make_synth(audio))
    _warm_history(executor)

    async def scenario():
        speech_id = await executor.kickoff(_chunk_with_pregen())
        assert speech_id == "next"
        # Wait for the asyncio.Task to finish.
        # The executor stashes it in _active; we spin until done.
        for _ in range(50):
            await asyncio.sleep(0.02)
            if "next" not in executor._active:
                break
        assert "next" not in executor._active
        result = executor.claim("next")
        assert result is not None
        assert isinstance(result, PreGenResult)
        assert result.speech_id == "next"
        assert result.decision is Decision.ACCEPT
        assert result.audio.shape == audio.shape
        # Sanity: the audio matches what synth returned.
        assert np.allclose(result.audio, audio)

    _run(scenario())
    metrics = executor.metrics
    assert metrics.kickoffs_total == 1
    assert metrics.pregens_scheduled == 1
    assert metrics.pregens_completed == 1


def test_executor_cancel_clears_results():
    """§6.4 #9: cancel after kickoff → claim returns None."""
    audio = _sine(duration_s=0.06)
    executor = SpeculativeExecutor(synth_callable=_make_synth(audio))
    _warm_history(executor)

    async def scenario():
        await executor.kickoff(_chunk_with_pregen())
        # Force-finish the task before cancelling.
        for _ in range(50):
            await asyncio.sleep(0.02)
            if "next" not in executor._active:
                break
        assert "next" in executor._results  # claimable pre-cancel
        count = await executor.cancel(reason="test_cancel")
        # Already-completed task isn't cancelable but results are cleared.
        assert count == 0  # no in-flight at this point
        assert "next" not in executor._results
        assert executor.is_cancelled()

    _run(scenario())


def test_executor_low_confidence_skips_kickoff():
    """Cold start → confidence < floor → no asyncio.Task created."""
    executor = SpeculativeExecutor(synth_callable=_make_synth(_sine()))

    async def scenario():
        speech_id = await executor.kickoff(_chunk_with_pregen())
        assert speech_id is None
        # Nothing was launched.
        assert executor._active == {}
        assert executor.metrics.pregens_rejected_confidence == 1
        assert executor.metrics.pregens_scheduled == 0

    _run(scenario())


def test_executor_quality_gate_rejects_silent_audio():
    """If the synth returns silence, the executor rejects it."""
    silent = np.zeros(16000, dtype=np.float32)
    executor = SpeculativeExecutor(synth_callable=_make_synth(silent))
    _warm_history(executor)

    async def scenario():
        await executor.kickoff(_chunk_with_pregen())
        for _ in range(50):
            await asyncio.sleep(0.02)
            if "next" not in executor._active:
                break
        # No cached result.
        assert executor.claim("next") is None
        assert executor.metrics.pregens_rejected_quality == 1

    _run(scenario())


def test_executor_records_actual_durations():
    """``record_synthesis_actual`` trims to history_window."""
    executor = SpeculativeExecutor(
        synth_callable=_make_synth(_sine()), history_window=3
    )
    for _ in range(10):
        executor.record_synthesis_actual(
            actual_duration_ms=100.0, estimated_duration_ms=100.0
        )
    # Trimmed to 3.
    assert len(executor._actual_durations_ms) == 3


def test_executor_ignores_nonpositive_durations():
    """Zero / negative samples are filtered out."""
    executor = SpeculativeExecutor(synth_callable=_make_synth(_sine()))
    executor.record_synthesis_actual(actual_duration_ms=0, estimated_duration_ms=100)
    executor.record_synthesis_actual(actual_duration_ms=100, estimated_duration_ms=0)
    assert executor._actual_durations_ms == []


def test_executor_kickoff_is_idempotent_under_resubmit():
    """Two kicks for the same speech_id → only one asyncio.Task."""
    executor = SpeculativeExecutor(synth_callable=_make_synth(_sine(duration_s=0.06)))
    _warm_history(executor)

    async def scenario():
        s1 = await executor.kickoff(_chunk_with_pregen())
        s2 = await executor.kickoff(_chunk_with_pregen())
        assert s1 == s2 == "next"
        # _active has only one task for that id.
        assert len(executor._active) == 1
        await executor.cancel(reason="dup")

    _run(scenario())


def test_executor_snapshot_reports_state():
    executor = SpeculativeExecutor(synth_callable=_make_synth(_sine(duration_s=0.06)))
    snap = executor.snapshot()
    assert snap["active"] == 0
    assert snap["cached"] == 0
    assert snap["cancelled"] is False
    assert "metrics" in snap


def test_executor_metrics_as_dict_contains_mean_latency():
    executor = SpeculativeExecutor(synth_callable=_make_synth(_sine(duration_s=0.06)))
    executor.observe_chunk_to_chunk_latency(150.0)
    executor.observe_chunk_to_chunk_latency(200.0)
    d = executor.metrics.as_dict()
    assert d["latency_chunk_to_chunk_ms_count"] == 2
    assert d["latency_chunk_to_chunk_ms_mean"] == pytest.approx(175.0)


def test_executor_metrics_zero_observations_means_zero_latency():
    executor = SpeculativeExecutor(synth_callable=_make_synth(_sine(duration_s=0.06)))
    d = executor.metrics.as_dict()
    assert d["latency_chunk_to_chunk_ms_mean"] == 0.0
    assert d["latency_chunk_to_chunk_ms_count"] == 0


def test_executor_rejects_none_synth_callable():
    with pytest.raises(ValueError):
        SpeculativeExecutor(synth_callable=None)


def test_executor_handles_synth_exception_gracefully():
    """If the synth raises, kickoff does not propagate."""
    def _bad_synth(*args, **kwargs):
        raise RuntimeError("synth failed")

    executor = SpeculativeExecutor(synth_callable=_bad_synth)
    _warm_history(executor)

    async def scenario():
        # Should not crash the test.
        await executor.kickoff(_chunk_with_pregen())
        for _ in range(50):
            await asyncio.sleep(0.02)
            if "next" not in executor._active:
                break
        # Nothing cached, quality-rejected counter incremented.
        assert executor.claim("next") is None
        assert executor.metrics.pregens_rejected_quality == 1

    _run(scenario())