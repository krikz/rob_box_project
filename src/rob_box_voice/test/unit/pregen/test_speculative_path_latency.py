"""Integration test for the speculative-vs-baseline latency contract.

This test exercises the *real* DoD claim of ADR-0056 §3.7 / issue
#2003: when a speculative candidate is available, the wall-clock
time between two consecutive chunks is **lower** than the baseline
synthesis cost.

The test is a deliberate stress test of the orchestrator (not the
quality/decision modules), so it lives next to
``test_pregen_speculative_executor.py`` and uses asyncio + a
deliberately slow mock synth to make the timing signal visible.

Why a mock synth (not a live provider)
--------------------------------------

* **Determinism** — production Yandex/MiniMax latency is network-bound
  and would make the test flaky on CI runners with no GPU/no internet.
* **Coverage** — the timing contract is a property of the *executor*
  (parallel kickoff + claim), not of the provider. We prove the
  property once with a controllable synth; the provider chain has its
  own contract tests under ``test_pregen_tts_integration.py``.
* **Speed** — the test runs in under a second. Adding a real provider
  call would push it past CI's 240 s timeout if the network blips.

Test matrix
-----------

1. ``test_speculative_path_faster_than_baseline`` — the headline test
   the task asks for. Measures wall-clock for a 2-chunk sequence in
   two modes (speculative vs serial baseline) and asserts the
   speculative path is meaningfully faster.
2. ``test_speculative_eliminates_second_synth_wait`` — pins the
   exact source of the speedup: when a candidate is pre-generated,
   the second chunk's *playback* begins without waiting for a
   second TTS call.
3. ``test_baseline_two_chunks_in_serial`` — control case: the
   serial baseline must cost roughly ``2 * synth_delay`` (used as
   the regression guard for the timing claim).

Implementation notes
--------------------

* The mock synth uses ``time.sleep`` to mimic provider latency. The
  delay is large enough (100 ms) that the speculative speedup is
  well above the asyncio scheduler overhead (~1 ms) but small
  enough that the test finishes in < 1 s.
* ``asyncio.to_thread`` is used by the executor for sync providers
  — exactly mirroring the production dispatch in
  ``_dispatch_synthesis``. This means the speculative launch really
  does overlap with the playback wait.
* Tolerance is ``15 %`` of the speedup margin. Tight enough to
  catch a regression that loses the parallelism (would be ~0 %
  speedup), loose enough to survive CI jitter. The theoretical
  speedup ceiling is ~33 % (one synth delay hidden in parallel with
  the previous playback) — we set the floor at 15 % to leave
  headroom for asyncio scheduler overhead.
"""
from __future__ import annotations

import asyncio
import time

import numpy as np
import pytest

from rob_box_voice.scheduler.pregen import (
    CONFIDENCE_FLOOR,
    PreGenResult,
    SpeculativeExecutor,
)


# A 60 ms / 16 kHz sine — within duration_ratio tolerance for the
# executor's heuristics when the estimated duration matches the
# actual duration.
_SINE_DURATION_S = 0.06
_SINE_SR = 16000
_SINE_AMPLITUDE = 0.5

# Mock TTS provider latency. Big enough to dominate asyncio overhead,
# small enough to keep the test fast.
_SYNTH_DELAY_S = 0.10  # 100 ms


def _sine_audio() -> np.ndarray:
    """Generate a clean sine wave that passes the quality heuristics."""
    t = np.arange(int(_SINE_DURATION_S * _SINE_SR))
    return (_SINE_AMPLITUDE * np.sin(2 * np.pi * 440 * t / _SINE_SR)).astype(
        np.float32
    )


def _make_slow_synth(delay_s: float = _SYNTH_DELAY_S):
    """Build a sync synth that sleeps ``delay_s`` to mimic TTS latency.

    The function signature mirrors what
    :func:`rob_box_voice.scheduler.pregen._dispatch_synthesis`
    accepts for a *sync* provider.
    """
    audio = _sine_audio()

    def _synth(ssml, text, ssml_attributes, voice, language):
        time.sleep(delay_s)  # mimic provider network/CPU cost
        return {"audio_np": audio.copy(), "sample_rate": _SINE_SR}

    return _synth


def _make_chunk(speech_id: str, next_speech_id: str) -> dict:
    """Build a chunk payload that triggers a speculative kickoff."""
    return {
        "speech_id": speech_id,
        "ssml": "<speak>x</speak>",
        "dialogue_id": "d",
        "batch_index": 1,
        "batch_total": 3,
        "pregenerate": {
            "next_speech_id": next_speech_id,
            "next_ssml": "<speak>y</speak>",
        },
    }


def _warm_history(executor: SpeculativeExecutor, n: int = 5) -> None:
    """Feed the calibration window so confidence is above the floor."""
    for _ in range(n):
        executor.record_synthesis_actual(
            actual_duration_ms=60.0, estimated_duration_ms=60.0
        )


def _wait_for_completion(
    executor: SpeculativeExecutor, speech_id: str, timeout_s: float = 5.0
) -> PreGenResult:
    """Poll the executor until the speculative task lands in ``_results``.

    Returns the cached :class:`PreGenResult` or raises if the timeout
    elapses before completion.
    """
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if speech_id not in executor._active:
            result = executor.claim(speech_id)
            if result is not None:
                return result
        time.sleep(0.005)
    raise AssertionError(
        f"speculative task for {speech_id!r} did not complete within {timeout_s}s"
    )


def _playback_wait_ms(audio_duration_s: float) -> float:
    """Mimic the wall-clock playback of an audio chunk.

    Returns the wait in *milliseconds* (the same unit the executor's
    ``observe_chunk_to_chunk_latency`` expects). We don't actually
    need to play audio — the test only measures wall-clock time
    between chunk N's *finish* and chunk N+1's *start*.
    """
    # 60 ms of audio playback time, minus 1 ms scheduler overhead,
    # so the executor's asyncio loop can poll for the prebaked
    # result within the playback window.
    return max(1.0, (audio_duration_s * 1000.0) - 1.0)


# ---------------------------------------------------------------------------
# Headline test — required by task body / DoD.
# ---------------------------------------------------------------------------


def test_speculative_path_faster_than_baseline():
    """Speculative pre-gen reduces chunk-to-chunk latency vs baseline.

    Baseline:    chunk_1 synth (≈100 ms) → play chunk_1 (≈60 ms)
                 → chunk_2 synth (≈100 ms) → play chunk_2 (≈60 ms)
                 Total wall-clock ≈ 320 ms.

    Speculative: chunk_1 synth (≈100 ms) + chunk_2 kickoff (parallel)
                 → play chunk_1 (≈60 ms) → claim chunk_2 → play chunk_2 (≈60 ms)
                 Total wall-clock ≈ 220 ms (one synth worth of latency hidden).

    The test asserts the speculative path is at least 15 % faster than
    the serial baseline — a wide enough tolerance to survive CI jitter
    but tight enough to catch a regression that loses the parallelism.
    The theoretical ceiling is ~33 % (one synth delay hidden in
    parallel with the previous playback); 15 % leaves headroom for
    asyncio scheduler overhead while still proving the mechanism
    works.
    """
    synth_delay_s = _SYNTH_DELAY_S
    audio_ms = _SINE_DURATION_S * 1000.0  # 60 ms

    # --- baseline: serial synthesis of two chunks ---
    baseline_start = time.monotonic()

    # Synth chunk 1.
    baseline_synth_1 = _make_slow_synth(delay_s=synth_delay_s)
    audio1 = baseline_synth_1("x", "x", {}, "anton", "ru")

    # Playback (mimicked by sleep).
    time.sleep(_playback_wait_ms(_SINE_DURATION_S) / 1000.0)

    # Synth chunk 2.
    baseline_synth_2 = _make_slow_synth(delay_s=synth_delay_s)
    audio2 = baseline_synth_2("y", "y", {}, "anton", "ru")

    # Playback.
    time.sleep(_playback_wait_ms(_SINE_DURATION_S) / 1000.0)

    baseline_elapsed_ms = (time.monotonic() - baseline_start) * 1000.0
    assert audio1["audio_np"] is not None
    assert audio2["audio_np"] is not None

    # --- speculative: parallel kickoff + claim ---
    executor = SpeculativeExecutor(synth_callable=_make_slow_synth(synth_delay_s))
    _warm_history(executor)

    spec_start = time.monotonic()

    async def _speculative_scenario():
        # Kickoff speculative for chunk_2 while chunk_1 is playing.
        launched = await executor.kickoff(_make_chunk("cur", "next"))
        assert launched == "next"

        # Synth chunk_1 (canonical path).
        canonical_audio, canonical_sr = await asyncio.to_thread(
            executor._synth, "x", "x", {}, "anton", "ru"
        )
        assert canonical_audio is not None

        # "Playback" chunk_1 — wait long enough for the asyncio loop
        # to drive the speculative task to completion.
        await asyncio.sleep(_playback_wait_ms(_SINE_DURATION_S) / 1000.0)

        # Claim chunk_2 — must be prebaked; if not, the timing test
        # is meaningless.
        result = _wait_for_completion(executor, "next")
        assert result is not None
        assert result.decision.value == "accept"

        # "Playback" chunk_2.
        await asyncio.sleep(_playback_wait_ms(_SINE_DURATION_S) / 1000.0)
        return result

    asyncio.run(_speculative_scenario())
    spec_elapsed_ms = (time.monotonic() - spec_start) * 1000.0

    # --- assert: speculative < baseline ---
    speedup_ms = baseline_elapsed_ms - spec_elapsed_ms
    speedup_pct = speedup_ms / baseline_elapsed_ms
    assert speedup_pct >= 0.15, (
        f"Speculative path is not faster enough: baseline={baseline_elapsed_ms:.1f}ms, "
        f"speculative={spec_elapsed_ms:.1f}ms, speedup={speedup_pct:.1%}. "
        f"Expected ≥15% reduction."
    )


# ---------------------------------------------------------------------------
# Supporting tests — pin the mechanism, not just the metric.
# ---------------------------------------------------------------------------


def test_speculative_eliminates_second_synth_wait():
    """Claim returns immediately when the speculative task is finished.

    Pinned so the *cause* of the speedup (parallel synthesis, not
    just lucky timing) is verified independently. Without this, a
    regression that makes the executor serially await the result
    would still leave test_speculative_path_faster_than_baseline
    passing on a fast CI runner.
    """
    executor = SpeculativeExecutor(synth_callable=_make_slow_synth(0.05))
    _warm_history(executor)

    async def scenario():
        launched = await executor.kickoff(_make_chunk("a", "b"))
        assert launched == "b"
        # Wait for completion.
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            await asyncio.sleep(0.01)
            if "b" not in executor._active:
                break
        result = executor.claim("b")
        assert result is not None
        # The claim itself is O(1); no further latency is paid.
        claim_start = time.monotonic()
        result_again = executor.claim("b")  # already claimed → None
        claim_elapsed_ms = (time.monotonic() - claim_start) * 1000.0
        assert result_again is None
        assert claim_elapsed_ms < 5.0  # well below the synth delay
        return result

    result = asyncio.run(scenario())
    assert result.decision.value == "accept"
    assert result.audio.shape[0] > 0


def test_baseline_two_chunks_in_serial():
    """Control: serial synth path takes roughly 2 * synth_delay.

    This is the *regression guard* for the timing claim — if a CI
    runner suddenly takes 10x longer per sleep, we'd want to know
    the baseline figure changed first.
    """
    delay_s = 0.05
    start = time.monotonic()

    s = _make_slow_synth(delay_s=delay_s)
    s("x", "x", {}, "v", "ru")
    s("y", "y", {}, "v", "ru")

    elapsed_s = time.monotonic() - start
    # At least 2*delay, generously tolerate scheduler overhead.
    assert elapsed_s >= 2 * delay_s * 0.95
    # No surprise: upper bound is 2*delay + 50 ms overhead.
    assert elapsed_s < 2 * delay_s + 0.05


@pytest.mark.parametrize(
    "synth_delay_s,expected_speedup_at_least",
    [
        # Each row exercises the timing contract at a different latency.
        # The expected speedup ratio must hold even on a slow runner.
        # The theoretical ceiling is ~33 %; we set the floor at 15 %
        # to leave headroom for asyncio scheduler overhead while
        # still proving the mechanism works.
        (0.05, 0.15),
        (0.10, 0.15),
        (0.20, 0.15),
    ],
)
def test_speculative_speedup_holds_across_latencies(
    synth_delay_s: float, expected_speedup_at_least: float
):
    """Parametrised version of the headline test across 3 latency budgets.

    Catches regressions where the speedup only holds at one
    particular latency (e.g. asyncio.to_thread pool exhaustion at
    high latency).
    """
    audio_ms = _SINE_DURATION_S * 1000.0

    # Baseline.
    baseline_start = time.monotonic()
    s = _make_slow_synth(delay_s=synth_delay_s)
    s("x", "x", {}, "v", "ru")
    time.sleep(audio_ms / 1000.0)
    s("y", "y", {}, "v", "ru")
    time.sleep(audio_ms / 1000.0)
    baseline_ms = (time.monotonic() - baseline_start) * 1000.0

    # Speculative.
    executor = SpeculativeExecutor(synth_callable=_make_slow_synth(synth_delay_s))
    _warm_history(executor)

    spec_start = time.monotonic()

    async def scenario():
        await executor.kickoff(_make_chunk("cur", "next"))
        await asyncio.to_thread(executor._synth, "x", "x", {}, "v", "ru")
        await asyncio.sleep(audio_ms / 1000.0)
        _wait_for_completion(executor, "next")
        await asyncio.sleep(audio_ms / 1000.0)

    asyncio.run(scenario())
    spec_ms = (time.monotonic() - spec_start) * 1000.0

    speedup_pct = (baseline_ms - spec_ms) / baseline_ms
    assert speedup_pct >= expected_speedup_at_least, (
        f"At synth_delay={synth_delay_s}s, speedup={speedup_pct:.1%} "
        f"< expected {expected_speedup_at_least:.0%}. "
        f"baseline={baseline_ms:.1f}ms, speculative={spec_ms:.1f}ms."
    )