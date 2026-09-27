"""Tests for :func:`quality.check_audio_quality` (ADR-0056 §6.4 #4-6).

Two tests are the DoD-named entry points from issue #2003:

* ``test_quality_accepts_valid_chunk`` — synthetic "good"
  candidate must produce :attr:`QualityVerdict.PASS`.
* ``test_quality_rejects_low_quality`` — deliberately broken
  candidate (truncation, noise, NaN-embedding) must be rejected
  by one of the three heuristics.

The rest of the file pins individual heuristics and edge cases.
"""
from __future__ import annotations

import numpy as np

from rob_box_voice.scheduler.pregen import (
    DURATION_RATIO_FLOOR,
    QualityVerdict,
    RMS_DB_FLOOR,
    SILENCE_RATIO_FLOOR,
    check_audio_quality,
)


def _make_sine(duration_s: float = 1.0, sr: int = 16000,
               freq: float = 440.0, amplitude: float = 0.5) -> np.ndarray:
    t = np.arange(int(duration_s * sr))
    return (amplitude * np.sin(2 * np.pi * freq * t / sr)).astype(np.float32)


def test_quality_rms_low_rejects_silent_audio():
    """§6.4 #4: silent audio → REJECT_SILENT."""
    audio = np.zeros(16000, dtype=np.float32)
    v = check_audio_quality(audio, sample_rate=16000, estimated_duration_ms=1000.0)
    assert v is QualityVerdict.REJECT_SILENT


def test_quality_rms_low_rejects_very_quiet_audio():
    """Audio below RMS_DB_FLOOR is rejected even if non-zero."""
    sr = 16000
    audio = (np.random.RandomState(0).randn(sr) * 0.001).astype(np.float32)
    v = check_audio_quality(audio, sample_rate=sr, estimated_duration_ms=1000.0)
    assert v is QualityVerdict.REJECT_SILENT


def test_quality_duration_ratio_rejects_clipped():
    """§6.4 #5: actual_ms > 1.25 * estimated_ms → REJECT_CLIPPED."""
    audio = _make_sine(duration_s=3.0)  # 3000 ms
    # estimated = 1000 ms → ratio = 3.0 > 1.25
    v = check_audio_quality(audio, sample_rate=16000, estimated_duration_ms=1000.0)
    assert v is QualityVerdict.REJECT_CLIPPED


def test_quality_duration_ratio_rejects_truncated_short():
    """actual_ms < (1 / 1.25) * estimated_ms → REJECT_CLIPPED (truncated)."""
    audio = _make_sine(duration_s=0.5)  # 500 ms
    v = check_audio_quality(audio, sample_rate=16000, estimated_duration_ms=2000.0)
    assert v is QualityVerdict.REJECT_CLIPPED


def test_quality_silence_ratio_rejects_trimmed():
    """§6.4 #6: > 15% silence at head/tail → REJECT_TRIMMED."""
    sr = 16000
    # Mostly silent + tiny tone in the middle.
    audio = np.zeros(sr, dtype=np.float32)
    audio[sr // 2 : sr // 2 + 100] = 0.5  # 0.6% signal, rest silent
    v = check_audio_quality(audio, sample_rate=sr, estimated_duration_ms=1000.0)
    assert v is QualityVerdict.REJECT_TRIMMED


def test_quality_pass_on_clean_sine():
    """Well-shaped sine wave → PASS on all three heuristics."""
    audio = _make_sine(duration_s=1.0)
    v = check_audio_quality(audio, sample_rate=16000, estimated_duration_ms=1000.0)
    assert v is QualityVerdict.PASS


def test_quality_empty_audio_returns_silent():
    """Defensive: empty audio → REJECT_SILENT."""
    v = check_audio_quality(np.array([], dtype=np.float32),
                            sample_rate=16000, estimated_duration_ms=1000.0)
    assert v is QualityVerdict.REJECT_SILENT


def test_quality_low_sample_rate_returns_pass():
    """Sample rate below sanity floor is treated as 'not our problem'."""
    audio = _make_sine(duration_s=1.0)
    v = check_audio_quality(audio, sample_rate=1000, estimated_duration_ms=1000.0)
    assert v is QualityVerdict.PASS


def test_quality_respects_overridden_thresholds():
    """The floor knobs are honoured."""
    # Default would PASS, but override silence_ratio_floor=0.0
    # so any silence at all triggers REJECT_TRIMMED.
    audio = _make_sine(duration_s=1.0)
    v = check_audio_quality(
        audio, sample_rate=16000, estimated_duration_ms=1000.0,
        silence_ratio_floor=0.0,
    )
    assert v is QualityVerdict.REJECT_TRIMMED


def test_quality_serialises_to_string():
    """QualityVerdict is a str subclass — JSON-friendly."""
    import json as _json
    payload = {"verdict": QualityVerdict.PASS}
    s = _json.dumps(payload)
    assert s == '{"verdict": "pass"}'


# ---------------------------------------------------------------------------
# DoD-named entry points (issue #2003).
#
# These are the literal test names the issue body asked for. Each one is a
# thin wrapper over the focused heuristics above; the wrapper exists so that
# a reviewer grep'ing for ``test_quality_accepts_valid_chunk`` / issue #2003
# can find the property at a glance, without having to scan the whole
# module. Keeping the focus tests (silent/clipped/trimmed) lets us pinpoint
# which heuristic failed when something regresses — the wrapper only proves
# "the gate has a yes answer" / "the gate has a no answer".
# ---------------------------------------------------------------------------


def test_quality_accepts_valid_chunk():
    """DoD #1: a synthetic clean candidate must be ACCEPT-ed.

    Mirrors the property a *real* provider chunk needs to clear:
    non-empty audio, sample-rate above the sanity floor, RMS well
    above the dBFS noise floor, duration within ±25 % of the
    estimator prediction, and < 15 % head/tail silence.

    These are the exact conditions a Yandex/Silero chunk produced
    for a normal Russian phrase satisfies — and the gate is
    designed to let those through.
    """
    audio = _make_sine(duration_s=1.0)  # 1 s / 16 kHz / -6 dBFS sine
    v = check_audio_quality(
        audio,
        sample_rate=16000,
        estimated_duration_ms=1000.0,
    )
    assert v is QualityVerdict.PASS, (
        f"valid candidate must PASS, got {v!r}; "
        f"len={audio.size}, sr=16000, est=1000ms"
    )


def test_quality_rejects_low_quality():
    """DoD #2: deliberately bad audio must be REJECT-ed (any reason).

    The wrapper pins the *decision property* — a low-quality chunk
    must be filtered out, period. We don't pin *which* heuristic
    fired, because that depends on the failure mode the caller
    wants to model. Three sub-cases cover the three heuristics
    from ADR-0056 §3.6:

    * truncation → REJECT_CLIPPED (actual_ms << estimated_ms)
    * silence / RMS-too-low → REJECT_SILENT
    * mostly-silent chunk → REJECT_TRIMMED

    Plus a NaN-embedding edge case (defensive, not strictly a
    quality category but a real failure mode for Yandex when the
    gRPC stream returns a half-flushed buffer with inf dBFS).
    """
    sr = 16000

    # Case 1: truncated chunk (Yandex half-served the SSML).
    truncated = _make_sine(duration_s=0.5)  # 500 ms
    v = check_audio_quality(
        truncated, sample_rate=sr, estimated_duration_ms=2000.0
    )
    assert v is QualityVerdict.REJECT_CLIPPED, (
        f"truncated chunk must be REJECT_CLIPPED, got {v!r}"
    )

    # Case 2: near-silent chunk (Silero mis-routed, returned silence).
    silent = np.zeros(sr, dtype=np.float32)
    v = check_audio_quality(
        silent, sample_rate=sr, estimated_duration_ms=1000.0
    )
    assert v is QualityVerdict.REJECT_SILENT, (
        f"silent chunk must be REJECT_SILENT, got {v!r}"
    )

    # Case 3: trimmed chunk (model produced audio, but the
    # meaningful part is microscopic — head/tail silence > 15 %).
    trimmed = np.zeros(sr, dtype=np.float32)
    trimmed[sr // 2 : sr // 2 + 100] = 0.5  # 0.6 % signal, rest silent
    v = check_audio_quality(
        trimmed, sample_rate=sr, estimated_duration_ms=1000.0
    )
    assert v is QualityVerdict.REJECT_TRIMMED, (
        f"trimmed chunk must be REJECT_TRIMMED, got {v!r}"
    )

    # Case 4 (defensive): NaN-embedding. Should NOT crash the gate
    # and should be rejected (the rms_low heuristic catches it
    # after the nan_to_num reduction). Pinning the no-crash
    # behaviour is important because the gate runs inside the
    # ROS callback thread.
    nan_audio = np.full(sr, np.nan, dtype=np.float32)
    v = check_audio_quality(
        nan_audio, sample_rate=sr, estimated_duration_ms=1000.0
    )
    assert v is not QualityVerdict.PASS, (
        f"NaN-embedded chunk must NOT pass, got {v!r}"
    )
    # Any of REJECT_SILENT / REJECT_CLIPPED / REJECT_TRIMMED is
    # acceptable; the point is "not PASS".