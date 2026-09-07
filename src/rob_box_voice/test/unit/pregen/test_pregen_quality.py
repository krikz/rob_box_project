"""Tests for :func:`quality.check_audio_quality` (ADR-0056 §6.4 #4-6)."""
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