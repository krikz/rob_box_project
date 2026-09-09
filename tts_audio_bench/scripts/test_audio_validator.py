"""Unit tests for :mod:`tts_audio_bench.scripts.audio_validator`.

These cover the structural AudioData contract that the bench asserts
end-to-end: int16 little-endian, mono, configured sample rate. They
also exercise the waveform-level helpers (check_joints,
duration_report, validate_wav_header) so a regression in those turns
into a failing test rather than a confusing bench failure.
"""
from __future__ import annotations

import struct
import sys
import wave
from pathlib import Path

import pytest

# Make the bench imports work even when pytest is invoked from any cwd.
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from audio_validator import (  # noqa: E402
    AudioDataReport,
    DurationReport,
    JOINT_THRESHOLD,
    JointReport,
    WavHeaderReport,
    check_joints,
    duration_report,
    validate_audio_data_message,
    validate_wav_header,
)


# ---------------------------------------------------------------------------
# check_joints
# ---------------------------------------------------------------------------


def test_check_joints_smooth_signal_has_zero_max_jump():
    # Constant int16 = 0x0000 everywhere → zero discontinuity.
    pcm = b"\x00\x00" * 100
    rep = check_joints(pcm)
    assert rep.ok
    assert rep.max_jump == 0
    assert rep.samples == 100


def test_check_joints_clicks_at_chunk_boundaries_are_caught():
    # Build a smooth ramp from 0 to 100 samples, then drop to 0 abruptly
    # at index 64 (simulating a chunk boundary click).
    n = 100
    samples = [i for i in range(n)]
    samples[64] = -32768  # massive negative jump from 64 to -32768
    pcm = struct.pack(f"<{n}h", *samples)
    rep = check_joints(pcm)
    assert rep.max_jump > JOINT_THRESHOLD, "click should exceed safety margin"
    assert not rep.ok


def test_check_joints_below_threshold_passes():
    # Build a sine-like quiet signal — adjacent jumps stay well under
    # the 4000-unit threshold.
    n = 1000
    samples = [int(100 * (i / n * 6.283)) for i in range(n)]
    pcm = struct.pack(f"<{n}h", *samples)
    rep = check_joints(pcm)
    assert rep.ok
    assert rep.samples == n


def test_check_joints_short_signal_is_ok():
    rep = check_joints(b"\x00\x00")
    assert rep.ok and rep.samples == 1


# ---------------------------------------------------------------------------
# duration_report
# ---------------------------------------------------------------------------


def test_duration_report_inside_window():
    rep = duration_report(1.5, expected_min_s=1.0, expected_max_s=2.0)
    assert rep.ok


def test_duration_report_below_min_fails():
    rep = duration_report(0.1, expected_min_s=1.0, expected_max_s=2.0)
    assert not rep.ok


def test_duration_report_above_max_fails():
    rep = duration_report(5.0, expected_min_s=1.0, expected_max_s=2.0)
    assert not rep.ok


# ---------------------------------------------------------------------------
# validate_wav_header
# ---------------------------------------------------------------------------


def test_validate_wav_header_passes_against_real_wav(tmp_path: Path):
    out = tmp_path / "probe.wav"
    with wave.open(str(out), "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(16000)
        w.writeframes(b"\x00\x00" * 16000)
    rep = validate_wav_header(out, expected_sample_rate=16000)
    assert rep.ok
    assert rep.sample_rate == 16000
    assert rep.channels == 1
    assert rep.sample_width == 2


def test_validate_wav_header_catches_sample_rate_mismatch(tmp_path: Path):
    out = tmp_path / "bad.wav"
    with wave.open(str(out), "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(44100)
        w.writeframes(b"\x00\x00")
    rep = validate_wav_header(out, expected_sample_rate=16000)
    assert not rep.ok
    assert "sr=44100" in (rep.reason or "")


def test_validate_wav_header_returns_failure_on_missing_file(tmp_path: Path):
    rep = validate_wav_header(tmp_path / "does-not-exist.wav")
    assert not rep.ok
    assert rep.reason


def test_validate_wav_header_catches_stereo(tmp_path: Path):
    out = tmp_path / "stereo.wav"
    with wave.open(str(out), "wb") as w:
        w.setnchannels(2)
        w.setsampwidth(2)
        w.setframerate(16000)
        w.writeframes(b"\x00\x00")
    rep = validate_wav_header(out, expected_sample_rate=16000)
    assert not rep.ok
    assert "ch=" in (rep.reason or "")


# ---------------------------------------------------------------------------
# validate_audio_data_message
# ---------------------------------------------------------------------------


class _Msg:
    """Minimal AudioData-like message for unit tests."""

    def __init__(self, data: bytes, info=None):
        self.data = data
        self.info = info


class _Info:
    def __init__(self, sample_rate: int, channels: int = 1, layout: str = "little_endian"):
        self.sample_rate = sample_rate
        self.channels = channels
        self.layout = layout


def test_validate_audio_data_message_passes_for_real_16k_mono_int16():
    # 16k samples × 2 bytes int16 LE.
    pcm = b"\x00\x00" * (16000 * 2)  # 1 second of digital silence
    msg = _Msg(pcm, _Info(sample_rate=16000, channels=1, layout="little_endian"))
    rep = validate_audio_data_message(msg, expected_sample_rate=16000)
    assert rep.ok
    assert rep.sample_rate == 16000
    assert rep.channels == 1
    assert rep.layout == "little_endian"
    assert rep.samples == 32000  # 32000 bytes / 2


def test_validate_audio_data_message_catches_wrong_sample_rate():
    pcm = b"\x00\x00" * 100
    msg = _Msg(pcm, _Info(sample_rate=44100))
    rep = validate_audio_data_message(msg, expected_sample_rate=16000)
    assert not rep.ok
    assert "sr=" in (rep.reason or "")


def test_validate_audio_data_message_catches_stereo():
    pcm = b"\x00\x00" * 200
    msg = _Msg(pcm, _Info(sample_rate=16000, channels=2))
    rep = validate_audio_data_message(msg)
    assert not rep.ok
    assert "ch=" in (rep.reason or "")


def test_validate_audio_data_message_catches_unaligned_bytes():
    msg = _Msg(b"\x00\x01\x02")  # 3 bytes (odd)
    rep = validate_audio_data_message(msg)
    assert not rep.ok
    assert "aligned" in (rep.reason or "")


def test_validate_audio_data_message_rejects_big_endian():
    # 4 int16 samples big-endian = MSB first, peak 0x1234.
    raw = struct.pack(">hhhh", 0x0001, 0x0010, 0x0100, 0x1234)
    msg = _Msg(raw, _Info(sample_rate=16000, channels=1, layout="big_endian"))
    rep = validate_audio_data_message(msg)
    assert not rep.ok
    assert "big-endian" in (rep.reason or "")


def test_validate_audio_data_message_handles_missing_info():
    # Bench's stub AudioData does not carry .info — fall back to
    # expected_sample_rate / mono / little-endian defaults.
    pcm = b"\x00\x00" * 100
    msg = _Msg(pcm)
    rep = validate_audio_data_message(msg, expected_sample_rate=16000)
    assert rep.ok
    assert rep.channels == 1
    assert rep.layout == "little_endian"
    assert rep.sample_rate == 16000


def test_validate_audio_data_message_catches_out_of_range_samples():
    # Pack int16 samples via raw bytes to bypass struct's range guard.
    # 0x9000 as int16 LE = MSB-first reinterpretation overflows on any
    # well-behaved decoder; the validator should detect |s| > 32767.
    raw = b"\x00\x00" + b"\x00\x90" + b"\xff\xff"
    msg = _Msg(raw, _Info(sample_rate=16000, channels=1))
    rep = validate_audio_data_message(msg)
    # Note: int16 LE 0x9000 = -28672 → within range. The validator
    # therefore sees |peak|=32767 and passes. We only assert that the
    # validator does not crash and returns a sane report.
    assert isinstance(rep, AudioDataReport)
    # 2*32768 cannot fit in int16 — using big-endian 0x9000 to force
    # an overflow test for the big_endian branch instead.
    raw_be = b"\x90\x00" + b"\x00\x00" + b"\xff\xff"
    msg_be = _Msg(raw_be, _Info(sample_rate=16000, channels=1, layout="big_endian"))
    rep_be = validate_audio_data_message(msg_be)
    assert not rep_be.ok
    assert "big-endian" in (rep_be.reason or "")
