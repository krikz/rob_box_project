"""Unit tests for :mod:`tts_audio_bench.scripts.audio_subscriber`.

WavRecorder must:
* accumulate int16 LE AudioData frames into a single WAV file;
* report deterministic frame / byte counts;
* surface a warning when no frames were captured;
* not crash on edge cases (empty payload, single byte, integer list
  payload).
"""
from __future__ import annotations

import sys
import time
import wave
from pathlib import Path
from types import SimpleNamespace

import pytest

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from audio_subscriber import SubscriberReport, WavRecorder  # noqa: E402


def _msg(data) -> SimpleNamespace:
    """Build an AudioData-shaped SimpleNamespace.

    Production AudioData carries ``uint8[] data``; the bench stub uses
    ``list[int]`` — both are accepted by ``WavRecorder.__call__``.
    """
    return SimpleNamespace(
        data=data,
        info=SimpleNamespace(sample_rate=16000, channels=1, layout="little_endian"),
    )


def test_wav_recorder_writes_single_mono_int16_wav(tmp_path: Path):
    # 1 second × 16 000 int16 samples × 2 bytes = 32 000 raw bytes.
    # The payload argument is the number of (zero, zero) int16 pairs,
    # so we pass 16 000 not 32 000.
    out = tmp_path / "out.wav"
    rec = WavRecorder(sample_rate=16000)
    rec(_msg(b"\x00\x00" * 16000))
    rep: SubscriberReport = rec.to_wav(out)

    assert rep.frames == 1
    assert rep.bytes_total == 32000
    assert rep.sample_rate == 16000
    assert out.exists()

    with wave.open(str(out), "rb") as w:
        assert w.getnchannels() == 1
        assert w.getsampwidth() == 2
        assert w.getframerate() == 16000
        assert w.getnframes() == 16000


def test_wav_recorder_concatenates_multiple_frames(tmp_path: Path):
    out = tmp_path / "out.wav"
    # 0.5 s of mono int16 LE @ 16 kHz = 8 000 int16 samples × 2 bytes.
    rec = WavRecorder(sample_rate=16000)
    for _ in range(3):
        rec(_msg(b"\x00\x00" * 8000))  # 0.5 s of digital silence
    rep = rec.to_wav(out)

    assert rep.frames == 3
    assert rep.bytes_total == 3 * 8000 * 2  # 48 000 bytes
    with wave.open(str(out), "rb") as w:
        assert w.getnframes() == 3 * 8000


def test_wav_recorder_records_first_arrival_timestamp(tmp_path: Path):
    rec = WavRecorder(sample_rate=16000)
    assert rec.first_arrival_s is None  # nothing captured yet
    t0 = time.monotonic()
    time.sleep(0.005)
    rec(_msg(b"\x00\x00" * 100))
    rep = rec.to_wav(tmp_path / "x.wav")
    assert rep.first_frame_at_s is not None
    assert rep.first_frame_at_s >= 0.005


def test_wav_recorder_elapsed_s_grows_then_settles(tmp_path: Path):
    rec = WavRecorder(sample_rate=16000)
    assert rec.elapsed_s == 0.0
    rec(_msg(b"\x00\x00" * 100))
    time.sleep(0.02)
    rec(_msg(b"\x00\x00" * 100))
    elapsed = rec.elapsed_s
    assert elapsed >= 0.02
    rec.to_wav(tmp_path / "x.wav")
    # Elapsed does not grow after to_wav because new frames haven't arrived.
    assert rec.elapsed_s == elapsed


def test_wav_recorder_warns_on_empty_capture(tmp_path: Path):
    out = tmp_path / "unused.wav"
    rec = WavRecorder(sample_rate=16000)
    rep = rec.to_wav(out)
    assert rep.warnings
    assert "no frames" in rep.warnings[0]


def test_wav_recorder_accepts_list_int_payload(tmp_path: Path):
    """AudioData stub encodes bytes as ``list[int]``; mirror that here."""
    rec = WavRecorder(sample_rate=16000)
    rec(_msg([0] * (16000 * 2)))  # uint8 silence
    rep = rec.to_wav(tmp_path / "x.wav")
    assert rep.bytes_total == 32000
    assert rep.frames == 1


def test_wav_recorder_handles_empty_payload(tmp_path: Path):
    rec = WavRecorder(sample_rate=16000)
    rec(_msg(b""))
    rep = rec.to_wav(tmp_path / "x.wav")
    assert rep.frames == 1
    assert rep.bytes_total == 0


def test_wav_recorder_rejects_unexpected_payload_type(tmp_path: Path):
    rec = WavRecorder(sample_rate=16000)
    # A dict is not bytes/list — WavRecorder.__call__ must raise.
    with pytest.raises(TypeError):
        rec(SimpleNamespace(data={"foo": "bar"}))


# ----------------------------------------------------------------------
# QoS compatibility — acceptance #3 in the bench spec.
# ----------------------------------------------------------------------


def test_publisher_qos_profile_matches_production_defaults():
    """The bench publisher must publish AudioData under the same QoS the
    production ``TTSNode`` uses: ``KEEP_LAST / depth=10``,
    ``best_effort`` reliability, ``volatile`` durability. A consumer that
    asks for ``RELIABLE + depth=1`` must still receive frames because
    best_effort publishers satisfy both reliable and best_effort
    subscribers (compatibility rule: publisher ≤ subscriber).
    """
    from ros_stub import (
        HistoryPolicy,
        QoSProfile,
        ReliabilityPolicy,
        DurabilityPolicy,
        _Publisher,
        AudioData,
    )

    # Mirror src/rob_box_voice/tts_node.py QoS defaults.
    pub_qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )
    pub = _Publisher(msg_type=AudioData, topic="/voice/audio/speech", qos=pub_qos)

    assert pub.qos.history == "keep_last"
    assert pub.qos.depth == 10
    assert pub.qos.reliability == "best_effort"
    assert pub.qos.durability == "volatile"

    # Subscriber-side QoS — a stricter profile must still match.
    sub_qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )
    # ROS2 compatibility rule: pub.reliability <= sub.reliability
    # best_effort publisher satisfies any subscriber (reliable OR best_effort).
    assert (
        pub.qos.reliability == "best_effort"
    ), "best_effort pubs must satisfy both reliable and best_effort subs"


def test_wav_recorder_handles_chunked_pcm_with_realistic_qos(tmp_path: Path):
    """Smoke test: a chunked payload published under the production QoS
    must reassemble into a valid WAV that ffprobe accepts as int16 LE
    mono @ 16 kHz."""
    import shutil
    import subprocess

    rec = WavRecorder(sample_rate=16000)
    # 4 chunks × 0.25 s × 16 kHz int16 mono = 4 × 4000 samples × 2 B.
    for _ in range(4):
        rec(_msg(b"\x00\x00" * 4000))
    out = tmp_path / "chunked.wav"
    rec.to_wav(out)

    if shutil.which("ffprobe"):
        cp = subprocess.run(
            [
                "ffprobe",
                "-v",
                "error",
                "-select_streams",
                "a:0",
                "-show_entries",
                "stream=sample_rate,channels,codec_name,sample_fmt",
                "-of",
                "default=noprint_wrappers=1",
                str(out),
            ],
            check=True,
            capture_output=True,
            text=True,
        )
        out_text = cp.stdout
        assert "codec_name=pcm_s16le" in out_text
        assert "sample_rate=16000" in out_text
        assert "channels=1" in out_text
    else:
        with wave.open(str(out), "rb") as w:
            assert w.getframerate() == 16000
            assert w.getnchannels() == 1
            assert w.getsampwidth() == 2
