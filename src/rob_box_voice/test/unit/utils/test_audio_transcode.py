"""Unit tests for :mod:`rob_box_voice.utils.audio_transcode`.

These tests exercise the transcode helper that bridges provider audio
blobs (PCM / WAV / MP3 / OGG) into ROS-ready int16 little-endian PCM mono
(see ADR-0003 §2.3).

The MP3 / OGG paths require optional ``pydub`` and ``ffmpeg`` — when
neither is available in the test environment, those cases should raise
:exc:`AudioTranscodeError` and we assert that explicitly so the failure
mode is documented (it is NOT a test regression).
"""

from __future__ import annotations

import io
import shutil
import struct
import subprocess
import wave
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest

from rob_box_llm.tts import TTSFormat

# Import the transcode module directly, bypassing utils/__init__.py which
# pulls in audio_utils → pyaudio (unavailable on dev containers without
# USB-audio stack). The transcode helper itself doesn't need pyaudio.
import importlib.util as _ilu
import sys as _sys

_TRANSCODE_PATH = __file__.rsplit("/test/unit/utils/", 1)[0] + "/rob_box_voice/utils/audio_transcode.py"
_spec = _ilu.spec_from_file_location("rob_box_voice_audio_transcode", _TRANSCODE_PATH)
_mod = _ilu.module_from_spec(_spec)
_sys.modules["rob_box_voice_audio_transcode"] = _mod
_spec.loader.exec_module(_mod)

AudioTranscodeError = _mod.AudioTranscodeError
DecodedAudio = _mod.DecodedAudio
to_pcm_int16 = _mod.to_pcm_int16


# ─────────────────────────────────────────────────────────────────────────────
# Fixtures / helpers
# ─────────────────────────────────────────────────────────────────────────────


def _make_pcm_bytes(samples: list[int]) -> bytes:
    """Pack a list of int16 samples as little-endian PCM bytes."""
    return struct.pack(f"<{len(samples)}h", *samples)


def _make_wav_bytes(samples: list[int], sample_rate: int = 16000, channels: int = 1) -> bytes:
    """Build a tiny in-memory WAV blob from int16 mono/stereo samples."""
    if channels > 1:
        # interleave
        cols = []
        n = len(samples) // channels
        for c in range(channels):
            cols.append(samples[c * n : (c + 1) * n])
        interleaved = []
        for i in range(n):
            for c in range(channels):
                interleaved.append(cols[c][i])
        samples = interleaved
    buf = io.BytesIO()
    with wave.open(buf, "wb") as w:
        w.setnchannels(channels)
        w.setsampwidth(2)  # 16-bit
        w.setframerate(sample_rate)
        w.writeframes(struct.pack(f"<{len(samples)}h", *samples))
    return buf.getvalue()


# ─────────────────────────────────────────────────────────────────────────────
# PCM passthrough
# ─────────────────────────────────────────────────────────────────────────────


class TestPCMPassthrough:
    def test_pcm_bytes_are_returned_verbatim(self):
        pcm = _make_pcm_bytes([100, -200, 32767, -32768])
        out = to_pcm_int16(pcm, TTSFormat.PCM, default_sample_rate=32000)
        assert isinstance(out, DecodedAudio)
        assert out.pcm == pcm
        assert out.sample_rate == 32000
        assert out.channels == 1
        assert out.source_format == TTSFormat.PCM

    def test_pcm_with_default_rate_zero_falls_back(self):
        pcm = _make_pcm_bytes([0, 0])
        out = to_pcm_int16(pcm, TTSFormat.PCM, default_sample_rate=0)
        # When caller passes 0 we trust it but log nothing — that's the contract;
        # the helper doesn't auto-pick a default. We just verify it doesn't crash.
        assert out.channels == 1


# ─────────────────────────────────────────────────────────────────────────────
# WAV decoding
# ─────────────────────────────────────────────────────────────────────────────


class TestWAVDecoding:
    def test_mono_wav_round_trips(self):
        # 1 kHz sine — sample values are repeatable.
        samples = [int(np.sin(2 * np.pi * 1000 * i / 16000) * 16384) for i in range(160)]
        wav_bytes = _make_wav_bytes(samples, sample_rate=16000, channels=1)
        out = to_pcm_int16(wav_bytes, TTSFormat.WAV)
        assert out.channels == 1
        assert out.sample_rate == 16000
        # Sample counts may differ slightly due to int round-trip, but should be close
        assert abs(len(out.pcm) // 2 - len(samples)) <= 2

    def test_stereo_wav_is_downmixed_to_mono(self):
        left = [1000, -1000, 3000]
        right = [-1000, 1000, 1000]
        wav_bytes = _make_wav_bytes(left + right, sample_rate=32000, channels=2)
        out = to_pcm_int16(wav_bytes, TTSFormat.WAV)
        assert out.channels == 1
        assert out.sample_rate == 32000
        assert struct.unpack("<3h", out.pcm) == (0, 0, 2000)

    def test_invalid_wav_raises_typed_error(self):
        bad = b"not a wav file"
        with pytest.raises(AudioTranscodeError) as exc_info:
            to_pcm_int16(bad, TTSFormat.WAV)
        assert exc_info.value.fmt == "wav"
        assert exc_info.value.reason == "bad_wav"


# ─────────────────────────────────────────────────────────────────────────────
# Empty / bad payload
# ─────────────────────────────────────────────────────────────────────────────


class TestEmpty:
    def test_empty_payload_raises(self):
        with pytest.raises(AudioTranscodeError) as exc_info:
            to_pcm_int16(b"", TTSFormat.PCM, default_sample_rate=32000)
        assert exc_info.value.reason == "empty"

    def test_odd_length_pcm_raises_typed_alignment_error(self):
        with pytest.raises(AudioTranscodeError) as exc_info:
            to_pcm_int16(b"\x00", TTSFormat.PCM, default_sample_rate=32000)
        assert exc_info.value.reason == "unaligned_pcm"


# ─────────────────────────────────────────────────────────────────────────────
# MP3 / OGG handling
# ─────────────────────────────────────────────────────────────────────────────


class TestCompressed:
    @staticmethod
    def _encode_tone(fmt: TTSFormat) -> bytes:
        ffmpeg = shutil.which("ffmpeg")
        if not ffmpeg:
            pytest.skip("ffmpeg is not installed")
        codec_args = ["-codec:a", "libmp3lame"] if fmt == TTSFormat.MP3 else ["-codec:a", "libvorbis"]
        proc = subprocess.run(
            [
                ffmpeg,
                "-hide_banner",
                "-loglevel",
                "error",
                "-f",
                "lavfi",
                "-i",
                "sine=frequency=440:duration=0.05:sample_rate=16000",
                *codec_args,
                "-f",
                fmt.value,
                "pipe:1",
            ],
            capture_output=True,
            check=True,
            timeout=10,
        )
        return proc.stdout

    @pytest.mark.parametrize("fmt", [TTSFormat.MP3, TTSFormat.OGG])
    def test_ffmpeg_decodes_compressed_audio_to_mono_pcm(self, fmt):
        encoded = self._encode_tone(fmt)
        out = to_pcm_int16(encoded, fmt)

        assert out.channels == 1
        assert out.sample_rate == 44100
        assert len(out.pcm) > 0
        assert len(out.pcm) % 2 == 0

    @pytest.mark.parametrize("fmt", [TTSFormat.MP3, TTSFormat.OGG])
    def test_no_decoder_available_raises_clean_error(self, fmt, monkeypatch):
        """When neither pydub nor ffmpeg is on PATH, we must raise a typed
        error (not a generic ImportError) so the caller can surface a
        useful message to the operator.
        """

        # Mock shutil.which on the already-imported transcode module's namespace.
        def fake_which(name):
            return None

        monkeypatch.setattr(_mod.shutil, "which", fake_which)

        with pytest.raises(AudioTranscodeError) as exc_info:
            to_pcm_int16(b"\x00\x00", fmt)
        assert exc_info.value.fmt == fmt.value
        assert exc_info.value.reason == "no_decoder"


# ─────────────────────────────────────────────────────────────────────────────
# Sample-rate from container
# ─────────────────────────────────────────────────────────────────────────────


class TestSampleRate:
    def test_wav_with_44100_sr_is_preserved(self):
        samples = [0] * 100
        wav_bytes = _make_wav_bytes(samples, sample_rate=44100, channels=1)
        out = to_pcm_int16(wav_bytes, TTSFormat.WAV)
        assert out.sample_rate == 44100


class TestRosPackageMetadata:
    def test_transcoder_runtime_dependencies_are_declared(self):
        package_xml = Path(__file__).parents[3] / "package.xml"
        root = ET.parse(package_xml).getroot()
        dependencies = {element.text for element in root.findall("exec_depend")}

        assert "python3-numpy" in dependencies
        assert "ffmpeg" in dependencies
