"""Tests for the TTS value objects in :mod:`rob_box_llm.tts`.

No network, no asyncio. Just dataclass + enum + frozen-args sanity checks.
"""

from __future__ import annotations

import pytest

from rob_box_llm.tts import TTSAudio, TTSChunk, TTSFormat, TTSSettings

# ---------------------------------------------------------------------------
# TTSFormat
# ---------------------------------------------------------------------------


class TestTTSFormat:
    def test_values(self):
        assert TTSFormat.PCM.value == "pcm"
        assert TTSFormat.WAV.value == "wav"
        assert TTSFormat.MP3.value == "mp3"
        assert TTSFormat.OGG.value == "ogg"

    def test_is_str(self):
        # str-enum — usable as a JSON-serialisable value directly.
        assert TTSFormat.PCM == "pcm"


# ---------------------------------------------------------------------------
# TTSSettings
# ---------------------------------------------------------------------------


class TestTTSSettings:
    def test_defaults(self):
        s = TTSSettings()
        assert s.model is None
        assert s.voice is None
        assert s.language is None
        assert s.speed is None
        assert s.volume is None
        assert s.pitch is None
        assert s.emotion is None
        assert s.sample_rate is None
        assert s.format == TTSFormat.PCM
        assert s.text_normalization is None
        assert s.extra == {}

    def test_frozen(self):
        s = TTSSettings(voice="X")
        with pytest.raises((AttributeError, Exception)):
            s.voice = "Y"  # type: ignore[misc]

    def test_extra_is_immutable_mapping(self):
        s = TTSSettings(extra={"foo": 1})
        # MappingProxyType forbids mutation
        with pytest.raises(TypeError):
            s.extra["foo"] = 2  # type: ignore[index]


# ---------------------------------------------------------------------------
# TTSAudio
# ---------------------------------------------------------------------------


class TestTTSAudio:
    def test_duration_pcm(self):
        # 1 second of mono 16-bit PCM at 16 kHz = 32_000 bytes
        samples = b"\x00\x00" * 16_000
        a = TTSAudio(samples=samples, sample_rate=16_000, format=TTSFormat.PCM)
        assert a.duration_s == pytest.approx(1.0)

    def test_duration_compressed_unknown(self):
        a = TTSAudio(samples=b"\x00" * 100, sample_rate=32_000, format=TTSFormat.MP3)
        # Compressed formats have unknown bytes-per-second — return 0.
        assert a.duration_s == 0.0

    def test_frozen(self):
        a = TTSAudio(samples=b"", sample_rate=16_000)
        with pytest.raises((AttributeError, Exception)):
            a.sample_rate = 32_000  # type: ignore[misc]


# ---------------------------------------------------------------------------
# TTSChunk
# ---------------------------------------------------------------------------


class TestTTSChunk:
    def test_default_finish_reason_is_none(self):
        c = TTSChunk()
        assert c.finish_reason is None
        assert c.samples == b""
        assert c.sample_rate == 0
        assert c.format == TTSFormat.PCM

    def test_final_chunk_marker(self):
        c = TTSChunk(samples=b"\x01\x02", sample_rate=16_000, finish_reason="stop")
        assert c.finish_reason == "stop"

    def test_error_chunk_marker(self):
        c = TTSChunk(finish_reason="error")
        assert c.finish_reason == "error"
