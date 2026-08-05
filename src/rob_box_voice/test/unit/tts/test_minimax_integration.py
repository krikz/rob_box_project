"""Unit tests for the MiniMax → ROS audio integration in tts_node.

These tests focus on the work covered by kanban task ``t_257dbfb9``:

* transcode of MiniMax audio containers (PCM/WAV/MP3/OGG) into int16 LE PCM
* retry classification per ADR-0003 §2.6
* per-chunk publication when ``minimax_streaming=true``
* mono→stereo for ReSpeaker (implicit, via shared downstream path)
* the format / retry / streaming ROS-параметры

We mock the heavy ROS surface via :mod:`test.unit.tts.conftest` and
inject Fake providers from ``rob_box_llm`` so no network / sound device
is involved.
"""

from __future__ import annotations

import asyncio
import io
import struct
import sys
import threading
import time
import wave
from unittest.mock import MagicMock

import numpy as np
import pytest

from rob_box_llm.errors import (
    TTSAuthError,
    TTSBadRequestError,
    TTSRateLimitError,
    TTSTimeoutError,
)
from rob_box_llm.tts import TTSAudio, TTSChunk, TTSFormat

# Import transcode helper directly (bypass utils/__init__.py → pyaudio).
import importlib.util as _ilu
import pathlib as _pl

# Use pathlib instead of __file__.rsplit: pytest 9.x passes ``__file__``
# with OS-native separators on Windows, so the previous rsplit('/test/.../')
# only worked under POSIX. parents[3] is robust across pytest versions.
_TRANSCODE_PATH = str(
    _pl.Path(__file__).resolve().parents[3]
    / "rob_box_voice"
    / "utils"
    / "audio_transcode.py"
)
_spec = _ilu.spec_from_file_location("rob_box_voice_audio_transcode_tts", _TRANSCODE_PATH)
_mod = _ilu.module_from_spec(_spec)
sys.modules["rob_box_voice_audio_transcode_tts"] = _mod
_spec.loader.exec_module(_mod)
to_pcm_int16 = _mod.to_pcm_int16
AudioTranscodeError = _mod.AudioTranscodeError

# Import tts_node AFTER mocks are installed by conftest.
from rob_box_voice import tts_node  # noqa: E402

# ─────────────────────────────────────────────────────────────────────────────
# Helpers
# ─────────────────────────────────────────────────────────────────────────────


def _make_pcm_bytes(samples: list[int]) -> bytes:
    return struct.pack(f"<{len(samples)}h", *samples)


def _make_wav_bytes(samples: list[int], sr: int = 16000, channels: int = 1) -> bytes:
    if channels > 1:
        n = len(samples) // channels
        cols = [samples[c * n : (c + 1) * n] for c in range(channels)]
        out = []
        for i in range(n):
            for c in range(channels):
                out.append(cols[c][i])
        samples = out
    buf = io.BytesIO()
    with wave.open(buf, "wb") as w:
        w.setnchannels(channels)
        w.setsampwidth(2)
        w.setframerate(sr)
        w.writeframes(struct.pack(f"<{len(samples)}h", *samples))
    return buf.getvalue()


def _make_fake_node():
    """Create a minimal stub with the attributes read by ``_synthesize_minimax_*``.

    We can't instantiate ``TTSNode()`` here (it touches rclpy internals).
    Instead we make a plain object with the right attributes and call the
    unbound methods (passing ``node`` as the first arg).
    """

    class _Stub:
        """Bare class — NOT a MagicMock (which is too strict on attrs)."""

        pass

    node = _Stub()
    node.minimax_max_retries = 2
    node.minimax_retry_backoff_ms = 1
    node.minimax_api_key = "test"
    node.minimax_group_id = "test"
    node.minimax_voice = "male-qn-qingse"
    node.minimax_model = "speech-02-hd"
    node.minimax_timeout = 30.0
    node.minimax_speed = 1.0
    node.minimax_language = "ru"
    node.minimax_sample_rate = 32000
    node.minimax_format = TTSFormat.PCM
    node.audio_output_sample_rate = 16000
    node.audio_pub = MagicMock()
    node.get_logger = MagicMock()
    node._decoded_audio_to_float32 = tts_node.TTSNode._decoded_audio_to_float32
    node._decode_minimax_audio = tts_node.TTSNode._decode_minimax_audio.__get__(node, type(node))
    node._ensure_minimax_provider = tts_node.TTSNode._ensure_minimax_provider.__get__(node, type(node))
    node._prepare_audio_for_topic = tts_node.TTSNode._prepare_audio_for_topic.__get__(node, type(node))
    return node


class _FakeProvider:
    """Minimal stand-in for ``MiniMaxTTSProvider``.

    Implements ``synthesize`` and ``stream`` with controllable outcomes so
    we can exercise error classification paths. Tracks call count for
    retry assertions.
    """

    def __init__(self, *, samples=b"", sample_rate=32000, fmt=TTSFormat.PCM):
        self._samples = samples
        self._sample_rate = sample_rate
        self._fmt = fmt
        self.synthesize_calls = 0
        self.stream_calls = 0

    async def synthesize(self, text, *, settings=None):
        self.synthesize_calls += 1
        return TTSAudio(
            samples=self._samples,
            sample_rate=self._sample_rate,
            format=self._fmt,
            raw={"text": text, "fake": True},
        )

    async def stream(self, text, *, settings=None):
        self.stream_calls += 1
        chunk = TTSChunk(
            samples=self._samples,
            sample_rate=self._sample_rate,
            format=self._fmt,
            finish_reason="stop",
        )
        yield chunk

    async def aclose(self):
        return None


# ─────────────────────────────────────────────────────────────────────────────
# transcode integration
# ─────────────────────────────────────────────────────────────────────────────


class TestTranscodeIntegration:
    """Cover ``_synthesize_minimax_async`` decode logic for each container."""

    def _make_node(self):
        """Produce a stub with the attributes read by the unbound method."""
        return _make_fake_node()

    def _run(self, coro):
        return asyncio.run(coro)

    def test_pcm_passthrough_preserves_sample_count(self):
        """PCM: no decode, samples come through identical."""
        node = self._make_node()
        pcm = _make_pcm_bytes([100, -200, 32767, -32768])
        provider = _FakeProvider(samples=pcm, sample_rate=32000, fmt=TTSFormat.PCM)
        node.minimax_provider = provider  # skip lazy init

        async def go():
            return await tts_node.TTSNode._synthesize_minimax_async(node, "hello", {})

        result = self._run(go())
        assert result["sample_rate"] == 32000
        assert len(result["audio_np"]) == 4  # 4 int16 samples → 4 float32
        assert abs(result["audio_np"][2] - 32767 / 32768.0) < 1e-3

    def test_wav_container_is_decoded_to_pcm(self):
        """WAV: header is stripped, mono preserved, sample_rate from header."""
        node = self._make_node()
        wav = _make_wav_bytes([10, -10] * 10, sr=16000, channels=1)
        provider = _FakeProvider(samples=wav, sample_rate=16000, fmt=TTSFormat.WAV)
        node.minimax_provider = provider

        async def go():
            return await tts_node.TTSNode._synthesize_minimax_async(node, "hello", {})

        result = self._run(go())
        assert result["sample_rate"] == 16000
        assert len(result["audio_np"]) == 20

    def test_stereo_wav_is_downmixed_to_mono(self):
        """WAV stereo is normalized to mono by the transcode helper."""
        node = self._make_node()
        left = [1000, -1000, 3000]
        right = [-1000, 1000, 1000]
        wav = _make_wav_bytes(left + right, sr=16000, channels=2)
        provider = _FakeProvider(samples=wav, sample_rate=16000, fmt=TTSFormat.WAV)
        node.minimax_provider = provider

        async def go():
            return await tts_node.TTSNode._synthesize_minimax_async(node, "hello", {})

        result = self._run(go())
        assert len(result["audio_np"]) == 3
        assert np.allclose(
            result["audio_np"],
            np.array([0, 0, 2000], dtype=np.float32) / 32768.0,
        )


class TestDefaultProviderPath:
    def test_yandex_path_publishes_without_referencing_streaming_result(self):
        """Non-MiniMax providers must not depend on MiniMax-only state."""
        node = _make_fake_node()
        node.provider = "yandex"
        node.normalize_text = False
        node.stop_requested = False
        node.processing_dialogue_id = None
        node.current_dialogue_id = None
        node.yandex_stub = object()
        node.yandex_speed = 1.0
        node._synthesize_yandex = MagicMock(
            return_value=np.zeros(2205, dtype=np.float32)
        )
        node.publish_state = MagicMock()
        node._publish_audio = MagicMock()
        node.get_logger = MagicMock()
        node.chipmunk_mode = False
        node.pitch_shift = 1.0
        node.volume_gain = 1.0
        node.device_index = None
        node.current_stream = None
        node.playback_manager = MagicMock()
        node.playback_manager.play_audio.return_value = True
        node.cleanup_playback_noise = MagicMock()
        node.finished_pub = MagicMock()

        tts_node.TTSNode._synthesize_and_play(
            node,
            "<speak>hello</speak>",
            "hello",
            None,
            {},
            None,
        )

        node._publish_audio.assert_called_once()
        node.playback_manager.play_audio.assert_called_once()


# ─────────────────────────────────────────────────────────────────────────────
# retry classification
# ─────────────────────────────────────────────────────────────────────────────


class TestRetryClassification:
    """Verify the policy in ADR-0003 §2.6:
    - timeout/rate/5xx → retried with exp backoff
    - auth/bad-request → NEVER retried (raises immediately)
    """

    def _patch_async(self, fake_async):
        """Bind ``_synthesize_minimax_async`` on a stubbed node.

        We use ``setattr`` so the method is looked up via ``self.X``
        inside the retry-loop, exactly as in production code.
        """
        node = _make_fake_node()
        node._synthesize_minimax_async = fake_async
        # Convert to a bound-method-like callable by wrapping.
        return node

    def _run(self, coro):
        return asyncio.run(coro)

    def test_retry_after_two_timeouts_succeeds(self):
        """2 timeout failures then success → 3 attempts, exp backoff."""
        call_count = [0]

        async def fake_async(text, ssml_attributes):
            call_count[0] += 1
            if call_count[0] < 3:
                raise TTSTimeoutError("net down")
            return {"audio_np": np.zeros(8, dtype=np.float32), "sample_rate": 32000}

        node = self._patch_async(fake_async)
        result = self._run(tts_node.TTSNode._synthesize_minimax_with_retry(node, "hi", {}))
        assert call_count[0] == 3
        assert result["sample_rate"] == 32000

    def test_auth_error_not_retried(self):
        """TTSAuthError → raised without retry."""
        call_count = [0]

        async def fake_async(text, ssml_attributes):
            call_count[0] += 1
            raise TTSAuthError("bad key", provider="minimax")

        node = self._patch_async(fake_async)
        with pytest.raises(Exception, match="bad key"):
            self._run(tts_node.TTSNode._synthesize_minimax_with_retry(node, "hi", {}))
        # Single attempt, no retry
        assert call_count[0] == 1

    def test_bad_request_not_retried(self):
        """TTSBadRequestError → raised without retry."""
        call_count = [0]

        async def fake_async(text, ssml_attributes):
            call_count[0] += 1
            raise TTSBadRequestError("invalid voice", provider="minimax")

        node = self._patch_async(fake_async)
        with pytest.raises(Exception, match="invalid voice"):
            self._run(tts_node.TTSNode._synthesize_minimax_with_retry(node, "hi", {}))
        assert call_count[0] == 1

    def test_rate_limit_is_retried(self):
        """TTSRateLimitError → at most one retry, regardless of generic budget."""
        call_count = [0]

        async def fake_async(text, ssml_attributes):
            call_count[0] += 1
            raise TTSRateLimitError("429", provider="minimax")

        node = self._patch_async(fake_async)
        node.minimax_max_retries = 3
        with pytest.raises(TTSRateLimitError, match="429"):
            self._run(tts_node.TTSNode._synthesize_minimax_with_retry(node, "hi", {}))
        assert call_count[0] == 2

    def test_exhaustion_raises_last_error(self):
        """After max_retries exhausted, propagate the last error."""
        call_count = [0]

        async def fake_async(text, ssml_attributes):
            call_count[0] += 1
            raise TTSTimeoutError("timeout", provider="minimax")

        node = self._patch_async(fake_async)
        # Force 0 retries
        node.minimax_max_retries = 0
        with pytest.raises(Exception, match="timeout"):
            self._run(tts_node.TTSNode._synthesize_minimax_with_retry(node, "hi", {}))
        # 1 attempt total
        assert call_count[0] == 1


# ─────────────────────────────────────────────────────────────────────────────
# streaming hook
# ─────────────────────────────────────────────────────────────────────────────


class TestStreamingHook:
    """Verify the streaming-mode publishing path publishes one AudioData.
    msg per chunk (even though today the provider returns a single chunk,
    so behaviour is identical to the sync path).
    """

    def _make_node(self):
        node = _make_fake_node()
        # Bind the REAL `_publish_audio` so AudioData publications land on
        # node.audio_pub. Without this binding the chain is just MagicMock
        # no-ops and we can't assert call counts.
        bound_publish = tts_node.TTSNode._publish_audio.__get__(node, type(node))
        node._publish_audio = bound_publish
        return node

    def test_publish_audio_clips_out_of_range_samples(self):
        node = self._make_node()

        tts_node.TTSNode._publish_audio(
            node,
            np.array([2.0, -2.0, 0.5], dtype=np.float32),
        )

        msg = node.audio_pub.publish.call_args.args[0]
        samples = np.frombuffer(bytes(msg.data), dtype="<i2")
        assert samples.tolist() == [32767, -32767, 16383]

    def test_streaming_publishes_one_msg_per_chunk(self):
        """Streaming: per chunk we publish a separate AudioData msg."""
        node = self._make_node()
        pcm = _make_pcm_bytes([100, -100, 50, -50, 25])

        # Build a fake _stream_minimax_chunks that yields 3 chunks.
        chunks = [
            TTSChunk(samples=pcm[:4], sample_rate=32000, format=TTSFormat.PCM),
            TTSChunk(samples=pcm[4:8], sample_rate=32000, format=TTSFormat.PCM),
            TTSChunk(
                samples=pcm[8:],
                sample_rate=32000,
                format=TTSFormat.PCM,
                finish_reason="stop",
            ),
        ]

        async def fake_stream(text, ssml_attrs):
            for c in chunks:
                yield c

        async def collect():
            node._stream_minimax_chunks = fake_stream
            return await asyncio.to_thread(
                lambda: tts_node.TTSNode._synthesize_minimax_streaming_publish(node, "hello", {})
            )

        result = asyncio.run(collect())
        # 3 AudioData msgs published (one per chunk).
        assert node.audio_pub.publish.call_count == 3
        # Final concatenated buffer has the same number of float32 samples as input.
        assert len(result["audio_np"]) == 5

    def test_streaming_publishes_each_chunk_before_requesting_the_next(self):
        """The first AudioData message must be visible before the next chunk is pulled."""
        node = self._make_node()
        pcm = _make_pcm_bytes([100, -100, 50, -50])

        async def fake_stream(text, ssml_attrs):
            yield TTSChunk(samples=pcm[:4], sample_rate=16000, format=TTSFormat.PCM)
            assert node.audio_pub.publish.call_count == 1
            yield TTSChunk(
                samples=pcm[4:],
                sample_rate=16000,
                format=TTSFormat.PCM,
                finish_reason="stop",
            )

        node._stream_minimax_chunks = fake_stream
        result = tts_node.TTSNode._synthesize_minimax_streaming_publish(node, "hello", {})

        assert node.audio_pub.publish.call_count == 2
        assert len(result["audio_np"]) == 4

    def test_streaming_resamples_chunk_to_declared_topic_rate(self):
        node = self._make_node()
        pcm = _make_pcm_bytes([100, -100, 50, -50])

        async def fake_stream(text, ssml_attrs):
            yield TTSChunk(
                samples=pcm,
                sample_rate=32000,
                format=TTSFormat.PCM,
                finish_reason="stop",
            )

        node._stream_minimax_chunks = fake_stream
        result = tts_node.TTSNode._synthesize_minimax_streaming_publish(
            node,
            "hello",
            {},
        )

        assert result["sample_rate"] == 32000
        msg = node.audio_pub.publish.call_args.args[0]
        assert len(msg.data) == 4  # 4 samples @ 32 kHz -> 2 samples @ 16 kHz

    def test_streaming_handles_finish_reason_error(self):
        """Stream reports finish_reason='error' → exception propagated up."""
        node = self._make_node()
        bound_publish = tts_node.TTSNode._publish_audio.__get__(node, type(node))
        node._publish_audio = bound_publish

        async def fake_stream(text, ssml_attrs):
            yield TTSChunk(
                samples=b"",
                sample_rate=32000,
                format=TTSFormat.PCM,
                finish_reason="error",
            )

        async def collect():
            node._stream_minimax_chunks = fake_stream
            return await asyncio.to_thread(
                lambda: tts_node.TTSNode._synthesize_minimax_streaming_publish(node, "hello", {})
            )

        with pytest.raises(Exception, match="error"):
            asyncio.run(collect())

    def test_streaming_handles_empty_chunks(self):
        """Zero chunks → typed error (no AudioData publication)."""
        node = self._make_node()
        bound_publish = tts_node.TTSNode._publish_audio.__get__(node, type(node))
        node._publish_audio = bound_publish

        async def fake_stream(text, ssml_attrs):
            if False:
                yield

        async def collect():
            node._stream_minimax_chunks = fake_stream
            return await asyncio.to_thread(
                lambda: tts_node.TTSNode._synthesize_minimax_streaming_publish(node, "hello", {})
            )

        with pytest.raises(Exception, match="no audio chunks"):
            asyncio.run(collect())
        assert node.audio_pub.publish.call_count == 0

    def test_streaming_error_after_audio_is_not_published_as_silence(self):
        """A mid-stream error must not make the bridge replay a partial buffer."""
        node = self._make_node()
        pcm = _make_pcm_bytes([100, -100])

        async def fake_stream(text, ssml_attrs):
            yield TTSChunk(samples=pcm, sample_rate=16000, format=TTSFormat.PCM)
            yield TTSChunk(
                samples=b"",
                sample_rate=16000,
                format=TTSFormat.PCM,
                finish_reason="error",
            )

        node._stream_minimax_chunks = fake_stream
        with pytest.raises(Exception, match="finish_reason=error"):
            tts_node.TTSNode._synthesize_minimax_streaming_publish(node, "hello", {})

        assert node.audio_pub.publish.call_count == 1


# ─────────────────────────────────────────────────────────────────────────────
# _parse_format
# ─────────────────────────────────────────────────────────────────────────────


class TestParseFormat:
    def test_valid_pcm(self):
        assert tts_node.TTSNode._parse_format("pcm") == TTSFormat.PCM

    def test_valid_mp3(self):
        assert tts_node.TTSNode._parse_format("mp3") == TTSFormat.MP3

    def test_valid_ogg(self):
        assert tts_node.TTSNode._parse_format("ogg") == TTSFormat.OGG

    def test_case_insensitive(self):
        assert tts_node.TTSNode._parse_format("PCM") == TTSFormat.PCM
        assert tts_node.TTSNode._parse_format(" Mp3 ") == TTSFormat.MP3

    def test_invalid_raises_value_error(self):
        with pytest.raises(ValueError, match="недопустим"):
            tts_node.TTSNode._parse_format("flac")


class TestProviderCleanup:
    def test_close_minimax_provider_closes_client_and_clears_reference(self):
        node = _make_fake_node()

        class Provider:
            closed = False

            async def aclose(self):
                self.closed = True

        provider = Provider()
        node.minimax_provider = provider

        tts_node.TTSNode.close_minimax_provider(node)

        assert provider.closed is True
        assert node.minimax_provider is None


class TestProviderLifecycleSynchronization:
    def test_concurrent_lazy_init_constructs_one_provider(self, monkeypatch):
        node = _make_fake_node()
        node.minimax_provider = None
        constructed = []

        class Provider:
            pass

        def factory(**kwargs):
            # Make the race window deterministic: the second caller must wait
            # for the first caller's constructor before it can inspect state.
            time.sleep(0.01)
            provider = Provider()
            constructed.append(provider)
            return provider

        monkeypatch.setattr(tts_node, "MiniMaxTTSProvider", factory)
        results = []
        errors = []

        def worker():
            try:
                results.append(tts_node.TTSNode._ensure_minimax_provider(node))
            except Exception as exc:  # pragma: no cover - assertion below
                errors.append(exc)

        threads = [threading.Thread(target=worker) for _ in range(8)]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join()

        assert errors == []
        assert len(constructed) == 1
        assert results == [constructed[0]] * len(results)
        assert node._minimax_provider_initialized is True

    def test_shutdown_skips_uninitialized_provider(self):
        node = _make_fake_node()
        node.minimax_provider = None

        tts_node.TTSNode.close_minimax_provider(node)

        assert node._minimax_shutdown_requested is True
        assert node.minimax_provider is None

    def test_shutdown_waits_for_construction_then_closes_provider(self, monkeypatch):
        node = _make_fake_node()
        node.minimax_provider = None
        constructor_started = threading.Event()
        allow_constructor_to_finish = threading.Event()

        class Provider:
            closed = False

            async def aclose(self):
                self.closed = True

        provider = Provider()

        def factory(**kwargs):
            constructor_started.set()
            assert allow_constructor_to_finish.wait(timeout=1.0)
            return provider

        monkeypatch.setattr(tts_node, "MiniMaxTTSProvider", factory)
        init_thread = threading.Thread(
            target=lambda: tts_node.TTSNode._ensure_minimax_provider(node)
        )
        init_thread.start()
        assert constructor_started.wait(timeout=1.0)

        close_thread = threading.Thread(
            target=lambda: tts_node.TTSNode.close_minimax_provider(node)
        )
        close_thread.start()
        allow_constructor_to_finish.set()
        init_thread.join(timeout=1.0)
        close_thread.join(timeout=1.0)

        assert not init_thread.is_alive()
        assert not close_thread.is_alive()
        assert provider.closed is True
        assert node.minimax_provider is None
        assert node._minimax_provider_initialized is False

    def test_shutdown_closes_initialized_provider_even_when_flag_missing(self):
        """Compatibility with lightweight test doubles made before the flag."""
        node = _make_fake_node()

        class Provider:
            closed = False

            async def aclose(self):
                self.closed = True

        provider = Provider()
        node.minimax_provider = provider

        tts_node.TTSNode.close_minimax_provider(node)

        assert provider.closed is True
        assert node.minimax_provider is None
