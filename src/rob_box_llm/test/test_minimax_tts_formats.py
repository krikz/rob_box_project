"""Format-specific tests for :class:`MiniMaxTTSProvider`.

Covers the contract for the three documented audio containers (WAV/MP3/PCM)
plus OGG's graceful degradation to MP3. Each test pins down:

* the ``audio_setting.format`` field sent on the wire,
* the bytes the provider hands back,
* the ``TTSAudio.format`` marker returned to callers.

We deliberately use small, recognisable byte patterns per format so failures
scream loudly — if WAV ever stops being recognised as WAV (wrong header
bytes) or MP3 arrives without sync, the assertion fires before downstream
transcode code can paper over it.
"""

from __future__ import annotations

import json
import struct
from typing import Any

import httpx
import pytest

from rob_box_llm.providers.minimax_tts import MiniMaxTTSProvider
from rob_box_llm.tts import TTSAudio, TTSFormat, TTSSettings


# ---------------------------------------------------------------------------
# Fixtures — known-good byte sequences for each container
# ---------------------------------------------------------------------------


def _wav_bytes(num_samples: int = 16, sample_rate: int = 16_000) -> bytes:
    """Minimal valid 16-bit mono PCM RIFF/WAVE blob.

    44-byte header (canonical WAVE format) + ``num_samples * 2`` bytes of
    PCM data. The ``fmt `` chunk declares PCM, mono, the requested sample
    rate, and a 16-bit sample size so soundfile/audioop can decode it.
    """
    pcm_data = b"\x00\x01" * num_samples
    byte_rate = sample_rate * 1 * 2  # channels=1, bits=16
    header = struct.pack(
        "<4sI4s4sIHHIIHH4sI",
        b"RIFF",
        36 + len(pcm_data),  # file size - 8
        b"WAVE",
        b"fmt ",
        16,                  # fmt chunk size
        1,                   # PCM
        1,                   # mono
        sample_rate,
        byte_rate,
        1 * 2,               # block align (channels * bits/8)
        16,                  # bits per sample
        b"data",
        len(pcm_data),
    )
    return header + pcm_data


# An MP3 starts with a frame sync marker (0xFFE) followed by version /
# layer / bitrate info. We don't decode MP3 here — just assert that the
# provider passes the bytes through and tags them with the right format.
_MP3_BYTES = bytes.fromhex(
    "ffe3b4040000000000000000000000000000000000000000000000000000" * 4
)


# ---------------------------------------------------------------------------
# Helpers (mirror test_minimax_tts_provider.py — small enough to keep local)
# ---------------------------------------------------------------------------


def _mock_client(handler) -> httpx.AsyncClient:
    transport = httpx.MockTransport(handler)
    return httpx.AsyncClient(transport=transport, base_url="https://api.minimax.io")


def _ok_envelope(audio_bytes: bytes, sample_rate: int = 32_000) -> dict[str, Any]:
    return {
        "data": {
            "audio": audio_bytes.hex(),
            "audio_sample_rate": sample_rate,
            "audio_length": len(audio_bytes),
        },
        "base_resp": {"status_code": 0, "status_msg": "success"},
    }


def _make_provider(
    *,
    handler,
    default_voice: str = "V",
    default_model: str = "M",
) -> MiniMaxTTSProvider:
    return MiniMaxTTSProvider(
        api_key="k",
        group_id="g",
        default_voice=default_voice,
        default_model=default_model,
        client=_mock_client(handler),
    )


# ---------------------------------------------------------------------------
# Synthesis contract — each format round-trips the container bytes
# ---------------------------------------------------------------------------


class TestFormatSynthesis:
    """Pin down: format on the wire ↔ bytes in ↔ TTSAudio.format out."""

    @pytest.mark.asyncio
    async def test_wav_format_returns_wav_bytes_with_riff_header(self):
        wav = _wav_bytes(num_samples=32, sample_rate=24_000)

        captured: list[httpx.Request] = []

        def handler(req: httpx.Request) -> httpx.Response:
            captured.append(req)
            return httpx.Response(200, json=_ok_envelope(wav, sample_rate=24_000))

        p = _make_provider(handler=handler)

        out = await p.synthesize(
            "hi", settings=TTSSettings(format=TTSFormat.WAV, sample_rate=24_000)
        )

        # Container identity preserved — header bytes survive hex decode.
        assert isinstance(out, TTSAudio)
        assert out.samples == wav
        # Sanity: starts with the RIFF/WAVE magic so downstream transcode
        # can recognise the container.
        assert out.samples[:4] == b"RIFF"
        assert out.samples[8:12] == b"WAVE"
        # Format marker reflects what the CALLER requested, not what
        # MiniMax reports back (provider may normalise, e.g. OGG→MP3,
        # but for WAV the marker is honest).
        assert out.format == TTSFormat.WAV
        assert out.sample_rate == 24_000

        # And on the wire: audio_setting.format = "wav".
        body = json.loads(captured[0].content)
        assert body["audio_setting"]["format"] == "wav"

    @pytest.mark.asyncio
    async def test_mp3_format_returns_mp3_bytes_and_marks_format(self):
        captured: list[httpx.Request] = []

        def handler(req: httpx.Request) -> httpx.Response:
            captured.append(req)
            return httpx.Response(200, json=_ok_envelope(_MP3_BYTES, sample_rate=44_100))

        p = _make_provider(handler=handler)

        out = await p.synthesize(
            "hi", settings=TTSSettings(format=TTSFormat.MP3, sample_rate=44_100)
        )

        assert out.format == TTSFormat.MP3
        assert out.samples == _MP3_BYTES
        assert out.sample_rate == 44_100

        body = json.loads(captured[0].content)
        assert body["audio_setting"]["format"] == "mp3"

    @pytest.mark.asyncio
    async def test_ogg_format_falls_back_to_mp3_on_wire_and_reports_actual_format(self):
        """MiniMax does not support OGG; the provider degrades audio_setting
        to ``mp3`` for the API call and the returned TTSAudio.format reports
        the actual container so downstream transcode selects the right
        decoder.
        """
        captured: list[httpx.Request] = []

        def handler(req: httpx.Request) -> httpx.Response:
            captured.append(req)
            return httpx.Response(200, json=_ok_envelope(_MP3_BYTES, sample_rate=24_000))

        p = _make_provider(handler=handler)

        out = await p.synthesize(
            "hi", settings=TTSSettings(format=TTSFormat.OGG, sample_rate=24_000)
        )

        # Wire contract: MP3 fallback.
        body = json.loads(captured[0].content)
        assert body["audio_setting"]["format"] == "mp3"

        # Actual payload contract: the API returned MP3, so expose MP3.
        assert out.format == TTSFormat.MP3
        assert out.samples == _MP3_BYTES

    @pytest.mark.asyncio
    async def test_pcm_default_format_returns_raw_bytes(self):
        """Default format=PCM and explicit format=PCM behave identically."""
        pcm = b"\x00\x01\x02\x03" * 50  # 200 bytes, 100 int16 samples

        captured: list[httpx.Request] = []

        def handler(req: httpx.Request) -> httpx.Response:
            captured.append(req)
            return httpx.Response(200, json=_ok_envelope(pcm, sample_rate=32_000))

        p = _make_provider(handler=handler)

        # No settings → defaults kick in (format=PCM).
        out_default = await p.synthesize("hi")
        assert out_default.format == TTSFormat.PCM
        assert out_default.samples == pcm

        # Explicit format=PCM yields the same on-wire behaviour.
        out_explicit = await p.synthesize("hi", settings=TTSSettings(format=TTSFormat.PCM))
        assert out_explicit.format == TTSFormat.PCM
        assert out_explicit.samples == pcm

        # Both requests asked for "pcm".
        for req in captured:
            body = json.loads(req.content)
            assert body["audio_setting"]["format"] == "pcm"


class TestFormatRoundTripHeaders:
    """Cross-cutting: every format request sends the documented headers."""

    @pytest.mark.asyncio
    @pytest.mark.parametrize(
        "fmt",
        [TTSFormat.PCM, TTSFormat.WAV, TTSFormat.MP3, TTSFormat.OGG],
    )
    async def test_auth_and_group_headers_present_for_every_format(self, fmt):
        captured: list[httpx.Request] = []

        def handler(req: httpx.Request) -> httpx.Response:
            captured.append(req)
            return httpx.Response(
                200,
                json=_ok_envelope(b"\x00" * 4, sample_rate=32_000),
            )

        p = _make_provider(handler=handler)
        await p.synthesize("hi", settings=TTSSettings(format=fmt))

        assert len(captured) == 1
        req = captured[0]
        # These apply to every format — format-switching shouldn't change
        # the auth envelope.
        assert req.headers["Authorization"] == "Bearer k"
        assert req.headers["Content-Type"] == "application/json"
        assert req.url.params["GroupId"] == "g"
        # Endpoint is fixed regardless of format.
        assert str(req.url).lower().startswith("https://api.minimax.io/v1/t2a_v2")


# ---------------------------------------------------------------------------
# Streaming — sample-rate, chunk ordering, and final-chunk contract
# ---------------------------------------------------------------------------


class TestStreamingSampleRate:
    """Sample rate should flow from the API's `audio_sample_rate` field."""

    @pytest.mark.asyncio
    async def test_streaming_uses_first_events_sample_rate_when_later_missing(self):
        """MiniMax's SSE may not repeat audio_sample_rate on every event.

        The provider remembers the first non-zero sample rate it sees.
        """
        chunk_a = b"\x00\x01" * 10
        chunk_b = b"\x02\x03" * 10
        body = (
            f"data:{json.dumps(_ok_envelope(chunk_a, sample_rate=32_000))}\n\n"
            f"data:{json.dumps({'data': {'audio': chunk_b.hex()}})}\n\n"  # no SR
            "data:[DONE]\n\n"
        )

        async def handler(req: httpx.Request) -> httpx.Response:
            return httpx.Response(200, text=body)

        p = MiniMaxTTSProvider(
            api_key="k",
            group_id="g",
            client=_mock_client(handler),
        )

        chunks = [c async for c in p.stream("hi")]
        assert len(chunks) == 3
        assert chunks[0].sample_rate == 32_000
        assert chunks[0].samples == chunk_a
        assert chunks[1].samples == chunk_b
        assert chunks[2].finish_reason == "stop"

    @pytest.mark.asyncio
    async def test_streaming_yields_many_chunks_before_terminal_chunk(self):
        """Stress: 5 SSE events → 5 audio chunks + a terminal chunk."""
        chunks_hex = [b"\x00\x01", b"\x02\x03", b"\x04\x05", b"\x06\x07", b"\x08\x09"]
        events = (
            "data:"
            + json.dumps(_ok_envelope(chunks_hex[0], sample_rate=24_000))
            + "\n\n"
            + "data:"
            + json.dumps(_ok_envelope(chunks_hex[1], sample_rate=24_000))
            + "\n\n"
            + "data:"
            + json.dumps(_ok_envelope(chunks_hex[2], sample_rate=24_000))
            + "\n\n"
            + "data:"
            + json.dumps(_ok_envelope(chunks_hex[3], sample_rate=24_000))
            + "\n\n"
            + "data:"
            + json.dumps(_ok_envelope(chunks_hex[4], sample_rate=24_000))
            + "\n\n"
            + "data:[DONE]\n\n"
        )

        async def handler(req: httpx.Request) -> httpx.Response:
            return httpx.Response(200, text=events)

        p = MiniMaxTTSProvider(
            api_key="k",
            group_id="g",
            client=_mock_client(handler),
        )

        chunks = [c async for c in p.stream("hi")]
        assert len(chunks) == 6
        assert [chunk.samples for chunk in chunks[:5]] == chunks_hex
        final = chunks[-1]
        assert final.finish_reason == "stop"
        assert final.samples == b""


# ---------------------------------------------------------------------------
# API-key safety — credentials must never appear in logs
# ---------------------------------------------------------------------------


class TestAPIKeySafety:
    """The API key and group id must never leak into logs (caplog).

    We seed a uniquely recognisable key, force the provider to exercise
    an error path that historically logged exception messages, and assert
    that the literal string never appears anywhere in the captured logs.
    """

    SECRET_API_KEY = "sk-MARKER-DO-NOT-LOG-9b7448e9"
    SECRET_GROUP_ID = "g-MARKER-DO-NOT-LOG-a51996c7"

    @pytest.mark.asyncio
    async def test_api_key_not_logged_on_http_401(self, caplog):
        def handler(req: httpx.Request) -> httpx.Response:
            return httpx.Response(401, text="unauthorized")

        p = MiniMaxTTSProvider(
            api_key=self.SECRET_API_KEY,
            group_id=self.SECRET_GROUP_ID,
            client=_mock_client(handler),
        )

        with caplog.at_level("DEBUG"):
            with pytest.raises(Exception):
                await p.synthesize("hi")

        all_log_text = "\n".join(rec.getMessage() for rec in caplog.records)
        assert self.SECRET_API_KEY not in all_log_text, (
            f"API key leaked into logs!\n--- logs ---\n{all_log_text}\n---"
        )
        assert self.SECRET_GROUP_ID not in all_log_text, (
            f"Group id leaked into logs!\n--- logs ---\n{all_log_text}\n---"
        )

    @pytest.mark.asyncio
    async def test_api_key_not_logged_on_timeout(self, caplog):
        def handler(req: httpx.Request) -> httpx.Response:
            raise httpx.ReadTimeout("read timed out")

        p = MiniMaxTTSProvider(
            api_key=self.SECRET_API_KEY,
            group_id=self.SECRET_GROUP_ID,
            client=_mock_client(handler),
        )

        with caplog.at_level("DEBUG"):
            with pytest.raises(Exception):
                await p.synthesize("hi")

        all_log_text = "\n".join(rec.getMessage() for rec in caplog.records)
        assert self.SECRET_API_KEY not in all_log_text
        assert self.SECRET_GROUP_ID not in all_log_text

    @pytest.mark.asyncio
    async def test_api_key_not_logged_on_success(self, caplog):
        """Even on the success path the request logging must not echo the key."""
        pcm = b"\x00\x01" * 20

        def handler(req: httpx.Request) -> httpx.Response:
            return httpx.Response(200, json=_ok_envelope(pcm, sample_rate=24_000))

        p = MiniMaxTTSProvider(
            api_key=self.SECRET_API_KEY,
            group_id=self.SECRET_GROUP_ID,
            client=_mock_client(handler),
        )

        with caplog.at_level("DEBUG"):
            await p.synthesize("hi", settings=TTSSettings(sample_rate=24_000))

        all_log_text = "\n".join(rec.getMessage() for rec in caplog.records)
        assert self.SECRET_API_KEY not in all_log_text
        assert self.SECRET_GROUP_ID not in all_log_text

    @pytest.mark.asyncio
    async def test_api_key_not_logged_on_streaming_error(self, caplog):
        """Streaming failure path must also scrub credentials."""
        def handler(req: httpx.Request) -> httpx.Response:
            return httpx.Response(500, text="boom")

        p = MiniMaxTTSProvider(
            api_key=self.SECRET_API_KEY,
            group_id=self.SECRET_GROUP_ID,
            client=_mock_client(handler),
        )

        with caplog.at_level("DEBUG"):
            with pytest.raises(Exception):
                async for _ in p.stream("hi"):
                    pass

        all_log_text = "\n".join(rec.getMessage() for rec in caplog.records)
        assert self.SECRET_API_KEY not in all_log_text
        assert self.SECRET_GROUP_ID not in all_log_text
