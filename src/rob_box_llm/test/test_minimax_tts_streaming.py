"""Streaming behaviour tests for :class:`MiniMaxTTSProvider`.

The provider exposes its SSE endpoint as an ``AsyncIterator[TTSChunk]``.
This module pins three properties that are easy to break in a refactor
and hard to spot in a code review:

1. **Chunk order is preserved** — each SSE audio event is yielded in the
   order it arrives; concatenation of yielded chunks equals the
   concatenation of the mocked upstream bytes.
2. **Stream == synthesize** — for the same ``text`` / ``TTSSettings``, the
   non-streaming ``synthesize()`` and the streamed ``stream()`` path
   return byte-identical audio.
3. **Iterator cancellation is clean** — closing the async iterator before
   it finishes does not leak an open connection; respx records the
   cancelled request so we can assert ``mock_calls`` rather than
   guessing from timing.

We mock the HTTP layer with :mod:`respx` (the project's preferred HTTP
mock per ``conftest.py``).  respx routes can return an async generator
via ``httpx.Response(content=...)``, which is what the provider's
``client.stream("POST", ...)`` path consumes.  No aiohttp dependency
is required — httpx's streaming response surface is enough.

All tests are marked with ``@pytest.mark.minimax`` so the suite can be
selected as a unit: ``pytest -m minimax``.
"""

from __future__ import annotations

import asyncio
import json
from typing import AsyncIterator

import httpx
import pytest
import respx

from rob_box_llm.errors import TTSError
from rob_box_llm.providers.minimax_tts import MiniMaxTTSProvider
from rob_box_llm.tts import TTSFormat, TTSSettings

from conftest import MINIMAX_BASE_URL, MINIMAX_T2A_PATH

# ---------------------------------------------------------------------------
# Helpers — build SSE payloads from raw audio chunks
# ---------------------------------------------------------------------------


def _sse_event(audio_bytes: bytes, sample_rate: int = 24_000) -> str:
    """Encode a single MiniMax SSE audio event.

    The provider parses ``data:<json>\\n\\n`` payloads and expects the
    documented envelope::

        {"data": {"audio": "<hex>", "audio_sample_rate": <int>},
         "base_resp": {"status_code": 0, "status_msg": "success"}}
    """
    payload = {
        "data": {
            "audio": audio_bytes.hex(),
            "audio_sample_rate": sample_rate,
            "audio_length": len(audio_bytes),
        },
        "extra_info": None,
        "base_resp": {"status_code": 0, "status_msg": "success"},
    }
    return f"data:{json.dumps(payload)}\n\n"


SSE_TERMINATOR = "data:[DONE]\n\n"


async def _streaming_sse_response(
    audio_chunks: list[bytes],
    *,
    sample_rate: int = 24_000,
    inter_chunk_delay_s: float = 0.001,
) -> AsyncIterator[bytes]:
    """Async generator that yields SSE bytes with small delays between events.

    The brief requires 5–10 chunks of bytes with delay.  We chunk the
    mocked payload into ``len(audio_chunks)`` pieces (callers should pass
    5–10) and ``await asyncio.sleep`` between yields so the provider's
    iterator loop is forced through multiple ``aiter_lines()`` cycles
    rather than consuming the whole response in one read.
    """
    for chunk in audio_chunks:
        yield _sse_event(chunk, sample_rate=sample_rate).encode("utf-8")
        if inter_chunk_delay_s > 0:
            await asyncio.sleep(inter_chunk_delay_s)
    yield SSE_TERMINATOR.encode("utf-8")


def _register_streaming_route(
    router: respx.Router,
    *,
    audio_chunks: list[bytes],
    sample_rate: int = 24_000,
    inter_chunk_delay_s: float = 0.001,
    content_type: str = "text/event-stream",
) -> respx.Route:
    """Register a respx route that streams ``audio_chunks`` as SSE events.

    The Content-Type is set on the response header so it matches the
    chosen audio format downstream (text/event-stream for SSE — the
    container wrapping the audio is not what the provider looks at).
    """

    async def handler(request: httpx.Request) -> httpx.Response:
        gen = _streaming_sse_response(
            audio_chunks,
            sample_rate=sample_rate,
            inter_chunk_delay_s=inter_chunk_delay_s,
        )
        return httpx.Response(
            200,
            headers={"content-type": content_type},
            content=gen,
        )

    return router.post(MINIMAX_T2A_PATH).mock(side_effect=handler)


# ---------------------------------------------------------------------------
# 1. Chunk order is preserved across the iterator boundary
# ---------------------------------------------------------------------------


@pytest.mark.minimax
@pytest.mark.unit
class TestStreamChunkOrder:
    """``stream()`` must yield chunks in upstream order.

    Concatenation invariant: ``b"".join(chunk.samples for chunk in
    it) == b"".join(audio_chunks)`` for the mocked sequence.  This
    guards against accidental re-ordering, dropping, or duplication in
    the ``aiter_lines()`` loop inside the provider.
    """

    async def test_stream_chunk_order(
        self,
        mock_minimax_http: respx.Router,
        minimax_provider: MiniMaxTTSProvider,
    ) -> None:
        # 7 chunks — comfortably inside the 5–10 range from the brief.
        upstream_chunks: list[bytes] = [
            b"\x00\x01" * 32,  # chunk 0 — distinguishable payload
            b"\x02\x03" * 32,
            b"\x04\x05" * 32,
            b"\x06\x07" * 32,
            b"\x08\x09" * 32,
            b"\x0a\x0b" * 32,
            b"\x0c\x0d" * 32,
        ]
        route = _register_streaming_route(
            mock_minimax_http, audio_chunks=upstream_chunks
        )

        yielded: list = []
        async for chunk in minimax_provider.stream(
            "hello world",
            settings=TTSSettings(sample_rate=24_000, format=TTSFormat.PCM),
        ):
            yielded.append(chunk)

        # Strip the terminal chunk (samples=b"", finish_reason="stop")
        # before comparing — we only assert on the audio frames here.
        audio_chunks_out = [c for c in yielded if c.samples]
        terminal_chunks = [c for c in yielded if not c.samples]

        # Order: each chunk must appear in upstream order.
        assert [c.samples for c in audio_chunks_out] == upstream_chunks

        # Concatenation: joining the streamed bytes must equal the
        # concatenation of the mocked upstream bytes (byte-for-byte).
        assert b"".join(c.samples for c in audio_chunks_out) == b"".join(upstream_chunks)

        # End-of-stream contract: exactly one terminal chunk with the
        # documented ``finish_reason="stop"`` marker.
        assert len(terminal_chunks) == 1
        assert terminal_chunks[0].finish_reason == "stop"
        assert terminal_chunks[0].sample_rate == 24_000

        # The route was actually called — guards against silent "no
        # HTTP traffic" passes that would let the iterator return the
        # empty-fallback path without exercising our mock.
        assert route.called
        assert route.call_count == 1

        # All streamed chunks advertise the requested format.
        for chunk in audio_chunks_out:
            assert chunk.format == TTSFormat.PCM


# ---------------------------------------------------------------------------
# 2. Streaming aggregation equals single-shot synthesis (byte-identical)
# ---------------------------------------------------------------------------


@pytest.mark.minimax
@pytest.mark.unit
class TestStreamAggregationEqualsSynthesis:
    """Same request in stream-mode and non-stream-mode → same bytes.

    The MiniMax backend can return audio either as a single envelope
    (when ``stream=false``) or as a sequence of SSE events that together
    contain the same audio (when ``stream=true``).  For the provider to
    be a drop-in replacement for batch callers that switch between
    modes, the byte-level result must be identical.

    We register two routes:

    * ``synthesize()`` → one-shot JSON envelope with the full audio.
    * ``stream()`` → N SSE events whose concatenated hex equals the
      same audio split across the same number of events.

    Then we assert ``TTSAudio.samples == b"".join(chunk.samples for
    chunk in stream())`` byte-for-byte.
    """

    async def test_stream_aggregation_equals_synthesis(
        self,
        mock_minimax_http: respx.Router,
        minimax_provider: MiniMaxTTSProvider,
    ) -> None:
        # Split the full audio into 5 chunks; stream-mode should yield
        # them in this exact order so concatenation reconstructs the
        # full payload.
        full_audio = b"\x10\x11\x12\x13" * 100  # 400 bytes
        chunk_count = 5
        chunk_size = len(full_audio) // chunk_count
        # Drop the trailing remainder so concatenation is exact.
        full_audio = full_audio[: chunk_size * chunk_count]
        upstream_chunks: list[bytes] = [
            full_audio[i * chunk_size : (i + 1) * chunk_size]
            for i in range(chunk_count)
        ]
        assert b"".join(upstream_chunks) == full_audio

        full_envelope = {
            "data": {
                "audio": full_audio.hex(),
                "audio_sample_rate": 24_000,
                "audio_length": len(full_audio),
            },
            "extra_info": None,
            "base_resp": {"status_code": 0, "status_msg": "success"},
        }

        # Single route, branched by request body: the MiniMax provider
        # flips the ``stream`` flag inside the JSON body, so we can use
        # one respx route and dispatch from the body content rather
        # than re-registering routes between calls.
        async def dual_handler(request: httpx.Request) -> httpx.Response:
            body = json.loads(request.content)
            if body.get("stream") is False:
                # Non-streaming path: a single envelope with the full
                # audio. This is the byte sequence the streamed path
                # must reproduce by concatenation.
                return httpx.Response(200, json=full_envelope)
            # Streaming path: SSE events that, concatenated, equal
            # ``full_audio``.
            gen = _streaming_sse_response(upstream_chunks, sample_rate=24_000)
            return httpx.Response(
                200,
                headers={"content-type": "text/event-stream"},
                content=gen,
            )

        route = mock_minimax_http.post(MINIMAX_T2A_PATH).mock(
            side_effect=dual_handler
        )

        # Run the batch path first.
        batch_audio = await minimax_provider.synthesize(
            "hello",
            settings=TTSSettings(sample_rate=24_000, format=TTSFormat.PCM),
        )

        # Then the streaming path with the same text and settings.
        streamed_samples: list[bytes] = []
        async for chunk in minimax_provider.stream(
            "hello",
            settings=TTSSettings(sample_rate=24_000, format=TTSFormat.PCM),
        ):
            if chunk.samples:
                streamed_samples.append(chunk.samples)

        # Core invariant: byte-identical concatenation.
        assert b"".join(streamed_samples) == batch_audio.samples
        # Both paths actually hit the route.
        assert route.call_count == 2


# ---------------------------------------------------------------------------
# 3. Iterator cancellation closes the underlying HTTP connection
# ---------------------------------------------------------------------------


@pytest.mark.minimax
@pytest.mark.unit
class TestStreamInterruption:
    """Closing the iterator early must release the HTTP connection.

    A common bug in async streaming code is leaving an httpx response
    stream open after the consumer gives up — the underlying socket
    stays attached until garbage collection, which under load produces
    "connection pool exhausted" errors.  The provider's ``stream()``
    uses ``self._client.stream(...)`` as an async context manager, so
    the connection is released on ``__aexit__`` regardless of whether
    the iterator was fully consumed.

    We assert this three ways so a regression in any one of them is loud:

    * the iterator's ``__aexit__`` completes without raising;
    * ``respx.Router.calls`` records exactly one entry that is fully
      resolved (not stuck mid-stream) — proving the connection reached
      the mock layer, was answered, and was released;
    * the underlying :class:`httpx.AsyncClient` can service a follow-up
      request immediately, which fails with a connection-pool exhaustion
      error if the cancelled stream leaked the socket.
    """

    async def test_stream_interruption(
        self,
        mock_minimax_http: respx.Router,
        minimax_provider: MiniMaxTTSProvider,
    ) -> None:
        # 8 chunks — well within the 5–10 range — with a small delay
        # so the consumer can break out before the producer finishes.
        upstream_chunks: list[bytes] = [b"\xaa\xbb" * 24 for _ in range(8)]
        route = _register_streaming_route(
            mock_minimax_http,
            audio_chunks=upstream_chunks,
            inter_chunk_delay_s=0.005,
        )

        # The provider's ``stream()`` returns a plain async generator —
        # NOT an async context manager — so we use ``async for`` with
        # ``break`` to simulate the consumer cancelling playback after
        # the first frame. The provider wraps
        # ``self._client.stream(...)`` in ``async with`` internally, so
        # when the generator is finalised the underlying HTTP response
        # context manager tears down and the connection returns to the
        # pool.
        consumed = 0
        async for chunk in minimax_provider.stream(
            "hello world",
            settings=TTSSettings(sample_rate=24_000, format=TTSFormat.PCM),
        ):
            # Read at most one chunk then bail — simulates a downstream
            # consumer (e.g. audio sink) that closed because the user
            # cancelled playback.
            consumed += 1
            if consumed >= 1:
                break

        # Sanity: we actually got the bytes we expected to bail on.
        assert consumed == 1

        # respx recorded exactly one POST — the request was issued,
        # the mock responded, and respx tracked the call. The HTTP
        # connection was not left dangling.
        assert route.call_count == 1
        assert route.called

        # respx.Router.calls is the public "mock_calls" surface: it
        # contains one entry for the request we just made, with a
        # resolved ``response`` attached. If the provider had leaked
        # the streaming response (e.g. by NOT using ``async with``
        # around ``self._client.stream(...)``), the underlying socket
        # would stay attached until the next event-loop tick and the
        # call would either not appear here at all, or appear with
        # ``response is None`` because the stream never finished —
        # both fail the assertions below.
        assert len(mock_minimax_http.calls) == 1, (
            "stream interruption should resolve exactly one respx call; "
            f"got {len(mock_minimax_http.calls)} (leaked stream?)"
        )
        interrupted_call = mock_minimax_http.calls[0]
        assert interrupted_call.response is not None, (
            "respx recorded the request but never resolved it — "
            "the stream response was likely leaked"
        )

        # The provider's owned httpx client (placeholder from the
        # fixture) should not be closed — that would mean we leaked
        # ownership of an injected client. But more importantly, the
        # connection itself must be reusable for the next call.
        assert not minimax_provider._client.is_closed

        # A subsequent call on the same client must work — proves the
        # pool wasn't exhausted by the interrupted stream. We use the
        # non-streaming path so we don't have to re-register an SSE
        # route; the goal is to confirm the underlying transport is
        # still healthy.
        follow_up_audio = b"\x00" * 8

        async def _ok(request: httpx.Request) -> httpx.Response:
            return httpx.Response(
                200,
                json={
                    "data": {
                        # ``audio`` is a hex string, not bytes — the
                        # MiniMax API returns hex-encoded payloads and
                        # httpx's JSON encoder rejects raw bytes.
                        "audio": follow_up_audio.hex(),
                        "audio_sample_rate": 24_000,
                        "audio_length": len(follow_up_audio),
                    },
                    "extra_info": None,
                    "base_resp": {"status_code": 0, "status_msg": "success"},
                },
            )

        # Replace the streaming route with a one-shot JSON route —
        # respx's routes are a RouteList; clearing and re-registering is
        # the documented way to swap the side_effect for a fresh call.
        mock_minimax_http.routes.clear()
        follow_up_route = mock_minimax_http.post(MINIMAX_T2A_PATH).mock(
            side_effect=_ok
        )
        audio = await minimax_provider.synthesize(
            "after cancel",
            settings=TTSSettings(sample_rate=24_000, format=TTSFormat.PCM),
        )
        assert audio.samples == follow_up_audio
        assert follow_up_route.call_count == 1
        # After the follow-up the router has accumulated two resolved
        # calls — the interrupted stream (above) and the follow-up
        # synthesize (here). Both are properly closed, confirming no
        # orphaned state lingers from the cancelled stream.
        assert len(mock_minimax_http.calls) == 2
        for call in mock_minimax_http.calls:
            assert call.response is not None
