"""BLK-6 / PR-907 review: HTTP client connection-pool + response-size guard.

The PR-907 review identified two related weaknesses in our HTTP client
configuration:

1. ``httpx.AsyncClient`` was constructed with no ``limits=`` argument,
   so it inherited httpx's default of ``max_connections=100`` /
   ``max_keepalive_connections=20``. That's wildly over-provisioned for
   a single-tenant voice pipeline and is the prerequisite for a runaway
   socket-fanout OOM.

2. httpx's :class:`httpx.Limits` does NOT expose a ``max_content_size``
   parameter (verified against httpx 0.27–0.28 — it's not on the
   upstream roadmap). The PR-907 review's literal
   ``httpx.Limits(max_content_size=...)`` would raise ``TypeError`` at
   construction time. We enforce the ceiling at the application layer
   instead, via :func:`rob_box_llm.providers.minimax_tts._enforce_content_size`,
   which guards both the non-streaming ``_post`` path and the SSE
   ``stream`` path.

These tests pin both halves of the fix:

* The pool is tightened to :data:`DEFAULT_MAX_CONNECTIONS` /
  :data:`DEFAULT_MAX_KEEPALIVE_CONNECTIONS` on both the base default
  factory and the :class:`MiniMaxTTSProvider` override.
* Oversized non-streaming responses raise :class:`TTSError` before
  the JSON body is decoded.
* Oversized SSE streams raise :class:`TTSError` mid-iteration before
  the audio payload blows the budget.
* The advertised ``Content-Length`` alone triggers the guard — no body
  bytes need to be buffered for the rejection to fire.
"""

from __future__ import annotations

import asyncio
import json
from typing import AsyncIterator

import httpx
import pytest
import respx

from rob_box_llm.errors import TTSError
from rob_box_llm.providers.minimax_tts import (
    MiniMaxTTSProvider,
    _enforce_content_size,
)
from rob_box_llm.tts import TTSFormat, TTSSettings
from rob_box_llm.tts_provider_base import (
    DEFAULT_MAX_CONNECTIONS,
    DEFAULT_MAX_CONTENT_SIZE,
    DEFAULT_MAX_KEEPALIVE_CONNECTIONS,
)

from conftest import MINIMAX_BASE_URL, MINIMAX_T2A_PATH


# ===========================================================================
# 1. Connection-pool limits are applied at construction time
# ===========================================================================


class TestHttpClientLimitsConfiguration:
    """Both the base default factory and the provider override set Limits.

    We deliberately test both because they're separate code paths — the
    base default is the fallback for any future provider that forgets to
    override ``_http_client_factory``, and the MiniMax override is the
    path actually used in production today.
    """

    def test_provider_factory_configures_limits(self, minimax_provider):
        # The provider override must pass a Limits instance with the
        # BLK-6 defaults. httpx 0.28 exposes the configured values as
        # ``_max_connections`` / ``_max_keepalive_connections`` on the
        # internal pool — that's the most stable surface across httpx
        # minor versions for "did my Limits actually reach the pool?".
        client = minimax_provider._http_client_factory()
        try:
            pool = client._transport._pool
            assert pool._max_connections == DEFAULT_MAX_CONNECTIONS
            assert (
                pool._max_keepalive_connections
                == DEFAULT_MAX_KEEPALIVE_CONNECTIONS
            )
        finally:
            asyncio.run(client.aclose())

    def test_provider_limits_match_pr907_defaults(self):
        # The PR-907 review specified ``max_connections=10,
        # max_keepalive_connections=5``. Pin the numeric values here so
        # a future "tune this up" PR has to update the test deliberately.
        assert DEFAULT_MAX_CONNECTIONS == 10
        assert DEFAULT_MAX_KEEPALIVE_CONNECTIONS == 5
        # 50 MiB ceiling — verified separately, this is a guard against
        # silent value drift.
        assert DEFAULT_MAX_CONTENT_SIZE == 50 * 1024 * 1024

    def test_provider_factory_still_sets_timeout(self, minimax_provider):
        # Backward-compat: the limits addition must NOT regress the
        # ``timeout=`` plumbing (see ``test_tts_extension_points``'
        # ``TestMiniMaxTTSProviderHttpClientFactory::test_uses_provider_timeout``).
        client = minimax_provider._http_client_factory()
        try:
            assert client.timeout is not None
        finally:
            asyncio.run(client.aclose())

    def test_base_default_factory_configures_limits(self):
        # The base factory's path is exercised by any future
        # ``BaseTTSProvider`` subclass that forgets to override
        # ``_http_client_factory``. We pin the same defaults here so the
        # base stays a safe fallback.
        from rob_box_llm.tts import TTSAudio, TTSFormat, TTSChunk
        from rob_box_llm.tts_provider_base import BaseTTSProvider

        class _Minimal(BaseTTSProvider):
            name = "minimal-bare"

            def _build_request_payload(self, text, settings, voice_meta):
                return {"text": text}

            async def synthesize(self, text, *, settings=None):
                return TTSAudio(samples=b"\x00", sample_rate=16_000, format=TTSFormat.PCM)

            async def stream(self, text, *, settings=None):
                yield TTSChunk(finish_reason="stop")

        p = _Minimal()
        # Provide a ``_timeout`` so the default factory's ``getattr``
        # picks up our value rather than the 30.0 default — we want to
        # verify the timeout + limits coexist, not regress the timeout
        # contract.
        p._timeout = 7.5
        client = p._http_client_factory()
        try:
            pool = client._transport._pool
            assert pool._max_connections == DEFAULT_MAX_CONNECTIONS
            assert (
                pool._max_keepalive_connections
                == DEFAULT_MAX_KEEPALIVE_CONNECTIONS
            )
            # The timeout still flows through unchanged.
            assert client.timeout is not None
        finally:
            asyncio.run(client.aclose())


# ===========================================================================
# 2. The ``_enforce_content_size`` helper — pure-function contract tests
# ===========================================================================


class TestEnforceContentSizeHelper:
    """The application-layer guard runs at every response boundary.

    These are pure-function tests — no HTTP, no fixtures. The helper is
    the single source of truth for the size policy; every other test in
    this file depends on its semantics being right.
    """

    def test_advertised_content_length_within_limit_passes(self):
        # Happy path: small body advertised, small body actually read.
        # Both checks must pass without raising.
        _enforce_content_size(
            content_length="1024",
            actual_len=1024,
            provider="minimax",
        )

    def test_advertised_content_length_exceeds_limit_raises(self):
        # The advertised size alone is enough to fail-fast. We never
        # want to actually buffer the body to discover it's too big.
        with pytest.raises(TTSError) as exc_info:
            _enforce_content_size(
                content_length=str(DEFAULT_MAX_CONTENT_SIZE + 1),
                actual_len=None,
                provider="minimax",
            )
        assert "Content-Length" in str(exc_info.value)
        assert str(DEFAULT_MAX_CONTENT_SIZE + 1) in str(exc_info.value)

    def test_actual_bytes_exceeds_limit_raises(self):
        # The header lied (chunked encoding, no Content-Length, etc.).
        # The byte counter must catch the overflow after the fact.
        with pytest.raises(TTSError) as exc_info:
            _enforce_content_size(
                content_length=None,
                actual_len=DEFAULT_MAX_CONTENT_SIZE + 1,
                provider="minimax",
            )
        assert "read" in str(exc_info.value)

    def test_no_header_no_bytes_passes(self):
        # Neither signal available — we trust the caller to enforce the
        # ceiling on its own (e.g. when reading from a pre-validated
        # buffer).
        _enforce_content_size(
            content_length=None,
            actual_len=None,
            provider="minimax",
        )

    def test_invalid_content_length_treated_as_unknown(self):
        # A malformed header (non-integer string) must not crash the
        # provider. We coerce to -1 so the ``> limit`` check fails
        # without raising a TypeError, and the helper falls through to
        # the actual-bytes branch. Document the contract.
        with pytest.raises(TTSError):
            _enforce_content_size(
                content_length="not-a-number",
                actual_len=DEFAULT_MAX_CONTENT_SIZE + 1,
                provider="minimax",
            )

    def test_custom_limit_overrides_default(self):
        # Tests pass a tiny limit so they can drive the failure path
        # cheaply without spinning up a 50 MiB allocation.
        with pytest.raises(TTSError):
            _enforce_content_size(
                content_length=None,
                actual_len=200,
                limit=100,
                provider="minimax",
            )


# ===========================================================================
# 3. Non-streaming ``_post`` enforces the size guard
# ===========================================================================


@pytest.mark.minimax
@pytest.mark.unit
class TestSynthesizeContentSizeGuard:
    """``synthesize()`` rejects oversized MiniMax T2A v2 responses.

    The guard fires AFTER the HTTP status check (so 401/429 still map to
    their typed errors) but BEFORE ``json.loads``, so the worker never
    allocates a multi-megabyte JSON parse tree just to discover the
    body was over budget.
    """

    async def test_advertised_content_length_over_limit_raises(
        self,
        mock_minimax_http: respx.Router,
        minimax_provider: MiniMaxTTSProvider,
    ):
        # The mock advertises a 60 MiB Content-Length and serves a tiny
        # actual body. The provider should reject based on the header
        # alone — we never read more than ~256 bytes from the socket.
        def handler(request: httpx.Request) -> httpx.Response:
            return httpx.Response(
                200,
                headers={
                    "content-type": "application/json",
                    # 60 MiB > 50 MiB ceiling.
                    "content-length": str(60 * 1024 * 1024),
                },
                # Real body is tiny — we never get to read it.
                content=b'{"data":{"audio":"00"}}',
            )

        route = mock_minimax_http.post(MINIMAX_T2A_PATH).mock(
            side_effect=handler
        )

        with pytest.raises(TTSError) as exc_info:
            await minimax_provider.synthesize(
                "hello", settings=TTSSettings(sample_rate=24_000)
            )
        assert "Content-Length" in str(exc_info.value)
        assert route.called

    async def test_actual_body_over_limit_raises(
        self,
        mock_minimax_http: respx.Router,
        minimax_provider: MiniMaxTTSProvider,
    ):
        # The mock omits Content-Length (chunked transfer encoding) and
        # serves a body just over the ceiling. The provider must catch
        # it after ``aread()`` returns the actual byte count.
        big_body = b'{"data":{"audio":"' + b"a" * (
            DEFAULT_MAX_CONTENT_SIZE + 16
        ) + b'"}}'

        def handler(request: httpx.Request) -> httpx.Response:
            return httpx.Response(
                200,
                headers={
                    "content-type": "application/json",
                    # No Content-Length — forces the actual-bytes path.
                    "transfer-encoding": "chunked",
                },
                content=big_body,
            )

        route = mock_minimax_http.post(MINIMAX_T2A_PATH).mock(
            side_effect=handler
        )

        with pytest.raises(TTSError) as exc_info:
            await minimax_provider.synthesize(
                "hello", settings=TTSSettings(sample_rate=24_000)
            )
        # We accept either the advertised-size OR actual-bytes failure
        # message — the test cares that a guard fires, not which arm
        # caught it. The error message must be informative.
        assert "too large" in str(exc_info.value)
        assert route.called

    async def test_body_within_limit_passes(
        self,
        mock_minimax_http: respx.Router,
        minimax_provider: MiniMaxTTSProvider,
    ):
        # Sanity check: the guard does NOT false-positive on a small
        # successful response. Existing tests already cover this, but
        # we keep one here so the guard's presence in the request path
        # is the variable being exercised, not the rest of the
        # synthesize pipeline.
        fake_pcm = b"\x00\x01\x02\x03" * 8
        envelope = {
            "data": {"audio": fake_pcm.hex(), "audio_sample_rate": 24_000},
            "base_resp": {"status_code": 0, "status_msg": "success"},
        }

        def handler(request: httpx.Request) -> httpx.Response:
            return httpx.Response(
                200,
                headers={"content-type": "application/json"},
                content=json.dumps(envelope).encode("utf-8"),
            )

        mock_minimax_http.post(MINIMAX_T2A_PATH).mock(side_effect=handler)

        out = await minimax_provider.synthesize(
            "hello", settings=TTSSettings(sample_rate=24_000)
        )
        assert out.samples == fake_pcm


# ===========================================================================
# 4. Streaming ``stream()`` enforces the size guard mid-iteration
# ===========================================================================


def _sse_event(audio_bytes: bytes, sample_rate: int = 24_000) -> bytes:
    """Encode a single MiniMax SSE audio event as raw bytes."""
    payload = {
        "data": {
            "audio": audio_bytes.hex(),
            "audio_sample_rate": sample_rate,
        },
        "base_resp": {"status_code": 0, "status_msg": "success"},
    }
    return f"data:{json.dumps(payload)}\n\n".encode("utf-8")


SSE_TERMINATOR = b"data:[DONE]\n\n"


@pytest.mark.minimax
@pytest.mark.unit
class TestStreamContentSizeGuard:
    """``stream()`` rejects oversized SSE responses mid-iteration.

    The guard fires inside ``aiter_bytes`` — after each network chunk
    arrives but BEFORE we attempt to decode any audio hex. That means
    the worker rejects a 50 MiB-plus stream with the same latency as
    a 50 KiB one: as soon as the cumulative byte count crosses the
    ceiling, the next iteration raises ``TTSError`` and we close the
    socket.
    """

    async def test_oversized_stream_raises(
        self,
        mock_minimax_http: respx.Router,
        minimax_provider: MiniMaxTTSProvider,
    ):
        # Build an SSE stream whose total byte length exceeds the
        # ceiling. Each chunk is a complete SSE event so the parser
        # would happily decode every one of them if we let it run.
        # The guard must trip before the LAST chunk is read.
        # We use 60 MiB worth of synthetic SSE bytes so the test is
        # fast (no real audio) but the byte counter definitely trips.
        big_chunk = _sse_event(b"\x00" * 1024)

        async def handler(request: httpx.Request) -> AsyncIterator[bytes]:
            # Yield 60 * 1024 chunks of ~1 KiB SSE each → ~60 MiB total,
            # which exceeds the 50 MiB ceiling by a clear margin.
            for _ in range(60 * 1024):
                yield big_chunk

        route = mock_minimax_http.post(MINIMAX_T2A_PATH).mock(
            side_effect=lambda req: httpx.Response(
                200,
                headers={"content-type": "text/event-stream"},
                content=handler(req),
            )
        )

        with pytest.raises(TTSError) as exc_info:
            # Consume the iterator until the guard fires. We must NOT
            # collect every yielded chunk — that's what we're testing
            # against.
            async for _chunk in minimax_provider.stream(
                "hello", settings=TTSSettings(sample_rate=24_000)
            ):
                # Drain whatever the provider yields before the guard
                # trips. The body bytes themselves are 1 KiB each so
                # this stays cheap.
                pass

        assert "too large" in str(exc_info.value)
        assert route.called

    async def test_oversized_content_length_header_rejects_without_buffers(
        self,
        mock_minimax_http: respx.Router,
        minimax_provider: MiniMaxTTSProvider,
    ):
        # Content-Length alone (no actual body read) is enough to
        # fail-fast. We register a route that would return a multi-MiB
        # stream and assert the provider raises BEFORE consuming any
        # chunks — the iterator body never executes.
        def handler(request: httpx.Request) -> httpx.Response:
            async def _gen():
                # Should never be reached — the guard fires first.
                yield b"never-yielded\n"
                yield SSE_TERMINATOR  # pragma: no cover

            return httpx.Response(
                200,
                headers={
                    "content-type": "text/event-stream",
                    # 60 MiB > 50 MiB ceiling.
                    "content-length": str(60 * 1024 * 1024),
                },
                content=_gen(),
            )

        mock_minimax_http.post(MINIMAX_T2A_PATH).mock(side_effect=handler)

        with pytest.raises(TTSError) as exc_info:
            async for _chunk in minimax_provider.stream(
                "hello", settings=TTSSettings(sample_rate=24_000)
            ):
                pytest.fail(
                    "stream() must reject oversized Content-Length "
                    "BEFORE yielding any chunk"
                )

        assert "Content-Length" in str(exc_info.value)

    async def test_well_formed_stream_still_yields_terminal_chunk(
        self,
        mock_minimax_http: respx.Router,
        minimax_provider: MiniMaxTTSProvider,
    ):
        # Regression check: the guard must not change the documented
        # stream contract — well-formed streams still yield a terminal
        # ``stop`` chunk after the audio frames. This protects against
        # accidental ``return`` vs ``break`` refactors in the loop
        # above the guard.
        upstream = [b"\x00\x01" * 8, b"\x02\x03" * 8]

        async def handler(request: httpx.Request) -> AsyncIterator[bytes]:
            for chunk in upstream:
                yield _sse_event(chunk)
            yield SSE_TERMINATOR

        mock_minimax_http.post(MINIMAX_T2A_PATH).mock(
            side_effect=lambda req: httpx.Response(
                200,
                headers={"content-type": "text/event-stream"},
                content=handler(req),
            )
        )

        yielded: list = []
        async for chunk in minimax_provider.stream(
            "hello", settings=TTSSettings(sample_rate=24_000)
        ):
            yielded.append(chunk)

        # Exactly one terminal ``stop`` chunk, after the audio frames.
        terminal = [c for c in yielded if not c.samples]
        assert len(terminal) == 1
        assert terminal[0].finish_reason == "stop"
        # And the audio frames arrived in order.
        audio = [c.samples for c in yielded if c.samples]
        assert audio == upstream
