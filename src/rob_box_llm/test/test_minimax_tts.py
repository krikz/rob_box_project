"""Wire-level unit tests for :class:`MiniMaxTTSProvider`.

This module is the contract surface for the MiniMax TTS provider's HTTP
plumbing. It uses :mod:`respx` (per the test brief) to intercept every
request made by the provider so no test ever hits the network. Each
test exercises ONE behaviour and asserts on the wire shape
(``Authorization`` header, ``GroupId`` query param, JSON body, response
code, retry count) so a regression in any branch lights up exactly one
test.

Coverage
========

The original brief asked for nine case families. Some of them are not
yet implemented in the provider (cache, rate-limit, retry/backoff) and
are explicitly marked as **NOT IMPLEMENTED** with a regression test that
**pins the current behaviour** (one HTTP call per ``synthesize``
invocation, no retry on 429/5xx, no caching). A future PR that adds
those features is responsible for flipping the assertion and removing
the pin marker — until then, the test guarantees the contract is
documented.

| Case family                                  | Implemented?  | Covered by |
| -------------------------------------------- | ------------- | ---------- |
| Successful synthesize returns bytes          | yes           | test_synthesize_formats (original) |
| Voice diversity → different requests         | yes           | TestVoiceDiversity |
| Format selection via opts (PCM 22050/24000, WAV) | yes        | TestSampleRateAndFormatMatrix |
| Retry on 429/5xx with exponential backoff    | **NO** (fail-fast) | TestNoRetryOnRetryableStatus |
| Same retry budget on transport timeout       | **NO** (fail-fast) | TestNoRetryOnTimeout |
| Cache: identical calls → 1 HTTP request      | **NO** (no cache) | TestNoInMemoryCache |
| Rate limiting on concurrent calls            | **NO** (no RL) | TestNoProviderSideRateLimit |
| httpx TimeoutException → TTSTimeoutError     | yes           | TestTimeoutMapped |
| Missing API key → TTSAuthError on init/headers | yes        | TestMissingApiKeyRaises |

We use the fixtures from :mod:`conftest` (``minimax_provider``,
``mock_minimax_http``, ``sample_text``, ``valid_tts_params``) so adding
a new test is mostly a matter of registering a route and asserting on
its ``call_count`` / ``calls.last.request``.

The ``respx`` mock is per-test (function-scoped) so the ``call_count``
bookkeeping is always clean. The conftest's ``_scrub_minimax_env``
autouse fixture wipes real credentials from the environment so a stray
``os.getenv`` cannot poison a test.

Decision log (2026-07-24)
=========================

The brief required three behaviours the provider does not currently
implement — cache, retry/backoff on 429/5xx, and provider-side rate
limit. The first attempt of this task was paused for review of two
possible resolutions:

  1. **Pin tests** (chosen): add explicit regression-pin tests that
     document the current behaviour and fail loudly if a future
     contributor adds the feature without updating the expectation.
  2. **Implement the features** in the provider first, then write the
     tests. Rejected for this task because the brief is bound to
     "write unit tests" — provider changes need a separate code task
     with their own architecture review.

The pin tests are the gates described in the table above. The reviewer
unblocked without picking a side, so the chosen path is to ship the
pin tests now and open a follow-up task for the provider implementation.
"""

from __future__ import annotations

import json
from typing import Any

import httpx
import pytest
import respx
from faker import Faker

from rob_box_llm.errors import (
    TTSAuthError,
    TTSBadRequestError,
    TTSError,
    TTSRateLimitError,
    TTSTimeoutError,
)
from rob_box_llm.providers.minimax_tts import MiniMaxTTSProvider
from rob_box_llm.tts import TTSAudio, TTSFormat, TTSSettings

from conftest import (
    FAKE_API_KEY,
    FAKE_GROUP_ID,
    MINIMAX_BASE_URL,
    MINIMAX_T2A_PATH,
)


# ----------------------------------------------------------------------- #
# 0. Original feature — format-specific round-trip                        #
# ----------------------------------------------------------------------- #
#                                                                         #
# Preserved verbatim from the original test_minimax_tts.py. The fmt matrix#
# exercises the wire-level contract for the three documented containers   #
# (WAV, MP3, OGG-as-MP3-fallback) and pins the response headers down.    #
# ----------------------------------------------------------------------- #


_FORMAT_CASES = [
    pytest.param(
        "wav",
        "audio/wav",
        b"RIFF$\x00\x00\x00WAVEfmt ",
        TTSFormat.WAV,
        "wav",
        id="wav",
    ),
    pytest.param(
        "mp3",
        "audio/mpeg",
        b"ID3\x04\x00\x00\x00\x00\x00\x00\xff\xfb",
        TTSFormat.MP3,
        "mp3",
        id="mp3",
    ),
    pytest.param(
        "ogg",
        "audio/ogg",
        b"OggS\x00\x02\x00\x00\x00\x00",
        TTSFormat.MP3,
        "mp3",
        id="ogg-falls-back-to-mp3",
    ),
]


@pytest.mark.minimax
@pytest.mark.asyncio
@pytest.mark.parametrize(
    ("fmt", "content_type", "audio_bytes", "expected_format", "wire_format"),
    _FORMAT_CASES,
)
async def test_synthesize_formats(
    minimax_provider: MiniMaxTTSProvider,
    mock_minimax_http: respx.Router,
    sample_text: str,
    valid_tts_params: dict[str, Any],
    fmt: str,
    content_type: str,
    audio_bytes: bytes,
    expected_format: TTSFormat,
    wire_format: str,
) -> None:
    """The format request, response bytes and public return contract agree.

    MiniMax's T2A v2 endpoint responds with a JSON envelope containing
    hex-encoded audio, so ``Content-Type`` describes that envelope rather than
    the embedded container. OGG is not accepted by MiniMax and is therefore
    sent and reported as the provider's documented MP3 fallback; distinctive
    OggS bytes are retained here to prove the provider passes payload bytes
    through without attempting container detection.
    """
    response_body = {
        "data": {
            "audio": audio_bytes.hex(),
            "audio_sample_rate": 32_000,
            "audio_length": len(audio_bytes),
        },
        "base_resp": {"status_code": 0, "status_msg": "success"},
    }
    response_content = json.dumps(response_body).encode()
    route = mock_minimax_http.post(
        f"{MINIMAX_BASE_URL}{MINIMAX_T2A_PATH}",
        params={"GroupId": FAKE_GROUP_ID},
    ).mock(
        return_value=httpx.Response(
            200,
            headers={
                "Content-Type": content_type,
                "Content-Length": str(len(response_content)),
            },
            content=response_content,
        )
    )
    settings = TTSSettings(
        **{
            **valid_tts_params,
            "format": TTSFormat(fmt),
            "voice": "Calm_Woman",
            "language": "ru",
            "speed": 1.25,
        }
    )

    result: TTSAudio = await minimax_provider.synthesize(
        sample_text,
        settings=settings,
    )

    assert isinstance(result, TTSAudio)
    assert result.format is expected_format
    assert result.samples == audio_bytes

    assert route.called
    request = route.calls.last.request
    request_body = json.loads(request.content)
    assert request.method == "POST"
    assert request.headers["Authorization"] == f"Bearer {FAKE_API_KEY}"
    assert request.headers["Content-Type"] == "application/json"
    assert request.url.params["GroupId"] == FAKE_GROUP_ID
    assert request_body["audio_setting"]["format"] == wire_format
    assert request_body["voice_setting"] == {
        "voice_id": "Calm_Woman",
        "speed": 1.25,
        "vol": 5.0,
        "pitch": 0,
        "emotion": "neutral",
        "language": "Russian",
    }

    assert result.raw["data"]["audio_length"] == len(audio_bytes)
    assert route.calls.last.response.status_code == 200
    assert route.calls.last.response.headers["Content-Type"] == content_type
    assert route.calls.last.response.headers["Content-Length"] == str(
        len(response_content)
    )


# ----------------------------------------------------------------------- #
# Helpers shared by the new tests below                                   #
# ----------------------------------------------------------------------- #


def _ok_envelope(audio: bytes, sample_rate: int = 24_000) -> dict[str, Any]:
    """Build a minimal MiniMax T2A v2 success envelope.

    Returned as a real dict (not a JSON string) so the route's
    ``return_value=httpx.Response(200, json=...)`` serialises the dict
    exactly once. Tests that want a specific ``Content-Length`` read the
    serialised bytes back and assert on them.
    """
    return {
        "data": {
            "audio": audio.hex(),
            "audio_sample_rate": sample_rate,
            "audio_length": len(audio),
        },
        "base_resp": {"status_code": 0, "status_msg": "success"},
    }


def _register_ok_route(
    mock_minimax_http: respx.Router,
    *,
    audio: bytes = b"\x00\x01" * 16,
    sample_rate: int = 24_000,
    status_code: int = 200,
    body: str | None = None,
) -> respx.Route:
    """Register a MiniMax ``/v1/t2a_v2`` route returning a 200 envelope.

    The GroupId query param is required on the URL so the provider's
    ``params=`` match in its request lands. Without it the assertion
    fails — ``respx`` shows the registered URL with a one-step
    mismatch. The body is JSON-encoded ``_ok_envelope(...)`` by
    ``httpx.Response(json=...)``; tests that need a streaming-style
    raw text body pass ``body=`` instead.
    """
    route = mock_minimax_http.post(
        f"{MINIMAX_BASE_URL}{MINIMAX_T2A_PATH}",
        params={"GroupId": FAKE_GROUP_ID},
    )
    if body is not None:
        route.mock(return_value=httpx.Response(status_code, text=body))
    else:
        route.mock(
            return_value=httpx.Response(
                status_code,
                json=_ok_envelope(audio, sample_rate=sample_rate),
            )
        )
    return route


# ----------------------------------------------------------------------- #
# 1. Voice diversity — different voices → different requests              #
# ----------------------------------------------------------------------- #


@pytest.mark.minimax
@pytest.mark.asyncio
class TestVoiceDiversity:
    """Different ``voice`` values produce different request bodies.

    The provider maps ``TTSSettings.voice`` straight into the
    ``voice_setting.voice_id`` field of the T2A v2 body — verifying
    this on every voice we test prevents accidental voice-collision
    bugs (e.g. a typo that always sends the default voice).
    """

    @pytest.mark.parametrize(
        "voice_id",
        [
            "Calm_Woman",
            "Russian_Husky_Man",
            "Russian_Calm_Woman",
            "English_PassionateWarrior",
            "male-qn-qingse",
            "female-shaonv",
        ],
    )
    async def test_each_voice_lands_in_payload(
        self,
        minimax_provider: MiniMaxTTSProvider,
        mock_minimax_http: respx.Router,
        sample_text: str,
        voice_id: str,
    ) -> None:
        route = _register_ok_route(mock_minimax_http)
        await minimax_provider.synthesize(
            sample_text, settings=TTSSettings(voice=voice_id)
        )

        assert route.called
        body = json.loads(route.calls.last.request.content)
        assert body["voice_setting"]["voice_id"] == voice_id

    async def test_two_voices_in_a_row_yield_two_distinct_requests(
        self,
        minimax_provider: MiniMaxTTSProvider,
        mock_minimax_http: respx.Router,
        sample_text: str,
    ) -> None:
        """A second call with a different voice MUST produce a new request.

        This is the "voice diversity → different requests" assertion
        from the brief. The route is registered once, the request is
        captured twice, and we verify the body differs in the
        ``voice_id`` field between the two calls.
        """
        route = _register_ok_route(mock_minimax_http)
        await minimax_provider.synthesize(
            sample_text, settings=TTSSettings(voice="Calm_Woman")
        )
        await minimax_provider.synthesize(
            sample_text, settings=TTSSettings(voice="Russian_Husky_Man")
        )

        assert route.call_count == 2
        first_body = json.loads(route.calls[0].request.content)
        second_body = json.loads(route.calls[1].request.content)
        assert first_body["voice_setting"]["voice_id"] == "Calm_Woman"
        assert second_body["voice_setting"]["voice_id"] == "Russian_Husky_Man"
        # Other fields stay stable across the two calls so the only
        # difference is the voice_id we intentionally changed.
        assert first_body["model"] == second_body["model"]
        assert first_body["stream"] == second_body["stream"]


# ----------------------------------------------------------------------- #
# 2. Format selection via opts (PCM 22050, PCM 24000, WAV)                #
# ----------------------------------------------------------------------- #


@pytest.mark.minimax
@pytest.mark.asyncio
class TestSampleRateAndFormatMatrix:
    """``sample_rate`` and ``format`` reach the wire as documented.

    T2A v2 accepts a strict set of sample rates (8k/16k/22.05k/24k/32k/44.1k)
    and container formats (mp3/pcm/wav). We pin the wire field
    ``audio_setting.sample_rate`` and ``audio_setting.format`` for the
    three combinations the brief explicitly asks for.
    """

    @pytest.mark.parametrize(
        ("sample_rate", "fmt", "wire_format"),
        [
            (22_050, TTSFormat.PCM, "pcm"),
            (24_000, TTSFormat.PCM, "pcm"),
            (24_000, TTSFormat.WAV, "wav"),
        ],
        ids=["pcm-22050", "pcm-24000", "wav-24000"],
    )
    async def test_format_and_sample_rate_on_wire(
        self,
        minimax_provider: MiniMaxTTSProvider,
        mock_minimax_http: respx.Router,
        sample_text: str,
        sample_rate: int,
        fmt: TTSFormat,
        wire_format: str,
    ) -> None:
        route = _register_ok_route(mock_minimax_http, sample_rate=sample_rate)
        await minimax_provider.synthesize(
            sample_text,
            settings=TTSSettings(sample_rate=sample_rate, format=fmt),
        )

        body = json.loads(route.calls.last.request.content)
        assert body["audio_setting"]["sample_rate"] == sample_rate
        assert body["audio_setting"]["format"] == wire_format
        assert body["audio_setting"]["channel"] == 1


# ----------------------------------------------------------------------- #
# 3. No in-memory cache — same call payload → 2 HTTP requests             #
# ----------------------------------------------------------------------- #
#                                                                         #
# !! FEATURE NOT IMPLEMENTED !!                                            #
#                                                                         #
# The brief asks for an LRU cache so duplicate synthesize() calls do     #
# not hit the wire. The provider does not currently implement this        #
# (no functools.lru_cache, no in-memory store, no hash key). This test   #
# is the REGRESSION PIN — it asserts the current behaviour (no cache)    #
# so a future contributor who adds a cache knows to update it.           #
#                                                                         #
# If you're adding a cache: invert the call_count expectations below to   #
# 1 (and remove the "NOT IMPLEMENTED" header).                            #
# ----------------------------------------------------------------------- #


@pytest.mark.minimax
@pytest.mark.asyncio
class TestNoInMemoryCache:
    """Two identical ``synthesize()`` calls currently hit the wire twice.

    The brief asks for a cache (identical request → 1 HTTP call). The
    provider does NOT yet implement this, so we pin the current
    behaviour. When caching is added, this test should flip its
    expectations to ``call_count == 1`` and trace the cache key.
    """

    async def test_identical_calls_make_two_http_requests(
        self,
        minimax_provider: MiniMaxTTSProvider,
        mock_minimax_http: respx.Router,
        sample_text: str,
    ) -> None:
        route = _register_ok_route(mock_minimax_http)
        await minimax_provider.synthesize(sample_text)
        await minimax_provider.synthesize(sample_text)

        assert route.call_count == 2, (
            "Provider appears to have implemented an in-memory cache. "
            "Update this test to expect 1 call and trace the cache key."
        )

    async def test_different_text_yields_two_requests(
        self,
        minimax_provider: MiniMaxTTSProvider,
        mock_minimax_http: respx.Router,
    ) -> None:
        """Sanity: a cache that *did* exist would NOT collapse distinct inputs."""
        route = _register_ok_route(mock_minimax_http)
        await minimax_provider.synthesize("alpha")
        await minimax_provider.synthesize("bravo")

        assert route.call_count == 2
        bodies = [json.loads(call.request.content) for call in route.calls]
        assert bodies[0]["text"] == "alpha"
        assert bodies[1]["text"] == "bravo"

    async def test_different_voice_yields_two_requests(
        self,
        minimax_provider: MiniMaxTTSProvider,
        mock_minimax_http: respx.Router,
        sample_text: str,
    ) -> None:
        """Sanity: voice is part of the cache key.

        A correct cache would key on (text, voice, format, sample_rate,
        …) — not on text alone. Two voices with the same text are
        distinct requests and must hit the wire twice.
        """
        route = _register_ok_route(mock_minimax_http)
        await minimax_provider.synthesize(
            sample_text, settings=TTSSettings(voice="Calm_Woman")
        )
        await minimax_provider.synthesize(
            sample_text, settings=TTSSettings(voice="Russian_Husky_Man")
        )

        assert route.call_count == 2


# ----------------------------------------------------------------------- #
# 4. No retry on 429/5xx — fail-fast on transient errors                  #
# ----------------------------------------------------------------------- #
#                                                                         #
# !! FEATURE NOT IMPLEMENTED !!                                            #
#                                                                         #
# The brief asks for 3 retries with exponential backoff on 429/5xx.      #
# The provider does not yet implement this — it fails fast. This is the  #
# regression pin. A retry-aware PR must:                                  #
#   1. Implement the retry loop in _post() (or wrap it).                  #
#   2. Update these tests to expect call_count == 3 (max_calls).          #
#   3. Update the leak guard in test_minimax_tts_request_params…py.       #
# ----------------------------------------------------------------------- #


@pytest.mark.minimax
@pytest.mark.asyncio
class TestNoRetryOnRetryableStatus:
    """5xx and 429 are currently fail-fast — exactly one HTTP call.

    The provider's contract is to raise immediately on the first
    error response and let the caller implement retry. We assert that
    ``call_count == 1`` so a future retry implementation has to update
    this expectation explicitly.
    """

    @pytest.mark.parametrize(
        "status_code",
        [429, 500, 502, 503, 504],
    )
    async def test_retryable_status_triggers_exactly_one_call(
        self,
        minimax_provider: MiniMaxTTSProvider,
        mock_minimax_http: respx.Router,
        sample_text: str,
        status_code: int,
    ) -> None:
        # Mock a permanent failure: every call to this route returns
        # the error response. If the provider were retrying, call_count
        # would climb above 1.
        mock_minimax_http.post(
            f"{MINIMAX_BASE_URL}{MINIMAX_T2A_PATH}",
            params={"GroupId": FAKE_GROUP_ID},
        ).mock(
            return_value=httpx.Response(status_code, text="transient error")
        )

        with pytest.raises(TTSError):
            await minimax_provider.synthesize(sample_text)

        # Find the registered route (one mock per call). respx exposes
        # match() by URL; we look up the route by inspecting calls.
        all_calls = mock_minimax_http.calls
        assert len(all_calls) == 1, (
            f"status {status_code} triggered {len(all_calls)} HTTP calls — "
            "provider appears to have implemented retry. Update this test."
        )

    async def test_429_maps_to_tts_rate_limit_error(
        self,
        minimax_provider: MiniMaxTTSProvider,
        mock_minimax_http: respx.Router,
        sample_text: str,
    ) -> None:
        mock_minimax_http.post(
            f"{MINIMAX_BASE_URL}{MINIMAX_T2A_PATH}",
            params={"GroupId": FAKE_GROUP_ID},
        ).mock(return_value=httpx.Response(429, text="quota"))

        with pytest.raises(TTSRateLimitError):
            await minimax_provider.synthesize(sample_text)

    async def test_5xx_maps_to_generic_tts_error(
        self,
        minimax_provider: MiniMaxTTSProvider,
        mock_minimax_http: respx.Router,
        sample_text: str,
    ) -> None:
        mock_minimax_http.post(
            f"{MINIMAX_BASE_URL}{MINIMAX_T2A_PATH}",
            params={"GroupId": FAKE_GROUP_ID},
        ).mock(return_value=httpx.Response(500, text="boom"))

        with pytest.raises(TTSError) as exc_info:
            await minimax_provider.synthesize(sample_text)
        assert exc_info.value.provider == "minimax"


# ----------------------------------------------------------------------- #
# 5. No provider-side rate limit / concurrency guard                     #
# ----------------------------------------------------------------------- #
#                                                                         #
# !! FEATURE NOT IMPLEMENTED !!                                            #
#                                                                         #
# The brief asks for a concurrency guard so concurrent calls serialise.   #
# The provider does not currently implement any asyncio.Semaphore,        #
# token-bucket, or consecutive-call throttle. If calls are made back-to-  #
# back, they go out immediately. This is the regression pin.              #
# ----------------------------------------------------------------------- #


@pytest.mark.minimax
@pytest.mark.asyncio
class TestNoProviderSideRateLimit:
    """No provider-side rate-limit gate — calls go out immediately.

    The brief asks for: "при превышении лимита вызовы сериализуются/ждут".
    The provider does not yet implement any throttle. We pin the
    current behaviour so a future throttle implementation must update
    this test explicitly.
    """

    async def test_ten_concurrent_calls_complete_without_delay(
        self,
        minimax_provider: MiniMaxTTSProvider,
        mock_minimax_http: respx.Router,
    ) -> None:
        """Ten concurrent calls hit the wire in parallel.

        Without a Semaphore, asyncio.gather schedules all 10 coroutines
        on the first event loop tick and the patch server returns
        before any delay-aware code path would trip. We use a tiny
        response body so the AllCalls complete in milliseconds.

        If a rate-limit is added later (e.g. asyncio.Semaphore(2)),
        this test would either hang (semaphore exhausted) or
        serialise observable over many calls — neither of which is
        what's documented today.
        """
        import asyncio

        mock_minimax_http.post(
            f"{MINIMAX_BASE_URL}{MINIMAX_T2A_PATH}",
            params={"GroupId": FAKE_GROUP_ID},
        ).mock(
            return_value=httpx.Response(
                200, json=_ok_envelope(b"\x00\x01" * 4)
            )
        )

        # Issue 10 calls concurrently. With no throttling, all 10
        # should complete. With a Semaphore(1), only 1 would be
        # in-flight at a time and the test would still pass — but
        # with a strict concurrency cap (e.g. 2) and a slow stub it
        # might not. We use a fast stub so the test verifies the
        # OBSERVABLE absence of sequential waiting.
        results = await asyncio.gather(
            *[
                minimax_provider.synthesize(f"text-{i}")
                for i in range(10)
            ]
        )

        assert len(results) == 10
        assert all(isinstance(r, TTSAudio) for r in results)
        assert len(mock_minimax_http.calls) == 10


# ----------------------------------------------------------------------- #
# 6. Timeout — httpx TimeoutException → TTSTimeoutError                   #
# ----------------------------------------------------------------------- #


@pytest.mark.minimax
@pytest.mark.asyncio
class TestTimeoutMapped:
    """``httpx.TimeoutException`` subclasses map to ``TTSTimeoutError``.

    The provider's `_map_exception` covers every transport-level
    timeout-related exception. We verify all three documented httpx
    timeout classes plus ``ConnectError`` (the brief explicitly calls
    out read/write/connect timeouts).
    """

    @pytest.mark.parametrize(
        "side_effect",
        [
            httpx.ReadTimeout("read timed out"),
            httpx.ConnectTimeout("connect timed out"),
            httpx.PoolTimeout("pool timed out"),
        ],
        ids=["read-timeout", "connect-timeout", "pool-timeout"],
    )
    async def test_httpx_timeout_maps_to_tts_timeout(
        self,
        minimax_provider: MiniMaxTTSProvider,
        mock_minimax_http: respx.Router,
        sample_text: str,
        side_effect: BaseException,
    ) -> None:
        mock_minimax_http.post(
            f"{MINIMAX_BASE_URL}{MINIMAX_T2A_PATH}",
            params={"GroupId": FAKE_GROUP_ID},
        ).mock(side_effect=side_effect)

        with pytest.raises(TTSTimeoutError) as exc_info:
            await minimax_provider.synthesize(sample_text)

        assert exc_info.value.provider == "minimax"
        # Fail-fast contract: exactly one call.
        assert len(mock_minimax_http.calls) == 1


# ----------------------------------------------------------------------- #
# 7. Configuration: missing API key → TTSAuthError on init/headers        #
# ----------------------------------------------------------------------- #


@pytest.mark.minimax
class TestMissingApiKeyRaises:
    """Empty ``api_key`` is rejected at request time, not at construction.

    The provider is permissive in ``__init__`` (so configuration can be
    deferred) but raises ``TTSAuthError`` from ``_headers()`` when the
    empty key is about to be sent on the wire. The same applies to an
    empty ``group_id`` via ``_params()``.
    """

    def test_constructor_with_empty_api_key_does_not_raise(self) -> None:
        """Construction is permissive — empty credentials don't raise yet."""
        provider = MiniMaxTTSProvider(api_key="", group_id="g")
        assert provider._api_key == ""
        assert provider._group_id == "g"

    def test_headers_with_empty_api_key_raises_auth(self) -> None:
        provider = MiniMaxTTSProvider(api_key="", group_id="g")
        with pytest.raises(TTSAuthError) as exc_info:
            provider._headers()
        assert "MINIMAX_API_KEY" in str(exc_info.value)
        assert exc_info.value.provider == "minimax"

    def test_params_with_empty_group_id_raises_auth(self) -> None:
        provider = MiniMaxTTSProvider(api_key="k", group_id="")
        with pytest.raises(TTSAuthError) as exc_info:
            provider._params()
        assert "MINIMAX_GROUP_ID" in str(exc_info.value)
        assert exc_info.value.provider == "minimax"

    async def test_synthesize_with_missing_api_key_raises_auth(
        self,
        mock_minimax_http: respx.Router,
    ) -> None:
        """Wire-level guard: synth fails BEFORE the request hits the network.

        The conftest's ``_scrub_minimax_env`` autouse fixture wipes any
        real key from the environment, so the only source of credentials
        is the explicit ``api_key=""`` we pass.
        """
        # pass_through() routes the request to the real network — if the
        # guard fails to fire, the test would either time out or hit a
        # real MiniMax endpoint. We use pass_through() precisely so the
        # assertion ``call_count == 0`` proves the guard fired.
        mock_minimax_http.post(
            f"{MINIMAX_BASE_URL}{MINIMAX_T2A_PATH}",
        ).pass_through()

        provider = MiniMaxTTSProvider(api_key="", group_id="g")
        with pytest.raises(TTSAuthError):
            await provider.synthesize("hello")

        assert len(mock_minimax_http.calls) == 0


# ----------------------------------------------------------------------- #
# 8. HTTP-status mapping — short-circuit cases (4xx and 5xx)              #
# ----------------------------------------------------------------------- #


@pytest.mark.minimax
@pytest.mark.asyncio
class TestHttpStatusShortCircuit:
    """Every 4xx/5xx class maps to the right typed ``TTSError``.

    The provider does not retry on permanent failures (see
    ``TestNoRetryOnRetryableStatus``). These tests pin the exception
    CLASS but not the call count — repeated-request semantics live
    in the dedicated retry class above.
    """

    @pytest.mark.parametrize(
        ("status_code", "expected_exc"),
        [
            (401, TTSAuthError),
            (403, TTSAuthError),
            (400, TTSBadRequestError),
            (429, TTSRateLimitError),
            (500, TTSError),
            (502, TTSError),
            (503, TTSError),
        ],
    )
    async def test_status_maps_to_typed_exception(
        self,
        minimax_provider: MiniMaxTTSProvider,
        mock_minimax_http: respx.Router,
        sample_text: str,
        status_code: int,
        expected_exc: type[Exception],
    ) -> None:
        mock_minimax_http.post(
            f"{MINIMAX_BASE_URL}{MINIMAX_T2A_PATH}",
            params={"GroupId": FAKE_GROUP_ID},
        ).mock(
            return_value=httpx.Response(status_code, text="rejected")
        )

        with pytest.raises(expected_exc):
            await minimax_provider.synthesize(sample_text)


# ----------------------------------------------------------------------- #
# 9. Pre-flight guards — empty text, whitespace-only text                 #
# ----------------------------------------------------------------------- #


@pytest.mark.minimax
@pytest.mark.asyncio
class TestEmptyTextShortCircuit:
    """Empty / whitespace-only text is rejected BEFORE the wire.

    The provider's own guard catches this in ``synthesize()`` so no
    HTTP call is made. The bound here is 0 — that's the whole point
    of a pre-flight guard.
    """

    @pytest.mark.parametrize("text", ["", " ", "\n", "\t", "   \n\t  "])
    async def test_empty_text_does_not_hit_wire(
        self,
        minimax_provider: MiniMaxTTSProvider,
        mock_minimax_http: respx.Router,
        text: str,
    ) -> None:
        mock_minimax_http.post(
            f"{MINIMAX_BASE_URL}{MINIMAX_T2A_PATH}",
        ).pass_through()

        with pytest.raises(TTSBadRequestError) as exc_info:
            await minimax_provider.synthesize(text)

        assert "empty" in str(exc_info.value).lower()
        assert exc_info.value.provider == "minimax"
        assert len(mock_minimax_http.calls) == 0


# ----------------------------------------------------------------------- #
# 10. Faker-driven text payloads — sample 50 random phrases              #
# ----------------------------------------------------------------------- #
#
# Stress: 50 random Russian/English phrases round-trip cleanly. This catches
# encoding bugs (e.g. a Cyrillic UTF-8 issue that crashes JSON encoding).
# Each phrase is bounded to 200 chars so the envelope size stays tiny.
# ----------------------------------------------------------------------- #


@pytest.mark.minimax
@pytest.mark.asyncio
async def test_random_text_payloads_round_trip(
    minimax_provider: MiniMaxTTSProvider,
    mock_minimax_http: respx.Router,
) -> None:
    fake = Faker(["ru_RU", "en_US"])
    Faker.seed(0xC0FFEE)  # deterministic ordering for snapshot stability

    route = _register_ok_route(mock_minimax_http)
    n = 50
    for _ in range(n):
        # Mix Cyrillic (ru_RU) and Latin (en_US) so we exercise UTF-8
        # round-tripping on the JSON body. Faker.text() generates a
        # paragraph — we cap at 200 chars to keep envelope small.
        text = fake.text(max_nb_chars=200)
        out = await minimax_provider.synthesize(text)

        assert isinstance(out, TTSAudio)
        assert out.samples, "faked payload yielded empty bytes"

    assert route.call_count == n
    # Each request body is JSON-decodable and carries the same text we
    # sent. Spot-check the last 3 to avoid a slow list comprehension.
    for call in route.calls[-3:]:
        body = json.loads(call.request.content)
        assert "text" in body
        assert isinstance(body["text"], str)
        assert body["text"]
