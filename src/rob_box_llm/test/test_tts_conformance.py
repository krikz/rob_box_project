"""Conformance suite — every TTS provider must satisfy the same contract.

This is the cross-provider equivalent of :mod:`test_tts_provider_contract`,
which only exercises :class:`FakeTTSProvider` against the ABC. Here we
parametrize over the concrete providers actually shipped in
``rob_box_llm.providers`` and run the same behaviour checks against each.

Why a separate file rather than expanding ``test_tts_provider_contract.py``?

* ``test_tts_provider_contract`` is about the *abstract* contract — the
  ABC must reject incomplete subclasses, the fake must obey the contract.
* This file is about *implementations* — every concrete provider must
  satisfy the contract over a representative set of inputs (audio formats,
  voices, languages, errors). Parametrising on provider class is the
  cheapest way to grow that matrix.

How providers plug in:

Each entry in ``CONFORMANCE_PROVIDERS`` is a 2-tuple:

    (provider_factory, settings_factory)

``provider_factory()`` returns a fresh, ready-to-call :class:`TTSProvider`
instance — for ``FakeTTSProvider`` it's just the constructor; for
``MiniMaxTTSProvider`` it builds a respx-mocked client so no live HTTP
is performed.

``settings_factory()`` returns a :class:`TTSSettings` instance tailored to
the provider's wire format (e.g. MiniMax needs a ``voice_id`` that the
provider doesn't enforce for the fake). Tests use the union of fields.

Adding a new provider:

1. Add a (factory, settings) pair to ``CONFORMANCE_PROVIDERS``.
2. Ensure the factory raises no warnings under the conformance suite.
3. Confirm the new entry passes every test below — failing one means
   the provider doesn't honour the ABC contract, not that the test is
   wrong.

What this suite does NOT cover (left to provider-specific tests):

* Provider-specific payload shape (e.g. MiniMax's ``voice_setting.voice_id``)
* Provider-specific error-code → exception mapping
* Streaming chunk granularities beyond "must emit at least one
  ``finish_reason``-bearing terminal chunk"

Those belong in :mod:`test_minimax_tts_provider` etc.
"""

from __future__ import annotations

import inspect
import json
from typing import Any, Awaitable, Callable, Iterator

import httpx
import pytest
import respx

from rob_box_llm.errors import TTSBadRequestError
from rob_box_llm.providers.minimax_tts import MiniMaxTTSProvider
from rob_box_llm.tts import (
    FakeTTSProvider,
    TTSAudio,
    TTSChunk,
    TTSFormat,
    TTSProvider,
    TTSSettings,
)

# Mirror the conftest sentinel credentials and endpoint constants here so
# this test module can be imported standalone (e.g. by IDEs, by ad-hoc
# ``python -m pytest test_tts_conformance.py``). If the upstream
# conftest.py changes these strings, update here too — the
# :func:`_minimax_provider` factory below MUST agree with the values
# ``MiniMaxTTSProvider`` is wired with so the mock transport intercepts
# the right URL.
FAKE_API_KEY = "fake-key-conftest-do-not-use-outside-tests"
FAKE_GROUP_ID = "g-FAKE-CONFTEST-00000000-0000-0000-0000-000000000000"
MINIMAX_T2A_PATH = "/v1/t2a_v2"
MINIMAX_BASE_URL = "https://api.minimax.io"


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


ProviderFactory = Callable[[], TTSProvider]
SettingsFactory = Callable[[], TTSSettings]


def _minimax_provider() -> MiniMaxTTSProvider:
    """Build a MiniMaxTTSProvider wired to a respx-mocked httpx client.

    The provider's HTTP plumbing is exercised against an in-memory
    :class:`httpx.MockTransport` so no socket is opened and no real
    MiniMax credentials are needed.

    The transport returns the appropriate body for each call:

    * ``synthesize`` — single JSON envelope (because ``_post`` calls
      ``resp.json()``).
    * ``stream`` — SSE-framed envelope (because ``stream`` parses
      ``aiter_lines()`` for ``data:{json}`` events).

    The conformance suite asserts *contract* behaviour (return types,
    finish_reason semantics, exception types) rather than payload-shape
    fidelity, which lives in :mod:`test_minimax_tts_provider`.
    """

    def _handler(request: httpx.Request) -> httpx.Response:
        # 32 kHz mono 16-bit PCM, 1 byte per sample — small but well-formed.
        fake_audio_bytes = b"\x00\x01" * 16  # 32 bytes ≈ 1 ms @ 16 kHz
        envelope = {
            "data": {
                "audio": fake_audio_bytes.hex(),
                "audio_sample_rate": 32_000,
                "audio_length": len(fake_audio_bytes),
            },
            "extra_info": None,
            "base_resp": {"status_code": 0, "status_msg": "success"},
        }
        # Decode the request body to know whether the caller asked for
        # streaming. ``_post`` calls ``resp.json()``; ``stream`` parses
        # SSE lines — they need different framing.
        try:
            req_payload = json.loads(request.content)
        except (ValueError, TypeError):
            req_payload = {}
        if req_payload.get("stream") is True:
            body = f"data:{json.dumps(envelope)}\n\ndata:[DONE]\n\n"
            return httpx.Response(
                200,
                headers={"content-type": "text/event-stream"},
                content=body,
            )
        return httpx.Response(
            200,
            headers={"content-type": "application/json"},
            json=envelope,
        )

    transport = httpx.MockTransport(_handler)
    # base_url must match MINIMAX_BASE_URL exactly so the provider's
    # already-lowercased host assertion still holds.
    client = httpx.AsyncClient(transport=transport, base_url=MINIMAX_BASE_URL)
    return MiniMaxTTSProvider(
        api_key=FAKE_API_KEY,
        group_id=FAKE_GROUP_ID,
        base_url=MINIMAX_BASE_URL,
        default_voice="Calm_Woman",
        default_model="speech-02-hd",
        client=client,
    )


def _minimax_settings() -> TTSSettings:
    """A TTSSettings that every MiniMax-conformant path will accept."""
    return TTSSettings(
        voice="Calm_Woman",
        model="speech-02-hd",
        language="ru",
        speed=1.0,
        volume=5.0,
        pitch=0,
        emotion="neutral",
        sample_rate=32_000,
        format=TTSFormat.PCM,
    )


def _fake_provider() -> FakeTTSProvider:
    return FakeTTSProvider(sample_rate=32_000)


def _fake_settings() -> TTSSettings:
    return TTSSettings(sample_rate=32_000, format=TTSFormat.PCM)


# Each entry is the (provider_factory, settings_factory) pair the suite
# parametrizes over. Order is stable so test IDs don't shuffle between
# runs.
CONFORMANCE_PROVIDERS: list[tuple[ProviderFactory, SettingsFactory]] = [
    (_fake_provider, _fake_settings),
    (_minimax_provider, _minimax_settings),
]


# Friendly IDs for pytest's `-v` output. Pulled from the class name so
# the IDs stay correct if the factory ever changes name.
CONFORMANCE_IDS: list[str] = [
    factory().__class__.__name__ for factory, _ in CONFORMANCE_PROVIDERS
]


# ---------------------------------------------------------------------------
# ABC conformance — methods, signatures, types
# ---------------------------------------------------------------------------


class TestABCConformance:
    """Every concrete provider must look like a :class:`TTSProvider`."""

    @pytest.mark.parametrize(
        "factory", [f for f, _ in CONFORMANCE_PROVIDERS], ids=CONFORMANCE_IDS
    )
    def test_is_a_tts_provider_subclass(self, factory: ProviderFactory) -> None:
        assert issubclass(factory().__class__, TTSProvider)

    @pytest.mark.parametrize(
        "factory", [f for f, _ in CONFORMANCE_PROVIDERS], ids=CONFORMANCE_IDS
    )
    def test_synthesize_is_coroutine(self, factory: ProviderFactory) -> None:
        """``synthesize`` MUST be an async coroutine function — not a
        generator, not a plain function. Downstream consumers ``await`` it
        directly; a synchronous implementation would silently break them
        under load."""
        assert inspect.iscoroutinefunction(factory().synthesize)

    @pytest.mark.parametrize(
        "factory", [f for f, _ in CONFORMANCE_PROVIDERS], ids=CONFORMANCE_IDS
    )
    def test_stream_is_async_generator(self, factory: ProviderFactory) -> None:
        """``stream`` MUST be an async generator function — it's declared
        ``async def`` with ``yield`` inside, so it returns an
        :class:`AsyncIterator[TTSChunk]` that callers consume with
        ``async for``. A regular coroutine returning a list would
        require buffering the entire synthesis in memory before any
        audio can be played."""
        method = factory().stream
        assert inspect.isasyncgenfunction(method), (
            "stream() must be an async generator function (async def "
            "with yield) so callers can consume frames incrementally"
        )

    @pytest.mark.parametrize(
        "factory", [f for f, _ in CONFORMANCE_PROVIDERS], ids=CONFORMANCE_IDS
    )
    def test_close_is_coroutine(self, factory: ProviderFactory) -> None:
        """``aclose`` MUST be a coroutine function. Synchronous close
        would block the event loop on socket teardown — silent perf
        regression."""
        assert inspect.iscoroutinefunction(factory().aclose)

    @pytest.mark.parametrize(
        "factory,settings_factory",
        CONFORMANCE_PROVIDERS,
        ids=CONFORMANCE_IDS,
    )
    def test_name_is_non_empty_string(
        self,
        factory: ProviderFactory,
        settings_factory: SettingsFactory,
    ) -> None:
        """``name`` MUST be a non-empty string — downstream code branches
        on it (e.g. ``provider.name == "fake"`` skips network)."""
        p = factory()
        assert isinstance(p.name, str)
        assert p.name


# ---------------------------------------------------------------------------
# Behavioural conformance — synthesized output contract
# ---------------------------------------------------------------------------


class TestSynthesizeConformance:
    @pytest.mark.parametrize(
        "factory,settings_factory",
        CONFORMANCE_PROVIDERS,
        ids=CONFORMANCE_IDS,
    )
    @pytest.mark.asyncio
    async def test_returns_tts_audio_instance(
        self,
        factory: ProviderFactory,
        settings_factory: SettingsFactory,
    ) -> None:
        p = factory()
        out = await p.synthesize("hello", settings=settings_factory())
        assert isinstance(out, TTSAudio)

    @pytest.mark.parametrize(
        "factory,settings_factory",
        CONFORMANCE_PROVIDERS,
        ids=CONFORMANCE_IDS,
    )
    @pytest.mark.asyncio
    async def test_returns_non_empty_samples_for_non_empty_input(
        self,
        factory: ProviderFactory,
        settings_factory: SettingsFactory,
    ) -> None:
        """A non-empty input MUST yield non-empty bytes. Empty bytes here
        would force every consumer to special-case "zero-length TTS"."""
        p = factory()
        out = await p.synthesize("hello", settings=settings_factory())
        assert out.samples, "synthesize must return non-empty bytes"

    @pytest.mark.parametrize(
        "factory,settings_factory",
        CONFORMANCE_PROVIDERS,
        ids=CONFORMANCE_IDS,
    )
    @pytest.mark.asyncio
    async def test_returns_positive_sample_rate(
        self,
        factory: ProviderFactory,
        settings_factory: SettingsFactory,
    ) -> None:
        p = factory()
        out = await p.synthesize("hello", settings=settings_factory())
        assert out.sample_rate > 0

    @pytest.mark.parametrize(
        "factory,settings_factory",
        CONFORMANCE_PROVIDERS,
        ids=CONFORMANCE_IDS,
    )
    @pytest.mark.asyncio
    async def test_returns_tts_format(
        self,
        factory: ProviderFactory,
        settings_factory: SettingsFactory,
    ) -> None:
        """``format`` MUST be a :class:`TTSFormat` value. The ROS bridge
        dispatches on this field to pick the right decoder — a raw
        string here would crash at runtime."""
        p = factory()
        out = await p.synthesize("hello", settings=settings_factory())
        assert isinstance(out.format, TTSFormat)


# ---------------------------------------------------------------------------
# Stream conformance
# ---------------------------------------------------------------------------


class TestStreamConformance:
    @pytest.mark.parametrize(
        "factory,settings_factory",
        CONFORMANCE_PROVIDERS,
        ids=CONFORMANCE_IDS,
    )
    @pytest.mark.asyncio
    async def test_stream_yields_at_least_one_chunk(
        self,
        factory: ProviderFactory,
        settings_factory: SettingsFactory,
    ) -> None:
        """The contract requires providers to emit AT LEAST one chunk
        even on success — that's how callers detect end-of-stream
        deterministically. Zero-chunk streams break every consumer
        that does ``async for c in stream(): …`` because the loop
        simply never runs."""
        p = factory()
        chunks = [c async for c in p.stream("hi", settings=settings_factory())]
        assert len(chunks) >= 1

    @pytest.mark.parametrize(
        "factory,settings_factory",
        CONFORMANCE_PROVIDERS,
        ids=CONFORMANCE_IDS,
    )
    @pytest.mark.asyncio
    async def test_last_chunk_has_finish_reason_set(
        self,
        factory: ProviderFactory,
        settings_factory: SettingsFactory,
    ) -> None:
        """The terminal chunk MUST carry ``finish_reason``. Without it,
        callers can't distinguish "stream still going" from "stream done"
        for providers that buffer the whole response into one chunk."""
        p = factory()
        chunks = [c async for c in p.stream("hi", settings=settings_factory())]
        assert chunks[-1].finish_reason is not None
        # And that value is one of the documented reasons.
        assert chunks[-1].finish_reason in {"stop", "error"}

    @pytest.mark.parametrize(
        "factory,settings_factory",
        CONFORMANCE_PROVIDERS,
        ids=CONFORMANCE_IDS,
    )
    @pytest.mark.asyncio
    async def test_intermediate_chunks_have_no_finish_reason(
        self,
        factory: ProviderFactory,
        settings_factory: SettingsFactory,
    ) -> None:
        """Every chunk EXCEPT the last MUST have ``finish_reason is None``
        — the contract pins finish_reason to the terminal chunk so callers
        can use ``async for`` to stream audio without losing the signal."""
        p = factory()
        chunks = [c async for c in p.stream("hi", settings=settings_factory())]
        for c in chunks[:-1]:
            assert c.finish_reason is None, (
                f"intermediate chunk {c!r} carries finish_reason; "
                "contract reserves it for the terminal chunk"
            )

    @pytest.mark.parametrize(
        "factory,settings_factory",
        CONFORMANCE_PROVIDERS,
        ids=CONFORMANCE_IDS,
    )
    @pytest.mark.asyncio
    async def test_chunks_are_tts_chunk_instances(
        self,
        factory: ProviderFactory,
        settings_factory: SettingsFactory,
    ) -> None:
        p = factory()
        chunks = [c async for c in p.stream("hi", settings=settings_factory())]
        assert chunks
        for c in chunks:
            assert isinstance(c, TTSChunk)


# ---------------------------------------------------------------------------
# Resource lifecycle — aclose idempotency
# ---------------------------------------------------------------------------


class TestCloseConformance:
    @pytest.mark.parametrize(
        "factory,settings_factory",
        CONFORMANCE_PROVIDERS,
        ids=CONFORMANCE_IDS,
    )
    @pytest.mark.asyncio
    async def test_aclose_is_idempotent(
        self,
        factory: ProviderFactory,
        settings_factory: SettingsFactory,
    ) -> None:
        """``aclose()`` MUST be safe to call multiple times. Callers
        typically wrap it in ``finally`` after exception paths, and
        double-close (once explicitly, once via ``async with``) is
        a real-world pattern."""
        p = factory()
        await p.aclose()
        await p.aclose()  # second call must not raise
        await p.aclose()  # third call must also not raise


# ---------------------------------------------------------------------------
# Input validation — pre-flight guards
# ---------------------------------------------------------------------------


class TestInputValidationConformance:
    @pytest.mark.parametrize(
        "factory,settings_factory",
        CONFORMANCE_PROVIDERS,
        ids=CONFORMANCE_IDS,
    )
    @pytest.mark.asyncio
    async def test_empty_text_rejected(
        self,
        factory: ProviderFactory,
        settings_factory: SettingsFactory,
    ) -> None:
        """Empty / whitespace-only text MUST raise :class:`TTSBadRequestError`
        BEFORE any HTTP request — otherwise providers either bill for an
        empty synthesis or hang on a server-side zero-length check."""
        p = factory()
        with pytest.raises(TTSBadRequestError):
            await p.synthesize("", settings=settings_factory())
        with pytest.raises(TTSBadRequestError):
            await p.synthesize("   ", settings=settings_factory())

    @pytest.mark.parametrize(
        "factory,settings_factory",
        CONFORMANCE_PROVIDERS,
        ids=CONFORMANCE_IDS,
    )
    @pytest.mark.asyncio
    async def test_empty_text_rejected_on_stream(
        self,
        factory: ProviderFactory,
        settings_factory: SettingsFactory,
    ) -> None:
        """Same pre-flight guard for the streaming path."""
        p = factory()
        with pytest.raises(TTSBadRequestError):
            async for _ in p.stream("", settings=settings_factory()):
                pytest.fail("stream yielded before empty-text guard fired")


# ---------------------------------------------------------------------------
# MiniMax-specific HTTP plumbing — exercise the respx path on conformance
# ---------------------------------------------------------------------------


class TestMiniMaxHttpConformance:
    """The conformance provider for MiniMax goes through real HTTP plumbing
    via the mocked transport. These tests confirm the wire shape we expect
    from any conforming MiniMax integration: Bearer auth, GroupId query
    param, POST to the documented T2A v2 path."""

    @pytest.fixture
    def provider_with_router(self) -> Iterator[tuple[MiniMaxTTSProvider, respx.Router]]:
        """Yield (provider, router). The provider is wired to a respx
        router that intercepts ``POST /v1/t2a_v2``."""
        envelope = {
            "data": {
                "audio": (b"\x00\x01" * 32).hex(),
                "audio_sample_rate": 32_000,
                "audio_length": 64,
            },
            "base_resp": {"status_code": 0, "status_msg": "success"},
        }
        with respx.mock(assert_all_called=False) as router:
            router.post(MINIMAX_T2A_PATH).respond(
                200,
                headers={"content-type": "application/json"},
                json=envelope,
            )
            client = httpx.AsyncClient(
                transport=httpx.MockTransport(router.handler),
                base_url=MINIMAX_BASE_URL,
            )
            yield (
                MiniMaxTTSProvider(
                    api_key=FAKE_API_KEY,
                    group_id=FAKE_GROUP_ID,
                    base_url=MINIMAX_BASE_URL,
                    default_voice="Calm_Woman",
                    default_model="speech-02-hd",
                    client=client,
                ),
                router,
            )

    @pytest.mark.asyncio
    async def test_synthesize_uses_bearer_auth_and_group_id(
        self,
        provider_with_router: tuple[MiniMaxTTSProvider, respx.Router],
    ) -> None:
        provider, router = provider_with_router
        await provider.synthesize(
            "hi", settings=TTSSettings(voice="Calm_Woman", sample_rate=32_000)
        )
        # One POST hit, with the Bearer auth header AND the GroupId
        # query param — providers that skip either will hit MiniMax's
        # auth wall and burn a real retry budget.
        assert router.calls.call_count == 1
        call = router.calls.last
        assert call.request.headers["Authorization"] == f"Bearer {FAKE_API_KEY}"
        assert call.request.url.params["GroupId"] == FAKE_GROUP_ID
        assert call.request.method == "POST"

    @pytest.mark.asyncio
    async def test_synthesize_uses_lower_case_official_endpoint(
        self,
        provider_with_router: tuple[MiniMaxTTSProvider, respx.Router],
    ) -> None:
        provider, router = provider_with_router
        await provider.synthesize(
            "hi", settings=TTSSettings(voice="Calm_Woman", sample_rate=32_000)
        )
        call = router.calls.last
        # MiniMax's host is lowercase; any uppercase letter on the wire
        # is a 404 from their CDN.
        assert call.request.url.host == "api.minimax.io"
        assert call.request.url.scheme == "https"


__all__ = [
    "CONFORMANCE_PROVIDERS",
    "CONFORMANCE_IDS",
]