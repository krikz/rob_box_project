"""PR-907 BLK-5 — per-phase ``httpx.Timeout`` for LLM and TTS clients.

Background
----------

The previous single-float ``timeout: float = 30.0`` constructor argument
flowed unchanged into ``AsyncOpenAI(timeout=...)`` and
``httpx.AsyncClient(timeout=...)``. In both libraries a bare ``float``
means "use this many seconds for every phase" — so a DNS / TLS hang on
the connect phase would block the user for the full 30 s, which is the
symptom the BLK-5 review set out to fix.

The fix splits the budget across the four phases:

* ``connect=5.0``  — TCP+TLS handshake. A slow connect usually means the
  host is unreachable and the user should hear about it fast.
* ``read=20.0``    — covers LLM streaming and long TTS downloads.
* ``write=10.0``   — request body upload.
* ``pool=5.0``     — waiting for a free connection from the pool.

What this module covers
-----------------------

1. The MiniMax LLM provider's default client is constructed with a
   per-phase ``httpx.Timeout`` (verified by inspecting the real
   ``AsyncOpenAI`` instance, not a fake).
2. A user-supplied ``httpx.Timeout`` flows through to the SDK unchanged.
3. A user-supplied ``float`` is normalised to "all phases" (httpx's
   behaviour, preserved for backwards compatibility).
4. The MiniMax TTS provider's default client carries the same per-phase
   defaults; user overrides of either shape are honoured.
5. :class:`BaseTTSProvider._http_client_factory` falls back to a
   per-phase default when the subclass never set ``self._timeout`` at
   all.
6. The :func:`rob_box_llm.providers.minimax._coerce_timeout` helper
   accepts every documented input shape.

Why a separate file (and not appended to the existing suites)
-------------------------------------------------------------

The existing ``test_minimax_provider.py`` uses a fake ``AsyncOpenAI`` so
none of those tests actually exercise the real SDK constructor. Putting
the new tests in their own module makes the BLK-5 surface area easy to
spot in a PR review ("here are the 6 tests that pin the per-phase
behaviour") and easy to delete if the timeout policy moves again.
"""

from __future__ import annotations

import asyncio

import httpx
from openai import AsyncOpenAI

from rob_box_llm.providers.minimax import (
    DEFAULT_TIMEOUT as LLM_DEFAULT_TIMEOUT,
    MiniMaxProvider,
    _coerce_timeout,
)
from rob_box_llm.providers.minimax_tts import MiniMaxTTSProvider
from rob_box_llm.tts_provider_base import BaseTTSProvider
from rob_box_llm.tts import TTSProvider as _TTSProvider  # for isinstance


# ---------------------------------------------------------------------------
# Sentinel credentials — match the rest of the TTS suite so the
# ``_scrub_minimax_env`` conftest autouse fixture stays happy.
# ---------------------------------------------------------------------------

_FAKE_API_KEY = "fake-key-blk5-do-not-use-outside-tests"
_FAKE_GROUP_ID = "g-BLK5-00000000-0000-0000-0000-000000000000"


# ---------------------------------------------------------------------------
# 1. MiniMax LLM provider — per-phase default on the real AsyncOpenAI
# ---------------------------------------------------------------------------


def test_minimax_default_timeout_is_per_phase_httpx_timeout() -> None:
    """The LLM provider's ``DEFAULT_TIMEOUT`` is an ``httpx.Timeout``
    with split per-phase budgets, not a single float.

    This is the BLK-5 invariant: a bare float would mean "30 s for every
    phase" and re-introduce the regression.
    """
    assert isinstance(LLM_DEFAULT_TIMEOUT, httpx.Timeout)
    assert LLM_DEFAULT_TIMEOUT.connect == 5.0
    assert LLM_DEFAULT_TIMEOUT.read == 20.0
    assert LLM_DEFAULT_TIMEOUT.write == 10.0
    assert LLM_DEFAULT_TIMEOUT.pool == 5.0


def test_minimax_provider_builds_real_asyncopenai_with_per_phase_timeout() -> None:
    """Without an injected client, ``MiniMaxProvider`` builds a real
    ``AsyncOpenAI`` whose ``timeout`` is the per-phase default.

    We have to pass ``thinking=None`` to dodge the openai SDK 2.x bug
    where a non-None ``thinking`` kwarg is forwarded as a top-level
    argument to ``AsyncOpenAI.__init__``, which the SDK rejects (the
    upstream fix is to push ``thinking`` into ``extra_body``, but the
    workaround documented in tests is to pass ``None``).
    """
    p = MiniMaxProvider(
        base_url="https://example.invalid",
        api_key="sk-test",
        model=MiniMaxProvider.DEFAULT_MODEL,
        thinking=None,
    )
    try:
        # ``p._client`` is the underlying ``AsyncOpenAI``. Inspecting
        # its ``timeout`` (an httpx.Timeout in OpenAI 1.x/2.x) confirms
        # the per-phase default landed in the SDK.
        client = p._client
        assert isinstance(client, AsyncOpenAI)
        sdk_timeout = client.timeout
        assert isinstance(sdk_timeout, httpx.Timeout), (
            "AsyncOpenAI should store an httpx.Timeout when given one; "
            f"got {type(sdk_timeout).__name__}"
        )
        assert sdk_timeout.connect == 5.0
        assert sdk_timeout.read == 20.0
        assert sdk_timeout.write == 10.0
        assert sdk_timeout.pool == 5.0
    finally:
        # OpenAI 2.x: ``AsyncOpenAI.close`` is sync (returns coroutine
        # via ``await`` from a different surface). The pattern is to
        # ``await client.close()``. We don't import the SDK's async
        # helper here — just call and, if it returned a coroutine, run
        # it.
        close_result = p._client.close()
        if asyncio.iscoroutine(close_result):
            asyncio.run(close_result)


def test_minimax_provider_honours_custom_httpx_timeout() -> None:
    """A user-supplied :class:`httpx.Timeout` flows through to the SDK."""
    custom = httpx.Timeout(connect=1.0, read=2.0, write=3.0, pool=4.0)
    p = MiniMaxProvider(
        base_url="https://example.invalid",
        api_key="sk-test",
        model=MiniMaxProvider.DEFAULT_MODEL,
        timeout=custom,
        thinking=None,
    )
    try:
        sdk_timeout = p._client.timeout
        assert isinstance(sdk_timeout, httpx.Timeout)
        assert sdk_timeout.connect == 1.0
        assert sdk_timeout.read == 2.0
        assert sdk_timeout.write == 3.0
        assert sdk_timeout.pool == 4.0
    finally:
        close_result = p._client.close()
        if asyncio.iscoroutine(close_result):
            asyncio.run(close_result)


def test_minimax_provider_honours_plain_float_timeout() -> None:
    """A bare float is preserved as "all phases" (httpx semantics).

    Older call-sites pass ``timeout=30.0``. They must keep working.
    """
    p = MiniMaxProvider(
        base_url="https://example.invalid",
        api_key="sk-test",
        model=MiniMaxProvider.DEFAULT_MODEL,
        timeout=42.0,
        thinking=None,
    )
    try:
        sdk_timeout = p._client.timeout
        # httpx normalises a single float to "all phases".
        assert isinstance(sdk_timeout, httpx.Timeout)
        assert sdk_timeout.connect == 42.0
        assert sdk_timeout.read == 42.0
        assert sdk_timeout.write == 42.0
        assert sdk_timeout.pool == 42.0
    finally:
        close_result = p._client.close()
        if asyncio.iscoroutine(close_result):
            asyncio.run(close_result)


# ---------------------------------------------------------------------------
# 2. _coerce_timeout helper — unit-level tests
# ---------------------------------------------------------------------------


class TestCoerceTimeout:
    def test_none_returns_per_phase_default(self) -> None:
        result = _coerce_timeout(None)
        assert result is LLM_DEFAULT_TIMEOUT
        assert result.connect == 5.0

    def test_httpx_timeout_passes_through_unchanged(self) -> None:
        custom = httpx.Timeout(connect=1.0, read=2.0, write=3.0, pool=4.0)
        assert _coerce_timeout(custom) is custom

    def test_float_is_normalised_to_all_phases(self) -> None:
        result = _coerce_timeout(7.0)
        assert isinstance(result, httpx.Timeout)
        assert result.connect == 7.0
        assert result.read == 7.0
        assert result.write == 7.0
        assert result.pool == 7.0

    def test_int_is_treated_as_float(self) -> None:
        result = _coerce_timeout(5)
        assert isinstance(result, httpx.Timeout)
        assert result.read == 5.0


# ---------------------------------------------------------------------------
# 3. MiniMax TTS provider — per-phase default on the real httpx client
# ---------------------------------------------------------------------------


def test_minimax_tts_default_timeout_is_per_phase_httpx_timeout() -> None:
    """The TTS provider's ``DEFAULT_TIMEOUT`` is an ``httpx.Timeout``
    with split per-phase budgets, not a single float.
    """
    tts_default = MiniMaxTTSProvider.DEFAULT_TIMEOUT
    assert isinstance(tts_default, httpx.Timeout)
    assert tts_default.connect == 5.0
    assert tts_default.read == 20.0
    assert tts_default.write == 10.0
    assert tts_default.pool == 5.0


def test_minimax_tts_default_client_carries_per_phase_timeout() -> None:
    """Building a provider with no ``client=`` kwarg uses the default
    ``_http_client_factory``, which must hand httpx the per-phase
    :class:`httpx.Timeout`.
    """
    p = MiniMaxTTSProvider(
        api_key=_FAKE_API_KEY,
        group_id=_FAKE_GROUP_ID,
    )
    try:
        assert isinstance(p._client, httpx.AsyncClient)
        client_timeout = p._client.timeout
        assert isinstance(client_timeout, httpx.Timeout)
        assert client_timeout.connect == 5.0
        assert client_timeout.read == 20.0
        assert client_timeout.write == 10.0
        assert client_timeout.pool == 5.0
    finally:
        import asyncio

        asyncio.run(p.aclose())


def test_minimax_tts_honours_custom_httpx_timeout() -> None:
    """A user-supplied :class:`httpx.Timeout` lands on the client."""
    custom = httpx.Timeout(connect=1.0, read=2.0, write=3.0, pool=4.0)
    p = MiniMaxTTSProvider(
        api_key=_FAKE_API_KEY,
        group_id=_FAKE_GROUP_ID,
        timeout=custom,
    )
    try:
        assert isinstance(p._client, httpx.AsyncClient)
        client_timeout = p._client.timeout
        assert isinstance(client_timeout, httpx.Timeout)
        assert client_timeout.connect == 1.0
        assert client_timeout.read == 2.0
        assert client_timeout.write == 3.0
        assert client_timeout.pool == 4.0
    finally:
        import asyncio

        asyncio.run(p.aclose())


def test_minimax_tts_honours_plain_float_timeout() -> None:
    """A bare float is preserved as "all phases" (backwards compat)."""
    p = MiniMaxTTSProvider(
        api_key=_FAKE_API_KEY,
        group_id=_FAKE_GROUP_ID,
        timeout=42.0,
    )
    try:
        client_timeout = p._client.timeout
        assert isinstance(client_timeout, httpx.Timeout)
        assert client_timeout.connect == 42.0
        assert client_timeout.read == 42.0
        assert client_timeout.write == 42.0
        assert client_timeout.pool == 42.0
    finally:
        import asyncio

        asyncio.run(p.aclose())


def test_minimax_tts_stores_coerced_timeout_on_instance() -> None:
    """``self._timeout`` mirrors the coerced value the factory will use.

    This is the surface that ``_http_client_factory`` reads; if it
    stored a wrong type the client would be built with the wrong
    timeout.
    """
    p = MiniMaxTTSProvider(
        api_key=_FAKE_API_KEY,
        group_id=_FAKE_GROUP_ID,
    )
    try:
        assert isinstance(p._timeout, httpx.Timeout)
        assert p._timeout.connect == 5.0
        assert p._timeout.read == 20.0
        assert p._timeout.write == 10.0
        assert p._timeout.pool == 5.0
    finally:
        import asyncio

        asyncio.run(p.aclose())


# ---------------------------------------------------------------------------
# 4. BaseTTSProvider._http_client_factory — per-phase default fallback
# ---------------------------------------------------------------------------


class _MinimalTTS(BaseTTSProvider):
    """Bare-minimum subclass that skips ``self._timeout`` entirely.

    Used to exercise the new per-phase fallback in
    :meth:`BaseTTSProvider._http_client_factory`. We override
    ``_build_request_payload`` because :class:`BaseTTSProvider` makes
    it abstract; we also stub the two ``TTSProvider``-level abstract
    methods (``synthesize`` / ``stream``) with no-ops because the
    factory under test does not need them to do anything useful.
    """

    name = "minimal-test"

    def _build_request_payload(self, text, settings, voice_meta):  # type: ignore[override]
        return {"text": text}

    async def synthesize(self, text, *, settings=None):  # type: ignore[override]
        raise NotImplementedError

    async def stream(self, text, *, settings=None):  # type: ignore[override]
        # The ABC says stream returns ``AsyncIterator[TTSChunk]``. An
        # ``async def`` that contains ``yield`` is itself an async
        # generator / async iterator, so this satisfies the contract
        # without doing any real work.
        if False:  # pragma: no cover
            yield None


def test_base_tts_factory_uses_per_phase_default_when_subclass_skips_timeout() -> None:
    """A subclass that never sets ``self._timeout`` falls back to a
    per-phase :class:`httpx.Timeout` (not the old single-float 30 s).
    """
    p = _MinimalTTS()
    client = p._http_client_factory()
    try:
        assert isinstance(client, httpx.AsyncClient)
        client_timeout = client.timeout
        assert isinstance(client_timeout, httpx.Timeout)
        # The per-phase default MUST be in effect — not a 30.0 fallback.
        assert client_timeout.connect == 5.0
        assert client_timeout.read == 20.0
        assert client_timeout.write == 10.0
        assert client_timeout.pool == 5.0
    finally:
        import asyncio

        asyncio.run(client.aclose())


def test_base_tts_factory_passes_through_subclass_float_timeout() -> None:
    """When the subclass sets a plain float, the factory forwards it as
    "all phases" — preserving the old behaviour for subclasses that
    opt in to a single number.
    """
    p = _MinimalTTS()
    p._timeout = 12.0
    client = p._http_client_factory()
    try:
        client_timeout = client.timeout
        assert isinstance(client_timeout, httpx.Timeout)
        assert client_timeout.connect == 12.0
        assert client_timeout.read == 12.0
        assert client_timeout.write == 12.0
        assert client_timeout.pool == 12.0
    finally:
        import asyncio

        asyncio.run(client.aclose())


def test_base_tts_factory_passes_through_subclass_httpx_timeout() -> None:
    """When the subclass sets a per-phase ``httpx.Timeout``, the
    factory forwards it unchanged.
    """
    p = _MinimalTTS()
    custom = httpx.Timeout(connect=1.0, read=2.0, write=3.0, pool=4.0)
    p._timeout = custom
    client = p._http_client_factory()
    try:
        client_timeout = client.timeout
        assert isinstance(client_timeout, httpx.Timeout)
        assert client_timeout.connect == 1.0
        assert client_timeout.read == 2.0
        assert client_timeout.write == 3.0
        assert client_timeout.pool == 4.0
    finally:
        import asyncio

        asyncio.run(client.aclose())


# ---------------------------------------------------------------------------
# 5. Sanity: MiniMaxTTSProvider still IS-A TTSProvider (PR-907 regression
#    guard unrelated to BLK-5, but cheap to keep here).
# ---------------------------------------------------------------------------


def test_minimax_tts_is_a_tts_provider() -> None:
    p = MiniMaxTTSProvider(
        api_key=_FAKE_API_KEY,
        group_id=_FAKE_GROUP_ID,
    )
    try:
        assert isinstance(p, _TTSProvider)
        assert isinstance(p, BaseTTSProvider)
    finally:
        import asyncio

        asyncio.run(p.aclose())
