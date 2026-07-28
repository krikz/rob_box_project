"""Tests for the TTS extension-points migration (t_8cbf9995, ADR-0008).

Three layers are covered here, each in its own section:

* :class:`BaseTTSProvider` + its value objects (TTSCapabilities, TTSVoice,
  TTSHealth) and the :data:`ProviderBuilder` callable type — pure
  contract tests, no network, no fixtures.
* :class:`MiniMaxTTSProvider` — the migration target. These tests
  confirm that the existing provider now inherits from
  :class:`BaseTTSProvider` and fills in all five extension hooks
  (``capabilities``, ``list_voices``, ``healthcheck``,
  ``_http_client_factory``, ``_build_request_payload``).
* :class:`TTSProviderRegistry` + :class:`TTSProviderFactory` +
  :func:`register_builtin_tts_providers` — the composition-root
  helpers.

Why these tests exist separately from ``test_minimax_tts_provider.py``
(and friends):

The existing suite already proves the *behaviour* of the provider
(end-to-end HTTP synthesis, streaming, error mapping, leak guards).
This file proves the *shape* of the extension surface — that a future
provider author can subclass :class:`BaseTTSProvider` and have a
sane default for everything they don't override. If a regression in
the base class breaks ``isinstance(provider, TTSProvider)`` (the
backward-compat promise), the existing suite passes (because the
provider overrides everything); these tests fail.
"""

from __future__ import annotations

import abc
import asyncio
from typing import Any

import httpx
import pytest

from rob_box_llm import (
    BaseTTSProvider,
    MiniMaxTTSProvider,
    ProviderBuilder,
    TTSCapabilities,
    TTSHealth,
    TTSProvider,
    TTSProviderFactory,
    TTSProviderRegistry,
    TTSVoice,
    register_builtin_tts_providers,
)
from rob_box_llm.providers.minimax_tts import _build_payload
from rob_box_llm.tts import FakeTTSProvider, TTSSettings
from rob_box_llm.tts_provider_registry import register_builtin_tts_providers as _rb


# Sentinel constants — use the same shape the conftest uses, but
# duplicated here so the extension-points tests do NOT need to load
# the conftest (and can run on their own if the test layout changes).
_FAKE_KEY = "extpoints-fake-key"
_FAKE_GROUP = "g-EXTPOINTS-FAKE-0000-0000-0000-000000000000"


def _stub_provider(**kwargs: Any) -> MiniMaxTTSProvider:
    """Build a provider that doesn't open a socket on construction.

    Mirrors ``conftest.minimax_provider`` — injecting a placeholder
    client keeps the constructor cheap (no httpx connection pool).
    """
    kwargs.setdefault("api_key", _FAKE_KEY)
    kwargs.setdefault("group_id", _FAKE_GROUP)
    kwargs.setdefault(
        "client",
        httpx.AsyncClient(transport=httpx.AsyncHTTPTransport(retries=0)),
    )
    return MiniMaxTTSProvider(**kwargs)


# ===========================================================================
# 1. BaseTTSProvider + value objects — contract tests
# ===========================================================================


class TestBaseTTSProviderContract:
    """``BaseTTSProvider`` IS-A ``TTSProvider`` and the ABC machinery works."""

    def test_base_is_subclass_of_tts_provider(self):
        # Backward-compat promise: every existing call site that
        # type-annotates ``TTSProvider`` must keep working unchanged.
        # If someone removes the inheritance this assertion catches
        # it before any test that uses a BaseTTSProvider subtype.
        assert issubclass(BaseTTSProvider, TTSProvider)

    def test_cannot_instantiate_base_directly(self):
        # BaseTTSProvider is abstract — its ``_build_request_payload``
        # method has no implementation, so instantiation must raise.
        with pytest.raises(TypeError):
            BaseTTSProvider()  # noqa: F841 — intentional instantiation

    def test_subclass_missing_payload_method_cannot_instantiate(self):
        # Subclasses that forget the mandatory hook fail at construction.
        class _Forgetting(BaseTTSProvider):
            name = "forgetful"

        with pytest.raises(TypeError):
            _Forgetting()  # noqa: F841

    def test_complete_subclass_instantiates(self):
        # Sanity check: a minimal but complete subclass DOES instantiate.
        # The base has TWO abstract surface areas — ``_build_request_payload``
        # (its own hook) and ``synthesize`` / ``stream`` (inherited from
        # TTSProvider). Both must be overridden.
        from rob_box_llm.tts import TTSAudio, TTSChunk, TTSFormat

        class _Complete(BaseTTSProvider):
            name = "complete"

            def _build_request_payload(self, text, settings, voice_meta):
                return {"text": text}

            async def synthesize(self, text, *, settings=None):
                return TTSAudio(samples=b"\x00\x00", sample_rate=16_000, format=TTSFormat.PCM)

            async def stream(self, text, *, settings=None):
                yield TTSChunk(finish_reason="stop")

        p = _Complete()
        assert isinstance(p, TTSProvider)
        assert p.name == "complete"
        # Default capabilities / list_voices are honest no-ops.
        assert p.capabilities() == TTSCapabilities()
        assert asyncio.run(p.list_voices()) == []

    def test_subclass_must_be_abc(self):
        # The base must remain an ABC — otherwise the mandatory
        # ``_build_request_payload`` becomes a silent NotImplementedError
        # at first call, which is exactly the failure mode we wanted
        # to prevent by introducing the base class.
        assert issubclass(BaseTTSProvider, abc.ABC)
        # ``_build_request_payload`` specifically carries @abc.abstractmethod.
        assert (
            getattr(
                BaseTTSProvider._build_request_payload,
                "__isabstractmethod__",
                False,
            )
            is True
        )


class TestTTSCapabilitiesDefaults:
    """``TTSCapabilities()`` is honest: every flag ``False``."""

    def test_all_flags_default_to_false(self):
        caps = TTSCapabilities()
        assert caps.streaming is False
        assert caps.voice_cloning is False
        assert caps.ssml is False
        assert caps.pronunciation_dict is False
        assert caps.audio_format_pcm is False
        assert caps.audio_format_mp3 is False
        assert caps.audio_format_ogg is False
        assert caps.custom_endpoint is False

    def test_frozen(self):
        caps = TTSCapabilities(streaming=True)
        with pytest.raises((AttributeError, Exception)):  # FrozenInstanceError
            caps.streaming = False  # type: ignore[misc]


class TestTTSVoiceFrozenAndExtras:
    """``TTSVoice.extra`` is locked against post-construction mutation."""

    def test_extra_is_readonly(self):
        v = TTSVoice(id="x", name="X", language="en", extra={"k": 1})
        # Either MappingProxyType (preferred) or another read-only view
        # is acceptable — the contract is "cannot mutate".
        with pytest.raises(TypeError):
            v.extra["k"] = 2  # type: ignore[index]

    def test_frozen(self):
        v = TTSVoice(id="x", name="X", language="en")
        with pytest.raises((AttributeError, Exception)):
            v.id = "y"  # type: ignore[misc]


class TestTTSHealthDefaults:
    """``TTSHealth`` defaults are stable."""

    def test_ok_true_construction(self):
        h = TTSHealth(ok=True, provider="minimax")
        assert h.latency_ms == 0.0
        assert h.reason is None

    def test_frozen(self):
        h = TTSHealth(ok=False, provider="minimax", reason="x")
        with pytest.raises((AttributeError, Exception)):
            h.reason = "y"  # type: ignore[misc]


class TestProviderBuilderType:
    """``ProviderBuilder`` is a callable that accepts a config dict."""

    def test_callable_accepts_dict(self):
        # Compile-time / import-time sanity check that the alias is
        # callable-shaped. We don't need to construct a real one — the
        # registry tests cover the full happy path below.
        assert callable(ProviderBuilder)


class TestBaseHttpClientFactory:
    """``BaseTTSProvider._http_client_factory()`` returns an httpx.AsyncClient."""

    def test_default_factory_returns_async_client(self):
        # A subclass without an override should still get a usable client.
        from rob_box_llm.tts import TTSAudio, TTSChunk, TTSFormat

        class _Minimal(BaseTTSProvider):
            name = "minimal"

            def _build_request_payload(self, text, settings, voice_meta):
                return {"text": text}

            async def synthesize(self, text, *, settings=None):
                return TTSAudio(samples=b"\x00\x00", sample_rate=16_000, format=TTSFormat.PCM)

            async def stream(self, text, *, settings=None):
                yield TTSChunk(finish_reason="stop")

        p = _Minimal()
        client = p._http_client_factory()
        try:
            assert isinstance(client, httpx.AsyncClient)
        finally:
            asyncio.run(client.aclose())


# ===========================================================================
# 2. MiniMaxTTSProvider — the migration target
# ===========================================================================


class TestMiniMaxTTSProviderMigration:
    """The existing provider now inherits from BaseTTSProvider."""

    def test_is_a_base_tts_provider(self):
        # The migration promise: MiniMaxTTSProvider IS-A BaseTTSProvider.
        assert issubclass(MiniMaxTTSProvider, BaseTTSProvider)

    def test_is_a_tts_provider_for_backward_compat(self):
        # Backward-compat: the public TTSProvider contract is unchanged.
        assert issubclass(MiniMaxTTSProvider, TTSProvider)

    def test_fake_tts_provider_is_NOT_a_base_subclass(self):
        # FakeTTSProvider is the original P0.5 impl. It does NOT migrate
        # to BaseTTSProvider — only the second opt-in provider triggers
        # the migration. Document the asymmetry so a future cleanup
        # doesn't accidentally "fix" it.
        assert not issubclass(FakeTTSProvider, BaseTTSProvider)
        assert issubclass(FakeTTSProvider, TTSProvider)

    def test_instance_passes_isinstance_both(self, minimax_provider):
        # Real instances must satisfy both type annotations —
        # ``TTSProvider``-typed call sites keep working, and any
        # future ``BaseTTSProvider``-typed call site can also accept them.
        assert isinstance(minimax_provider, BaseTTSProvider)
        assert isinstance(minimax_provider, TTSProvider)


class TestMiniMaxTTSProviderCapabilities:
    """``capabilities()`` returns a documented TTSCapabilities."""

    def test_returns_tts_capabilities(self, minimax_provider):
        caps = minimax_provider.capabilities()
        assert isinstance(caps, TTSCapabilities)

    def test_minimax_capability_flags(self, minimax_provider):
        caps = minimax_provider.capabilities()
        # MiniMax T2A v2 advertises streaming + PCM/MP3 + voice cloning.
        assert caps.streaming is True
        assert caps.voice_cloning is True
        assert caps.audio_format_pcm is True
        assert caps.audio_format_mp3 is True
        # SSML and OGG are NOT supported — flags must be honest.
        assert caps.ssml is False
        assert caps.audio_format_ogg is False
        # No custom endpoint — MiniMax is cloud-only as far as we model it.
        assert caps.custom_endpoint is False

    def test_pronunciation_dict_unset_until_value_object_adds_field(
        self, minimax_provider
    ):
        # ``pronunciation_dict`` is left False because the public
        # TTSSettings value object doesn't surface that field yet.
        # If a future PR adds the field, this test is the canary —
        # update it AND consider flipping the flag to True.
        caps = minimax_provider.capabilities()
        assert caps.pronunciation_dict is False


class TestMiniMaxTTSProviderListVoices:
    """``list_voices()`` returns a static, well-formed catalogue."""

    @pytest.mark.asyncio
    async def test_returns_six_voices(self, minimax_provider):
        voices = await minimax_provider.list_voices()
        assert isinstance(voices, list)
        assert len(voices) == 6
        for v in voices:
            assert isinstance(v, TTSVoice)

    @pytest.mark.asyncio
    async def test_voice_ids_are_unique(self, minimax_provider):
        voices = await minimax_provider.list_voices()
        ids = [v.id for v in voices]
        assert len(ids) == len(set(ids))

    @pytest.mark.asyncio
    async def test_includes_russian_calm_woman(self, minimax_provider):
        # The voice used by the project today. Pin it so a catalogue
        # rewrite that drops it fails loudly.
        voices = await minimax_provider.list_voices()
        ids = {v.id for v in voices}
        assert "Russian_CalmWoman" in ids

    @pytest.mark.asyncio
    async def test_no_upstream_call(self, minimax_provider, mock_minimax_http):
        # list_voices() must NOT issue HTTP — the catalogue is static.
        # The mock_minimax_http router is empty; if the provider
        # accidentally hits the wire, respx would raise.
        await minimax_provider.list_voices()
        assert not mock_minimax_http.routes


class TestMiniMaxTTSProviderHealthcheck:
    """``healthcheck()`` is a pure credential-presence check."""

    @pytest.mark.asyncio
    async def test_ok_when_credentials_present(self, minimax_provider):
        h = await minimax_provider.healthcheck()
        assert isinstance(h, TTSHealth)
        assert h.ok is True
        assert h.provider == "minimax"
        assert h.reason is None

    @pytest.mark.asyncio
    async def test_not_ok_when_api_key_missing(self):
        p = _stub_provider(api_key="", group_id=_FAKE_GROUP)
        h = await p.healthcheck()
        assert h.ok is False
        assert "MINIMAX_API_KEY" in (h.reason or "")

    @pytest.mark.asyncio
    async def test_not_ok_when_group_id_missing(self):
        p = _stub_provider(api_key=_FAKE_KEY, group_id="")
        h = await p.healthcheck()
        assert h.ok is False
        assert "MINIMAX_GROUP_ID" in (h.reason or "")

    @pytest.mark.asyncio
    async def test_no_upstream_call(self, minimax_provider, mock_minimax_http):
        # Pre-flight must be free of network calls — that's its
        # whole point. If this starts hitting the wire, the health
        # check stops being a pre-flight and starts being a quota burner.
        await minimax_provider.healthcheck()
        assert not mock_minimax_http.routes


class TestMiniMaxTTSProviderHttpClientFactory:
    """``_http_client_factory()`` returns an httpx.AsyncClient with timeout."""

    def test_returns_async_client(self, minimax_provider):
        client = minimax_provider._http_client_factory()
        assert isinstance(client, httpx.AsyncClient)

    def test_uses_provider_timeout(self):
        p = _stub_provider(timeout=12.5)
        client = p._http_client_factory()
        try:
            # httpx exposes the timeout via ``timeout`` (object) or
            # ``_timeout`` (raw float). Either is acceptable — the
            # important thing is the configured value flowed through.
            timeout_obj = client.timeout
            assert timeout_obj.connect == 12.5 or timeout_obj.read == 12.5
        finally:
            asyncio.run(client.aclose())


class TestMiniMaxTTSProviderBuildRequestPayload:
    """``_build_request_payload`` delegates to the module-level helper."""

    def test_delegates_to_module_helper(self, minimax_provider):
        # The hook MUST return the same dict shape the public
        # synthesize()/stream() build via the module-level helper —
        # otherwise the contract drifts and tests that pin the shape
        # of one but not the other start failing in confusing ways.
        settings = TTSSettings(voice="Calm_Woman", language="ru", speed=1.0)
        from_hook = minimax_provider._build_request_payload("hi", settings, None)
        from_helper = _build_payload(
            "hi",
            settings,
            stream=False,
            default_voice=minimax_provider._default_voice,
            default_model=minimax_provider._default_model,
        )
        assert from_hook == from_helper

    def test_no_network_or_io(self, minimax_provider, mock_minimax_http):
        # The hook MUST be pure — no HTTP, no logging of secrets, no
        # global state mutation. Tested in isolation without any
        # transport mocking needed (the empty respx router would raise
        # if anything tried to reach the wire).
        settings = TTSSettings(voice="Calm_Woman")
        minimax_provider._build_request_payload("hello", settings, None)
        assert not mock_minimax_http.routes


# ===========================================================================
# 3. TTSProviderRegistry + TTSProviderFactory + register_builtin_tts_providers
# ===========================================================================


class TestTTSProviderRegistry:
    """In-memory ``name → builder`` registry."""

    def test_register_and_resolve(self):
        reg = TTSProviderRegistry()

        def builder(cfg):
            return _stub_provider(**cfg)

        reg.register("minimax", builder)
        assert reg.resolve("minimax") is builder

    def test_names_is_sorted(self):
        reg = TTSProviderRegistry()
        # Register in non-alphabetical order to prove ``names()`` sorts.
        for n in ("zeta", "alpha", "mu"):

            def _b(cfg, _n=n):
                return _stub_provider(**cfg)

            reg.register(n, _b)
        assert reg.names() == ["alpha", "mu", "zeta"]

    def test_duplicate_register_raises(self):
        reg = TTSProviderRegistry()

        def builder(cfg):
            return _stub_provider(**cfg)

        reg.register("minimax", builder)
        with pytest.raises(ValueError):
            reg.register("minimax", builder)

    def test_resolve_unknown_raises_keyerror(self):
        reg = TTSProviderRegistry()
        with pytest.raises(KeyError):
            reg.resolve("does-not-exist")

    def test_resolve_keyerror_lists_available(self):
        # The error message must be useful — the registry advertises
        # what IS registered so the user can see their typo.
        reg = TTSProviderRegistry()
        reg.register("minimax", lambda c: _stub_provider(**c))
        with pytest.raises(KeyError, match="Available"):
            reg.resolve("minimaxx")


class TestTTSProviderFactory:
    """Caches built instances per ``(name, config_hash)``."""

    def setup_method(self):
        # Each test gets a clean cache; the class-level cache is
        # shared state that survives across tests in the same process
        # unless reset, which would silently couple unrelated tests.
        TTSProviderFactory.reset_cache()

    def _teardown_provider(self, p: MiniMaxTTSProvider) -> None:
        # The provider owns the client it built; closing it is the
        # caller's responsibility. Without this we'd leak sockets
        # across hundreds of tests.
        asyncio.run(p.aclose())

    def test_create_returns_instance(self):
        reg = TTSProviderRegistry()
        reg.register("minimax", lambda c: _stub_provider(**c))
        try:
            p = TTSProviderFactory.create(
                "minimax", {"api_key": _FAKE_KEY, "group_id": _FAKE_GROUP}, reg
            )
            assert isinstance(p, MiniMaxTTSProvider)
        finally:
            self._teardown_provider(p)

    def test_create_caches_by_name_and_config(self):
        # Two calls with identical config return the same instance.
        # Important: long-running processes that re-resolve the same
        # provider must NOT spawn two httpx connection pools.
        reg = TTSProviderRegistry()
        call_count = {"n": 0}

        def builder(cfg):
            call_count["n"] += 1
            return _stub_provider(**cfg)

        reg.register("minimax", builder)
        cfg = {"api_key": _FAKE_KEY, "group_id": _FAKE_GROUP}
        p1 = TTSProviderFactory.create("minimax", cfg, reg)
        try:
            p2 = TTSProviderFactory.create("minimax", cfg, reg)
            assert p1 is p2
            assert call_count["n"] == 1
        finally:
            # Closing the cached instance clears it for the next test.
            TTSProviderFactory.reset_cache()

    def test_create_different_config_does_not_hit_cache(self):
        # A different config (different api_key) must produce a fresh
        # instance — caching by name alone would leak credentials
        # across users in multi-tenant setups.
        reg = TTSProviderRegistry()
        reg.register("minimax", lambda c: _stub_provider(**c))
        try:
            p1 = TTSProviderFactory.create(
                "minimax", {"api_key": "k1", "group_id": _FAKE_GROUP}, reg
            )
            p2 = TTSProviderFactory.create(
                "minimax", {"api_key": "k2", "group_id": _FAKE_GROUP}, reg
            )
            assert p1 is not p2
        finally:
            TTSProviderFactory.reset_cache()

    def test_create_unknown_name_raises(self):
        reg = TTSProviderRegistry()
        with pytest.raises(KeyError):
            TTSProviderFactory.create("nobody", {}, reg)

    def test_reset_cache_clears_state(self):
        # Test-only escape hatch — after reset, the next create()
        # builds a fresh instance.
        reg = TTSProviderRegistry()
        reg.register("minimax", lambda c: _stub_provider(**c))
        cfg = {"api_key": _FAKE_KEY, "group_id": _FAKE_GROUP}
        p1 = TTSProviderFactory.create("minimax", cfg, reg)
        TTSProviderFactory.reset_cache()
        try:
            p2 = TTSProviderFactory.create("minimax", cfg, reg)
            assert p1 is not p2
        finally:
            TTSProviderFactory.reset_cache()


class TestRegisterBuiltinTtsProviders:
    """The composition-root helper registers MiniMax under its canonical name."""

    def setup_method(self):
        TTSProviderFactory.reset_cache()

    def test_creates_fresh_registry_when_none_passed(self):
        reg = register_builtin_tts_providers()
        assert isinstance(reg, TTSProviderRegistry)
        assert "minimax" in reg.names()

    def test_appends_to_passed_in_registry(self):
        # 3rd-party packages register their own providers on top of
        # the built-ins — the helper must NOT replace an existing registry.
        reg = TTSProviderRegistry()

        def fake_builder(cfg):
            return _stub_provider(**cfg)

        reg.register("my-custom", fake_builder)
        out = register_builtin_tts_providers(reg)
        assert out is reg
        assert "minimax" in reg.names()
        assert "my-custom" in reg.names()

    def test_duplicate_registration_raises(self):
        # Calling the helper twice on the same registry would try to
        # re-register ``"minimax"`` and fail loudly. That's the
        # intended contract — see TTSProviderRegistry.register().
        reg = register_builtin_tts_providers()
        with pytest.raises(ValueError):
            register_builtin_tts_providers(reg)

    def test_builder_produces_real_provider(self):
        # The registered builder, when invoked, returns a usable
        # MiniMaxTTSProvider instance — proving end-to-end that the
        # registry, the factory, and the builder all line up.
        reg = register_builtin_tts_providers()
        builder = reg.resolve("minimax")
        instance = builder({"api_key": _FAKE_KEY, "group_id": _FAKE_GROUP})
        try:
            assert isinstance(instance, MiniMaxTTSProvider)
        finally:
            asyncio.run(instance.aclose())
        # Module re-export alias matches the implementation — pin so a
        # future refactor that moves the helper doesn't break callers
        # that import from either path.
        assert _rb is register_builtin_tts_providers


# ===========================================================================
# Cross-cutting: a third-party subclass can plug in via the extension points
# ===========================================================================


class TestThirdPartySubclassViaRegistry:
    """A fictional provider authors can subclass :class:`BaseTTSProvider`,.
    register a builder, and have ``TTSProviderFactory.create`` hand back
    their instance. This is the whole point of the extension surface."""

    def setup_method(self):
        TTSProviderFactory.reset_cache()

    @pytest.mark.asyncio
    async def test_dummy_provider_via_registry(self):
        from rob_box_llm.tts import TTSAudio, TTSChunk, TTSFormat

        class _DummyTTS(BaseTTSProvider):
            name = "dummy"

            def capabilities(self) -> TTSCapabilities:
                return TTSCapabilities(streaming=False, audio_format_pcm=True)

            def _build_request_payload(self, text, settings, voice_meta):
                return {"text": text}

            async def synthesize(self, text, *, settings=None):
                # Echo back as 1 sample — the contract is "valid TTSAudio",
                # not "real audio bytes".
                return TTSAudio(
                    samples=b"\x00\x00", sample_rate=16_000, format=TTSFormat.PCM
                )

            async def stream(self, text, *, settings=None):
                yield TTSChunk(finish_reason="stop")

        reg = TTSProviderRegistry()
        reg.register("dummy", lambda cfg: _DummyTTS())
        p = TTSProviderFactory.create("dummy", {}, reg)
        assert isinstance(p, _DummyTTS)
        # The capability hook flowed through, not the BaseTTSProvider default.
        assert p.capabilities().audio_format_pcm is True
        # And the public contract still works.
        audio = await p.synthesize("hi")
        assert audio.sample_rate == 16_000
