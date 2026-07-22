"""Tests for ``BaseTTSProvider`` extension surface (t_25b8e221 / ADR-0008).

These tests verify:

* ``BaseTTSProvider`` IS-A ``TTSProvider`` (backward-compat contract).
* ``MiniMaxTTSProvider`` correctly fills all 5 extension points.
* ``TTSProviderRegistry`` and ``TTSProviderFactory`` work as advertised.
* The build cache in ``TTSProviderFactory`` is keyed by ``(name, config_hash)``.
* Existing ``MiniMaxTTSProvider`` behaviour is preserved after the migration
  to ``BaseTTSProvider`` (covered by the existing 204-test suite; we just
  add explicit coverage for the new hooks here).
"""

from __future__ import annotations

import asyncio
from typing import Any, AsyncIterator, Mapping

import httpx
import pytest

from rob_box_llm import (
    BaseTTSProvider,
    ProviderBuilder,
    TTSAudio,
    TTSCapabilities,
    TTSChunk,
    TTSHealth,
    TTSProvider,
    TTSProviderFactory,
    TTSProviderRegistry,
    TTSVoice,
    register_builtin_tts_providers,
)
from rob_box_llm.providers.minimax_tts import MiniMaxTTSProvider
from rob_box_llm.tts import TTSSettings


class _AnonBase(BaseTTSProvider):
    """Minimal ``BaseTTSProvider`` subclass for default-value tests.

    Implements only the 3 abstract methods (``synthesize`` / ``stream``
    inherited from ``TTSProvider``, plus ``_build_request_payload``). The
    optional extension points keep their default no-op behaviour — which
    is exactly what we want to test.
    """

    name = "anon"

    def _build_request_payload(
        self,
        text: str,
        settings: TTSSettings,
        voice_meta: TTSVoice | None,
    ) -> dict[str, Any]:
        return {}

    async def synthesize(
        self,
        text: str,
        *,
        settings: TTSSettings | None = None,
    ) -> TTSAudio:
        return TTSAudio(samples=b"", sample_rate=16000)

    async def stream(
        self,
        text: str,
        *,
        settings: TTSSettings | None = None,
    ) -> AsyncIterator[TTSChunk]:
        yield TTSChunk(finish_reason="stop")


# ---------------------------------------------------------------------------
# BaseTTSProvider contract
# ---------------------------------------------------------------------------


def test_base_tts_provider_is_a_tts_provider() -> None:
    """``BaseTTSProvider`` IS-A ``TTSProvider`` (forward-compat guarantee).

    The extension-points design (ADR-0008) explicitly requires that every
    existing call site type-annotating ``TTSProvider`` continues to accept
    a ``BaseTTSProvider`` instance.
    """
    assert issubclass(BaseTTSProvider, TTSProvider)


def test_base_tts_provider_default_capabilities_are_empty() -> None:
    """Default ``capabilities()`` returns all-False — capability-honest default."""
    provider = _AnonBase()
    caps = provider.capabilities()
    assert caps == TTSCapabilities()
    # every flag must be False
    for field in TTSCapabilities.__dataclass_fields__:
        assert getattr(caps, field) is False, field


def test_base_tts_provider_default_list_voices_is_empty() -> None:
    provider = _AnonBase()
    assert asyncio.run(provider.list_voices()) == []


def test_base_tts_provider_default_healthcheck_is_ok() -> None:
    provider = _AnonBase()
    health = asyncio.run(provider.healthcheck())
    assert health.ok is True
    assert health.provider == "anon"
    assert health.reason is None


def test_base_tts_provider_cannot_be_instantiated_directly() -> None:
    """``BaseTTSProvider`` is abstract — must fail to instantiate.

    It carries abstract methods ``synthesize``, ``stream`` (from
    ``TTSProvider``) and ``_build_request_payload`` (own).
    """
    with pytest.raises(TypeError):
        BaseTTSProvider()  # type: ignore[abstract]


def test_base_tts_provider_default_http_client_factory_uses_timeout() -> None:
    """Default factory respects ``_timeout`` if set on the subclass."""

    class _AnonTimeout(_AnonBase):
        _timeout: float = 7.5

    provider = _AnonTimeout()
    client = provider._http_client_factory()
    try:
        # httpx.Timeout exposes ``connect``/``read`` etc; total == our timeout.
        assert client.timeout.connect == 7.5
    finally:
        asyncio.run(client.aclose())


# ---------------------------------------------------------------------------
# MiniMaxTTSProvider — extension-point fills
# ---------------------------------------------------------------------------


@pytest.fixture
def provider() -> MiniMaxTTSProvider:
    """Build a fully-configured MiniMax provider for extension-point tests."""
    return MiniMaxTTSProvider(api_key="test-key", group_id="test-group")


def test_minimax_is_base_tts_provider(provider: MiniMaxTTSProvider) -> None:
    assert isinstance(provider, BaseTTSProvider)
    assert isinstance(provider, TTSProvider)
    assert provider.name == "minimax"


def test_minimax_capabilities_match_documented_surface(provider: MiniMaxTTSProvider) -> None:
    caps = provider.capabilities()
    assert caps.streaming is True  # SSE
    assert caps.voice_cloning is True  # timbre_weights via extra
    assert caps.ssml is False
    assert caps.pronunciation_dict is False  # opt-in via extra only
    assert caps.audio_format_pcm is True
    assert caps.audio_format_mp3 is True
    assert caps.audio_format_ogg is False  # API rejects; we fall back to MP3
    assert caps.custom_endpoint is False


def test_minimax_list_voices_returns_static_catalogue(provider: MiniMaxTTSProvider) -> None:
    voices = asyncio.run(provider.list_voices())
    assert len(voices) >= 4  # we ship at least 4 well-known voices
    assert all(isinstance(v, TTSVoice) for v in voices)
    ids = {v.id for v in voices}
    # Sanity-check a couple of well-documented voices.
    assert "male-qn-qingse" in ids
    assert "Russian_Husky_Man" in ids or "Russian_Calm_Woman" in ids


def test_minimax_healthcheck_ok_when_credentials_configured() -> None:
    provider = MiniMaxTTSProvider(api_key="k", group_id="g")
    health = asyncio.run(provider.healthcheck())
    assert health.ok is True
    assert health.provider == "minimax"
    assert health.reason is None
    assert health.latency_ms >= 0.0


def test_minimax_healthcheck_fails_when_api_key_missing(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.delenv("MINIMAX_API_KEY", raising=False)
    monkeypatch.delenv("MINIMAX_GROUP_ID", raising=False)
    provider = MiniMaxTTSProvider()
    health = asyncio.run(provider.healthcheck())
    assert health.ok is False
    assert health.reason is not None
    assert "MINIMAX_API_KEY" in health.reason


def test_minimax_healthcheck_fails_when_group_id_missing(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.delenv("MINIMAX_API_KEY", raising=False)
    monkeypatch.delenv("MINIMAX_GROUP_ID", raising=False)
    provider = MiniMaxTTSProvider(api_key="k")
    health = asyncio.run(provider.healthcheck())
    assert health.ok is False
    assert "MINIMAX_GROUP_ID" in health.reason


def test_minimax_build_request_payload_uses_settings_voice(provider: MiniMaxTTSProvider) -> None:
    s = TTSSettings(voice="Russian_Husky_Man", language="ru")
    payload = provider._build_request_payload("привет", s, voice_meta=None)
    assert payload["voice_setting"]["voice_id"] == "Russian_Husky_Man"
    assert payload["voice_setting"]["language"] == "Russian"


def test_minimax_build_request_payload_falls_back_to_voice_meta(provider: MiniMaxTTSProvider) -> None:
    """If ``TTSSettings.voice`` is unset but ``voice_meta`` is given, use it."""
    s = TTSSettings()  # voice=None
    voice_meta = TTSVoice(
        id="Russian_Calm_Woman",
        name="Russian Calm Woman",
        language="ru",
        gender="female",
    )
    payload = provider._build_request_payload("привет", s, voice_meta=voice_meta)
    assert payload["voice_setting"]["voice_id"] == "Russian_Calm_Woman"


def test_minimax_build_request_payload_settings_voice_wins_over_meta(provider: MiniMaxTTSProvider) -> None:
    """Explicit ``TTSSettings.voice`` takes precedence over ``voice_meta``."""
    s = TTSSettings(voice="ExplicitVoice")
    voice_meta = TTSVoice(
        id="MetaVoice",
        name="Meta Voice",
        language="ru",
    )
    payload = provider._build_request_payload("x", s, voice_meta=voice_meta)
    assert payload["voice_setting"]["voice_id"] == "ExplicitVoice"


def test_minimax_http_client_factory_uses_configured_timeout() -> None:
    provider = MiniMaxTTSProvider(api_key="k", group_id="g", timeout=12.5)
    client = provider._http_client_factory()
    try:
        assert client.timeout.connect == 12.5
    finally:
        asyncio.run(client.aclose())


# ---------------------------------------------------------------------------
# Registry / factory
# ---------------------------------------------------------------------------


def test_registry_register_and_resolve() -> None:
    registry = TTSProviderRegistry()

    def _builder(config: Mapping[str, Any]) -> BaseTTSProvider:
        return MiniMaxTTSProvider(**config)

    registry.register("minimax", _builder)
    assert "minimax" in registry.names()
    assert registry.resolve("minimax") is _builder


def test_registry_register_rejects_duplicate() -> None:
    registry = TTSProviderRegistry()

    def _builder(config: Mapping[str, Any]) -> BaseTTSProvider:
        return MiniMaxTTSProvider(**config)

    registry.register("minimax", _builder)
    with pytest.raises(ValueError, match="already registered"):
        registry.register("minimax", _builder)


def test_registry_resolve_raises_for_unknown_name() -> None:
    registry = TTSProviderRegistry()
    with pytest.raises(KeyError, match="Unknown TTS provider"):
        registry.resolve("does-not-exist")


def test_registry_unregister_removes_entry() -> None:
    registry = TTSProviderRegistry()

    def _builder(config: Mapping[str, Any]) -> BaseTTSProvider:
        return MiniMaxTTSProvider(**config)

    registry.register("minimax", _builder)
    registry.unregister("minimax")
    assert "minimax" not in registry.names()


def test_factory_creates_cached_instance() -> None:
    TTSProviderFactory.reset_cache()
    registry = register_builtin_tts_providers()
    cfg = {"api_key": "k", "group_id": "g"}
    p1 = TTSProviderFactory.create("minimax", cfg, registry)
    p2 = TTSProviderFactory.create("minimax", cfg, registry)
    assert p1 is p2  # cached


def test_factory_distinguishes_by_config_hash() -> None:
    """Same name with different config yields a fresh instance."""
    TTSProviderFactory.reset_cache()
    registry = register_builtin_tts_providers()
    p1 = TTSProviderFactory.create(
        "minimax", {"api_key": "k1", "group_id": "g1"}, registry
    )
    p2 = TTSProviderFactory.create(
        "minimax", {"api_key": "k2", "group_id": "g2"}, registry
    )
    assert p1 is not p2


def test_factory_raises_for_unknown_provider() -> None:
    TTSProviderFactory.reset_cache()
    registry = TTSProviderRegistry()
    with pytest.raises(KeyError, match="Unknown TTS provider"):
        TTSProviderFactory.create("unknown", {}, registry)


def test_factory_reset_cache_clears_singleton() -> None:
    TTSProviderFactory.reset_cache()
    registry = register_builtin_tts_providers()
    cfg = {"api_key": "k", "group_id": "g"}
    p1 = TTSProviderFactory.create("minimax", cfg, registry)
    TTSProviderFactory.reset_cache()
    p2 = TTSProviderFactory.create("minimax", cfg, registry)
    assert p1 is not p2


def test_register_builtin_providers_includes_minimax() -> None:
    registry = TTSProviderRegistry()
    register_builtin_tts_providers(registry)
    assert "minimax" in registry.names()


def test_register_builtin_providers_returns_fresh_registry_by_default() -> None:
    """Calling without args produces a new registry — never mutates a shared one."""
    r1 = register_builtin_tts_providers()
    r2 = register_builtin_tts_providers()
    assert r1 is not r2
    # but both should still know about minimax
    assert "minimax" in r1.names()
    assert "minimax" in r2.names()


# ---------------------------------------------------------------------------
# End-to-end through the registry: BaseTTSProvider contract is preserved
# ---------------------------------------------------------------------------


def test_registry_built_provider_passes_tts_provider_typecheck() -> None:
    """The whole point of ``BaseTTSProvider`` IS-A ``TTSProvider`` — verify
    that the registry output satisfies ``TTSProvider``-typed callers.
    """
    TTSProviderFactory.reset_cache()
    registry = register_builtin_tts_providers()
    provider = TTSProviderFactory.create(
        "minimax", {"api_key": "k", "group_id": "g"}, registry
    )
    # type-narrow: must be both
    typed_as_base: BaseTTSProvider = provider
    typed_as_legacy: TTSProvider = provider
    assert typed_as_base is typed_as_legacy is provider
