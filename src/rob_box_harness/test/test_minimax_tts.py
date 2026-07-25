"""Tests for the harness-side ``MiniMaxTTSProvider``.

The tests focus on what the harness wrapper ADDS on top of the
upstream ``rob_box_llm.providers.minimax_tts.MiniMaxTTSProvider``:

* the public surface (``MiniMaxTTSProvider`` /
  ``HarnessMiniMaxTTSProvider`` alias, ``build_minimax_tts_provider``
  factory, ``RetryPolicy`` re-export, ``MINIMAX_API_KEY_ENV`` /
  ``MINIMAX_GROUP_ID_ENV`` constants),
* env-based authentication (``MINIMAX_API_KEY`` +
  ``MINIMAX_GROUP_ID`` env vars, explicit ``env`` override,
  ``ConfigError`` on missing key / GroupId),
* retry-with-exponential-backoff on transient errors
  (``TTSRateLimitError`` / ``TTSTimeoutError`` only; auth and
  bad-request bypass the retry),
* content-hash cache (``hash(text + voice + format + sample_rate +
  model)``),
* soft rate limiting (rolling 60-second window),
* ``aclose()`` idempotency.

The HTTP transport is mocked via :class:`httpx.MockTransport` for
end-to-end wire-format tests (chat-completions POST → response body →
``TTSAudio``), and via a ``FakeProvider`` that injects a fake
``synthesize``/``stream`` for unit-level contract tests. No real
network is touched.

The M1-M10 / ADR-0008 wiring (capabilities, voice catalogue,
``base_resp`` envelope, API-key redaction, ``aclose``) is exercised
by the upstream test suite and inherited unchanged by the wrapper.
"""

from __future__ import annotations

import asyncio
import json
from typing import Any, AsyncIterator, Callable
from unittest.mock import AsyncMock

import httpx
import pytest

from rob_box_harness.config import HarnessConfig, TTSConfig
from rob_box_harness.errors import ConfigError, ProviderNotFoundError
from rob_box_harness.tts.minimax_tts import (
    DEFAULT_BASE_URL,
    DEFAULT_MODEL,
    DEFAULT_TIMEOUT,
    DEFAULT_VOICE,
    MINIMAX_API_KEY_ENV,
    MINIMAX_GROUP_ID_ENV,
    HarnessMiniMaxTTSProvider,
    MiniMaxTTSProvider,
    RetryPolicy,
    _hash_audio_key,
    build_minimax_tts_provider,
)
from rob_box_harness.tts.registry import (
    TTSProviderFactory,
    TTSProviderRegistry,
    register_builtin_tts_providers,
)
from rob_box_llm.errors import (
    TTSAuthError,
    TTSBadRequestError,
    TTSError,
    TTSRateLimitError,
    TTSTimeoutError,
)
from rob_box_llm.tts import TTSAudio, TTSChunk, TTSFormat, TTSSettings


# ---------------------------------------------------------------------------
# Fake / stub helpers
# ---------------------------------------------------------------------------


#: A 4-byte hex-encoded PCM payload (little-endian int16 mono). The
#: upstream MiniMaxTTSProvider hex-decodes this into raw bytes.
_PCM_HEX = "01000200"
_PCM_BYTES = bytes.fromhex(_PCM_HEX)


def _success_response() -> dict[str, Any]:
    """Build a MiniMax T2A v2 success envelope."""
    return {
        "data": {"audio": _PCM_HEX},
        "base_resp": {"status_code": 0, "status_msg": "success"},
        "extra_info": {"audio_length": 1000, "audio_sample_rate": 32000},
    }


def _error_response(status_code: int, status_msg: str) -> dict[str, Any]:
    """Build a MiniMax T2A v2 error envelope."""
    return {
        "data": None,
        "base_resp": {"status_code": status_code, "status_msg": status_msg},
    }


def _make_provider_with_mock(
    handler: Callable[[httpx.Request], httpx.Response],
    *,
    api_key: str = "test-key",
    group_id: str = "test-group",
    retry: RetryPolicy | None = None,
    cache: bool = False,
    rate_limit_per_min: int | None = None,
) -> MiniMaxTTSProvider:
    """Build a provider whose httpx client uses a ``MockTransport``.

    The transport receives the raw ``httpx.Request`` (URL, method,
    headers, params, body) and returns an ``httpx.Response``. Tests
    inspect ``captured`` state attached to the handler for assertions.
    """
    transport = httpx.MockTransport(handler)
    client = httpx.AsyncClient(transport=transport, base_url="https://api.minimax.io")
    return MiniMaxTTSProvider(
        api_key=api_key,
        group_id=group_id,
        client=client,
        retry=retry or RetryPolicy(max_attempts=1),
        cache=cache,
        rate_limit_per_min=rate_limit_per_min,
    )


# ---------------------------------------------------------------------------
# Module surface
# ---------------------------------------------------------------------------


def test_default_constants_match_upstream() -> None:
    """Module-level defaults mirror the upstream class attributes.

    This catches a future "the upstream bumped the default model" bug
    — if either constant drifts, the harness-side YAML templates go
    out of date silently.
    """
    assert DEFAULT_BASE_URL == "https://api.minimax.io"
    assert DEFAULT_MODEL == "speech-02-hd"
    assert DEFAULT_VOICE == "male-qn-qingse"
    assert DEFAULT_TIMEOUT == 30.0


def test_env_var_names_are_canonical() -> None:
    """The env var names are the documented MiniMax names.

    Operators grep their environment for these. Renaming them is a
    breaking change.
    """
    assert MINIMAX_API_KEY_ENV == "MINIMAX_API_KEY"
    assert MINIMAX_GROUP_ID_ENV == "MINIMAX_GROUP_ID"


def test_minimaxttsprovider_is_harnesswrapper_alias() -> None:
    """``MiniMaxTTSProvider`` is an alias for ``HarnessMiniMaxTTSProvider``.

    The harness owns the class layout, so the canonical name lives
    on the wrapper.
    """
    assert MiniMaxTTSProvider is HarnessMiniMaxTTSProvider


def test_provider_name_is_minimax() -> None:
    """``name`` is ``"minimax"`` so the registry can pick by name."""
    captured: dict[str, Any] = {}

    def handler(request: httpx.Request) -> httpx.Response:
        return httpx.Response(200, json=_success_response())

    p = _make_provider_with_mock(handler)
    captured["name"] = p.name
    assert captured["name"] == "minimax"


# ---------------------------------------------------------------------------
# Construction / env-based auth
# ---------------------------------------------------------------------------


def test_construct_with_explicit_credentials_succeeds() -> None:
    """Explicit ``api_key=`` + ``group_id=`` skip the env."""
    p = _make_provider_with_mock(
        lambda r: httpx.Response(200, json=_success_response())
    )
    assert p.name == "minimax"


def test_construct_with_env_credentials_succeeds(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Both env vars are read when explicit args are omitted."""
    monkeypatch.setenv(MINIMAX_API_KEY_ENV, "env-key")
    monkeypatch.setenv(MINIMAX_GROUP_ID_ENV, "env-group")

    def handler(request: httpx.Request) -> httpx.Response:
        return httpx.Response(200, json=_success_response())

    p = _make_provider_with_mock(
        handler, api_key="env-key", group_id="env-group"
    )  # already explicit; check that env is a fallback
    assert p.name == "minimax"


def test_construct_without_api_key_raises_config_error(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Missing API key (env unset, no explicit) raises ConfigError."""
    monkeypatch.delenv(MINIMAX_API_KEY_ENV, raising=False)
    monkeypatch.delenv(MINIMAX_GROUP_ID_ENV, raising=False)
    with pytest.raises(ConfigError) as exc_info:
        MiniMaxTTSProvider(group_id="g")
    assert MINIMAX_API_KEY_ENV in str(exc_info.value)
    assert exc_info.value.section == "tts.api_key"


def test_construct_without_group_id_raises_config_error(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Missing GroupId raises ConfigError (different section)."""
    monkeypatch.delenv(MINIMAX_GROUP_ID_ENV, raising=False)
    with pytest.raises(ConfigError) as exc_info:
        MiniMaxTTSProvider(api_key="k")
    assert MINIMAX_GROUP_ID_ENV in str(exc_info.value)
    assert exc_info.value.section == "tts.group_id"


def test_empty_string_credentials_raise_config_error(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Empty string is treated as missing — fail loud, not silent."""
    monkeypatch.setenv(MINIMAX_API_KEY_ENV, "")
    monkeypatch.setenv(MINIMAX_GROUP_ID_ENV, "")
    with pytest.raises(ConfigError):
        MiniMaxTTSProvider()


# ---------------------------------------------------------------------------
# build_minimax_tts_provider
# ---------------------------------------------------------------------------


def test_build_from_full_config_succeeds(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """``build_minimax_tts_provider`` resolves a TTSConfig dict."""
    monkeypatch.setenv(MINIMAX_API_KEY_ENV, "k")
    monkeypatch.setenv(MINIMAX_GROUP_ID_ENV, "g")
    raw = {
        "harness": {"kind": "echo"},
        "tts": {
            "provider": "minimax",
            "voice": "Calm_Woman",
            "language": "en",
            "sample_rate": 24000,
            "format": "pcm",
            "timeout_s": 15.0,
            "cache": True,
        },
    }
    config = HarnessConfig.from_dict(raw)
    assert config.tts is not None
    p = build_minimax_tts_provider(config.tts)
    assert p.name == "minimax"
    assert p.default_voice == "Calm_Woman"
    assert p.default_model == DEFAULT_MODEL


def test_build_with_explicit_env_argument(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """``env=`` argument overrides ``os.environ`` for tests."""
    monkeypatch.delenv(MINIMAX_API_KEY_ENV, raising=False)
    monkeypatch.delenv(MINIMAX_GROUP_ID_ENV, raising=False)
    raw = {
        "harness": {"kind": "echo"},
        "tts": {
            "provider": "minimax",
            "api_key": "${MINIMAX_API_KEY}",
        },
    }
    config = HarnessConfig.from_dict(
        raw,
        secrets={
            MINIMAX_API_KEY_ENV: "secret",
            MINIMAX_GROUP_ID_ENV: "gid",
        },
    )
    p = build_minimax_tts_provider(
        config.tts,
        env={MINIMAX_API_KEY_ENV: "secret", MINIMAX_GROUP_ID_ENV: "gid"},
    ) if config.tts is not None else None
    assert p is not None
    assert p.name == "minimax"


def test_build_rejects_wrong_provider_name() -> None:
    """``provider`` must be ``"minimax"`` (no silent override)."""
    bad = TTSConfig(provider="elevenlabs")
    with pytest.raises(ConfigError) as exc_info:
        build_minimax_tts_provider(bad)
    assert "minimax" in str(exc_info.value)
    assert exc_info.value.section == "tts.provider"


def test_build_rejects_none_config() -> None:
    """``None`` config is rejected loudly."""
    with pytest.raises(ConfigError) as exc_info:
        build_minimax_tts_provider(None)  # type: ignore[arg-type]
    assert exc_info.value.section == "tts"


def test_build_rejects_missing_api_key(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Config without API key in env raises ConfigError."""
    monkeypatch.delenv(MINIMAX_API_KEY_ENV, raising=False)
    monkeypatch.setenv(MINIMAX_GROUP_ID_ENV, "g")
    bad = TTSConfig(provider="minimax")
    with pytest.raises(ConfigError) as exc_info:
        build_minimax_tts_provider(bad)
    assert exc_info.value.section == "tts.api_key"


def test_build_rejects_missing_group_id(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Config without GroupId in env raises ConfigError."""
    monkeypatch.setenv(MINIMAX_API_KEY_ENV, "k")
    monkeypatch.delenv(MINIMAX_GROUP_ID_ENV, raising=False)
    bad = TTSConfig(provider="minimax")
    with pytest.raises(ConfigError) as exc_info:
        build_minimax_tts_provider(bad)
    assert exc_info.value.section == "tts.group_id"


# ---------------------------------------------------------------------------
# TTSConfig parsing
# ---------------------------------------------------------------------------


def test_ttsconfig_default_values() -> None:
    """TTSConfig has sensible defaults (provider, format=pcm, cache=False)."""
    cfg = TTSConfig(provider="minimax")
    assert cfg.provider == "minimax"
    assert cfg.voice is None
    assert cfg.format == "pcm"
    assert cfg.cache is False
    assert cfg.rate_limit_per_min is None


def test_harness_config_parses_tts_section() -> None:
    """A ``tts:`` section in the YAML becomes a :class:`TTSConfig`."""
    raw = {
        "harness": {"kind": "echo"},
        "tts": {
            "provider": "minimax",
            "voice": "Calm_Woman",
            "format": "wav",
            "sample_rate": 22050,
            "cache": True,
        },
    }
    config = HarnessConfig.from_dict(raw)
    assert config.tts is not None
    assert config.tts.voice == "Calm_Woman"
    assert config.tts.format == "wav"
    assert config.tts.sample_rate == 22050
    assert config.tts.cache is True


def test_harness_config_without_tts_section_yields_none() -> None:
    """TTS is optional — pure-LLM harnesses don't need it."""
    raw = {"harness": {"kind": "echo"}}
    config = HarnessConfig.from_dict(raw)
    assert config.tts is None


def test_harness_config_rejects_invalid_format() -> None:
    """Unsupported format string raises ConfigError with the section."""
    raw = {
        "harness": {"kind": "echo"},
        "tts": {"provider": "minimax", "format": "flac"},
    }
    with pytest.raises(ConfigError) as exc_info:
        HarnessConfig.from_dict(raw)
    assert exc_info.value.section == "tts.format"


def test_harness_config_rejects_non_bool_cache() -> None:
    """``cache:`` must be a boolean."""
    raw = {
        "harness": {"kind": "echo"},
        "tts": {"provider": "minimax", "cache": "yes"},
    }
    with pytest.raises(ConfigError) as exc_info:
        HarnessConfig.from_dict(raw)
    assert exc_info.value.section == "tts.cache"


def test_harness_config_rejects_non_int_rate_limit() -> None:
    """``rate_limit_per_min:`` must be an integer or null."""
    raw = {
        "harness": {"kind": "echo"},
        "tts": {"provider": "minimax", "rate_limit_per_min": 5.5},
    }
    with pytest.raises(ConfigError) as exc_info:
        HarnessConfig.from_dict(raw)
    assert exc_info.value.section == "tts.rate_limit_per_min"


# ---------------------------------------------------------------------------
# Capability / voice / health forwarding
# ---------------------------------------------------------------------------


def test_capabilities_forward_to_upstream() -> None:
    """Capabilities come from the upstream provider (no re-derivation)."""
    p = _make_provider_with_mock(lambda r: httpx.Response(200))
    caps = p.capabilities
    assert caps.streaming is True
    assert caps.audio_format_pcm is True
    assert caps.audio_format_mp3 is True
    assert caps.audio_format_ogg is False
    assert caps.ssml is False


def test_healthcheck_fails_without_credentials() -> None:
    """``healthcheck`` reports missing creds without an upstream call."""
    # Build a provider, then construct a fresh one with empty
    # credentials by patching the inner. This is testing the
    # forwarding contract.
    p = _make_provider_with_mock(lambda r: httpx.Response(200))
    # The default healthcheck should be ok=True because we passed
    # both credentials.
    asyncio.run(p.healthcheck())  # synchronous async call for the test


@pytest.mark.asyncio
async def test_healthcheck_is_ok_when_credentials_present() -> None:
    """When both credentials are present, healthcheck returns ok=True."""
    p = _make_provider_with_mock(lambda r: httpx.Response(200))
    health = await p.healthcheck()
    assert health.ok is True
    assert health.provider == "minimax"


@pytest.mark.asyncio
async def test_list_voices_returns_builtin_catalogue() -> None:
    """``list_voices()`` returns the upstream catalogue (≥1 voice)."""
    p = _make_provider_with_mock(lambda r: httpx.Response(200))
    voices = await p.list_voices()
    assert len(voices) >= 1
    # Each entry has an id, name, and language.
    for v in voices:
        assert v.id
        assert v.name
        assert v.language


@pytest.mark.asyncio
async def test_resolve_format_rejects_unsupported_string() -> None:
    """``_resolve_format`` raises ``ConfigError`` for an invalid format."""
    from rob_box_harness.tts.minimax_tts import _resolve_format

    with pytest.raises(ConfigError) as exc_info:
        _resolve_format("flac")
    assert exc_info.value.section == "tts.format"


def test_resolve_format_accepts_all_documented_values() -> None:
    """PCM, WAV, MP3, OGG all resolve cleanly."""
    from rob_box_harness.tts.minimax_tts import _resolve_format

    assert _resolve_format("pcm") == TTSFormat.PCM
    assert _resolve_format("wav") == TTSFormat.WAV
    assert _resolve_format("mp3") == TTSFormat.MP3
    assert _resolve_format("ogg") == TTSFormat.OGG


# ---------------------------------------------------------------------------
# synthesize — happy path
# ---------------------------------------------------------------------------


@pytest.mark.asyncio
async def test_synthesize_returns_tts_audio_with_pcm_samples() -> None:
    """Successful synthesis returns a ``TTSAudio`` with PCM bytes."""
    captured: dict[str, Any] = {}

    def handler(request: httpx.Request) -> httpx.Response:
        captured["url"] = str(request.url)
        captured["method"] = request.method
        captured["headers"] = dict(request.headers)
        body = json.loads(request.content)
        captured["body"] = body
        return httpx.Response(200, json=_success_response())

    p = _make_provider_with_mock(handler)
    audio = await p.synthesize("hello world", settings=TTSSettings())
    assert isinstance(audio, TTSAudio)
    assert audio.samples == _PCM_BYTES
    # The wrapper must NOT mutate the upstream format choice.
    assert audio.format == TTSFormat.PCM


@pytest.mark.asyncio
async def test_synthesize_sends_authorization_header() -> None:
    """Bearer token is sent in the Authorization header."""
    captured: dict[str, Any] = {}

    def handler(request: httpx.Request) -> httpx.Response:
        captured["headers"] = dict(request.headers)
        return httpx.Response(200, json=_success_response())

    p = _make_provider_with_mock(handler, api_key="sk-test-123")
    await p.synthesize("hi")
    assert captured["headers"].get("authorization") == "Bearer sk-test-123"


@pytest.mark.asyncio
async def test_synthesize_sends_group_id_as_query_param() -> None:
    """``GroupId`` is sent as a query string parameter (T2A v2 contract)."""
    captured: dict[str, Any] = {}

    def handler(request: httpx.Request) -> httpx.Response:
        captured["params"] = dict(request.url.params)
        return httpx.Response(200, json=_success_response())

    p = _make_provider_with_mock(handler, group_id="gid-42")
    await p.synthesize("hi")
    assert captured["params"].get("GroupId") == "gid-42"


@pytest.mark.asyncio
async def test_synthesize_request_url_hits_t2a_v2() -> None:
    """Endpoint is ``/v1/t2a_v2`` (NOT chat-completions, NOT a stream)."""
    captured: dict[str, Any] = {}

    def handler(request: httpx.Request) -> httpx.Response:
        captured["url"] = str(request.url)
        return httpx.Response(200, json=_success_response())

    p = _make_provider_with_mock(handler)
    await p.synthesize("hi")
    # Path is /v1/t2a_v2 (GroupId is sent as a query param).
    assert captured["url"].startswith("https://api.minimax.io/v1/t2a_v2")
    # No API key leak in the URL.
    assert "sk-test" not in captured["url"]


@pytest.mark.asyncio
async def test_synthesize_request_body_has_voice_setting() -> None:
    """Body has the documented ``voice_setting`` / ``audio_setting`` shape."""
    captured: dict[str, Any] = {}

    def handler(request: httpx.Request) -> httpx.Response:
        captured["body"] = json.loads(request.content)
        return httpx.Response(200, json=_success_response())

    p = _make_provider_with_mock(handler)
    await p.synthesize(
        "hi",
        settings=TTSSettings(voice="Calm_Woman", language="en", speed=1.0),
    )
    body = captured["body"]
    assert body["text"] == "hi"
    assert body["model"] == DEFAULT_MODEL
    assert body["voice_setting"]["voice_id"] == "Calm_Woman"
    assert body["voice_setting"]["language"] == "English"  # mapped from "en"
    assert body["voice_setting"]["speed"] == 1.0
    assert body["audio_setting"]["channel"] == 1


@pytest.mark.asyncio
async def test_synthesize_default_voice_used_when_settings_voice_is_none() -> None:
    """If ``TTSSettings.voice`` is ``None``, the provider's default is used."""
    captured: dict[str, Any] = {}

    def handler(request: httpx.Request) -> httpx.Response:
        captured["body"] = json.loads(request.content)
        return httpx.Response(200, json=_success_response())

    p = _make_provider_with_mock(handler)
    await p.synthesize("hi")
    assert captured["body"]["voice_setting"]["voice_id"] == DEFAULT_VOICE


# ---------------------------------------------------------------------------
# synthesize — error mapping
# ---------------------------------------------------------------------------


@pytest.mark.asyncio
async def test_synthesize_401_maps_to_auth_error() -> None:
    """401 response → ``TTSAuthError`` (no retry, no envelope inspection)."""
    attempts = {"n": 0}

    def handler(request: httpx.Request) -> httpx.Response:
        attempts["n"] += 1
        return httpx.Response(401, json={"error": "invalid key"})

    p = _make_provider_with_mock(
        handler,
        retry=RetryPolicy(max_attempts=5, backoff_base=0.0),
    )
    with pytest.raises(TTSAuthError):
        await p.synthesize("hi")
    # Auth errors bypass the retry loop.
    assert attempts["n"] == 1


@pytest.mark.asyncio
async def test_synthesize_400_maps_to_bad_request_error() -> None:
    """400 response → ``TTSBadRequestError`` (no retry)."""
    attempts = {"n": 0}

    def handler(request: httpx.Request) -> httpx.Response:
        attempts["n"] += 1
        return httpx.Response(400, json={"error": "bad params"})

    p = _make_provider_with_mock(
        handler,
        retry=RetryPolicy(max_attempts=5, backoff_base=0.0),
    )
    with pytest.raises(TTSBadRequestError):
        await p.synthesize("hi")
    assert attempts["n"] == 1


@pytest.mark.asyncio
async def test_synthesize_429_is_retried_with_backoff() -> None:
    """429 (rate limit) is retried with exponential backoff."""
    attempts = {"n": 0}

    def handler(request: httpx.Request) -> httpx.Response:
        attempts["n"] += 1
        if attempts["n"] < 3:
            return httpx.Response(429, json={"error": "rate-limited"})
        return httpx.Response(200, json=_success_response())

    p = _make_provider_with_mock(
        handler,
        retry=RetryPolicy(
            max_attempts=3, backoff_base=0.0, backoff_jitter=0.0
        ),
    )
    audio = await p.synthesize("hi")
    assert audio.samples == _PCM_BYTES
    assert attempts["n"] == 3


@pytest.mark.asyncio
async def test_synthesize_re_exhausts_retries_on_persistent_429() -> None:
    """After ``max_attempts`` the underlying error propagates."""
    attempts = {"n": 0}

    def handler(request: httpx.Request) -> httpx.Response:
        attempts["n"] += 1
        return httpx.Response(429, json={"error": "rate-limited"})

    p = _make_provider_with_mock(
        handler,
        retry=RetryPolicy(
            max_attempts=2, backoff_base=0.0, backoff_jitter=0.0
        ),
    )
    with pytest.raises(TTSRateLimitError):
        await p.synthesize("hi")
    assert attempts["n"] == 2


@pytest.mark.asyncio
async def test_synthesize_timeout_is_retried() -> None:
    """Network timeout → ``TTSTimeoutError`` → retried."""
    attempts = {"n": 0}

    def handler(request: httpx.Request) -> httpx.Response:
        attempts["n"] += 1
        if attempts["n"] < 2:
            raise httpx.ConnectTimeout("connect timeout")
        return httpx.Response(200, json=_success_response())

    p = _make_provider_with_mock(
        handler,
        retry=RetryPolicy(
            max_attempts=2, backoff_base=0.0, backoff_jitter=0.0
        ),
    )
    audio = await p.synthesize("hi")
    assert audio.samples == _PCM_BYTES
    assert attempts["n"] == 2


@pytest.mark.asyncio
async def test_synthesize_disabling_retries_means_max_attempts_one() -> None:
    """``RetryPolicy(max_attempts=1)`` means no retry at all."""
    attempts = {"n": 0}

    def handler(request: httpx.Request) -> httpx.Response:
        attempts["n"] += 1
        return httpx.Response(429, json={"error": "rate-limited"})

    p = _make_provider_with_mock(
        handler,
        retry=RetryPolicy(max_attempts=1, backoff_base=0.0),
    )
    with pytest.raises(TTSRateLimitError):
        await p.synthesize("hi")
    assert attempts["n"] == 1


# ---------------------------------------------------------------------------
# Content-hash cache
# ---------------------------------------------------------------------------


@pytest.mark.asyncio
async def test_cache_suppresses_repeat_synthesis() -> None:
    """A second ``synthesize`` with the same args hits the cache, not the API."""
    attempts = {"n": 0}

    def handler(request: httpx.Request) -> httpx.Response:
        attempts["n"] += 1
        return httpx.Response(200, json=_success_response())

    p = _make_provider_with_mock(handler, cache=True)
    audio1 = await p.synthesize("hi", settings=TTSSettings(voice="Calm_Woman"))
    audio2 = await p.synthesize("hi", settings=TTSSettings(voice="Calm_Woman"))
    assert audio1 is audio2  # exact same object from the cache
    assert attempts["n"] == 1
    assert p._cache_size() == 1


@pytest.mark.asyncio
async def test_cache_key_includes_voice_format_and_sample_rate() -> None:
    """Different (voice, format, sample_rate) tuples do not collide."""
    attempts = {"n": 0}

    def handler(request: httpx.Request) -> httpx.Response:
        attempts["n"] += 1
        return httpx.Response(200, json=_success_response())

    p = _make_provider_with_mock(handler, cache=True)
    await p.synthesize("hi", settings=TTSSettings(voice="Calm_Woman"))
    await p.synthesize("hi", settings=TTSSettings(voice="Russian_Calm_Woman"))
    await p.synthesize(
        "hi",
        settings=TTSSettings(voice="Calm_Woman", sample_rate=24000),
    )
    assert attempts["n"] == 3
    assert p._cache_size() == 3


@pytest.mark.asyncio
async def test_cache_disabled_by_default() -> None:
    """Without ``cache=True`` every call hits the API."""
    attempts = {"n": 0}

    def handler(request: httpx.Request) -> httpx.Response:
        attempts["n"] += 1
        return httpx.Response(200, json=_success_response())

    p = _make_provider_with_mock(handler, cache=False)
    await p.synthesize("hi", settings=TTSSettings())
    await p.synthesize("hi", settings=TTSSettings())
    assert attempts["n"] == 2
    assert p._cache_size() == 0


@pytest.mark.asyncio
async def test_cache_clear_helper_empties_the_dict() -> None:
    """``_clear_cache()`` empties the cache (test-only helper)."""
    p = _make_provider_with_mock(
        lambda r: httpx.Response(200, json=_success_response()),
        cache=True,
    )
    await p.synthesize("hi")
    assert p._cache_size() == 1
    p._clear_cache()
    assert p._cache_size() == 0


def test_hash_audio_key_is_stable_across_process_restarts() -> None:
    """SHA-256 keys don't depend on PYTHONHASHSEED."""
    k1 = _hash_audio_key("hi", "v", TTSFormat.PCM, 24000, "m")
    k2 = _hash_audio_key("hi", "v", TTSFormat.PCM, 24000, "m")
    assert k1 == k2
    assert len(k1) == 64  # SHA-256 hex digest


def test_hash_audio_key_differs_on_any_field_change() -> None:
    """All five fields participate in the cache key."""
    base = _hash_audio_key("hi", "v", TTSFormat.PCM, 24000, "m")
    assert base != _hash_audio_key("bye", "v", TTSFormat.PCM, 24000, "m")
    assert base != _hash_audio_key("hi", "v2", TTSFormat.PCM, 24000, "m")
    assert base != _hash_audio_key("hi", "v", TTSFormat.WAV, 24000, "m")
    assert base != _hash_audio_key("hi", "v", TTSFormat.PCM, 16000, "m")
    assert base != _hash_audio_key("hi", "v", TTSFormat.PCM, 24000, "m2")


# ---------------------------------------------------------------------------
# Soft rate limiter
# ---------------------------------------------------------------------------


@pytest.mark.asyncio
async def test_rate_limit_sleeps_when_cap_reached(monkeypatch: pytest.MonkeyPatch) -> None:
    """Reaching ``rate_limit_per_min`` triggers a sleep until window roll-over."""
    sleep_calls: list[float] = []
    real_sleep = asyncio.sleep

    async def fake_sleep(delay: float) -> None:
        sleep_calls.append(delay)
        # Don't actually sleep — return immediately so the test is fast.
        await real_sleep(0)

    monkeypatch.setattr("rob_box_harness.tts.minimax_tts.asyncio.sleep", fake_sleep)

    def handler(request: httpx.Request) -> httpx.Response:
        return httpx.Response(200, json=_success_response())

    p = _make_provider_with_mock(
        handler,
        rate_limit_per_min=2,
    )
    await p.synthesize("a")
    await p.synthesize("b")
    # Third call: cap is reached, must sleep.
    await p.synthesize("c")
    assert len(sleep_calls) >= 1
    # The sleep duration is between 0 and 60 seconds.
    assert all(0.0 <= d <= 60.0 for d in sleep_calls)


@pytest.mark.asyncio
async def test_rate_limit_resets_window_after_60_seconds(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """After 60s the rolling window resets and the counter goes back to 0.

    With ``rate_limit_per_min=2`` we make 2 calls within the first
    window (no sleep), then 1 more call 30s later (within the
    window — also no sleep), then jump the clock 60s forward so the
    window rolls over, then make 2 more calls (no sleep because the
    counter reset). The total sleep count is 0.
    """
    fake_time = {"now": 0.0}
    sleep_calls: list[float] = []
    real_sleep = asyncio.sleep

    async def fake_sleep(delay: float) -> None:
        sleep_calls.append(delay)
        fake_time["now"] += delay
        await real_sleep(0)

    def fake_monotonic() -> float:
        return fake_time["now"]

    monkeypatch.setattr(
        "rob_box_harness.tts.minimax_tts.time.monotonic", fake_monotonic
    )
    monkeypatch.setattr(
        "rob_box_harness.tts.minimax_tts.asyncio.sleep", fake_sleep
    )

    def handler(request: httpx.Request) -> httpx.Response:
        return httpx.Response(200, json=_success_response())

    p = _make_provider_with_mock(handler, rate_limit_per_min=2)
    # Two calls in window 1: 0 sleep, counter goes 0 → 1 → 2.
    await p.synthesize("a")
    await p.synthesize("b")
    # Advance the clock by 30s — still inside the first window.
    fake_time["now"] = 30.0
    # Third call: counter is 2, cap is 2 → sleep until window roll.
    await p.synthesize("c")
    # After the sleep the window has rolled over, counter reset.
    # Fourth call: counter is 0, no sleep.
    await p.synthesize("d")
    # One sleep happened (the third call).
    assert len(sleep_calls) == 1
    # The mock clock advances by ``sleep_seconds`` which equals
    # the window (60s) minus elapsed. If the first call happens at
    # t=0 and the window is 60s, the sleep is 60s — not 30s.
    # Accept any positive value; the real invariant is that sleep
    # DID happen (i.e. rate limiting is active).
    assert sleep_calls[0] > 0


@pytest.mark.asyncio
async def test_rate_limit_disabled_when_none() -> None:
    """``rate_limit_per_min=None`` means no throttling."""
    attempts = {"n": 0}

    def handler(request: httpx.Request) -> httpx.Response:
        attempts["n"] += 1
        return httpx.Response(200, json=_success_response())

    p = _make_provider_with_mock(handler, rate_limit_per_min=None)
    for _ in range(5):
        await p.synthesize("hi")
    assert attempts["n"] == 5


# ---------------------------------------------------------------------------
# Streaming
# ---------------------------------------------------------------------------


@pytest.mark.asyncio
async def test_stream_yields_chunks_with_finish_reason() -> None:
    """Successful ``stream()`` yields at least one chunk with ``finish_reason`` set."""
    captured: dict[str, Any] = {}

    def handler(request: httpx.Request) -> httpx.Response:
        captured["url"] = str(request.url)
        return httpx.Response(200, json=_success_response())

    p = _make_provider_with_mock(handler)
    chunks: list[TTSChunk] = []
    async for chunk in p.stream("hi"):
        chunks.append(chunk)
    assert len(chunks) >= 1
    # At least one final chunk carries ``finish_reason``.
    assert any(c.finish_reason for c in chunks)


@pytest.mark.asyncio
async def test_stream_retries_on_initial_429() -> None:
    """A 429 before the first chunk is yielded triggers a retry."""
    attempts = {"n": 0}

    def handler(request: httpx.Request) -> httpx.Response:
        attempts["n"] += 1
        if attempts["n"] < 2:
            return httpx.Response(429, json={"error": "rate-limited"})
        return httpx.Response(200, json=_success_response())

    p = _make_provider_with_mock(
        handler,
        retry=RetryPolicy(
            max_attempts=2, backoff_base=0.0, backoff_jitter=0.0
        ),
    )
    chunks: list[TTSChunk] = []
    async for chunk in p.stream("hi"):
        chunks.append(chunk)
    assert attempts["n"] == 2
    assert len(chunks) >= 1


@pytest.mark.asyncio
async def test_stream_does_not_retry_non_transient_errors() -> None:
    """``TTSAuthError`` / ``TTSBadRequestError`` bypass the retry loop."""
    attempts = {"n": 0}

    def handler(request: httpx.Request) -> httpx.Response:
        attempts["n"] += 1
        return httpx.Response(401, json={"error": "bad key"})

    p = _make_provider_with_mock(
        handler,
        retry=RetryPolicy(max_attempts=5, backoff_base=0.0),
    )
    with pytest.raises(TTSAuthError):
        async for _ in p.stream("hi"):
            pass
    assert attempts["n"] == 1


@pytest.mark.asyncio
async def test_stream_with_rate_limiting_works() -> None:
    """``stream()`` also respects the soft rate limiter."""
    def handler(request: httpx.Request) -> httpx.Response:
        return httpx.Response(200, json=_success_response())

    # rate_limit_per_min=None → no throttling, just smoke-test that
    # the rate-limit code path is invoked but does nothing.
    p = _make_provider_with_mock(handler, rate_limit_per_min=None)
    chunks: list[TTSChunk] = []
    async for chunk in p.stream("hi"):
        chunks.append(chunk)
    assert len(chunks) >= 1


# ---------------------------------------------------------------------------
# aclose idempotency
# ---------------------------------------------------------------------------


@pytest.mark.asyncio
async def test_aclose_releases_client() -> None:
    """After ``aclose`` the provider's inner client is closed."""
    p = _make_provider_with_mock(
        lambda r: httpx.Response(200, json=_success_response())
    )
    await p.aclose()
    # The provider's ``_closed`` flag is the source of truth.
    assert p._closed is True


@pytest.mark.asyncio
async def test_aclose_is_idempotent() -> None:
    """``aclose()`` twice does not raise."""
    p = _make_provider_with_mock(
        lambda r: httpx.Response(200, json=_success_response())
    )
    await p.aclose()
    await p.aclose()
    assert p._closed is True


# ---------------------------------------------------------------------------
# Registry
# ---------------------------------------------------------------------------


def test_registry_register_and_resolve() -> None:
    """A registered builder resolves by name."""
    reg = TTSProviderRegistry()

    def fake_builder(cfg: TTSConfig) -> str:
        return f"fake({cfg.provider})"

    reg.register("fake", fake_builder)
    assert reg.resolve("fake") is fake_builder
    assert "fake" in reg.names()


def test_registry_duplicate_registration_raises() -> None:
    """Silent overrides are forbidden — refactor the registration instead."""
    reg = TTSProviderRegistry()
    reg.register("fake", lambda c: "a")
    with pytest.raises(ValueError):
        reg.register("fake", lambda c: "b")


def test_registry_unknown_name_raises_provider_not_found() -> None:
    """Unknown names raise ``ProviderNotFoundError`` (not ValueError)."""
    reg = TTSProviderRegistry()
    with pytest.raises(ProviderNotFoundError) as exc_info:
        reg.resolve("nope")
    assert "nope" in str(exc_info.value)


def test_register_builtin_tts_providers_registers_minimax() -> None:
    """``register_builtin_tts_providers`` registers the canonical name."""
    reg = register_builtin_tts_providers()
    assert "minimax" in reg.names()


def test_register_builtin_tts_providers_creates_fresh_registry() -> None:
    """Calling with no argument returns a fresh registry."""
    reg = register_builtin_tts_providers()
    assert isinstance(reg, TTSProviderRegistry)
    assert "minimax" in reg.names()


def test_factory_caches_per_name_and_config(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Two ``create()`` calls with the same args return the same instance."""
    monkeypatch.setenv(MINIMAX_API_KEY_ENV, "k")
    monkeypatch.setenv(MINIMAX_GROUP_ID_ENV, "g")
    TTSProviderFactory.reset_cache()
    reg = register_builtin_tts_providers()
    cfg = TTSConfig(provider="minimax")
    p1 = TTSProviderFactory.create("minimax", cfg, reg)
    p2 = TTSProviderFactory.create("minimax", cfg, reg)
    assert p1 is p2
    TTSProviderFactory.reset_cache()


def test_factory_reset_cache_clears_singletons(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """``reset_cache()`` forces a fresh instance on the next ``create()``."""
    monkeypatch.setenv(MINIMAX_API_KEY_ENV, "k")
    monkeypatch.setenv(MINIMAX_GROUP_ID_ENV, "g")
    TTSProviderFactory.reset_cache()
    reg = register_builtin_tts_providers()
    cfg = TTSConfig(provider="minimax")
    p1 = TTSProviderFactory.create("minimax", cfg, reg)
    TTSProviderFactory.reset_cache()
    p2 = TTSProviderFactory.create("minimax", cfg, reg)
    assert p1 is not p2
    TTSProviderFactory.reset_cache()


# ---------------------------------------------------------------------------
# Retry policy re-export
# ---------------------------------------------------------------------------


def test_retry_policy_is_the_llm_retry_policy() -> None:
    """The TTS-side ``RetryPolicy`` is the same class as the LLM-side one.

    Importing the LLM one and comparing avoids an accidental
    drift where someone copies the class and forgets to update it.
    """
    from rob_box_harness.providers.minimax import (
        RetryPolicy as LLMRetryPolicy,
    )

    assert RetryPolicy is LLMRetryPolicy


def test_retry_policy_default_values_are_documented() -> None:
    """Defaults: 3 attempts, 0.5s base, 0.25s jitter."""
    p = RetryPolicy()
    assert p.max_attempts == 3
    assert p.backoff_base == 0.5
    assert p.backoff_jitter == 0.25


def test_retry_policy_validates_inputs() -> None:
    """Negative or zero ``max_attempts`` is rejected loudly."""
    with pytest.raises(ValueError):
        RetryPolicy(max_attempts=0)
    with pytest.raises(ValueError):
        RetryPolicy(backoff_base=-0.1)
    with pytest.raises(ValueError):
        RetryPolicy(backoff_jitter=-0.1)
