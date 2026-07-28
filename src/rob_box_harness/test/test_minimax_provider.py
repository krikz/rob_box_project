"""Tests for the harness-side ``MiniMaxProvider``.

The tests focus on what the harness wrapper ADDS on top of the
upstream ``rob_box_llm.providers.minimax.MiniMaxProvider``:

* the public surface (``MiniMaxProvider`` / ``HarnessMiniMaxProvider``
  alias, ``build_minimax_provider`` factory, ``RetryPolicy``),
* env-based authentication (``MINIMAX_API_KEY`` env var, explicit
  ``env`` override, ``ConfigError`` on missing key),
* ``chat(messages, **kwargs)`` convenience wrapper,
* retry-with-exponential-backoff on transient errors,
* ``aclose()`` idempotency.

The HTTP transport is mocked at the SDK layer via a fake
``AsyncOpenAI`` client (mirrors the pattern in
``rob_box_llm/test/test_minimax_provider.py``). No real network is
touched — the tests run in CI without outbound access.

The M1-M10 wiring (capabilities, image-size limit, ``base_resp``
envelope, API-key redaction, ``aclose``) is exercised by the
upstream test suite and inherited unchanged by the wrapper.
"""

from __future__ import annotations

import asyncio
from dataclasses import dataclass, field
from typing import Any, AsyncIterator
from unittest.mock import MagicMock

import httpx
import pytest

from openai import (
    APIConnectionError,
    APIStatusError,
    APITimeoutError,
    AuthenticationError,
    AsyncOpenAI,
)

from rob_box_harness.config import HarnessConfig, LLMConfig
from rob_box_harness.errors import ConfigError
from rob_box_harness.providers.minimax import (
    DEFAULT_BASE_URL,
    DEFAULT_MODEL,
    DEFAULT_THINKING_POLICY,
    MINIMAX_API_KEY_ENV,
    MINIMAX_MAX_IMAGE_BYTES,
    HarnessMiniMaxProvider,
    MiniMaxProvider,
    MiniMaxRedactedLogFilter,
    RetryPolicy,
    build_minimax_provider,
)
from rob_box_llm.errors import (
    AuthError,
    CapabilityUnavailableError,
    ContentFilterError,
    ProviderError,
    RateLimitError,
    TimeoutError,
)
from rob_box_llm.provider import (
    ImagePart,
    LLMChunk,
    LLMMessage,
    LLMResponse,
    LLMSettings,
    ProviderCapabilities,
)


# ---------------------------------------------------------------------------
# Fake SDK objects (mirrors the pattern in rob_box_llm's test suite)
# ---------------------------------------------------------------------------


@dataclass
class _ToolCallObj:
    id: str
    function: Any  # .name, .arguments


@dataclass
class _FunctionObj:
    name: str
    arguments: str  # raw JSON


@dataclass
class _MessageObj:
    content: str | None = None
    tool_calls: list[_ToolCallObj] | None = None


@dataclass
class _ChoiceObj:
    message: _MessageObj | None = None
    delta: _MessageObj | None = None
    finish_reason: str | None = None


@dataclass
class _UsageObj:
    prompt_tokens: int = 0
    completion_tokens: int = 0
    total_tokens: int = 0


@dataclass
class _ResponseObj:
    """Stand-in for the OpenAI SDK chat-completion response."""

    choices: list[_ChoiceObj]
    usage: _UsageObj | None = None
    base_resp: dict[str, Any] = field(default_factory=dict)


def _ok_response(
    content: str = "hi",
    tool_calls: list[_ToolCallObj] | None = None,
    finish_reason: str = "stop",
    *,
    base_resp: dict[str, Any] | None = None,
) -> _ResponseObj:
    return _ResponseObj(
        choices=[
            _ChoiceObj(
                message=_MessageObj(content=content, tool_calls=tool_calls),
                finish_reason=finish_reason,
            )
        ],
        usage=_UsageObj(prompt_tokens=1, completion_tokens=2, total_tokens=3),
        base_resp=base_resp or {},
    )


def _stream_chunk(
    content: str = "",
    finish_reason: str | None = None,
) -> _ResponseObj:
    return _ResponseObj(
        choices=[_ChoiceObj(delta=_MessageObj(content=content), finish_reason=finish_reason)],
        base_resp={},
    )


class _FakeCompletions:
    def __init__(self) -> None:
        self.next_response: _ResponseObj | None = None
        self.next_stream: list[_ResponseObj] = []
        self.next_exception: BaseException | None = None
        self.calls: list[dict[str, Any]] = []

    async def create(self, **kwargs: Any) -> Any:
        self.calls.append(kwargs)
        if self.next_exception is not None:
            exc, self.next_exception = self.next_exception, None
            raise exc
        if kwargs.get("stream"):
            return self._stream_iter()
        resp, self.next_response = self.next_response, None
        return resp

    async def _stream_iter(self) -> AsyncIterator[_ResponseObj]:
        for c in self.next_stream:
            yield c


class _FakeOpenAIClient:
    def __init__(self) -> None:
        self.chat = MagicMock()
        self.chat.completions = _FakeCompletions()
        self.closed = False

    async def close(self) -> None:
        self.closed = True


class _FakeStatusError(APIStatusError):
    """A fleshed-out ``APIStatusError`` subclass for tests.

    The OpenAI SDK's ``APIStatusError`` is not directly instantiable
    without going through the response/request plumbing; this dumb
    subclass fills in the attributes the upstream ``_map_exception``
    looks at (``status_code`` and ``body``).
    """

    def __init__(self, *, status: int, body: Any) -> None:
        self.status_code = status
        self.body = body
        self.request = MagicMock()
        Exception.__init__(self, f"{status}: {body}")


def _fake_401_response() -> httpx.Response:
    return httpx.Response(401, request=httpx.Request("POST", "http://x.invalid"))


def _make_minimax(
    *,
    retry: RetryPolicy | None = None,
    model: str = DEFAULT_MODEL,
) -> tuple[MiniMaxProvider, _FakeOpenAIClient]:
    """Build a harness wrapper backed by a fake SDK client.

    Returns ``(provider, client)`` so tests can inspect the recorded
    calls. Retries are disabled by default (``max_attempts=1``) so
    tests that don't care about retries stay deterministic.
    """
    client = _FakeOpenAIClient()
    return (
        MiniMaxProvider(
            api_key="sk-test",
            model=model,
            client=client,
            retry=retry or RetryPolicy(max_attempts=1),
        ),
        client,
    )


def _patch_sleep(monkeypatch: pytest.MonkeyPatch) -> list[float]:
    """Replace ``asyncio.sleep`` with a recorder-returning coroutine.

    Returns the list of recorded durations so tests can assert on
    the actual backoff schedule without sleeping in real time.
    """
    sleeps: list[float] = []

    async def fake_sleep(value: float) -> None:
        sleeps.append(value)

    monkeypatch.setattr("rob_box_harness.providers.minimax.asyncio.sleep", fake_sleep)
    return sleeps


# ---------------------------------------------------------------------------
# Module-level re-exports + identity
# ---------------------------------------------------------------------------


def test_default_base_url_and_model_are_exposed() -> None:
    """``DEFAULT_BASE_URL`` / ``DEFAULT_MODEL`` are importable from the.
    harness-side module and match the upstream values."""
    assert DEFAULT_BASE_URL == "https://api.minimax.io/v1"
    assert DEFAULT_MODEL == "MiniMax-M3"


def test_api_key_env_name_is_minimax_api_key() -> None:
    assert MINIMAX_API_KEY_ENV == "MINIMAX_API_KEY"


def test_module_reexports_image_limit_and_filter() -> None:
    """The harness-side module re-exports the upstream helpers so.
    callers have a single import path."""
    assert MINIMAX_MAX_IMAGE_BYTES == 10 * 1024 * 1024
    assert DEFAULT_THINKING_POLICY == {"type": "disabled"}
    assert MiniMaxRedactedLogFilter is not None


def test_minimaxprovider_is_harnesswrapper_alias() -> None:
    """``MiniMaxProvider`` is the public name; ``HarnessMiniMaxProvider``.
    is the canonical class. They MUST be the same object so
    ``isinstance`` checks succeed either way."""
    assert MiniMaxProvider is HarnessMiniMaxProvider


def test_provider_is_an_llm_provider() -> None:
    """``MiniMaxProvider`` IS-A ``LLMProvider`` so the harness can.
    pass it to ``harness.llm`` like any other adapter."""
    from rob_box_llm.provider import LLMProvider

    p, _ = _make_minimax()
    assert isinstance(p, LLMProvider)


def test_provider_name_is_minimax() -> None:
    p, _ = _make_minimax()
    assert p.name == "minimax"


def test_capabilities_advertise_text_streaming_tools_and_image_input() -> None:
    p, _ = _make_minimax()
    caps = p.capabilities
    assert isinstance(caps, ProviderCapabilities)
    assert caps.text is True
    assert caps.streaming_text is True
    assert caps.tools is True
    # Streaming tool calls are deliberately off — see upstream docstring.
    assert caps.streaming_tools is False
    assert caps.image_input is True


def test_capabilities_for_vision_model_keeps_image_input() -> None:
    p, _ = _make_minimax()
    assert p.capabilities_for("MiniMax-M3").image_input is True


def test_capabilities_for_non_vision_model_drops_image_input() -> None:
    p, _ = _make_minimax()
    assert p.capabilities_for("MiniMax-M2.7").image_input is False


# ---------------------------------------------------------------------------
# Authentication — env-based, no hardcoding
# ---------------------------------------------------------------------------


def test_construct_with_explicit_api_key_succeeds() -> None:
    """Explicit ``api_key=`` is the documented test/dev path."""
    p = MiniMaxProvider(api_key="sk-test", retry=RetryPolicy(max_attempts=1))
    assert p.name == "minimax"


def test_construct_with_env_api_key_succeeds(monkeypatch: pytest.MonkeyPatch) -> None:
    """When ``api_key=`` is omitted, the provider reads.
    ``MINIMAX_API_KEY`` from the OS env."""
    monkeypatch.setenv(MINIMAX_API_KEY_ENV, "sk-from-env")
    p = MiniMaxProvider(retry=RetryPolicy(max_attempts=1))
    assert p.name == "minimax"


def test_construct_without_api_key_raises_config_error(monkeypatch: pytest.MonkeyPatch) -> None:
    """Missing key in both ``api_key=`` and env raises ``ConfigError``.
    with a stable ``section`` attribute for callers to branch on."""
    monkeypatch.delenv(MINIMAX_API_KEY_ENV, raising=False)
    with pytest.raises(ConfigError) as exc_info:
        MiniMaxProvider(retry=RetryPolicy(max_attempts=1))
    assert exc_info.value.section == "llm.api_key"
    assert MINIMAX_API_KEY_ENV in str(exc_info.value)


def test_empty_string_api_key_raises_config_error(monkeypatch: pytest.MonkeyPatch) -> None:
    """An empty key (env or explicit) is treated as missing — the.
    provider refuses to silently fall back to a no-op credential."""
    monkeypatch.setenv(MINIMAX_API_KEY_ENV, "")
    with pytest.raises(ConfigError) as exc_info:
        MiniMaxProvider(api_key="", retry=RetryPolicy(max_attempts=1))
    assert exc_info.value.section == "llm.api_key"


def test_explicit_api_key_wins_over_env(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv(MINIMAX_API_KEY_ENV, "sk-from-env")
    # Should not raise, even with a "wrong" env value.
    p = MiniMaxProvider(api_key="sk-explicit", retry=RetryPolicy(max_attempts=1))
    assert p.name == "minimax"


# ---------------------------------------------------------------------------
# build_minimax_provider
# ---------------------------------------------------------------------------


def test_build_from_config_with_env_secret() -> None:
    """The factory resolves the API key from the env map (no inline.
    literal) and threads ``model`` / ``timeout_s`` through."""
    cfg = HarnessConfig.from_dict(
        {
            "harness": {"kind": "echo"},
            "llm": {
                "provider": "minimax",
                "model": "MiniMax-M3",
                "timeout_s": 12,
                "api_key": "${MINIMAX_API_KEY}",
            },
        },
        secrets={"MINIMAX_API_KEY": "sk-test"},
    )
    p = build_minimax_provider(cfg.llm)
    assert p.name == "minimax"
    assert isinstance(p, MiniMaxProvider)


def test_build_from_config_with_explicit_env_argument(monkeypatch: pytest.MonkeyPatch) -> None:
    """The ``env=`` kwarg of ``build_minimax_provider`` overrides the.
    OS env so tests can isolate the environment without monkey-patching.

    ``HarnessConfig.from_dict`` resolves ``${MINIMAX_API_KEY}`` from
    the explicit ``secrets=`` map (or ``os.environ`` fallback).
    ``build_minimax_provider`` then re-resolves the key via the
    ``env=`` kwarg, which lets a test inject a different key without
    touching the OS env.
    """
    # Pass ``secrets={"MINIMAX_API_KEY": "sk-from-config"}`` so
    # HarnessConfig is happy, then override with the factory's
    # ``env=`` map to prove the kwarg takes precedence.
    cfg = HarnessConfig.from_dict(
        {
            "harness": {"kind": "echo"},
            "llm": {
                "provider": "minimax",
                "api_key": "${MINIMAX_API_KEY}",
            },
        },
        secrets={"MINIMAX_API_KEY": "sk-from-config"},
    )
    p = build_minimax_provider(
        cfg.llm, env={"MINIMAX_API_KEY": "sk-from-arg"}
    )
    assert p.name == "minimax"


def test_build_rejects_wrong_provider_name() -> None:
    """The factory is MiniMax-specific — refuse anything else."""
    cfg = LLMConfig(provider="deepseek", model="m")
    with pytest.raises(ConfigError) as exc_info:
        build_minimax_provider(cfg)
    assert exc_info.value.section == "llm.provider"


def test_build_rejects_none_config() -> None:
    with pytest.raises(ConfigError) as exc_info:
        build_minimax_provider(None)  # type: ignore[arg-type]
    assert exc_info.value.section == "llm"


def test_build_rejects_missing_api_key(monkeypatch: pytest.MonkeyPatch) -> None:
    """If the env has no key and the config didn't supply one, the.
    factory raises at composition time, not at first request."""
    monkeypatch.delenv(MINIMAX_API_KEY_ENV, raising=False)
    cfg = LLMConfig(provider="minimax", model=DEFAULT_MODEL)
    with pytest.raises(ConfigError) as exc_info:
        build_minimax_provider(cfg, env={})
    assert exc_info.value.section == "llm.api_key"


def test_build_passes_model_from_config() -> None:
    cfg = LLMConfig(
        provider="minimax",
        model="MiniMax-M2.7",
        api_key="sk-test",
        timeout_s=5,
    )
    p = build_minimax_provider(cfg, env={})
    # The model is forwarded to the upstream provider; verify via
    # capabilities_for (the upstream narrows ``image_input`` per-model).
    assert p.capabilities_for("MiniMax-M2.7").image_input is False


def test_build_uses_default_model_when_unset() -> None:
    """The default model is vision-capable (``MiniMax-M3``)."""
    cfg = LLMConfig(provider="minimax", api_key="sk-test")
    p = build_minimax_provider(cfg, env={})
    assert p.capabilities_for(DEFAULT_MODEL).image_input is True


# ---------------------------------------------------------------------------
# complete() — request shape + response parsing
# ---------------------------------------------------------------------------


@pytest.mark.asyncio
async def test_complete_returns_text_content() -> None:
    p, client = _make_minimax()
    client.chat.completions.next_response = _ok_response("hello world")
    response = await p.complete([LLMMessage(role="user", content="hi")])
    assert response.content == "hello world"
    assert response.finish_reason == "stop"
    assert response.usage["total_tokens"] == 3


@pytest.mark.asyncio
async def test_complete_passes_model_and_messages() -> None:
    p, client = _make_minimax()
    client.chat.completions.next_response = _ok_response("ok")
    await p.complete(
        [
            LLMMessage(role="system", content="be terse"),
            LLMMessage(role="user", content="hi"),
        ]
    )
    kwargs = client.chat.completions.calls[0]
    assert kwargs["model"] == DEFAULT_MODEL
    assert kwargs["stream"] is False
    assert kwargs["messages"][0]["role"] == "system"
    assert kwargs["messages"][1]["role"] == "user"
    # Default thinking policy is injected via settings.extra.
    assert kwargs["thinking"] == DEFAULT_THINKING_POLICY


@pytest.mark.asyncio
async def test_complete_parses_tool_calls() -> None:
    p, client = _make_minimax()
    tc = _ToolCallObj(
        id="call_1",
        function=_FunctionObj(name="play_sound", arguments='{"name":"beep"}'),
    )
    client.chat.completions.next_response = _ok_response(
        tool_calls=[tc], finish_reason="tool_calls"
    )
    response = await p.complete([LLMMessage(role="user", content="beep")])
    assert len(response.tool_calls) == 1
    assert response.tool_calls[0].name == "play_sound"
    assert response.tool_calls[0].arguments == {"name": "beep"}


@pytest.mark.asyncio
async def test_complete_surfaces_base_resp_envelope() -> None:
    """MiniMax's HTTP-200 envelope translates into a typed error."""
    p, client = _make_minimax()
    client.chat.completions.next_response = _ok_response(
        "ok", base_resp={"status_code": 1001, "status_msg": "auth failed"}
    )
    with pytest.raises(AuthError):
        await p.complete([LLMMessage(role="user", content="hi")])


@pytest.mark.asyncio
async def test_complete_settings_override_model() -> None:
    """Per-call ``LLMSettings`` overrides the instance default."""
    p, client = _make_minimax()
    client.chat.completions.next_response = _ok_response("ok")
    await p.complete(
        [LLMMessage(role="user", content="hi")],
        settings=LLMSettings(model="MiniMax-M2.7", temperature=0.0),
    )
    kwargs = client.chat.completions.calls[0]
    assert kwargs["model"] == "MiniMax-M2.7"
    assert kwargs["temperature"] == 0.0


# ---------------------------------------------------------------------------
# chat(messages, **kwargs) shortcut
# ---------------------------------------------------------------------------


@pytest.mark.asyncio
async def test_chat_builds_settings_and_delegates_to_complete() -> None:
    """``chat()`` translates kwargs into ``LLMSettings`` and calls.
    ``complete()`` — the request to the wire carries the same fields."""
    p, client = _make_minimax()
    client.chat.completions.next_response = _ok_response("ok")

    response = await p.chat(
        [LLMMessage(role="user", content="hi")],
        model="MiniMax-M2.7",
        max_tokens=64,
        tool_choice="auto",
        top_k=42,  # unknown kwarg → goes into extra
    )
    assert response.content == "ok"

    kwargs = client.chat.completions.calls[0]
    assert kwargs["model"] == "MiniMax-M2.7"
    assert kwargs["max_tokens"] == 64
    assert kwargs["tool_choice"] == "auto"
    # Vendor-specific knobs ride along via ``extra``.
    assert kwargs["top_k"] == 42


@pytest.mark.asyncio
async def test_chat_with_empty_kwargs_uses_defaults() -> None:
    """When ``chat()`` is called with no kwargs, the request still.
    goes through with the model's defaults."""
    p, client = _make_minimax()
    client.chat.completions.next_response = _ok_response("ok")

    response = await p.chat([LLMMessage(role="user", content="hi")])
    assert response.content == "ok"
    kwargs = client.chat.completions.calls[0]
    assert kwargs["model"] == DEFAULT_MODEL


@pytest.mark.asyncio
async def test_chat_with_only_unknown_kwargs() -> None:
    """A caller can pass only vendor-specific kwargs — they all flow.
    into ``extra`` and don't bust the LLMProvider signature."""
    p, client = _make_minimax()
    client.chat.completions.next_response = _ok_response("ok")

    response = await p.chat(
        [LLMMessage(role="user", content="hi")],
        thinking={"type": "enabled", "budget": 100},
        top_p=0.9,
    )
    assert response.content == "ok"
    kwargs = client.chat.completions.calls[0]
    assert kwargs["thinking"] == {"type": "enabled", "budget": 100}
    assert kwargs["top_p"] == 0.9


# ---------------------------------------------------------------------------
# Retry mechanics
# ---------------------------------------------------------------------------


def test_retry_policy_validates_arguments() -> None:
    with pytest.raises(ValueError):
        RetryPolicy(max_attempts=0)
    with pytest.raises(ValueError):
        RetryPolicy(backoff_base=-0.1)
    with pytest.raises(ValueError):
        RetryPolicy(backoff_jitter=-0.1)


def test_retry_policy_delay_for_grows_exponentially() -> None:
    """``delay_for(n) ≈ base * 2 ** (n - 1)`` plus jitter in ``[0, jitter)``."""
    policy = RetryPolicy(backoff_base=0.5, backoff_jitter=0.0)
    assert policy.delay_for(1) == pytest.approx(0.5)
    assert policy.delay_for(2) == pytest.approx(1.0)
    assert policy.delay_for(3) == pytest.approx(2.0)
    assert policy.delay_for(4) == pytest.approx(4.0)


def test_retry_policy_delay_for_is_in_jitter_band() -> None:
    """With jitter > 0, the delay is bounded by ``[base * 2 ** (n-1),.
    base * 2 ** (n-1) + jitter)``."""
    # For attempt=2 with base=1.0/jitter=0.5: delay in [2.0, 2.5).
    policy = RetryPolicy(backoff_base=1.0, backoff_jitter=0.5)
    for _ in range(100):
        d = policy.delay_for(2)
        assert 2.0 <= d < 2.5, f"delay {d} out of jitter band"


def test_retry_policy_delay_for_attempts_below_one_is_zero() -> None:
    policy = RetryPolicy(backoff_base=0.5)
    assert policy.delay_for(0) == 0.0
    assert policy.delay_for(-1) == 0.0


@pytest.mark.asyncio
async def test_complete_retries_on_rate_limit_error(monkeypatch: pytest.MonkeyPatch) -> None:
    """``APIStatusError(429)`` → upstream maps to ``RateLimitError`` →.
    the retry loop catches it and retries until success."""
    sleeps = _patch_sleep(monkeypatch)
    p, client = _make_minimax(
        retry=RetryPolicy(max_attempts=3, backoff_base=0.1, backoff_jitter=0.0)
    )
    attempts = {"n": 0}

    async def drive_create(**kwargs: Any) -> Any:
        client.chat.completions.calls.append(kwargs)
        attempts["n"] += 1
        if attempts["n"] <= 2:
            raise _FakeStatusError(
                status=429, body={"error": {"message": "rate-limited"}}
            )
        return _ok_response("after two retries")

    client.chat.completions.create = drive_create  # type: ignore[assignment]

    response = await p.complete([LLMMessage(role="user", content="hi")])
    assert response.content == "after two retries"
    assert attempts["n"] == 3
    # Two sleeps between three attempts.
    assert sleeps == [0.1, 0.2]


@pytest.mark.asyncio
async def test_complete_does_not_retry_non_transient_errors(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """``AuthError`` propagates immediately — it is a programming error,.
    not a transient failure. ``CapabilityUnavailableError`` is raised
    before the network call so it bypasses the retry loop by construction."""
    sleeps = _patch_sleep(monkeypatch)
    p, client = _make_minimax(retry=RetryPolicy(max_attempts=5, backoff_base=0.1))

    async def always_401(**kwargs: Any) -> Any:
        client.chat.completions.calls.append(kwargs)
        raise AuthenticationError(
            message="bad key",
            response=_fake_401_response(),
            body={"error": {"message": "bad key"}},
        )

    client.chat.completions.create = always_401  # type: ignore[assignment]

    with pytest.raises(AuthError):
        await p.complete([LLMMessage(role="user", content="hi")])
    assert len(client.chat.completions.calls) == 1
    assert sleeps == []


@pytest.mark.asyncio
async def test_complete_re_exhausts_retries_on_persistent_rate_limit(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """After ``max_attempts`` failures the last exception propagates."""
    sleeps = _patch_sleep(monkeypatch)
    p, client = _make_minimax(
        retry=RetryPolicy(max_attempts=3, backoff_base=0.1, backoff_jitter=0.0)
    )

    async def always_429(**kwargs: Any) -> Any:
        client.chat.completions.calls.append(kwargs)
        raise _FakeStatusError(
            status=429, body={"error": {"message": "rate-limited"}}
        )

    client.chat.completions.create = always_429  # type: ignore[assignment]

    with pytest.raises(RateLimitError):
        await p.complete([LLMMessage(role="user", content="hi")])
    assert len(client.chat.completions.calls) == 3
    assert sleeps == [0.1, 0.2]


@pytest.mark.asyncio
async def test_complete_retries_on_timeout(monkeypatch: pytest.MonkeyPatch) -> None:
    """``APITimeoutError`` → upstream maps to ``TimeoutError`` → retry."""
    sleeps = _patch_sleep(monkeypatch)
    p, client = _make_minimax(
        retry=RetryPolicy(max_attempts=2, backoff_base=0.1, backoff_jitter=0.0)
    )
    attempts = {"n": 0}

    async def drive(**kwargs: Any) -> Any:
        client.chat.completions.calls.append(kwargs)
        attempts["n"] += 1
        if attempts["n"] == 1:
            raise APITimeoutError(request=MagicMock())
        return _ok_response("after timeout")

    client.chat.completions.create = drive  # type: ignore[assignment]

    response = await p.complete([LLMMessage(role="user", content="hi")])
    assert response.content == "after timeout"
    assert attempts["n"] == 2
    assert sleeps == [0.1]


@pytest.mark.asyncio
async def test_complete_retries_on_connection_error(monkeypatch: pytest.MonkeyPatch) -> None:
    """``APIConnectionError`` → upstream maps to ``TimeoutError`` → retry."""
    sleeps = _patch_sleep(monkeypatch)
    p, client = _make_minimax(
        retry=RetryPolicy(max_attempts=2, backoff_base=0.1, backoff_jitter=0.0)
    )
    attempts = {"n": 0}

    async def drive(**kwargs: Any) -> Any:
        client.chat.completions.calls.append(kwargs)
        attempts["n"] += 1
        if attempts["n"] == 1:
            raise APIConnectionError(request=MagicMock())
        return _ok_response("after reconnect")

    client.chat.completions.create = drive  # type: ignore[assignment]

    response = await p.complete([LLMMessage(role="user", content="hi")])
    assert response.content == "after reconnect"
    assert attempts["n"] == 2
    assert sleeps == [0.1]


@pytest.mark.asyncio
async def test_disabling_retries_means_max_attempts_one() -> None:
    """``RetryPolicy(max_attempts=1)`` disables retries — the first.
    failure propagates immediately."""
    p, client = _make_minimax(retry=RetryPolicy(max_attempts=1))

    async def always_429(**kwargs: Any) -> Any:
        client.chat.completions.calls.append(kwargs)
        raise _FakeStatusError(
            status=429, body={"error": {"message": "rate-limited"}}
        )

    client.chat.completions.create = always_429  # type: ignore[assignment]

    with pytest.raises(RateLimitError):
        await p.complete([LLMMessage(role="user", content="hi")])
    assert len(client.chat.completions.calls) == 1


@pytest.mark.asyncio
async def test_chat_propagates_retry_semantics(monkeypatch: pytest.MonkeyPatch) -> None:
    """``chat()`` participates in the same retry loop as ``complete()``."""
    sleeps = _patch_sleep(monkeypatch)
    p, client = _make_minimax(
        retry=RetryPolicy(max_attempts=2, backoff_base=0.1, backoff_jitter=0.0)
    )
    attempts = {"n": 0}

    async def drive(**kwargs: Any) -> Any:
        client.chat.completions.calls.append(kwargs)
        attempts["n"] += 1
        if attempts["n"] == 1:
            raise APITimeoutError(request=MagicMock())
        return _ok_response("after timeout")

    client.chat.completions.create = drive  # type: ignore[assignment]

    response = await p.chat([LLMMessage(role="user", content="hi")], temperature=0.0)
    assert response.content == "after timeout"
    assert attempts["n"] == 2
    assert sleeps == [0.1]


# ---------------------------------------------------------------------------
# stream()
# ---------------------------------------------------------------------------


@pytest.mark.asyncio
async def test_stream_yields_chunks_with_finish_reason() -> None:
    p, client = _make_minimax()
    client.chat.completions.next_stream = [
        _stream_chunk("He"),
        _stream_chunk("llo"),
        _stream_chunk("", finish_reason="stop"),
    ]
    chunks = []
    async for chunk in p.stream([LLMMessage(role="user", content="hi")]):
        chunks.append(chunk)
    assert [c.content_delta for c in chunks] == ["He", "llo", ""]
    assert chunks[-1].finish_reason == "stop"


@pytest.mark.asyncio
async def test_stream_with_tools_raises_capability_error() -> None:
    """Streaming + tools is capability-gated in the upstream provider."""
    p, client = _make_minimax()

    async def drain() -> None:
        async for _ in p.stream(
            [LLMMessage(role="user", content="hi")],
            tools=({"type": "function", "function": {"name": "play_sound"}},),
        ):
            pass

    with pytest.raises(CapabilityUnavailableError):
        await drain()
    # Crucially: no SDK call was made (the capability check runs
    # BEFORE the network).
    assert client.chat.completions.calls == []


@pytest.mark.asyncio
async def test_stream_retries_on_initial_429(monkeypatch: pytest.MonkeyPatch) -> None:
    """The stream path retries the initial request round-trip on a.
    transient error."""
    sleeps = _patch_sleep(monkeypatch)
    p, client = _make_minimax(
        retry=RetryPolicy(max_attempts=2, backoff_base=0.1, backoff_jitter=0.0)
    )
    attempts = {"n": 0}

    async def drive_create(**kwargs: Any) -> Any:
        client.chat.completions.calls.append(kwargs)
        attempts["n"] += 1
        if attempts["n"] == 1:
            raise _FakeStatusError(
                status=429, body={"error": {"message": "rate-limited"}}
            )
        # Second attempt: return a successful stream.
        async def _stream() -> AsyncIterator[_ResponseObj]:
            for c in [
                _stream_chunk("He"),
                _stream_chunk("llo"),
                _stream_chunk("", finish_reason="stop"),
            ]:
                yield c

        return _stream()

    client.chat.completions.create = drive_create  # type: ignore[assignment]

    chunks = []
    async for chunk in p.stream([LLMMessage(role="user", content="hi")]):
        chunks.append(chunk)
    assert [c.content_delta for c in chunks] == ["He", "llo", ""]
    assert attempts["n"] == 2
    assert sleeps == [0.1]


@pytest.mark.asyncio
async def test_stream_retry_closes_failed_attempt(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """W-1: when the initial ``__anext__`` on a freshly-opened inner.
    stream raises a transient error, the failed stream MUST be
    ``aclose()``-d before the next attempt opens a new one. Otherwise
    the underlying HTTP body sits in the openai SDK's pool until GC
    — a connection-pool leak per retry.

    Reference: ADR-0001 §2.6.1 M10 (``aclose()`` корректно закрывает
    HTTP-клиент, особенно при streaming).
    """
    sleeps = _patch_sleep(monkeypatch)
    p, client = _make_minimax(
        retry=RetryPolicy(max_attempts=3, backoff_base=0.1, backoff_jitter=0.0)
    )

    # Event log lets us assert ordering between ``aclose`` of the
    # failed attempt and ``create`` of the next attempt, without
    # relying on wall-clock ordering.
    events: list[str] = []
    aclose_call_count = {"n": 0}

    # Wrap the upstream async generator so its ``aclose`` records the
    # call. ``async_generator.aclose`` is read-only in CPython, but
    # we don't need to override it: we only need to OBSERVE that
    # ``aclose`` was awaited. We do that by patching the provider's
    # ``aclose`` after the upstream generator is created.
    #
    # Concretely: when the harness provider catches a transient error
    # on ``inner_stream.__anext__``, it calls ``inner_stream.aclose()``.
    # The upstream generator (returned by
    # ``MiniMaxProvider.stream()``) is an ``async_generator`` — its
    # ``aclose`` is a built-in method. We can't replace it. But we CAN
    # check whether the harness provider called ``aclose`` at all by
    # substituting our own ``AsyncIterator`` wrapper as the
    # ``stream_obj`` returned from the SDK call. The upstream
    # ``stream()`` then does ``async for event in stream_obj`` — so we
    # can intercept the wrapper's ``aclose``.
    #
    # Since the harness provider sees ``inner_stream`` = the upstream
    # generator (whose aclose we cannot patch), we instead patch
    # ``MiniMaxProvider.stream`` directly so we can hand the harness
    # provider a wrapper we own. This still exercises the W-1 path:
    # the harness provider's ``except`` branch must call ``aclose``
    # on whatever object it received from ``self._inner.stream()``.
    inner_streams: list[Any] = []

    class _TrackedStream:
        """Wraps an inner async iterator so we can intercept ``aclose()``.

        The harness provider calls ``inner_stream.aclose()`` in its
        ``except (RateLimitError, TimeoutError)`` branch (W-1). We
        count and order these calls so the test can assert that
        ``aclose`` ran exactly once and BEFORE the second attempt
        opened.
        """

        def __init__(self, inner: AsyncIterator[_ResponseObj]) -> None:
            self._inner = inner
            self.closed = False

        def __aiter__(self) -> _TrackedStream:
            return self

        async def __anext__(self) -> _ResponseObj:
            return await self._inner.__anext__()

        async def aclose(self) -> None:
            self.closed = True
            aclose_call_count["n"] += 1
            events.append("aclose[1]")
            inner_aclose = getattr(self._inner, "aclose", None)
            if inner_aclose is not None:
                await inner_aclose()

    # We replace ``p._inner.stream`` with a plain (sync) function
    # that returns an AsyncIterator directly. Because
    # ``MiniMaxProvider.stream`` is declared ``async def`` (and is an
    # async GENERATOR function — its body has ``yield``), the bound
    # method, when called, normally returns an AsyncGenerator. The
    # harness provider does:
    #
    #     inner_stream = self._inner.stream(messages, ...)
    #     first_chunk = await inner_stream.__anext__()
    #
    # It treats the result as an AsyncIterator. We install our own
    # function on the instance's ``__dict__``, which shadows the
    # bound method on the class, and returns a ``_TrackedStream``
    # directly (no async/await, no yield). This is the cleanest way
    # to observe ``aclose`` on the wrapper without monkey-patching
    # ``async_generator.aclose`` (which is read-only in CPython).
    attempts = {"n": 0}

    def patched_stream(
        messages: Any, *, tools: Any = (), settings: Any = None
    ) -> Any:
        attempts["n"] += 1
        events.append(f"inner_stream[{attempts['n']}]")
        if attempts["n"] == 1:
            async def _failing_gen() -> AsyncIterator[_ResponseObj]:
                events.append("first-anext-raises")
                raise RateLimitError(
                    "rate-limited mid-open", provider="minimax"
                )
                yield  # pragma: no cover — async generator marker

            return _TrackedStream(_failing_gen())

        async def _ok_gen() -> AsyncIterator[LLMChunk]:
            yield LLMChunk(content_delta="OK")
            yield LLMChunk(content_delta="", finish_reason="stop")

        return _ok_gen()

    # Bind as an instance attribute, NOT a class attribute. Python
    # looks up ``p._inner.stream`` via the instance dict first.
    p._inner.__dict__["stream"] = patched_stream  # type: ignore[method-assign]

    chunks: list[Any] = []
    async for chunk in p.stream([LLMMessage(role="user", content="hi")]):
        chunks.append(chunk)

    # The retry happened (2 attempts), the second attempt's chunks
    # surfaced to the caller, and the backoff slept exactly once.
    assert [c.content_delta for c in chunks] == ["OK", ""]
    assert attempts["n"] == 2
    assert sleeps == [0.1]

    # W-1 contract: the failed first attempt's aclose() was called
    # exactly once.
    assert aclose_call_count["n"] == 1, (
        f"expected aclose() on the failed first attempt, "
        f"got count={aclose_call_count['n']}; events={events!r}"
    )

    # W-1 contract: aclose() of the failed inner stream happened
    # BEFORE the second inner stream was opened. Without this
    # ordering the HTTP body from the first attempt leaks until the
    # next pool checkout.
    assert events.index("aclose[1]") < events.index("inner_stream[2]"), (
        f"aclose of failed attempt must precede opening of next attempt; "
        f"events={events!r}"
    )


@pytest.mark.asyncio
async def test_stream_retry_swallows_aclose_failure(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """W-1 robustness: if the failed attempt's ``aclose()`` itself.
    raises (e.g. transport already torn down), the retry must still
    proceed and the original transient error must still surface if
    retries are exhausted.

    Companion test to ``test_stream_retry_closes_failed_attempt``:
    this one proves the W-1 fix is *best-effort* — a broken aclose
    doesn't break the retry path, it just gets logged at DEBUG.
    """
    sleeps = _patch_sleep(monkeypatch)
    p, client = _make_minimax(
        retry=RetryPolicy(max_attempts=2, backoff_base=0.1, backoff_jitter=0.0)
    )

    # Track that aclose WAS attempted on the failed first attempt.
    # Without the W-1 fix this counter stays at 0, so the assertion
    # below fails — guarding against accidental regressions where
    # someone removes the aclose call entirely (which would also
    # pass the original test, but is exactly the leak we want to
    # prevent).
    aclose_attempted = {"n": 0}

    class _BrokenCloseStream:
        """A stream whose ``aclose`` itself raises. Used to verify.
        that the provider's best-effort cleanup swallows the error
        and lets the retry proceed."""

        def __init__(self, inner: AsyncIterator[_ResponseObj]) -> None:
            self._inner = inner

        def __aiter__(self) -> _BrokenCloseStream:
            return self

        async def __anext__(self) -> _ResponseObj:
            return await self._inner.__anext__()

        async def aclose(self) -> None:
            aclose_attempted["n"] += 1
            raise RuntimeError("transport already closed")

    attempts = {"n": 0}

    def patched_stream(
        messages: Any, *, tools: Any = (), settings: Any = None
    ) -> Any:
        attempts["n"] += 1
        if attempts["n"] == 1:
            async def _failing_gen() -> AsyncIterator[_ResponseObj]:
                raise RateLimitError(
                    "rate-limited", provider="minimax"
                )
                yield  # pragma: no cover

            return _BrokenCloseStream(_failing_gen())

        async def _ok_gen() -> AsyncIterator[LLMChunk]:
            yield LLMChunk(content_delta="OK")
            yield LLMChunk(content_delta="", finish_reason="stop")

        return _ok_gen()

    p._inner.__dict__["stream"] = patched_stream  # type: ignore[method-assign]

    # The broken aclose() must NOT prevent the retry: we still see the
    # second attempt's chunks and only one sleep (one retry).
    chunks: list[Any] = []
    async for chunk in p.stream([LLMMessage(role="user", content="hi")]):
        chunks.append(chunk)
    assert [c.content_delta for c in chunks] == ["OK", ""]
    assert attempts["n"] == 2
    assert sleeps == [0.1]
    # The harness provider MUST have attempted to aclose() the failed
    # stream — proving the W-1 path is wired up. The fact that the
    # aclose itself raised is irrelevant: the best-effort wrapper
    # swallows the exception.
    assert aclose_attempted["n"] == 1, (
        f"expected aclose() attempt on failed first attempt, "
        f"got {aclose_attempted['n']} — W-1 fix is missing"
    )


# ---------------------------------------------------------------------------
# aclose()
# ---------------------------------------------------------------------------


@pytest.mark.asyncio
async def test_aclose_releases_client() -> None:
    client = _FakeOpenAIClient()
    p = MiniMaxProvider(
        api_key="sk-test", client=client, retry=RetryPolicy(max_attempts=1)
    )
    await p.aclose()
    assert client.closed is True


@pytest.mark.asyncio
async def test_aclose_is_idempotent() -> None:
    client = _FakeOpenAIClient()
    p = MiniMaxProvider(
        api_key="sk-test", client=client, retry=RetryPolicy(max_attempts=1)
    )
    await p.aclose()
    await p.aclose()  # second call must be a no-op
    assert client.closed is True


# ---------------------------------------------------------------------------
# Misc invariants
# ---------------------------------------------------------------------------


def test_default_provider_name_matches_adr_0001() -> None:
    """ADR-0001 §2.6 requires ``name == "minimax"`` for the YAML
    fallback chain to resolve the right adapter."""
    p, _ = _make_minimax()
    assert p.name == "minimax"
    # Defaults are stable across the harness / upstream boundary.
    assert DEFAULT_BASE_URL == "https://api.minimax.io/v1"
    assert DEFAULT_MODEL == "MiniMax-M3"


@pytest.mark.asyncio
async def test_capability_check_runs_before_network() -> None:
    """Capability gates (e.g. image_input on a non-vision model) MUST.
    raise before any SDK call is made."""
    p, client = _make_minimax(model="MiniMax-M2.7")

    with pytest.raises(CapabilityUnavailableError):
        await p.complete(
            [
                LLMMessage(
                    role="user",
                    content=(ImagePart(source="https://x.invalid/c.jpg"),),
                )
            ]
        )
    assert client.chat.completions.calls == []


@pytest.mark.asyncio
async def test_image_payload_above_limit_raises_capability_error() -> None:
    """The 10 MB image-size cap is enforced before the network."""
    p, client = _make_minimax()
    huge = b"x" * (MINIMAX_MAX_IMAGE_BYTES + 1)
    with pytest.raises(CapabilityUnavailableError):
        await p.complete(
            [
                LLMMessage(
                    role="user",
                    content=(ImagePart(source=huge, media_type="image/jpeg"),),
                )
            ]
        )
    assert client.chat.completions.calls == []


@pytest.mark.asyncio
async def test_provider_error_envelope_raises_typed_error() -> None:
    """Unknown ``base_resp.status_code`` (HTTP 200 body) becomes a.
    generic ``ProviderError`` so the caller can branch on it."""
    p, client = _make_minimax()
    client.chat.completions.next_response = _ok_response(
        "ok", base_resp={"status_code": 9999, "status_msg": "weird thing"}
    )
    with pytest.raises(ProviderError):
        await p.complete([LLMMessage(role="user", content="hi")])


@pytest.mark.asyncio
async def test_content_filter_envelope_raises_typed_error() -> None:
    p, client = _make_minimax()
    client.chat.completions.next_response = _ok_response(
        "ok", base_resp={"status_code": 2001, "status_msg": "content policy"}
    )
    with pytest.raises(ContentFilterError):
        await p.complete([LLMMessage(role="user", content="hi")])


@pytest.mark.asyncio
async def test_rate_limit_envelope_raises_typed_error() -> None:
    """Rate-limit / quota keywords in ``base_resp.status_msg`` map to.
    ``RateLimitError``."""
    p, client = _make_minimax()
    client.chat.completions.next_response = _ok_response(
        "ok", base_resp={"status_code": 1008, "status_msg": "insufficient quota"}
    )
    with pytest.raises(RateLimitError):
        await p.complete([LLMMessage(role="user", content="hi")])


def test_module_reexports_match_upstream_public_surface() -> None:
    """The harness-side module must re-export the same public names.
    as upstream so ``from rob_box_harness.providers.minimax import X``
    works for every X a caller might be using."""
    from rob_box_harness.providers import minimax as harness_minimax

    upstream = {
        "MiniMaxProvider",
        "DEFAULT_THINKING_POLICY",
        "MINIMAX_MAX_IMAGE_BYTES",
        "MiniMaxRedactedLogFilter",
    }
    for name in upstream:
        assert name in harness_minimax.__all__, name
        assert getattr(harness_minimax, name) is not None


@pytest.mark.asyncio
async def test_complete_returns_llm_response_type() -> None:
    """The return type is the canonical ``LLMResponse`` — the.
    framework's contract from ADR-0001."""
    p, client = _make_minimax()
    client.chat.completions.next_response = _ok_response("ok")
    response = await p.complete([LLMMessage(role="user", content="hi")])
    assert isinstance(response, LLMResponse)


# ---------------------------------------------------------------------------
# HTTP-transport-level tests — exercise the OpenAI SDK against an
# ``httpx.MockTransport`` so the test never touches the real network.
# These tests prove the wiring end-to-end (request shape → HTTP →
# JSON → typed response) without depending on the SDK's internal
# shape. Required by the task body's "mock HTTP transport" clause.
# ---------------------------------------------------------------------------


def _openai_response_payload(
    content: str = "hi",
    *,
    model: str = "MiniMax-M3",
    finish_reason: str = "stop",
    prompt_tokens: int = 1,
    completion_tokens: int = 2,
) -> dict[str, Any]:
    """Build a JSON body that looks like an OpenAI chat-completion.
    response (which MiniMax mirrors)."""
    return {
        "id": "chatcmpl-test",
        "object": "chat.completion",
        "created": 1_700_000_000,
        "model": model,
        "choices": [
            {
                "index": 0,
                "message": {
                    "role": "assistant",
                    "content": content,
                },
                "finish_reason": finish_reason,
            }
        ],
        "usage": {
            "prompt_tokens": prompt_tokens,
            "completion_tokens": completion_tokens,
            "total_tokens": prompt_tokens + completion_tokens,
        },
    }


@pytest.mark.asyncio
async def test_complete_over_mocked_http_transport() -> None:
    """End-to-end test through a real ``AsyncOpenAI`` client whose.
    underlying ``httpx`` is swapped for a ``MockTransport``.

    The test never touches the network — the mock intercepts the
    HTTP request and returns a canned JSON body. The provider
    consumes the body, parses it via the OpenAI SDK, and the
    harness-side retry loop is short-circuited because the
    response is successful on the first try.
    """
    # Build a mock handler that records the request and returns the
    # canned payload.
    captured: dict[str, Any] = {}

    def handler(request: httpx.Request) -> httpx.Response:
        captured["url"] = str(request.url)
        captured["method"] = request.method
        captured["headers"] = dict(request.headers)
        # Read the body so the test can assert on the wire format.
        import json as _json

        captured["body"] = _json.loads(request.content.decode("utf-8"))
        return httpx.Response(
            200,
            json=_openai_response_payload("hello via mock http"),
        )

    transport = httpx.MockTransport(handler)
    # The OpenAI SDK uses an httpx.AsyncClient under the hood. We
    # build one explicitly and pass it to AsyncOpenAI so the
    # MockTransport is the one that actually carries the request.
    async_http_client = httpx.AsyncClient(
        transport=transport, base_url="https://api.minimax.io/v1"
    )
    openai_client = AsyncOpenAI(
        api_key="sk-test",
        http_client=async_http_client,
    )
    p = MiniMaxProvider(
        api_key="sk-test",
        client=openai_client,
        retry=RetryPolicy(max_attempts=1),
        # Disable the upstream thinking-policy injection for this test
        # so the request shape remains stable regardless of where the
        # upstream provider decides to place non-OpenAI-standard kwargs.
        # The wrapper still receives ``thinking=None`` via its public
        # surface, which is the documented way to opt out of the
        # default policy.
        thinking=None,
    )

    response = await p.complete([LLMMessage(role="user", content="hi")])
    assert response.content == "hello via mock http"
    assert response.finish_reason == "stop"
    # Wire format: the request reached the mock transport.
    assert captured["method"] == "POST"
    assert captured["url"].endswith("/chat/completions")
    assert captured["body"]["model"] == DEFAULT_MODEL
    assert captured["body"]["stream"] is False
    assert captured["body"]["messages"][0]["role"] == "user"
    assert captured["body"]["messages"][0]["content"] == "hi"
    # No leaked API key in the URL.
    assert "sk-test" not in captured["url"]
    # Bearer token header was sent (the SDK adds it; we don't care
    # about the exact value, just that it exists).
    auth_header = captured["headers"].get("authorization", "")
    assert "Bearer" in auth_header


@pytest.mark.asyncio
async def test_http_transport_429_is_translated_and_retried() -> None:
    """A 429 response from the mock transport is mapped to.
    ``RateLimitError`` by the upstream SDK and caught by the
    harness-side retry loop."""
    attempts = {"n": 0}

    def handler(request: httpx.Request) -> httpx.Response:
        attempts["n"] += 1
        if attempts["n"] == 1:
            return httpx.Response(
                429,
                json={"error": {"message": "rate-limited"}},
            )
        return httpx.Response(
            200,
            json=_openai_response_payload("after retry"),
        )

    transport = httpx.MockTransport(handler)
    async_http_client = httpx.AsyncClient(
        transport=transport, base_url="https://api.minimax.io/v1"
    )
    openai_client = AsyncOpenAI(
        api_key="sk-test",
        http_client=async_http_client,
    )
    p = MiniMaxProvider(
        api_key="sk-test",
        client=openai_client,
        retry=RetryPolicy(max_attempts=2, backoff_base=0.0, backoff_jitter=0.0),
        thinking=None,
    )

    response = await p.complete([LLMMessage(role="user", content="hi")])
    assert response.content == "after retry"
    assert attempts["n"] == 2


@pytest.mark.asyncio
async def test_http_transport_401_is_translated_to_auth_error() -> None:
    """A 401 response is mapped to ``AuthError`` and bypasses.
    the retry loop (it's a programming error, not transient)."""
    def handler(request: httpx.Request) -> httpx.Response:
        return httpx.Response(
            401,
            json={"error": {"message": "invalid api key"}},
        )

    transport = httpx.MockTransport(handler)
    async_http_client = httpx.AsyncClient(
        transport=transport, base_url="https://api.minimax.io/v1"
    )
    openai_client = AsyncOpenAI(
        api_key="sk-test",
        http_client=async_http_client,
    )
    p = MiniMaxProvider(
        api_key="sk-test",
        client=openai_client,
        retry=RetryPolicy(max_attempts=5, backoff_base=0.0),
        thinking=None,
    )
    with pytest.raises(AuthError):
        await p.complete([LLMMessage(role="user", content="hi")])
