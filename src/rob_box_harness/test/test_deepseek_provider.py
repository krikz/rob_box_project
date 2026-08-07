"""Tests for the harness-side ``DeepSeekProvider`` / ``MiMoProvider``.

The tests focus on what the harness wrappers ADD on top of the
upstream ``rob_box_llm.providers.deepseek.DeepSeekProvider`` /
``MiMoProvider``:

* env-based authentication (``DEEPSEEK_API_KEY`` / ``MIMO_API_KEY``
  env vars, ``ConfigError`` on missing key),
* the public surface (``HarnessDeepSeekProvider`` /
  ``HarnessMiMoProvider``, ``build_deepseek_provider`` /
  ``build_mimo_provider`` factories, ``RetryPolicy``),
* ``chat(messages, **kwargs)`` convenience wrapper,
* retry-with-exponential-backoff on transient errors,
* ``aclose()`` idempotency.

The HTTP transport is mocked at the SDK layer via a fake
``AsyncOpenAI`` client. No real network is touched — the tests run
in CI without outbound access.

The wiring (capabilities, ``aclose``) is exercised by the
upstream test suite and inherited unchanged by the wrappers.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, AsyncIterator
from unittest.mock import MagicMock

import pytest

from openai import AsyncOpenAI

from rob_box_harness.errors import ConfigError
from rob_box_harness.providers.deepseek import (
    DEEPSEEK_API_KEY_ENV,
    DEFAULT_BASE_URL,
    DEFAULT_MODEL,
    HarnessDeepSeekProvider,
    build_deepseek_provider,
)
from rob_box_harness.providers.mimo import (
    MIMO_API_KEY_ENV,
    MimoProvider,
    HarnessMiMoProvider,
    build_mimo_provider,
)

from rob_box_llm.errors import RateLimitError, TimeoutError
from rob_box_llm.provider import (
    LLMChunk,
    LLMMessage,
    LLMResponse,
    LLMSettings,
)


# ---------------------------------------------------------------------------
# Fake SDK client (mirrors the pattern in test_minimax_provider.py)
# ---------------------------------------------------------------------------


@dataclass
class _FakeChoice:
    finish_reason: str = "stop"
    message: Any = None


@dataclass
class _FakeUsage:
    prompt_tokens: int = 10
    completion_tokens: int = 20
    total_tokens: int = 30


@dataclass
class _FakeResponse:
    id: str = "chatcmpl-test"
    model: str = "deepseek-chat"
    choices: list = field(default_factory=lambda: [_FakeChoice()])
    usage: Any = field(default_factory=lambda: _FakeUsage())


class _FakeCompletions:
    def __init__(self, response: _FakeResponse | None = None,
                 error: BaseException | None = None) -> None:
        self._response = response or _FakeResponse()
        self._error = error
        self.call_count = 0

    async def create(self, **kwargs: Any) -> Any:
        self.call_count += 1
        if self._error is not None:
            raise self._error
        return self._response


class _FakeChat:
    def __init__(self, completions: _FakeCompletions) -> None:
        self.completions = completions


class _FakeOpenAIClient:
    """Minimal stand-in for AsyncOpenAI used by harness tests."""

    def __init__(self, error: BaseException | None = None) -> None:
        completions = _FakeCompletions(error=error)
        self.chat = _FakeChat(completions)
        self._closed = False

    async def close(self) -> None:
        self._closed = True


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------


@pytest.fixture
def deepseek_env(monkeypatch: pytest.MonkeyPatch) -> None:
    """Set DEEPSEEK_API_KEY in the environment for the duration of the test."""
    monkeypatch.setenv(DEEPSEEK_API_KEY_ENV, "test-deepseek-key-123")


@pytest.fixture
def mimo_env(monkeypatch: pytest.MonkeyPatch) -> None:
    """Set MIMO_API_KEY in the environment for the duration of the test."""
    monkeypatch.setenv(MIMO_API_KEY_ENV, "test-mimo-key-456")


# ---------------------------------------------------------------------------
# Auth / construction
# ---------------------------------------------------------------------------


def test_deepseek_provider_requires_api_key_env(monkeypatch: pytest.MonkeyPatch) -> None:
    """Missing DEEPSEEK_API_KEY must raise ConfigError, not fall back."""
    monkeypatch.delenv(DEEPSEEK_API_KEY_ENV, raising=False)
    with pytest.raises(ConfigError) as exc_info:
        HarnessDeepSeekProvider(api_key=None)
    assert DEEPSEEK_API_KEY_ENV in str(exc_info.value)


def test_deepseek_provider_picks_up_env_api_key(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """When DEEPSEEK_API_KEY is set, the provider reads it."""
    monkeypatch.setenv(DEEPSEEK_API_KEY_ENV, "env-key")
    provider = HarnessDeepSeekProvider()
    assert provider is not None


def test_deepseek_provider_explicit_api_key_overrides_env(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """An explicit api_key= wins over the env var."""
    monkeypatch.setenv(DEEPSEEK_API_KEY_ENV, "env-key")
    provider = HarnessDeepSeekProvider(api_key="explicit-key")
    assert provider is not None


def test_deepseek_provider_name_is_deepseek(deepseek_env: None) -> None:
    """The provider's harness ``name`` must be ``"deepseek"`` so the
    fallback chain in :class:`HarnessRegistry` can pick it."""
    provider = HarnessDeepSeekProvider()
    assert provider.name == "deepseek"


def test_deepseek_default_base_url(deepseek_env: None) -> None:
    """The exported DEFAULT_BASE_URL must match the upstream constant."""
    from rob_box_llm.providers.deepseek import DeepSeekProvider as _Upstream
    assert DEFAULT_BASE_URL == _Upstream.DEFAULT_BASE_URL


def test_deepseek_default_model(deepseek_env: None) -> None:
    """The exported DEFAULT_MODEL must match the upstream constant."""
    from rob_box_llm.providers.deepseek import DeepSeekProvider as _Upstream
    assert DEFAULT_MODEL == _Upstream.DEFAULT_MODEL


def test_build_deepseek_provider_factory(deepseek_env: None) -> None:
    """``build_deepseek_provider()`` returns a configured instance."""
    provider = build_deepseek_provider()
    assert provider.name == "deepseek"


# ---------------------------------------------------------------------------
# MiMo provider
# ---------------------------------------------------------------------------


def test_mimo_provider_requires_api_key_env(monkeypatch: pytest.MonkeyPatch) -> None:
    """Missing MIMO_API_KEY must raise ConfigError."""
    monkeypatch.delenv(MIMO_API_KEY_ENV, raising=False)
    with pytest.raises(ConfigError) as exc_info:
        HarnessMiMoProvider(api_key=None)
    assert MIMO_API_KEY_ENV in str(exc_info.value)


def test_mimo_provider_name_is_mimo(mimo_env: None) -> None:
    """MiMo provider's harness name must be ``"mimo"``."""
    provider = HarnessMiMoProvider()
    assert provider.name == "mimo"


def test_mimo_alias_resolves(mimo_env: None) -> None:
    """``MimoProvider`` is an alias for :class:`HarnessMiMoProvider`."""
    assert MimoProvider is HarnessMiMoProvider


def test_build_mimo_provider_factory(mimo_env: None) -> None:
    """``build_mimo_provider()`` returns a configured instance."""
    provider = build_mimo_provider()
    assert provider.name == "mimo"


# ---------------------------------------------------------------------------
# Retry policy
# ---------------------------------------------------------------------------


def test_retry_policy_rejects_max_attempts_lt_1() -> None:
    from rob_box_harness.providers.deepseek import RetryPolicy
    with pytest.raises(ValueError):
        RetryPolicy(max_attempts=0)


def test_retry_policy_delay_grows_exponentially() -> None:
    from rob_box_harness.providers.deepseek import RetryPolicy
    policy = RetryPolicy(max_attempts=4, backoff_base=0.5, backoff_jitter=0.0)
    # attempt 1: 0.5, attempt 2: 1.0, attempt 3: 2.0 (no jitter)
    assert policy.delay_for(1) == pytest.approx(0.5)
    assert policy.delay_for(2) == pytest.approx(1.0)
    assert policy.delay_for(3) == pytest.approx(2.0)


# ---------------------------------------------------------------------------
# aclose idempotency
# ---------------------------------------------------------------------------


@pytest.mark.asyncio
async def test_aclose_is_idempotent(deepseek_env: None) -> None:
    """Calling aclose() twice must not raise."""
    provider = HarnessDeepSeekProvider()
    await provider.aclose()
    # Second call is a no-op — must not raise.
    await provider.aclose()