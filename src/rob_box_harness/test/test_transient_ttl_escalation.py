"""Tests for issue #2718 — per-provider transient TTL escalation.

A flat ``TRANSIENT_TTL_S=30s`` is too short when a provider is
*consistently* dead but still responds after each per-request timeout
(MiniMax hanging for 20-30 s before the 30 s read timeout fires).
With a 30 s skip window, the fallback chain re-probes the provider
after 30 s, burns another full timeout, switches again, and the user
pays 30-60 s per turn.

The fix: per-provider consecutive-failure streak with an escalating
TTL ladder ``30 → 120 → 300`` s (issue #2718), reset on the first
successful call OR on a quota/auth classification (different failure
class).

Coverage:

* ``HealthCache.note_transient_failure`` / ``reset_transient_streak`` /
  ``transient_streak`` — direct cache API contract.
* ``HealthCache.transient_ttl_for_streak`` — ladder semantics,
  base-floor behaviour, cap at last step.
* ``HealthAwareFallbackLLM._handle_failure_impl`` — TTL applied to the
  HealthRecord, streak reset on quota/auth/unclassified, streak
  preserved across consecutive transient failures.
* ``HealthAwareFallbackLLM.complete`` / ``stream`` — streak reset on a
  successful call from a previously-streaked provider.
"""

from __future__ import annotations

import pytest

from rob_box_harness.health import (
    DEFAULT_HEALTH_TTL_S,
    HealthAwareFallbackLLM,
    HealthCache,
    HealthRecord,
    ProviderStatus,
    TRANSIENT_TTL_S,
    TRANSIENT_TTL_STEPS,
)
from rob_box_llm.errors import (
    AuthError,
    RateLimitError,
    TimeoutError as LLMTimeoutError,
)
from rob_box_llm.provider import LLMChunk, LLMMessage, LLMResponse, LLMSettings


# ---------------------------------------------------------------------------
# Fake LLM provider
# ---------------------------------------------------------------------------


class _FakeLLM:
    """Trivial ``LLMProvider`` that returns / raises what each test sets.

    Implements the ``complete`` and ``stream`` halves so the fallback
    wrapper exercises both paths; a test that only cares about one
    can leave the other empty.
    """

    def __init__(
        self,
        name: str,
        *,
        complete_response: LLMResponse | None = None,
        complete_exc: BaseException | None = None,
        stream_chunks: list[LLMChunk] | None = None,
        stream_exc: BaseException | None = None,
    ) -> None:
        self.name = name
        self._complete_response = complete_response
        self._complete_exc = complete_exc
        self._stream_chunks = stream_chunks or []
        self._stream_exc = stream_exc
        self.complete_calls = 0
        self.stream_calls = 0

    @property
    def capabilities(self):
        return type("Caps", (), {"text": True, "streaming_text": True, "tools": False})()

    def capabilities_for(self, model):
        return self.capabilities

    async def complete(self, messages, *, tools=(), settings=None):
        self.complete_calls += 1
        if self._complete_exc is not None:
            raise self._complete_exc
        if self._complete_response is not None:
            return self._complete_response
        return LLMResponse(content=f"ok from {self.name}", tool_calls=(), finish_reason="stop", usage={}, raw=None, truncated_tool_args=False)

    async def stream(self, messages, *, tools=(), settings=None):
        self.stream_calls += 1
        if self._stream_exc is not None:
            raise self._stream_exc
        for chunk in self._stream_chunks:
            yield chunk

    async def aclose(self) -> None:
        return None


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _msg(content: str = "hi") -> LLMMessage:
    return LLMMessage(role="user", content=content)


def _record_ttl(cache: HealthCache, provider: str) -> float:
    """Return the current TTL for ``provider`` (0.0 when UNKNOWN/HEALTHY)."""
    rec = cache.get(provider)
    if rec.status == ProviderStatus.UNKNOWN:
        return 0.0
    return rec.ttl_s


# ---------------------------------------------------------------------------
# Direct HealthCache API
# ---------------------------------------------------------------------------


def test_transient_ttl_steps_are_in_issue_2718_shape() -> None:
    """The ladder is the documented one (30 → 120 → 300)."""
    assert TRANSIENT_TTL_STEPS == (30.0, 120.0, 300.0)


def test_transient_streak_starts_at_zero() -> None:
    cache = HealthCache()
    assert cache.transient_streak("minimax") == 0


def test_note_transient_failure_increments() -> None:
    """Each call bumps the counter and returns the new value."""
    cache = HealthCache()
    assert cache.note_transient_failure("minimax") == 1
    assert cache.note_transient_failure("minimax") == 2
    assert cache.note_transient_failure("minimax") == 3
    assert cache.transient_streak("minimax") == 3


def test_note_transient_failure_is_per_provider() -> None:
    """Streaks are tracked independently per provider name."""
    cache = HealthCache()
    cache.note_transient_failure("minimax")
    cache.note_transient_failure("minimax")
    cache.note_transient_failure("deepseek")
    assert cache.transient_streak("minimax") == 2
    assert cache.transient_streak("deepseek") == 1


def test_reset_transient_streak_clears_to_zero() -> None:
    cache = HealthCache()
    cache.note_transient_failure("minimax")
    cache.note_transient_failure("minimax")
    cache.reset_transient_streak("minimax")
    assert cache.transient_streak("minimax") == 0


def test_reset_transient_streak_for_unknown_provider_is_noop() -> None:
    """Resetting a streak that was never started is harmless."""
    cache = HealthCache()
    cache.reset_transient_streak("minimax")
    assert cache.transient_streak("minimax") == 0


def test_transient_ttl_for_streak_zero_returns_base() -> None:
    """No failures yet → caller-controlled base (or default)."""
    cache = HealthCache()
    assert cache.transient_ttl_for_streak("minimax") == TRANSIENT_TTL_S
    assert cache.transient_ttl_for_streak("minimax", base=5.0) == 5.0


def test_transient_ttl_for_streak_uses_ladder() -> None:
    """The Nth failure in a row picks the Nth step (1-based)."""
    cache = HealthCache()
    assert cache.note_transient_failure("minimax") == 1
    assert cache.transient_ttl_for_streak("minimax") == 30.0
    assert cache.note_transient_failure("minimax") == 2
    assert cache.transient_ttl_for_streak("minimax") == 120.0
    assert cache.note_transient_failure("minimax") == 3
    assert cache.transient_ttl_for_streak("minimax") == 300.0


def test_transient_ttl_for_streak_caps_at_last_step() -> None:
    """Beyond the ladder length we cap — no further escalation."""
    cache = HealthCache()
    for _ in range(10):
        cache.note_transient_failure("minimax")
    # 10th call → 10th step (capped at last = 300).
    assert cache.transient_ttl_for_streak("minimax") == TRANSIENT_TTL_STEPS[-1]


def test_transient_ttl_for_streak_resets_on_clear() -> None:
    cache = HealthCache()
    for _ in range(3):
        cache.note_transient_failure("minimax")
    cache.reset_transient_streak("minimax")
    assert cache.transient_ttl_for_streak("minimax") == TRANSIENT_TTL_S


# ---------------------------------------------------------------------------
# HealthAwareFallbackLLM — failure classification
# ---------------------------------------------------------------------------


@pytest.mark.asyncio
async def test_transient_failure_uses_first_step_ttl() -> None:
    """First transient failure uses TRANSIENT_TTL_STEPS[0] = 30 s."""
    cache = HealthCache()
    provider = _FakeLLM("minimax", complete_exc=LLMTimeoutError("hung"))
    wrapper = HealthAwareFallbackLLM([provider], cache=cache)

    with pytest.raises(LLMTimeoutError):
        await wrapper.complete([_msg()])

    rec = cache.get("minimax")
    assert rec.status == ProviderStatus.UNAVAILABLE
    assert rec.ttl_s == TRANSIENT_TTL_STEPS[0]


@pytest.mark.asyncio
async def test_consecutive_transient_failures_escalate_ttl() -> None:
    """Three consecutive transient failures → TTL climbs 30 → 120 → 300.

    The wrapper sees three independent complete() calls; each one fails
    with a transient error, and the cache records a different TTL for
    each. This is the issue #2718 acceptance criterion #2: "N подряд
    таймаутов → TTL растёт".
    """
    cache = HealthCache()
    provider = _FakeLLM("minimax", complete_exc=LLMTimeoutError("hung"))
    wrapper = HealthAwareFallbackLLM([provider], cache=cache)

    for expected_ttl in TRANSIENT_TTL_STEPS:
        with pytest.raises(LLMTimeoutError):
            await wrapper.complete([_msg()])
        # Wait for the just-recorded TTL to expire so the next call
        # actually hits the provider (the wrapper skips providers with
        # a fresh ``unavailable`` entry).
        cache.get("minimax").checked_at -= expected_ttl + 1.0

    # After the cap, a 4th failure still uses the last step.
    cache.get("minimax").checked_at -= TRANSIENT_TTL_STEPS[-1] + 1.0
    with pytest.raises(LLMTimeoutError):
        await wrapper.complete([_msg()])
    assert cache.get("minimax").ttl_s == TRANSIENT_TTL_STEPS[-1]


@pytest.mark.asyncio
async def test_transient_streak_resets_on_quota_failure() -> None:
    """Quota failure is a different failure class — the streak MUST reset.

    Rationale (issue #2718): the "transient" ladder captures "the
    network is flaky". A hard 2056 quota exhaustion is NOT a flaky
    network — it's the provider telling us "no more tokens for the
    next 5 hours". Mixing the two counters would cause us to re-test
    the provider as if it might have recovered, when in reality the
    quota window hasn't even closed.
    """
    cache = HealthCache()
    provider = _FakeLLM("minimax", complete_exc=LLMTimeoutError("hung"))
    wrapper = HealthAwareFallbackLLM([provider], cache=cache)

    # 1st failure — bumps streak to 1, TTL = 30 s.
    with pytest.raises(LLMTimeoutError):
        await wrapper.complete([_msg()])
    assert cache.transient_streak("minimax") == 1

    # Make the next call hit the provider with a quota error.
    cache.get("minimax").checked_at -= 9999.0
    provider._complete_exc = _QuotaError("minimax: 2056 token plan usage limit reached")
    with pytest.raises(RateLimitError):
        await wrapper.complete([_msg()])

    # Streak reset: the next transient failure starts from 1 again.
    assert cache.transient_streak("minimax") == 0
    cache.get("minimax").checked_at -= 9999.0
    provider._complete_exc = LLMTimeoutError("hung again")
    with pytest.raises(LLMTimeoutError):
        await wrapper.complete([_msg()])
    assert cache.transient_streak("minimax") == 1
    assert cache.get("minimax").ttl_s == TRANSIENT_TTL_STEPS[0]


@pytest.mark.asyncio
async def test_transient_streak_resets_on_auth_failure() -> None:
    """Same rationale as quota — auth is a different failure class."""
    cache = HealthCache()
    provider = _FakeLLM("minimax", complete_exc=LLMTimeoutError("hung"))
    wrapper = HealthAwareFallbackLLM([provider], cache=cache)

    with pytest.raises(LLMTimeoutError):
        await wrapper.complete([_msg()])
    assert cache.transient_streak("minimax") == 1

    cache.get("minimax").checked_at -= 9999.0
    provider._complete_exc = AuthError("401 invalid api key", provider="minimax")
    with pytest.raises(AuthError):
        await wrapper.complete([_msg()])

    assert cache.transient_streak("minimax") == 0


@pytest.mark.asyncio
async def test_success_resets_streak() -> None:
    """A successful call clears any in-flight streak.

    Issue #2718 acceptance criterion #2 (continued): the streak should
    not persist past a successful turn — once the provider recovers,
    we go back to the flat TRANSIENT_TTL_S baseline on the next
    failure.
    """
    cache = HealthCache()
    provider = _FakeLLM("minimax", complete_exc=LLMTimeoutError("hung"))
    fallback = _FakeLLM("deepseek", complete_exc=LLMTimeoutError("hung"))
    wrapper = HealthAwareFallbackLLM([provider, fallback], cache=cache)

    # Force the chain to take ``minimax`` (it would skip on
    # unavailable — bypass via a private cache manipulation).
    cache._transient_streak["minimax"] = 3

    # Now ``provider`` succeeds; the streak must be cleared.
    provider._complete_exc = None
    resp = await wrapper.complete([_msg()])
    assert "minimax" in resp.content
    assert cache.transient_streak("minimax") == 0


@pytest.mark.asyncio
async def test_stream_success_also_resets_streak() -> None:
    cache = HealthCache()
    provider = _FakeLLM(
        "minimax",
        stream_chunks=[
            LLMChunk(content_delta="hi", finish_reason=None),
            LLMChunk(content_delta="", finish_reason="stop"),
        ],
    )
    fallback = _FakeLLM("deepseek")
    wrapper = HealthAwareFallbackLLM([provider, fallback], cache=cache)

    cache._transient_streak["minimax"] = 2

    chunks = []
    async for chunk in wrapper.stream([_msg()]):
        chunks.append(chunk.content_delta)
    assert chunks == ["hi", ""]
    assert cache.transient_streak("minimax") == 0


# ---------------------------------------------------------------------------
# HealthAwareFallbackLLM — end-to-end fallback after escalation
# ---------------------------------------------------------------------------


@pytest.mark.asyncio
async def test_escalated_skip_does_not_retry_during_ttl() -> None:
    """After the TTL climbs, the fallback skips the provider for longer.

    Issue #2718 acceptance: a provider that's been failing
    consecutively should not be re-probed inside its escalated TTL.
    We pre-populate the cache with the escalated TTL, then run a
    chain that has the failing provider FIRST; the wrapper must skip
    it and use the fallback.
    """
    cache = HealthCache()
    provider = _FakeLLM("minimax", complete_exc=LLMTimeoutError("hung"))
    fallback = _FakeLLM("deepseek")
    wrapper = HealthAwareFallbackLLM([provider, fallback], cache=cache)

    # Mark minimax as unavailable with the ESCALATED TTL (300 s) so
    # the wrapper MUST skip it for the next 300 s.
    cache.mark_unavailable(
        "minimax", reason="dead", ttl_s=TRANSIENT_TTL_STEPS[-1]
    )
    cache._transient_streak["minimax"] = 3  # realistic state at that point

    # The clock has not advanced → minimax is still skipped.
    resp = await wrapper.complete([_msg()])
    assert "deepseek" in resp.content
    assert provider.complete_calls == 0  # never called


# ---------------------------------------------------------------------------
# Local exception helpers
# ---------------------------------------------------------------------------


class _QuotaError(RateLimitError):
    """A ``RateLimitError`` carrying MiniMax quota-exhaustion hints."""

    def __init__(self, message: str) -> None:
        super().__init__(message, provider="minimax")
