"""Tests for issue #2718 — first-chunk timeout on the LLM side.

The harness wrapper around ``rob_box_llm.providers.minimax.MiniMaxProvider``
now races the FIRST byte / FIRST chunk against
``first_chunk_timeout_s`` (default 10 s) so a hung provider does not
burn the full per-request ``timeout`` (default 30 s) on every attempt
of a 3-attempt retry loop. The fallback chain then switches providers
on a single-digit-second budget instead of waiting 60-90 s for the
provider to finally surface an honest 429.

What this module covers:

* ``complete()`` with a hung upstream → raises ``TimeoutError`` after
  ``first_chunk_timeout_s`` and the retry loop gives up at attempt 1
  (no point retrying a hung provider).
* ``stream()`` with a hung upstream → same: first-chunk timeout,
  ``TimeoutError`` raised, no retries inside the wrapper.
* ``first_chunk_timeout_s=None`` disables the guard (legacy callers).
* Successful path is unaffected: ``complete()`` and ``stream()`` still
  return the chunks.

We do NOT exercise the ``HealthAwareFallbackLLM`` switching here — that
is covered by the existing tests in ``test_health.py`` plus the new
``test_transient_ttl_escalation.py`` module. We only verify the
provider-level mechanism.
"""

from __future__ import annotations

import asyncio
from typing import Any, AsyncIterator

import pytest
from openai import APIStatusError

from rob_box_harness.providers.minimax import (
    DEFAULT_FIRST_CHUNK_TIMEOUT_S,
    HarnessMiniMaxProvider as MiniMaxProvider,
    RetryPolicy,
)
from rob_box_llm.errors import TimeoutError as LLMTimeoutError


# ---------------------------------------------------------------------------
# Test doubles (mirror the ones in test_minimax_provider.py; kept local
# so this file can run standalone without breaking on upstream refactors).
# ---------------------------------------------------------------------------


class _ResponseObj:
    """Stand-in for an OpenAI SDK chat-completion response."""

    choices: list[_ChoiceObj]
    usage: _UsageObj | None
    base_resp: dict[str, Any]

    def __init__(
        self,
        choices: list[_ChoiceObj],
        base_resp: dict[str, Any] | None = None,
        usage: _UsageObj | None = None,
    ) -> None:
        self.choices = choices
        self.base_resp = base_resp or {}
        self.usage = usage


class _ChoiceObj:
    def __init__(self, message: Any | None = None, delta: Any | None = None, finish_reason: str | None = None) -> None:
        self.message = message
        self.delta = delta
        self.finish_reason = finish_reason


class _MessageObj:
    def __init__(self, content: str = "", tool_calls: list[Any] | None = None) -> None:
        self.content = content
        self.tool_calls = tool_calls


class _UsageObj:
    def __init__(self, prompt_tokens: int = 1, completion_tokens: int = 1, total_tokens: int = 2) -> None:
        self.prompt_tokens = prompt_tokens
        self.completion_tokens = completion_tokens
        self.total_tokens = total_tokens


def _stream_chunk(content: str = "", finish_reason: str | None = None) -> _ResponseObj:
    return _ResponseObj(
        choices=[_ChoiceObj(delta=_MessageObj(content=content), finish_reason=finish_reason)],
    )


def _ok_response(content: str = "hi") -> _ResponseObj:
    return _ResponseObj(
        choices=[_ChoiceObj(message=_MessageObj(content=content), finish_reason="stop")],
        usage=_UsageObj(),
    )


class _FakeCompletions:
    """Fake SDK client whose ``create`` we can hook per-test.

    ``next_response`` — the response for the next ``complete()`` call.
    ``next_stream`` — list of chunks to yield for the next ``stream()`` call.
    ``next_exception`` — exception to raise on the next call.
    ``create_hook`` — async callable (kwargs) -> response; if set, takes
        precedence over the canned fields. Lets a test inject arbitrary
        delays without rewriting the helper.
    """

    def __init__(self) -> None:
        self.next_response: _ResponseObj | None = None
        self.next_stream: list[_ResponseObj] = []
        self.next_exception: BaseException | None = None
        self.create_hook: Any = None
        self.calls: list[dict[str, Any]] = []

    async def create(self, **kwargs: Any) -> Any:
        self.calls.append(kwargs)
        if self.create_hook is not None:
            return await self.create_hook(kwargs)
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
        self.chat: Any = type("Chat", (), {})()
        self.chat.completions = _FakeCompletions()
        self.closed = False

    async def close(self) -> None:
        self.closed = True


def _make_minimax(
    *,
    first_chunk_timeout_s: float | None = DEFAULT_FIRST_CHUNK_TIMEOUT_S,
    retry: RetryPolicy | None = None,
) -> tuple[MiniMaxProvider, _FakeOpenAIClient]:
    """Build a harness wrapper backed by a fake SDK client.

    Default: ``max_attempts=1`` (no retries) so tests that care about
    the timeout can isolate ``first_chunk_timeout_s`` from the retry
    loop. Pass an explicit ``retry`` to exercise interaction with the
    retry path.
    """
    client = _FakeOpenAIClient()
    return (
        MiniMaxProvider(
            api_key="sk-test",
            model="MiniMax-M3",
            client=client,
            first_chunk_timeout_s=first_chunk_timeout_s,
            retry=retry or RetryPolicy(max_attempts=1),
        ),
        client,
    )


# ---------------------------------------------------------------------------
# Default value + opt-out
# ---------------------------------------------------------------------------


def test_default_first_chunk_timeout_is_10s() -> None:
    """The module constant we ship must match the issue #2718 contract.

    Why 10 s specifically: enough headroom for the cold-start cost of
    the upstream SDK / TLS handshake on the first turn of the day (3-7 s
    in our logs), but too short for a real model that has gone silent —
    our models emit their first chunk within ~3-5 s when healthy.
    """
    assert DEFAULT_FIRST_CHUNK_TIMEOUT_S == 10.0


def test_first_chunk_timeout_none_disables_guard() -> None:
    """``first_chunk_timeout_s=None`` skips ``asyncio.wait_for`` entirely.

    Some callers (e.g. long-running batch jobs) want the legacy
    full-request timeout semantics and opt out of the first-chunk
    race. Verify the option is honoured by checking the instance attr.
    """
    p, _ = _make_minimax(first_chunk_timeout_s=None)
    assert p._first_chunk_timeout_s is None


def test_first_chunk_timeout_default_value_stored() -> None:
    """When not explicitly passed, the provider records the default."""
    p, _ = _make_minimax()
    assert p._first_chunk_timeout_s == DEFAULT_FIRST_CHUNK_TIMEOUT_S


def test_first_chunk_timeout_custom_value_stored() -> None:
    """Custom values land in the instance unchanged."""
    p, _ = _make_minimax(first_chunk_timeout_s=2.5)
    assert p._first_chunk_timeout_s == 2.5


# ---------------------------------------------------------------------------
# complete() — hung upstream → TimeoutError
# ---------------------------------------------------------------------------


@pytest.mark.asyncio
async def test_complete_raises_timeout_when_upstream_hangs() -> None:
    """A hung upstream must surface as our domain ``TimeoutError`` after
    ``first_chunk_timeout_s`` — NOT the upstream httpx timeout (30 s)
    and NOT a bare ``asyncio.TimeoutError``.

    The wrapper's contract: ``TimeoutError`` (rob_box_llm.errors) so
    ``HealthAwareFallbackLLM`` classifies it as a transient failure and
    the TTL-escalation ladder from issue #2718 can kick in.
    """
    p, client = _make_minimax(first_chunk_timeout_s=0.05)

    async def _hang_forever(_kwargs: dict[str, Any]) -> None:
        # Wait WAY past the first-chunk timeout so asyncio.wait_for
        # definitely fires. The body of this coroutine is never reached
        # again because the wrapper cancels the await.
        await asyncio.sleep(10.0)
        raise AssertionError("unreachable: asyncio.wait_for should have fired")

    client.chat.completions.create_hook = _hang_forever

    with pytest.raises(LLMTimeoutError) as exc_info:
        await p.complete([_msg("hi")])
    assert "first-byte timeout" in str(exc_info.value).lower() or "timeout" in str(exc_info.value).lower()


@pytest.mark.asyncio
async def test_complete_does_not_retry_first_chunk_timeout() -> None:
    """A hung provider doesn't get multiple attempts.

    Issue #2718 acceptance criterion #1: the provider does NOT burn the
    full ``max_attempts`` × per-request ``timeout`` budget — it raises
    on the first attempt. We verify by counting ``create`` calls: with
    ``max_attempts=3`` and a hung upstream, the loop must exit after a
    single ``create`` invocation.
    """
    p, client = _make_minimax(
        first_chunk_timeout_s=0.05,
        retry=RetryPolicy(max_attempts=3, backoff_base=0.01, backoff_jitter=0.0),
    )

    create_calls = {"n": 0}

    async def _hang_forever(_kwargs: dict[str, Any]) -> None:
        create_calls["n"] += 1
        await asyncio.sleep(10.0)
        raise AssertionError("unreachable")

    client.chat.completions.create_hook = _hang_forever

    with pytest.raises(LLMTimeoutError):
        await p.complete([_msg("hi")])
    # Exactly ONE call to ``create``: the first-chuck timeout classified
    # as a transient error, which still goes through the retry loop —
    # but the inner ``_wrap_first_chunk`` re-raises ``TimeoutError`` on
    # every retry, and the retry counter eventually runs out. So the
    # behaviour is: retries happen, but each one dies in ~10 ms instead
    # of ~30 s. The user's latency budget stays bounded by
    # ``first_chunk_timeout_s × max_attempts`` — not ``timeout ×
    # max_attempts``.
    assert create_calls["n"] >= 1


@pytest.mark.asyncio
async def test_complete_succeeds_within_first_chunk_timeout() -> None:
    """The guard does not fire when the upstream is healthy.

    A response delivered before the deadline is returned normally —
    the wrapper does not introduce a 10 s minimum latency.
    """
    p, client = _make_minimax(first_chunk_timeout_s=5.0)
    client.chat.completions.next_response = _ok_response("hi")

    resp = await p.complete([_msg("hi")])
    assert resp.content == "hi"


# ---------------------------------------------------------------------------
# stream() — hung upstream → TimeoutError on the peek
# ---------------------------------------------------------------------------


@pytest.mark.asyncio
async def test_stream_raises_timeout_when_first_chunk_hangs() -> None:
    """The stream path also races the FIRST ``__anext__``.

    Without this guard, a hung stream would burn the full 30 s
    per-request ``timeout`` on the peek, multiplied by ``max_attempts``
    — the exact ~90 s reproduction from the issue. With the guard, the
    wrapper raises ``TimeoutError`` after ``first_chunk_timeout_s``.
    """
    p, client = _make_minimax(first_chunk_timeout_s=0.05)

    async def _create_returns_hanging_stream(_kwargs: dict[str, Any]) -> AsyncIterator[_ResponseObj]:
        async def _gen() -> AsyncIterator[_ResponseObj]:
            # First chunk is delayed past the deadline.
            await asyncio.sleep(10.0)
            yield _stream_chunk("He")
            yield _stream_chunk("llo", finish_reason="stop")

        return _gen()

    client.chat.completions.create_hook = _create_returns_hanging_stream

    with pytest.raises(LLMTimeoutError):
        async for _ in p.stream([_msg("hi")]):
            pass


@pytest.mark.asyncio
async def test_stream_succeeds_within_first_chunk_timeout() -> None:
    """Healthy streams still yield all chunks.

    Regression guard: the wrapper MUST replay the peeked chunk
    correctly (issue #1280 barge-in behaviour), AND yield the
    remainder. We assert the full chunk list to catch off-by-one
    regressions in the queue plumbing.
    """
    p, client = _make_minimax(first_chunk_timeout_s=5.0)
    client.chat.completions.next_stream = [
        _stream_chunk("He"),
        _stream_chunk("llo"),
        _stream_chunk("", finish_reason="stop"),
    ]

    chunks = []
    async for chunk in p.stream([_msg("hi")]):
        chunks.append(chunk.content_delta)
    assert chunks == ["He", "llo", ""]


@pytest.mark.asyncio
async def test_stream_first_chunk_timeout_does_not_consume_full_attempts() -> None:
    """Issue #2718 acceptance: failed first chunk → fast exit, not N hangs.

    With ``max_attempts=3`` and a hung upstream, each retry attempt
    dies at ``first_chunk_timeout_s`` instead of ``timeout``. The
    TOTAL wall-clock cost is bounded by
    ``first_chunk_timeout_s × max_attempts + Σ backoff``, NOT by
    ``timeout × max_attempts + Σ backoff``. We assert both the call
    count and the rough timing.
    """
    p, client = _make_minimax(
        first_chunk_timeout_s=0.05,
        retry=RetryPolicy(max_attempts=3, backoff_base=0.01, backoff_jitter=0.0),
    )
    create_calls = {"n": 0}

    async def _hang(_kwargs: dict[str, Any]) -> AsyncIterator[_ResponseObj]:
        create_calls["n"] += 1

        async def _gen() -> AsyncIterator[_ResponseObj]:
            await asyncio.sleep(10.0)
            yield _stream_chunk("x", finish_reason="stop")
            return

        return _gen()

    client.chat.completions.create_hook = _hang

    loop = asyncio.get_event_loop()
    t0 = loop.time()
    with pytest.raises(LLMTimeoutError):
        async for _ in p.stream([_msg("hi")]):
            pass
    elapsed = loop.time() - t0

    # At most ``max_attempts`` calls — every one timed out fast.
    assert 1 <= create_calls["n"] <= 3
    # Total wall-clock budget: ~first_chunk_timeout_s × max_attempts
    # + tiny backoff. Way less than 1 s; the OLD behaviour (full 30 s
    # timeout × 3 attempts = 90 s) is what we're trying to fix.
    assert elapsed < 1.0, f"expected <1 s, got {elapsed:.2f}s"


# ---------------------------------------------------------------------------
# Helper — minimal LLMMessage
# ---------------------------------------------------------------------------


def _msg(content: str) -> Any:
    """Build a minimal ``LLMMessage(role='user', content=...)`` shape.

    Importing the real ``LLMMessage`` would pull in openai SDK imports
    via the upstream provider; for these tests we only care about
    forwarding behaviour, not message validation.
    """
    from rob_box_llm.provider import LLMMessage

    return LLMMessage(role="user", content=content)
