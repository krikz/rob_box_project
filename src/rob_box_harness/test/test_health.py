"""Tests for ``rob_box_harness.health`` — provider health-check / quota cache.

Covers the issue #1082 contract:

* :func:`is_quota_exhausted` — MiniMax 2056/1008 must be classified as
  quota exhaustion (NOT transient), generic 429 as transient.
* :class:`HealthCache` — TTL semantics, skip-while-fresh, re-check after
  expiry, optional JSON persistence.
* :func:`check_deepseek_balance` — real ``/user/balance`` parsing with a
  mocked HTTP transport; a broken balance API returns ``None`` (provider
  stays healthy).
* :class:`HealthAwareFallbackLLM` — the dynamic chain
  ``[healthy] + [unchecked]`` with dead providers skipped; proactive
  probing; reactive error-code detection; auth/quota handling; stream
  switching.

No real network is touched — the HTTP transport is mocked at the
``httpx`` layer.
"""

from __future__ import annotations

from typing import Any, AsyncIterator, Iterable, Mapping

import httpx
import pytest

from rob_box_harness.health import (
    DEFAULT_HEALTH_TTL_S,
    TRANSIENT_TTL_S,
    HealthAwareFallbackLLM,
    HealthCache,
    ProviderStatus,
    check_deepseek_balance,
    is_auth_failure,
    is_quota_exhausted,
)
from rob_box_llm.errors import (
    AuthError,
    ProviderError,
    RateLimitError,
    TimeoutError as LLMTimeoutError,
)
from rob_box_llm.provider import LLMChunk, LLMMessage, LLMResponse, LLMSettings


# ---------------------------------------------------------------------------
# Fake LLM provider
# ---------------------------------------------------------------------------


class _FakeProvider:
    """Minimal LLMProvider double with call counting + optional failure."""

    def __init__(self, name: str, fail: BaseException | None = None) -> None:
        self.name = name
        self.fail = fail
        self.calls = 0
        self.stream_calls = 0
        self.closed = False

    async def complete(
        self,
        messages: Iterable[LLMMessage],
        *,
        tools: Iterable[Mapping[str, Any]] = (),
        settings: LLMSettings | None = None,
    ) -> LLMResponse:
        self.calls += 1
        if self.fail is not None:
            raise self.fail
        return LLMResponse(content=f"from-{self.name}")

    async def stream(
        self,
        messages: Iterable[LLMMessage],
        *,
        tools: Iterable[Mapping[str, Any]] = (),
        settings: LLMSettings | None = None,
    ) -> AsyncIterator[LLMChunk]:
        self.stream_calls += 1
        if self.fail is not None:
            raise self.fail
        yield LLMChunk(content_delta=f"from-{self.name}", finish_reason="stop")

    async def aclose(self) -> None:
        self.closed = True

    @property
    def capabilities(self):
        from rob_box_llm.provider import ProviderCapabilities

        return ProviderCapabilities(text=True)


class _FakeClock:
    """Injectable clock for HealthCache TTL tests."""

    def __init__(self, start: float = 1000.0) -> None:
        self.now = start

    def __call__(self) -> float:
        return self.now


_QUOTA_MSG = "429: rate_limit_error Token Plan usage limit reached (2056)"


# ---------------------------------------------------------------------------
# Error classification
# ---------------------------------------------------------------------------


def test_is_quota_exhausted_recognizes_minimax_2056() -> None:
    assert is_quota_exhausted(RateLimitError(_QUOTA_MSG, provider="minimax")) is True


def test_is_quota_exhausted_recognizes_1008_insufficient_balance() -> None:
    assert is_quota_exhausted(RateLimitError("minimax: 1008 insufficient balance")) is True


def test_is_quota_exhausted_false_for_generic_429_burst() -> None:
    # A plain rate-limited 429 is transient — the provider retry loop
    # must still handle it.
    assert is_quota_exhausted(RateLimitError("429: rate-limited", provider="minimax")) is False


def test_is_quota_exhausted_false_for_timeout_and_other() -> None:
    assert is_quota_exhausted(LLMTimeoutError("timeout")) is False
    assert is_quota_exhausted(ProviderError("boom")) is False
    assert is_quota_exhausted(ValueError("boom")) is False


def test_is_auth_failure_detects_auth_error_and_401_403() -> None:
    assert is_auth_failure(AuthError("bad key", provider="minimax")) is True
    assert is_auth_failure(ProviderError("401 Unauthorized")) is True
    assert is_auth_failure(ProviderError("403 Forbidden")) is True
    assert is_auth_failure(RateLimitError(_QUOTA_MSG)) is False


# ---------------------------------------------------------------------------
# HealthCache
# ---------------------------------------------------------------------------


def test_cache_defaults_to_unknown() -> None:
    cache = HealthCache()
    assert cache.status("minimax") == ProviderStatus.UNKNOWN
    assert cache.is_unavailable("minimax") is False


def test_cache_mark_unavailable_skips_within_ttl() -> None:
    clock = _FakeClock(start=1000.0)
    cache = HealthCache(clock=clock)
    cache.mark_unavailable("minimax", reason="quota 2056")
    assert cache.status("minimax") == ProviderStatus.UNAVAILABLE
    assert cache.is_unavailable("minimax") is True
    # TTL has not passed → still dead.
    clock.now += DEFAULT_HEALTH_TTL_S - 1
    assert cache.is_unavailable("minimax") is True


def test_cache_expired_unavailable_flips_back_to_unknown() -> None:
    clock = _FakeClock(start=1000.0)
    cache = HealthCache(ttl_s=300.0, clock=clock)
    cache.mark_unavailable("minimax", reason="quota 2056")
    clock.now += 301.0  # TTL passed
    assert cache.status("minimax") == ProviderStatus.UNKNOWN
    assert cache.is_unavailable("minimax") is False


def test_cache_mark_healthy_returns_healthy() -> None:
    cache = HealthCache()
    cache.mark_healthy("deepseek", balance=110.0)
    assert cache.status("deepseek") == ProviderStatus.HEALTHY
    assert cache.is_unavailable("deepseek") is False


def test_cache_transient_ttl_shorter_than_default() -> None:
    clock = _FakeClock(start=1000.0)
    cache = HealthCache(clock=clock)
    cache.mark_unavailable("minimax", reason="burst 429", ttl_s=TRANSIENT_TTL_S)
    assert cache.is_unavailable("minimax") is True
    clock.now += TRANSIENT_TTL_S + 1
    assert cache.is_unavailable("minimax") is False


def test_cache_persistence_roundtrip(tmp_path) -> None:
    clock = _FakeClock(start=1000.0)
    path = tmp_path / "llm_health.json"
    cache = HealthCache(persist_path=path, clock=clock)
    cache.mark_unavailable("minimax", reason="quota 2056")

    # A fresh cache (simulating a robot restart) loads the record.
    reloaded = HealthCache(persist_path=path, clock=clock)
    assert reloaded.is_unavailable("minimax") is True
    assert reloaded.get("minimax").reason == "quota 2056"


def test_cache_persistence_expired_on_load_is_unknown(tmp_path) -> None:
    clock = _FakeClock(start=1000.0)
    path = tmp_path / "llm_health.json"
    cache = HealthCache(persist_path=path, clock=clock)
    cache.mark_unavailable("minimax", reason="quota 2056")
    # Restart much later — TTL passed during the downtime.
    clock.now += 10_000.0
    reloaded = HealthCache(persist_path=path, clock=clock)
    assert reloaded.is_unavailable("minimax") is False
    assert reloaded.status("minimax") == ProviderStatus.UNKNOWN


def test_cache_persistence_missing_file_starts_empty(tmp_path) -> None:
    cache = HealthCache(persist_path=tmp_path / "nope.json")
    assert cache.status("minimax") == ProviderStatus.UNKNOWN


def test_cache_persistence_corrupt_file_starts_empty(tmp_path) -> None:
    path = tmp_path / "llm_health.json"
    path.write_text("{not json", encoding="utf-8")
    cache = HealthCache(persist_path=path)
    assert cache.status("minimax") == ProviderStatus.UNKNOWN


# ---------------------------------------------------------------------------
# check_deepseek_balance (mocked HTTP transport)
# ---------------------------------------------------------------------------


def _patch_httpx_client(monkeypatch: pytest.MonkeyPatch, handler) -> httpx.AsyncClient:
    """Swap ``httpx.AsyncClient`` for one using ``MockTransport``."""
    import rob_box_harness.health as health_mod

    client = httpx.AsyncClient(transport=httpx.MockTransport(handler))
    monkeypatch.setattr(health_mod.httpx, "AsyncClient", lambda *a, **k: client)
    return client


@pytest.mark.asyncio
async def test_check_deepseek_balance_returns_total(monkeypatch: pytest.MonkeyPatch) -> None:
    def handler(request: httpx.Request) -> httpx.Response:
        assert request.url.path.endswith("/user/balance")
        assert request.headers["authorization"] == "Bearer sk-test"
        return httpx.Response(
            200,
            json={
                "is_available": True,
                "balance_infos": [
                    {"currency": "CNY", "total_balance": "100.00"},
                    {"currency": "CNY", "total_balance": "10.00"},
                ],
            },
            request=request,
        )

    client = _patch_httpx_client(monkeypatch, handler)
    balance = await check_deepseek_balance("https://api.deepseek.com", "sk-test")
    assert balance == pytest.approx(110.0)
    await client.aclose()


@pytest.mark.asyncio
async def test_check_deepseek_balance_unavailable_returns_zero(monkeypatch: pytest.MonkeyPatch) -> None:
    def handler(request: httpx.Request) -> httpx.Response:
        return httpx.Response(
            200, json={"is_available": False, "balance_infos": []}, request=request
        )

    client = _patch_httpx_client(monkeypatch, handler)
    balance = await check_deepseek_balance("https://api.deepseek.com", "sk-test")
    assert balance == 0.0
    await client.aclose()


@pytest.mark.asyncio
async def test_check_deepseek_balance_network_error_returns_none(monkeypatch: pytest.MonkeyPatch) -> None:
    def handler(request: httpx.Request) -> httpx.Response:
        raise httpx.ConnectError("boom", request=request)

    client = _patch_httpx_client(monkeypatch, handler)
    balance = await check_deepseek_balance("https://api.deepseek.com", "sk-test")
    assert balance is None
    await client.aclose()


@pytest.mark.asyncio
async def test_check_deepseek_balance_http_error_returns_none(monkeypatch: pytest.MonkeyPatch) -> None:
    def handler(request: httpx.Request) -> httpx.Response:
        return httpx.Response(500, request=request)

    client = _patch_httpx_client(monkeypatch, handler)
    balance = await check_deepseek_balance("https://api.deepseek.com", "sk-test")
    assert balance is None
    await client.aclose()


# ---------------------------------------------------------------------------
# HealthAwareFallbackLLM — complete
# ---------------------------------------------------------------------------


def _msg(text: str = "hi") -> list[LLMMessage]:
    return [LLMMessage(role="user", content=text)]


@pytest.mark.asyncio
async def test_fallback_switches_on_quota_exhaustion_and_marks_dead() -> None:
    """Primary (MiniMax) 2056 → fallback answers, primary marked unavailable."""
    primary = _FakeProvider("minimax", fail=RateLimitError(_QUOTA_MSG, provider="minimax"))
    fallback = _FakeProvider("deepseek")
    cache = HealthCache()
    wrapper = HealthAwareFallbackLLM([primary, fallback], cache=cache)

    response = await wrapper.complete(_msg())

    assert response.content == "from-deepseek"
    assert primary.calls == 1  # exactly ONE attempt, no retry cycle
    assert fallback.calls == 1
    assert cache.is_unavailable("minimax") is True


@pytest.mark.asyncio
async def test_premarked_unavailable_primary_not_called_at_all() -> None:
    """The 'первый же запрос идёт на fallback' acceptance: a provider in the
    cache as unavailable is skipped BEFORE any request."""
    primary = _FakeProvider("minimax")
    fallback = _FakeProvider("deepseek")
    cache = HealthCache()
    cache.mark_unavailable("minimax", reason="quota 2056")

    wrapper = HealthAwareFallbackLLM([primary, fallback], cache=cache)
    response = await wrapper.complete(_msg())

    assert response.content == "from-deepseek"
    assert primary.calls == 0  # dead provider never touched
    assert fallback.calls == 1


@pytest.mark.asyncio
async def test_after_ttl_expiry_primary_tried_again() -> None:
    """TTL expiry → the provider is re-checked (quota may have refilled)."""
    clock = _FakeClock(start=1000.0)
    primary = _FakeProvider("minimax")
    fallback = _FakeProvider("deepseek")
    cache = HealthCache(clock=clock)
    cache.mark_unavailable("minimax", reason="quota 2056")

    wrapper = HealthAwareFallbackLLM([primary, fallback], cache=cache)
    # Still inside TTL → fallback.
    assert (await wrapper.complete(_msg())).content == "from-deepseek"
    assert primary.calls == 0

    clock.now += DEFAULT_HEALTH_TTL_S + 1  # TTL expired
    response = await wrapper.complete(_msg())
    assert response.content == "from-minimax"
    assert primary.calls == 1


@pytest.mark.asyncio
async def test_balance_probe_zero_marks_unavailable_before_first_call() -> None:
    """DeepSeek balance=0 → provider marked unavailable by the probe, no
    request is sent to it."""
    primary = _FakeProvider("deepseek")
    fallback = _FakeProvider("minimax")
    cache = HealthCache()
    wrapper = HealthAwareFallbackLLM(
        [primary, fallback],
        cache=cache,
        balance_checkers={"deepseek": lambda: _balance(0.0)},
    )

    response = await wrapper.complete(_msg())

    assert response.content == "from-minimax"
    assert primary.calls == 0
    assert cache.is_unavailable("deepseek") is True
    assert cache.get("deepseek").reason == "balance=0.0"


@pytest.mark.asyncio
async def test_balance_probe_healthy_keeps_provider_first() -> None:
    primary = _FakeProvider("deepseek")
    fallback = _FakeProvider("minimax")
    cache = HealthCache()
    wrapper = HealthAwareFallbackLLM(
        [primary, fallback],
        cache=cache,
        balance_checkers={"deepseek": lambda: _balance(50.0)},
    )

    response = await wrapper.complete(_msg())

    assert response.content == "from-deepseek"
    assert primary.calls == 1
    assert fallback.calls == 0
    assert cache.status("deepseek") == ProviderStatus.HEALTHY


@pytest.mark.asyncio
async def test_balance_probe_error_keeps_provider_usable() -> None:
    """Balance API down → provider must NOT be blocked (edge case #1082)."""
    primary = _FakeProvider("deepseek")
    fallback = _FakeProvider("minimax")
    cache = HealthCache()

    async def broken_probe() -> float:
        raise RuntimeError("balance api down")

    wrapper = HealthAwareFallbackLLM(
        [primary, fallback],
        cache=cache,
        balance_checkers={"deepseek": broken_probe},
    )

    response = await wrapper.complete(_msg())

    assert response.content == "from-deepseek"
    assert primary.calls == 1
    assert cache.is_unavailable("deepseek") is False


@pytest.mark.asyncio
async def test_balance_probe_cached_not_called_every_request() -> None:
    """TTL-cache: the balance API is not hammered per request."""
    calls = {"n": 0}

    async def probing_probe() -> float:
        calls["n"] += 1
        return 50.0

    primary = _FakeProvider("deepseek")
    fallback = _FakeProvider("minimax")
    cache = HealthCache()
    wrapper = HealthAwareFallbackLLM(
        [primary, fallback],
        cache=cache,
        balance_checkers={"deepseek": probing_probe},
    )

    await wrapper.complete(_msg())
    await wrapper.complete(_msg())
    await wrapper.complete(_msg())

    assert calls["n"] == 1  # probed once, then cached as HEALTHY


@pytest.mark.asyncio
async def test_auth_error_marks_unavailable_and_uses_fallback() -> None:
    primary = _FakeProvider("minimax", fail=AuthError("2049 invalid api key", provider="minimax"))
    fallback = _FakeProvider("deepseek")
    cache = HealthCache()
    wrapper = HealthAwareFallbackLLM([primary, fallback], cache=cache)

    response = await wrapper.complete(_msg())

    assert response.content == "from-deepseek"
    assert primary.calls == 1
    assert cache.is_unavailable("minimax") is True


@pytest.mark.asyncio
async def test_generic_error_switches_but_does_not_mark_unavailable() -> None:
    """A one-off provider bug must not take the provider out of rotation."""
    primary = _FakeProvider("minimax", fail=ProviderError("boom"))
    fallback = _FakeProvider("deepseek")
    cache = HealthCache()
    wrapper = HealthAwareFallbackLLM([primary, fallback], cache=cache)

    response = await wrapper.complete(_msg())

    assert response.content == "from-deepseek"
    assert cache.is_unavailable("minimax") is False


@pytest.mark.asyncio
async def test_transient_error_marks_short_unavailable() -> None:
    primary = _FakeProvider("minimax", fail=RateLimitError("429: rate-limited"))
    fallback = _FakeProvider("deepseek")
    cache = HealthCache()
    wrapper = HealthAwareFallbackLLM([primary, fallback], cache=cache)

    response = await wrapper.complete(_msg())

    assert response.content == "from-deepseek"
    rec = cache.get("minimax")
    assert rec.status == ProviderStatus.UNAVAILABLE
    assert rec.ttl_s == TRANSIENT_TTL_S


@pytest.mark.asyncio
async def test_all_providers_unavailable_raises() -> None:
    primary = _FakeProvider("minimax")
    fallback = _FakeProvider("deepseek")
    cache = HealthCache()
    cache.mark_unavailable("minimax", reason="quota")
    cache.mark_unavailable("deepseek", reason="quota")

    wrapper = HealthAwareFallbackLLM([primary, fallback], cache=cache)

    with pytest.raises(ProviderError):
        await wrapper.complete(_msg())
    assert primary.calls == 0
    assert fallback.calls == 0


@pytest.mark.asyncio
async def test_last_error_propagates_when_chain_exhausted() -> None:
    primary = _FakeProvider("minimax", fail=RateLimitError(_QUOTA_MSG, provider="minimax"))
    fallback = _FakeProvider("deepseek", fail=ProviderError("deepseek down"))
    wrapper = HealthAwareFallbackLLM([primary, fallback])

    with pytest.raises(ProviderError, match="deepseek down"):
        await wrapper.complete(_msg())


# ---------------------------------------------------------------------------
# HealthAwareFallbackLLM — stream
# ---------------------------------------------------------------------------


@pytest.mark.asyncio
async def test_stream_switches_on_quota_and_marks_dead() -> None:
    primary = _FakeProvider("minimax", fail=RateLimitError(_QUOTA_MSG, provider="minimax"))
    fallback = _FakeProvider("deepseek")
    cache = HealthCache()
    wrapper = HealthAwareFallbackLLM([primary, fallback], cache=cache)

    chunks = [chunk async for chunk in wrapper.stream(_msg())]

    assert [c.content_delta for c in chunks] == ["from-deepseek"]
    assert primary.stream_calls == 1
    assert cache.is_unavailable("minimax") is True


@pytest.mark.asyncio
async def test_stream_skips_premarked_unavailable_primary() -> None:
    primary = _FakeProvider("minimax")
    fallback = _FakeProvider("deepseek")
    cache = HealthCache()
    cache.mark_unavailable("minimax", reason="quota 2056")
    wrapper = HealthAwareFallbackLLM([primary, fallback], cache=cache)

    chunks = [chunk async for chunk in wrapper.stream(_msg())]

    assert [c.content_delta for c in chunks] == ["from-deepseek"]
    assert primary.stream_calls == 0


# ---------------------------------------------------------------------------
# misc
# ---------------------------------------------------------------------------


@pytest.mark.asyncio
async def test_aclose_closes_all_providers() -> None:
    primary = _FakeProvider("minimax")
    fallback = _FakeProvider("deepseek")
    wrapper = HealthAwareFallbackLLM([primary, fallback])

    await wrapper.aclose()

    assert primary.closed is True
    assert fallback.closed is True


def test_empty_provider_list_rejected() -> None:
    with pytest.raises(ValueError):
        HealthAwareFallbackLLM([])


def test_capabilities_forward_from_primary() -> None:
    primary = _FakeProvider("minimax")
    fallback = _FakeProvider("deepseek")
    wrapper = HealthAwareFallbackLLM([primary, fallback])
    # Default LLMProvider.capabilities is text-only.
    assert wrapper.capabilities.text is True


async def _balance(value: float) -> float:
    return value
