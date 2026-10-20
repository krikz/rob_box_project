"""ProviderStateReporter: публикует изменения и озвучивает отказ провайдера одной фразой (#3693)."""

from __future__ import annotations

from rob_box_harness.health import HealthAwareFallbackLLM, HealthCache
from rob_box_llm.errors import RateLimitError, TimeoutError as LLMTimeoutError
from rob_box_voice.core.provider_reporter import ProviderStateReporter


class _P:
    def __init__(self, name):
        self.name = name
        self.capabilities = None


def _make(now):
    cache = HealthCache(ttl_s=300.0, clock=lambda: now[0])
    llm = HealthAwareFallbackLLM([_P("minimax"), _P("deepseek")], cache=cache)
    published, spoken = [], []
    rep = ProviderStateReporter(llm, published.append, spoken.append, clock=lambda: now[0])
    return llm, rep, published, spoken


def test_healthy_chain_publishes_once_and_stays_silent():
    now = [0.0]
    _, rep, published, spoken = _make(now)
    rep.tick()
    rep.tick()
    assert len(published) == 1 and spoken == []


def test_quota_is_published_and_spoken_once():
    now = [0.0]
    llm, rep, published, spoken = _make(now)
    rep.tick()
    llm._handle_failure("minimax", RateLimitError("429 Token Plan usage limit reached (2056)", provider="minimax"))
    rep.tick()
    rep.tick()
    assert [s.health for s in published[-1]] == ["quota", "ok"]
    assert len(spoken) == 1
    assert "minimax" in spoken[0] and "кончились деньги" in spoken[0] and "deepseek" in spoken[0]


def test_second_failure_within_ten_minutes_is_silent():
    now = [0.0]
    llm, rep, _, spoken = _make(now)
    rep.tick()
    llm._handle_failure("minimax", LLMTimeoutError("t"))
    rep.tick()
    now[0] = 400.0  # TTL истёк - ok; новый отказ, но < 10 мин от первой фразы
    rep.tick()
    llm._handle_failure("minimax", LLMTimeoutError("t"))
    rep.tick()
    assert len(spoken) == 1


def test_single_provider_without_snapshot_is_disabled():
    published, spoken = [], []
    rep = ProviderStateReporter(object(), published.append, spoken.append)
    rep.tick()
    assert not rep.enabled and published == [] and spoken == []
