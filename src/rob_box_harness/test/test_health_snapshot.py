"""HealthAwareFallbackLLM.snapshot / HealthRecord.health_class: класс причины для ProviderState (ADR-0150 §8, #3693)."""

from __future__ import annotations

from rob_box_harness.health import HealthAwareFallbackLLM, HealthCache
from rob_box_llm.errors import AuthError, RateLimitError, TimeoutError as LLMTimeoutError


class _P:
    def __init__(self, name: str) -> None:
        self.name = name
        self.capabilities = None


def _llm(clock):
    cache = HealthCache(ttl_s=300.0, clock=clock)
    return HealthAwareFallbackLLM([_P("minimax"), _P("deepseek")], cache=cache), cache


def test_fresh_chain_is_all_ok():
    llm, _ = _llm(lambda: 100.0)
    assert [(s["name"], s["health"]) for s in llm.snapshot()] == [("minimax", "ok"), ("deepseek", "ok")]


def test_failures_map_to_health_classes():
    llm, _ = _llm(lambda: 100.0)
    llm._handle_failure("minimax", RateLimitError("429 Token Plan usage limit reached (2056)", provider="minimax"))
    llm._handle_failure("deepseek", AuthError("401 invalid api key"))
    snap = {s["name"]: s for s in llm.snapshot()}
    assert snap["minimax"]["health"] == "quota" and snap["minimax"]["since"] == 100.0
    assert snap["deepseek"]["health"] == "auth"


def test_timeout_and_burst_rate_limit_classes():
    llm, _ = _llm(lambda: 100.0)
    llm._handle_failure("minimax", LLMTimeoutError("timed out"))
    llm._handle_failure("deepseek", RateLimitError("429: rate-limited", provider="deepseek"))
    snap = {s["name"]: s["health"] for s in llm.snapshot()}
    assert snap == {"minimax": "timeout", "deepseek": "degraded"}


def test_expired_record_returns_to_ok():
    now = [100.0]
    llm, _ = _llm(lambda: now[0])
    llm._handle_failure("minimax", LLMTimeoutError("timed out"))
    now[0] += 10_000.0
    assert llm.snapshot()[0]["health"] == "ok"


def test_health_class_survives_restart(tmp_path):
    path = tmp_path / "h.json"
    HealthCache(persist_path=path, clock=lambda: 100.0).mark_unavailable("minimax", reason="x", health_class="quota")
    assert HealthCache(persist_path=path, clock=lambda: 101.0).get("minimax").health_class == "quota"


def test_minimax_adapter_turns_vendor_codes_into_quota_class():
    from rob_box_harness.health import is_quota_exhausted
    from rob_box_harness.providers.minimax import _classify_quota
    from rob_box_llm.errors import QuotaExhaustedError

    for text in ("base_resp 2056", "code 1008", "your current token plan"):
        out = _classify_quota(RateLimitError(text, provider="minimax"))
        assert isinstance(out, QuotaExhaustedError) and is_quota_exhausted(out)
    burst = RateLimitError("429: rate-limited", provider="minimax")
    assert _classify_quota(burst) is burst and not is_quota_exhausted(burst)
