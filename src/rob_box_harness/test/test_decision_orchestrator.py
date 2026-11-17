"""Офлайн-тесты decision layer (issue #3084). Сеть не используется.

Главный контракт: один и тот же сценарий при провайдере доступном /
в таймауте / недоступном / с невалидным ответом / с circuit open даёт
валидное решение, а при любой неудаче — ровно детерминированный fallback.
"""

from __future__ import annotations

import asyncio
import math

import pytest

from rob_box_harness.decision.health import CircuitBreaker, CircuitState, DecisionMetrics
from rob_box_harness.decision.orchestrator import DecisionOrchestrator, answer_confidence
from rob_box_harness.decision.providers import DecisionProvider, DeterministicProvider, FakeDecisionProvider
from rob_box_harness.decision.routing import ClassRoute, RoutingConfig
from rob_box_harness.decision.types import (
    ChoiceAnswer,
    ChoiceQuestion,
    DecisionClass,
    NoulAnswer,
    NoulQuestion,
    ProviderAuthError,
    ProviderInvalidResponse,
    ProviderRateLimited,
    ProviderUnavailable,
    ScoreQuestion,
    validate_answers,
)

QUESTIONS = {
    "action": ChoiceQuestion("Что сделать с новой фразой?", ("ignore", "replace", "pending_llm")),
    "needs_review": NoulQuestion("Нужно ли подтверждение человека?"),
}

MODEL_ANSWERS = {
    "action": ChoiceAnswer("replace", {"ignore": 0.02, "replace": 0.95, "pending_llm": 0.03}, 0.95),
    "needs_review": NoulAnswer(0.03),
}

FALLBACK_ANSWERS = {
    "action": ChoiceAnswer("pending_llm", {"pending_llm": 1.0}, 1.0),
    "needs_review": NoulAnswer(1.0),
}

STATE = {"utterance": "стоп", "active_task": "music"}


def _fallback():
    return FALLBACK_ANSWERS


def _routing(providers=("jev",), deadline_ms=200, min_confidence=0.8):
    route = ClassRoute(tuple(providers), deadline_ms, min_confidence)
    return RoutingConfig({c: route for c in DecisionClass})


def _run(orch, decision_class=DecisionClass.QUALITY_CRITICAL):
    return asyncio.run(orch.decide(decision_class, STATE, QUESTIONS, fallback=_fallback))


# --- та же сцена, 5 состояний провайдера -------------------------------------

FAILURE_MODES = {
    "timeout": FakeDecisionProvider("jev", answers=MODEL_ANSWERS, delay_s=5.0),
    "unreachable": FakeDecisionProvider("jev", error=ProviderUnavailable("connect refused")),
    "auth": FakeDecisionProvider("jev", error=ProviderAuthError("401")),
    "rate_limited": FakeDecisionProvider("jev", error=ProviderRateLimited("429")),
    "invalid_option": FakeDecisionProvider(
        "jev",
        answers={**MODEL_ANSWERS, "action": ChoiceAnswer("drive_off_cliff", {"drive_off_cliff": 1.0}, 1.0)},
    ),
    "missing_answer": FakeDecisionProvider("jev", answers={"action": MODEL_ANSWERS["action"]}),
    "nan_probability": FakeDecisionProvider("jev", answers={**MODEL_ANSWERS, "needs_review": NoulAnswer(math.nan)}),
    "client_crash": FakeDecisionProvider("jev", error=RuntimeError("bug in client")),
}

EXPECTED_LABEL = {
    "timeout": "jev_timeout",
    "unreachable": "jev_error",
    "auth": "jev_error",
    "rate_limited": "jev_error",
    "invalid_option": "jev_invalid",
    "missing_answer": "jev_invalid",
    "nan_probability": "jev_invalid",
    "client_crash": "jev_error",
}


def test_available_provider_answer_is_used():
    orch = DecisionOrchestrator({"jev": FakeDecisionProvider("jev", answers=MODEL_ANSWERS)}, routing=_routing())
    decision = _run(orch)
    assert decision.provider == "jev"
    assert decision.fallback_used is False
    assert decision.answers["action"].answer == "replace"
    assert decision.latency_ms is not None
    assert orch.metrics.counters() == {"jev_success": 1}


@pytest.mark.parametrize("mode", sorted(FAILURE_MODES))
def test_failure_modes_fall_back_to_deterministic(mode):
    orch = DecisionOrchestrator({"jev": FAILURE_MODES[mode]}, routing=_routing(deadline_ms=100))
    decision = _run(orch)
    assert decision.provider == "deterministic"
    assert decision.fallback_used is True
    assert decision.answers == FALLBACK_ANSWERS
    counters = orch.metrics.counters()
    assert counters[EXPECTED_LABEL[mode]] == 1
    assert counters["fallback_used"] == 1


def test_timeout_stops_waiting_at_deadline():
    slow = FakeDecisionProvider("jev", answers=MODEL_ANSWERS, delay_s=10.0)
    orch = DecisionOrchestrator({"jev": slow}, routing=_routing(deadline_ms=50))

    async def timed():
        loop = asyncio.get_running_loop()
        started = loop.time()
        decision = await orch.decide(DecisionClass.QUALITY_CRITICAL, STATE, QUESTIONS, fallback=_fallback)
        return decision, loop.time() - started

    decision, elapsed = asyncio.run(timed())
    assert decision.fallback_reason == "jev_timeout"
    assert elapsed < 0.5, f"orchestrator waited {elapsed:.3f}s past a 50ms deadline"


def test_no_provider_registered_uses_fallback():
    orch = DecisionOrchestrator({}, routing=_routing())
    decision = _run(orch)
    assert decision.fallback_used and decision.fallback_reason == "jev_unconfigured"
    assert orch.metrics.counters() == {"jev_unconfigured": 1, "fallback_used": 1}


def test_low_confidence_falls_back():
    unsure = {
        "action": ChoiceAnswer("replace", {"ignore": 0.3, "replace": 0.4, "pending_llm": 0.3}, 0.4),
        "needs_review": NoulAnswer(0.03),
    }
    orch = DecisionOrchestrator({"jev": FakeDecisionProvider("jev", answers=unsure)}, routing=_routing())
    decision = _run(orch)
    assert decision.fallback_reason == "jev_low_confidence"


def test_noul_near_half_counts_as_low_confidence():
    assert answer_confidence(NoulAnswer(0.55)) == pytest.approx(0.55)
    assert answer_confidence(NoulAnswer(0.02)) == pytest.approx(0.98)


def test_chain_moves_to_next_provider():
    jev = FakeDecisionProvider("jev", error=ProviderUnavailable("dns"))
    laya = FakeDecisionProvider("laya", answers=MODEL_ANSWERS)
    orch = DecisionOrchestrator({"jev": jev, "laya": laya}, routing=_routing(("jev", "laya")))
    decision = _run(orch)
    assert decision.provider == "laya"
    assert orch.metrics.counters() == {"jev_error": 1, "laya_success": 1}


def test_deadline_is_shared_across_chain():
    jev = FakeDecisionProvider("jev", answers=MODEL_ANSWERS, delay_s=10.0)
    laya = FakeDecisionProvider("laya", answers=MODEL_ANSWERS, delay_s=10.0)
    orch = DecisionOrchestrator({"jev": jev, "laya": laya}, routing=_routing(("jev", "laya"), deadline_ms=60))

    async def timed():
        loop = asyncio.get_running_loop()
        started = loop.time()
        decision = await orch.decide(DecisionClass.QUALITY_CRITICAL, STATE, QUESTIONS, fallback=_fallback)
        return decision, loop.time() - started

    decision, elapsed = asyncio.run(timed())
    assert decision.fallback_used
    assert elapsed < 0.5


def test_circuit_opens_and_skips_provider():
    now = [0.0]
    broken = FakeDecisionProvider("jev", error=ProviderUnavailable("down"))
    orch = DecisionOrchestrator(
        {"jev": broken},
        routing=_routing(),
        breaker_factory=lambda: CircuitBreaker(failure_threshold=2, reset_after_s=30, clock=lambda: now[0]),
    )
    for _ in range(4):
        _run(orch)
    assert broken.calls == 2, "after 2 failures the circuit must stop calling the provider"
    assert orch.breaker("jev").state is CircuitState.OPEN
    assert orch.metrics.counters()["jev_circuit_open"] == 2
    now[0] = 31.0
    assert orch.breaker("jev").state is CircuitState.HALF_OPEN
    _run(orch)
    assert broken.calls == 3
    assert orch.breaker("jev").state is CircuitState.OPEN


def test_circuit_closes_after_success():
    breaker = CircuitBreaker(failure_threshold=1, reset_after_s=0.0, clock=lambda: 0.0)
    breaker.record_failure()
    assert breaker.state is CircuitState.HALF_OPEN
    breaker.record_success()
    assert breaker.state is CircuitState.CLOSED


def test_runtime_provider_switch_without_restart():
    jev = FakeDecisionProvider("jev", answers=MODEL_ANSWERS)
    laya = FakeDecisionProvider("laya", answers=MODEL_ANSWERS)
    orch = DecisionOrchestrator({"jev": jev, "laya": laya}, routing=_routing(("jev",)))
    assert _run(orch).provider == "jev"
    orch.set_routing(_routing(("laya",)))
    assert _run(orch).provider == "laya"
    orch.unregister("laya")
    assert _run(orch).provider == "deterministic"


def test_equivalent_safe_behaviour_across_providers():
    """Тот же сценарий через jev / laya / deterministic → одинаковое решение."""
    rules = DeterministicProvider(lambda state, questions: MODEL_ANSWERS)
    results = []
    for name in ("jev", "laya"):
        orch = DecisionOrchestrator(
            {name: FakeDecisionProvider(name, answers=MODEL_ANSWERS)}, routing=_routing((name,))
        )
        results.append(_run(orch).answers)
    results.append(asyncio.run(rules.decide(STATE, QUESTIONS)).answers)
    assert results[0] == results[1] == results[2]


def test_fakes_satisfy_protocol():
    assert isinstance(FakeDecisionProvider("jev"), DecisionProvider)
    assert isinstance(DeterministicProvider(lambda s, q: {}), DecisionProvider)


def test_deterministic_cannot_be_registered_as_remote():
    orch = DecisionOrchestrator()
    with pytest.raises(ValueError):
        orch.register(FakeDecisionProvider("deterministic"))


def test_broken_fallback_fails_loud():
    orch = DecisionOrchestrator({}, routing=_routing())
    with pytest.raises(ProviderInvalidResponse):
        asyncio.run(orch.decide(DecisionClass.QUALITY_CRITICAL, STATE, QUESTIONS, fallback=lambda: {}))


# --- validation -------------------------------------------------------------


def test_validate_score_levels():
    questions = {"risk": ScoreQuestion("Насколько рискованно?", ("low", "medium", "high"))}
    ok = {"risk": ChoiceAnswer("high", {"low": 0.1, "medium": 0.1, "high": 0.8}, 0.8)}
    validate_answers(questions, ok)
    bad_sum = {"risk": ChoiceAnswer("high", {"low": 0.5, "high": 0.8}, 0.8)}
    with pytest.raises(ProviderInvalidResponse):
        validate_answers(questions, bad_sum)


def test_question_rejects_duplicate_options():
    with pytest.raises(ValueError):
        ChoiceQuestion("x", ("a", "a"))
    with pytest.raises(TypeError):
        ChoiceQuestion("x", ["a", "b"])  # type: ignore[arg-type]


# --- metrics ----------------------------------------------------------------


def test_latency_percentiles():
    metrics = DecisionMetrics()
    for value in range(1, 101):
        metrics.observe_latency("jev", float(value))
    summary = metrics.latency("jev")
    assert (summary.count, summary.p50_ms, summary.p95_ms) == (100, 50.0, 95.0)
    assert metrics.latency("laya").count == 0
