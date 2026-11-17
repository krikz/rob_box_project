"""HTTP-провайдер System One (Jev / Laya-через-Ollaya) на httpx.MockTransport.

Сети нет: транспорт подменяется. Проверяем wire-контракт (по исходникам
typesafe-sdk-python 0.7.2), маппинг ошибок и то, что ключ не утекает.
"""

from __future__ import annotations

import asyncio
import json
import logging

import pytest

httpx = pytest.importorskip("httpx")

from rob_box_harness.decision.orchestrator import DecisionOrchestrator  # noqa: E402
from rob_box_harness.decision.routing import ClassRoute, RoutingConfig  # noqa: E402
from rob_box_harness.decision.systemone_http import (  # noqa: E402
    JEV_DEFAULT_MODEL,
    SystemOneHttpProvider,
    build_request,
    jev_from_env,
    laya_from_env,
)
from rob_box_harness.decision.types import (  # noqa: E402
    ChoiceAnswer,
    ChoiceQuestion,
    DecisionClass,
    NoulAnswer,
    NoulQuestion,
    ProviderAuthError,
    ProviderConfigError,
    ProviderInvalidResponse,
    ProviderRateLimited,
    ProviderTimeout,
    ProviderUnavailable,
    ScoreQuestion,
)

SECRET = "sk-test-DO-NOT-LOG-0123456789"

QUESTIONS = {
    "stop": NoulQuestion("The robot must stop now."),
    "action": ChoiceQuestion("Next action?", ("go", "dock", "wait")),
    "risk": ScoreQuestion("Collision risk", ("none", "low", "high")),
}

# Пример ответа по схеме SDK (сконструирован, НЕ получен от сервера).
RESPONSE = {
    "model": "jev-1.13.0",
    "answers": {
        "stop": {"type": "noul", "noul": 0.91},
        "action": {
            "type": "choice",
            "choice": "wait",
            "confidence": 0.8,
            "probabilities": {"go": 0.05, "dock": 0.15, "wait": 0.8},
        },
        "risk": {
            "type": "score",
            "score": 1.7,
            "confidence": 0.75,
            "legend": {"0": "none", "1": "low", "2": "high"},
            "probabilities": {"0": 0.05, "1": 0.2, "2": 0.75},
        },
    },
    "usage": {"input_tokens": 120, "output_tokens": 12},
}


def _provider(handler, *, name="jev", api_key_env="JEV_API_KEY"):
    def factory(timeout_s):
        return httpx.AsyncClient(transport=httpx.MockTransport(handler), timeout=timeout_s)

    return SystemOneHttpProvider(
        name,
        base_url="https://api.test/",
        model=JEV_DEFAULT_MODEL,
        api_key_env=api_key_env,
        client_factory=factory,
    )


def test_build_request_matches_sdk_wire_shape():
    body = build_request("jev-1.13.0", {"battery_pct": 14}, QUESTIONS)
    assert body == {
        "model": "jev-1.13.0",
        "state": {"battery_pct": 14},
        "questions": {
            "stop": {"type": "noul", "instructions": "The robot must stop now."},
            "action": {
                "type": "choice",
                "instructions": "Next action?",
                "criteria": {"go": None, "dock": None, "wait": None},
            },
            "risk": {"type": "score", "instructions": "Collision risk", "criteria": ["none", "low", "high"]},
        },
    }


def test_decide_parses_response(monkeypatch):
    monkeypatch.setenv("JEV_API_KEY", SECRET)
    seen = {}

    def handler(request):
        seen["url"] = str(request.url)
        seen["auth"] = request.headers.get("authorization")
        seen["body"] = json.loads(request.content)
        return httpx.Response(200, json=RESPONSE)

    decision = asyncio.run(_provider(handler).decide({"battery_pct": 14}, QUESTIONS))
    assert seen["url"] == "https://api.test/v1/systemone"
    assert seen["auth"] == f"Bearer {SECRET}"
    assert seen["body"]["model"] == JEV_DEFAULT_MODEL
    assert decision.model == "jev-1.13.0"
    assert decision.answers["stop"] == NoulAnswer(0.91)
    assert decision.answers["action"].answer == "wait"
    risk = decision.answers["risk"]
    assert isinstance(risk, ChoiceAnswer)
    assert risk.answer == "high" and risk.probabilities == {"none": 0.05, "low": 0.2, "high": 0.75}
    assert decision.usage == {"input_tokens": 120, "output_tokens": 12}


def test_laya_via_ollaya_sends_no_auth_header():
    seen = {}

    def handler(request):
        seen["auth"] = request.headers.get("authorization")
        return httpx.Response(200, json=RESPONSE)

    asyncio.run(_provider(handler, name="laya", api_key_env=None).decide("state", QUESTIONS))
    assert seen["auth"] is None


@pytest.mark.parametrize(
    "status,error",
    [
        (401, ProviderAuthError),
        (403, ProviderAuthError),
        (429, ProviderRateLimited),
        (404, ProviderConfigError),
        (422, ProviderConfigError),
        (500, ProviderUnavailable),
        (503, ProviderUnavailable),
    ],
)
def test_http_status_mapping(monkeypatch, status, error):
    monkeypatch.setenv("JEV_API_KEY", SECRET)
    provider = _provider(lambda request: httpx.Response(status, json={"detail": "x"}))
    with pytest.raises(error) as info:
        asyncio.run(provider.decide("s", QUESTIONS))
    assert SECRET not in str(info.value)


@pytest.mark.parametrize(
    "exc,error",
    [(httpx.ConnectError("dns"), ProviderUnavailable), (httpx.ReadTimeout("slow"), ProviderTimeout)],
)
def test_transport_errors(monkeypatch, exc, error):
    monkeypatch.setenv("JEV_API_KEY", SECRET)

    def handler(request):
        raise exc

    with pytest.raises(error):
        asyncio.run(_provider(handler).decide("s", QUESTIONS))


@pytest.mark.parametrize(
    "payload",
    [
        {"model": "jev", "answers": {}},
        {"model": "jev"},
        {"model": "jev", "answers": {**RESPONSE["answers"], "action": {"type": "choice", "choice": "wait"}}},
        {
            "model": "jev",
            "answers": {**RESPONSE["answers"], "risk": {**RESPONSE["answers"]["risk"], "probabilities": {"7": 1.0}}},
        },
        [1, 2, 3],
    ],
)
def test_malformed_responses_raise_invalid(monkeypatch, payload):
    monkeypatch.setenv("JEV_API_KEY", SECRET)
    with pytest.raises(ProviderInvalidResponse):
        asyncio.run(_provider(lambda r: httpx.Response(200, json=payload)).decide("s", QUESTIONS))


def test_non_json_body_is_invalid(monkeypatch):
    monkeypatch.setenv("JEV_API_KEY", SECRET)
    with pytest.raises(ProviderInvalidResponse):
        asyncio.run(_provider(lambda r: httpx.Response(200, text="<html>")).decide("s", QUESTIONS))


def test_missing_key_is_config_error_without_network(monkeypatch):
    monkeypatch.delenv("JEV_API_KEY", raising=False)
    calls = []

    def handler(request):
        calls.append(request)
        return httpx.Response(200, json=RESPONSE)

    with pytest.raises(ProviderConfigError):
        asyncio.run(_provider(handler).decide("s", QUESTIONS))
    assert calls == []


def test_list_models(monkeypatch):
    monkeypatch.setenv("JEV_API_KEY", SECRET)
    body = {"models": [{"name": "jev-latest", "description": "", "release_date": "2026-09-15"}]}
    provider = _provider(lambda r: httpx.Response(200, json=body))
    assert asyncio.run(provider.list_models()) == ["jev-latest"]


def test_env_factories_are_optional():
    assert jev_from_env({}) is None
    assert laya_from_env({}) is None
    jev = jev_from_env({"JEV_API_KEY": SECRET})
    assert jev is not None and jev.model == JEV_DEFAULT_MODEL and jev.base_url == "https://api.typesafe.ai"
    laya = laya_from_env({"LAYA_BASE_URL": "http://127.0.0.1:11435/"})
    assert laya is not None and laya.base_url == "http://127.0.0.1:11435" and laya.model == "laya"


def test_orchestrator_with_http_errors_never_logs_key(monkeypatch, caplog):
    """E2E офлайн: 500 от Jev → fallback, ключ нигде в логах."""
    monkeypatch.setenv("JEV_API_KEY", SECRET)
    provider = _provider(lambda r: httpx.Response(500, text=f"echo {r.headers.get('authorization')}"))
    route = ClassRoute(("jev",), 500, 0.5)
    orch = DecisionOrchestrator({"jev": provider}, routing=RoutingConfig({c: route for c in DecisionClass}))
    fallback = {
        "stop": NoulAnswer(1.0),
        "action": ChoiceAnswer("wait", {"wait": 1.0}, 1.0),
        "risk": ChoiceAnswer("high", {"high": 1.0}, 1.0),
    }
    with caplog.at_level(logging.DEBUG):
        decision = asyncio.run(orch.decide(DecisionClass.SAFETY_CRITICAL, "s", QUESTIONS, fallback=lambda: fallback))
    assert decision.fallback_used and decision.fallback_reason == "jev_error"
    assert SECRET not in caplog.text
