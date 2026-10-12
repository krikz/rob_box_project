"""Провайдер ``llm_harness`` (OpenAI-совместимый chat) и ``descriptions`` в criteria Jev (ADR-0157 PR-7, #3564).

Сети нет: ``httpx.MockTransport``.
"""

from __future__ import annotations

import asyncio
import json

import pytest

httpx = pytest.importorskip("httpx")

from rob_box_harness.decision.chat_choice import (  # noqa: E402
    API_KEY_ENV,
    BASE_URL_ENV,
    MODEL_ENV,
    ChatChoiceProvider,
    chat_choice_from_env,
)
from rob_box_harness.decision.systemone_http import build_request  # noqa: E402
from rob_box_harness.decision.types import (  # noqa: E402
    ChoiceQuestion,
    NoulQuestion,
    ProviderAuthError,
    ProviderConfigError,
    ProviderInvalidResponse,
)

SECRET = "sk-test-DO-NOT-LOG-0123456789"
QUESTIONS = {"pick": ChoiceQuestion("Что играть?", ("a", "b"), {"a": "первый", "b": "второй"})}


def provider(handler):
    def factory(timeout_s):
        return httpx.AsyncClient(transport=httpx.MockTransport(handler), timeout=timeout_s)

    return ChatChoiceProvider(base_url="https://llm.test/v1", model="m-1", client_factory=factory)


def completion(content, usage=None):
    body = {"choices": [{"message": {"content": content}}]}
    return {**body, "usage": usage} if usage else body


def test_jev_criteria_carry_descriptions_and_default_to_null():
    body = build_request("jev-1.13.0", "s", {**QUESTIONS, "other": ChoiceQuestion("?", ("x", "y"))})
    assert body["questions"]["pick"]["criteria"] == {"a": "первый", "b": "второй"}
    assert body["questions"]["other"]["criteria"] == {"x": None, "y": None}


def test_decide_builds_strict_schema_request_and_parses_answer_and_usage(monkeypatch):
    monkeypatch.setenv(API_KEY_ENV, SECRET)
    seen = {}

    def handler(request):
        seen.update(auth=request.headers["authorization"], url=str(request.url), body=json.loads(request.content))
        return httpx.Response(200, json=completion('{"choice": "b", "confidence": 0.66}',
                                                   {"prompt_tokens": 100, "completion_tokens": 7}))

    decision = asyncio.run(provider(handler).decide({"recent": ["a"]}, QUESTIONS))
    assert seen["url"] == "https://llm.test/v1/chat/completions" and seen["auth"] == f"Bearer {SECRET}"
    body = seen["body"]
    assert body["temperature"] == 0 and body["response_format"]["json_schema"]["strict"] is True
    assert body["response_format"]["json_schema"]["schema"]["properties"]["choice"]["enum"] == ["a", "b"]
    assert "- a: первый" in body["messages"][1]["content"] and '"recent": ["a"]' in body["messages"][1]["content"]
    answer = decision.answers["pick"]
    assert (answer.answer, answer.confidence) == ("b", 0.66)
    assert decision.provider == "llm_harness" and decision.usage == {"input_tokens": 100, "output_tokens": 7}


@pytest.mark.parametrize("content", ["не JSON", '{"choice": "b"}', '{"confidence": 0.5}', "[]"])
def test_malformed_completion_is_invalid(monkeypatch, content):
    monkeypatch.setenv(API_KEY_ENV, SECRET)
    with pytest.raises(ProviderInvalidResponse):
        asyncio.run(provider(lambda r: httpx.Response(200, json=completion(content))).decide("s", QUESTIONS))


def test_only_choice_questions_and_missing_key_and_auth_error(monkeypatch):
    monkeypatch.delenv(API_KEY_ENV, raising=False)
    with pytest.raises(ProviderConfigError) as err:
        asyncio.run(provider(lambda r: httpx.Response(200)).decide("s", QUESTIONS))
    assert SECRET not in str(err.value)
    monkeypatch.setenv(API_KEY_ENV, SECRET)
    with pytest.raises(ProviderInvalidResponse):
        asyncio.run(provider(lambda r: httpx.Response(200)).decide("s", {"q": NoulQuestion("?")}))
    with pytest.raises(ProviderAuthError):
        asyncio.run(provider(lambda r: httpx.Response(401)).decide("s", QUESTIONS))


def test_from_env_needs_base_url_model_and_key():
    full = {BASE_URL_ENV: "https://llm.test/v1", MODEL_ENV: "m", API_KEY_ENV: SECRET}
    assert chat_choice_from_env(full).name == "llm_harness"
    for missing in full:
        assert chat_choice_from_env({k: v for k, v in full.items() if k != missing}) is None
