"""Спорный стоп (ADR-0150 В19, #3696): Chooser ``{stop, part_of_command}`` поверх общего ``ModelChooser``.

Провайдер — настоящий ``ChatChoiceProvider`` decision layer на ``httpx.MockTransport``: проверяется весь путь,
которым пойдёт робот (строгая схема с ``enum``, дедлайн, ответ вне перечня), а не заглушка ``choose``.
"""

from __future__ import annotations

import json

import httpx
import pytest

from rob_box_harness.decision.chat_choice import ChatChoiceProvider
from rob_box_mcp_tools.engine.chooser_model import ModelChooser

from rob_box_voice.core.stop_chooser import (
    PART_OF_COMMAND,
    STOP,
    StopChooser,
    question,
    state_text,
    stop_chooser_from_env,
)


@pytest.fixture(autouse=True)
def key(monkeypatch):
    monkeypatch.setenv("LLM_HARNESS_API_KEY", "sk-test-DO-NOT-LOG")


class _Log:
    def __init__(self):
        self.lines = []

    def info(self, text):
        self.lines.append(("info", text))

    def warning(self, text):
        self.lines.append(("warning", text))


def _chooser(handler, log):
    client = lambda timeout_s: httpx.AsyncClient(transport=httpx.MockTransport(handler), timeout=timeout_s)  # noqa: E731
    provider = ChatChoiceProvider(base_url="https://llm.test/v1/", model="m-1", client_factory=client)
    return StopChooser(ModelChooser(provider, deadline_ms=1000, state=state_text), log=lambda: log)


def _answer(choice):
    content = json.dumps({"choice": choice, "confidence": 0.8})
    return httpx.Response(200, json={"choices": [{"message": {"content": content}}]})


@pytest.mark.parametrize("choice", [STOP, PART_OF_COMMAND])
def test_model_answer_is_the_verdict(choice):
    seen = {}

    def handler(request):
        seen["body"] = json.loads(request.content)
        return _answer(choice)

    log = _Log()
    assert _chooser(handler, log).decide("Робот, спой песню про стоп") == (choice, "model")
    body = seen["body"]
    assert body["response_format"]["json_schema"]["schema"]["properties"]["choice"]["enum"] == [STOP, PART_OF_COMMAND]
    user = body["messages"][1]["content"]
    assert "спой песню про стоп" in user and "очки кода" not in user and "сыграно" not in user
    assert any("[stop-chooser] verdict=" + choice in text for _, text in log.lines)


def test_answer_outside_the_choices_is_part_of_command():
    log = _Log()
    verdict, stage = _chooser(lambda request: _answer("mute"), log).decide("покажи анимацию стоп")
    assert verdict == PART_OF_COMMAND and stage.startswith("timeout")


def test_provider_failure_is_part_of_command():
    def handler(request):
        raise httpx.ReadTimeout("slow", request=request)

    verdict, stage = _chooser(handler, _Log()).decide("найди песню хватит")
    assert verdict == PART_OF_COMMAND and stage.startswith("timeout")


def test_without_model_robot_does_not_go_silent():
    log = _Log()
    assert StopChooser(None, log=lambda: log).decide("спой про стоп") == (PART_OF_COMMAND, "off")


def test_from_env_without_key_is_off_with_one_log_line(monkeypatch):
    for name in ("LLM_HARNESS_API_KEY", "LLM_HARNESS_API_KEY_ENV", "DEEPSEEK_API_KEY", "LLM_HARNESS_PROVIDER"):
        monkeypatch.delenv(name, raising=False)
    log = _Log()
    chooser = stop_chooser_from_env(env={}, log=lambda: log)
    assert chooser.decide("спой про стоп")[1] == "off"
    assert [level for level, _ in log.lines].count("warning") == 1


def test_question_has_two_labelled_choices_and_no_weights():
    q = question("покажи анимацию стоп")
    assert q.ids() == (STOP, PART_OF_COMMAND) and dict(q.weights) == {}
    assert all(o.label for o in q.options) and "part_of_command" in q.instructions
