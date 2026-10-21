"""#3798: DeepSeek V4 выключает размышления только полем ``thinking: {"type": "disabled"}``.

Замер API 21.10 (из контейнера voice-assistant): ``enable_thinking: false`` модель не выключает — в ответе есть
``reasoning_content``, ``tool_choice="required"`` получает 400 «Thinking mode does not support this tool_choice», а
второй запрос хода после тула — 400 «reasoning_content must be passed back». С ``thinking: disabled`` все три
запроса проходят. Тест — на тело запроса, которое уходит в API.
"""

from __future__ import annotations

import asyncio
import json
from types import SimpleNamespace
from typing import Any

from rob_box_llm.provider import LLMMessage, LLMSettings, ToolCall
from rob_box_llm.providers.deepseek import DeepSeekProvider, _OpenAICompatibleProvider

TOOL = {"type": "function", "function": {"name": "command", "description": "команда",
                                         "parameters": {"type": "object", "properties": {}}}}


class _Completions:
    def __init__(self) -> None:
        self.calls: list[dict[str, Any]] = []

    async def create(self, **kwargs: Any) -> Any:
        self.calls.append(kwargs)
        message = SimpleNamespace(content="ок", tool_calls=None)
        return SimpleNamespace(choices=[SimpleNamespace(message=message, finish_reason="stop")], usage=None)


def _sent(provider_cls: Any, messages: list, settings: LLMSettings | None = None, tools: list | None = None) -> dict:
    completions = _Completions()
    client = SimpleNamespace(chat=SimpleNamespace(completions=completions))
    provider = provider_cls(api_key="sk-test", client=client)
    asyncio.run(provider.complete(messages, tools=tools or [], settings=settings))
    return completions.calls[0]


def test_every_deepseek_request_disables_thinking_with_the_field_the_api_honours() -> None:
    sent = _sent(DeepSeekProvider, [LLMMessage(role="user", content="привет")])
    assert sent["extra_body"] == {"thinking": {"type": "disabled"}}
    assert "enable_thinking" not in sent["extra_body"]  # API его не слушает (#3798)
    json.dumps(sent["extra_body"])  # тело сериализуемо: SDK шлёт extra_body как JSON


def test_command_mode_request_keeps_required_tool_choice_and_disabled_thinking() -> None:
    sent = _sent(DeepSeekProvider, [LLMMessage(role="user", content="привет")],
                 LLMSettings(tool_choice="required", extra={"thinking": None}), tools=[TOOL])
    assert sent["tool_choice"] == "required"
    assert sent["extra_body"]["thinking"] == {"type": "disabled"}  # настройки хода размышления не включают


def test_second_request_of_a_turn_after_a_tool_also_disables_thinking() -> None:
    history = [
        LLMMessage(role="user", content="включи звук"),
        LLMMessage(role="assistant", content="", tool_calls=(ToolCall(id="c1", name="play_sound", arguments={}),)),
        LLMMessage(role="tool", content="done", tool_call_id="c1"),
    ]
    assert _sent(DeepSeekProvider, history)["extra_body"]["thinking"] == {"type": "disabled"}


def _other(**kwargs: Any) -> _OpenAICompatibleProvider:
    return _OpenAICompatibleProvider(name="mimo", base_url="http://x", default_model="m", **kwargs)


def test_other_openai_compatible_adapters_send_no_thinking_field() -> None:
    assert "extra_body" not in _sent(_other, [LLMMessage(role="user", content="привет")])


def test_deepseek_declares_no_thinking_because_it_switches_it_off() -> None:
    caps = DeepSeekProvider(api_key="sk-test").capabilities
    assert caps.tool_choice and not caps.thinking
