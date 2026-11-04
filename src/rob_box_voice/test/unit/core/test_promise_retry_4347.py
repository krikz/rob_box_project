"""Повтор моста судят те же гарды обещаний, что и первую попытку (#4347, М10 n604).

Лог n604: ``verdict=invalid:music_promise`` на первой попытке,
``retry=1 violations=['music_promise'] → verdict=command`` на повторе. Здесь: повтор с обещанием из таблицы снова
невалиден - гард на повторе работает; n604 прошёл, потому что слова повтора («рок будет») - не форма обещания, а
смену темы по словам человека исполняет ход (``theme_switch``).
"""

from __future__ import annotations

import asyncio

import pytest

from rob_box_dialog import knowledge as kn
from rob_box_llm.provider import LLMMessage, LLMResponse, ToolCall
from rob_box_llm.providers.deepseek import DeepSeekProvider
from rob_box_voice.core import command_bridge as cb

CAPS = DeepSeekProvider(api_key="test").capabilities
TOOLS = [{"type": "function",
          "function": {"name": name, "description": name, "parameters": {"type": "object", "properties": {}}}}
         for name in ("dj_set", "request_music", "play_animation", "set_voice", "set_speed", "web_search")]


class _Inner:
    def __init__(self, replies):
        self.name, self.model, self._replies, self.requests = "deepseek", "deepseek-model", list(replies), []

    @property
    def capabilities(self):
        return CAPS

    def capabilities_for(self, model):
        return CAPS

    async def complete(self, messages, *, tools=(), settings=None):
        self.requests.append(list(messages))
        return self._replies.pop(0)


def _speech(text: str) -> LLMResponse:
    return LLMResponse(tool_calls=(ToolCall(id="c1", name=kn.COMMAND_TOOL, arguments={"speech": text}),))


@pytest.mark.parametrize("first, again, reason", [
    ("Понял: сворачиваю в рок.", "Переключаю на рок, держись!", kn.VIOLATION_MUSIC_PROMISE),
    ("Заговорю помедленнее.", "Хорошо, заговорю помедленнее.", kn.VIOLATION_VOICE_PROMISE),
    ("Поищу ещё раз.", "Сейчас поищу ещё раз.", kn.VIOLATION_SEARCH_PROMISE),
])
def test_the_retry_is_judged_by_the_same_promise_guards(first: str, again: str, reason: str) -> None:
    log: list = []
    inner = _Inner([_speech(first), _speech(again)])
    msgs = [LLMMessage(role="system", content="ты РОББОКС"), LLMMessage(role="user", content="уводи в сторону рока")]
    out = asyncio.run(cb.CommandModeLLM(inner, log.append).complete(msgs, tools=TOOLS))
    (call,) = out.tool_calls
    assert len(inner.requests) == 2 and out.content == ""
    assert call.arguments[kn.COMMAND_INVALID_FIELD] == reason
    assert f"retry=1 violations=['{reason}'] → verdict=invalid:{reason}" in log[2]
