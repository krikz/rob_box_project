"""#4138 (n904, 28.10) на мосту режима команд: косметика (``earcon`` не из перечня, лишний аргумент ``play_animation``)
выбрасывается кодом - повтора нет, действие уходит исполнителю, строка лога называет выброшенное (``fields_dropped``).
Без поля, несущего смысл, и для неизвестного тула команда по-прежнему возвращается модели с нарушением."""

from __future__ import annotations

import asyncio
from typing import Any, AsyncIterator, Iterable, List, Mapping, Optional

from rob_box_dialog import knowledge as kn
from rob_box_llm.provider import LLMChunk, LLMMessage, LLMResponse, LLMSettings, ProviderCapabilities, ToolCall
from rob_box_llm.providers.deepseek import DeepSeekProvider
from rob_box_voice.core import command_bridge as cb

DEEPSEEK_CAPS = DeepSeekProvider(api_key="test").capabilities
SAD = {"animation": "sad"}


def _tool(name: str) -> Mapping[str, Any]:
    return {"type": "function", "function": {"name": name, "description": name,
                                             "parameters": {"type": "object", "properties": {}}}}


TOOLS = [_tool(n) for n in ("play_animation", "memory_save", "play_sound")]


class _Inner:
    """Провайдер-двойник: заготовленные ответы по очереди, запросы - в ``requests``."""

    def __init__(self, replies: List[LLMResponse]) -> None:
        self.name, self.model, self._replies, self.requests = "deepseek", "deepseek-model", list(replies), []

    @property
    def capabilities(self) -> ProviderCapabilities:
        return DEEPSEEK_CAPS

    def capabilities_for(self, model: Optional[str]) -> ProviderCapabilities:
        return DEEPSEEK_CAPS

    async def complete(self, messages: Iterable[LLMMessage], *, tools: Iterable[Mapping[str, Any]] = (),
                       settings: Optional[LLMSettings] = None) -> LLMResponse:
        self.requests.append(list(messages))
        return self._replies.pop(0)

    async def stream(self, messages: Iterable[LLMMessage], *, tools: Iterable[Mapping[str, Any]] = (),
                     settings: Optional[LLMSettings] = None) -> AsyncIterator[LLMChunk]:  # pragma: no cover
        raise NotImplementedError

    async def aclose(self) -> None:
        return None


def _reply(**fields: Any) -> LLMResponse:
    return LLMResponse(tool_calls=(ToolCall(id="c1", name=kn.COMMAND_TOOL, arguments=fields),))


def _show(args: Mapping[str, Any] = SAD, **expression: Any) -> LLMResponse:
    return _reply(expression={"emotion": "sad", **expression}, actions=[{"tool": "play_animation", "args": dict(args)}])


def _answer(*replies: LLMResponse) -> tuple:
    inner, log = _Inner(list(replies)), []
    msgs = [LLMMessage(role="system", content="ты РОББОКС"),
            LLMMessage(role="user", content="Робот, покажи, как тебе грустно")]
    out = asyncio.run(cb.CommandModeLLM(inner, log.append).complete(msgs, tools=TOOLS))
    return out, inner.requests, log


def _command_args(out: LLMResponse) -> Mapping[str, Any]:
    (call,) = out.tool_calls
    assert call.name == kn.COMMAND_TOOL
    return call.arguments


def test_4138_earcon_none_is_dropped_with_no_retry_and_the_animation_goes_to_the_executor():
    out, requests, log = _answer(_show(earcon="none"))
    assert len(requests) == 1, "повтора нет: косметика не нарушение"
    args = _command_args(out)
    assert args["actions"] == [{"tool": "play_animation", "args": SAD}]
    assert "earcon" not in args["expression"] and args["expression"]["emotion"] == "sad"
    (line,) = [entry for entry in log if entry.startswith(cb.COMMAND_LOG_MARK)]
    assert "verdict=command" in line and "fields_dropped=['expression.earcon']" in line


def test_4138_extra_animated_argument_is_dropped_with_no_retry_and_never_reaches_the_executor():
    out, requests, log = _answer(_show({**SAD, "animated": True}))
    assert len(requests) == 1
    assert _command_args(out)["actions"] == [{"tool": "play_animation", "args": SAD}]
    (line,) = [entry for entry in log if entry.startswith(cb.COMMAND_LOG_MARK)]
    assert "verdict=command" in line and "fields_dropped=['actions[0].play_animation.animated']" in line


def test_4138_both_n904_answers_in_a_row_no_longer_end_in_the_stub():
    """Живой n904: ответ с ``earcon=none``, затем повтор с ``animated``. Теперь первый же ответ исполняется."""
    out, requests, _ = _answer(_show(earcon="none"), _show({**SAD, "animated": True}))
    assert len(requests) == 1 and kn.COMMAND_INVALID_FIELD not in _command_args(out)


def test_4138_only_an_extra_argument_means_no_meaning_the_model_gets_the_violation_and_fixes_it():
    out, requests, log = _answer(_reply(actions=[{"tool": "play_animation", "args": {"animated": True}}]),
                                 _show())
    assert len(requests) == 2 and "play_animation.animated: лишнее поле" in requests[1][-1].content
    assert _command_args(out)["actions"] == [{"tool": "play_animation", "args": SAD}]
    assert any("retry=1 violations=['bad_args'] → verdict=command" in line for line in log)


def test_4138_unknown_tool_is_still_invalid_after_the_retry():
    wrong = _reply(actions=[{"tool": "no_such_tool", "args": {"animated": True}}])
    out, requests, _ = _answer(wrong, wrong)
    assert len(requests) == 2
    assert _command_args(out)[kn.COMMAND_INVALID_FIELD] == "tool_not_available"
