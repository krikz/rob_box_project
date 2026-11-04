"""#4311 (марафон-9 04.11, n903): подхват бэклога голым «робот» → «а теперь сирену и мигалку покажи, напугаем Валеру».
Модель ответила действиями (``play_sound`` + ``play_animation``) и речью про них — ``speech_with_actions`` (речь при
громких действиях не звучит, §4.1). На повторе она прислала те же действия без речи, но с лишней ``}`` в JSON:
адаптер DeepSeek отдал пустые аргументы ``command`` — ``bad_json``, и ход закрылся «Не понял, как это сделать».

Код решает (ADR-0148): (1) объект, за которым только лишние закрывающие скобки, — целый объект, не ``bad_json``;
(2) повтор снова невалиден и исполнить в нём нечего — ход исполняет допустимые действия отвергнутой команды
(``Invalid.salvage``), её речь отброшена."""

from __future__ import annotations

import asyncio
import json
from typing import Any, AsyncIterator, Iterable, List, Mapping, Optional

from rob_box_dialog import knowledge as kn
from rob_box_dialog.command import Command, Invalid
from rob_box_llm.provider import LLMChunk, LLMMessage, LLMResponse, LLMSettings, ProviderCapabilities, ToolCall
from rob_box_llm.providers import deepseek as ds
from rob_box_voice.core import command_bridge as cb

DEEPSEEK_CAPS = ds.DeepSeekProvider(api_key="test").capabilities
N903_ACTIONS = [{"tool": "play_sound", "args": {"sound": "robot_alert"}},
                {"tool": "play_animation", "args": {"animation": "police_lights", "duration": 10}}]
#: Первый ответ хода n903 как в логе Sound Pi (00:13:42 UTC).
N903_FIRST = {"actions": N903_ACTIONS, "speech": "Сирена и мигалка включены, Саша. Валера, держись!"}
#: Сырые аргументы повтора n903 (00:13:43 UTC, «Could not parse tool-call arguments JSON»): лишняя ``}`` в конце.
N903_RETRY_RAW = ('{"actions": [{"args": {"sound": "robot_alert"}, "tool": "play_sound"}, {"args": {"animation": '
                  '"police_lights", "duration": 10}, "tool": "play_animation"}], "expression": {"emotion": "surprised", '
                  '"face": {"eyes": "wide", "mouth": "o", "tint": "red"}, "rear": "hazard"}}}')


def _tool(name: str) -> Mapping[str, Any]:
    return {"type": "function", "function": {"name": name, "description": name,
                                             "parameters": {"type": "object", "properties": {}}}}


TOOLS = [_tool(n) for n in ("memory_save", "memory_search", "get_current_time", "search_web", "play_animation",
                            "play_sound", "get_sound_info")]


class _Inner:
    """Провайдер-двойник: заготовленные ответы по очереди, запросы — в ``requests``."""

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


def _reply(arguments: Mapping[str, Any]) -> LLMResponse:
    return LLMResponse(tool_calls=(ToolCall(id="c1", name=kn.COMMAND_TOOL, arguments=arguments),))


def _answer(*replies: LLMResponse) -> tuple:
    inner, log = _Inner(list(replies)), []
    msgs = [LLMMessage(role="system", content="ты РОББОКС"),
            LLMMessage(role="user", content="[говорит: Саша] а теперь сирену и мигалку покажи напугаем валеру")]
    out = asyncio.run(cb.CommandModeLLM(inner, log.append).complete(msgs, tools=TOOLS))
    return out, inner.requests, log


def _actions(out: cb.CommandResponse) -> List[str]:
    """Действия, которые исполнит ход: команды или допустимые невалидной (``Invalid.salvage``)."""
    cmd = out.command
    return [a.tool for a in (cmd.salvage if isinstance(cmd, Invalid) else cmd.actions)]


def test_n903_retry_with_extra_closing_brace_is_the_whole_object_not_bad_json():
    assert ds._safe_json(N903_RETRY_RAW) == json.loads(N903_RETRY_RAW[:-1])
    assert ds._LAST_SAFE_JSON_TRUNCATED is False


def test_tail_other_than_closers_stays_bad_json_and_cut_object_stays_truncated():
    assert ds._safe_json('{"speech": "привет"} и ещё') == {}
    assert ds._safe_json('{"speech": "привет"}{"speech": "второй"}') == {}
    assert ds._safe_json('{"actions": [{"tool": "play_sound"') == {} and ds._LAST_SAFE_JSON_TRUNCATED is True


def test_n903_retry_with_empty_arguments_runs_the_first_answers_actions_without_its_speech():
    """Повтор отдал пустые аргументы (``bad_json``) — ход получает действия первого ответа, речи нет."""
    out, requests, log = _answer(_reply(N903_FIRST), _reply({}))
    assert len(requests) == 2 and isinstance(out.command, Invalid) and out.command.reason == "bad_json"
    assert _actions(out) == ["play_sound", "play_animation"]
    assert any("retry=1 violations=['speech_with_actions'] → verdict=invalid:bad_json" in line
               and "salvage=['play_sound', 'play_animation']" in line for line in log)


def test_n903_retry_parsed_as_whole_object_is_a_valid_command_with_the_actions():
    out, _, _ = _answer(_reply(N903_FIRST), _reply(ds._safe_json(N903_RETRY_RAW)))
    assert isinstance(out.command, Command) and not out.command.speech
    assert _actions(out) == ["play_sound", "play_animation"]


def test_retry_with_its_own_valid_actions_keeps_them_not_the_first_ones():
    """Повтор невалиден, но со своими допустимыми действиями — исполняются его, а не первого ответа."""
    own = {"actions": [{"tool": "play_sound", "args": {"sound": "robot_alert"}}], "speech": "Вот!"}
    out, _, _ = _answer(_reply(N903_FIRST), _reply(own))
    assert isinstance(out.command, Invalid) and out.command.reason == kn.VIOLATION_SPEECH_WITH_ACTIONS
    assert _actions(out) == ["play_sound"]


def test_first_answer_without_valid_actions_still_closes_invalid_with_nothing_to_run():
    """Настоящее нарушение не маскируется: нечего было исполнять — ход по-прежнему невалиден без действий."""
    out, _, _ = _answer(_reply({"actions": [{"tool": "play_sound", "args": {}}]}), _reply({}))
    assert isinstance(out.command, Invalid) and (out.command.reason, out.command.salvage) == ("bad_json", ())
