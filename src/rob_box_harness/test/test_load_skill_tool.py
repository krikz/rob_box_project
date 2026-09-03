"""Фаза 3 change'а skill-scoped-dialogue-context — модельная активация.

Спека, требование «Активация скилла», сценарии:

* роутер промахнулся, LLM зовёт загрузку сама → текст приходит
  tool-результатом, ход ПРОДОЛЖАЕТСЯ (новой реплики юзера не требуется);
* запрошено несуществующее имя → ошибка со списком доступных, ход не
  прерывается;
* смена домена внутри сессии.

Почему ``load_skill`` живёт в harness, а не в каталоге инструментов:
каталог описывает то, что робот УМЕЕТ ДЕЛАТЬ, а этот вызов ничего на
роботе не исполняет — он меняет состав промпта и возвращает текст.
Фрагменты лежат в rob_box_voice, а mcp_server — отдельная нода в
отдельном контейнере, ему до них не дотянуться.
"""

from __future__ import annotations

import asyncio
from typing import Any, AsyncIterator, Iterable, Mapping

from rob_box_harness.core.dialog_core import LOAD_SKILL_TOOL, DialogCore
from rob_box_harness.core.dialogue_state_machine import (
    DialogueEvent,
    DialogueStateMachine,
)
from rob_box_harness.memory.base import InMemoryStore
from rob_box_harness.tools import FakeToolProvider
from rob_box_llm.provider import LLMChunk, LLMMessage, LLMResponse, ToolCall

_COMPOSER = "СКИЛЛ КОМПОЗИТОРА: зови compose_music."
_PLAYER = "СКИЛЛ ПЛЕЕРА: зови gen_list_library."
_PROMPTS = {"composer": _COMPOSER, "player": _PLAYER}


class _ScriptedProvider:
    """Отдаёт заранее заданную последовательность ответов."""

    name = "scripted"

    def __init__(self, *responses: LLMResponse) -> None:
        self._responses = list(responses)
        self.calls: list[list[LLMMessage]] = []
        self.offered_tools: list[list[Mapping[str, Any]]] = []

    def _next(self, messages, tools) -> LLMResponse:
        self.calls.append(list(messages))
        self.offered_tools.append([dict(t) for t in tools])
        if self._responses:
            return self._responses.pop(0)
        return LLMResponse(content="готово", finish_reason="stop")

    async def complete(self, messages, *, tools=(), **_: Any) -> LLMResponse:
        return self._next(messages, tools)

    async def stream(self, messages, *, tools=(), **_: Any) -> AsyncIterator[LLMChunk]:
        response = self._next(messages, tools)
        for call in response.tool_calls:
            yield LLMChunk(content_delta="", tool_call_delta=call)
        yield LLMChunk(
            content_delta=response.content or "",
            finish_reason=response.finish_reason or "stop",
        )


def _dsm() -> DialogueStateMachine:
    dsm = DialogueStateMachine()
    dsm.on_event(DialogueEvent.WAKE_WORD)
    dsm.on_event(DialogueEvent.STT_RESULT)
    return dsm


def _core(provider: _ScriptedProvider, prompts=None) -> DialogCore:
    return DialogCore(
        llm=provider,
        tools=FakeToolProvider(),
        memory=InMemoryStore(),
        dsm=_dsm(),
        system_prompt="ты РОББОКС",
        use_streaming=False,
        skill_prompts=_PROMPTS if prompts is None else prompts,
    )


def _run(core: DialogCore, text: str = "сыграй что-нибудь") -> Any:
    return asyncio.run(
        core.process_input(text, preclassified_event=DialogueEvent.STT_RESULT)
    )


def _load_call(skill: str, call_id: str = "c1") -> LLMResponse:
    return LLMResponse(
        content="",
        tool_calls=(
            ToolCall(id=call_id, name=LOAD_SKILL_TOOL, arguments={"skill": skill}),
        ),
        finish_reason="tool_calls",
    )


# ── Инструмент предъявляется ─────────────────────────────────────────────


def test_load_skill_is_offered_when_fragments_exist() -> None:
    provider = _ScriptedProvider()
    _run(_core(provider))

    names = [t["function"]["name"] for t in provider.offered_tools[0]]
    assert LOAD_SKILL_TOOL in names


def test_enum_lists_only_skills_that_have_text() -> None:
    """Модель не должна угадывать строку — имена встроены в схему."""
    provider = _ScriptedProvider()
    _run(_core(provider))

    spec = next(
        t for t in provider.offered_tools[0]
        if t["function"]["name"] == LOAD_SKILL_TOOL
    )
    enum = spec["function"]["parameters"]["properties"]["skill"]["enum"]
    assert sorted(enum) == ["composer", "player"]


def test_load_skill_is_absent_when_skills_are_off() -> None:
    """``skills_enabled=false`` → инструмента нет вообще."""
    provider = _ScriptedProvider()
    _run(_core(provider, prompts={}))

    names = [t["function"]["name"] for t in provider.offered_tools[0]]
    assert LOAD_SKILL_TOOL not in names


# ── Сценарий «роутер промахнулся, LLM грузит сама» ───────────────────────


def test_fragment_comes_back_as_a_tool_result_and_the_turn_continues() -> None:
    """Задача 3.5 — текст в хвосте messages, ход НЕ прерывается."""
    provider = _ScriptedProvider(
        _load_call("composer"),
        LLMResponse(content="поехали", finish_reason="stop"),
    )
    core = _core(provider)

    result = _run(core)

    assert result.error is None
    assert len(provider.calls) == 2, "модель должна была получить второй запрос"
    second = provider.calls[1]
    tool_messages = [m for m in second if m.role == "tool"]
    assert tool_messages, "результат load_skill не попал в промпт"
    assert _COMPOSER in str(tool_messages[-1].content)
    # Позиция максимальной свежести: tool-результат — последнее сообщение.
    assert second[-1].role == "tool"


def test_loading_a_skill_activates_it_for_later_turns() -> None:
    provider = _ScriptedProvider(
        _load_call("player"),
        LLMResponse(content="ок", finish_reason="stop"),
    )
    core = _core(provider)

    _run(core)

    assert core.active_skill == "player"


def test_counters_track_router_misses() -> None:
    """Задача 3.7 — сколько раз домен пришлось грузить вызовом LLM."""
    provider = _ScriptedProvider(
        _load_call("composer"),
        LLMResponse(content="ок", finish_reason="stop"),
    )
    core = _core(provider)
    assert core.skill_load_counters == (0, 0)

    _run(core)

    loaded, misses = core.skill_load_counters
    assert loaded == 1
    assert misses == 0


# ── Сценарий «запрошен несуществующий скилл» ─────────────────────────────


def test_unknown_skill_returns_an_error_listing_the_known_ones() -> None:
    """Задача 3.3 — ошибка со списком, ход НЕ прерывается."""
    provider = _ScriptedProvider(
        _load_call("нет-такого"),
        LLMResponse(content="ладно, обойдусь", finish_reason="stop"),
    )
    core = _core(provider)

    result = _run(core)

    assert result.error is None, "ход не должен обрываться"
    tool_messages = [m for m in provider.calls[1] if m.role == "tool"]
    content = str(tool_messages[-1].content)
    assert "composer" in content and "player" in content
    assert core.active_skill == "none", "несуществующий домен не активируется"
    assert core.skill_load_counters == (0, 1)


def test_missing_skill_argument_is_handled() -> None:
    """``load_skill({})`` — тоже промах, а не падение."""
    provider = _ScriptedProvider(
        LLMResponse(
            content="",
            tool_calls=(ToolCall(id="c1", name=LOAD_SKILL_TOOL, arguments={}),),
            finish_reason="tool_calls",
        ),
        LLMResponse(content="ок", finish_reason="stop"),
    )
    core = _core(provider)

    result = _run(core)

    assert result.error is None
    assert core.skill_load_counters == (0, 1)


# ── Смена домена внутри сессии ───────────────────────────────────────────


def test_switching_domains_within_a_session() -> None:
    """Задача 3.6."""
    provider = _ScriptedProvider(
        _load_call("composer"),
        _load_call("player", call_id="c2"),
        LLMResponse(content="ок", finish_reason="stop"),
    )
    core = _core(provider)

    _run(core)

    assert core.active_skill == "player"
    assert core.skill_load_counters == (2, 0)


def test_load_skill_never_reaches_the_tool_provider() -> None:
    """Вызов исполняется внутри harness, до робота он не доезжает."""
    executed: list[str] = []

    class _Watching(FakeToolProvider):
        async def execute(self, call):  # type: ignore[override]
            executed.append(call.name)
            return await super().execute(call)

    provider = _ScriptedProvider(
        _load_call("composer"),
        LLMResponse(content="ок", finish_reason="stop"),
    )
    core = DialogCore(
        llm=provider,
        tools=_Watching(),
        memory=InMemoryStore(),
        dsm=_dsm(),
        system_prompt="ты РОББОКС",
        use_streaming=False,
        skill_prompts=_PROMPTS,
    )

    _run(core)

    assert LOAD_SKILL_TOOL not in executed
