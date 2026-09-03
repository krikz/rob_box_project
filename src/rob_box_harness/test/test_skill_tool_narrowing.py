"""Фаза 7 change'а skill-scoped-dialogue-context — Move B.

Спека, требование «Сужение набора инструментов выключено по умолчанию»,
три сценария:

* сужение выключено (дефолт) → LLM получает полный каталог;
* сужение включено → core плюс инструменты активного скилла;
* сужение включено, запрос вне активного скилла → модель может
  подгрузить нужный домен ``load_skill``-ом и выполнить его в том же ходу.

Move B — единственная часть change'а, которая реально что-то отнимает у
модели. Поэтому она за отдельным флагом, выключенным по умолчанию, и
поэтому у неё есть fallback: сужение, оставившее пустой список, отдаёт
полный каталог обратно — пустой ``tools=`` доезжает до юзера как «нет
такой функции», а это хуже длинного промпта.
"""

from __future__ import annotations

import asyncio
from typing import Any, AsyncIterator, Iterable, Mapping

import pytest

from rob_box_core.tool_catalog import llm_visible_tools, tools_for_skill
from rob_box_harness.core.dialog_core import LOAD_SKILL_TOOL, DialogCore
from rob_box_harness.core.dialogue_state_machine import (
    DialogueEvent,
    DialogueStateMachine,
)
from rob_box_harness.core.tool_registry import ToolRegistry
from rob_box_harness.memory.base import InMemoryStore
from rob_box_harness.tools import FakeToolProvider, ToolProvider
from rob_box_llm.provider import LLMChunk, LLMResponse, ToolCall, ToolResult

_PROMPTS = {"composer": "СКИЛЛ КОМПОЗИТОРА", "player": "СКИЛЛ ПЛЕЕРА"}


class _CatalogProvider(ToolProvider):
    """ToolProvider, отдающий реальный каталог робота."""

    name = "catalog"

    def __init__(self) -> None:
        self._registry = ToolRegistry()

    async def discover(self):
        return self._registry.list_tools()

    async def execute(self, call: ToolCall) -> ToolResult:
        return ToolResult(tool_call_id=call.id, content="ok", is_error=False)


class _Scripted:
    name = "scripted"

    def __init__(self, *responses: LLMResponse) -> None:
        self._responses = list(responses)
        self.offered: list[list[str]] = []

    def _next(self, tools) -> LLMResponse:
        self.offered.append(
            [t["function"]["name"] for t in tools]
        )
        if self._responses:
            return self._responses.pop(0)
        return LLMResponse(content="готово", finish_reason="stop")

    async def complete(self, messages, *, tools=(), **_: Any) -> LLMResponse:
        return self._next(list(tools))

    async def stream(self, messages, *, tools=(), **_: Any) -> AsyncIterator[LLMChunk]:
        response = self._next(list(tools))
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


def _core(provider, *, narrow: bool, tools=None) -> DialogCore:
    return DialogCore(
        llm=provider,
        tools=tools or _CatalogProvider(),
        memory=InMemoryStore(),
        dsm=_dsm(),
        system_prompt="ты РОББОКС",
        use_streaming=False,
        skill_prompts=_PROMPTS,
        narrow_tools_to_skill=narrow,
    )


def _run(core: DialogCore, text: str = "сыграй бит") -> Any:
    return asyncio.run(
        core.process_input(text, preclassified_event=DialogueEvent.STT_RESULT)
    )


# ── Сценарий «сужение выключено» (поведение по умолчанию) ────────────────


def test_narrowing_off_offers_the_whole_catalog() -> None:
    provider = _Scripted()
    core = _core(provider, narrow=False)
    core.set_active_skill("composer")

    _run(core)

    offered = set(provider.offered[0]) - {LOAD_SKILL_TOOL}
    assert offered == {e.name for e in llm_visible_tools()}


def test_narrowing_defaults_to_off() -> None:
    """Флаг не передан — значит выключено."""
    provider = _Scripted()
    core = DialogCore(
        llm=provider,
        tools=_CatalogProvider(),
        memory=InMemoryStore(),
        dsm=_dsm(),
        system_prompt="ты РОББОКС",
        use_streaming=False,
        skill_prompts=_PROMPTS,
    )
    core.set_active_skill("composer")

    _run(core)

    offered = set(provider.offered[0]) - {LOAD_SKILL_TOOL}
    assert len(offered) == len(llm_visible_tools())


# ── Сценарий «сужение включено» ──────────────────────────────────────────


def test_narrowing_on_offers_core_plus_active_skill() -> None:
    provider = _Scripted()
    core = _core(provider, narrow=True)
    core.set_active_skill("composer")

    _run(core)

    offered = set(provider.offered[0]) - {LOAD_SKILL_TOOL}
    assert offered == {e.name for e in tools_for_skill("composer")}
    assert "speak_text" in offered, "core обязан остаться"
    assert len(offered) < len(llm_visible_tools())


def test_narrowing_without_an_active_skill_changes_nothing() -> None:
    """Скилл не активирован — сужать не по чему."""
    provider = _Scripted()
    core = _core(provider, narrow=True)

    _run(core)

    offered = set(provider.offered[0]) - {LOAD_SKILL_TOOL}
    assert len(offered) == len(llm_visible_tools())


def test_unknown_active_skill_falls_back_to_the_full_catalog() -> None:
    """Рассинхрон не имеет права оставить модель без инструментов."""
    provider = _Scripted()
    core = _core(provider, narrow=True)
    core.set_active_skill("нет-такого")

    _run(core)

    offered = set(provider.offered[0]) - {LOAD_SKILL_TOOL}
    assert len(offered) == len(llm_visible_tools())


def test_narrowing_never_yields_an_empty_tool_list() -> None:
    """Пустой tools= доезжает до юзера как «нет такой функции»."""
    provider = _Scripted()
    # Провайдер, у которого вообще нет инструментов активного скилла.
    core = _core(provider, narrow=True, tools=FakeToolProvider())
    core.set_active_skill("composer")

    _run(core)

    assert provider.offered[0], "список инструментов оказался пустым"


# ── Сценарий «запрос вне активного скилла» ───────────────────────────────


def test_model_can_reach_another_domain_via_load_skill() -> None:
    """Задача 7.3, третий сценарий спеки.

    Домен сужен до composer, а нужен player. Модель зовёт load_skill и
    получает инструменты плеера уже на следующей итерации ТОГО ЖЕ хода.
    """
    provider = _Scripted(
        LLMResponse(
            content="",
            tool_calls=(
                ToolCall(
                    id="c1",
                    name=LOAD_SKILL_TOOL,
                    arguments={"skill": "player"},
                ),
            ),
            finish_reason="tool_calls",
        ),
        LLMResponse(content="нашёл", finish_reason="stop"),
    )
    core = _core(provider, narrow=True)
    core.set_active_skill("composer")

    result = _run(core, "включи тот трек")

    assert result.error is None
    first = set(provider.offered[0])
    second = set(provider.offered[1])
    assert "gen_play_from_library" not in first, "изначально плеера быть не должно"
    assert "gen_play_from_library" in second, "после load_skill плеер обязан появиться"


def test_load_skill_is_always_offered_even_when_narrowed() -> None:
    """Иначе из суженного домена не выбраться."""
    provider = _Scripted()
    core = _core(provider, narrow=True)
    core.set_active_skill("composer")

    _run(core)

    assert LOAD_SKILL_TOOL in provider.offered[0]
