"""Issue #3165 — ход, исполненный мимо модели, виден модели.

Роутер медиакоманд (#3134) сам исполняет «выключи музыку» и говорит
фиксированную фразу. Живой прогон 29.09.2026 00:12: через 11 с модель
правдиво сказала «я останавливал трек», а гуард #2559 счёл это фантомом —
в её истории не было ни реплики, ни вызова ``stop_music``.
``AgentCore.record_external_turn`` кладёт такой ход в окно: пара
user/assistant + тулы в блоке «выполнено в прошлых ходах».
"""

from __future__ import annotations

import asyncio
from typing import Any

from rob_box_harness.core.agent_core import AgentCore
from rob_box_harness.core.dialogue_state_machine import (
    DialogueEvent,
    DialogueStateMachine,
)
from rob_box_llm.provider import LLMResponse


class _FakeLLM:
    name = "fake_llm"

    def __init__(self, text: str) -> None:
        self.text = text
        self.calls: list[list[Any]] = []

    async def complete(self, messages: Any = None, **_kw: Any) -> Any:
        self.calls.append(list(messages or []))
        return LLMResponse(content=self.text, tool_calls=())

    async def aclose(self) -> None:
        return None


class _NoTools:
    name = "no_tools"

    async def discover(self) -> tuple:
        return ()

    async def execute(self, call: Any) -> Any:  # pragma: no cover
        raise AssertionError

    async def aclose(self) -> None:
        return None


class _NoMemory:
    async def save_fact(self, *a: Any, **k: Any) -> None:
        return None

    async def search_facts(self, *a: Any, **k: Any) -> list:
        return []

    async def aclose(self) -> None:
        return None


def _core(llm: _FakeLLM) -> AgentCore:
    return AgentCore(
        llm=llm, tools=_NoTools(), memory=_NoMemory(),
        dsm=DialogueStateMachine(), system_prompt="ПРОМПТ",
        history_trim_limit=40,
    )


def _ask(core: AgentCore, text: str) -> None:
    core._dsm.on_event(DialogueEvent.WAKE_WORD)
    asyncio.run(core.process_input(text))


def test_router_stop_is_in_history_and_executed_block() -> None:
    llm = _FakeLLM("Одиннадцать секунд назад я остановил трек, сейчас тишина.")
    core = _core(llm)
    core.record_external_turn("выключи музыку", "Выключил музыку.", ["stop_music"])

    _ask(core, "а какую музыку ты сейчас включал")

    messages = llm.calls[-1]
    roles = [m.role for m in messages]
    assert roles.count("system") == 2, roles
    executed = messages[1].content
    assert "[выполнено в прошлых ходах]" in executed
    assert "- на «выключи музыку»: stop_music" in executed
    history = [(m.role, m.content) for m in messages[2:-1]]
    assert history == [
        ("user", "выключи музыку"),
        ("assistant", "Выключил музыку."),
    ]


def test_router_turn_is_not_retracted_by_discard_last_reply() -> None:
    """``discard_last_reply`` снимает только ответ модели, не ход роутера."""
    llm = _FakeLLM("Готово.")
    core = _core(llm)
    core.record_external_turn("выключи музыку", "Выключил музыку.", ["stop_music"])

    assert asyncio.run(core.discard_last_reply()) is False
    assert [t.content for t in core._turn_window] == [
        "выключи музыку", "Выключил музыку.",
    ]


def test_failed_router_tools_are_not_listed() -> None:
    """Тул, который не сработал, роутер не передаёт — строки нет."""
    llm = _FakeLLM("Понял.")
    core = _core(llm)
    core.record_external_turn(
        "выключи музыку", "Не получилось выключить музыку.", []
    )

    _ask(core, "что случилось")

    messages = llm.calls[-1]
    assert not any(
        "[выполнено в прошлых ходах]" in str(m.content) for m in messages
    )
    assert ("assistant", "Не получилось выключить музыку.") in [
        (m.role, m.content) for m in messages
    ]


def test_empty_phrase_or_text_records_nothing() -> None:
    core = _core(_FakeLLM("x"))
    core.record_external_turn("выключи музыку", "", ["stop_music"])
    core.record_external_turn("", "Выключил музыку.", ["stop_music"])
    assert list(core._turn_window) == []
