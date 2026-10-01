"""ADR-0129 (issue #3000) — ``AgentCore.clear_history(keep=...)``.

Смена DJ-сета вычищает из окна обмены прошлых сетов, не трогая остальной
разговор. Фильтр приходит из shell'а (``rob_box_voice.core.dj_set_boundary``);
ядро только применяет его и не теряет отзываемый ответ хода.
"""

from __future__ import annotations

import asyncio
from typing import Any

from rob_box_harness.core.agent_core import AgentCore
from rob_box_harness.core.dialogue_state_machine import (
    DialogueEvent,
    DialogueStateMachine,
)
from rob_box_harness.memory import Turn
from rob_box_llm.provider import LLMResponse


class _FakeLLM:
    name = "fake_llm"

    async def complete(self, messages: Any = None, **_kw: Any) -> Any:
        return LLMResponse(content="ответ", tool_calls=())

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


def _core() -> AgentCore:
    return AgentCore(
        llm=_FakeLLM(), tools=_NoTools(), memory=_NoMemory(),
        dsm=DialogueStateMachine(), system_prompt="ПРОМПТ",
        history_trim_limit=40,
    )


def _window() -> list[Turn]:
    return [
        Turn(role="user", content="ты диджей Старый"),
        Turn(role="assistant", content="Я Старый.", metadata={"tools_called": ["set_dj_mode"]}),
        Turn(role="user", content="какой сегодня праздник"),
        Turn(role="assistant", content="День молодёжи."),
    ]


def test_default_clears_everything() -> None:
    core = _core()
    core._turn_window.extend(_window())
    core.clear_history()
    assert list(core._turn_window) == []


def test_keep_filter_decides_what_stays() -> None:
    core = _core()
    core._turn_window.extend(_window())
    core.clear_history(keep=lambda turns: turns[2:])
    assert [t.content for t in core._turn_window] == ["какой сегодня праздник", "День молодёжи."]


def test_kept_reply_can_still_be_retracted() -> None:
    core = _core()
    core._dsm.on_event(DialogueEvent.WAKE_WORD)
    asyncio.run(core.process_input("вопрос"))
    core.clear_history(keep=lambda turns: turns)
    assert asyncio.run(core.discard_last_reply()) is True


def test_dropped_reply_is_forgotten() -> None:
    core = _core()
    core._dsm.on_event(DialogueEvent.WAKE_WORD)
    asyncio.run(core.process_input("вопрос"))
    core.clear_history(keep=lambda turns: [])
    assert asyncio.run(core.discard_last_reply()) is False
