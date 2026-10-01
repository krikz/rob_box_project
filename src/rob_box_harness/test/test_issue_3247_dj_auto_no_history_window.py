"""Issue #3247 — DJ_AUTO-ход не видит окна ходов разговора.

Живой прогон 30.09.2026: финал сета «море и чайки» прочитал в окне
реплики сета №2 («включи диджей сет на тему пираты», «[TG] Клод доложи
обстановку» → «Пиратский сет … Maniac 126 BPM») и перезапустил сет с темой
«пираты», ответив почти дословно: «Пираты вышли на палубу — Maniac 126
BPM». DJ-промпт самодостаточен (тема, план, сыгранное, темп) — окно
разговора с людьми ему не нужно.
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

    def __init__(self) -> None:
        self.calls: list[list[Any]] = []

    async def complete(self, messages: Any = None, **_kw: Any) -> Any:
        self.calls.append(list(messages or []))
        return LLMResponse(content="ок", tool_calls=())

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


_PAST_SET = [
    Turn(role="user", content="[Speaker:unknown] включи диджей сет на тему пираты"),
    Turn(
        role="assistant",
        content="Пиратский сет уже вовсю рубит на палубе — Maniac 126 BPM",
        metadata={"tools_called": ["compose_music", "set_dj_mode"]},
    ),
]


def _core(llm: _FakeLLM) -> AgentCore:
    core = AgentCore(
        llm=llm, tools=_NoTools(), memory=_NoMemory(),
        dsm=DialogueStateMachine(), system_prompt="ПРОМПТ",
        history_trim_limit=40,
    )
    core._turn_window.extend(_PAST_SET)
    return core


def _text(messages: list[Any]) -> str:
    return "\n".join(str(m.content) for m in messages)


def test_dj_auto_turn_sees_no_past_set() -> None:
    llm = _FakeLLM()
    core = _core(llm)
    asyncio.run(core.process_input(
        '[DJ_AUTO переход #4 — ФИНАЛЬНЫЙ ТРЕК] Тема вечеринки: "море и чайки".',
        is_dj_auto=True,
    ))
    sent = llm.calls[0]
    assert [m.role for m in sent] == ["system", "user"]
    assert "пират" not in _text(sent).lower()
    assert "Maniac" not in _text(sent)
    assert "[выполнено в прошлых ходах]" not in _text(sent)
    # Окно не тронуто: следующий ход человека видит разговор целиком.
    assert list(core._turn_window) == _PAST_SET


def test_human_turn_still_sees_window() -> None:
    llm = _FakeLLM()
    core = _core(llm)
    core._dsm.on_event(DialogueEvent.WAKE_WORD)
    asyncio.run(core.process_input("а что сейчас играет?"))
    assert "Maniac" in _text(llm.calls[0])
