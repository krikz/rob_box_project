"""Issue #3145 — гигиена контекста в ``AgentCore``.

* не больше двух system на запрос, ни одного посреди истории (пробник
  28.09: облачный MiniMax-M3 читает mid-system как инструкцию);
* след тулов — один блок «выполнено в прошлых ходах» во втором system;
* ``discard_last_reply`` снимает ответ ТОЛЬКО последнего хода и не
  трогает чужой; тулы отозванного ответа не теряются;
* двух assistant подряд в запросе нет.
"""

from __future__ import annotations

import asyncio
from typing import Any

from rob_box_harness.core.agent_core import (
    _EXECUTED_ACTIONS_MAX,
    AgentCore,
)
from rob_box_harness.core.dialogue_state_machine import (
    DialogueEvent,
    DialogueStateMachine,
)
from rob_box_harness.memory import Turn
from rob_box_llm.provider import LLMResponse


class _FakeLLM:
    name = "fake_llm"

    def __init__(self, text: str = "ок") -> None:
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


def _core(llm: _FakeLLM, **kw: Any) -> AgentCore:
    return AgentCore(
        llm=llm, tools=_NoTools(), memory=_NoMemory(),
        dsm=DialogueStateMachine(), system_prompt="ПРОМПТ",
        history_trim_limit=40, **kw,
    )


def _turn(core: AgentCore, text: str, **kw: Any) -> None:
    core._dsm.on_event(DialogueEvent.WAKE_WORD)
    asyncio.run(core.process_input(text, **kw))


def _music_window() -> list[Turn]:
    turns: list[Turn] = []
    for i in range(6):
        turns.append(Turn(role="user", content=f"сыграй трек номер {i}"))
        turns.append(Turn(
            role="assistant", content=f"Играю трек {i}.",
            metadata={"tools_called": ["compose_music", "speak_text"]},
        ))
    return turns


class TestSystemMessagesInRequest:
    def test_at_most_two_system_and_none_mid_history(self) -> None:
        llm = _FakeLLM()
        core = _core(llm)
        core._turn_window.extend(_music_window())

        _turn(
            core, "а теперь джаз",
            speaker_context="Контекст о собеседнике: Саша.",
            dynamic_system="<system_context/>",
        )

        roles = [m.role for m in llm.calls[0]]
        assert roles.count("system") <= 2, roles
        assert roles[:2] == ["system", "system"], roles
        assert "system" not in roles[2:], roles
        header = llm.calls[0][1].content
        assert header.startswith("Контекст о собеседнике: Саша.")
        assert "[выполнено в прошлых ходах]" in header
        assert "на «сыграй трек номер 5»: compose_music, speak_text" in header

    def test_no_evidence_markup_in_history_messages(self) -> None:
        llm = _FakeLLM()
        core = _core(llm)
        core._turn_window.extend(_music_window())

        _turn(core, "а теперь джаз")

        for message in llm.calls[0][2:]:
            assert "выполнено в прошл" not in message.content, message

    def test_block_is_capped(self) -> None:
        llm = _FakeLLM()
        core = _core(llm)
        for i in range(_EXECUTED_ACTIONS_MAX + 4):
            core._turn_window.append(Turn(role="user", content=f"запрос {i}"))
            core._turn_window.append(Turn(
                role="assistant", content=f"ответ {i}",
                metadata={"tools_called": ["echo"]},
            ))

        _turn(core, "дальше")

        header = llm.calls[0][1].content
        lines = [ln for ln in header.splitlines() if ln.startswith("- ")]
        assert len(lines) == _EXECUTED_ACTIONS_MAX
        # Свежие ходы остаются, старые уходят.
        assert f"запрос {_EXECUTED_ACTIONS_MAX + 3}" in lines[-1]
        assert "«запрос 0»" not in header

    def test_plain_dialogue_has_one_system(self) -> None:
        llm = _FakeLLM()
        core = _core(llm)
        core._turn_window.extend([
            Turn(role="user", content="привет"),
            Turn(role="assistant", content="здорово"),
        ])
        _turn(core, "как дела")
        assert [m.role for m in llm.calls[0]].count("system") == 1


class TestDiscardLastReply:
    def test_discards_reply_of_last_turn_only(self) -> None:
        llm = _FakeLLM("Точка удалена.")
        core = _core(llm)
        core._turn_window.extend([
            Turn(role="user", content="привет"),
            Turn(role="assistant", content="Привет!"),
        ])
        _turn(core, "удали точку кухня")

        assert asyncio.run(core.discard_last_reply()) is True
        contents = [t.content for t in core._turn_window]
        assert contents == ["привет", "Привет!", "удали точку кухня"]
        # Повторный отзыв (music-гуард + общий путь) — no-op.
        assert asyncio.run(core.discard_last_reply()) is False
        assert [t.content for t in core._turn_window] == contents

    def test_turn_without_persisted_reply_discards_nothing(self) -> None:
        """DJ-переход ответ в историю не пишет — отзывать нечего, а старый
        код сносил последний ответ ЮЗЕРУ."""
        llm = _FakeLLM("Клубняк!")
        core = _core(llm)
        core._turn_window.extend([
            Turn(role="user", content="привет"),
            Turn(role="assistant", content="Привет!"),
        ])
        _turn(core, "[DJ_AUTO] переход #2", is_dj_auto=True)

        assert asyncio.run(core.discard_last_reply()) is False
        assert [t.content for t in core._turn_window] == ["привет", "Привет!"]

    def test_tools_of_retracted_reply_survive_in_block(self) -> None:
        llm = _FakeLLM("ок")
        core = _core(llm)
        core._turn_window.append(Turn(role="user", content="сыграй бит"))
        reply = Turn(
            role="assistant", content="Сохранил пресет.",
            metadata={"tools_called": ["compose_music"]},
        )
        core._turn_window.append(reply)
        core._turn_reply = reply  # как будто его записал последний ход

        assert asyncio.run(core.discard_last_reply()) is True
        _turn(core, "[CRITICAL] retry", is_synthetic=True)

        header = llm.calls[0][1].content
        assert "на «сыграй бит»: compose_music" in header
        window = [(t.role, t.content) for t in core._turn_window]
        assert window == [("user", "сыграй бит"), ("assistant", "ок")]

    def test_clear_history_forgets_turn_reply(self) -> None:
        core = _core(_FakeLLM("ответ"))
        _turn(core, "вопрос")
        core.clear_history()
        assert asyncio.run(core.discard_last_reply()) is False


class TestNoTwoAssistantsInRequest:
    def test_consecutive_assistants_collapse_to_latest(self) -> None:
        llm = _FakeLLM()
        core = _core(llm)
        core._turn_window.extend([
            Turn(role="user", content="ты диджей Снупдог"),
            Turn(role="assistant", content="Клубняк в клубе, погнали дальше!"),
            Turn(role="assistant", content="Йо, Снупдог за пультом."),
        ])

        _turn(core, "что играет")

        sent = llm.calls[0]
        roles = [m.role for m in sent]
        assert not any(
            a == b == "assistant" for a, b in zip(roles, roles[1:])
        ), roles
        contents = [m.content for m in sent]
        assert "Клубняк в клубе, погнали дальше!" not in contents
        assert "Йо, Снупдог за пультом." in contents
