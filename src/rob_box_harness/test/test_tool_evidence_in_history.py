"""test_tool_evidence_in_history.py — регрессия «робот растерялся» (live 01.09).

Живой лог vision-pi, 01.09 09:42. Юзер восемь раз просит музыку, робот
отвечает красивой фразой про саксофон и НЕ вызывает ни одного тула; гуард
дважды ретраит и включает «Я тут растерялся — бит не запустился».

Вербоз-трейс запроса к MiniMax показал, что видит модель::

    [4] user: 'сыграй жесткий барабанный бит'
    [5] assistant: 'Бочка как кувалда, малый хлёсткий — бьёт без промаха.'
    [6] user: 'останови музыку'
    [7] assistant: 'Тишина.'
    [8] user: 'играем легкий джаз'
    [9] assistant: 'Саксофон мурлычет в тёплом свете...'

Ход [4]→[5] РЕАЛЬНО вызвал ``compose_music`` + ``speak_text`` (в логе
``tools=['compose_music', 'speak_text']``), но в историю лёг голым текстом:
``append_turn`` сохранял только ``role`` и ``content``. Транскрипт, который
модель перечитывает каждый ход, превращался в few-shot «просьбу о музыке
закрывают словами» — и она добросовестно его продолжала. Чем длиннее
сессия, тем больше таких примеров и тем стабильнее отказ.

Здесь проверяется, что след вызова доживает до промпта:

  * ``process_input`` пишет ``tools_called`` в ``metadata`` ассистентского
    хода;
  * ``_resolve_history`` разворачивает его в ``system``-сообщение ПЕРЕД
    ответом ассистента (не в текст ассистента — модель копирует свои же
    реплики, ретро 20.08 с утечкой ``[Spkr:...]`` в TTS);
  * ход без тулов историю не засоряет;
  * битая metadata из SQLite не роняет ход.
"""

import asyncio
from typing import Any

import pytest

from rob_box_harness.core.dialog_core import DialogCore
from rob_box_harness.core.dialogue_state_machine import DialogueStateMachine
from rob_box_harness.memory import Turn
from rob_box_llm.provider import LLMResponse, ToolCall

from rob_box_harness.tools import ToolSpec
from rob_box_llm.provider import ToolResult


class _FakeLLMProvider:
    """Отдаёт очередь ответов; запоминает messages каждого вызова."""

    name = "fake_llm"

    def __init__(
        self,
        response_text: str = "ok",
        responses: list[object] | None = None,
    ) -> None:
        self.response_text = response_text
        self.responses: list[object] = list(responses or [])
        self.calls: list[tuple[list[Any], Any]] = []

    async def complete(
        self,
        messages: Any = None,
        *,
        tools: Any = (),
        settings: Any = None,
        **_kwargs: Any,
    ) -> Any:
        self.calls.append((list(messages or []), tools))
        if self.responses:
            return self.responses.pop(0)
        return LLMResponse(content=self.response_text, tool_calls=())

    async def aclose(self) -> None:
        return None


class _FakeToolProvider:
    """Единственный тул ``echo`` — стенд-ин музыкального."""

    name = "fake_tools"

    def __init__(self) -> None:
        self._manifest = (
            ToolSpec(
                name="echo",
                description="Echo back the provided arguments.",
                parameters={
                    "type": "object",
                    "properties": {"text": {"type": "string"}},
                },
            ),
        )

    async def discover(self) -> tuple[Any, ...]:
        return self._manifest

    async def execute(self, call: Any) -> Any:
        return ToolResult(
            tool_call_id=call.id,
            content=str(dict(call.arguments).get("text", "")),
            is_error=False,
        )

    async def aclose(self) -> None:
        return None


class _FakeMemoryStore:
    """Пишет ходы в список, отдаёт хвост окна — контракт MemoryStore."""

    def __init__(self) -> None:
        self.turns: list[Turn] = []

    async def append_turn(self, scope: str, turn: Any) -> None:
        self.turns.append(turn)

    async def load_recent(self, scope: str, limit: int = 10) -> list[Any]:
        return list(self.turns[-limit:])

    async def save_fact(self, scope: str, fact: Any) -> None:
        return None

    async def search_facts(self, scope: str, query: str, limit: int = 5) -> list[Any]:
        return []

    async def aclose(self) -> None:
        return None


def _music_llm() -> _FakeLLMProvider:
    """Модель зовёт ``echo`` (стенд-ин музыкального тула), затем отвечает."""
    return _FakeLLMProvider(
        responses=[
            LLMResponse(
                content="",
                tool_calls=(
                    ToolCall(id="c1", name="echo", arguments={"text": "бит"}),
                ),
            ),
            LLMResponse(content="Бочка как кувалда.", tool_calls=()),
        ]
    )


def _core(llm: Any, memory: _FakeMemoryStore, **kwargs: Any) -> DialogCore:
    obj = DialogCore(
        llm=llm,
        tools=_FakeToolProvider(),
        memory=memory,
        dsm=DialogueStateMachine(),
        **kwargs,
    )
    asyncio.run(obj.handle_wake_word(""))
    return obj


def test_assistant_turn_records_called_tools() -> None:
    memory = _FakeMemoryStore()
    core = _core(_music_llm(), memory)

    result = asyncio.run(core.process_input("сыграй жесткий барабанный бит"))

    assert "echo" in result.tools_called
    assistant_turns = [t for t in memory.turns if t.role == "assistant"]
    assert assistant_turns, "ассистентский ход не сохранён"
    assert assistant_turns[-1].metadata.get("tools_called") == ["echo"]


def test_turn_without_tools_stores_no_evidence() -> None:
    memory = _FakeMemoryStore()
    core = _core(_FakeLLMProvider(response_text="просто ответ"), memory)

    asyncio.run(core.process_input("который час"))

    assistant_turns = [t for t in memory.turns if t.role == "assistant"]
    assert assistant_turns
    assert "tools_called" not in assistant_turns[-1].metadata


def test_evidence_reaches_the_prompt_before_the_assistant_reply() -> None:
    memory = _FakeMemoryStore()
    memory.turns.extend([
        Turn(role="user", content="сыграй жесткий барабанный бит"),
        Turn(
            role="assistant",
            content="Бочка как кувалда.",
            metadata={"tools_called": ["compose_music", "speak_text"]},
        ),
    ])
    llm = _FakeLLMProvider(response_text="ок")
    core = _core(llm, memory, system_prompt="ПРОМПТ", history_trim_limit=20)

    asyncio.run(core.process_input("играем легкий джаз"))

    sent = llm.calls[0][0]
    roles = [m.role for m in sent]
    contents = [m.content for m in sent]
    idx = contents.index("Бочка как кувалда.")
    assert roles[idx - 1] == "system"
    assert "compose_music" in contents[idx - 1]
    assert "speak_text" in contents[idx - 1]
    # Текст ассистента не тронут — модели нечего копировать в TTS.
    assert contents[idx] == "Бочка как кувалда."


def test_plain_history_gets_no_extra_system_messages() -> None:
    memory = _FakeMemoryStore()
    memory.turns.extend([
        Turn(role="user", content="привет"),
        Turn(role="assistant", content="здорово"),
    ])
    llm = _FakeLLMProvider(response_text="ок")
    core = _core(llm, memory, system_prompt="ПРОМПТ", history_trim_limit=20)

    asyncio.run(core.process_input("как дела"))

    sent = llm.calls[0][0]
    # Один system — базовый промпт; никаких «[выполнено ...]».
    assert [m.role for m in sent].count("system") == 1


@pytest.mark.parametrize(
    "broken",
    [
        {"tools_called": "compose_music"},   # строка вместо списка
        {"tools_called": []},                # пустой список
        {"tools_called": [None, 42]},        # мусор внутри
        {"tools_called": None},
    ],
)
def test_malformed_metadata_does_not_break_the_turn(broken: dict) -> None:
    memory = _FakeMemoryStore()
    memory.turns.extend([
        Turn(role="user", content="сыграй"),
        Turn(role="assistant", content="ответ", metadata=broken),
    ])
    llm = _FakeLLMProvider(response_text="ок")
    core = _core(llm, memory, system_prompt="ПРОМПТ", history_trim_limit=20)

    result = asyncio.run(core.process_input("играем легкий джаз"))

    assert result.error is None
    sent = llm.calls[0][0]
    assert [m.role for m in sent].count("system") == 1
