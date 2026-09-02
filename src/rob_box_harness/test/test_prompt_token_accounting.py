"""Фаза 0 change'а skill-scoped-dialogue-context — учёт размера промпта.

Спека (``specs/dialogue-skill-context/spec.md``), требование
«Наблюдаемость размера промпта», два сценария:

* провайдер вернул usage  → публикуется его число, ``estimated=False``;
* провайдер не вернул     → публикуется клиентская оценка,
  ``estimated=True``, и обращение к LLM НЕ прерывается.

Плюс регресс на то, ради чего фаза вообще существует: до неё
``_stream_response`` жёстко ставил ``usage=None``, а stream-цикл делал
``return`` на finish_reason — то есть терминальный чанк с usage,
который OpenAI-совместимые API присылают ПОСЛЕ finish, не читался
никогда.
"""

from __future__ import annotations

import asyncio
from typing import Any, AsyncIterator, Iterable, Mapping

import pytest

from rob_box_core.token_estimate import estimate_prompt_tokens, estimate_tokens
from rob_box_harness.core.dialog_core import DialogCore, PromptStats
from rob_box_harness.core.dialogue_state_machine import (
    DialogueEvent,
    DialogueStateMachine,
)
from rob_box_harness.memory.base import InMemoryStore
from rob_box_harness.tools import FakeToolProvider
from rob_box_llm.provider import LLMChunk, LLMMessage, LLMResponse


class _RecordingProvider:
    """LLM-двойник с настраиваемым usage и обоими путями (stream/complete)."""

    name = "fake-provider"

    def __init__(self, usage: Mapping[str, int] | None) -> None:
        self._usage = usage
        self.seen_tools: list[Any] = []

    async def complete(
        self,
        messages: Iterable[LLMMessage],
        *,
        tools: Iterable[Mapping[str, Any]] = (),
        **_: Any,
    ) -> LLMResponse:
        self.seen_tools.append(list(tools))
        return LLMResponse(content="ок", finish_reason="stop", usage=self._usage)

    async def stream(
        self,
        messages: Iterable[LLMMessage],
        *,
        tools: Iterable[Mapping[str, Any]] = (),
        **_: Any,
    ) -> AsyncIterator[LLMChunk]:
        yield LLMChunk(content_delta="ок")
        # Терминальный чанк несёт usage — ровно так его отдаёт
        # OpenAI-совместимый стрим при stream_options.include_usage.
        yield LLMChunk(content_delta="", finish_reason="stop", usage=self._usage)


def _dialogue_dsm() -> DialogueStateMachine:
    """DSM, уже доведённый до ``DIALOGUE``.

    ``process_input`` зовёт LLM только из состояния DIALOGUE. В проде
    переход делает ``dialogue_node._on_stt`` и передаёт
    ``preclassified_event``; тест обязан воспроизвести то же, иначе ход
    молча схлопывается в IDLE и LLM не вызывается вообще.
    """
    dsm = DialogueStateMachine()
    dsm.on_event(DialogueEvent.WAKE_WORD)
    dsm.on_event(DialogueEvent.STT_RESULT)
    return dsm


def _core(
    provider: Any,
    observed: list[PromptStats] | None = None,
    *,
    streaming: bool,
) -> DialogCore:
    return DialogCore(
        llm=provider,
        tools=FakeToolProvider(),
        memory=InMemoryStore(),
        dsm=_dialogue_dsm(),
        system_prompt="ты робот",
        use_streaming=streaming,
        on_prompt=None if observed is None else observed.append,
    )


# ── Требование: наблюдаемость размера промпта ────────────────────────────


@pytest.mark.parametrize("streaming", [False, True])
def test_provider_usage_is_published_as_exact(streaming: bool) -> None:
    """Сценарий «провайдер вернул расход токенов»."""
    observed: list[PromptStats] = []
    core = _core(
        _RecordingProvider({"prompt_tokens": 12345}), observed, streaming=streaming
    )

    asyncio.run(core.process_input("привет", preclassified_event=DialogueEvent.STT_RESULT))

    assert observed, "наблюдатель не был вызван ни разу"
    stats = observed[0]
    assert stats.prompt_tokens == 12345
    assert stats.estimated is False
    assert stats.provider == "fake-provider"


@pytest.mark.parametrize("streaming", [False, True])
def test_missing_usage_falls_back_to_estimate(streaming: bool) -> None:
    """Сценарий «провайдер не вернул расход токенов»: оценка + метка."""
    observed: list[PromptStats] = []
    core = _core(_RecordingProvider(None), observed, streaming=streaming)

    result = asyncio.run(core.process_input("привет", preclassified_event=DialogueEvent.STT_RESULT))

    assert result.error is None, "отсутствие usage не должно ронять ход"
    assert observed
    stats = observed[0]
    assert stats.estimated is True
    assert stats.prompt_tokens > 0


@pytest.mark.parametrize("streaming", [False, True])
def test_zero_usage_is_treated_as_unknown(streaming: bool) -> None:
    """``prompt_tokens=0`` на непустом запросе — враньё, а не измерение.

    Записать ноль в гистограмму значит испортить перцентили, поэтому
    такой ответ трактуется как «не сообщил» и уходит в оценку.
    """
    observed: list[PromptStats] = []
    core = _core(
        _RecordingProvider({"prompt_tokens": 0}), observed, streaming=streaming
    )

    asyncio.run(core.process_input("привет", preclassified_event=DialogueEvent.STT_RESULT))

    assert observed
    assert observed[0].estimated is True
    assert observed[0].prompt_tokens > 0


def test_observer_exception_does_not_break_the_turn() -> None:
    """Телеметрия не имеет права ронять живой ход."""

    def _boom(_stats: PromptStats) -> None:
        raise RuntimeError("метрика упала")

    core = DialogCore(
        llm=_RecordingProvider({"prompt_tokens": 10}),
        tools=FakeToolProvider(),
        memory=InMemoryStore(),
        dsm=_dialogue_dsm(),
        system_prompt="ты робот",
        use_streaming=True,
        on_prompt=_boom,
    )

    result = asyncio.run(core.process_input("привет", preclassified_event=DialogueEvent.STT_RESULT))

    assert result.error is None


def test_no_observer_is_allowed() -> None:
    """``on_prompt=None`` — прежнее поведение, без учёта и без падений."""
    core = _core(_RecordingProvider(None), streaming=True)

    result = asyncio.run(core.process_input("привет", preclassified_event=DialogueEvent.STT_RESULT))

    assert result.error is None


def test_tools_are_counted_in_the_estimate() -> None:
    """Схемы инструментов — большая часть промпта, их обязано быть видно.

    Регресс на класс ошибок «оценили только сообщения»: у нас 16 240
    токенов схем против ~11 тысяч промпта, и оценка без tools занижала бы
    измеряемую величину примерно вдвое.
    """
    messages = [LLMMessage(role="user", content="привет")]
    fat_tool = {
        "type": "function",
        "function": {
            "name": "compose_music",
            "description": "описание " * 200,
            "parameters": {"type": "object", "properties": {}},
        },
    }

    without_tools = estimate_prompt_tokens(messages, [])
    with_tools = estimate_prompt_tokens(messages, [fat_tool])

    assert with_tools > without_tools * 5


# ── Оценщик токенов ──────────────────────────────────────────────────────


def test_estimator_charges_cyrillic_more_than_latin() -> None:
    """Кириллица дороже латиницы — иначе русский промпт занижен втрое.

    Тексты одной длины в символах, разный алфавит.
    """
    ru = "Робот, поезжай на кухню и включи музыку погромче пожалуйста"
    en = "Please navigate to the kitchen and turn up the music volume"
    assert len(ru) == len(en), "образцы должны быть одной длины в символах"

    assert estimate_tokens(ru) > estimate_tokens(en) * 2


def test_estimator_edge_cases() -> None:
    assert estimate_tokens("") == 0
    # Непустой текст не может стоить ноль токенов.
    assert estimate_tokens("   ") >= 1
    assert estimate_prompt_tokens([], []) == 0


def test_estimator_counts_tool_calls_in_history() -> None:
    """Вызовы инструментов уезжают на провод и стоят токенов.

    Ход с батчем из восьми вызовов без этого учёта выглядел бы как
    пустое сообщение ассистента.
    """
    from rob_box_llm.provider import ToolCall

    plain = [LLMMessage(role="assistant", content="")]
    with_calls = [
        LLMMessage(
            role="assistant",
            content="",
            tool_calls=(
                ToolCall(
                    id="1",
                    name="compose_music",
                    arguments={"prompt": "жёсткий барабанный бит" * 20},
                ),
            ),
        )
    ]

    assert estimate_prompt_tokens(with_calls, []) > estimate_prompt_tokens(plain, [])
