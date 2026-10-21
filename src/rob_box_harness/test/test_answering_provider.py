"""Имя ответившего провайдера в метрике промпта (ADR-0150 §14 A5; #3696 ч.5).

Живая проверка ч.4 (21.10): строка ``🧮 [prompt]`` писала ``provider=health-aware-fallback`` — имя обёртки цепочки,
одно на DeepSeek и MiniMax, и A5 по провайдерам не собиралась. Теперь цепочка помнит, кто ответил
(``HealthAwareFallbackLLM.answered_by``), а ``AgentCore`` берёт имя оттуда.
"""

from __future__ import annotations

import asyncio
from typing import Any, AsyncIterator, Iterable, Mapping

import pytest

from rob_box_harness.core.agent_core import AgentCore, PromptStats, answering_provider
from rob_box_harness.core.dialogue_state_machine import DialogueEvent, DialogueStateMachine
from rob_box_harness.health import HealthAwareFallbackLLM, HealthCache
from rob_box_harness.memory.base import InMemoryStore
from rob_box_harness.tools import FakeToolProvider
from rob_box_llm.errors import ProviderError
from rob_box_llm.provider import LLMChunk, LLMMessage, LLMResponse


class _Provider:
    def __init__(self, name: str, fail: bool = False) -> None:
        self.name = name
        self._fail = fail

    async def complete(self, messages: Iterable[LLMMessage], *, tools: Iterable[Mapping[str, Any]] = (),
                       **_: Any) -> LLMResponse:
        if self._fail:
            raise ProviderError(f"{self.name} down")
        return LLMResponse(content="ок", finish_reason="stop", usage={"prompt_tokens": 100})

    async def stream(self, messages: Iterable[LLMMessage], *, tools: Iterable[Mapping[str, Any]] = (),
                     **_: Any) -> AsyncIterator[LLMChunk]:
        if self._fail:
            raise ProviderError(f"{self.name} down")
        yield LLMChunk(content_delta="ок")
        yield LLMChunk(finish_reason="stop", usage={"prompt_tokens": 100})

    async def aclose(self) -> None:
        return None


def _dsm() -> DialogueStateMachine:
    dsm = DialogueStateMachine()
    dsm.on_event(DialogueEvent.WAKE_WORD)
    dsm.on_event(DialogueEvent.STT_RESULT)
    return dsm


def test_answering_provider_prefers_chain_answer_then_name() -> None:
    class _Plain:
        name = "deepseek"

    class _Chain:
        name = "health-aware-fallback"
        answered_by = "minimax"

    class _ChainBeforeCall:
        name = "health-aware-fallback"
        answered_by = None

    assert answering_provider(_Plain()) == "deepseek"
    assert answering_provider(_Chain()) == "minimax"
    assert answering_provider(_ChainBeforeCall()) == "health-aware-fallback"
    assert answering_provider(object()) == "unknown"


@pytest.mark.parametrize("streaming", [False, True])
def test_prompt_stats_name_the_provider_that_answered(streaming: bool) -> None:
    """Primary упал — ответил запасной: в метрике его имя, а не имя обёртки."""
    chain = HealthAwareFallbackLLM([_Provider("deepseek", fail=True), _Provider("minimax")], cache=HealthCache())
    observed: list[PromptStats] = []
    core = AgentCore(llm=chain, tools=FakeToolProvider(), memory=InMemoryStore(), dsm=_dsm(),
                     system_prompt="ты робот", use_streaming=streaming, on_prompt=observed.append)

    asyncio.run(core.process_input("привет", preclassified_event=DialogueEvent.STT_RESULT))

    assert chain.answered_by == "minimax"
    assert [s.provider for s in observed] == ["minimax"]
    assert observed[0].prompt_tokens == 100 and observed[0].estimated is False
