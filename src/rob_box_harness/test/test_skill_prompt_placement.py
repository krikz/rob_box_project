"""Фаза 2 change'а skill-scoped-dialogue-context — Move A.

Спека, требование «Инструкции скилла доставляются рядом с текущим ходом»:
текст активного скилла ДОЛЖЕН стоять после последнего сообщения истории
и не дальше от реплики юзера, чем снимок рантайм-состояния.

Ради чего: инструкция на позиции 0 проигрывает двадцати ходам истории,
которые демонстрируют обратное поведение. Тот же принцип уже применён в
этом файле к ``<system_context>`` (agent_core: «волатильный runtime-стейт
должен стоять там, где он и по времени — рядом с последним user-ходом»).
"""

from __future__ import annotations

import asyncio
from typing import Any, AsyncIterator, Iterable, Mapping

import pytest

from rob_box_harness.core.agent_core import AgentCore
from rob_box_harness.core.dialogue_state_machine import (
    DialogueEvent,
    DialogueStateMachine,
)
from rob_box_harness.memory.base import InMemoryStore, Turn
from rob_box_harness.tools import FakeToolProvider
from rob_box_llm.provider import LLMChunk, LLMMessage, LLMResponse

_COMPOSER_TEXT = "СКИЛЛ КОМПОЗИТОРА: вызывай compose_music, не описывай словами."
_PLAYER_TEXT = "СКИЛЛ ПЛЕЕРА: gen_list_library потом gen_play_from_library."


class _CapturingProvider:
    """Запоминает список сообщений, с которым его позвали."""

    name = "capture"

    def __init__(self) -> None:
        self.calls: list[list[LLMMessage]] = []

    async def complete(
        self,
        messages: Iterable[LLMMessage],
        *,
        tools: Iterable[Mapping[str, Any]] = (),
        **_: Any,
    ) -> LLMResponse:
        self.calls.append(list(messages))
        return LLMResponse(content="ок", finish_reason="stop")

    async def stream(
        self,
        messages: Iterable[LLMMessage],
        *,
        tools: Iterable[Mapping[str, Any]] = (),
        **_: Any,
    ) -> AsyncIterator[LLMChunk]:
        self.calls.append(list(messages))
        yield LLMChunk(content_delta="ок", finish_reason="stop")


def _dialogue_dsm() -> DialogueStateMachine:
    dsm = DialogueStateMachine()
    dsm.on_event(DialogueEvent.WAKE_WORD)
    dsm.on_event(DialogueEvent.STT_RESULT)
    return dsm


def _core(
    provider: _CapturingProvider,
    *,
    skill_prompts: Mapping[str, str] | None = None,
    memory: InMemoryStore | None = None,
    dsm: DialogueStateMachine | None = None,
) -> AgentCore:
    return AgentCore(
        llm=provider,
        tools=FakeToolProvider(),
        memory=memory or InMemoryStore(),
        dsm=dsm or _dialogue_dsm(),
        system_prompt="ты РОББОКС",
        use_streaming=False,
        skill_prompts=skill_prompts,
        history_trim_limit=20,
    )


def _run(
    core: AgentCore,
    text: str = "сыграй бит",
    dsm: DialogueStateMachine | None = None,
) -> None:
    """Прогнать один ход.

    ``process_input`` в конце гонит DSM обратно в IDLE (DIALOGUE_END), а
    LLM зовётся только из DIALOGUE. Поэтому перед вторым и последующими
    ходами машину надо взвести заново — в проде это делает
    ``dialogue_node._on_stt``.
    """
    if dsm is not None:
        dsm.on_event(DialogueEvent.WAKE_WORD)
        dsm.on_event(DialogueEvent.STT_RESULT)
    asyncio.run(
        core.process_input(text, preclassified_event=DialogueEvent.STT_RESULT)
    )


def _index_of(messages: list[LLMMessage], needle: str) -> int:
    for i, msg in enumerate(messages):
        if msg.content and needle in str(msg.content):
            return i
    raise AssertionError(f"{needle!r} не найдено в промпте")


# ── Требование: инструкции скилла рядом с текущим ходом ──────────────────


def test_skill_text_lands_after_the_history() -> None:
    """Задача 2.4 — фрагмент ПОСЛЕ последнего сообщения истории."""
    provider = _CapturingProvider()
    core = _core(provider, skill_prompts={"composer": _COMPOSER_TEXT})
    # История живёт в окне AgentCore, а не в MemoryStore (b5cf5daf:
    # «turns live in memory only, never in SQLite»).
    core._turn_window.extend([
        Turn(role="user", content="старый вопрос"),
        Turn(role="assistant", content="старый ответ"),
    ])
    core.set_active_skill("composer")
    _run(core)

    messages = provider.calls[0]
    assert _index_of(messages, _COMPOSER_TEXT) > _index_of(messages, "старый ответ")


def test_skill_text_survives_a_deep_session() -> None:
    """Задача 2.5 — 20 ходов в окне, фрагмент всё равно после них.

    Это и есть сценарий, из-за которого change существует: чем длиннее
    сессия, тем дальше инструкция с позиции 0 от момента, когда она нужна.
    """
    provider = _CapturingProvider()
    core = _core(provider, skill_prompts={"composer": _COMPOSER_TEXT})
    for i in range(20):
        core._turn_window.append(Turn(role="user", content=f"вопрос {i}"))
        core._turn_window.append(Turn(role="assistant", content=f"ответ {i}"))
    core.set_active_skill("composer")
    _run(core)

    messages = provider.calls[0]
    skill_at = _index_of(messages, _COMPOSER_TEXT)
    last_history_at = max(
        i
        for i, m in enumerate(messages)
        if m.content and ("вопрос" in str(m.content) or "ответ" in str(m.content))
    )
    assert skill_at > last_history_at


def test_skill_text_is_the_last_system_message() -> None:
    """Ближе системного сообщения к реплике юзера уже некуда."""
    provider = _CapturingProvider()
    core = _core(provider, skill_prompts={"composer": _COMPOSER_TEXT})
    core.set_active_skill("composer")
    _run(core, "сыграй бит")

    messages = provider.calls[0]
    system_positions = [i for i, m in enumerate(messages) if m.role == "system"]
    assert _index_of(messages, _COMPOSER_TEXT) == max(system_positions)
    assert messages[-1].role == "user"


def test_system_prompt_stays_at_position_zero() -> None:
    """Инварианты остаются в начале — скилл их не вытесняет."""
    provider = _CapturingProvider()
    core = _core(provider, skill_prompts={"composer": _COMPOSER_TEXT})
    core.set_active_skill("composer")
    _run(core)

    messages = provider.calls[0]
    assert messages[0].role == "system"
    assert "РОББОКС" in str(messages[0].content)


def test_no_active_skill_injects_nothing() -> None:
    """Задача 2.6 — без активации промпт не содержит доменных фрагментов."""
    provider = _CapturingProvider()
    core = _core(provider, skill_prompts={"composer": _COMPOSER_TEXT})
    _run(core)

    joined = " ".join(str(m.content or "") for m in provider.calls[0])
    assert _COMPOSER_TEXT not in joined


def test_empty_skill_prompts_is_byte_for_byte_the_old_behaviour() -> None:
    """Move A выключен → список сообщений совпадает с доскилловым."""
    before = _CapturingProvider()
    _run(_core(before))

    after = _CapturingProvider()
    core = _core(after, skill_prompts={})
    core.set_active_skill("composer")
    _run(core)

    assert [(m.role, m.content) for m in before.calls[0]] == [
        (m.role, m.content) for m in after.calls[0]
    ]


# ── Активация ────────────────────────────────────────────────────────────


def test_switching_skill_switches_the_text() -> None:
    """Смена домена внутри сессии подставляет другой фрагмент."""
    provider = _CapturingProvider()
    dsm = _dialogue_dsm()
    core = _core(
        provider,
        skill_prompts={"composer": _COMPOSER_TEXT, "player": _PLAYER_TEXT},
        dsm=dsm,
    )

    core.set_active_skill("composer")
    _run(core, "сыграй бит")
    core.set_active_skill("player")
    _run(core, "включи тот трек", dsm=dsm)

    assert _COMPOSER_TEXT in " ".join(str(m.content or "") for m in provider.calls[0])
    second = " ".join(str(m.content or "") for m in provider.calls[1])
    assert _PLAYER_TEXT in second
    assert _COMPOSER_TEXT not in second


def test_unknown_skill_does_not_break_the_turn() -> None:
    """Промах активации — не повод обрывать разговор.

    Скилл остаётся неактивным, модель видит полный каталог, как сегодня.
    """
    provider = _CapturingProvider()
    core = _core(provider, skill_prompts={"composer": _COMPOSER_TEXT})
    core.set_active_skill("нет-такого")

    _run(core)

    assert provider.calls, "ход должен был дойти до LLM"
    assert core.active_skill == "нет-такого"


def test_deactivation_restores_the_plain_prompt() -> None:
    provider = _CapturingProvider()
    dsm = _dialogue_dsm()
    core = _core(provider, skill_prompts={"composer": _COMPOSER_TEXT}, dsm=dsm)
    core.set_active_skill("composer")
    _run(core)
    core.set_active_skill(None)
    _run(core, dsm=dsm)

    assert _COMPOSER_TEXT not in " ".join(
        str(m.content or "") for m in provider.calls[1]
    )
    assert core.active_skill == "none"


def test_known_skills_lists_what_has_text() -> None:
    core = _core(
        _CapturingProvider(),
        skill_prompts={"composer": _COMPOSER_TEXT, "player": _PLAYER_TEXT},
    )
    assert core.known_skills() == ("composer", "player")


def test_active_skill_labels_the_prompt_metric() -> None:
    """Метка ``skill`` метрики берётся из активного скилла (фаза 0 + 2)."""
    observed: list[Any] = []
    provider = _CapturingProvider()
    core = AgentCore(
        llm=provider,
        tools=FakeToolProvider(),
        memory=InMemoryStore(),
        dsm=_dialogue_dsm(),
        system_prompt="ты РОББОКС",
        use_streaming=False,
        skill_prompts={"composer": _COMPOSER_TEXT},
        on_prompt=observed.append,
    )
    core.set_active_skill("composer")
    _run(core)

    assert observed
    assert observed[0].skill == "composer"
