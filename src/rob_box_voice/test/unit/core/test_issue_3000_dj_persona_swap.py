"""ADR-0129 (issue #3000) — смена DJ-сета вытесняет прошлые сеты из контекста.

Живой повтор 01.10.2026 10:47: сет «ВосьмиБитный монмтр / денди» закончился
(финал, DJ Mode OFF), реплика «[TG] Ты Диджй Ускоглазый и у нас сегодня
вечеринка любителей Азидтской музыки» пошла в LLM (опечатка «Диджй» мимо
роутера), модель вызвала ``set_dj_mode(theme='вечеринка любителей денди',
persona='ВосьмиБитный монмтр')`` — из окна, где пять старых «включи диджей
сет на тему …» и последний обмен прошлого сета.

Здесь — настоящие ``DJModeController`` и ``AgentCore`` (с фейковой LLM):
что уходит в запрос модели после смены сета.
"""

from __future__ import annotations

import asyncio
import json
import logging
from typing import Any

from rob_box_harness.core.agent_core import AgentCore
from rob_box_harness.core.dialogue_state_machine import (
    DialogueEvent,
    DialogueStateMachine,
)
from rob_box_harness.memory import Turn
from rob_box_llm.provider import LLMResponse
from rob_box_voice.core.dj_mode import DJHook, DJModeController
from rob_box_voice.core.dj_set_boundary import (
    DJSetBoundary,
    apply_dj_mode_message,
    dj_state_lines,
    keep_current_set_turns,
    settle_dj_set_boundary,
)

OLD_PERSONA = "ВосьмиБитный монмтр"
OLD_THEME = "вечеринка любителей поиграть в денди погнали"
NEW_PERSONA = "Диджей ПанАзиат"
NEW_THEME = "вечеринка любителей азиатской музыки"
DJ_TOOLS = {"tools_called": ["compose_music", "set_dj_mode"]}


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


def _core(llm: _FakeLLM) -> AgentCore:
    return AgentCore(
        llm=llm, tools=_NoTools(), memory=_NoMemory(),
        dsm=DialogueStateMachine(), system_prompt="ПРОМПТ",
        history_trim_limit=40,
    )


def _dj() -> DJModeController:
    hook = DJHook(
        dispatch=lambda prompt, from_tick=False: None,
        is_active=lambda: False,
        is_dialogue_active=lambda: False,
    )
    return DJModeController(hook=hook, logger=logging.getLogger("test"))


def _live_window() -> list[Turn]:
    """Окно с робота 01.10 10:47 (сокращено): прошлые сеты + не-DJ ходы."""
    turns: list[Turn] = []
    for theme in ("киберпанк", "море и чайки", "весенний лес"):
        turns.append(Turn(role="user", content=f"включи диджей сет на тему {theme}"))
        turns.append(Turn(role="assistant", content=f"Сет «{theme}» стартовал.",
                          metadata=dict(DJ_TOOLS)))
        turns.append(Turn(role="user", content="останови музыку"))
        turns.append(Turn(role="assistant", content="Выключил музыку.",
                          metadata={"tools_called": ["stop_music"]}))
    turns.append(Turn(role="user", content="какой сегодня праздник"))
    turns.append(Turn(role="assistant", content="Сегодня День молодёжи."))
    turns.append(Turn(role="user", content=(
        f"Ты диджей {OLD_PERSONA} и у нас сегодня вечеринка любителей "
        "поиграть в денди погнали")))
    turns.append(Turn(role="assistant", content=f"Я диджей {OLD_PERSONA}, запускаю сет.",
                      metadata=dict(DJ_TOOLS)))
    return turns


def _set_dj(boundary: DJSetBoundary, dj: DJModeController, **payload: Any) -> None:
    apply_dj_mode_message(boundary, dj, json.dumps(payload))


def _sent_text(llm: _FakeLLM) -> str:
    return "\n".join(str(m.content) for m in llm.calls[-1])


def _human_turn(core: AgentCore, boundary: DJSetBoundary, dj: DJModeController,
                text: str) -> None:
    """Ход человека так, как его собирает нода: settle → штамп → process_input."""
    settle_dj_set_boundary(boundary, dj, core)
    dynamic = "\n".join(["<system_context>", *dj_state_lines(boundary, dj), "</system_context>"])
    core._dsm.on_event(DialogueEvent.WAKE_WORD)
    asyncio.run(core.process_input(text, dynamic_system=dynamic))


class TestLiveRepeatSetEndedThenNewRequest:
    """Сет закончился → следующий ход человека не видит прошлых сетов."""

    def _ended_set(self) -> tuple:
        llm, dj, boundary = _FakeLLM(), _dj(), DJSetBoundary()
        core = _core(llm)
        core._turn_window.extend(_live_window())
        _set_dj(boundary, dj, enabled=True, persona=OLD_PERSONA, theme=OLD_THEME)
        _set_dj(boundary, dj, enabled=False)  # финал сета
        return llm, dj, boundary, core

    def test_old_sets_are_not_in_request(self) -> None:
        llm, dj, boundary, core = self._ended_set()
        _human_turn(core, boundary, dj,
                    "[TG] Ты Диджй Ускоглазый и у нас сегодня вечеринка любителей Азидтской музыки")
        sent = _sent_text(llm)
        assert OLD_PERSONA not in sent
        assert "денди" not in sent
        assert "киберпанк" not in sent and "весенний лес" not in sent

    def test_non_dj_turns_survive(self) -> None:
        llm, dj, boundary, core = self._ended_set()
        _human_turn(core, boundary, dj, "[TG] Ты Диджй Ускоглазый")
        sent = _sent_text(llm)
        assert "какой сегодня праздник" in sent
        assert "Выключил музыку." in sent

    def test_stamp_says_no_set_and_take_theme_from_current_utterance(self) -> None:
        llm, dj, boundary, core = self._ended_set()
        _human_turn(core, boundary, dj, "[TG] Ты Диджй Ускоглазый")
        sent = _sent_text(llm)
        assert '<dj_state enabled="no">' in sent
        assert "ТОЛЬКО из его текущей реплики" in sent


class TestPersonaChangeMidSet:
    """ADR-0129 §1.1: «Ля-Классик» → «8-битный монстр» внутри сета."""

    def test_only_current_set_exchange_kept_and_stamped(self) -> None:
        llm, dj, boundary = _FakeLLM(), _dj(), DJSetBoundary()
        core = _core(llm)
        _set_dj(boundary, dj, enabled=True, persona=OLD_PERSONA, theme=OLD_THEME)
        core._turn_window.extend(_live_window())
        _human_turn(core, boundary, dj, "кто за пультом")
        # Смена персоны и темы: обмен «ты диджей ПанАзиат» с set_dj_mode.
        _set_dj(boundary, dj, enabled=True, persona=NEW_PERSONA, theme=NEW_THEME)
        core.record_external_turn(
            "ты диджей ПанАзиат, вечеринка азиатской музыки",
            f"Теперь я {NEW_PERSONA}.", ["set_dj_mode"],
        )
        _human_turn(core, boundary, dj, "что сейчас играет")
        sent = _sent_text(llm)
        assert OLD_PERSONA not in sent
        assert f"Теперь я {NEW_PERSONA}." in sent
        assert '<dj_state enabled="yes">' in sent
        assert f"<persona>{NEW_PERSONA}</persona>" in sent
        assert f"<theme>{NEW_THEME}</theme>" in sent
        assert "прошлые сеты завершены" in sent


class TestBoundaryDetection:
    def test_transition_echo_is_not_a_boundary(self) -> None:
        dj, boundary = _dj(), DJSetBoundary()
        _set_dj(boundary, dj, enabled=True, persona=OLD_PERSONA, theme=OLD_THEME)
        assert boundary.take(dj.state) is True
        # DJ_AUTO повторяет set_dj_mode с теми же темой/персоной и планом.
        _set_dj(boundary, dj, enabled=True, persona=OLD_PERSONA, theme=OLD_THEME,
                plan="Трек 1: x\nТрек 2: y")
        assert boundary.take(dj.state) is False

    def test_silent_reset_by_stop_command_is_a_boundary(self) -> None:
        """#2897: стоп-команда гасит DJ мимо топика, эхо уже ничего не меняет."""
        dj, boundary = _dj(), DJSetBoundary()
        _set_dj(boundary, dj, enabled=True, persona=OLD_PERSONA, theme=OLD_THEME)
        boundary.take(dj.state)
        dj.reset_silently()
        _set_dj(boundary, dj, enabled=False)  # эхо _publish_dj_off
        assert boundary.take(dj.state) is True

    def test_no_stamp_before_any_set(self) -> None:
        assert dj_state_lines(DJSetBoundary(), _dj()) == []

    def test_no_boundary_no_clear(self) -> None:
        llm, dj, boundary = _FakeLLM(), _dj(), DJSetBoundary()
        core = _core(llm)
        core._turn_window.extend(_live_window())
        assert settle_dj_set_boundary(boundary, dj, core) is False
        assert len(core._turn_window) == len(_live_window())


class TestKeepFilter:
    def test_dj_on_keeps_latest_set_exchange_only(self) -> None:
        kept = keep_current_set_turns(True)(_live_window())
        contents = [t.content for t in kept]
        assert f"Я диджей {OLD_PERSONA}, запускаю сет." in contents
        assert not any("включи диджей сет" in c for c in contents)

    def test_dj_off_drops_all_set_exchanges(self) -> None:
        kept = keep_current_set_turns(False)(_live_window())
        assert not any("set_dj_mode" in (t.metadata or {}).get("tools_called", ())
                       for t in kept)
        assert [t.content for t in kept][-2:] == [
            "какой сегодня праздник", "Сегодня День молодёжи."]

    def test_pending_retry_tools_on_user_turn_count(self) -> None:
        """discard_last_reply переносит тулы на user-ход — он тоже граница."""
        turns = [Turn(role="user", content="ты диджей X", metadata=dict(DJ_TOOLS)),
                 Turn(role="user", content="привет")]
        assert [t.content for t in keep_current_set_turns(False)(turns)] == ["привет"]
