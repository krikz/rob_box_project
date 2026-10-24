"""ADR-0150 PR-7c ч.4б (#3700) — жёсткие стопы музыки v1 идут владельцу плеера, а не топиком уборки.

Раньше «новая сессия» (#2835) и «стоп-команда + пустой ответ LLM» (#992 Bug C) публиковали причину в топик уборки
музыки, а ``mcp_server`` по ней снимал деку. Теперь оба стопа — тем же путём, что Tier-1 «выключи музыку»:
``dj_set(stop)`` + ``stop_music`` исполнителем MCP-тулов (``tier1_host.runner_of(node).stop_music()``).
Мягкой уборки после речи больше нет: ход без стопа музыку не трогает (``test_issue_3174_cleanup_spares_dj_set``).
"""

from __future__ import annotations

import asyncio
import json
import threading
from unittest.mock import MagicMock

import pytest

from rob_box_harness.core.agent_core import DialogResult
from rob_box_llm.provider import ToolResult
from rob_box_voice.core.llm_skip_reasons import new_llm_skip_counter
from rob_box_voice.dialogue_node import DialogueNode
import rob_box_voice.dialogue_node as dialogue_node_module

STOP_BY_OWNER = [("dj_set", {"action": "stop"}), ("stop_music", {})]


class _FakeExecutor:
    def __init__(self) -> None:
        self.calls = []

    def begin_turn(self) -> None:
        pass

    async def execute(self, call):
        self.calls.append((call.name, dict(call.arguments)))
        return ToolResult(tool_call_id=call.id, content=json.dumps({"success": True}), is_error=False)


@pytest.fixture
def run_plans(monkeypatch):
    """Перехватить ``run_coroutine_threadsafe`` и выполнить корутины сразу."""
    pending = []
    monkeypatch.setattr(dialogue_node_module.asyncio, "run_coroutine_threadsafe",
                        lambda coro, loop: pending.append(coro) or MagicMock())

    def _drain():
        loop = asyncio.new_event_loop()
        try:
            while pending:
                loop.run_until_complete(pending.pop(0))
        finally:
            loop.close()

    return _drain


def _reset_node() -> DialogueNode:
    """Узел с атрибутами, которые трогает ``_reset_dialogue_session`` (как в test_new_session_resets_music)."""
    n = object.__new__(DialogueNode)
    n.get_logger = lambda: MagicMock()
    n._cancel_run = MagicMock()
    n._speech_client_pubs = {"cancel": MagicMock()}
    n._pending_backlog_flush = False
    n._dsm = MagicMock()
    n._speaker_lock = threading.Lock()
    n._current_speaker = {"is_known": False}
    n._speaker_by_text = {}
    n._speaker_tracker = MagicMock()
    n._maybe_record_session_end = MagicMock()
    n._publish_state = MagicMock()
    n._publish_response = MagicMock()
    n._tool_executor = _FakeExecutor()
    n._loop = MagicMock()
    return n


def _turn_node() -> DialogueNode:
    """Узел для ``_handle_result`` (форма как в test_issue_3174_cleanup_spares_dj_set)."""
    n = object.__new__(DialogueNode)
    logger = MagicMock()
    n.get_logger = lambda: logger
    n._response_pub = MagicMock()
    n._state_pub = MagicMock()
    n._sound_trigger_pub = MagicMock()
    n._speech_client_pubs = {"cancel": MagicMock()}
    n._dsm = MagicMock()
    n._dsm.current_state = MagicMock()
    n._active_tg_chat_id = None
    n._effects = MagicMock()
    n._verbose_llm = False
    n._run_task = None
    n._task_lock = MagicMock()
    n._startup_greeting_fired = False
    n._dispatch_turn = MagicMock()
    n._music_player_state = None
    n._llm_skipped_counter = new_llm_skip_counter()
    n._cancel_run = MagicMock()
    n._speak_direct = MagicMock()
    n._tool_executor = _FakeExecutor()
    n._loop = MagicMock()
    n._core = MagicMock()
    return n


def test_new_session_stops_the_music_by_the_player_owner(run_plans):
    n = _reset_node()

    n._reset_dialogue_session()
    run_plans()

    assert n._tool_executor.calls == STOP_BY_OWNER


def test_stop_command_with_an_empty_llm_reply_stops_the_music_by_the_player_owner(run_plans):
    """#992 Bug C: «выключи музыку», а LLM вернула пустоту без тулов — музыку всё равно снимаем."""
    n = _turn_node()

    n._handle_result(DialogResult(spoken_text="", tools_called=[], finish_reason="stop"), user_input="выключи музыку")
    run_plans()

    assert n._tool_executor.calls == STOP_BY_OWNER


def test_empty_reply_without_a_stop_command_does_not_touch_the_music(run_plans):
    n = _turn_node()

    n._handle_result(DialogResult(spoken_text="", tools_called=[], finish_reason="stop"), user_input="как дела")
    run_plans()

    assert n._tool_executor.calls == []


def test_without_a_tool_executor_the_stop_is_reported_not_raised():
    n = _reset_node()
    n._tool_executor = None

    assert n._stop_music_by_owner("new_session") is False
