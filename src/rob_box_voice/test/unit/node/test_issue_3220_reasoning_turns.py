"""Issue #3220 — нода выставляет ``TURN_REASONING`` на время хода.

Провайдер MiniMax включает thinking, читая ``TURN_REASONING`` из контекста
хода. Здесь проверяем ноду: настоящий ``_run_turn`` (фикстура через
``object.__new__``, как в ``test_issue_1195_tg_source.py``) и настоящий
диспатч DJ-хода; ``process_input`` записывает флаг в момент вызова LLM.

* обычный голосовой ход → ``False``; заказ музыки → ``True``;
* DJ-переход по тику → ``True``; ретрай Bug B (``from_tick=False``) → ``False``;
* синтетический ретрай → ``False``; после хода флаг сброшен.
"""

from __future__ import annotations

import asyncio
import threading
from types import SimpleNamespace
from unittest.mock import AsyncMock, MagicMock

import pytest

from rob_box_harness.providers.reasoning import TURN_REASONING
from rob_box_voice import dialogue_node as dialogue_node_module
from rob_box_voice.core.music_guard import MusicGuard
from rob_box_voice.dialogue_node import DialogueNode


def _turn_node():
    """Минимальная нода для настоящего ``_run_turn`` (см. test_issue_1195)."""
    n = object.__new__(DialogueNode)
    n._task_lock = threading.Lock()
    n._run_cancelled = False
    n._babble_retry_used = False
    n._music_guard = MusicGuard()
    n._pending_music_cleanup = False
    n._speaker_id_enabled = False
    n._handle_speaker_turn = MagicMock()
    n._apply_speaker_identity = MagicMock()
    n._build_dynamic_system_context = MagicMock(return_value="<system_context/>")
    n._llm = MagicMock()
    n._speak_direct = MagicMock()
    n._active_batches = set()
    n._dsm = MagicMock()
    n._dsm.current_state = "idle"
    n._publish_state = MagicMock()
    n._apply_music_guard = MagicMock()
    n._apply_tool_skipped_guard = MagicMock(return_value=False)
    n._publish_music_cleanup = MagicMock()
    n._maybe_record_session_end = MagicMock()
    n._track_mode_music_active = False
    n._action_claim_retry_used = False
    n._code_speech_retry_used = False
    logger = MagicMock()
    n.get_logger = lambda: logger
    n._handle_result = MagicMock()

    seen: list[bool] = []

    async def _process_input(*_a, **_kw):
        seen.append(TURN_REASONING.get())
        return SimpleNamespace(spoken_text="ок", tools_called=(), error=None)

    n._core = MagicMock()
    n._core.process_input = AsyncMock(side_effect=_process_input)
    return n, seen


@pytest.mark.parametrize(
    ("text", "kwargs", "expected"),
    [
        ("как тебя зовут", {"raw_user_command": "как тебя зовут"}, False),
        ("сыграй что-нибудь про осень", {"raw_user_command": "сыграй что-нибудь про осень"}, True),
        ("[DJ_AUTO — ПЕРЕХОД #2] смени трек", {"is_dj_auto": True, "dj_transition": True}, True),
        ("[CRITICAL] DJ retry — call compose_music", {"is_dj_auto": True}, False),
        ("[CRITICAL] retry", {"is_synthetic": True, "raw_user_command": "сыграй про осень"}, False),
    ],
)
def test_run_turn_exposes_reasoning_decision_to_the_llm_call(text, kwargs, expected):
    n, seen = _turn_node()

    asyncio.run(n._run_turn(text, **kwargs))

    assert seen == [expected], f"{text!r} {kwargs!r}: LLM видела TURN_REASONING={seen!r}"


def test_flag_is_reset_after_the_turn():
    n, _seen = _turn_node()

    async def _turn_then_read():
        await n._run_turn("[DJ_AUTO — ПЕРЕХОД #2]", is_dj_auto=True, dj_transition=True)
        return TURN_REASONING.get()

    assert asyncio.run(_turn_then_read()) is False


# ── диспатч DJ-хода ─────────────────────────────────────────────────


def _dispatch_node(monkeypatch):
    n = object.__new__(DialogueNode)
    n.get_logger = lambda: MagicMock()
    n._loop = None
    n._pending_music_cleanup = False
    n._session_started_at = None
    n._music_guard = MagicMock()
    calls: list[dict] = []

    def fake_run_turn(user_input, **kwargs):
        calls.append({"user_input": user_input, **kwargs})
        return None

    n._run_turn = fake_run_turn
    monkeypatch.setattr(
        dialogue_node_module.asyncio,
        "run_coroutine_threadsafe",
        lambda coro, loop: None,
    )
    return n, calls


@pytest.mark.parametrize(("from_tick", "expected"), [(True, True), (False, False)])
def test_dj_dispatch_marks_only_tick_as_fresh_transition(monkeypatch, from_tick, expected):
    n, calls = _dispatch_node(monkeypatch)

    n._dispatch_dj_turn("[DJ_AUTO — ПЕРЕХОД #2]", from_tick)

    assert calls[0]["is_dj_auto"] is True
    assert calls[0]["dj_transition"] is expected


def test_user_dispatch_is_never_a_dj_transition(monkeypatch):
    n, calls = _dispatch_node(monkeypatch)

    n._dispatch_turn("сыграй про осень", raw_user_command="сыграй про осень")

    assert calls[0]["dj_transition"] is False
