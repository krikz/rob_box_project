"""Issue #3134 — адаптер ``DialogueNode``: медиакоманды исполняет код до LLM.

Харнесс без ROS2 (``object.__new__``, как ``test_command_intent_gate``):
реальные ``_on_stt`` → ``SttAdmission`` → ``MediaCommandStep`` →
``_route_media_command``; мок — только исполнитель тулов (вместо
``SchedulerToolExecutor`` → ``/mcp/execute``), ``_dispatch_turn`` (вход
LLM-хода) и ``_speak_direct`` (TTS). Корутина плана перехватывается у
``asyncio.run_coroutine_threadsafe`` и прогоняется в тестовом loop'е.
"""

from __future__ import annotations

import asyncio
import json
from unittest.mock import MagicMock

import pytest

from rob_box_llm.provider import ToolResult

from rob_box_voice.core.dialogue_text import DEFAULT_WAKE_WORDS
from rob_box_voice.core.llm_skip_reasons import new_llm_skip_counter
from rob_box_voice.core.media_router import NOT_STARTED_TEXT, NOTHING_PLAYING_TEXT
from rob_box_voice.core.music_player_state import MusicPlayerState
from rob_box_voice.dialogue_node import DialogueNode
import rob_box_voice.dialogue_node as dialogue_node_module


class _FakeExecutor:
    """Стоит на месте ``SchedulerToolExecutor``: тот же ``execute(ToolCall)``."""

    def __init__(self, fail: bool = False) -> None:
        self.calls = []
        self._fail = fail

    async def execute(self, call):
        self.calls.append((call.name, dict(call.arguments)))
        return ToolResult(
            tool_call_id=call.id,
            content=json.dumps({"success": not self._fail}),
            is_error=self._fail,
        )


def _make_node(*, playing=False, dj=False, track=None, fail=False, state_name="IDLE"):
    n = object.__new__(DialogueNode)
    logger = MagicMock()
    n.get_logger = lambda: logger
    n._wake_words = list(DEFAULT_WAKE_WORDS)
    n._command_intent_gate_enabled = False
    n._speaker_by_text = {}
    n._llm_skipped_counter = new_llm_skip_counter()
    n._maybe_log_skip_summary = MagicMock()
    n._dsm = MagicMock()
    n._dsm.current_state = MagicMock()
    n._dsm.current_state.name = state_name
    n._cancel_run = MagicMock()
    n._sound_trigger_pub = MagicMock()
    n._publish_state = MagicMock()
    n._dispatch_turn = MagicMock()
    n._speak_direct = MagicMock()
    n._verbose_llm = False
    n._active_tg_chat_id = None
    # Снимок плеера /voice/music/state (#3133) — источник «играет», DJ-сета и
    # названия трека (поле ``dj`` движка v2, ADR-0149 §2.3).
    n._music_player_state = MusicPlayerState(
        state="playing" if playing else "idle", dj=dj,
        dj_info={"enabled": dj, "title": track} if track else {"enabled": dj},
    )
    n._scheduler_executor = _FakeExecutor(fail=fail)
    n._loop = MagicMock()
    return n


@pytest.fixture
def run_plans(monkeypatch):
    """Перехватить ``run_coroutine_threadsafe`` и выполнить корутину сразу."""
    pending = []

    def _capture(coro, loop):  # noqa: ARG001
        pending.append(coro)
        return MagicMock()

    monkeypatch.setattr(
        dialogue_node_module.asyncio, "run_coroutine_threadsafe", _capture
    )

    def _drain():
        loop = asyncio.new_event_loop()
        try:
            while pending:
                loop.run_until_complete(pending.pop(0))
        finally:
            loop.close()

    return _drain


def _stt(node: DialogueNode, text: str) -> None:
    msg = MagicMock()
    msg.data = text
    node._on_stt(msg)


STATES = {
    "playing": {"playing": True, "track": "Still Dre"},
    "quiet": {},
    "dj": {"playing": True, "dj": True, "track": "Still Dre"},
}


# ── громкость × состояние ──────────────────────────────────────────────


@pytest.mark.parametrize(
    ("phrase", "action"),
    [
        ("Робот, играй громче", "louder"),
        ("Робот, потише", "quieter"),
        ("Робот, на максимум", "max"),
    ],
)
@pytest.mark.parametrize("state", ["playing", "quiet", "dj"])
def test_volume_is_executed_by_code_not_llm(run_plans, phrase, action, state):
    n = _make_node(**STATES[state])
    _stt(n, phrase)
    run_plans()
    n._dispatch_turn.assert_not_called()  # LLM не вызывается
    assert n._llm_skipped_counter["media_command"] == 1
    if state == "quiet":
        assert n._scheduler_executor.calls == []
        n._speak_direct.assert_called_once_with(NOTHING_PLAYING_TEXT)
    else:
        assert n._scheduler_executor.calls == [("set_music_volume", {"action": action})]
        n._speak_direct.assert_called_once()
    n._cancel_run.assert_not_called()


def test_track_name_louder_while_that_track_plays(run_plans):
    n = _make_node(playing=True, track="В пещере горного короля")
    _stt(n, "Робот, горный король погромче")
    run_plans()
    n._dispatch_turn.assert_not_called()
    assert n._scheduler_executor.calls == [("set_music_volume", {"action": "louder"})]


def test_volume_tool_failure_is_said_honestly(run_plans):
    n = _make_node(playing=True, fail=True)
    _stt(n, "Робот, громче")
    run_plans()
    n._speak_direct.assert_called_once()
    assert "Не получилось" in n._speak_direct.call_args[0][0]


# ── стоп × состояние ───────────────────────────────────────────────────


@pytest.mark.parametrize("state", ["playing", "quiet", "dj"])
def test_stop_is_executed_by_code(run_plans, state):
    n = _make_node(**STATES[state])
    _stt(n, "Робот, выключи музыку")
    run_plans()
    n._dispatch_turn.assert_not_called()
    assert n._scheduler_executor.calls == [("dj_set", {"action": "stop"}), ("stop_music", {})]
    n._cancel_run.assert_called_once()
    n._speak_direct.assert_called_once()


# ── «ты диджей X» × состояние ──────────────────────────────────────────


@pytest.mark.parametrize("state", ["playing", "quiet", "dj"])
def test_dj_persona_starts_engine_set_without_llm(run_plans, state):
    # ADR-0149 PR-13a: ни превью compose_music, ни set_dj_mode — dj_set движка v2;
    # фраза об успехе только по ``started`` (здесь событий нет → честное «не заиграла»).
    n = _make_node(**STATES[state])
    _stt(n, "Робот, ты диджей Снупдог, давай сет")
    run_plans()
    n._dispatch_turn.assert_not_called()
    assert n._scheduler_executor.calls == [
        ("dj_set", {"action": "start", "persona": "диджей Снупдог"})
    ]
    n._speak_direct.assert_called_once_with(NOT_STARTED_TEXT)


def test_open_dj_request_goes_to_llm_without_tools(run_plans):
    n = _make_node()
    text = "Робот, ты диджей Пёс, сыграй Still Dre и Next Episode"
    _stt(n, text)
    run_plans()
    assert n._scheduler_executor.calls == []
    n._dispatch_turn.assert_called_once()
    n._speak_direct.assert_not_called()


# ── не медиакоманда — LLM как раньше ───────────────────────────────────


@pytest.mark.parametrize("state", ["playing", "quiet", "dj"])
@pytest.mark.parametrize(
    "phrase",
    [
        # «сыграй в пещере горного короля» — заказ по имени (#3176), путь
        # через lookup_melody: test_issue_3176_play_named_node.py.
        "Робот, расскажи анекдот",
        "Робот, говори громче",
    ],
)
def test_not_a_media_command_goes_to_llm(run_plans, state, phrase):
    n = _make_node(**STATES[state])
    _stt(n, phrase)
    run_plans()
    n._dispatch_turn.assert_called_once()
    assert n._scheduler_executor.calls == []
    assert n._llm_skipped_counter["media_command"] == 0


# ── каждый ход: продолжение разговора, DJ-режим, Telegram ──────────────


def test_router_fires_mid_dialogue(run_plans):
    n = _make_node(playing=True, state_name="DIALOGUE")
    _stt(n, "Робот, потише")
    run_plans()
    n._dispatch_turn.assert_not_called()
    assert n._scheduler_executor.calls == [("set_music_volume", {"action": "quieter"})]


def test_router_fires_on_telegram_without_wake_word(run_plans):
    n = _make_node(playing=True)
    _stt(n, "[TG:42] громче")
    run_plans()
    n._dispatch_turn.assert_not_called()
    assert n._scheduler_executor.calls == [("set_music_volume", {"action": "louder"})]


def test_without_mcp_tools_falls_back_to_llm(run_plans):
    n = _make_node(playing=True)
    n._scheduler_executor = None  # tool_provider=fake/none
    _stt(n, "Робот, громче")
    run_plans()
    n._dispatch_turn.assert_called_once()


def test_media_state_is_the_single_accessor():
    n = _make_node(playing=True, dj=True, track="Still Dre")
    state = n._media_state()
    assert (state.music_playing, state.dj_enabled, state.track_name) == (
        True, True, "Still Dre"
    )
    n._music_player_state = MusicPlayerState(state="idle")
    assert n._media_state().music_playing is False
    assert n._media_state().track_name is None  # название — только пока играет

