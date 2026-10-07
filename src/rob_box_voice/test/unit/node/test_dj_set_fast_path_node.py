"""Заказ сета словами человека: ``_on_stt`` → роутер → ``dj_set`` без хода LLM (живой замер 06.10 19:08 UTC).

Харнесс без ROS2, как ``test_issue_3134_media_router_node``: настоящие ``_on_stt`` → ``SttAdmission`` →
``_route_media_command`` → ``_execute_media_plan``; мок — исполнитель тулов (с настоящим ``MusicTurn``),
``_dispatch_turn`` (вход хода LLM) и ``_speak_direct`` (TTS).

Проверяем: LLM не зовётся; скрытый контекст хода для ``dj_set`` (``_mcp_turn_context``) — ЭТОЙ реплики, а не
прошлого хода LLM; повторная реплика — новый ход и чистый ``MusicTurn``; фразу об успехе/отказе строит код.
"""

from __future__ import annotations

import asyncio
import json
from unittest.mock import MagicMock

import pytest

from rob_box_llm.provider import ToolResult
from rob_box_voice.core.dialogue_text import DEFAULT_WAKE_WORDS
from rob_box_voice.core.llm_skip_reasons import new_llm_skip_counter
from rob_box_voice.core.media_phrases import DJ_FAIL_TEXT
from rob_box_voice.core.media_router import NOT_STARTED_TEXT
from rob_box_voice.core.music_player_state import MusicPlayerState
from rob_box_voice.core.music_turn import MusicTurn
from rob_box_voice.dialogue_node import DialogueNode
import rob_box_voice.dialogue_node as dialogue_node_module

LIVE_1908 = ("Робот, ты диджей 8битный и нас сегодня вечеринка любителей денди и классической музыки "
             "замути сэт на 30 минут")
#: Прошлый ход LLM: его реплика и длина не должны доехать до dj_set команды.
OLD_TURN_TEXT = "Робот, сыграй увертюру 1812 сет на 10 треков"


class _Executor:
    """На месте ``SchedulerToolExecutor``: ``execute``, ``begin_turn`` и ``MusicTurn``, как у настоящего.

    Скрытые аргументы хода снимает с ноды в момент вызова тула — как ``LLMToolCallAdapter`` (``turn_context``).
    """

    def __init__(self, node: DialogueNode, *, fail: bool = False) -> None:
        self._node = node
        self._fail = fail
        self.calls = []
        self.music_turn = MusicTurn()
        self.turns = 0

    def begin_turn(self) -> None:
        self.turns += 1
        self.music_turn.reset()

    async def execute(self, call):
        hidden = self._node._mcp_turn_context()
        keys = ("heard_text", "heard_tracks", "turn_id")
        self.calls.append((call.name, dict(call.arguments), {k: hidden[k] for k in keys}))
        body = {"success": False, "error": "not_started"} if self._fail else {"ok": True, "track_id": "s1:01"}
        content = json.dumps(body)
        self.music_turn.record(call.name, call.arguments, is_error=False, content=content)
        return ToolResult(tool_call_id=call.id, content=content, is_error=False)


def _make_node(*, fail: bool = False) -> DialogueNode:
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
    n._dsm.current_state.name = "IDLE"
    n._cancel_run = MagicMock()
    n._sound_trigger_pub = MagicMock()
    n._publish_state = MagicMock()
    n._dispatch_turn = MagicMock()
    n._speak_direct = MagicMock()
    n._verbose_llm = False
    n._active_tg_chat_id = None
    n._music_player_state = MusicPlayerState(state="idle")
    n._loop = MagicMock()
    n._core = MagicMock()
    # Контекст прошлого хода LLM (выставил его _run_turn).
    n._turn_id = "old-llm-turn"
    n._turn_heard_text = OLD_TURN_TEXT
    n._turn_set_tracks = 10
    n._scheduler_executor = _Executor(n, fail=fail)
    return n


@pytest.fixture
def run_plans(monkeypatch):
    """Перехватить ``run_coroutine_threadsafe`` и выполнить корутину сразу."""
    pending = []

    def _capture(coro, loop):  # noqa: ARG001
        pending.append(coro)
        return MagicMock()

    monkeypatch.setattr(dialogue_node_module.asyncio, "run_coroutine_threadsafe", _capture)

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


def test_live_phrase_starts_dj_set_without_llm(run_plans):
    n = _make_node()
    _stt(n, LIVE_1908)
    run_plans()
    n._dispatch_turn.assert_not_called()  # ни load_skill, ни второго раунда LLM
    assert n._llm_skipped_counter["media_command"] == 1
    [(name, args, hidden)] = n._scheduler_executor.calls
    assert (name, args) == ("dj_set", {"action": "start", "persona": "диджей 8битный", "tracks": 24})
    # Скрытый контекст — этой реплики: тему (денди + классика) dj_set выделит из неё, длина 24.
    assert "денди" in hidden["heard_text"] and "классической" in hidden["heard_text"]
    assert hidden["heard_tracks"] == 24
    assert hidden["turn_id"] not in (None, "old-llm-turn")
    n._cancel_run.assert_called_once()
    # Событий плеера в харнессе нет → честное «не заиграла», а не «включаю» (A14).
    n._speak_direct.assert_called_once_with(NOT_STARTED_TEXT)


def test_repeated_phrase_is_a_new_turn_and_clean_music_turn(run_plans):
    n = _make_node()
    for _ in range(2):
        _stt(n, LIVE_1908)
        run_plans()
    executor = n._scheduler_executor
    first, second = (hidden["turn_id"] for _name, _args, hidden in executor.calls)
    assert first != second
    assert executor.turns == 2
    assert len(executor.music_turn.launches) == 1  # запуск прошлой команды забыт границей хода
    assert executor.music_turn.speech_allowed()
    n._dispatch_turn.assert_not_called()


def test_refused_set_gets_the_honest_code_phrase(run_plans):
    n = _make_node(fail=True)
    _stt(n, LIVE_1908)
    run_plans()
    n._speak_direct.assert_called_once_with(DJ_FAIL_TEXT)
    n._dispatch_turn.assert_not_called()


def test_unrecognised_phrase_goes_to_llm_and_keeps_llm_turn_context(run_plans):
    n = _make_node()
    _stt(n, "Робот, ты диджей вася замути сэт и поставь still dre")
    run_plans()
    n._dispatch_turn.assert_called_once()
    assert n._scheduler_executor.calls == []
    assert (n._turn_id, n._turn_heard_text, n._turn_set_tracks) == ("old-llm-turn", OLD_TURN_TEXT, 10)


def test_volume_over_llm_turn_does_not_touch_its_context(run_plans):
    n = _make_node()
    n._music_player_state = MusicPlayerState(state="playing")
    _stt(n, "Робот, громче")
    run_plans()
    assert [c[0] for c in n._scheduler_executor.calls] == ["set_music_volume"]
    assert n._scheduler_executor.calls[0][2]["turn_id"] == "old-llm-turn"
    assert n._scheduler_executor.turns == 0
