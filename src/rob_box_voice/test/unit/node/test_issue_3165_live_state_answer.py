"""Issue #3165 — живые сценарии 29.09.2026 через ``DialogueNode._handle_result``.

Прогон 00:10–00:13 UTC (деплой develop ``1652cbf7d``), фразы без
пунктуации — как отдаёт Yandex STT::

    Факт 1. «Робот что сейчас играет» (плеер: idle)
      spoken='Сейчас тишина — ничего не играет.' tools=[]
      Bug E music_state → retry → retry → «Не получилось выполнить…»
    Факт 2. «Робот выключи музыку» → роутер stop_music ✅
            «Робот а какую музыку ты сейчас включал»
      spoken='Одиннадцать секунд назад я останавливал трек без названия…'
      [issue 2559] phantom-action → retry, ответ на 12,6 с вместо ~7

Уровень — integration, как ``test_issue_2780_memory_save_fallback.py``:
настоящий ``DialogueNode`` через ``object.__new__``, настоящая цепочка
гуардов ``_handle_result``. Ретрай = вызов ``_dispatch_turn``; что ушло в
TTS — ``_response_pub``. Роутер — настоящий ``_route_media_command``
(исполнитель тулов подменён, как в ``test_issue_3134_media_router_node``).
"""

from __future__ import annotations

import asyncio
import json
from unittest.mock import MagicMock

import pytest

from rob_box_harness.core.agent_core import DialogResult
from rob_box_llm.provider import ToolResult
from rob_box_voice.core.dialogue_guards import ACTION_CLAIM_NOTHING_DONE_TEXT
from rob_box_voice.core.llm_skip_reasons import new_llm_skip_counter
from rob_box_voice.core.music_guard import MusicGuard
from rob_box_voice.core.music_player_state import MusicPlayerState
from rob_box_voice.dialogue_node import DialogueNode
import rob_box_voice.dialogue_node as dialogue_node_module

LIVE_QUESTION = "[Speaker:unknown] что сейчас играет"
LIVE_IDLE_ANSWER = "Сейчас тишина — ничего не играет."
LIVE_WHAT_DID_YOU_PLAY = "[Speaker:unknown] а какую музыку ты сейчас включал"
LIVE_AFTER_STOP_ANSWER = (
    "Одиннадцать секунд назад я останавливал трек без названия, сейчас тишина."
)
FAILURE_PHRASE = "Не получилось выполнить"


class _FakeExecutor:
    """Вместо ``SchedulerToolExecutor`` → ``/mcp/execute``."""

    def __init__(self) -> None:
        self.calls = []

    async def execute(self, call):
        self.calls.append(call.name)
        return ToolResult(
            tool_call_id=call.id,
            content=json.dumps({"status": "queued"}),
            is_error=False,
        )


def _make_node(*, playing: bool) -> DialogueNode:
    n = object.__new__(DialogueNode)
    logger = MagicMock()
    n._logger = logger
    n.get_logger = lambda: logger
    n._response_pub = MagicMock()
    n._state_pub = MagicMock()
    n._sound_trigger_pub = MagicMock()
    n._tts_control_pub = MagicMock()
    n._music_cleanup_pub = MagicMock()
    n._dj_mode_pub = None
    n._dsm = MagicMock()
    n._dsm.current_state = MagicMock()
    n._dj = MagicMock()
    n._dj.state.enabled = False
    n._active_tg_chat_id = None
    n._pending_music_cleanup = False
    n._active_batches = {}
    n._effects = MagicMock()
    n._verbose_llm = False
    n._babble_retry_used = False
    n._action_claim_retry_used = False
    n._code_speech_retry_used = False
    n._track_mode_music_active = False
    n._retry_dispatched_in_turn = False
    n._run_task = None
    n._task_lock = MagicMock()
    n._startup_greeting_fired = False
    n._consume_synthetic_retry = MagicMock(return_value=True)
    n._dispatch_turn = MagicMock()
    n._check_babble_and_retry = MagicMock(return_value=False)
    n._check_embedded_renardo_code_and_retry = MagicMock(return_value=False)
    n._music_guard = MusicGuard()
    n._generated_music_state = None
    n._music_player_state = MusicPlayerState(state="playing" if playing else "idle")
    # роутер медиакоманд (#3134)
    n._llm_skipped_counter = new_llm_skip_counter()
    n._cancel_run = MagicMock()
    n._force_dj_off_for_stop_command = MagicMock()
    n._speak_direct = MagicMock()
    n._music_form_track = None
    n._scheduler_executor = _FakeExecutor()
    n._loop = MagicMock()
    n._core = MagicMock()
    return n


def _result(spoken: str, tools=()) -> DialogResult:
    return DialogResult(
        spoken_text=spoken, tools_called=list(tools), finish_reason="stop",
    )


def _published(node: DialogueNode) -> list:
    return [c.args[0].data for c in node._response_pub.publish.call_args_list]


def _said(node: DialogueNode) -> str:
    return " | ".join(_published(node))


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


# ── Факт 1: «что сейчас играет» при idle ───────────────────────────────


def test_fact1_idle_answer_goes_to_tts_without_retry() -> None:
    n = _make_node(playing=False)
    n._handle_result(_result(LIVE_IDLE_ANSWER), user_input=LIVE_QUESTION)

    n._dispatch_turn.assert_not_called()
    assert LIVE_IDLE_ANSWER in _said(n)
    assert FAILURE_PHRASE not in _said(n)


def test_idle_player_but_model_says_playing_still_retries() -> None:
    """Ложь против снимка по-прежнему ловит Bug E ``music_state``."""
    n = _make_node(playing=False)
    n._handle_result(
        _result("Сейчас играет клубный трек."), user_input=LIVE_QUESTION
    )

    n._dispatch_turn.assert_called_once()
    assert "клубный трек" not in _said(n)


def test_no_snapshot_keeps_old_retry() -> None:
    """Снимка нет (``playing="unknown"``) — прежнее поведение, ретрай."""
    n = _make_node(playing=False)
    n._music_player_state = None
    n._handle_result(_result(LIVE_IDLE_ANSWER), user_input=LIVE_QUESTION)

    n._dispatch_turn.assert_called_once()


# ── Факт 2: стоп роутером, потом «что ты включал» ──────────────────────


def test_fact2_router_stop_backs_the_true_claim(run_plans) -> None:
    n = _make_node(playing=True)
    assert n._route_media_command("выключи музыку") is True
    run_plans()
    assert n._scheduler_executor.calls == ["stop_music"]
    # Ход роутера записан для модели: реплика, фраза, вызванный тул.
    n._core.record_external_turn.assert_called_once_with(
        "выключи музыку", "Выключил музыку.", ["stop_music"]
    )

    n._music_player_state = MusicPlayerState(state="idle")
    n._handle_result(
        _result(LIVE_AFTER_STOP_ANSWER), user_input=LIVE_WHAT_DID_YOU_PLAY
    )

    n._dispatch_turn.assert_not_called()
    assert LIVE_AFTER_STOP_ANSWER in _said(n)


def test_same_claim_without_router_action_is_still_phantom() -> None:
    n = _make_node(playing=False)
    n._handle_result(
        _result(LIVE_AFTER_STOP_ANSWER), user_input=LIVE_WHAT_DID_YOU_PLAY
    )

    n._dispatch_turn.assert_called_once()
    assert LIVE_AFTER_STOP_ANSWER not in _said(n)


def test_router_action_expires(run_plans, monkeypatch) -> None:
    """Через ``MEDIA_ACTION_BACKING_S`` стоп роутера заявление не подкрепляет."""
    n = _make_node(playing=True)
    n._route_media_command("выключи музыку")
    run_plans()
    later = dialogue_node_module.time.time() + DialogueNode.MEDIA_ACTION_BACKING_S + 1
    monkeypatch.setattr(dialogue_node_module.time, "time", lambda: later)

    n._music_player_state = MusicPlayerState(state="idle")
    n._handle_result(
        _result(LIVE_AFTER_STOP_ANSWER), user_input=LIVE_WHAT_DID_YOU_PLAY
    )

    n._dispatch_turn.assert_called_once()


# ── Пункт 4: фраза-отказ без попытки не звучит ─────────────────────────


def test_repeated_claim_without_any_tool_is_not_called_a_failure() -> None:
    """Ретрай #2549 потрачен, заявление повторилось, тулов в ходе не было."""
    n = _make_node(playing=False)
    n._universal_action_claim_retry_used = True
    n._handle_result(
        _result("Сейчас тишина, последний трек я остановил."),
        user_input=LIVE_QUESTION,
    )

    said = _said(n)
    assert FAILURE_PHRASE not in said
    assert ACTION_CLAIM_NOTHING_DONE_TEXT in said
    assert "я остановил" not in said


def test_failed_tool_keeps_honest_failure_phrase() -> None:
    """Тул упал — «Не получилось выполнить» по-прежнему правда (#2949)."""
    n = _make_node(playing=False)
    n._universal_action_claim_retry_used = True
    published = n._publish_universal_action_claim_fallback_if_needed(
        spoken="Записала пресет.",
        tools_called=("save_arrangement_preset",),
        user_input="сохрани пресет",
        raw_user_command="сохрани пресет",
        is_dj_auto=False,
        has_error=False,
        speak_text_real=0,
        tool_error_occurred=True,
    )

    assert published is True
    assert FAILURE_PHRASE in _said(n)
