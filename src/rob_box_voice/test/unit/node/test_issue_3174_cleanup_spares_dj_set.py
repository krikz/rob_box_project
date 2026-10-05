"""Issue #3174 — DJ-сет роутера пережил ход без музыкальных тулов.

Живой прогон 29.09.2026 04:59–05:02 UTC (деплой develop ``65b7f5ffa``)::

    04:59:36 «Робот ты диджей Снупдог» → роутер: превью club + set_dj_mode
    05:00:12 «Робот поставь к Элизе»
             spoken='Сейчас поставлю к Элизе!' tools=[]
             music_cleanup sent: reason=tts_batch_complete
             [mcp_server] Авто-стоп … (issue #935) → сет замолчал на 38 с

Две причины, обе здесь:

1. ``_schedule_music_cleanup`` взводил cleanup на любом ходе без
   музыкальных тулов, хотя по снимку плеера (ADR-0141) играла музыка,
   запущенная НЕ этим ходом (роутер исполняет тулы в обход LLM-хода).
2. Обещание «поставлю» без тула пропустил гуард #2549: журнал роутера
   (#3165, ``_claim_backing_tools``) — превью ``compose_music`` +
   ``set_dj_mode`` 36 с назад — считался подкреплением любого заявления,
   в том числе обещания нового действия в будущем времени. Bug C молчал
   штатно: «поставь к Элизе» не музыкальный запрос для ``user_wants_music``
   (мораторий #3132 — регексы не расширяем).

Сторона ``mcp_server`` (мягкий cleanup не гасит DJ-сет) —
``rob_box_mcp_tools/test/test_mcp_server.py``.

ADR-0149 PR-13a: сет запускает ``dj_set`` движка v2 (без превью
``compose_music`` и ``set_dj_mode``), локального DJ-флага у диалога нет —
«сет идёт» знает только снимок плеера.
"""

from __future__ import annotations

import asyncio
import json
from unittest.mock import MagicMock

import pytest

from rob_box_harness.core.agent_core import DialogResult
from rob_box_llm.provider import ToolResult
from rob_box_voice.core.llm_skip_reasons import new_llm_skip_counter
from rob_box_voice.core.music_player_state import MusicPlayerState
from rob_box_voice.dialogue_node import DialogueNode
import rob_box_voice.dialogue_node as dialogue_node_module

LIVE_DJ_COMMAND = "ты диджей снупдог"
LIVE_ELISE = "[Speaker:unknown] поставь к элизе"
LIVE_PROMISE = "Сейчас поставлю к Элизе!"


class _FakeExecutor:
    def __init__(self) -> None:
        self.calls = []

    def begin_turn(self) -> None:
        pass

    async def execute(self, call):
        self.calls.append(call.name)
        return ToolResult(
            tool_call_id=call.id,
            content=json.dumps({"status": "ok"}),
            is_error=False,
        )


def _make_node(snapshot) -> DialogueNode:
    n = object.__new__(DialogueNode)
    logger = MagicMock()
    n._logger = logger
    n.get_logger = lambda: logger
    n._response_pub = MagicMock()
    n._state_pub = MagicMock()
    n._sound_trigger_pub = MagicMock()
    n._tts_control_pub = MagicMock()
    n._music_cleanup_pub = MagicMock()
    n._dsm = MagicMock()
    n._dsm.current_state = MagicMock()
    n._active_tg_chat_id = None
    n._pending_music_cleanup = False
    n._active_batches = {}
    n._effects = MagicMock()
    n._verbose_llm = False
    n._babble_retry_used = False
    n._action_claim_retry_used = False
    n._track_mode_music_active = False
    n._retry_dispatched_in_turn = False
    n._run_task = None
    n._task_lock = MagicMock()
    n._startup_greeting_fired = False
    n._consume_synthetic_retry = MagicMock(return_value=True)
    n._dispatch_turn = MagicMock()
    n._check_babble_and_retry = MagicMock(return_value=False)
    n._generated_music_state = None
    n._music_player_state = snapshot
    n._llm_skipped_counter = new_llm_skip_counter()
    n._cancel_run = MagicMock()
    n._speak_direct = MagicMock()
    n._scheduler_executor = _FakeExecutor()
    n._loop = MagicMock()
    n._core = MagicMock()
    return n


def _dj_set_snapshot() -> MusicPlayerState:
    """Снимок плеера после ``dj_set(start)`` роутера."""
    return MusicPlayerState(state="playing", track_id="t1", dj=True)


def _result(spoken: str, tools=(), speak_text_count: int = 0) -> DialogResult:
    r = DialogResult(
        spoken_text=spoken, tools_called=list(tools), finish_reason="stop",
    )
    r.speak_text_count = speak_text_count
    return r


def _cleanups(node: DialogueNode) -> list:
    return [
        json.loads(c.args[0].data).get("reason")
        for c in node._music_cleanup_pub.publish.call_args_list
    ]


def _finalize(node: DialogueNode, result, user_input: str) -> None:
    node._finalize_music_cleanup_policy(
        result=result,
        raw_user_command=user_input,
        user_input=user_input,
    )


def _said(node: DialogueNode) -> str:
    return " | ".join(
        c.args[0].data for c in node._response_pub.publish.call_args_list
    )


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


def _start_dj_set_via_router(node: DialogueNode, run_plans) -> None:
    node._music_player_state = MusicPlayerState(state="idle")
    assert node._route_media_command(LIVE_DJ_COMMAND) is True
    run_plans()
    assert node._scheduler_executor.calls == ["dj_set"]
    node._music_player_state = _dj_set_snapshot()


# ── Причина 1: cleanup после хода без музыкальных тулов ────────────────


def test_router_dj_set_then_turn_without_tools_does_not_arm_cleanup(run_plans):
    n = _make_node(None)
    _start_dj_set_via_router(n, run_plans)

    _finalize(n, _result(LIVE_PROMISE), LIVE_ELISE)

    assert n._pending_music_cleanup is False
    assert _cleanups(n) == []


def test_foreign_track_playing_without_dj_is_not_stopped_either():
    """Трек прошлого хода / роутера (не DJ) — тоже не наш, не трогаем."""
    n = _make_node(MusicPlayerState(state="playing", track_id="t1"))

    _finalize(n, _result("Привет!"), "[Speaker:unknown] как дела")

    assert n._pending_music_cleanup is False
    assert _cleanups(n) == []


@pytest.mark.parametrize(
    "snapshot", [None, MusicPlayerState(state="idle")], ids=["no_snapshot", "idle"]
)
def test_silent_player_keeps_old_cleanup_arming(snapshot):
    """Ничего не играет — прежнее поведение (#935, профилактический cleanup)."""
    n = _make_node(snapshot)

    _finalize(n, _result("Привет!"), "[Speaker:unknown] как дела")

    assert _cleanups(n) == ["tts_batch_complete"]


def test_backing_under_rap_is_still_stopped_after_speech():
    """BACKING этого хода (бит + 2 speak_text под рэп) гасится после речи."""
    n = _make_node(MusicPlayerState(state="playing", track_id="beat"))

    _finalize(
        n,
        _result("", tools=("gen_play_from_library", "speak_text"), speak_text_count=2),
        "[Speaker:unknown] зачитай рэп про котов под бит",
    )

    assert _cleanups(n) == ["tts_batch_complete"]


def test_stop_music_tool_still_defers_cleanup_during_dj_set():
    n = _make_node(_dj_set_snapshot())
    n._active_batches = {"b1": object()}

    _finalize(n, _result("Выключаю.", tools=("stop_music",)), "выключи музыку")

    assert n._pending_music_cleanup is True


# ── Причина 2: обещание «поставлю» без тула ────────────────────────────


def test_promise_without_tool_after_router_dj_set_is_retried(run_plans):
    n = _make_node(None)
    _start_dj_set_via_router(n, run_plans)

    n._handle_result(_result(LIVE_PROMISE), user_input=LIVE_ELISE)

    n._dispatch_turn.assert_called_once()
    assert LIVE_PROMISE not in _said(n)


def test_promise_without_tool_is_retried_without_router_too():
    n = _make_node(_dj_set_snapshot())

    n._handle_result(_result(LIVE_PROMISE), user_input=LIVE_ELISE)

    n._dispatch_turn.assert_called_once()
    assert LIVE_PROMISE not in _said(n)


def test_past_tense_retelling_of_router_action_is_still_backed(run_plans):
    """#3165 не ломаем: «я включил сет» после роутера — пересказ, не фантом."""
    n = _make_node(None)
    _start_dj_set_via_router(n, run_plans)
    retelling = "Я включил диджей-сет в стиле Снупдога."

    n._handle_result(
        _result(retelling), user_input="[Speaker:unknown] что ты сделал"
    )

    n._dispatch_turn.assert_not_called()
    assert retelling in _said(n)
