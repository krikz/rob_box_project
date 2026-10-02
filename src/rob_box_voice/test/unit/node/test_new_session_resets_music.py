"""Issue #2835 — «новая сессия» гасит музыку и хвосты старых ходов.

Живой прогон 23.09.2026: после «стоп диджей» + «забудь всё» запоздалый
ретрай-ход пережил сброс; ``[CRITICAL]``-ретрай стартовал в ту же секунду,
что и ``session reset``.

Поведения (acceptance issue):
1. new-session: music_cleanup опубликован, одна фраза подтверждения.
2. Ретраи от хода, отменённого barge-in'ом или сбросом, не запускаются.

ADR-0149 PR-13a: DJ-контроллер старого пути, забор на запоздалый
``set_dj_mode(enabled=true)`` и ``MusicGuard`` удалены — их проверки тоже.

DialogueNode собирается через ``object.__new__`` (как в
test_barge_in_policy.py / test_new_session_reset.py), rclpy замокан в conftest.
"""

from __future__ import annotations

import asyncio
import json
import threading
from collections import deque
from unittest.mock import AsyncMock, MagicMock, patch

from rob_box_harness.core.dialogue_state_machine import DialogueStateKind
from rob_box_voice.core.session_epoch import TURN_EPOCH
from rob_box_voice.dialogue_node import DialogueNode


def _make_reset_node() -> DialogueNode:
    """Узел с атрибутами, которые трогает ``_reset_dialogue_session``."""
    n = object.__new__(DialogueNode)
    n.get_logger = lambda: MagicMock()
    n._cancel_run = MagicMock()
    n._tts_control_pub = MagicMock()
    n._pending_backlog_flush = False
    n._dsm = MagicMock()
    n._speaker_lock = threading.Lock()
    n._current_speaker = {"is_known": False}
    n._speaker_by_text = {}
    n._speaker_tracker = MagicMock()
    n._maybe_record_session_end = MagicMock()
    n._publish_state = MagicMock()
    n._publish_response = MagicMock()
    n._pending_user_messages = deque()
    n._music_cleanup_pub = MagicMock()
    n._pending_music_cleanup = True
    n._track_mode_music_active = True
    return n


def _published(pub: MagicMock) -> list[dict]:
    return [json.loads(c.args[0].data) for c in pub.publish.call_args_list]


class _Result:
    spoken_text = "Окей"
    tools_called = ()
    error = None


def _make_turn_node() -> DialogueNode:
    """Узел для ``_run_turn`` (форма как в test_barge_in_policy.py)."""
    n = object.__new__(DialogueNode)
    n._task_lock = threading.Lock()
    n._run_cancelled = False
    n._babble_retry_used = False
    n._pending_music_cleanup = False
    n._speaker_id_enabled = False
    n._handle_speaker_turn = MagicMock()
    n._apply_speaker_identity = MagicMock()
    n._build_dynamic_system_context = MagicMock(
        return_value="<system_context/>"
    )
    n._llm = MagicMock()
    n._speak_direct = MagicMock()
    n._active_batches = set()
    n._dsm = MagicMock()
    n._dsm.current_state = DialogueStateKind.DIALOGUE
    n._publish_state = MagicMock()
    n._apply_tool_skipped_guard = MagicMock(return_value=False)
    n._publish_music_cleanup = MagicMock()
    n._maybe_record_session_end = MagicMock()
    n.get_logger = lambda: MagicMock()
    n._core = MagicMock()
    n._core.process_input = AsyncMock(return_value=_Result())
    n._handle_result = MagicMock()
    n._dispatch_turn = MagicMock()
    n._pending_user_messages = deque()
    n._track_mode_music_active = False
    n._action_claim_retry_used = False
    return n


# ---------------------------------------------------------------------------
# 1. new-session: музыка остановлена, одна фраза
# ---------------------------------------------------------------------------


class TestNewSessionResetsMusic:
    def test_only_the_reset_confirmation_is_spoken(self):
        n = _make_reset_node()

        n._reset_dialogue_session()

        n._publish_response.assert_called_once()
        assert "новую сессию" in n._publish_response.call_args.args[0]


    def test_music_cleanup_published(self):
        n = _make_reset_node()

        n._reset_dialogue_session()

        assert {"reason": "new_session"} in _published(n._music_cleanup_pub)
        assert n._pending_music_cleanup is False
        assert n._track_mode_music_active is False


    def test_reset_advances_session_epoch(self):
        n = _make_reset_node()
        before = n._session_epoch_gate().current

        n._reset_dialogue_session()

        assert n._session_epoch_gate().current == before + 1


# ---------------------------------------------------------------------------
# 2. Ретраи отменённого / пережившего сброс хода не запускаются
# ---------------------------------------------------------------------------


class TestCancelledTurnDoesNotRetry:
    def test_normal_turn_still_runs_post_turn_guards(self):
        """Контроль: без отмены/сброса гуард работает как раньше."""
        n = _make_turn_node()

        asyncio.run(n._run_turn("который час", session_epoch=0))

        n._apply_tool_skipped_guard.assert_called_once()

    def test_barge_in_cancelled_turn_dispatches_no_retry(self):
        """Отменённый ход приходил в finally с ``result=None`` →
        ``tools_called=()`` → Bug C слал [CRITICAL]-ретрай."""
        n = _make_turn_node()
        n._core.process_input = AsyncMock(side_effect=asyncio.CancelledError())

        asyncio.run(n._run_turn("который час", session_epoch=0))

        n._apply_tool_skipped_guard.assert_not_called()

    def test_turn_outliving_session_reset_dispatches_no_retry(self):
        n = _make_turn_node()

        async def _llm_while_reset(*_a, **_kw):
            n._session_epoch_gate().advance()  # «забудь всё» посреди хода
            return _Result()

        n._core.process_input = AsyncMock(side_effect=_llm_while_reset)

        asyncio.run(n._run_turn("который час", session_epoch=0))

        n._apply_tool_skipped_guard.assert_not_called()

    def test_real_tool_guard_retry_not_dispatched_for_cancelled_turn(self):
        """Без мока гуарда: реальный tool-skipped гуард на «который час» с
        пустыми tools отдал бы ретрай. После отмены — ни одного _dispatch_turn."""
        n = _make_turn_node()
        del n._apply_tool_skipped_guard
        n._retry_dispatched_in_turn = False
        n._tool_retry_used = False
        n._synthetic_retries_left = 3
        n._reopen_dialogue_for_retry = MagicMock()
        n._core.process_input = AsyncMock(side_effect=asyncio.CancelledError())

        asyncio.run(n._run_turn("который час", session_epoch=0))

        n._dispatch_turn.assert_not_called()

    def test_retry_queued_before_reset_is_dropped_at_start(self):
        """Ретрай, поставленный в loop изнутри хода старой сессии, несёт
        её поколение и после сброса не доходит до LLM."""
        n = _make_turn_node()
        del n._dispatch_turn  # настоящий _dispatch_turn
        n._loop = MagicMock()
        n._session_started_at = None
        captured = []

        async def _parent_turn():
            TURN_EPOCH.set(n._session_epoch_gate().current)
            with patch(
                "rob_box_voice.dialogue_node.asyncio.run_coroutine_threadsafe",
                side_effect=lambda coro, _loop: captured.append(coro),
            ):
                n._dispatch_turn("[CRITICAL] ...", is_synthetic=True)
            n._session_epoch_gate().advance()  # сброс до старта ретрая
            # Issue #3145 — синтетический ретрай сначала планирует отзыв
            # отвергнутого ответа, ход ретрая — последний в очереди.
            await captured[-1]

        asyncio.run(_parent_turn())

        n._core.process_input.assert_not_called()


# ---------------------------------------------------------------------------
# 3. Ход, начатый до сброса, не включает DJ после сброса
# ---------------------------------------------------------------------------


