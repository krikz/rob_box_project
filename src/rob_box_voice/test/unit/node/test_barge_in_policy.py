"""Node-тесты barge_in_policy (S1.3, scheduler-segments-merge plan).

Проверяет диспетч STOP/no-STOP в ``_on_stt`` в зависимости от параметра
``barge_in_policy``. Образец фикстуры — ``test_speech_backlog_accumulator.py``
(``object.__new__(DialogueNode)`` + ручные атрибуты, без реального ROS2).
"""

import asyncio
import threading
import time
from collections import deque
from unittest.mock import AsyncMock, MagicMock

import pytest

from rob_box_harness.core.dialogue_state_machine import DialogueStateKind
from rob_box_voice.core.music_guard import MusicGuard
from rob_box_voice.dialogue_node import DialogueNode


@pytest.fixture
def node():
    n = object.__new__(DialogueNode)
    logger = MagicMock()
    n.get_logger = lambda: logger

    n._wake_words = ["робок", "робот", "роббокс", "робокс", "robbox", "rob box"]
    n._dsm = MagicMock()
    n._dsm.current_state = DialogueStateKind.DIALOGUE
    n._dj = MagicMock()
    n._dj.state.enabled = False
    n._sound_trigger_pub = MagicMock()
    n._publish_state = MagicMock()
    n._dispatch_turn = MagicMock()
    n._verbose_llm = False
    n._speaker_by_text = {}
    n._llm_skipped_counter = {
        "no_wake_word": 0,
        "silenced": 0,
        "silence_command": 0,
        "empty_after_strip": 0,
        "stt_rejected": 0,
        "music_stop": 0,
        "command_intent": 0,
        "quick_decide_ignore": 0,
    }
    n._maybe_log_skip_summary = MagicMock()
    n._active_tg_chat_id = None

    # Real _cancel_run — this is what the test verifies.
    n._run_task = None
    n._task_lock = MagicMock()
    n._task_lock.__enter__ = MagicMock(return_value=None)
    n._task_lock.__exit__ = MagicMock(return_value=False)
    n._loop = MagicMock()
    n._loop.call_soon_threadsafe = lambda fn, *a, **kw: fn(*a, **kw)
    n._run_cancelled = False
    n._effects = MagicMock()
    n._tts_control_pub = MagicMock()

    n._barge_in_policy = "replace"
    n._pending_user_messages = deque()
    # Attributes normally set in DialogueNode.__init__ — ``object.__new__`` skips
    # ``__init__``, so anything the SUT touches must be pre-seeded here. The
    # list grows whenever upstream code starts reading a new field; the
    # upstream-regression t_5e06c47d added these three (see
    # dialogue_node.py:640 ``_track_mode_music_active``, :678
    # ``_action_claim_retry_used``, :682 ``_code_speech_retry_used``).
    # Without seeding all three, ``_handle_result`` cascades AttributeError
    # across ``_check_embedded_renardo_code_and_retry`` /
    # ``_check_unbacked_action_claim_and_retry`` /
    # ``_check_embedded_renardo_code_and_retry`` in chain order.
    n._track_mode_music_active = False
    n._action_claim_retry_used = False
    n._code_speech_retry_used = False
    return n


def _stt(data):
    msg = MagicMock()
    msg.data = data
    return msg


class TestBargeInPolicyDispatch:
    def test_replace_publishes_stop(self, node):
        """policy=replace — регресс: STOP уходит на новый STT-ввод."""
        node._barge_in_policy = "replace"
        node._on_stt(_stt("робот спой про комара"))
        node._tts_control_pub.publish.assert_called_once()
        published = node._tts_control_pub.publish.call_args.args[0]
        assert published.data == "STOP"

    def test_classify_does_not_publish_stop(self, node):
        """policy=classify — сегмент доигрывает, STOP не уходит."""
        node._barge_in_policy = "classify"
        node._on_stt(_stt("робот и ещё про енота"))
        node._tts_control_pub.publish.assert_not_called()

    def test_classify_still_cancels_turn_and_releases_awaiters(self, node):
        """R1: даже без STOP турн отменяется и awaiter'ы отпускаются —
        иначе speak_helpers._tts_events залипнут навсегда."""
        node._barge_in_policy = "classify"
        node._on_stt(_stt("робот и ещё про енота"))
        assert node._run_cancelled is True
        node._effects.release_all_tts.assert_called_once()
        node._effects.clear_sound_event.assert_called_once()

    def test_classify_still_dispatches_turn(self, node):
        node._barge_in_policy = "classify"
        node._on_stt(_stt("робот и ещё про енота"))
        node._dispatch_turn.assert_called_once()


# ---------------------------------------------------------------------------
# S4.2 — quick_decide verdict dispatch under barge_in_policy=classify
# ---------------------------------------------------------------------------


class TestQuickDecideDispatch:
    def test_ignore_verdict_skips_turn_entirely(self, node):
        """IGNORE — турн не запускается вовсе: no cancel, no dispatch."""
        node._barge_in_policy = "classify"
        node._on_stt(_stt("робот угу"))
        node._dispatch_turn.assert_not_called()
        node._effects.release_all_tts.assert_not_called()
        node._tts_control_pub.publish.assert_not_called()
        assert node._run_cancelled is False
        assert node._llm_skipped_counter["quick_decide_ignore"] == 1

    def test_replace_verdict_stops_tts_and_dispatches(self, node):
        """REPLACE (explicit imperative) — same as today's unconditional
        behaviour: STOP published, turn still dispatches.

        Uses "стоп" rather than "хватит"/"замолчи": those two are ALSO
        matched by the separate, earlier ``is_silence_command`` gate
        (dialogue_text.DEFAULT_SILENCE_COMMANDS: "помолч"/"замолч"/
        "хватит"), which intercepts before quick_decide ever runs —
        that pre-existing path is out of scope here.
        """
        node._barge_in_policy = "classify"
        node._on_stt(_stt("робот стоп"))
        node._tts_control_pub.publish.assert_called_once()
        published = node._tts_control_pub.publish.call_args.args[0]
        assert published.data == "STOP"
        node._dispatch_turn.assert_called_once()

    def test_pending_llm_verdict_does_not_stop_tts_and_dispatches(self, node):
        """PENDING_LLM — no STOP, turn dispatches (already covered by
        TestBargeInPolicyDispatch, repeated here under the new dispatch
        path's own test class for S4.2 traceability)."""
        node._barge_in_policy = "classify"
        node._on_stt(_stt("робот и ещё про енота"))
        node._tts_control_pub.publish.assert_not_called()
        node._dispatch_turn.assert_called_once()

    def test_replace_policy_never_calls_quick_decide(self, node, monkeypatch):
        """policy=replace — quick_decide must not even be called (S1
        regression path stays untouched)."""
        import rob_box_voice.dialogue_node as dn

        called = MagicMock()
        monkeypatch.setattr(dn, "quick_decide", called)
        node._barge_in_policy = "replace"
        node._on_stt(_stt("робот угу"))
        called.assert_not_called()


# ---------------------------------------------------------------------------
# S7 — pending_user_messages queue (scheduler-segments-merge plan)
#
# Closes the hole S1 opens: with barge_in_policy=classify a PENDING_LLM
# verdict no longer stops TTS, but the LLM turn itself is still
# cancelled unconditionally today — these tests pin the NEW behaviour:
# while a turn is genuinely still in flight (_run_task alive), a
# PENDING_LLM phrase must not start a second concurrent turn at all —
# it queues, and _run_turn's ``finally`` drains it as ONE follow-up
# turn once the slot frees up (§4.7.3 "фраза ставится в очередь
# user_messages для следующего хода").
# ---------------------------------------------------------------------------


class TestPendingUserMessagesQueueOnStt:
    def test_pending_llm_with_live_task_queues_instead_of_dispatching(self, node):
        """A turn is still in flight (_run_task not done) when a
        PENDING_LLM phrase arrives — queue it, don't start a second
        turn, don't touch the running turn at all (no cancel, no STOP,
        no awaiter release)."""
        node._barge_in_policy = "classify"
        live_task = MagicMock()
        live_task.done.return_value = False
        node._run_task = live_task

        node._on_stt(_stt("робот и ещё про енота"))

        node._dispatch_turn.assert_not_called()
        node._tts_control_pub.publish.assert_not_called()
        node._effects.release_all_tts.assert_not_called()
        assert node._run_cancelled is False
        assert len(node._pending_user_messages) == 1
        queued_text, _ts = node._pending_user_messages[0]
        assert "енота" in queued_text

    def test_pending_llm_with_finished_task_dispatches_normally(self, node):
        """_run_task exists but is done() — free slot, dispatch as usual
        (S4.2 regression, repeated here under S7 for traceability)."""
        node._barge_in_policy = "classify"
        finished_task = MagicMock()
        finished_task.done.return_value = True
        node._run_task = finished_task

        node._on_stt(_stt("робот и ещё про енота"))

        node._dispatch_turn.assert_called_once()
        assert len(node._pending_user_messages) == 0

    def test_pending_llm_with_no_task_dispatches_normally(self, node):
        node._barge_in_policy = "classify"
        node._run_task = None

        node._on_stt(_stt("робот и ещё про енота"))

        node._dispatch_turn.assert_called_once()
        assert len(node._pending_user_messages) == 0

    def test_replace_verdict_dispatches_even_with_live_task(self, node):
        """An explicit imperative ("стоп") always cuts in — never queued,
        even while a turn is in flight."""
        node._barge_in_policy = "classify"
        live_task = MagicMock()
        live_task.done.return_value = False
        node._run_task = live_task

        node._on_stt(_stt("робот стоп"))

        node._dispatch_turn.assert_called_once()
        node._tts_control_pub.publish.assert_called_once()
        assert len(node._pending_user_messages) == 0

    def test_replace_policy_never_queues_even_with_live_task(self, node):
        """policy=replace regression: never touches the queue at all —
        always cancels + dispatches (S1 behaviour untouched)."""
        node._barge_in_policy = "replace"
        live_task = MagicMock()
        live_task.done.return_value = False
        node._run_task = live_task

        node._on_stt(_stt("робот и ещё про енота"))

        node._dispatch_turn.assert_called_once()
        node._tts_control_pub.publish.assert_called_once()
        assert len(node._pending_user_messages) == 0

    def test_ignore_verdict_with_live_task_still_does_not_queue(self, node):
        """Rule-level noise (IGNORE) is dropped outright — it never
        enters the pending queue even while a turn is in flight."""
        node._barge_in_policy = "classify"
        live_task = MagicMock()
        live_task.done.return_value = False
        node._run_task = live_task

        node._on_stt(_stt("робот угу"))

        node._dispatch_turn.assert_not_called()
        assert len(node._pending_user_messages) == 0

    def test_queue_overflow_drops_oldest_and_logs(self, node):
        node._barge_in_policy = "classify"
        live_task = MagicMock()
        live_task.done.return_value = False
        node._run_task = live_task

        from rob_box_voice.dialogue_node import _PENDING_USER_MESSAGES_MAX

        for i in range(_PENDING_USER_MESSAGES_MAX):
            node._on_stt(_stt(f"робот и ещё про {i}"))
        assert len(node._pending_user_messages) == _PENDING_USER_MESSAGES_MAX
        oldest_before = node._pending_user_messages[0][0]

        node._on_stt(_stt("робот и ещё про переполнение"))

        assert len(node._pending_user_messages) == _PENDING_USER_MESSAGES_MAX
        assert node._pending_user_messages[0][0] != oldest_before
        assert "переполнение" in node._pending_user_messages[-1][0]
        node.get_logger().warning.assert_called()


def _make_turn_node() -> DialogueNode:
    """Minimal DialogueNode wired for ``_run_turn`` — same shape as
    ``test_issue_1195_tg_source.py``'s ``test_run_turn_from_tg_*``
    fixture, plus a ``_pending_user_messages`` queue to drain."""
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
    n._dsm.current_state = DialogueStateKind.DIALOGUE
    n._publish_state = MagicMock()
    n._apply_music_guard = MagicMock(return_value=False)
    n._publish_music_cleanup = MagicMock()
    n._maybe_record_session_end = MagicMock()
    n.get_logger = lambda: MagicMock()

    class _Result:
        spoken_text = "verse1"
        tools_called = ()
        error = None

    n._core = MagicMock()
    n._core.process_input = AsyncMock(return_value=_Result())
    n._handle_result = MagicMock()
    n._dispatch_turn = MagicMock()
    n._pending_user_messages = deque()
    # see t_5e06c47d — without these the S7 / drain tests hit AttributeError
    # on ``_apply_music_guard`` → ``_build_music_retry_prompt`` (line 3763)
    # plus the in-line _check_* chain inside a mocked-but-reachable
    # ``_handle_result`` if the test ever crosses it.
    n._track_mode_music_active = False
    n._action_claim_retry_used = False
    n._code_speech_retry_used = False
    return n


class TestPendingUserMessagesDrain:
    """S7 — draining the queue as ONE follow-up turn (_run_turn's finally)."""

    def test_drains_single_queued_message_as_one_turn(self):
        n = _make_turn_node()
        n._pending_user_messages.append(("и ещё про енота", time.monotonic()))

        asyncio.run(n._run_turn("спой про комара"))

        n._dispatch_turn.assert_called_once()
        dispatched_text = n._dispatch_turn.call_args.args[0]
        assert "енота" in dispatched_text
        assert len(n._pending_user_messages) == 0

    def test_drains_multiple_queued_messages_as_a_single_glued_turn(self):
        """Several accumulated phrases must produce ONE follow-up turn,
        not N."""
        n = _make_turn_node()
        n._pending_user_messages.append(("и ещё про енота", time.monotonic()))
        n._pending_user_messages.append(("и покороче", time.monotonic()))

        asyncio.run(n._run_turn("спой про комара"))

        n._dispatch_turn.assert_called_once()
        dispatched_text = n._dispatch_turn.call_args.args[0]
        assert "енота" in dispatched_text
        assert "покороче" in dispatched_text

    def test_empty_queue_does_not_dispatch_a_follow_up_turn(self):
        n = _make_turn_node()

        asyncio.run(n._run_turn("спой про комара"))

        n._dispatch_turn.assert_not_called()

    def test_drain_suppresses_dialogue_end_for_this_turn(self):
        """A drained follow-up turn is a continuation, not the end of
        the session — DIALOGUE_END must not fire on the turn that
        dispatches it (mirrors babble_retry_pending /
        music_retry_dispatched)."""
        n = _make_turn_node()
        n._pending_user_messages.append(("и ещё про енота", time.monotonic()))

        asyncio.run(n._run_turn("спой про комара"))

        n._dsm.on_event.assert_not_called()


class TestPendingUserMessagesClearedOnReset:
    def test_reset_dialogue_session_clears_pending_queue(self):
        n = object.__new__(DialogueNode)
        n.get_logger = lambda: MagicMock()
        n._cancel_run = MagicMock()
        n._tts_control_pub = MagicMock()
        n._pending_backlog_flush = False
        n._dsm = MagicMock()
        n._speaker_lock = threading.Lock()
        n._current_speaker = {"is_known": False}
        n._speaker_by_text = {"x": {}}
        n._speaker_tracker = MagicMock()
        n._maybe_record_session_end = MagicMock()
        n._publish_state = MagicMock()
        n._publish_response = MagicMock()
        n._pending_user_messages = deque([("и ещё про енота", time.monotonic())])

        n._reset_dialogue_session()

        assert len(n._pending_user_messages) == 0


# ---------------------------------------------------------------------------
# Issue #1734 — dialogue_node публикует barge_in_policy на latched-топик,
# чтобы stt_node узнавал актуальное значение без второго параметра в своём
# YAML (см. src/rob_box_voice/rob_box_voice/stt_node.py::publish_result /
# barge_in_policy_callback — они читают именно этот топик).
# ---------------------------------------------------------------------------


class TestPublishBargeInPolicy:
    def test_publishes_current_policy_value(self):
        n = object.__new__(DialogueNode)
        n._barge_in_policy = "classify"
        n._barge_in_policy_pub = MagicMock()

        n._publish_barge_in_policy()

        n._barge_in_policy_pub.publish.assert_called_once()
        published = n._barge_in_policy_pub.publish.call_args.args[0]
        assert published.data == "classify"

    def test_no_publisher_yet_is_a_no_op(self):
        """object.__new__(DialogueNode) в тестах не всегда создаёт
        паблишер (как и _speech_accumulator/_pending_user_messages
        в других местах этого файла) — не должно падать."""
        n = object.__new__(DialogueNode)
        n._barge_in_policy = "replace"

        n._publish_barge_in_policy()  # не должно бросить AttributeError


class TestParametersCallbackBargeInPolicy:
    """``ros2 param set /dialogue_node barge_in_policy classify`` — ровно
    тот workflow, которым баг #1734 воспроизводили на роботе (см. raw
    evidence в issue). Теперь runtime-изменение должно реально доходить
    до stt_node через ``_publish_barge_in_policy``."""

    @pytest.fixture
    def n(self):
        node = object.__new__(DialogueNode)
        node.get_logger = lambda: MagicMock()
        node._barge_in_policy = "replace"
        node._barge_in_policy_pub = MagicMock()
        node._voice_input_mode = "on"
        return node

    @staticmethod
    def _param(name, value):
        p = MagicMock()
        p.name = name
        p.value = value
        return p

    def test_valid_change_updates_and_republishes(self, n):
        result = n.parameters_callback([self._param("barge_in_policy", "classify")])
        assert n._barge_in_policy == "classify"
        n._barge_in_policy_pub.publish.assert_called_once()
        published = n._barge_in_policy_pub.publish.call_args.args[0]
        assert published.data == "classify"
        assert result.successful is True

    def test_invalid_value_ignored_no_republish(self, n):
        n.parameters_callback([self._param("barge_in_policy", "yolo")])
        assert n._barge_in_policy == "replace"
        n._barge_in_policy_pub.publish.assert_not_called()

    def test_unrelated_param_does_not_touch_barge_in_policy(self, n):
        n.parameters_callback([self._param("voice_input_mode", "off")])
        assert n._barge_in_policy == "replace"
        n._barge_in_policy_pub.publish.assert_not_called()
        assert n._voice_input_mode == "off"
