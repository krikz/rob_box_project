"""Node-тесты barge_in_policy (S1.3, scheduler-segments-merge plan).

Проверяет диспетч STOP/no-STOP в ``_on_stt`` в зависимости от параметра
``barge_in_policy``. Образец фикстуры — ``test_speech_backlog_accumulator.py``
(``object.__new__(DialogueNode)`` + ручные атрибуты, без реального ROS2).
"""

from unittest.mock import MagicMock

import pytest

from rob_box_harness.core.dialogue_state_machine import DialogueStateKind
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
