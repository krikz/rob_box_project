"""Node-тесты бэклог-аккумулятора фоновой речи без wake-слова."""

import threading
from unittest.mock import MagicMock

import pytest

from rob_box_harness.core.dialogue_state_machine import DialogueStateKind
from rob_box_voice.core.speech_accumulator import SpeechAccumulator
from rob_box_voice.dialogue_node import DialogueNode


@pytest.fixture
def node():
    n = object.__new__(DialogueNode)
    logger = MagicMock()
    n.get_logger = lambda: logger

    n._wake_words = ["робок", "робот", "роббокс", "робокс", "robbox", "rob box"]
    n._dsm = MagicMock()
    n._dsm.current_state = DialogueStateKind.IDLE
    n._dj = MagicMock()
    n._dj.state.enabled = False
    n._cancel_run = MagicMock()
    n._sound_trigger_pub = MagicMock()
    n._publish_state = MagicMock()
    n._dispatch_turn = MagicMock()
    n._verbose_llm = False
    n._speaker_by_text = {}
    n._speaker_lock = threading.Lock()
    n._current_speaker = {"is_known": False}
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

    n._speech_accumulator = SpeechAccumulator(window_sec=180.0)
    n._accumulate_no_wake_enabled = True
    n._pending_backlog_flush = False

    # Для _build_dynamic_system_context
    n.get_parameter = MagicMock(return_value=MagicMock(value="minimax"))
    n._generated_music_state = {}
    return n


def _stt(node, data):
    msg = MagicMock()
    msg.data = data
    return msg


class TestNoWakeAccumulates:
    def test_no_wake_text_accumulates_not_dropped(self, node):
        node._on_stt(_stt(node, "расскажи про погоду"))
        assert not node._speech_accumulator.is_empty()
        node._dispatch_turn.assert_not_called()
        assert node._llm_skipped_counter["no_wake_word"] == 0

    def test_feature_off_drops(self, node):
        node._accumulate_no_wake_enabled = False
        node._on_stt(_stt(node, "расскажи про погоду"))
        assert node._speech_accumulator.is_empty()
        assert node._llm_skipped_counter["no_wake_word"] == 1


class TestWakeFlushes:
    def test_wake_with_backlog_sets_flush_flag_and_dispatches(self, node):
        node._on_stt(_stt(node, "расскажи про погоду"))
        node._on_stt(_stt(node, "робот включи свет"))
        assert node._pending_backlog_flush is True
        node._dispatch_turn.assert_called_once()

    def test_bare_wake_word_flushes_backlog(self, node):
        node._on_stt(_stt(node, "расскажи про погоду"))
        node._on_stt(_stt(node, "робот"))
        assert node._pending_backlog_flush is True
        dispatched = node._dispatch_turn.call_args.args[0]
        assert dispatched.startswith("робот")
        assert "ФОНОВЫЙ ЗАПРОС" in dispatched
        assert "расскажи про погоду" in dispatched
        node._dispatch_turn.assert_called_once()

    def test_wake_with_real_command_injects_hint(self, node):
        node._on_stt(_stt(node, "расскажи про погоду"))
        node._on_stt(_stt(node, "робот включи свет"))
        dispatched = node._dispatch_turn.call_args.args[0]
        assert dispatched.startswith("включи свет")
        assert "ФОНОВЫЙ ЗАПРОС" in dispatched
        assert "расскажи про погоду" in dispatched
        node._dispatch_turn.assert_called_once()

    def test_vague_follow_up_injects_hint(self, node):
        node._on_stt(_stt(node, "расскажи про погоду"))
        node._on_stt(_stt(node, "робот может ты"))
        dispatched = node._dispatch_turn.call_args.args[0]
        assert dispatched.startswith("может ты")
        assert "ФОНОВЫЙ ЗАПРОС" in dispatched
        assert "расскажи про погоду" in dispatched
        node._dispatch_turn.assert_called_once()

    def test_empty_backlog_no_flag(self, node):
        node._on_stt(_stt(node, "робот включи свет"))
        assert node._pending_backlog_flush is False
        node._dispatch_turn.assert_called_once()


class TestBuildDynamicContextFlushes:
    def test_flush_inserts_block_and_clears(self, node):
        node._on_stt(_stt(node, "расскажи про погоду"))
        node._on_stt(_stt(node, "робот"))
        ctx = node._build_dynamic_system_context()
        assert "<speech_backlog>" in ctx
        assert "расскажи про погоду" in ctx
        assert node._pending_backlog_flush is False
        assert node._speech_accumulator.is_empty()
