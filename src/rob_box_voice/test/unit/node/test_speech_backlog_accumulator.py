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

    def test_flush_logs_backlog_handled_marker(self, node):
        """Issue #1766 — в логе при сливе backlog должен быть маркер
        ``backlog_handled=true``, чтобы оператор / e2e могли грепом
        проверить «был ли в этом turn бэклог» и сравнить с acceptance
        (LLM должен выполнить явную команду из бэклога)."""
        node._on_stt(_stt(node, "включи трек про весну"))
        node._on_stt(_stt(node, "робот"))
        # Clear logger mocks from STT path so we can assert only the flush log.
        logger = node.get_logger()
        logger.reset_mock()
        node._build_dynamic_system_context()
        # Найти вызов info() с backlog_handled=true.
        calls = [
            c
            for c in logger.info.call_args_list
            if c.args and "backlog_handled=true" in str(c.args[0])
        ]
        assert calls, (
            "Expected at least one logger.info call with 'backlog_handled=true' "
            f"after backlog flush. All info calls: {logger.info.call_args_list!r}"
        )
        # В логе должно быть entries=N — операторский диагностический счётчик.
        msg = calls[0].args[0]
        assert "entries=1" in msg

    def test_flush_does_not_log_when_no_backlog(self, node):
        """Без backlog-флаша backlog_handled=true НЕ должно появляться в логе."""
        logger = node.get_logger()
        logger.reset_mock()
        node._build_dynamic_system_context()
        calls = [
            c
            for c in logger.info.call_args_list
            if c.args and "backlog_handled=true" in str(c.args[0])
        ]
        assert not calls, (
            f"Unexpected backlog_handled=true without backlog flush: {calls!r}"
        )

    def test_user_input_logs_backlog_pending_marker(self, node):
        """Issue #1766 — при backlog_pending=true в user-turn должна появиться
        отметка в логе, парная к backlog_handled=true (для сматчивания в e2e)."""
        logger = node.get_logger()
        node._on_stt(_stt(node, "включи трек про весну"))
        logger.reset_mock()
        node._on_stt(_stt(node, "робот"))
        # Ищем именно новый маркер backlog_pending=true.
        calls = [
            c
            for c in logger.info.call_args_list
            if c.args and "backlog_pending=true" in str(c.args[0])
        ]
        assert calls, (
            f"Expected logger.info call with 'backlog_pending=true'. "
            f"All info calls: {logger.info.call_args_list!r}"
        )

    def test_user_input_no_backlog_no_marker(self, node):
        """Без backlog_pending не должно быть backlog_pending=true в логе."""
        logger = node.get_logger()
        node._on_stt(_stt(node, "робот расскажи анекдот"))
        calls = [
            c
            for c in logger.info.call_args_list
            if c.args and "backlog_pending=true" in str(c.args[0])
        ]
        assert not calls, f"Unexpected backlog_pending=true: {calls!r}"
