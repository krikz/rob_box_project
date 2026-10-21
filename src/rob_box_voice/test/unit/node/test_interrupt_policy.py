"""Node-тесты одной политики перебивания (ADR-0150 §6.2, PR-5 #3694; приёмка A12/A16).

Новая адресованная фраза принимает ход (ход в полёте отменяется), но речь не глушит — рвёт её только ясная
стоп-команда по ``/dialog/interrupt_request`` (``InterruptGate``). Фикстура — ``object.__new__(DialogueNode)``
с ручными атрибутами, без ROS2 (как в соседних тестах ``unit/node``).
"""

from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

from rob_box_harness.core.dialogue_state_machine import DialogueStateKind
from rob_box_voice.core.interrupt_request import InterruptGate
from rob_box_voice.dialogue_node import DialogueNode
from ros_stubs import make_utterance  # noqa: E402


@pytest.fixture
def node():
    n = object.__new__(DialogueNode)
    logger = MagicMock()
    n.get_logger = lambda: logger
    n._wake_words = ["робок", "робот", "роббокс", "робокс", "robbox", "rob box"]
    n._dsm = MagicMock()
    n._dsm.current_state = DialogueStateKind.DIALOGUE
    n._sound_trigger_pub = MagicMock()
    n._publish_state = MagicMock()
    n._dispatch_turn = MagicMock()
    n._verbose_llm = False
    n._speaker_by_text = {}
    n._llm_skipped_counter = {k: 0 for k in (
        "no_wake_word", "silenced", "silence_command", "empty_after_strip", "stt_rejected", "music_stop",
        "command_intent")}
    n._maybe_log_skip_summary = MagicMock()
    n._active_tg_chat_id = None
    n._run_task = None
    n._task_lock = MagicMock()
    n._task_lock.__enter__ = MagicMock(return_value=None)
    n._task_lock.__exit__ = MagicMock(return_value=False)
    n._loop = MagicMock()
    n._loop.call_soon_threadsafe = lambda fn, *a, **kw: fn(*a, **kw)
    n._run_cancelled = False
    n._effects = MagicMock()
    n._speech_client_pubs = {"cancel": MagicMock()}
    n._track_mode_music_active = False
    n._action_claim_retry_used = False
    return n


class TestNewPhraseTakesTurnWithoutCuttingSpeech:
    def test_addressed_phrase_does_not_publish_speech_cancel(self, node):
        """«и ещё про енота» во время куплета: куплет доигрывает (A16: речь рвёт не каждый вейк)."""
        node._on_stt(make_utterance("робот и ещё про енота"))
        node._speech_client_pubs["cancel"].publish.assert_not_called()

    def test_turn_in_flight_is_cancelled_and_awaiters_released(self, node):
        """Даже без отмены речи ход отменяется, а awaiter'ы отпускаются — иначе _tts_events залипнут."""
        task = MagicMock()
        task.done.return_value = False
        node._run_task = task
        node._on_stt(make_utterance("робот расскажи другое"))
        assert node._run_cancelled is True
        node._loop  # call_soon_threadsafe вызывает task.cancel синхронно
        task.cancel.assert_called_once()
        node._effects.release_all_tts.assert_called_once()
        node._effects.clear_sound_event.assert_called_once()
        assert node._handed_over_task is task

    def test_phrase_is_dispatched(self, node):
        node._on_stt(make_utterance("робот и ещё про енота"))
        node._dispatch_turn.assert_called_once()


class TestInterruptGateOnNode:
    def _gate(self, node):
        epoch = SimpleNamespace(current=7)
        node._session_epoch_gate = lambda: epoch
        from rob_box_core import speech_client

        return InterruptGate(
            lambda reason: speech_client.cancel(node, agent="personality", epoch=epoch.current, reason=reason),
            lambda: node._wake_words, node.get_logger().info)

    def test_clear_stop_cancels_current_epoch_speech(self, node):
        gate = self._gate(node)
        gate.on_request(SimpleNamespace(utterance_id="u1", text_partial="робот хватит"))
        sent = node._speech_client_pubs["cancel"].publish.call_args.args[0]
        assert (sent.agent, sent.epoch, sent.reason) == ("personality", 7, "interrupt:u1")

    def test_new_request_does_not_cancel(self, node):
        gate = self._gate(node)
        gate.on_request(SimpleNamespace(utterance_id="u2", text_partial="робот спой про стоп"))
        gate.on_request(SimpleNamespace(utterance_id="u3", text_partial="робот который час"))
        node._speech_client_pubs["cancel"].publish.assert_not_called()


def test_unknown_param_warns_not_rejected(node):
    """Политики перебивания больше нет — любой runtime-SetParameters не роняет ноду (как и раньше)."""
    result = node.parameters_callback([SimpleNamespace(name="voice_preset", value="technical")])
    assert result.successful is True
    node.get_logger().warning.assert_called()
