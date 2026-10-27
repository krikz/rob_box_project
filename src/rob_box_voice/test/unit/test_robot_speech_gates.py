"""Эхо-гейты микрофона по речи робота (PR-15 15-0b, #3711).

``audio_node`` (сброс буфера, grace после речи) и ``stt_node`` (глушит распознавание при software-AEC, grace при
hardware-AEC) раньше держали каждый свой список строк String-топика состояния TTS — и списки разъезжались
(``stopped``/``tts_silero_warming`` залипали). Теперь оба подписаны одним помощником
``speech_client.watch_speaking`` на ``SpeechEvent``: «звучит» решает ``SpeakingWatch`` (реплика любого агента от
``started`` до ``finished|cancelled|rejected``), нода реагирует только на фронт.

Ноды тянут rclpy/PyAudio/Vosk: модули грузятся с заглушками этих зависимостей (как в корневых тестах ``test/``)
и выгружаются после теста; события идут через настоящую подписку ``watch_speaking``.
"""

from __future__ import annotations

import importlib
import sys
import time
import types
from unittest.mock import MagicMock

import pytest

from rob_box_core import speech_client
from rob_box_core.msg_stub import install_dialog_msgs_stub

M = install_dialog_msgs_stub()
SpeechEvent = M.SpeechEvent
NODE_MODULES = ("rob_box_voice.audio_node", "rob_box_voice.stt_node")


class _Node:
    """rclpy.node.Node без ROS: подписки записываются по топику."""

    def __init__(self, *a, **kw):
        self.subscriptions_by_topic = {}

    def create_subscription(self, msg_type, topic, callback, qos):
        self.subscriptions_by_topic[topic] = (msg_type, callback)

    def get_logger(self):
        return MagicMock()


def _forget_node_modules() -> None:
    import rob_box_voice

    for name in NODE_MODULES:
        sys.modules.pop(name, None)
        leaf = name.rsplit(".", 1)[1]
        if hasattr(rob_box_voice, leaf):
            delattr(rob_box_voice, leaf)


@pytest.fixture
def node_modules(monkeypatch):
    """``(audio_node, stt_node)`` поверх заглушек rclpy/std_msgs/PyAudio/Vosk; после теста модули выгружаются."""
    stubs = {
        "rclpy": types.SimpleNamespace(
            init=None, shutdown=None, spin=None, ok=lambda: False
        ),
        "rclpy.node": types.SimpleNamespace(Node=_Node),
        "rclpy.qos": MagicMock(),
        "std_msgs": MagicMock(),
        "std_msgs.msg": MagicMock(),
        "audio_common_msgs": MagicMock(),
        "audio_common_msgs.msg": MagicMock(),
        "pyaudio": MagicMock(),
        "vosk": MagicMock(),
        "grpc": MagicMock(),
        "rob_box_voice.utils.audio_utils": MagicMock(),
        "rob_box_voice.utils.respeaker_interface": MagicMock(),
    }
    for name, module in stubs.items():
        monkeypatch.setitem(sys.modules, name, module)
    _forget_node_modules()
    yield tuple(importlib.import_module(name) for name in NODE_MODULES)
    _forget_node_modules()


def _event(kind: int, sid: str = "s1", agent: str = "personality"):
    return SpeechEvent(kind=kind, speech_id=sid, agent=agent)


def _audio_node(module):
    node = module.AudioNode.__new__(module.AudioNode)
    _Node.__init__(node)
    node.get_logger = lambda: MagicMock()
    node.tts_active, node._tts_ended_at, node.tts_grace_s = False, 0.0, 2.5
    node.music_active, node.barge_in_with_music = False, True
    node.speech_audio_buffer, node.speech_prefetch_buffer, node.is_speeching = (
        b"echo",
        b"tail",
        True,
    )
    node._robot_speech = speech_client.watch_speaking(node, node._on_robot_speech)
    return node


def _stt_node(module, aec_mode: str):
    node = module.STTNode.__new__(module.STTNode)
    _Node.__init__(node)
    node.get_logger = lambda: MagicMock()
    node.aec_mode, node.is_robot_speaking, node._tts_ended_at = aec_mode, False, 0.0
    node._robot_speech = speech_client.watch_speaking(node, node._on_robot_speech)
    return node


def _say(node, kind: int, sid: str = "s1", agent: str = "personality") -> None:
    msg_type, callback = node.subscriptions_by_topic[speech_client.SPEECH_EVENT_TOPIC]
    assert msg_type is SpeechEvent
    callback(_event(kind, sid, agent))


def test_audio_node_gate_follows_robot_speech_and_opens_grace_after_it(node_modules):
    """started: эхо в буферах сброшено, VAD проходит (barge-in #993); finished: grace — VAD не речь (#989)."""
    node = _audio_node(node_modules[0])
    _say(node, SpeechEvent.KIND_STARTED)
    assert node.tts_active is True
    assert (
        node.speech_audio_buffer,
        node.speech_prefetch_buffer,
        node.is_speeching,
    ) == (b"", b"", False)
    assert node._vad_gated(True) is True
    before = time.monotonic()
    _say(node, SpeechEvent.KIND_FINISHED)
    assert node.tts_active is False
    assert node._tts_ended_at >= before
    assert node._vad_gated(True) is False


@pytest.mark.parametrize("end", [SpeechEvent.KIND_CANCELLED, SpeechEvent.KIND_REJECTED])
def test_audio_node_gate_does_not_stick_on_cancel_or_reject(node_modules, end):
    node = _audio_node(node_modules[0])
    _say(node, SpeechEvent.KIND_STARTED)
    _say(node, end)
    assert node.tts_active is False


def test_audio_node_ignores_synthesis_without_sound_and_overlap_ends_with_the_last_reply(
    node_modules,
):
    """queued/synthesized (пред-синтез выступления) — не речь; оператор поверх личности — конец по последней."""
    node = _audio_node(node_modules[0])
    _say(node, SpeechEvent.KIND_QUEUED)
    _say(node, SpeechEvent.KIND_SYNTHESIZED)
    assert node.tts_active is False and node.speech_audio_buffer == b"echo"
    _say(node, SpeechEvent.KIND_STARTED, "a")
    _say(node, SpeechEvent.KIND_STARTED, "op", agent="operator")
    _say(node, SpeechEvent.KIND_FINISHED, "a")
    assert node.tts_active is True
    _say(node, SpeechEvent.KIND_FINISHED, "op", agent="operator")
    assert node.tts_active is False


@pytest.mark.parametrize("aec_mode", ["software", "hardware"])
def test_stt_node_robot_speaking_follows_speech_event(node_modules, aec_mode):
    """started -> is_robot_speaking (software-AEC глушит фразы); finished -> конец речи с меткой для grace."""
    node = _stt_node(node_modules[1], aec_mode)
    _say(node, SpeechEvent.KIND_STARTED)
    assert node.is_robot_speaking is True
    before = time.monotonic()
    _say(node, SpeechEvent.KIND_FINISHED)
    assert node.is_robot_speaking is False
    assert node._tts_ended_at >= before


def test_stt_node_does_not_stick_on_cancelled_speech(node_modules):
    """STOP/barge-in (cancelled) — конец речи: иначе hardware-AEC молча режет «робот», «стоп», «да» (< 0.8 с)."""
    node = _stt_node(node_modules[1], "hardware")
    _say(node, SpeechEvent.KIND_STARTED)
    _say(node, SpeechEvent.KIND_CANCELLED)
    assert node.is_robot_speaking is False
