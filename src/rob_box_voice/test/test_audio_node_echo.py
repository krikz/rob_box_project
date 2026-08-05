#!/usr/bin/env python3
"""
test_audio_node_echo.py — Unit-тесты анти-эхо гейтов audio_node (issue 989).

Проверяет:
- Fix B: grace period после TTS — VAD не считается речью (tts_active,
  _tts_ended_at, tts_grace_s).
- Fix C: при активной музыке (music_active=True) программный гейт по RMS
  (music_vad_min_db) подавляет VAD, порог поднимается на ReSpeaker.

Тест НЕ требует rclpy/железа: AudioNode.__init__ замокан через MagicMock,
тестируем именно чистую логику _vad_gated / _on_tts_state / _on_music_state.
"""

from __future__ import annotations

import sys
import time
from unittest.mock import MagicMock

import pytest

_OPTIONAL_DEPS = {
    "rclpy": True,
    "rclpy.node": True,
    "rclpy.qos": True,
    "std_msgs": True,
    "std_msgs.msg": True,
    "audio_common_msgs": True,
    "audio_common_msgs.msg": True,
    "pyaudio": False,
}


def _node_no_op(self, *a, **kw):
    return None


def _ensure_rclpy_mock(monkeypatch):
    """Минимальный rclpy mock (как в test_stt_node_fallback)."""

    class _NodeBase:
        def __init__(self, *a, **kw):
            pass

        def declare_parameter(self, *a, **kw):
            pass

        def get_parameter(self, name):
            return MagicMock(value="")

        def create_publisher(self, *a, **kw):
            return MagicMock()

        def create_subscription(self, *a, **kw):
            return MagicMock()

        def create_timer(self, *a, **kw):
            return MagicMock()

        def get_logger(self):
            return MagicMock(
                info=_node_no_op,
                warning=_node_no_op,
                warn=_node_no_op,
                error=_node_no_op,
                debug=_node_no_op,
            )

        def get_clock(self):
            return MagicMock(now=MagicMock(return_value=MagicMock(nanoseconds=0)))

    class _NodeMod:
        Node = _NodeBase

    class _Rclpy:
        node = _NodeMod()

        @staticmethod
        def init(*a, **kw):
            pass

        @staticmethod
        def shutdown(*a, **kw):
            pass

        @staticmethod
        def spin(*a, **kw):
            pass

    monkeypatch.setitem(sys.modules, "rclpy", _Rclpy())
    monkeypatch.setitem(sys.modules, "rclpy.node", _Rclpy.node)

    class _QoSMod:
        QoSProfile = MagicMock()
        ReliabilityPolicy = MagicMock()
        DurabilityPolicy = MagicMock()

    monkeypatch.setitem(sys.modules, "rclpy.qos", _QoSMod())

    class _Msg:
        Bool = MagicMock()
        Int32 = MagicMock()
        String = MagicMock()

    monkeypatch.setitem(sys.modules, "std_msgs", _Msg())
    monkeypatch.setitem(sys.modules, "std_msgs.msg", _Msg)

    class _AudioMsg:
        AudioData = MagicMock()

    monkeypatch.setitem(sys.modules, "audio_common_msgs", _AudioMsg())
    monkeypatch.setitem(sys.modules, "audio_common_msgs.msg", _AudioMsg)

    monkeypatch.setitem(sys.modules, "pyaudio", MagicMock())
    monkeypatch.setitem(
        sys.modules,
        "rob_box_voice.utils.audio_utils",
        MagicMock(
            calculate_rms=MagicMock(return_value=0.1),
            calculate_db=MagicMock(return_value=-20.0),
            find_respeaker_device=MagicMock(return_value=0),
            list_audio_devices=MagicMock(return_value=[]),
        ),
    )
    monkeypatch.setitem(
        sys.modules,
        "rob_box_voice.utils.respeaker_interface",
        MagicMock(ReSpeakerInterface=MagicMock),
    )


@pytest.fixture(autouse=True)
def _ensure_optional_deps(monkeypatch):
    for cached in ["rob_box_voice.audio_node"]:
        sys.modules.pop(cached, None)
    _ensure_rclpy_mock(monkeypatch)
    yield


def _make_audio_node_stub(**param_overrides):
    """Создаёт AudioNode-инстанс без rclpy и без железа."""
    from rob_box_voice import audio_node as audio_node_module

    defaults = dict(
        sample_rate=16000,
        channels=1,
        chunk_size=1024,
        vad_threshold=3.5,
        publish_rate=10,
        device_index=-1,
        device_name="ReSpeaker 4 Mic Array",
        speech_continuation=3.0,
        speech_prefetch=0.5,
        speech_min_duration=0.3,
        speech_max_duration=10.0,
        tts_grace_s=2.5,
        music_vad_threshold=6.0,
        music_vad_min_db=-35.0,
    )
    defaults.update(param_overrides)

    node = audio_node_module.AudioNode.__new__(audio_node_module.AudioNode)
    _params = dict(defaults)

    def _declare(name, value, *a, **kw):
        _params[name] = value

    def _get_param(name):
        return MagicMock(value=_params.get(name, ""))

    node.declare_parameter = _declare
    node.get_parameter = _get_param
    node.get_logger = lambda: MagicMock(
        info=lambda *a, **kw: None,
        warning=lambda *a, **kw: None,
        warn=lambda *a, **kw: None,
        error=lambda *a, **kw: None,
        debug=lambda *a, **kw: None,
    )
    node.create_publisher = lambda *a, **kw: MagicMock()
    node.create_subscription = lambda *a, **kw: MagicMock()
    node.create_timer = lambda *a, **kw: MagicMock()
    node.get_clock = lambda: MagicMock(now=MagicMock(return_value=MagicMock(nanoseconds=0)))

    audio_node_module.AudioNode.__init__(node)

    for k, v in defaults.items():
        setattr(node, k, v)
    node.respeaker = MagicMock()
    node.respeaker.is_connected = MagicMock(return_value=False)
    node.respeaker.set_vad_threshold = MagicMock(return_value=True)
    node.audio_pub = MagicMock()
    node.speech_audio_pub = MagicMock()
    node.vad_pub = MagicMock()
    node.direction_pub = MagicMock()
    node.state_pub = MagicMock()
    node.tts_control_pub = MagicMock()
    return node


@pytest.fixture
def audio_node():
    return _make_audio_node_stub()


class TestTTSGraceGate:
    """Fix B: пока TTS активен или в течение tts_grace_s — VAD не речь."""

    def test_default_grace_2_5s(self, audio_node):
        assert audio_node.tts_grace_s == 2.5

    def test_vad_false_when_tts_active(self, audio_node):
        audio_node.tts_active = True
        assert audio_node._vad_gated(True) is False

    def test_vad_false_inside_grace_after_tts(self, audio_node):
        audio_node.tts_active = False
        audio_node.tts_grace_s = 2.5
        audio_node._tts_ended_at = time.monotonic() - 1.0  # 1с назад — внутри grace
        assert audio_node._vad_gated(True) is False

    def test_vad_true_outside_grace(self, audio_node):
        audio_node.tts_active = False
        audio_node.tts_grace_s = 2.5
        audio_node._tts_ended_at = time.monotonic() - 10.0  # вне grace
        audio_node.music_active = False
        assert audio_node._vad_gated(True) is True

    def test_vad_false_when_hardware_silence(self, audio_node):
        audio_node.tts_active = False
        audio_node._tts_ended_at = 0.0
        assert audio_node._vad_gated(False) is False

    def test_tts_state_transitions(self, audio_node):
        """_on_tts_state: playing → гейт, ready → grace сбрасывает буфер."""
        audio_node.speech_audio_buffer = b"some-echo-data"
        audio_node.is_speeching = True

        playing = MagicMock()
        playing.data = "playing"
        audio_node._on_tts_state(playing)
        assert audio_node.tts_active is True
        assert audio_node.speech_audio_buffer == b""  # эхо сброшено
        assert audio_node.is_speeching is False

        ready = MagicMock()
        ready.data = "ready"
        audio_node._on_tts_state(ready)
        assert audio_node.tts_active is False
        assert audio_node._tts_ended_at > 0.0


class TestMusicStrictGate:
    """Fix C: при активной музыке VAD гейтится по RMS, порог поднимается."""

    def test_music_active_raises_threshold_on_respeaker(self, audio_node):
        audio_node.respeaker.is_connected = MagicMock(return_value=True)
        audio_node.respeaker.set_vad_threshold = MagicMock(return_value=True)

        playing = MagicMock()
        playing.data = "playing"
        audio_node._on_music_state(playing)

        assert audio_node.music_active is True
        audio_node.respeaker.set_vad_threshold.assert_called_with(6.0)

    def test_music_idle_restores_threshold(self, audio_node):
        audio_node.respeaker.is_connected = MagicMock(return_value=True)
        audio_node.respeaker.set_vad_threshold = MagicMock(return_value=True)
        audio_node.music_active = True

        idle = MagicMock()
        idle.data = "idle"
        audio_node._on_music_state(idle)

        assert audio_node.music_active is False
        audio_node.respeaker.set_vad_threshold.assert_called_with(3.5)

    def test_vad_suppressed_when_music_and_quiet_signal(self, audio_node):
        """Музыка активна, уровень сигнала ниже music_vad_min_db → VAD подавлен."""
        audio_node.tts_active = False
        audio_node._tts_ended_at = 0.0
        audio_node.music_active = True
        audio_node.music_vad_min_db = -35.0
        audio_node._current_db = -50.0  # тихо — это музыка/шум

        assert audio_node._vad_gated(True) is False

    def test_vad_passes_when_music_and_loud_signal(self, audio_node):
        """Музыка активна, но уровень сигнала высокий (голос поверх) → VAD проходит."""
        audio_node.tts_active = False
        audio_node._tts_ended_at = 0.0
        audio_node.music_active = True
        audio_node.music_vad_min_db = -35.0
        audio_node._current_db = -20.0  # громко — голос поверх музыки

        assert audio_node._vad_gated(True) is True

    def test_music_state_noop_on_same_state(self, audio_node):
        audio_node.respeaker.is_connected = MagicMock(return_value=True)
        audio_node.respeaker.set_vad_threshold = MagicMock(return_value=True)
        audio_node.music_active = True

        playing = MagicMock()
        playing.data = "playing"
        audio_node._on_music_state(playing)

        # Повторный "playing" не дёргает set_vad_threshold снова
        audio_node.respeaker.set_vad_threshold.assert_not_called()
