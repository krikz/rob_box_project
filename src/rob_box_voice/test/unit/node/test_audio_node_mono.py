"""
test_audio_node_mono.py — Unit-тесты mono-даунмикса audio_node (issue #1076).

Проверяет:
- interleaved_to_mono() исключает playback-reference каналы (Ch5/Ch6)
  из mean при 6-канальном входе ReSpeaker (mic_channels=4).
- Выход остаётся int16 mono (2 байта/сэмпл).
- Совместимость с другими устройствами: 1/2/4 канала даунмиксятся
  корректно, mic_channels кламперуется к числу каналов.
- Параметр mic_channels настраивается через конфиг (stub override)
  и env MIC_CHANNELS.

Тест НЕ требует rclpy/железа: AudioNode.__init__ замокан через
MagicMock (паттерн из test_audio_node_echo.py), тестируем чистую логику.
"""

from __future__ import annotations

import sys
from unittest.mock import MagicMock

import numpy as np
import pytest


def _node_no_op(self, *a, **kw):
    return None


def _ensure_rclpy_mock(monkeypatch):
    """Минимальный rclpy mock (как в test_audio_node_echo.py)."""

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
        # NOTE: mic_channels НЕ в defaults — его default вычисляет __init__
        # (env MIC_CHANNELS → 4). Здесь он попадёт в defaults только через
        # param_overrides (эмуляция конфига audio_node.yaml).
        chunk_size=4096,
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


# ─────────────────────────────────────────────────────────────────────────────
#  interleaved_to_mono — чистая функция даунмикса
# ─────────────────────────────────────────────────────────────────────────────

def _interleaved(frames: list[list[int]]) -> bytes:
    """Собрать interleaved PCM16 bytes из списка фреймов (список каналов)."""
    arr = np.asarray(frames, dtype=np.int16)
    return arr.reshape(-1).tobytes()


@pytest.fixture
def mono_fn():
    """Ленивый импорт interleaved_to_mono (после установки rclpy-моков)."""
    from rob_box_voice.audio_node import interleaved_to_mono

    return interleaved_to_mono


class TestInterleavedToMono:
    def test_6ch_excludes_playback_reference(self, mono_fn):
        """Ch5/Ch6 (playback) не влияют на результат — mean только Ch1..Ch4.

        Микрофоны дают 1000/2000/3000/4000 (среднее 2500), а playback
        каналы — 32000 (громкое эхо TTS). Если бы playback усреднялся,
        среднее было бы ~12333. Проверяем именно 2500.
        """
        frames = [
            [1000, 2000, 3000, 4000, 32000, 32000],
            [1000, 2000, 3000, 4000, 32000, 32000],
        ]
        out = mono_fn(_interleaved(frames), channels=6, mic_channels=4)
        mono = np.frombuffer(out, dtype=np.int16)
        assert mono.tolist() == [2500, 2500]

    def test_6ch_playback_value_irrelevant(self, mono_fn):
        """Изменение playback-каналов НЕ меняет mono (главный acceptance)."""
        base = _interleaved([
            [1000, 2000, 3000, 4000, 0, 0],
            [1000, 2000, 3000, 4000, 0, 0],
        ])
        loud = _interleaved([
            [1000, 2000, 3000, 4000, 32767, -32768],
            [1000, 2000, 3000, 4000, 32767, -32768],
        ])
        assert mono_fn(base, 6, 4) == mono_fn(loud, 6, 4)

    def test_output_is_int16_mono(self, mono_fn):
        """Выход — ровно 2 байта на сэмпл (int16), фреймов столько же."""
        frames = [[1000, 2000, 3000, 4000, 5000, 6000]] * 8
        out = mono_fn(_interleaved(frames), channels=6, mic_channels=4)
        assert len(out) == 8 * 2  # 8 моно-сэмплов int16
        assert len(out) % 2 == 0

    def test_4ch_averages_all(self, mono_fn):
        """4-канальное устройство: mic_channels=4 → усредняются все 4."""
        frames = [[1000, 2000, 3000, 4000], [1000, 2000, 3000, 4000]]
        out = mono_fn(_interleaved(frames), channels=4, mic_channels=4)
        assert np.frombuffer(out, dtype=np.int16).tolist() == [2500, 2500]

    def test_2ch_averages_both(self, mono_fn):
        """2-канальное устройство: mic_channels кламперуется к channels."""
        frames = [[1000, 3000], [1000, 3000]]
        out = mono_fn(_interleaved(frames), channels=2, mic_channels=4)
        assert np.frombuffer(out, dtype=np.int16).tolist() == [2000, 2000]

    def test_1ch_identity(self, mono_fn):
        """1-канальное устройство: даунмикс = сам сигнал."""
        frames = [[1234], [5678]]
        out = mono_fn(_interleaved(frames), channels=1, mic_channels=4)
        assert np.frombuffer(out, dtype=np.int16).tolist() == [1234, 5678]

    def test_mic_channels_gt_channels_clamped(self, mono_fn):
        """mic_channels > channels → используем все каналы (без исключений)."""
        frames = [[1000, 2000, 3000, 4000, 5000, 6000]]
        out = mono_fn(_interleaved(frames), channels=6, mic_channels=99)
        mono = np.frombuffer(out, dtype=np.int16)
        assert mono.tolist() == [3500]  # (1000+...+6000)/6

    def test_mic_channels_zero_or_negative_uses_one(self, mono_fn):
        """mic_channels <= 0 → защитный fallback на 1 канал."""
        frames = [[1000, 9000, 9000, 9000, 9000, 9000]]
        out = mono_fn(_interleaved(frames), channels=6, mic_channels=0)
        assert np.frombuffer(out, dtype=np.int16).tolist() == [1000]

    def test_short_chunk_returns_input(self, mono_fn):
        """Чанк короче одного полного фрейма (overflow) — не падаем."""
        partial = b"\x01\x00" * 3  # 3 сэмпла при 6 каналах — нет ни одного фрейма
        out = mono_fn(partial, channels=6, mic_channels=4)
        assert out == partial


# ─────────────────────────────────────────────────────────────────────────────
#  audio_callback — публикация mono через ноду
# ─────────────────────────────────────────────────────────────────────────────

class TestAudioCallbackMono:
    def _run_callback(self, frames, channels, mic_channels):
        node = _make_audio_node_stub(channels=channels, mic_channels=mic_channels)
        node.is_running = True
        node.audio_callback(_interleaved(frames), len(frames), {}, 0)
        msg = node.audio_pub.publish.call_args[0][0]
        return np.frombuffer(bytes(msg.data), dtype=np.int16).tolist()

    def test_6ch_node_publishes_mic_only_mono(self):
        frames = [[1000, 2000, 3000, 4000, 32000, 32000]] * 2
        mono = self._run_callback(frames, channels=6, mic_channels=4)
        assert mono == [2500, 2500]

    def test_2ch_node_publishes_stereo_mean(self):
        frames = [[1000, 3000], [1000, 3000]]
        mono = self._run_callback(frames, channels=2, mic_channels=4)
        assert mono == [2000, 2000]

    def test_1ch_node_passthrough(self):
        node = _make_audio_node_stub(channels=1, mic_channels=4)
        node.is_running = True
        raw = _interleaved([[1000], [2000]])
        node.audio_callback(raw, 2, {}, 0)
        msg = node.audio_pub.publish.call_args[0][0]
        assert bytes(msg.data) == raw

    def test_default_mic_channels_is_4(self):
        node = _make_audio_node_stub()  # без override
        assert node.mic_channels == 4

    def test_mic_channels_override_via_config(self):
        node = _make_audio_node_stub(channels=6, mic_channels=2)
        assert node.mic_channels == 2
        frames = [[1000, 3000, 9000, 9000, 9000, 9000]]
        mono = self._run_callback(frames, channels=6, mic_channels=2)
        assert mono == [2000]  # mean только Ch1/Ch2


class TestMicChannelsEnv:
    def test_env_override(self, monkeypatch):
        monkeypatch.setenv("MIC_CHANNELS", "2")
        node = _make_audio_node_stub(channels=6)
        assert node.mic_channels == 2

    def test_env_invalid_falls_back_to_4(self, monkeypatch):
        monkeypatch.setenv("MIC_CHANNELS", "abc")
        node = _make_audio_node_stub(channels=6)
        assert node.mic_channels == 4

    def test_no_env_default_4(self, monkeypatch):
        monkeypatch.delenv("MIC_CHANNELS", raising=False)
        node = _make_audio_node_stub(channels=6)
        assert node.mic_channels == 4
