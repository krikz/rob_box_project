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
        mix_channels=[],  # issue 1076: пусто = все каналы (legacy)
        chunk_size=4096,  # issue #1050: 1024 → 4096 (paInputOverflow fix)
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
    """Fix B (989) + Fix 993 barge-in:

    - После окончания TTS в течение tts_grace_s — VAD не речь (эхо).
    - Во время самого TTS VAD НЕ гейтится (issue 993): barge-in должен
      работать. Wake-word gate в dialogue_node отсекает эхо без
      помощи VAD-гейта.
    """

    def test_default_grace_2_5s(self, audio_node):
        assert audio_node.tts_grace_s == 2.5

    def test_vad_true_during_tts_active_for_barge_in(self, audio_node):
        """Issue 993: во время TTS VAD пропускает речь — barge-in работает.

        Раньше (Fix B из 989) VAD гейтился пока робот говорит, поэтому
        пользователь не мог перебить рэп командой. Теперь гейт снят —
        wake-word в dialogue_node сам отсекает эхо собственного голоса.
        Симулируем «TTS сейчас играет» через _on_tts_state(playing):
        он выставляет tts_active=True, и _vad_gated пропускает VAD
        (issue 993 barge-in).
        """
        audio_node.music_active = False
        playing = MagicMock()
        playing.data = "playing"
        audio_node._on_tts_state(playing)
        assert audio_node.tts_active is True
        assert audio_node._vad_gated(True) is True  # barge-in!

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
        audio_node._tts_ended_at = 0.0  # «давно, не в grace»
        assert audio_node._vad_gated(False) is False

    def test_tts_state_transitions(self, audio_node):
        """_on_tts_state: playing → reset буфера, ready → grace с timestamp."""
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
        assert audio_node._tts_ended_at > 0.0  # момент окончания зафиксирован


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
        audio_node._tts_ended_at = 0.0  # «давно, не в grace»
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


class TestInputOverflowHandling:
    """Issue 1050: paInputOverflow (PyAudio status=2) — rate-limited лог.

    status=2 == paInputOverflow: входной буфер переполнен, сэмплы потеряны
    (Python-callback не успел за периодом). Callback должен (а) не падать
    на коротком in_data, (б) продолжать публикацию, (в) логировать не чаще
    раза в окно, (г) сообщать сколько байт потеряно.
    """

    @staticmethod
    def _capture_warnings(audio_node):
        warnings = []

        def _logger():
            return MagicMock(
                info=lambda *a, **kw: None,
                warning=lambda *a, **kw: warnings.append(a[0] if a else ""),
                warn=lambda *a, **kw: None,
                error=lambda *a, **kw: None,
                debug=lambda *a, **kw: None,
            )

        audio_node.get_logger = _logger
        return warnings

    def test_overflow_short_chunk_no_crash_and_publishes(self, audio_node):
        """Короткий in_data при overflow (потеря кадров) — без исключений."""
        audio_node.is_running = True
        short_data = b"\x00\x00" * 256  # 512 байт вместо ожидаемых 2048
        audio_node.audio_callback(short_data, 1024, {}, 2)
        audio_node.audio_pub.publish.assert_called_once()
        # prefetch продолжает накапливаться
        assert len(audio_node.speech_prefetch_buffer) == 512

    def test_overflow_no_log_when_status_0(self, audio_node):
        audio_node.is_running = True
        warnings = self._capture_warnings(audio_node)
        audio_node.audio_callback(b"\x00\x00" * 1024, 1024, {}, 0)
        assert audio_node._overflow_count == 0
        assert warnings == []

    def test_overflow_rate_limited_log(self, audio_node):
        """100 переполнений подряд → одна строка в лог, счётчик растёт."""
        audio_node.is_running = True
        warnings = self._capture_warnings(audio_node)
        for _ in range(100):
            audio_node.audio_callback(b"\x00\x00" * 1024, 1024, {}, 2)
        assert audio_node._overflow_count == 100
        assert len(warnings) == 1
        assert "paInputOverflow" in warnings[0]

    def test_overflow_log_again_after_window(self, audio_node):
        """После окончания окна следующее переполнение снова логируется."""
        audio_node.is_running = True
        warnings = self._capture_warnings(audio_node)
        audio_node.audio_callback(b"\x00\x00" * 1024, 1024, {}, 2)
        assert len(warnings) == 1
        # «Окно» прошло — следующее переполнение даёт вторую строку
        audio_node._overflow_last_logged = 0.0
        audio_node.audio_callback(b"\x00\x00" * 1024, 1024, {}, 2)
        assert audio_node._overflow_count == 2
        assert len(warnings) == 2

    def test_overflow_log_reports_lost_bytes(self, audio_node):
        """В логе видно сколько байт ожидалось/пришло/потеряно."""
        audio_node.is_running = True
        warnings = self._capture_warnings(audio_node)
        audio_node.audio_callback(b"\x00\x00" * 256, 1024, {}, 2)
        assert len(warnings) == 1
        assert "512/2048" in warnings[0]  # got/expected
        assert "потеряно ~1536" in warnings[0]


class TestMixChannels:
    """Issue 1076: миксер каналов ReSpeaker 6-канального RAW-режима.

    ReSpeaker Mic Array v2.0 в 6-канальном режиме отдаёт:
      Ch1-4 = сырые микрофоны, Ch5-6 = playback-референс (AEC-референс).
    Раньше audio_callback усреднял ВСЕ 6 каналов, из-за чего собственный
    голос/музыка робота смешивались в STT-сигнал (Yandex `empty`, vosk
    галлюцинировал). mix_channels=[0,1,2,3] исключает референс.
    """

    @staticmethod
    def _interleaved_6ch(frame_count: int) -> bytes:
        """Синтетический 6-канальный кадр: Ch0-3 = тишина, Ch4-5 = громкий тон.

        Возвращает bytes (int16 LE, interleaved) как от PyAudio в RAW-режиме.
        """
        import numpy as np

        frames = np.zeros((frame_count, 6), dtype=np.int16)
        frames[:, 4] = 20000  # playback-референс (Ch5)
        frames[:, 5] = 20000  # playback-референс (Ch6)
        return frames.tobytes()

    def test_mix_channels_excludes_playback_reference(self, audio_node):
        """mix_channels=[0,1,2,3] → референс (Ch5-6) НЕ попадает в моно."""
        audio_node.is_running = True
        audio_node.channels = 6
        audio_node.mix_channels = [0, 1, 2, 3]

        audio_node.audio_callback(self._interleaved_6ch(256), 256, {}, 0)

        published = audio_node.audio_pub.publish.call_args[0][0]
        samples = bytes(published.data)
        # Все сэмплы = 0 (микрофоны тихие, референс исключён)
        assert all(b == 0 for b in samples), "playback-референс попал в моно-микс!"

    def test_legacy_all_channels_keeps_reference(self, audio_node):
        """Пустой mix_channels (legacy) → усредняются ВСЕ каналы, как раньше."""
        audio_node.is_running = True
        audio_node.channels = 6
        audio_node.mix_channels = []

        audio_node.audio_callback(self._interleaved_6ch(256), 256, {}, 0)

        published = audio_node.audio_pub.publish.call_args[0][0]
        samples = bytes(published.data)
        # 4 тихих + 2 громких канала → среднее ≈ 20000*2/6 ≈ 6667 > 0
        assert any(b != 0 for b in samples), "legacy-микс должен содержать референс"

    def test_invalid_mix_channel_index_ignored(self, audio_node):
        """Индексы вне диапазона каналов отбрасываются без падения."""
        audio_node.is_running = True
        audio_node.channels = 6
        audio_node.mix_channels = [0, 1, 2, 3, 99, -1]

        audio_node.audio_callback(self._interleaved_6ch(128), 128, {}, 0)

        published = audio_node.audio_pub.publish.call_args[0][0]
        samples = bytes(published.data)
        assert all(b == 0 for b in samples), "невалидные индексы не должны ломать микс"

    def test_single_channel_passthrough(self, audio_node):
        """channels=1 → данные публикуются как есть (legacy путь, без микса)."""
        audio_node.is_running = True
        audio_node.channels = 1
        audio_node.mix_channels = [0, 1, 2, 3]  # игнорируется для 1 канала

        raw = b"\x10\x00" * 64  # 64 сэмпла int16 = 2000... нет, 0x0010=16
        audio_node.audio_callback(raw, 64, {}, 0)

        published = audio_node.audio_pub.publish.call_args[0][0]
        assert bytes(published.data) == raw


class TestTelemetrySilenceToPhrase:
    """Issue 1076 (телеметрия): честный «замолчал → акцепт».

    audio_node логирует silence_to_phrase_s (включает speech_continuation),
    stt_node логирует phrase_to_accept_ms. Полный замер = сумма.
    """

    class _FakeTime:
        """Мини-заменитель rclpy Time: поддерживает вычитание."""

        def __init__(self, nanos: float):
            self._nanos = nanos

        @property
        def nanoseconds(self) -> float:
            return self._nanos

        def __sub__(self, other: "TestTelemetrySilenceToPhrase._FakeTime"):
            return TestTelemetrySilenceToPhrase._FakeTime(
                self._nanos - other._nanos
            )

    def test_silence_to_phrase_logged(self, audio_node):
        """При публикации speech_audio пишется telemetry-строка с паузой."""
        logs = []

        def _logger():
            return MagicMock(
                info=lambda *a, **kw: logs.append(a[0] if a else ""),
                warning=lambda *a, **kw: None,
                warn=lambda *a, **kw: None,
                error=lambda *a, **kw: None,
                debug=lambda *a, **kw: None,
            )

        audio_node.get_logger = _logger
        audio_node.sample_rate = 16000
        audio_node.speech_min_duration = 0.3
        audio_node.speech_max_duration = 10.0
        audio_node.speech_continuation = 3.0
        audio_node.is_speeching = True
        audio_node.speech_audio_buffer = b"\x00\x00" * 16000  # 1с речи
        audio_node.speech_stopped_time = self._FakeTime(0)
        audio_node.get_clock = lambda: MagicMock(
            now=MagicMock(return_value=self._FakeTime(int(3.5 * 1e9)))
        )
        audio_node.vad_pub = MagicMock()
        audio_node.direction_pub = MagicMock()
        audio_node.speech_audio_pub = MagicMock()
        audio_node.prev_vad = False
        audio_node.tts_active = False
        audio_node._tts_ended_at = 0.0
        audio_node.music_active = False
        audio_node.respeaker = MagicMock()
        audio_node.respeaker.is_connected = MagicMock(return_value=True)
        audio_node.respeaker.get_vad = MagicMock(return_value=False)
        audio_node.respeaker.get_direction = MagicMock(return_value=None)

        audio_node.check_vad_and_doa()

        assert any("silence_to_phrase_s=" in line for line in logs), (
            f"ожидалась telemetry-строка silence_to_phrase_s, логи: {logs}"
        )
        telemetry_line = next(l for l in logs if "silence_to_phrase_s=" in l)
        assert "speech_continuation=3.0" in telemetry_line
        # time_since_stop = 3.5с (>= speech_continuation)
        assert "silence_to_phrase_s=3.50" in telemetry_line
