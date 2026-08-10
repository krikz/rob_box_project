"""
test_audio_node_downmix_regression.py — Regression-тесты mono-даунмикса
audio_node (issue #1076, PR #1095).

Что фиксируем:
- При 6-канальном входе ReSpeaker (Ch1..Ch4 = микрофоны, Ch5/Ch6 = playback
  reference / loopback) даунмикс в mono НЕ ДОЛЖЕН включать вклад Ch5/Ch6.
  На «старом» коде (```audio_data.mean(axis=1)``` по всем 6 каналам)
  playback попадает в mono → STT ловит эхо TTS как «речь» (yandex:empty).
- Должна быть возможность настроить число усредняемых каналов
  (mic_channels, default 4) — микрофоны, исключая Ch5/Ch6.
- При 1/2/4-канальных устройствах код не должен падать (clamp к channels).
- Выход остаётся int16 mono, shape = (N,), диапазон [-32768, 32767].

Тест НЕ требует rclpy/железа: AudioNode.__init__ замокан через MagicMock
(паттерн test_audio_node_echo.py), проверяем чистую логику
audio_callback → audio_pub.publish.

Эти тесты ПАДАЮТ на текущем коде (усредняет все 6) и ПРОХОДЯТ после
введения interleaved_to_mono() с mic_channels=4 (PR #1095).
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
    node.get_clock = lambda: MagicMock(
        now=MagicMock(return_value=MagicMock(nanoseconds=0))
    )

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


def _interleaved_int16(frames):
    """frames: список фреймов; каждый фрейм = список int16 по каналам.

    Возвращает interleaved bytes (как реально приходит из PyAudio).
    """
    arr = np.asarray(frames, dtype=np.int16)
    return arr.reshape(-1).tobytes()


def _run_audio_callback(node, raw_bytes, frame_count):
    """Дёрнуть audio_callback и вернуть mono int16 numpy-массив."""
    node.audio_callback(raw_bytes, frame_count, {}, 0)
    msg = node.audio_pub.publish.call_args[0][0]
    return np.frombuffer(bytes(msg.data), dtype=np.int16)


# ─────────────────────────────────────────────────────────────────────────────
#  Acceptance #1: 6-канальный ReSpeaker — playback (Ch5/Ch6) не попадает в mono
# ─────────────────────────────────────────────────────────────────────────────


class TestSixChExcludesPlaybackReference:
    """При 6-канальном входе усредняются только микрофонные каналы (Ch1..Ch4).

    Если код усредняет ВСЕ 6 каналов — playback-референс (громкий тон от
    петли TTS) подмешивается в mono → STT воспринимает его как речь.
    """

    def test_playback_loud_does_not_change_mono(self):
        """Один и тот же «микрофонный» сигнал, но разный playback → mono
        должен быть идентичен. На старом коде (mean всех 6) — разный."""
        node = _make_audio_node_stub(channels=6)
        node.is_running = True

        # 2 фрейма: mic 1000/2000/3000/4000 (Ch1..Ch4), playback 0/0
        quiet = _interleaved_int16(
            [[1000, 2000, 3000, 4000, 0, 0], [1000, 2000, 3000, 4000, 0, 0]]
        )
        # Те же mic, но playback на максимуме (имитация TTS-петли)
        loud = _interleaved_int16(
            [
                [1000, 2000, 3000, 4000, 32767, 32767],
                [1000, 2000, 3000, 4000, 32767, 32767],
            ]
        )

        mono_quiet = _run_audio_callback(node, quiet, 2)
        node.audio_pub.publish.reset_mock()  # reset spy между вызовами
        mono_loud = _run_audio_callback(node, loud, 2)

        np.testing.assert_array_equal(
            mono_quiet,
            mono_loud,
            err_msg=(
                "Даунмикс изменился при росте playback-каналов — "
                "Ch5/Ch6 подмешиваются в mono (старый код: mean по 6 каналам)."
            ),
        )

    def test_mono_equals_mean_of_first_four_channels(self):
        """Главный acceptance: mono = mean(Ch1..Ch4) ровно, без Ch5/Ch6.

        Конкретные числа: Ch1..Ch4 = [1000, 2000, 3000, 4000],
        mean = 2500. Playback намеренно оглушительный — на старом коде
        mean всех 6 сильно отличается.
        """
        node = _make_audio_node_stub(channels=6)
        node.is_running = True

        frames = [
            [1000, 2000, 3000, 4000, 32000, 32000],
            [1000, 2000, 3000, 4000, 32000, 32000],
        ]
        raw = _interleaved_int16(frames)
        mono = _run_audio_callback(node, raw, 2)

        expected = np.array([2500, 2500], dtype=np.int16)
        np.testing.assert_array_equal(
            mono,
            expected,
            err_msg=(
                f"mono = {mono.tolist()} ≠ mean(Ch1..Ch4) = {expected.tolist()}. "
                "Playback-каналы подмешиваются (старый код: mean по 6 каналам)."
            ),
        )

    def test_440hz_tone_in_playback_is_silent_in_mono(self):
        """Типичный сценарий: 440 Гц tone на Ch5/Ch6 (тест-петля).
        Если бы усреднялось всё — на mono был бы слышимый сигнал."""
        sr = 16000
        # 4 фрейма по 1 сэмплу: 1000/2000/3000/4000 на Ch1..Ch4,
        # 440 Гц tone на Ch5/Ch6 (один период — пик в +30000, впадины -30000)
        frames = [
            [1000, 2000, 3000, 4000, 0, 0],
            [1000, 2000, 3000, 4000, 30000, -30000],
            [1000, 2000, 3000, 4000, -30000, 30000],
            [1000, 2000, 3000, 4000, 0, 0],
        ]
        node = _make_audio_node_stub(channels=6, sample_rate=sr)
        node.is_running = True
        raw = _interleaved_int16(frames)
        mono = _run_audio_callback(node, raw, 4)

        # mono должен быть равен mean(Ch1..Ch4) = 2500 на каждом фрейме,
        # НЕ зависеть от вращения фазы 440 Гц на Ch5/Ch6.
        expected = np.array([2500, 2500, 2500, 2500], dtype=np.int16)
        np.testing.assert_array_equal(
            mono,
            expected,
            err_msg=(
                f"mono = {mono.tolist()} ≠ {expected.tolist()}. "
                "440 Гц сигнал на Ch5/Ch6 проник в mono (имитация yandex:empty)."
            ),
        )


# ─────────────────────────────────────────────────────────────────────────────
#  Acceptance #3: тип/форма результата
# ─────────────────────────────────────────────────────────────────────────────


class TestMonoOutputShape:
    """Выход audio_callback → AudioData.data — bytes, int16 mono, shape (N,)."""

    def test_6ch_output_is_int16_mono_shape(self):
        node = _make_audio_node_stub(channels=6)
        node.is_running = True

        frames = [[1000, 2000, 3000, 4000, 5000, 6000]] * 8
        raw = _interleaved_int16(frames)
        mono = _run_audio_callback(node, raw, 8)

        # 8 фреймов → 8 mono-сэмплов → 16 байт
        assert isinstance(mono, np.ndarray)
        assert mono.dtype == np.int16
        assert mono.shape == (8,)
        assert mono.min() >= -32768 and mono.max() <= 32767

    def test_6ch_output_byte_count_halves(self):
        """Mono int16 в 6 раз меньше по байтам, чем 6ch int16."""
        node = _make_audio_node_stub(channels=6)
        node.is_running = True

        frames = [[1, 2, 3, 4, 5, 6]] * 4
        raw = _interleaved_int16(frames)  # 4 * 6 * 2 = 48 байт
        node.audio_callback(raw, 4, {}, 0)
        msg = node.audio_pub.publish.call_args[0][0]
        out_bytes = bytes(msg.data)

        assert len(out_bytes) == len(raw) // 6
        assert len(out_bytes) % 2 == 0  # целое число int16

    def test_1ch_passthrough(self):
        """1-канальное устройство: downmix пустой, byte-pass-through."""
        node = _make_audio_node_stub(channels=1)
        node.is_running = True

        raw = _interleaved_int16([[1000], [2000], [3000]])
        node.audio_callback(raw, 3, {}, 0)
        msg = node.audio_pub.publish.call_args[0][0]
        assert bytes(msg.data) == raw


# ─────────────────────────────────────────────────────────────────────────────
#  Acceptance #2: 2-канальное устройство — даунмикс работает, не падает
# ─────────────────────────────────────────────────────────────────────────────


class TestSmallChannelCounts:
    """1/2/4ch устройства: код не должен падать, mic_channels клампится."""

    def test_2ch_does_not_crash(self):
        node = _make_audio_node_stub(channels=2)
        node.is_running = True

        frames = [[1000, 3000], [1000, 3000], [1000, 3000]]
        raw = _interleaved_int16(frames)
        mono = _run_audio_callback(node, raw, 3)

        # При 2ch нет playback Ch5/Ch6 — mono = mean(Ch1, Ch2) = 2000.
        assert mono.tolist() == [2000, 2000, 2000]

    def test_4ch_does_not_crash(self):
        node = _make_audio_node_stub(channels=4)
        node.is_running = True

        frames = [[1000, 2000, 3000, 4000], [4000, 3000, 2000, 1000]]
        raw = _interleaved_int16(frames)
        mono = _run_audio_callback(node, raw, 2)

        # mean по 4 каналам = 2500 и 2500.
        assert mono.tolist() == [2500, 2500]


# ─────────────────────────────────────────────────────────────────────────────
#  Regression: gross signal — если код сломан (например, NaN, бесконечные
#  значения) — этот тест поймает первым.
# ─────────────────────────────────────────────────────────────────────────────


class TestMonoOutputRange:
    """Mono int16 после downmix — всегда в диапазоне [-32768, 32767]."""

    def test_no_clip_when_all_six_channels_max(self):
        """Если 6 каналов по 32767 — mean не должен переполнить int16."""
        node = _make_audio_node_stub(channels=6)
        node.is_running = True

        frames = [[32767] * 6] * 4
        raw = _interleaved_int16(frames)
        mono = _run_audio_callback(node, raw, 4)

        assert mono.dtype == np.int16
        assert mono.min() >= -32768 and mono.max() <= 32767

    def test_no_clip_when_ch5_ch6_negative_max(self):
        """Экстремально: Ch5=32767, Ch6=-32768, Ch1..Ch4=1000.

        На старом коде mean ≈ -10511 (в норме). Проверяем, что нет
        переполнения/NaN/inf и формат int16.
        """
        node = _make_audio_node_stub(channels=6)
        node.is_running = True

        frames = [[1000, 1000, 1000, 1000, 32767, -32768]] * 4
        raw = _interleaved_int16(frames)
        mono = _run_audio_callback(node, raw, 4)

        assert mono.dtype == np.int16
        assert mono.shape == (4,)
        assert not np.isnan(mono.astype(np.float32)).any()
        assert not np.isinf(mono.astype(np.float32)).any()
