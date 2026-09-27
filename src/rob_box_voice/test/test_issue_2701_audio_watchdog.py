#!/usr/bin/env python3
"""
test_issue_2701_audio_watchdog.py — watchdog захвата аудио (issue #2701).

История (17.09.2026, Vision Pi). Робот простоял ночь и перестал слышать
голос, хотя через Telegram отвечал. `docker logs voice-assistant` (`audio_node`)
показывал: весь вечер `paInputOverflow` (поток живой), 22:29 —
`⚠️ ReSpeaker не принял threshold 3.5 dB`, дальше от захвата НИ ОДНОГО
сообщения. VAD/DoA идут по USB HID отдельным каналом и продолжали
работать — 17.09 каждая фраза уходила в `❌ Речь отклонена: 0.00с` (10 раз
подряд), STT ничего не получал. Контейнер был `Up 20 hours (healthy)`.
`docker restart voice-assistant` вылечило немедленно.

Существующий ретрай (`_schedule_audio_retry`, тест
`test/unit/test_audio_node_stream_retry.py`) чинит только «поток не
открылся на старте» — `open_audio_stream()` возвращается сразу же, если
`self.stream` уже не `None`. Он НЕ видит случай «поток открыт, но
PortAudio молча перестал звать audio_callback».

Этот файл проверяет два независимых признака зависания (issue #2701 §1-2)
и то, что они НЕ путают TTS-мут (issue #993) с зависанием (issue #2701 §3
критерий приёмки):

1. `_check_audio_watchdog` — таймер 1Hz, следит за возрастом последнего
   `audio_callback` с данными (`_last_audio_data_monotonic`); истечение
   `audio_stall_timeout_s` → WARN + переоткрытие потока + счётчик.
2. `check_vad_and_doa` — K подряд `Речь отклонена: 0.00с` при VAD=речь
   (буфер захвата пуст, хотя VAD по HID сработал) → то же переоткрытие;
   счётчик сбрасывается на первой успешной фразе.
3. Пока `audio_callback` продолжает вызываться (как и бывает во время
   TTS-мута — гейтится только VAD, не сам callback), watchdog не должен
   срабатывать ложно.

Стиль стаба — как в test_audio_node_first_capture.py: AudioNode.__init__
выполняется по-настоящему с замоканными rclpy/pyaudio (без реального ROS
или железа), тестируется реальная логика методов.
"""

from __future__ import annotations

import sys
import time
from unittest.mock import MagicMock

import pytest

# ---------------------------------------------------------------------------
# Общий rclpy/pyaudio стаб (тот же подход, что test_audio_node_first_capture.py
# и test_audio_node_echo.py — независимая копия, а не импорт друг у друга,
# т.к. эти файлы истории сознательно не шарят fixtures между собой).
# ---------------------------------------------------------------------------


def _node_no_op(self, *a, **kw):
    return None


def _ensure_rclpy_mock(monkeypatch):
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

    class _DurMod:
        class Duration:
            def __init__(self, seconds=0, nanoseconds=0):
                self.seconds = seconds
                self.nanoseconds = int(seconds * 1e9) + nanoseconds

    monkeypatch.setitem(sys.modules, "rclpy.duration", _DurMod())

    class _NodeMod:
        Node = _NodeBase

    class _Rclpy:
        node = _NodeMod()
        duration = _DurMod()

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
    """Создаёт AudioNode-инстанс без rclpy и без железа (реальный __init__)."""
    from rob_box_voice import audio_node as audio_node_module

    defaults = dict(
        sample_rate=16000,
        channels=1,
        chunk_size=4096,
        vad_threshold=3.5,
        publish_rate=10,
        device_index=-1,
        device_name="ReSpeaker 4 Mic Array",
        audio_retry_period=5.0,
        speech_continuation=3.0,
        speech_prefetch=1.5,
        speech_min_duration=0.3,
        speech_max_duration=10.0,
        tts_grace_s=2.5,
        music_vad_threshold=6.0,
        music_vad_min_db=-35.0,
        barge_in_with_music=True,
        # Issue #2701
        audio_stall_timeout_s=5.0,
        audio_stall_empty_speech_count=3,
        audio_heartbeat_file="",  # выключено по умолчанию в тестах
    )
    defaults.update(param_overrides)

    node = audio_node_module.AudioNode.__new__(audio_node_module.AudioNode)
    _params = dict(defaults)

    def _declare(name, value, *a, **kw):
        _params.setdefault(name, value)

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
    # initialize_hardware() в __init__ не смог найти железо (моки) — стрим
    # не открыт. Тесты watchdog'а сами выставляют node.stream по сценарию.
    node.stream = None
    node.pyaudio_instance = MagicMock()
    node._audio_retry_timer = None
    node._audio_retry_count = 0
    return node


@pytest.fixture
def audio_node():
    return _make_audio_node_stub()


class _FakeTime:
    """Минимальный rclpy-time-подобный объект (только nanoseconds)."""

    def __init__(self, nanoseconds: int = 0):
        self.nanoseconds = nanoseconds

    def __sub__(self, other):
        if hasattr(other, "nanoseconds"):
            return _FakeTime(self.nanoseconds - other.nanoseconds)
        return _FakeTime(self.nanoseconds - int(other.seconds * 1e9))


class _FakeClock:
    def __init__(self, start_ns: int = 0):
        self._ns = start_ns

    def now(self):
        return _FakeTime(self._ns)

    def advance(self, seconds: float):
        self._ns += int(seconds * 1e9)


def _install_fake_clock(node, start_ns: int = 0) -> _FakeClock:
    clock = _FakeClock(start_ns)
    node.get_clock = lambda: clock
    node.speech_stopped_time = _FakeTime(
        clock._ns - int((node.speech_continuation + 1.0) * 1e9)
    )
    return clock


def _fake_stream():
    stream = MagicMock()
    stream.stop_stream = MagicMock()
    stream.close = MagicMock()
    return stream


SILENCE_CHUNK = b"\x00\x00" * 4096


def _speech_chunk(tag: int) -> bytes:
    sample = (tag << 8) | 0x01
    return sample.to_bytes(2, "little") * 4096


# ---------------------------------------------------------------------------
# 1) Watchdog: callback перестал вызываться → переоткрытие + счётчик + лог.
# ---------------------------------------------------------------------------


class TestAudioCallbackWatchdog:
    def test_last_audio_data_timestamp_updates_on_callback(self, audio_node, monkeypatch):
        """audio_callback с данными обновляет _last_audio_data_monotonic."""
        t = [100.0]
        monkeypatch.setattr(time, "monotonic", lambda: t[0])
        audio_node.is_running = True
        assert audio_node._last_audio_data_monotonic is None

        audio_node.audio_callback(SILENCE_CHUNK, 4096, {}, 0)
        assert audio_node._last_audio_data_monotonic == 100.0

        t[0] = 105.3
        audio_node.audio_callback(SILENCE_CHUNK, 4096, {}, 0)
        assert audio_node._last_audio_data_monotonic == 105.3

    def test_watchdog_skips_when_stream_not_open(self, audio_node):
        """Пока поток не открыт (self.stream is None) — не наше дело,
        этим занимается обычный _schedule_audio_retry."""
        audio_node.stream = None
        audio_node._reopen_audio_stream = MagicMock()

        audio_node._check_audio_watchdog()

        audio_node._reopen_audio_stream.assert_not_called()

    def test_watchdog_skips_before_first_callback(self, audio_node):
        """Поток открыт, но callback ещё ни разу не пришёл — рано судить."""
        audio_node.stream = _fake_stream()
        audio_node._last_audio_data_monotonic = None
        audio_node._reopen_audio_stream = MagicMock()

        audio_node._check_audio_watchdog()

        audio_node._reopen_audio_stream.assert_not_called()

    def test_watchdog_does_not_trigger_before_timeout(self, audio_node, monkeypatch):
        """Возраст последнего callback < audio_stall_timeout_s — тихо."""
        t = [1000.0]
        monkeypatch.setattr(time, "monotonic", lambda: t[0])
        audio_node.audio_stall_timeout_s = 5.0
        audio_node.stream = _fake_stream()
        audio_node._last_audio_data_monotonic = 1000.0
        audio_node._reopen_audio_stream = MagicMock()

        t[0] = 1004.9  # 4.9s < 5.0s таймаут
        audio_node._check_audio_watchdog()

        audio_node._reopen_audio_stream.assert_not_called()

    def test_watchdog_reopens_after_timeout(self, audio_node, monkeypatch):
        """Главный сценарий issue #2701: callback перестал вызываться →
        через N секунд поток переоткрыт."""
        t = [2000.0]
        monkeypatch.setattr(time, "monotonic", lambda: t[0])
        audio_node.audio_stall_timeout_s = 5.0
        stream = _fake_stream()
        audio_node.stream = stream
        audio_node._last_audio_data_monotonic = 2000.0
        audio_node._schedule_audio_retry = MagicMock()

        t[0] = 2005.1  # 5.1s > 5.0s таймаут — захват "завис"
        audio_node._check_audio_watchdog()

        # Зависший поток закрыт...
        stream.stop_stream.assert_called_once()
        stream.close.assert_called_once()
        assert audio_node.stream is None
        # ...и запланирован ретрай через существующий механизм.
        audio_node._schedule_audio_retry.assert_called_once()
        # Есть счётчик переоткрытий (для лога/наблюдаемости).
        assert audio_node._audio_stall_reopen_count == 1

    def test_watchdog_reopen_count_increments_across_stalls(self, audio_node, monkeypatch):
        """Счётчик переоткрытий растёт при повторных зависаниях."""
        t = [0.0]
        monkeypatch.setattr(time, "monotonic", lambda: t[0])
        audio_node.audio_stall_timeout_s = 5.0
        audio_node._schedule_audio_retry = MagicMock()

        for i in range(3):
            audio_node.stream = _fake_stream()
            audio_node._last_audio_data_monotonic = t[0]
            t[0] += 6.0
            audio_node._check_audio_watchdog()

        assert audio_node._audio_stall_reopen_count == 3
        assert audio_node._schedule_audio_retry.call_count == 3

    def test_watchdog_reopen_logs_warning(self, audio_node, monkeypatch):
        """Требование issue: «есть лог» — WARN должен реально уйти в logger."""
        t = [50.0]
        monkeypatch.setattr(time, "monotonic", lambda: t[0])
        audio_node.audio_stall_timeout_s = 5.0
        audio_node.stream = _fake_stream()
        audio_node._last_audio_data_monotonic = 50.0
        audio_node._schedule_audio_retry = MagicMock()
        warnings = []
        audio_node.get_logger = lambda: MagicMock(
            info=lambda *a, **kw: None,
            warning=lambda msg, *a, **kw: warnings.append(msg),
            warn=lambda *a, **kw: None,
            error=lambda *a, **kw: None,
            debug=lambda *a, **kw: None,
        )

        t[0] = 60.0  # 10s > 5s таймаут
        audio_node._check_audio_watchdog()

        assert any("завис" in w or "issue 2701" in w for w in warnings), warnings

    def test_reopen_resets_device_index_for_reenumeration(self, audio_node):
        """Индекс сбрасывается к параметру — ReSpeaker мог переехать при
        повторном перечислении PortAudio (та же логика, что в
        open_audio_stream на ветке "Ошибка открытия")."""
        audio_node._device_index_param = -1
        audio_node.device_index = 7  # "устаревший" индекс с прошлого open()
        audio_node.stream = _fake_stream()
        audio_node._schedule_audio_retry = MagicMock()

        audio_node._reopen_audio_stream()

        assert audio_node.device_index == -1


# ---------------------------------------------------------------------------
# 2) K подряд "Речь отклонена: 0.00с" при VAD=речь → переоткрытие.
# ---------------------------------------------------------------------------


class TestEmptySpeechStreakDetector:
    def _speech_cycle_with_empty_buffer(self, audio_node, clock):
        """Симулирует один цикл check_vad_and_doa: VAD включился и
        выключился, но audio_callback ни разу не наполнил буфер (буфер
        пуст → длительность 0.00с) — ровно как в логах issue #2701
        (17.09, 10 раз подряд)."""
        audio_node.respeaker.is_connected = MagicMock(return_value=True)
        audio_node.respeaker.get_direction = MagicMock(return_value=None)

        # VAD=True — is_speeching включается.
        audio_node.respeaker.get_vad = MagicMock(return_value=True)
        audio_node.check_vad_and_doa()
        assert audio_node.is_speeching is True
        assert audio_node.speech_audio_buffer == b""  # callback не наполнял

        # Время идёт, VAD=False, speech_continuation истекает.
        clock.advance(audio_node.speech_continuation + 0.1)
        audio_node.respeaker.get_vad = MagicMock(return_value=False)
        audio_node.check_vad_and_doa()
        assert audio_node.is_speeching is False

    def test_single_empty_speech_does_not_reopen(self, audio_node):
        """Один случай «0.00с» — это штатный шум VAD, не повод дёргать поток."""
        clock = _install_fake_clock(audio_node, start_ns=10 * 10**9)
        audio_node._tts_ended_at = 0.0
        audio_node.music_active = False
        audio_node.audio_stall_empty_speech_count = 3
        audio_node._reopen_audio_stream = MagicMock()

        self._speech_cycle_with_empty_buffer(audio_node, clock)

        assert audio_node._empty_speech_streak == 1
        audio_node._reopen_audio_stream.assert_not_called()

    def test_k_consecutive_empty_speech_reopens_stream(self, audio_node):
        """Главный сценарий issue #2701 п.2: K=3 «0.00с» подряд →
        переоткрытие потока."""
        clock = _install_fake_clock(audio_node, start_ns=10 * 10**9)
        audio_node._tts_ended_at = 0.0
        audio_node.music_active = False
        audio_node.audio_stall_empty_speech_count = 3
        audio_node._reopen_audio_stream = MagicMock()

        for _ in range(3):
            self._speech_cycle_with_empty_buffer(audio_node, clock)

        assert audio_node._empty_speech_streak == 3
        audio_node._reopen_audio_stream.assert_called_once()

    def test_streak_resets_on_first_successful_phrase(self, audio_node):
        """Счётчик сбрасывается на первой успешно принятой фразе — ровно
        как требует issue #2701."""
        clock = _install_fake_clock(audio_node, start_ns=10 * 10**9)
        audio_node._tts_ended_at = 0.0
        audio_node.music_active = False
        audio_node.audio_stall_empty_speech_count = 3
        audio_node._reopen_audio_stream = MagicMock()
        audio_node.respeaker.is_connected = MagicMock(return_value=True)
        audio_node.respeaker.get_direction = MagicMock(return_value=None)

        # Два «пустых» случая подряд (ещё не дотянули до K=3).
        self._speech_cycle_with_empty_buffer(audio_node, clock)
        self._speech_cycle_with_empty_buffer(audio_node, clock)
        assert audio_node._empty_speech_streak == 2

        # Успешная фраза: VAD включился, callback реально наполнил буфер.
        # Два чанка (2 * 256ms = 512ms) — выше speech_min_duration (0.3s
        # по умолчанию), иначе фраза сама отклонится как "слишком короткая"
        # и не попадёт в ветку успеха, которую тестируем.
        audio_node.respeaker.get_vad = MagicMock(return_value=True)
        audio_node.check_vad_and_doa()
        assert audio_node.is_speeching is True
        audio_node.audio_callback(_speech_chunk(0x55), 4096, {}, 0)
        audio_node.audio_callback(_speech_chunk(0x56), 4096, {}, 0)
        assert len(audio_node.speech_audio_buffer) > 0

        clock.advance(audio_node.speech_continuation + 0.1)
        audio_node.respeaker.get_vad = MagicMock(return_value=False)
        audio_node.check_vad_and_doa()

        assert audio_node._empty_speech_streak == 0
        audio_node._reopen_audio_stream.assert_not_called()

        # Дальше снова 3 пустых подряд — должны отработать с нуля.
        for _ in range(3):
            self._speech_cycle_with_empty_buffer(audio_node, clock)
        audio_node._reopen_audio_stream.assert_called_once()

    def test_non_empty_out_of_range_rejection_does_not_touch_streak(self, audio_node):
        """Отказ по длительности (не 0с, а, скажем, слишком долгая фраза)
        — штатное поведение VAD, не должен путаться со «стримом завис»."""
        clock = _install_fake_clock(audio_node, start_ns=10 * 10**9)
        audio_node._tts_ended_at = 0.0
        audio_node.music_active = False
        audio_node.speech_max_duration = 0.1  # искусственно узкое окно
        audio_node.audio_stall_empty_speech_count = 3
        audio_node._reopen_audio_stream = MagicMock()
        audio_node.respeaker.is_connected = MagicMock(return_value=True)
        audio_node.respeaker.get_direction = MagicMock(return_value=None)

        audio_node.respeaker.get_vad = MagicMock(return_value=True)
        audio_node.check_vad_and_doa()
        # Реальные данные пришли — буфер НЕ пуст, но длиннее max_duration.
        audio_node.audio_callback(_speech_chunk(0x33), 4096, {}, 0)
        audio_node.audio_callback(_speech_chunk(0x34), 4096, {}, 0)

        clock.advance(audio_node.speech_continuation + 0.1)
        audio_node.respeaker.get_vad = MagicMock(return_value=False)
        audio_node.check_vad_and_doa()

        assert audio_node._empty_speech_streak == 0
        audio_node._reopen_audio_stream.assert_not_called()


# ---------------------------------------------------------------------------
# 3) TTS-мут (issue #993) не должен давать ложных срабатываний watchdog'а.
# ---------------------------------------------------------------------------


class TestNoFalsePositiveDuringTtsMute:
    def test_callback_keeps_firing_during_tts_mute(self, audio_node, monkeypatch):
        """Пока TTS активен (VAD программно замьючен через _vad_gated),
        audio_callback продолжает вызываться штатно — watchdog смотрит на
        сам факт вызова, а не на «была ли речь», поэтому не должен видеть
        зависание."""
        t = [500.0]
        monkeypatch.setattr(time, "monotonic", lambda: t[0])
        audio_node.audio_stall_timeout_s = 5.0
        audio_node.is_running = True
        audio_node.stream = _fake_stream()
        audio_node._reopen_audio_stream = MagicMock()

        # TTS начал говорить — issue #993/#989 гейтит VAD, но НЕ callback.
        playing = MagicMock()
        playing.data = "playing"
        audio_node._on_tts_state(playing)
        assert audio_node.tts_active is True

        # Callback продолжает вызываться каждые 256ms в течение TTS,
        # watchdog тикает каждую секунду — ни разу не должен сработать.
        for _ in range(40):  # ~10s "разговора" TTS
            audio_node.audio_callback(_speech_chunk(0x11), 4096, {}, 0)
            t[0] += 0.25
            audio_node._check_audio_watchdog()

        audio_node._reopen_audio_stream.assert_not_called()

    def test_vad_gated_true_during_tts_active_is_not_stall_signal(self, audio_node):
        """_vad_gated не подавляет VAD=True во время TTS (barge-in, issue
        #993) — эффективный VAD может пройти. Это не имеет отношения к
        watchdog'у (тот следит только за callback), но регрессия здесь
        сломала бы предпосылку "callback снаружи TTS не гейтится"."""
        audio_node.tts_active = True
        audio_node._tts_ended_at = 0.0
        audio_node.music_active = False

        assert audio_node._vad_gated(True) is True

    def test_empty_speech_streak_not_falsely_incremented_by_tts_grace(self, audio_node):
        """В grace-периоде после TTS (issue #989) VAD программно False —
        is_speeching никогда не включается, поэтому «пустых» веток в
        check_vad_and_doa просто не бывает: streak не растёт впустую."""
        clock = _install_fake_clock(audio_node, start_ns=10 * 10**9)
        audio_node.respeaker.is_connected = MagicMock(return_value=True)
        audio_node.respeaker.get_direction = MagicMock(return_value=None)
        audio_node.music_active = False
        audio_node.tts_active = False
        audio_node._tts_ended_at = time.monotonic()  # только что закончился TTS
        audio_node.audio_stall_empty_speech_count = 3
        audio_node._reopen_audio_stream = MagicMock()

        # Аппаратный VAD дребезжит True/False во время grace — не должно
        # попасть в is_speeching вообще (_vad_gated возвращает False).
        for vad_hw in (True, False, True):
            audio_node.respeaker.get_vad = MagicMock(return_value=vad_hw)
            audio_node.check_vad_and_doa()
            assert audio_node.is_speeching is False

        assert audio_node._empty_speech_streak == 0
        audio_node._reopen_audio_stream.assert_not_called()


class TestHeartbeatFile:
    """Issue #2701 п.3: heartbeat-файл для healthcheck контейнера."""

    def test_healthy_watchdog_tick_touches_heartbeat_file(self, audio_node, tmp_path, monkeypatch):
        heartbeat = tmp_path / "audio_heartbeat"
        audio_node.audio_heartbeat_file = str(heartbeat)
        audio_node.audio_stall_timeout_s = 5.0
        audio_node.stream = _fake_stream()

        t = [1000.0]
        monkeypatch.setattr(time, "monotonic", lambda: t[0])
        monkeypatch.setattr(time, "time", lambda: 1234567890.0)
        audio_node._last_audio_data_monotonic = 1000.0

        audio_node._check_audio_watchdog()

        assert heartbeat.exists()
        assert heartbeat.read_text(encoding="utf-8") == "1234567890.0"

    def test_stalled_watchdog_does_not_touch_heartbeat(self, audio_node, tmp_path, monkeypatch):
        """Пока поток завис, heartbeat не обновляется — healthcheck должен
        это увидеть (файл стареет)."""
        heartbeat = tmp_path / "audio_heartbeat"
        heartbeat.write_text("0", encoding="utf-8")
        audio_node.audio_heartbeat_file = str(heartbeat)
        audio_node.audio_stall_timeout_s = 5.0
        audio_node.stream = _fake_stream()
        audio_node._schedule_audio_retry = MagicMock()

        t = [2000.0]
        monkeypatch.setattr(time, "monotonic", lambda: t[0])
        audio_node._last_audio_data_monotonic = 2000.0
        t[0] = 2010.0  # завис

        audio_node._check_audio_watchdog()

        assert heartbeat.read_text(encoding="utf-8") == "0"

    def test_empty_heartbeat_path_disables_write(self, audio_node, monkeypatch):
        """Пустая строка (дефолт в тестах) — heartbeat выключен, никаких
        файловых операций, никаких исключений."""
        audio_node.audio_heartbeat_file = ""
        audio_node.audio_stall_timeout_s = 5.0
        audio_node.stream = _fake_stream()
        t = [10.0]
        monkeypatch.setattr(time, "monotonic", lambda: t[0])
        audio_node._last_audio_data_monotonic = 10.0

        # Не должно бросить исключение.
        audio_node._check_audio_watchdog()


if __name__ == "__main__":
    import pytest as _pytest

    _pytest.main([__file__, "-v"])
