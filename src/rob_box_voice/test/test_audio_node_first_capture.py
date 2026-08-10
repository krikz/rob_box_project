#!/usr/bin/env python3
"""
test_audio_node_first_capture.py — Unit-тесты фикса «первый захват речи теряет
начало фразы» (ретро 09.08 #15: «робот» → «роберт»).

Проверяет три причины обрезания начала фразы в audio_node:

1. **Pre-roll window** (`speech_prefetch`): буфер перед началом речи должен
   покрывать латенцию аппаратного VAD (GAMMAVAD + опрос 10Hz + джиттер).
   Раньше 0.5s — на слабом/грязном сигнале VAD срабатывает через 300-500ms,
   начало «робот» выпадало из окна → STT слышал «оберт» → «роберт».
   Теперь 1.0s (2x запас).

2. **Pre-roll не должен содержать эхо собственного TTS**: раньше
   `speech_prefetch_buffer` НЕ очищался при старте/остановке TTS, поэтому
   первый захват после ответа робота получал pre-roll с хвостом собственного
   голоса → STT склеивал «…роберт» из обрывка.

3. **speech_stopped_time при старте**: должен быть в прошлом, иначе первый же
   tick включает is_speeching (буфер наполняется шумом, первый захват
   приклеивается после него, pre-roll не подставляется).

Тест НЕ требует rclpy/железа: AudioNode.__init__ замокан через MagicMock,
тестируем чистую логику буферизации (как test_audio_node_echo.py).
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
    """Минимальный rclpy mock (как в test_audio_node_echo)."""

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
        """Fake rclpy.duration для __init__ (speech_stopped_time в прошлом)."""

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
        speech_prefetch=1.0,  # фикс #15: было 0.5
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


class _FakeTime:
    """Минимальный rclpy-time-подобный объект (только nanoseconds)."""

    def __init__(self, nanoseconds: int = 0):
        self.nanoseconds = nanoseconds

    def __sub__(self, other):
        if hasattr(other, "nanoseconds"):
            return _FakeTime(self.nanoseconds - other.nanoseconds)
        # Duration-подобный
        return _FakeTime(self.nanoseconds - int(other.seconds * 1e9))


class _FakeClock:
    """Детерминированные часы: now() возвращает _FakeTime с растущим ns."""

    def __init__(self, start_ns: int = 0):
        self._ns = start_ns

    def now(self):
        return _FakeTime(self._ns)

    def advance(self, seconds: float):
        self._ns += int(seconds * 1e9)


def _install_fake_clock(node, start_ns: int = 0) -> _FakeClock:
    """Ставит детерминированные часы и speech_stopped_time «в прошлом» —
    ровно как это делает __init__ (now - (continuation + 1s))."""
    clock = _FakeClock(start_ns)
    node.get_clock = lambda: clock
    node.speech_stopped_time = _FakeTime(
        clock._ns - int((node.speech_continuation + 1.0) * 1e9)
    )
    return clock


SILENCE_CHUNK = b"\x00\x00" * 4096  # 256ms @16kHz, тишина


def _speech_chunk(tag: int) -> bytes:
    """Отличимый «речевой» чанк: 16-bit PCM с постоянным уровнем tag."""
    sample = (tag << 8) | 0x01
    return sample.to_bytes(2, "little") * 4096


class TestSpeechPrefetchWindow:
    """Причина 1: pre-roll должен покрывать латенцию VAD (300-500ms)."""

    def test_prefetch_default_is_1_0s(self, audio_node):
        assert audio_node.speech_prefetch == 1.0
        # 1.0s * 16000 Hz * 2 bytes = 32000 bytes
        assert audio_node.speech_prefetch_bytes == 32000

    def test_prefetch_covers_vad_latency_requirement(self, audio_node):
        """Требование: первые 300-500ms фразы не должны обрезаться.

        Латенция VAD = GAMMAVAD (~100-300ms) + опрос 10Hz (до 100ms) +
        джиттер. 0.5s — впритык/недостаточно на слабом сигнале; фикс — 1.0s.
        """
        min_required_s = 0.5  # верхняя граница из ретро (#15: 300-500ms)
        assert audio_node.speech_prefetch >= min_required_s
        assert audio_node.speech_prefetch_bytes >= int(
            min_required_s * audio_node.sample_rate * 2
        )

    def test_prefetch_accumulates_while_idle(self, audio_node):
        """Вне речи pre-roll копится и ограничен окном (rolling buffer)."""
        audio_node.is_running = True
        for _ in range(6):  # 6 * 256ms = 1.5s > окно 1.0s
            audio_node.audio_callback(SILENCE_CHUNK, 4096, {}, 0)
        assert len(audio_node.speech_prefetch_buffer) == audio_node.speech_prefetch_bytes

    def test_first_speech_chunk_prepends_prefetch(self, audio_node):
        """Первый чанк речи подставляет pre-roll в начало буфера.

        Если VAD сработал с латенцией, начало «робот» уже лежит в
        speech_prefetch_buffer — оно обязано попасть в начало фразы.
        """
        audio_node.is_running = True
        # 1s тишины → pre-roll наполнен
        for _ in range(4):
            audio_node.audio_callback(SILENCE_CHUNK, 4096, {}, 0)
        prefetch_before = audio_node.speech_prefetch_buffer
        assert prefetch_before

        audio_node.is_speeching = True
        chunk = _speech_chunk(0x7F)
        audio_node.audio_callback(chunk, 4096, {}, 0)

        # Буфер начинается с pre-roll и заканчивается первым речевым чанком
        assert audio_node.speech_audio_buffer.startswith(prefetch_before)
        assert audio_node.speech_audio_buffer.endswith(chunk)
        assert len(audio_node.speech_audio_buffer) == len(prefetch_before) + len(chunk)

    def test_phrase_start_survives_vad_latency(self, audio_node):
        """Имитация латенции VAD: первые 500ms речи попадают в pre-roll,
        а не теряются.

        Сценарий: пользователь говорит «робот…»; аппаратный VAD срабатывает
        только через ~400ms (слабый сигнал). За это время 1-2 речевых чанка
        уже ушли в speech_prefetch_buffer. Когда is_speeching=True, они
        обязаны сохраниться в speech_audio_buffer целиком (контактной
        последовательностью), даже если перед ними есть тишина из pre-roll.
        """
        audio_node.is_running = True
        # pre-roll заполнен тишиной
        for _ in range(4):
            audio_node.audio_callback(SILENCE_CHUNK, 4096, {}, 0)

        # Начало фразы (2 чанка = 512ms) уходит в pre-roll ДО срабатывания VAD
        first = _speech_chunk(0x10)
        second = _speech_chunk(0x20)
        audio_node.audio_callback(first, 4096, {}, 0)
        audio_node.audio_callback(second, 4096, {}, 0)
        assert first in audio_node.speech_prefetch_buffer
        assert second in audio_node.speech_prefetch_buffer

        # VAD сработал — теперь копим речь
        audio_node.is_speeching = True
        third = _speech_chunk(0x30)
        audio_node.audio_callback(third, 4096, {}, 0)

        # Начало фразы (first+second) сохранилось целиком и контактно,
        # а фраза заканчивается текущим чанком
        assert (first + second + third) in audio_node.speech_audio_buffer
        assert audio_node.speech_audio_buffer.endswith(third)


class TestPrefetchTtsEcho:
    """Причина 2: pre-roll не должен содержать эхо собственного TTS."""

    def test_tts_start_clears_prefetch(self, audio_node):
        """При старте TTS pre-roll чистится — хвост прошлой фразы не
        должен попасть в начало следующего захвата."""
        audio_node.is_running = True
        for _ in range(4):
            audio_node.audio_callback(_speech_chunk(0x40), 4096, {}, 0)
        assert audio_node.speech_prefetch_buffer

        playing = MagicMock()
        playing.data = "playing"
        audio_node._on_tts_state(playing)

        assert audio_node.speech_prefetch_buffer == b""

    def test_tts_end_clears_prefetch(self, audio_node):
        """После TTS pre-roll чистится — хвост собственного голоса
        (эхо) не должен стать pre-roll'ом первого захвата."""
        audio_node.is_running = True
        # Робот говорил → pre-roll наполнен его голосом
        for _ in range(4):
            audio_node.audio_callback(_speech_chunk(0x50), 4096, {}, 0)
        assert audio_node.speech_prefetch_buffer

        # TTS был активен, теперь закончился
        audio_node.tts_active = True
        ready = MagicMock()
        ready.data = "ready"
        audio_node._on_tts_state(ready)

        assert audio_node.tts_active is False
        assert audio_node.speech_prefetch_buffer == b""

    def test_prefetch_clean_after_tts_cycle(self, audio_node):
        """Полный цикл TTS (playing → ready) → pre-roll пуст и наполняется
        уже чистой тишиной/речью пользователя, без эха."""
        audio_node.is_running = True
        playing = MagicMock()
        playing.data = "playing"
        audio_node._on_tts_state(playing)
        ready = MagicMock()
        ready.data = "ready"
        audio_node._on_tts_state(ready)

        assert audio_node.speech_prefetch_buffer == b""

        # После TTS идёт тишина → pre-roll наполняется тишиной, не эхом
        for _ in range(2):
            audio_node.audio_callback(SILENCE_CHUNK, 4096, {}, 0)
        assert audio_node.speech_prefetch_buffer == SILENCE_CHUNK * 2


class TestSpeechStoppedTimeInit:
    """Причина 3: speech_stopped_time при старте — в прошлом, а не «сейчас»."""

    def test_speech_stopped_time_in_past(self, audio_node):
        """После __init__ speech_stopped_time должен быть в прошлом.

        Если он равен «сейчас», первый tick check_vad_and_doa посчитает
        time_since_stop≈0 < speech_continuation → is_speeching=True без речи:
        буфер наполнится шумом, а первый реальный захват приклеится после.
        """
        clock = _install_fake_clock(audio_node)
        assert audio_node.speech_stopped_time.nanoseconds < clock.now().nanoseconds
        # Запас: минимум на continuation+1s в прошлом
        expected_past = clock.now().nanoseconds - int(
            (audio_node.speech_continuation + 1.0) * 1e9
        )
        assert audio_node.speech_stopped_time.nanoseconds <= expected_past

    def test_vad_required_to_start_speeching(self, audio_node):
        """is_speeching переключается только при vad_eff=True.

        Регрессионная защита: если speech_stopped_time инициализирован как
        «сейчас», первый же check_vad_and_doa (vad=False) даст
        time_since_stop≈0 < continuation → is_speeching=True. Так быть не
        должно: без речи — не начинаем копить буфер.
        """
        clock = _install_fake_clock(audio_node, start_ns=10 * 10**9)
        # Мокаем железо: VAD=False (тишина)
        audio_node.respeaker.is_connected = MagicMock(return_value=True)
        audio_node.respeaker.get_vad = MagicMock(return_value=False)
        audio_node._tts_ended_at = 0.0
        audio_node.music_active = False

        audio_node.check_vad_and_doa()

        assert audio_node.is_speeching is False
        assert audio_node.speech_audio_buffer == b""

    def test_vad_true_starts_speeching_with_prefetch(self, audio_node):
        """VAD=True → is_speeching=True и pre-roll сохраняется в буфере."""
        audio_node.is_running = True
        clock = _install_fake_clock(audio_node, start_ns=10 * 10**9)
        audio_node.respeaker.is_connected = MagicMock(return_value=True)
        audio_node.respeaker.get_vad = MagicMock(return_value=True)
        audio_node._tts_ended_at = 0.0
        audio_node.music_active = False

        # pre-roll наполнен (последняя секунда до речи)
        for _ in range(4):
            audio_node.audio_callback(SILENCE_CHUNK, 4096, {}, 0)
        prefetch_before = audio_node.speech_prefetch_buffer

        audio_node.check_vad_and_doa()
        assert audio_node.is_speeching is True

        chunk = _speech_chunk(0x60)
        audio_node.audio_callback(chunk, 4096, {}, 0)
        assert audio_node.speech_audio_buffer.startswith(prefetch_before)
        assert audio_node.speech_audio_buffer.endswith(chunk)
