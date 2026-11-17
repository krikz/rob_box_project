"""Сегментатор ``audio_node`` и ``/audio/vad`` (#4155): какой сигнал начинает фразу, которую потом получает STT.

Для приглушения музыки под голос человека (``mcp_server`` → ``SpeechDuck.on_vad``) нужен сигнал «человек заговорил»,
который стоит у начала фразы. Замер на роботе 28.10 (комментарий к #4155) сложился в «``/audio/vad`` почти не поднимается
и на настоящих фразах». Здесь — поведение самого сегментатора (``check_vad_and_doa`` + ``audio_callback``) на сценарии
«VAD открылся на ``burst`` секунд»: каждая фраза, ушедшая на ``/audio/speech_audio`` (её читает STT), открыта подъёмом
``/audio/vad`` ровно за ``burst - 0.1 + speech_continuation`` до публикации. Другого сигнала начала фразы у
сегментатора нет: ``/audio/speech_detected`` объявлен в docstring ``audio_node``, но не публикуется (ADR-0137 F6).
"""

from __future__ import annotations

import importlib
import sys
import types
from unittest.mock import MagicMock

import pytest

TICK_S = 0.1  # период опроса VAD (publish_rate 10 Гц)
RATE = 16000
CHUNK = bytes(int(TICK_S * RATE) * 2)  # 0,1 с тишины, 16 бит моно
CONTINUATION_S, PREFETCH_S = (
    1.5,
    1.5,
)  # дефолты parameter speech_continuation / speech_prefetch


class _Time:
    def __init__(self, seconds: float) -> None:
        self.seconds = seconds

    def __sub__(self, other: "_Time") -> types.SimpleNamespace:
        return types.SimpleNamespace(
            nanoseconds=int(round((self.seconds - other.seconds) * 1e9))
        )


class _Clock:
    def __init__(self) -> None:
        self.t = 0.0

    def now(self) -> _Time:
        return _Time(self.t)


class _Node:
    def __init__(self, *a, **kw):
        pass


@pytest.fixture
def audio_module(monkeypatch):
    """``rob_box_voice.audio_node`` поверх заглушек rclpy/std_msgs/PyAudio; после теста модуль выгружается."""
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
        "rob_box_voice.utils.audio_utils": MagicMock(),
        "rob_box_voice.utils.respeaker_interface": MagicMock(),
    }
    for name, module in stubs.items():
        monkeypatch.setitem(sys.modules, name, module)
    import rob_box_voice

    def forget():
        sys.modules.pop("rob_box_voice.audio_node", None)
        if hasattr(rob_box_voice, "audio_node"):
            delattr(rob_box_voice, "audio_node")

    forget()
    yield importlib.import_module("rob_box_voice.audio_node")
    forget()


def _segmenter(module, clock, events):
    """``AudioNode`` без ROS и железа: настоящие ``audio_callback`` и ``check_vad_and_doa``, VAD читается из ``vad``."""
    node = module.AudioNode.__new__(module.AudioNode)
    node.logger = MagicMock()
    node.get_logger = lambda: node.logger
    node.get_clock = lambda: clock
    node.respeaker = MagicMock()
    node.respeaker.is_connected.return_value = True
    node.sample_rate, node.channels, node.mix_channels, node.is_running = (
        RATE,
        1,
        [],
        True,
    )
    node.speech_continuation, node.speech_prefetch = CONTINUATION_S, PREFETCH_S
    node.speech_min_duration, node.speech_max_duration = 0.3, 15.0
    node.speech_prefetch_bytes = int(PREFETCH_S * RATE * 2)
    node.speech_audio_buffer = node.speech_prefetch_buffer = b""
    node.is_speeching = node.prev_vad = False
    node.speech_stopped_time = _Time(-10.0)
    node.music_active = node.barge_in_with_music = (
        True  # музыка играет, AEC пропускает VAD без RMS-гейта (#1764)
    )
    node.tts_active, node._current_db = False, -20.0
    node._empty_speech_streak, node.audio_stall_empty_speech_count = 0, 3
    node._last_audio_data_monotonic = None
    node._read_and_publish_doa = lambda: None
    node.audio_pub, node.speech_audio_pub, node.vad_pub = (
        MagicMock(),
        MagicMock(),
        MagicMock(),
    )
    node.vad_pub.publish.side_effect = lambda m: events.append(
        (clock.t, "vad", bool(m.data))
    )
    node.speech_audio_pub.publish.side_effect = lambda m: events.append(
        (clock.t, "phrase", len(m.data) / 2 / RATE)
    )
    return node


def _run(node, clock, vad_ticks: set, seconds: float) -> None:
    """Каждые ``TICK_S``: ``audio_callback`` (0,1 с звука) и опрос VAD; ``vad_ticks`` — номера тиков с поднятым VAD."""
    for step in range(int(round(seconds / TICK_S)) + 1):
        clock.t = round(step * TICK_S, 6)
        node.respeaker.get_vad.return_value = 1 if step in vad_ticks else 0
        node.audio_callback(CHUNK, len(CHUNK) // 2, None, 0)
        node.check_vad_and_doa()


@pytest.mark.parametrize("burst_s", [0.1, 0.4, 1.2])
def test_every_phrase_for_stt_is_opened_by_a_vad_rise_and_a_short_burst_makes_a_three_second_phrase(
    audio_module, burst_s
):
    """Всплеск VAD в 0,1 с (как под вокалом кавера: 0,09–0,40 с) — фраза ~3 с: pre-roll 1,5 с + хвост 1,5 с."""
    clock, events = _Clock(), []
    node = _segmenter(audio_module, clock, events)
    first = 20  # тик 2,0 с
    ticks = set(range(first, first + int(round(burst_s / TICK_S))))
    _run(node, clock, ticks, 8.0)

    kinds = [(kind, value) for _, kind, value in events if kind == "vad"]
    assert kinds == [
        ("vad", True),
        ("vad", False),
    ], "фронты /audio/vad: подъём и спад, больше ничего"
    rise_t, fall_t = events[0][0], events[1][0]
    phrases = [(t, seconds) for t, kind, seconds in events if kind == "phrase"]
    assert len(phrases) == 1, f"одна фраза на один всплеск: {events}"
    phrase_t, phrase_s = phrases[0]
    assert rise_t < fall_t < phrase_t, "подъём VAD раньше фразы"
    last_high_t = (first + len(ticks) - 1) * TICK_S
    assert (rise_t, fall_t) == pytest.approx((first * TICK_S, last_high_t + TICK_S))
    assert phrase_t == pytest.approx(
        last_high_t + CONTINUATION_S
    )  # прогон 28.10: 0,1 с → подъём 2,0 с, фраза 3,5 с
    assert phrase_s == pytest.approx(
        PREFETCH_S + (phrase_t - rise_t)
    )  # 0,1 с → 3,0 с; замер: речь=3,06–3,57 с
    node.logger.warn.assert_not_called()


def test_without_a_vad_rise_nothing_goes_to_stt(audio_module):
    clock, events = _Clock(), []
    node = _segmenter(audio_module, clock, events)
    _run(node, clock, set(), 6.0)
    assert events == []
