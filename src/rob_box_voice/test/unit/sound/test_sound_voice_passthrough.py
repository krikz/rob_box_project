"""test_sound_voice_passthrough.py — unit-тесты голосового passthrough (P1, рация).

Товарищ Шифу: рация = /avatar/voice_in → стриминговое воспроизведение
голоса оператора через ReSpeaker без обработки. Контракты:

1. Подписка на /avatar/voice_in (AudioData, best-effort + volatile QoS).
2. Стрим: первый чанк открывает OutputStream (16 kHz, stereo int16),
   последующие пишутся в него; тишина > VOICE_SILENCE_TIMEOUT закрывает
   стрим; пустой чанк не падает.
3. Координация с эффектами: триггер эффекта не рвёт активный стрим;
   STOP (/voice/sound/stop) закрывает стрим.
"""

import numpy as np
import pytest
from unittest.mock import MagicMock

import rob_box_voice.sound_node as sn_module
from rob_box_voice.sound_node import SoundNode


@pytest.fixture
def node(monkeypatch):
    """Свежий SoundNode с изолированным sd (см. conftest.py).

    module-level MagicMock ``sounddevice`` кэширует return value
    ``OutputStream(...)`` между тестами, из-за чего call_count накапливается.
    Здесь на каждый тест подставляем свежий ``sd``, чтобы счётчики вызовов
    были локальными.
    """
    fresh_sd = MagicMock()
    monkeypatch.setattr(sn_module, "sd", fresh_sd)
    return SoundNode()


def _audio_msg(data: bytes):
    from audio_common_msgs.msg import AudioData

    msg = AudioData()
    msg.data = data
    return msg


def _string_msg(data: str):
    from std_msgs.msg import String

    msg = String()
    msg.data = data
    return msg


def _find_subscription(node, topic):
    for args, _kwargs in node._subscriptions:
        if args[1] == topic:
            return args
    return None


def test_subscribes_to_avatar_voice_in_best_effort(node):
    """SoundNode подписан на /avatar/voice_in (AudioData, best-effort, volatile)."""
    args = _find_subscription(node, "/avatar/voice_in")
    assert args is not None, "нет подписки на /avatar/voice_in"

    msg_type, topic, callback, qos = args
    assert topic == "/avatar/voice_in"
    assert msg_type.__name__ == "AudioData"
    assert callable(callback)
    assert qos.reliability == "best_effort"
    assert qos.durability == "volatile"
    assert qos.depth == 10


# План P1 Task 1.2: тишина дольше 300 мс закрывает голосовой стрим.
SILENCE_TIMEOUT = 0.3


def test_first_chunk_opens_stream(node):
    """Первый чанк открывает OutputStream (16 kHz, stereo int16)."""
    node.voice_in_callback(_audio_msg(np.zeros(320, dtype=np.int16).tobytes()))

    assert node._voice_stream is not None, "первый чанк должен открыть stream"
    sn_module.sd.OutputStream.assert_called_once_with(samplerate=16000, channels=2, dtype="int16", device=None)
    node._voice_stream.write.assert_called_once()


def test_subsequent_chunks_reuse_stream(node):
    """Последующие чанки пишутся в тот же stream (не переоткрывают)."""
    data = np.zeros(320, dtype=np.int16).tobytes()

    node.voice_in_callback(_audio_msg(data))
    stream = node._voice_stream
    node.voice_in_callback(_audio_msg(data))

    assert node._voice_stream is stream, "чанки должны идти в один stream"
    assert stream.write.call_count == 2


def test_mono_duplicated_to_stereo(node):
    """Mono-чанк дублируется в stereo (ReSpeaker требует 2 канала)."""
    mono = np.array([1, 2, 3], dtype=np.int16)

    node.voice_in_callback(_audio_msg(mono.tobytes()))

    written = node._voice_stream.write.call_args[0][0]
    assert written.shape == (3, 2)
    np.testing.assert_array_equal(written[:, 0], mono)
    np.testing.assert_array_equal(written[:, 1], mono)


def test_silence_closes_stream(node):
    """Тишина > SILENCE_TIMEOUT закрывает stream."""
    node.voice_in_callback(_audio_msg(np.zeros(320, dtype=np.int16).tobytes()))
    assert node._voice_stream is not None

    closed = node._close_voice_stream_if_idle(now=node._voice_last_activity + SILENCE_TIMEOUT + 0.01)

    assert closed is True
    assert node._voice_stream is None


def test_silence_below_threshold_keeps_stream(node):
    """Тишина < SILENCE_TIMEOUT НЕ закрывает stream."""
    node.voice_in_callback(_audio_msg(np.zeros(320, dtype=np.int16).tobytes()))

    closed = node._close_voice_stream_if_idle(now=node._voice_last_activity + SILENCE_TIMEOUT - 0.01)

    assert closed is False
    assert node._voice_stream is not None


def test_empty_chunk_is_safe(node):
    """Пустой чанк не падает и не открывает stream."""
    node.voice_in_callback(_audio_msg(b""))

    assert node._voice_stream is None


def test_effect_trigger_does_not_interrupt_stream(node, monkeypatch):
    """Во время голосового стрима триггер эффекта не рвёт поток."""
    node.voice_in_callback(_audio_msg(np.zeros(320, dtype=np.int16).tobytes()))
    stream = node._voice_stream

    selected = []
    monkeypatch.setattr(node, "select_sound", lambda trigger: selected.append(trigger) or None)

    node.trigger_callback(_string_msg("talk"))

    assert selected == [], "при активном голосовом стриме эффект не должен выбираться"
    assert node._voice_stream is stream, "голосовой стрим не должен рваться эффектом"


def test_stop_closes_voice_stream(node):
    """STOP на /voice/sound/stop закрывает голосовой стрим."""
    node.voice_in_callback(_audio_msg(np.zeros(320, dtype=np.int16).tobytes()))
    assert node._voice_stream is not None

    node.sound_stop_callback(_string_msg("STOP"))

    assert node._voice_stream is None
