"""Запрос Yandex SpeechKit v3 из ``tts_node``: в ``Hints`` только голос и скорость (#4240).

Цепочка SSML-просодии (``<prosody pitch/volume>`` → ``Hints(pitch_shift, volume)``, issue #1780)
удалена: с провода (ADR-0160 §2.3, #4239) атрибуты больше не приходят, скорость — ROS-параметр
``yandex_speed``. ``pitch_shift`` в ``build_yandex_request`` остаётся только у вокала песни
(``sing_synth``, см. ``test_sing_presets``).
"""
from __future__ import annotations

import sys
import types
from pathlib import Path


_PACKAGE_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(_PACKAGE_ROOT))

from test.unit.tts.conftest import _install_all_mocks

_install_all_mocks()

# ── Замокать yandex.cloud.ai.tts.v3 ДО импорта tts_node (как в
# test_yandex_chunking.py). Без этого ``tts_service_pb2_grpc`` падает
# на ``grpc.__version__`` (MagicMock не имеет атрибута).
_yandex_v3 = sys.modules.get("yandex.cloud.ai.tts.v3")
if _yandex_v3 is None or not hasattr(_yandex_v3, "tts_pb2"):

    class _Container:
        WAV = 0

        def __init__(self, container_audio_type=None):
            self.container_audio_type = container_audio_type

    class _AudioFormatOptions:
        def __init__(self, container_audio=None):
            self.container_audio = container_audio

    class _Hints:
        def __init__(self, voice=None, speed=None, pitch_shift=None, volume=None, role=None):
            self.voice = voice
            self.speed = speed
            self.pitch_shift = pitch_shift
            self.volume = volume
            self.role = role

    class _UtteranceSynthesisRequest:
        LUFS = 0

        def __init__(self, text=None, output_audio_spec=None, hints=None,
                     loudness_normalization_type=None):
            self.text = text
            self.output_audio_spec = output_audio_spec
            self.hints = hints
            self.loudness_normalization_type = loudness_normalization_type

    class _TTS_pb2:
        ContainerAudio = _Container
        AudioFormatOptions = _AudioFormatOptions
        Hints = _Hints
        UtteranceSynthesisRequest = _UtteranceSynthesisRequest

    if _yandex_v3 is None:
        yandex_root = types.ModuleType("yandex")
        yc = types.ModuleType("yandex.cloud")
        yc_ai = types.ModuleType("yandex.cloud.ai")
        yc_ai_tts = types.ModuleType("yandex.cloud.ai.tts")
        yc_ai_tts_v3 = types.ModuleType("yandex.cloud.ai.tts.v3")
        yc_ai_tts_v3.tts_pb2 = _TTS_pb2()
        yc_ai_tts_v3.tts_service_pb2_grpc = types.SimpleNamespace(
            SynthesizerStub=lambda *a, **kw: None
        )
        sys.modules["yandex"] = yandex_root
        sys.modules["yandex.cloud"] = yc
        sys.modules["yandex.cloud.ai"] = yc_ai
        sys.modules["yandex.cloud.ai.tts"] = yc_ai_tts
        sys.modules["yandex.cloud.ai.tts.v3"] = yc_ai_tts_v3
    else:
        _yandex_v3.tts_pb2 = _TTS_pb2()
        _yandex_v3.tts_service_pb2_grpc = types.SimpleNamespace(
            SynthesizerStub=lambda *a, **kw: None
        )

if "rob_box_voice.tts_node" in sys.modules:
    del sys.modules["rob_box_voice.tts_node"]

from rob_box_voice.tts_node import TTSNode  # noqa: E402


# ─────────────────────────────────────────────────────────────────────────────
#  _synthesize_yandex — Hints из параметров ноды
# ─────────────────────────────────────────────────────────────────────────────


def _make_wav_bytes(num_samples: int = 2205, sample_rate: int = 22050) -> bytes:
    """Минимальный mono PCM16 WAV — то же, что в test_yandex_chunking.py."""
    import io
    import struct
    import wave

    buf = io.BytesIO()
    with wave.open(buf, "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(sample_rate)
        frames = b""
        for i in range(num_samples):
            v = int(8000 * (i % 7 - 3))
            frames += struct.pack("<h", v)
        w.writeframes(frames)
    return buf.getvalue()


class _CapturingYandexStub:
    """Захватывает каждый ``UtteranceSynthesis`` и его hints."""

    def __init__(self):
        self.calls: list[tuple[str, list]] = []

    def UtteranceSynthesis(self, request, metadata=None):
        self.calls.append((request.text, list(request.hints or [])))
        from test.unit.tts.test_yandex_chunking import _FakeResponse

        return iter([_FakeResponse(_make_wav_bytes())])


def _build_node(stub) -> TTSNode:
    from rob_box_voice.tts_node import (
        DEFAULT_MAX_RETRIES,
        MIN_CHUNK_CHARS,
        YANDEX_MAX_CHUNK_CHARS,
    )

    node = TTSNode.__new__(TTSNode)
    node.yandex_stub = stub
    node.yandex_voice = "anton"
    node.yandex_speed = 1.0
    node.yandex_api_key = "test-key"
    node.chunk_max_chars_yandex = YANDEX_MAX_CHUNK_CHARS
    node.chunk_max_retries = DEFAULT_MAX_RETRIES
    node.chunk_min_chars = MIN_CHUNK_CHARS

    class _Logger:
        def info(self, *a, **kw):
            pass

        def warn(self, *a, **kw):
            pass

        def error(self, *a, **kw):
            pass

        def debug(self, *a, **kw):
            pass

    node.get_logger = lambda: _Logger()
    return node


def _hints_to_kwargs(hints: list) -> dict:
    """Сливаем список Hints в плоский dict (по одному ключу на hint)."""
    merged: dict = {}
    for h in hints:
        if h.voice is not None:
            merged["voice"] = h.voice
        if h.speed is not None:
            merged["speed"] = h.speed
        if h.pitch_shift is not None:
            merged["pitch_shift"] = h.pitch_shift
        if h.volume is not None:
            merged["volume"] = h.volume
    return merged


def test_synthesize_yandex_hints_are_voice_and_speed_only():
    """Hints — голос и скорость из параметров ноды; pitch_shift/volume нет (Yandex берёт свои дефолты)."""
    stub = _CapturingYandexStub()
    node = _build_node(stub)
    TTSNode._synthesize_yandex(node, "Просто текст.")  # type: ignore[arg-type]

    assert len(stub.calls) == 1
    kwargs = _hints_to_kwargs(stub.calls[0][1])
    assert "pitch_shift" not in kwargs
    assert "volume" not in kwargs
    # voice и speed — обязательные.
    assert kwargs["voice"] == "anton"
    assert kwargs["speed"] == 1.0
