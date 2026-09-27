"""Юнит-тесты для issue #1780: применение SSML ``<prosody pitch/volume>``
к Yandex Cloud TTS gRPC v3 через ``Hints(pitch_shift, volume)``.

До фикса:
    ``tts_node.py:2792`` (старая нумерация) логировал «SSML pitch=…
    (не применяется в Yandex TTS)», а ``_synthesize_yandex_single``
    собирал ``hints=[Hints(voice=...), Hints(speed=...)]`` — никакой
    конвертации SSML → Yandex не было, ключ ``prosody_pitch`` был
    удалён из конфига ещё в issue #1004.

После фикса:
    * ``_ssml_pitch_to_hz``      — float/процент/имя → Hz-offset;
    * ``_ssml_volume_to_lufs_target`` — dB/процент/имя → LUFS-цель;
    * ``_parse_ssml_attributes`` — извлекает ``pitch`` и ``volume`` из SSML;
    * ``_synthesize_yandex``     — пробрасывает ``pitch_hz``/``volume_lufs``
      в ``_synthesize_yandex_single``;
    * ``_synthesize_yandex_single`` — кладёт их в ``Hints(pitch_shift, volume)``.

Тесты покрывают конвертацию (модульный уровень) и сквозной путь
SSML → Hints (с замоканным gRPC-stub-ом).
"""
from __future__ import annotations

import re
import sys
import types
from pathlib import Path

import pytest

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
        def __init__(self, voice=None, speed=None, pitch_shift=None, volume=None):
            self.voice = voice
            self.speed = speed
            self.pitch_shift = pitch_shift
            self.volume = volume

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

from rob_box_voice.tts_node import (  # noqa: E402
    TTSNode,
    YANDEX_BASELINE_PITCH_HZ,
    YANDEX_BASELINE_VOLUME_LUFS,
    _ssml_pitch_to_hz,
    _ssml_volume_to_lufs_target,
)


# ─────────────────────────────────────────────────────────────────────────────
#  _ssml_pitch_to_hz — модульные тесты
# ─────────────────────────────────────────────────────────────────────────────


@pytest.mark.parametrize(
    ("pitch", "expected_hz"),
    [
        # Числовой множитель 1.0 = без сдвига.
        (1.0, 0.0),
        (None, None),
        # +10% множителя → +10% от baseline 130 Hz.
        ("+10%", pytest.approx(YANDEX_BASELINE_PITCH_HZ * 0.1)),
        ("-25%", pytest.approx(-YANDEX_BASELINE_PITCH_HZ * 0.25)),
        # Именованные уровни SSML.
        ("high", pytest.approx(YANDEX_BASELINE_PITCH_HZ * 0.2)),
        ("low", pytest.approx(-YANDEX_BASELINE_PITCH_HZ * 0.2)),
        ("x-high", pytest.approx(YANDEX_BASELINE_PITCH_HZ * 0.5)),
        ("x-low", pytest.approx(-YANDEX_BASELINE_PITCH_HZ * 0.5)),
        ("medium", 0.0),
        # "robot" — спец-эффект, не тон → None (Silero использует, Yandex — нет).
        ("robot", None),
        # Числовой множитель как строка.
        ("1.2", pytest.approx(YANDEX_BASELINE_PITCH_HZ * 0.2)),
        # Мусор → None.
        ("мусор", None),
        ("", None),
        ("high;", None),  # trailing ';' в шаблоне attrs_str режется regex'ом,
                          # но как самостоятельный вход функции → None.
    ],
)
def test_ssml_pitch_to_hz(pitch, expected_hz):
    if expected_hz is None:
        assert _ssml_pitch_to_hz(pitch) is None
    else:
        assert _ssml_pitch_to_hz(pitch) == pytest.approx(expected_hz)


def test_ssml_pitch_to_hz_clamps_extreme():
    """Коэффициент, выходящий за [-1000/130+1; 1000/130+1], clamp'ится."""
    # factor=100 → 130 * 99 = 12870 Hz → clamp до 1000 Hz.
    assert _ssml_pitch_to_hz(100.0) == pytest.approx(1000.0)
    assert _ssml_pitch_to_hz(0.0) == pytest.approx(-YANDEX_BASELINE_PITCH_HZ)
    assert _ssml_pitch_to_hz(-100.0) == pytest.approx(-1000.0)


# ─────────────────────────────────────────────────────────────────────────────
#  _ssml_volume_to_lufs_target — модульные тесты
# ─────────────────────────────────────────────────────────────────────────────


@pytest.mark.parametrize(
    ("volume", "expected_lufs"),
    [
        # medium = дефолт Yandex (без смещения).
        ("medium", YANDEX_BASELINE_VOLUME_LUFS),
        # Именованные уровни SSML: 6 dB шаг.
        ("loud", YANDEX_BASELINE_VOLUME_LUFS + 6.0),
        ("x-loud", YANDEX_BASELINE_VOLUME_LUFS + 12.0),
        ("soft", YANDEX_BASELINE_VOLUME_LUFS - 6.0),
        ("x-soft", YANDEX_BASELINE_VOLUME_LUFS - 12.0),
        ("silent", -145.0),  # clamp в нижний предел.
        # Числовые dB-offset'ы: "+5dB" = baseline+5, "-3dB" = baseline-3.
        ("+5dB", YANDEX_BASELINE_VOLUME_LUFS + 5.0),
        ("-3dB", YANDEX_BASELINE_VOLUME_LUFS - 3.0),
        # Проценты: 100% = +6 dB, 50% = +3 dB.
        ("+100%", YANDEX_BASELINE_VOLUME_LUFS + 6.0),
        ("-50%", YANDEX_BASELINE_VOLUME_LUFS - 3.0),
        # Без суффикса dB — голое число как dB-offset.
        ("+5", YANDEX_BASELINE_VOLUME_LUFS + 5.0),
        # Числовое значение как float/int.
        (5.0, YANDEX_BASELINE_VOLUME_LUFS + 5.0),
        (-3, YANDEX_BASELINE_VOLUME_LUFS - 3.0),
        # None / мусор → None.
        (None, None),
        ("мусор", None),
    ],
)
def test_ssml_volume_to_lufs_target(volume, expected_lufs):
    if expected_lufs is None:
        assert _ssml_volume_to_lufs_target(volume) is None
    else:
        assert _ssml_volume_to_lufs_target(volume) == pytest.approx(expected_lufs)


def test_ssml_volume_clamps_to_yandex_range():
    """target LUFS clamp'ится в [-145; 0)."""
    # +1000 dB → должно быть clamp'нуто до -1 LUFS (верхняя граница).
    assert _ssml_volume_to_lufs_target("+1000dB") == pytest.approx(-1.0)
    # -1000 dB → должно быть clamp'нуто до -145 LUFS (нижняя граница).
    assert _ssml_volume_to_lufs_target("-1000dB") == pytest.approx(-145.0)
    # Невалидный процент → None.
    assert _ssml_volume_to_lufs_target("+abc%") is None


# ─────────────────────────────────────────────────────────────────────────────
#  _parse_ssml_attributes — end-to-end SSML → dict
# ─────────────────────────────────────────────────────────────────────────────


@pytest.mark.parametrize(
    ("ssml", "expected"),
    [
        # Без prosody — пустой dict.
        ("<speak>Просто текст</speak>", {}),
        # Только pitch.
        (
            '<speak><prosody pitch="+10%">текст</prosody></speak>',
            {"pitch": pytest.approx(1.1)},
        ),
        # Pitch + rate.
        (
            '<speak><prosody pitch="1.2" rate="1.1">текст</prosody></speak>',
            {"pitch": pytest.approx(1.2), "rate": pytest.approx(1.1)},
        ),
        # Pitch + volume (числовой dB-offset).
        (
            '<speak><prosody pitch="high" volume="loud">текст</prosody></speak>',
            {
                "pitch": pytest.approx(1.2),
                "volume": pytest.approx(YANDEX_BASELINE_VOLUME_LUFS + 6.0),
            },
        ),
        # Volume как именованный SSML-уровень → LUFS-цель.
        (
            '<speak><prosody volume="soft">текст</prosody></speak>',
            {"volume": pytest.approx(YANDEX_BASELINE_VOLUME_LUFS - 6.0)},
        ),
    ],
)
def test_parse_ssml_attributes_extracts_pitch_volume(ssml, expected):
    node = TTSNode.__new__(TTSNode)
    attrs = TTSNode._parse_ssml_attributes(node, ssml)
    assert attrs == expected


# ─────────────────────────────────────────────────────────────────────────────
#  _synthesize_yandex_single — Hints конструируется с pitch_shift/volume
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


def test_synthesize_yandex_applies_ssml_pitch_and_volume():
    """Issue #1780: pitch=high + volume=loud → Hints содержат
    pitch_shift=+26 Hz и volume=-13 LUFS."""
    stub = _CapturingYandexStub()
    node = _build_node(stub)
    ssml = (
        '<speak><prosody pitch="high" volume="loud">'
        "Привет, робот"
        "</prosody></speak>"
    )
    # Парсим атрибуты через штатный метод (так делает _synthesize_yandex).
    attrs = TTSNode._parse_ssml_attributes(node, ssml)
    audio = TTSNode._synthesize_yandex(node, "Привет, робот", attrs)  # type: ignore[arg-type]

    assert isinstance(audio, type(audio))  # np.ndarray duck-check
    assert len(stub.calls) == 1
    kwargs = _hints_to_kwargs(stub.calls[0][1])
    # high = 1.2 → +26 Hz (1.2 - 1.0) * 130.
    assert kwargs["pitch_shift"] == pytest.approx(YANDEX_BASELINE_PITCH_HZ * 0.2)
    # loud = +6 dB → -13 LUFS (baseline -19 + 6).
    assert kwargs["volume"] == pytest.approx(YANDEX_BASELINE_VOLUME_LUFS + 6.0)


def test_synthesize_yandex_no_ssml_attributes_omits_hints():
    """Без SSML pitch/volume — Hints НЕ содержит pitch_shift/volume
    (Yandex возьмёт свои дефолты)."""
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


def test_synthesize_yandex_partial_ssml_only_pitch():
    """Только pitch в SSML — только pitch_shift в Hints, без volume."""
    stub = _CapturingYandexStub()
    node = _build_node(stub)
    attrs = TTSNode._parse_ssml_attributes(
        node, '<speak><prosody pitch="low">тихо</prosody></speak>'
    )
    TTSNode._synthesize_yandex(node, "тихо", attrs)  # type: ignore[arg-type]

    kwargs = _hints_to_kwargs(stub.calls[0][1])
    # low = 0.8 → -26 Hz.
    assert kwargs["pitch_shift"] == pytest.approx(-YANDEX_BASELINE_PITCH_HZ * 0.2)
    assert "volume" not in kwargs


def test_synthesize_yandex_partial_ssml_only_volume():
    """Только volume в SSML — только volume в Hints, без pitch_shift."""
    stub = _CapturingYandexStub()
    node = _build_node(stub)
    attrs = TTSNode._parse_ssml_attributes(
        node, '<speak><prosody volume="x-loud">громко</prosody></speak>'
    )
    TTSNode._synthesize_yandex(node, "громко", attrs)  # type: ignore[arg-type]

    kwargs = _hints_to_kwargs(stub.calls[0][1])
    # x-loud = +12 dB → -7 LUFS.
    assert kwargs["volume"] == pytest.approx(YANDEX_BASELINE_VOLUME_LUFS + 12.0)
    assert "pitch_shift" not in kwargs


def test_synthesize_yandex_logs_application():
    """Лог «применяется в Yandex TTS» содержит оба параметра, не «не применяется»."""
    captured: list[str] = []

    class _CapturingLogger:
        def info(self, msg, *a, **kw):
            captured.append(str(msg))

    stub = _CapturingYandexStub()
    node = _build_node(stub)
    node.get_logger = lambda: _CapturingLogger()
    attrs = TTSNode._parse_ssml_attributes(
        node, '<speak><prosody pitch="+5%" volume="loud">x</prosody></speak>'
    )
    TTSNode._synthesize_yandex(node, "x", attrs)  # type: ignore[arg-type]

    applied_logs = [m for m in captured if "применяется в Yandex TTS" in m]
    assert applied_logs, "Ожидался лог 'применяется в Yandex TTS'"
    assert "не применяется" not in " ".join(captured), (
        "Старый лог «не применяется в Yandex TTS» не должен появляться"
    )
    # В логе должны быть обе величины.
    assert "pitch_shift" in applied_logs[0]
    assert "volume" in applied_logs[0]
