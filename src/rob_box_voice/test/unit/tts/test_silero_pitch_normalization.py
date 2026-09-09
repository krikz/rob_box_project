"""Юнит-тесты нормализации pitch для Silero fallback (issue #1064).

Silero v5 ``apply_tts`` принимает в ``<prosody pitch="...">`` только
``x-low|low|medium|high|x-high|robot``. LLM/MiniMax-стиль SSML генерирует
числовые множители (``1.2``, ``+10%``) — их прямая передача роняет Silero
с ``Invalid <prosody> tag``, и fallback молчит. Эти тесты проверяют, что:

- ``normalize_silero_pitch`` приводит любой вход к валидному уровню;
- ``_synthesize_silero`` с числовым/процентным/мусорным pitch возвращает
  аудио, а не exception (стаб имитирует строгую валидацию Silero v5);
- Yandex-путь не затронут: ``_parse_ssml_attributes`` по-прежнему отдаёт
  pitch как float, нормализация вызывается только для Silero.
"""
from __future__ import annotations

import re
import sys
from pathlib import Path

import numpy as np
import pytest

_PACKAGE_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(_PACKAGE_ROOT))

from test.unit.tts.conftest import _install_all_mocks

_install_all_mocks()

from rob_box_voice.tts_node import TTSNode, normalize_silero_pitch


class _Logger:
    def info(self, *args, **kwargs):
        pass

    def warn(self, *args, **kwargs):
        pass


class _FakeTensor:
    def __init__(self, values: list[float]):
        self._values = values

    def numpy(self) -> np.ndarray:
        return np.asarray(self._values, dtype=np.float32)


class _StrictPitchSilero:
    """Фейковый Silero v5: реджектит невалидный pitch в ``<prosody>``.

    Повторяет поведение реального ``apply_tts``: невалидный pitch →
    ``RuntimeError("Invalid <prosody> tag, pitch should be in ...")``.
    Если нормализация не сработает, тест упадёт ровно так, как падал
    реальный робот в e2e 08.08.
    """

    VALID_PITCH = {"x-low", "low", "medium", "high", "x-high", "robot"}

    def __init__(self):
        self.calls: list[str] = []
        self.pitches: list[str | None] = []

    def apply_tts(self, *, ssml_text: str, **kwargs):
        self.calls.append(ssml_text)
        m = re.search(r'<prosody pitch="([^"]+)"', ssml_text)
        pitch = m.group(1) if m else None
        self.pitches.append(pitch)
        if pitch is not None and pitch not in self.VALID_PITCH:
            raise RuntimeError(
                "Invalid <prosody> tag, pitch should be in "
                "x-low, low, medium, high, x-high, robot"
            )
        return _FakeTensor([float(len(ssml_text))])


def _build_silero_node(model: _StrictPitchSilero) -> TTSNode:
    node = TTSNode.__new__(TTSNode)
    node.silero_model = model
    node.silero_speaker = "baya"
    node.silero_sample_rate = 48000
    node.silero_put_accent = True
    node.silero_put_yo = True
    node.silero_put_stress_homo = True
    node.silero_put_yo_homo = True
    node.chunk_max_chars_silero = 800
    node.chunk_max_retries = 3
    node.chunk_min_chars = 50
    node.get_logger = lambda: _Logger()
    return node


# ─────────────────────────────────────────────────────────────────────────────
#  normalize_silero_pitch
# ─────────────────────────────────────────────────────────────────────────────

@pytest.mark.parametrize(
    ("pitch", "expected"),
    [
        (None, "medium"),
        (1.2, "high"),
        (0.8, "low"),
        (1.0, "medium"),
        ("1.2", "high"),
        ("+10%", "medium"),
        ("+25%", "high"),
        ("-30%", "low"),
        ("high", "high"),
        ("low", "low"),
        ("medium", "medium"),
        ("robot", "robot"),
        ("x-low", "x-low"),
        ("x-high", "x-high"),
        ("мусор", "medium"),
        ("abc", "medium"),
        ("", "medium"),
        (0.5, "x-low"),
        (1.5, "x-high"),
    ],
)
def test_normalize_silero_pitch(pitch, expected):
    assert normalize_silero_pitch(pitch) == expected


# ─────────────────────────────────────────────────────────────────────────────
#  _synthesize_silero с любым pitch → аудио, не exception
# ─────────────────────────────────────────────────────────────────────────────

@pytest.mark.parametrize(
    "pitch",
    [1.2, "+10%", "high", "мусор", None, 0.5, "robot", 1.0],
)
def test_synthesize_silero_accepts_any_pitch(pitch):
    model = _StrictPitchSilero()
    node = _build_silero_node(model)
    text = "Привет, робот. Это проверка fallback."

    audio = TTSNode._synthesize_silero(node, text, {"pitch": pitch})

    assert audio.dtype == np.float32
    # Стаб реджектит невалидный pitch, как реальный Silero v5, поэтому
    # успешный возврат означает, что нормализация сработала.
    assert len(model.calls) >= 1
    for p in model.pitches:
        assert p in _StrictPitchSilero.VALID_PITCH


def test_synthesize_silero_no_attributes_defaults_to_medium():
    model = _StrictPitchSilero()
    node = _build_silero_node(model)

    audio = TTSNode._synthesize_silero(node, "Просто текст.")

    assert audio.dtype == np.float32
    assert model.pitches == ["medium"]


# ─────────────────────────────────────────────────────────────────────────────
#  Yandex-путь не сломан: pitch остаётся float
# ─────────────────────────────────────────────────────────────────────────────

def test_parse_ssml_attributes_pitch_is_float():
    """Yandex-путь получает pitch как float (как и до фикса)."""
    node = TTSNode.__new__(TTSNode)
    attrs = TTSNode._parse_ssml_attributes(
        node, '<speak><prosody pitch="1.2" rate="1.1">текст</prosody></speak>'
    )
    assert attrs["pitch"] == 1.2
    assert isinstance(attrs["pitch"], float)
    assert attrs["rate"] == 1.1


def test_parse_ssml_attributes_percent_is_float():
    node = TTSNode.__new__(TTSNode)
    attrs = TTSNode._parse_ssml_attributes(
        node, '<speak><prosody pitch="+10%">текст</prosody></speak>'
    )
    assert attrs["pitch"] == pytest.approx(1.1)
    assert isinstance(attrs["pitch"], float)
