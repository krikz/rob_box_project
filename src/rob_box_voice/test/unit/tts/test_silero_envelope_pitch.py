"""Silero получает валидный ``<prosody pitch="medium">`` (issue #1064, #4240).

Silero v5 ``apply_tts`` принимает в ``<prosody pitch>`` только ``x-low|low|medium|high|x-high|robot``,
на числовой множитель падает с ``Invalid <prosody> tag``. Нормализация pitch из SSML-атрибутов
(``normalize_silero_pitch``) удалена вместе с цепочкой ``ssml_attributes`` (#4240): с провода
просодия не приходит, конверт ``tts_ssml.silero_ssml`` всегда ставит ``medium``. Стаб повторяет
строгую валидацию Silero v5.
"""
from __future__ import annotations

import re
import sys
from pathlib import Path

import numpy as np

_PACKAGE_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(_PACKAGE_ROOT))

from test.unit.tts.conftest import _install_all_mocks

_install_all_mocks()

from rob_box_voice.tts_providers.silero import SileroTTSProvider


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


def _build_silero_node(model: _StrictPitchSilero) -> SileroTTSProvider:
    """Адаптер Silero (#4240) с фейковой моделью."""
    adapter = SileroTTSProvider(logger=_Logger(), default_speaker="baya")
    adapter.model = model
    return adapter


def test_synthesize_silero_envelope_pitch_is_medium():
    model = _StrictPitchSilero()
    node = _build_silero_node(model)

    audio = node.synthesize_np("Просто текст.")

    assert audio.dtype == np.float32
    assert model.pitches == ["medium"]
