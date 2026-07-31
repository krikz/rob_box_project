"""Hot-path regression tests for TTS retry-halving (gap G-933-A)."""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

_PACKAGE_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(_PACKAGE_ROOT))

from test.unit.tts.conftest import _install_all_mocks

_install_all_mocks()

from rob_box_voice.tts_node import TTSNode


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


class _ThresholdSilero:
    def __init__(self, max_chars: int):
        self.max_chars = max_chars
        self.calls: list[str] = []
        self.successful_calls: list[str] = []

    def apply_tts(self, *, ssml_text: str, **kwargs):
        text = ssml_text.split(">", 2)[-1].split("<", 1)[0]
        self.calls.append(text)
        if len(text) > self.max_chars:
            raise RuntimeError("Model couldn't generate your text: length limit")
        self.successful_calls.append(text)
        return _FakeTensor([float(len(text))])


def _build_silero_node(model: _ThresholdSilero) -> TTSNode:
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


def test_silero_1005_chars_splits_at_configured_limit() -> None:
    model = _ThresholdSilero(max_chars=800)
    node = _build_silero_node(model)
    text = "А. " * 335
    assert len(text) == 1005

    audio = TTSNode._synthesize_silero(node, text)

    assert len(model.calls) > 1
    assert all(len(chunk) <= node.chunk_max_chars_silero for chunk in model.calls)
    assert audio.dtype == np.float32


def test_silero_retries_by_halving_provider_rejection() -> None:
    model = _ThresholdSilero(max_chars=220)
    node = _build_silero_node(model)
    text = ("Retry this Silero phrase. " * 20).strip()
    assert 220 < len(text) < node.chunk_max_chars_silero

    audio = TTSNode._synthesize_silero(node, text)

    assert model.calls[0] == text
    assert len(model.calls) > 1
    assert len(model.successful_calls) == 4
    assert all(len(chunk) <= model.max_chars for chunk in model.successful_calls)
    assert audio.dtype == np.float32
