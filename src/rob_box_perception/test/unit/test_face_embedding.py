"""Unit-тесты для face_embedding (issue #2599 PR-B, ADR-0123).

Запуск:
    pytest src/rob_box_perception/test/unit/test_face_embedding.py -v

Что покрываем (pure-Python, БЕЗ hailo_platform / rclpy / реального железа;
Hailo-слой в тестах ArcFaceEmbedder подменяется фейком через monkeypatch
``_ensure_initialized`` — см. ``_install_fake_hailo``):
  1. crop_face — геометрия: расширение margin, клампинг к границам кадра,
     вырожденный/нулевой бокс -> None, бокс целиком снаружи -> None.
  2. cosine_similarity — идентичные/ортогональные векторы, несовпадение
     формы, нулевой вектор.
  3. ArcFaceEmbedder.embed — с фейковым Hailo-слоем: L2-нормализованные
     векторы правильной длины; битый кроп внутри списка -> None без
     исключения; embed([]) -> [] без обращения к железу.
  4. Отсутствующий HEF -> FileNotFoundError с путём в сообщении.
  5. prepare_arcface_input / sharpness / encode_jpeg — cv2-зависимые
     проверки (importorskip('cv2')).
"""

from __future__ import annotations

import importlib
import sys
from pathlib import Path

import numpy as np
import pytest


# ---------- import target under test ------------------------------------

# `parents[2]` = .../src/rob_box_perception → содержит пакет rob_box_perception/.
_HERE = Path(__file__).resolve()
_PKG_ROOT = _HERE.parents[2]
if str(_PKG_ROOT) not in sys.path:
    sys.path.insert(0, str(_PKG_ROOT))
face_mod = importlib.import_module('rob_box_perception.face_embedding')


# ---------- helpers: фейковый Hailo-слой ---------------------------------

class _FakeInputBuffer:
    def __init__(self) -> None:
        self.buffer = None

    def set_buffer(self, tensor) -> None:
        self.buffer = tensor


class _FakeOutputBuffer:
    def __init__(self, data) -> None:
        self._data = data

    def get_buffer(self):
        return self._data


class _FakeBindings:
    def __init__(self, output_data) -> None:
        self._input = _FakeInputBuffer()
        self._output = _FakeOutputBuffer(output_data)

    def input(self, name):
        return self._input

    def output(self, name):
        return self._output


class _FakeConfigured:
    def __init__(self) -> None:
        self.run_calls = 0

    def run(self, bindings_list, timeout=1000):
        self.run_calls += 1


def _install_fake_hailo(embedder, output_dim: int = 512):
    """Подменить ``_ensure_initialized`` так, чтобы embed() не трогал железо.

    Мимикрирует состояние, которое реальный ``_init_locked`` выставляет
    после успешной инициализации: ``_configured``/``_bindings`` не None,
    ``_embedding_dim`` из фактической формы выхода (не константа).
    """
    fake_configured = _FakeConfigured()
    fake_output = np.arange(output_dim, dtype=np.uint8)
    fake_bindings = _FakeBindings(fake_output)

    def _fake_ensure_initialized():
        embedder._configured = fake_configured
        embedder._bindings = fake_bindings
        embedder._input_name = 'input_layer1'
        embedder._output_name = 'fc1'
        embedder._embedding_dim = output_dim
        embedder._quant_scale = None
        embedder._quant_zero_point = None

    embedder._ensure_initialized = _fake_ensure_initialized
    return fake_configured


# ---------- crop_face ------------------------------------------------------

def test_crop_face_margin_expansion():
    frame = np.zeros((200, 200, 3), dtype=np.uint8)
    # bbox: cx=0.5, cy=0.5, w=0.2, h=0.2 -> 40x40 px без margin.
    crop = face_mod.crop_face(frame, (0.5, 0.5, 0.2, 0.2), margin=0.0)
    assert crop is not None
    assert crop.shape[:2] == (40, 40)

    # margin=0.5 -> сторона расширяется в (1 + 2*0.5) = 2 раза -> 80x80.
    crop2 = face_mod.crop_face(frame, (0.5, 0.5, 0.2, 0.2), margin=0.5)
    assert crop2 is not None
    assert crop2.shape[:2] == (80, 80)


def test_crop_face_clamps_at_frame_edges():
    frame = np.zeros((100, 100, 3), dtype=np.uint8)
    # bbox у левого верхнего угла, margin достаточно большой чтобы
    # расширение вышло за границы кадра.
    crop = face_mod.crop_face(frame, (0.05, 0.05, 0.1, 0.1), margin=0.4)
    assert crop is not None
    # Клампится к (0, 0) сверху-слева, размер меньше "идеального".
    assert crop.shape[0] > 0 and crop.shape[1] > 0
    assert crop.shape[0] <= 100 and crop.shape[1] <= 100


def test_crop_face_degenerate_box_returns_none():
    frame = np.zeros((100, 100, 3), dtype=np.uint8)
    assert face_mod.crop_face(frame, (0.5, 0.5, 0.0, 0.2), margin=0.4) is None
    assert face_mod.crop_face(frame, (0.5, 0.5, 0.2, 0.0), margin=0.4) is None
    assert face_mod.crop_face(frame, (0.5, 0.5, -0.1, 0.2), margin=0.4) is None


def test_crop_face_fully_outside_frame_returns_none():
    frame = np.zeros((100, 100, 3), dtype=np.uint8)
    # Центр бокса далеко за пределами [0, 1] -> после клампа пустая область.
    assert face_mod.crop_face(frame, (2.0, 2.0, 0.1, 0.1), margin=0.4) is None
    assert face_mod.crop_face(frame, (-1.0, -1.0, 0.1, 0.1), margin=0.4) is None


def test_crop_face_none_frame_returns_none():
    assert face_mod.crop_face(None, (0.5, 0.5, 0.2, 0.2)) is None


# ---------- cosine_similarity ---------------------------------------------

def test_cosine_similarity_identical_vectors():
    a = np.array([1.0, 2.0, 3.0], dtype=np.float32)
    assert face_mod.cosine_similarity(a, a) == pytest.approx(1.0, abs=1e-6)


def test_cosine_similarity_orthogonal_vectors():
    a = np.array([1.0, 0.0], dtype=np.float32)
    b = np.array([0.0, 1.0], dtype=np.float32)
    assert face_mod.cosine_similarity(a, b) == pytest.approx(0.0, abs=1e-6)


def test_cosine_similarity_shape_mismatch_returns_zero():
    a = np.array([1.0, 2.0, 3.0], dtype=np.float32)
    b = np.array([1.0, 2.0], dtype=np.float32)
    assert face_mod.cosine_similarity(a, b) == 0.0


def test_cosine_similarity_zero_vector_returns_zero():
    a = np.zeros(4, dtype=np.float32)
    b = np.array([1.0, 2.0, 3.0, 4.0], dtype=np.float32)
    assert face_mod.cosine_similarity(a, b) == 0.0
    assert face_mod.cosine_similarity(a, a) == 0.0


# ---------- ArcFaceEmbedder.embed (fake Hailo layer) -----------------------

def test_embed_returns_normalized_vectors_of_right_length():
    embedder = face_mod.ArcFaceEmbedder(hef_path='/tmp/fake_arcface.hef')
    _install_fake_hailo(embedder, output_dim=512)

    crop = np.random.randint(0, 255, size=(100, 120, 3), dtype=np.uint8)
    results = embedder.embed([crop])

    assert len(results) == 1
    vec = results[0]
    assert vec is not None
    assert vec.shape == (512,)
    assert vec.dtype == np.float32
    assert np.linalg.norm(vec) == pytest.approx(1.0, abs=1e-4)
    assert embedder.embedding_dim == 512


def test_embed_bad_crop_yields_none_without_raising():
    embedder = face_mod.ArcFaceEmbedder(hef_path='/tmp/fake_arcface.hef')
    _install_fake_hailo(embedder, output_dim=512)

    good_crop = np.random.randint(0, 255, size=(80, 80, 3), dtype=np.uint8)
    bad_crop = np.zeros((0, 50, 3), dtype=np.uint8)  # вырожденная форма

    results = embedder.embed([good_crop, bad_crop])

    assert len(results) == 2
    assert results[0] is not None
    assert results[0].shape == (512,)
    assert results[1] is None


def test_embed_empty_list_returns_empty_without_touching_device():
    embedder = face_mod.ArcFaceEmbedder(hef_path='/tmp/fake_arcface.hef')

    def _boom():
        raise AssertionError('embed([]) не должен инициализировать железо')

    embedder._ensure_initialized = _boom

    assert embedder.embed([]) == []


def test_embedding_dim_fallback_before_init():
    embedder = face_mod.ArcFaceEmbedder(hef_path='/tmp/fake_arcface.hef')
    # До инициализации свойство не должно трогать железо и обязано
    # вернуть разумный fallback (512 — фактическая форма выхода
    # arcface_mobilefacenet, см. модульный docstring).
    assert embedder.embedding_dim == 512


def test_missing_hef_raises_file_not_found_with_path():
    embedder = face_mod.ArcFaceEmbedder(hef_path='/tmp/definitely_missing_arcface.hef')
    crop = np.zeros((80, 80, 3), dtype=np.uint8)

    with pytest.raises(FileNotFoundError) as excinfo:
        embedder.embed([crop])

    assert '/tmp/definitely_missing_arcface.hef' in str(excinfo.value)


# ---------- prepare_arcface_input / sharpness / encode_jpeg (cv2) ----------

def test_prepare_arcface_input_resizes_to_112():
    pytest.importorskip('cv2')
    crop = np.random.randint(0, 255, size=(50, 70, 3), dtype=np.uint8)
    prepared = face_mod.prepare_arcface_input(crop)
    assert prepared is not None
    assert prepared.shape == (face_mod.ARCFACE_INPUT_SIZE, face_mod.ARCFACE_INPUT_SIZE, 3)
    assert prepared.dtype == np.uint8


def test_prepare_arcface_input_none_on_empty_crop():
    pytest.importorskip('cv2')
    empty = np.zeros((0, 10, 3), dtype=np.uint8)
    assert face_mod.prepare_arcface_input(empty) is None
    assert face_mod.prepare_arcface_input(None) is None


def test_sharpness_returns_nonnegative_float():
    pytest.importorskip('cv2')
    crop = np.random.randint(0, 255, size=(80, 80, 3), dtype=np.uint8)
    score = face_mod.sharpness(crop)
    assert isinstance(score, float)
    assert score >= 0.0


def test_sharpness_zero_on_empty_crop():
    empty = np.zeros((0, 10, 3), dtype=np.uint8)
    assert face_mod.sharpness(empty) == 0.0
    assert face_mod.sharpness(None) == 0.0


def test_encode_jpeg_roundtrip_produces_bytes():
    pytest.importorskip('cv2')
    crop = np.random.randint(0, 255, size=(64, 64, 3), dtype=np.uint8)
    data = face_mod.encode_jpeg(crop, quality=85)
    assert data is not None
    assert isinstance(data, (bytes, bytearray))
    assert len(data) > 0
    # JPEG magic bytes.
    assert bytes(data[:2]) == b'\xff\xd8'


def test_encode_jpeg_none_on_none_input():
    assert face_mod.encode_jpeg(None) is None
