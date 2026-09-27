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
  6. crop_brightness_contrast — метрика ворот качества кропа (issue #2749):
     почти чёрный кроп даёт низкие mean/контраст, светлый/контрастный —
     нет; ``None`` при вырожденном/пустом кропе (cv2-зависимые проверки).
  7. align_face (issue #2773) — similarity-transform по 5 точкам на
     канонический шаблон ArcFace: точное совпадение шаблона -> выход
     совпадает с шаблоном; повёрнутый/смещённый/масштабированный набор
     точек -> выравнивание восстанавливает канонические позиции;
     None-пути (landmarks None/неверной длины/NaN, cv2 недоступен,
     вырожденные точки).

ЧЕСТНО (issue #2773 acceptance): geometрические тесты ``align_face``
ниже проверяют, что функция считает и применяет similarity-transform
корректно на СИНТЕТИЧЕСКИХ точках (математика cv2), а не на реальном
лице с реального HEF — насколько лучше реальные эмбеддинги на живом
роботе, тестами не проверено, это отдельный замер на железе.
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
    pytest.importorskip('cv2')  # prepare_arcface_input -> cv2.resize
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
    pytest.importorskip('cv2')  # prepare_arcface_input -> cv2.resize
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


# ---------- crop_brightness_contrast (ворота качества кропа, issue #2749) --
#
# Пороговые значения (DEFAULT_MIN_CROP_MEAN=20, DEFAULT_MIN_CROP_CONTRAST=25
# в face_recognition.py) подобраны по живому замеру на Vision Pi 22.09.2026:
# фантомная запись (тень в тёмной комнате) — mean 6.4-7.4/255,
# контраст(p95-p5) 28-31; живой человек в той же базе — mean 76.6-185.4/255,
# контраст 170-208. Тесты ниже проверяют саму функцию метрики, не пороги
# (пороги и их обоснование — в test_face_recognition.py, раздел 10).

def test_crop_brightness_contrast_near_black_crop_is_dim_and_flat():
    pytest.importorskip('cv2')
    # Не буквальный 0 — воспроизводим шумовой пол матрицы, как в реальном
    # фантоме issue #2749 (mean ~7/255), а не идеализированный чёрный.
    rng = np.random.RandomState(1)
    crop = rng.randint(0, 8, size=(80, 80, 3)).astype(np.uint8)

    result = face_mod.crop_brightness_contrast(crop)

    assert result is not None
    mean, contrast = result
    assert mean < 20.0, 'почти чёрный кроп обязан давать низкую среднюю яркость'
    assert mean == pytest.approx(3.5, abs=2.0)


def test_crop_brightness_contrast_bright_textured_crop_passes():
    pytest.importorskip('cv2')
    rng = np.random.RandomState(2)
    crop = rng.randint(0, 256, size=(80, 80, 3)).astype(np.uint8)

    result = face_mod.crop_brightness_contrast(crop)

    assert result is not None
    mean, contrast = result
    assert mean > 100.0
    assert contrast > 100.0, 'равномерный шум 0..255 обязан давать контраст сильно выше порога 25'


def test_crop_brightness_contrast_flat_gray_crop_has_zero_contrast():
    """Пересвеченный/плоский кроп: яркость в норме, а разброса нет —
    второй, независимый от mean сигнал ворот (см. DEFAULT_MIN_CROP_CONTRAST).
    """
    pytest.importorskip('cv2')
    crop = np.full((80, 80, 3), 200, dtype=np.uint8)

    result = face_mod.crop_brightness_contrast(crop)

    assert result is not None
    mean, contrast = result
    assert mean == pytest.approx(200.0, abs=1.0)
    assert contrast == pytest.approx(0.0, abs=1.0)


def test_crop_brightness_contrast_none_on_empty_or_none_crop():
    empty = np.zeros((0, 10, 3), dtype=np.uint8)
    assert face_mod.crop_brightness_contrast(empty) is None
    assert face_mod.crop_brightness_contrast(None) is None


# ---------- align_face (issue #2773) ----------------------------------------
#
# Канонический ArcFace-препроцессинг вместо невыровненного растянутого
# кропа (crop_face + prepare_arcface_input) — корень внутриперсонного
# разброса эмбеддингов 0.3-0.85 на живом роботе (issue #2773). Тесты
# ниже проверяют геометрию similarity-transform на синтетических точках:
# они не могут подтвердить улучшение на реальных лицах (для этого нужен
# живой HEF + живая галерея), только то, что cv2.estimateAffinePartial2D
# + cv2.warpAffine действительно восстанавливают канонические позиции
# точек при известном повороте/масштабе/сдвиге.

def _rotate_scale_translate(points, angle_deg, scale, tx, ty):
    """Применить известный similarity-transform к набору 2D точек (numpy)."""
    theta = np.deg2rad(angle_deg)
    cos_t, sin_t = np.cos(theta), np.sin(theta)
    rot = np.array([[cos_t, -sin_t], [sin_t, cos_t]]) * scale
    pts = np.asarray(points, dtype=np.float64)
    return pts @ rot.T + np.array([tx, ty], dtype=np.float64)


def test_similarity_transform_umeyama_identity():
    pts = np.array(
        [[0.0, 0.0], [1.0, 0.0], [0.0, 1.0], [1.0, 1.0], [0.5, 0.5]],
        dtype=np.float64,
    )
    m = face_mod._similarity_transform_umeyama(pts, pts)
    assert m is not None
    assert np.allclose(m[:, :2], np.eye(2), atol=1e-9)
    assert np.allclose(m[:, 2], [0.0, 0.0], atol=1e-9)


def test_similarity_transform_umeyama_recovers_known_transform():
    """Точки, полученные из шаблона известным R/scale/t, обязаны точно
    восстанавливаться обратно — прямая проверка формулы Umeyama, без
    промежуточного round-trip через изображение/warpAffine."""
    template = np.asarray(face_mod._ARCFACE_TEMPLATE_112, dtype=np.float64)
    src = _rotate_scale_translate(
        template, angle_deg=32.0, scale=1.4, tx=-20.0, ty=50.0
    )
    m = face_mod._similarity_transform_umeyama(src, template)
    assert m is not None
    recovered = (m[:, :2] @ src.T).T + m[:, 2]
    assert np.allclose(recovered, template, atol=1e-6)


def test_similarity_transform_umeyama_is_deterministic():
    """Формула детерминирована по построению (в отличие от RANSAC) —
    повторный вызов на тех же точках обязан дать побитово тот же результат."""
    template = np.asarray(face_mod._ARCFACE_TEMPLATE_112, dtype=np.float64)
    src = _rotate_scale_translate(
        template, angle_deg=-10.0, scale=0.7, tx=5.0, ty=5.0
    )
    m1 = face_mod._similarity_transform_umeyama(src, template)
    m2 = face_mod._similarity_transform_umeyama(src, template)
    assert m1 is not None and m2 is not None
    assert np.array_equal(m1, m2)


def test_similarity_transform_umeyama_none_on_degenerate_src():
    dst = np.asarray(face_mod._ARCFACE_TEMPLATE_112, dtype=np.float64)
    src = np.array([[10.0, 10.0]] * 5, dtype=np.float64)
    assert face_mod._similarity_transform_umeyama(src, dst) is None


def test_align_face_recovers_canonical_template_under_rotation_scale_shift():
    """Точки, полученные из шаблона известным поворотом/масштабом/сдвигом,
    после align_face обязаны лечь обратно в канонические позиции шаблона
    — маркеры-цвета, нарисованные в исходном кадре в этих точках, должны
    оказаться в выровненном 112x112 РОВНО там, где их ожидает канон.
    """
    pytest.importorskip('cv2')
    import cv2  # noqa: E402 (importorskip выше гарантирует наличие)

    template = np.asarray(face_mod._ARCFACE_TEMPLATE_112, dtype=np.float64)
    # Известный transform: поворот 15°, масштаб 1.8x, сдвиг (60, 40) —
    # заведомо НЕ идентичность, чтобы тест не проходил случайно.
    src_pts = _rotate_scale_translate(
        template, angle_deg=15.0, scale=1.8, tx=60.0, ty=40.0
    )

    frame_w, frame_h = 400, 400
    frame = np.zeros((frame_h, frame_w, 3), dtype=np.uint8)
    colors = [
        (255, 0, 0),
        (0, 255, 0),
        (0, 0, 255),
        (255, 255, 0),
        (0, 255, 255),
    ]
    for (x, y), color in zip(src_pts, colors):
        cv2.circle(
            frame, (int(round(x)), int(round(y))), radius=6,
            color=color, thickness=-1,
        )

    landmarks: list = []
    for x, y in src_pts:
        landmarks.append(x / frame_w)
        landmarks.append(y / frame_h)

    aligned = face_mod.align_face(frame, landmarks, output_size=112)

    assert aligned is not None
    assert aligned.shape == (112, 112, 3)
    assert aligned.dtype == np.uint8

    for (tx, ty), color in zip(template, colors):
        xi, yi = int(round(tx)), int(round(ty))
        patch = aligned[max(0, yi - 2):yi + 3, max(0, xi - 2):xi + 3]
        mean_color = patch.reshape(-1, 3).mean(axis=0)
        assert np.allclose(mean_color, color, atol=40), (
            f'канонический маркер {color} не найден у шаблонной точки '
            f'({tx}, {ty}): получено {mean_color}'
        )


def test_align_face_output_size_scaling():
    """output_size != 112 -> шаблон масштабируется пропорционально, а не
    остаётся зафиксированным на 112x112 (иначе выход был бы обрезан)."""
    pytest.importorskip('cv2')
    output_size = 224
    scale = output_size / face_mod.ARCFACE_INPUT_SIZE
    template = np.asarray(face_mod._ARCFACE_TEMPLATE_112, dtype=np.float64) * scale

    frame = np.zeros((output_size, output_size, 3), dtype=np.uint8)
    landmarks: list = []
    for x, y in template:
        landmarks.append(x / output_size)
        landmarks.append(y / output_size)

    aligned = face_mod.align_face(frame, landmarks, output_size=output_size)
    assert aligned is not None
    assert aligned.shape == (output_size, output_size, 3)


def test_align_face_none_when_landmarks_none():
    frame = np.zeros((200, 200, 3), dtype=np.uint8)
    assert face_mod.align_face(frame, None) is None


def test_align_face_none_when_landmarks_wrong_length():
    frame = np.zeros((200, 200, 3), dtype=np.uint8)
    assert face_mod.align_face(frame, [0.1, 0.2, 0.3]) is None
    assert face_mod.align_face(frame, []) is None


def test_align_face_none_when_frame_none():
    landmarks = [0.1] * 10
    assert face_mod.align_face(None, landmarks) is None


def test_align_face_none_on_non_finite_landmarks():
    pytest.importorskip('cv2')
    frame = np.zeros((200, 200, 3), dtype=np.uint8)
    landmarks = [0.1] * 10
    landmarks[0] = float('nan')
    assert face_mod.align_face(frame, landmarks) is None

    landmarks_inf = [0.1] * 10
    landmarks_inf[3] = float('inf')
    assert face_mod.align_face(frame, landmarks_inf) is None


def test_align_face_none_on_degenerate_points():
    """5 совпадающих точек -> вырожденная геометрия: дисперсия src ~ 0,
    similarity-transform (Umeyama) математически не определён (масштаб
    считается делением на дисперсию src) -> align_face обязан вернуть
    None, а не упасть и не подставить произвольный transform.
    """
    pytest.importorskip('cv2')
    frame = np.zeros((200, 200, 3), dtype=np.uint8)
    landmarks = [0.5, 0.5] * 5
    assert face_mod.align_face(frame, landmarks) is None


def test_align_face_is_deterministic_across_repeated_calls():
    """Один и тот же вход дважды -> побитово одинаковый результат.

    Ловит именно регрессию на RANSAC (``cv2.estimateAffinePartial2D``
    без явного ``method``): RANSAC сэмплит случайные подмножества из 5
    точек и может отдать разный transform от вызова к вызову для ОДНОГО
    И ТОГО ЖЕ входа, что в системе узнавания означает разный эмбеддинг
    для одного и того же кадра. ``_similarity_transform_umeyama`` —
    детерминированная closed-form формула, поэтому результат обязан
    совпадать побитово при повторном вызове с теми же аргументами.
    """
    pytest.importorskip('cv2')
    template = np.asarray(face_mod._ARCFACE_TEMPLATE_112, dtype=np.float64)
    src_pts = _rotate_scale_translate(
        template, angle_deg=-27.0, scale=0.9, tx=15.0, ty=-8.0
    )
    frame_w, frame_h = 300, 300
    frame = np.random.RandomState(7).randint(
        0, 255, size=(frame_h, frame_w, 3)
    ).astype(np.uint8)

    landmarks: list = []
    for x, y in src_pts:
        landmarks.append(x / frame_w)
        landmarks.append(y / frame_h)

    result_a = face_mod.align_face(frame, landmarks, output_size=112)
    result_b = face_mod.align_face(frame, landmarks, output_size=112)

    assert result_a is not None
    assert result_b is not None
    assert np.array_equal(result_a, result_b), (
        'align_face обязан быть детерминированным — побитово одинаковый '
        'результат на одинаковом входе (issue #2773: RANSAC-регрессия)'
    )


def test_align_face_none_without_cv2(monkeypatch):
    """cv2 недоступен -> None, а не исключение (модуль обязан работать
    без cv2 — см. модульный docstring про ленивый импорт)."""
    import builtins

    real_import = builtins.__import__

    def _fake_import(name, *args, **kwargs):
        if name == 'cv2':
            raise ImportError('cv2 недоступен (тест)')
        return real_import(name, *args, **kwargs)

    monkeypatch.setattr(builtins, '__import__', _fake_import)

    frame = np.zeros((200, 200, 3), dtype=np.uint8)
    landmarks = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 0.05]
    assert face_mod.align_face(frame, landmarks) is None
