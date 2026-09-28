"""Unit-тесты observation_geometry (ADR-0138, этап 1 ADR-0130).

Что покрываем (pure-Python + numpy, без rclpy/cv2):
  1. estimate_position: bbox + глубина + K → точка в optical frame.
  2. Честные статусы: нет глубины / нет K / все нули / бокс вне кадра /
     статус шва «Взгляд» пробрасывается как есть.
  3. Устойчивость: выбросы (фон в боксе) не сдвигают медиану.
  4. build_observation: нормированный bbox → пиксели, только person/face,
     без имени (ADR-0130 §5.1 п.1), candidate_* без собственного id.

Запуск:
    python -m pytest src/rob_box_perception/test/unit/test_observation_geometry.py -q --no-cov
"""

from __future__ import annotations

import importlib
import json
import math
import sys
from pathlib import Path

import pytest

np = pytest.importorskip('numpy')

_HERE = Path(__file__).resolve()
_PKG_ROOT = _HERE.parents[2]
if str(_PKG_ROOT) not in sys.path:
    sys.path.insert(0, str(_PKG_ROOT))

geo = importlib.import_module('rob_box_perception.observation_geometry')

# OAK-D Lite RGB 1280x720 — порядок величин, не калибровка робота.
W, H = 1280, 720
K = (1000.0, 1000.0, 640.0, 360.0)


def _depth(value_mm: int = 0) -> 'np.ndarray':
    return np.full((H, W), value_mm, dtype=np.uint16)


# ============================================================================
# estimate_position
# ============================================================================

def test_center_box_on_optical_axis_gives_pure_z():
    depth = _depth(2000)
    est = geo.estimate_position(depth, K, (640.0, 360.0, 200.0, 400.0))
    assert est.valid is True
    assert est.status == geo.STATUS_OK
    x, y, z = est.position
    assert x == pytest.approx(0.0, abs=1e-9)
    assert y == pytest.approx(0.0, abs=1e-9)
    assert z == pytest.approx(2.0)
    assert est.distance_m == pytest.approx(2.0)
    assert est.stddev_m == pytest.approx(0.0, abs=1e-9)


def test_off_axis_box_uses_intrinsics():
    """x = (u−cx)·Z/fx, y = (v−cy)·Z/fy; distance = |p|."""
    depth = _depth(3000)
    est = geo.estimate_position(depth, K, (940.0, 160.0, 100.0, 100.0))
    x, y, z = est.position
    assert z == pytest.approx(3.0)
    assert x == pytest.approx((940.0 - 640.0) * 3.0 / 1000.0)  # 0.9
    assert y == pytest.approx((160.0 - 360.0) * 3.0 / 1000.0)  # -0.6
    assert est.distance_m == pytest.approx(math.sqrt(0.9 ** 2 + 0.6 ** 2 + 9.0))


def test_background_outliers_do_not_move_median():
    """Человек 1.5 м, в ядре бокса 30 % фона на 6 м — медиана = человек."""
    depth = _depth(1500)
    # Ядро бокса (центральные 50 %): x 590..690, y 260..460.
    depth[260:460, 590:620] = 6000
    est = geo.estimate_position(depth, K, (640.0, 360.0, 200.0, 400.0))
    assert est.valid is True
    assert est.position[2] == pytest.approx(1.5)
    # Фон отсечён как выброс — разброс инлаеров нулевой.
    assert est.stddev_m == pytest.approx(0.0, abs=1e-9)


def test_stddev_reflects_depth_spread():
    depth = _depth(2000)
    depth[260:460, 590:640] = 2100  # половина ядра на 10 см дальше
    est = geo.estimate_position(depth, K, (640.0, 360.0, 200.0, 400.0))
    assert est.valid is True
    assert 0.03 < est.stddev_m < 0.07


def test_all_zero_depth_in_box_is_depth_invalid():
    est = geo.estimate_position(_depth(0), K, (640.0, 360.0, 200.0, 400.0))
    assert est.valid is False
    assert est.status == geo.STATUS_DEPTH_INVALID
    assert all(math.isnan(v) for v in est.position)
    assert est.distance_m == -1.0
    assert est.stddev_m == -1.0


def test_sparse_valid_pixels_below_fraction_is_depth_invalid():
    depth = _depth(0)
    depth[360, 640] = 2000  # один валидный пиксель на 20000 в ядре
    est = geo.estimate_position(depth, K, (640.0, 360.0, 200.0, 400.0))
    assert est.status == geo.STATUS_DEPTH_INVALID


def test_no_depth_is_no_depth_stream():
    est = geo.estimate_position(None, K, (640.0, 360.0, 200.0, 400.0))
    assert est.valid is False
    assert est.status == geo.STATUS_NO_DEPTH_STREAM
    assert est.distance_m == -1.0


def test_no_intrinsics_is_no_camera_info():
    est = geo.estimate_position(_depth(2000), None, (640.0, 360.0, 10.0, 10.0))
    assert est.status == geo.STATUS_NO_CAMERA_INFO
    zero_k = geo.estimate_position(
        _depth(2000), (0.0, 0.0, 640.0, 360.0), (640.0, 360.0, 10.0, 10.0)
    )
    assert zero_k.status == geo.STATUS_NO_CAMERA_INFO


def test_upstream_status_is_passed_through():
    """Статус шва «Взгляд» (рассинхрон) не подменяется «ok» или нулями."""
    est = geo.estimate_position(
        _depth(2000), K, (640.0, 360.0, 200.0, 400.0),
        upstream_status=geo.STATUS_DEPTH_OUT_OF_SYNC,
    )
    assert est.valid is False
    assert est.status == geo.STATUS_DEPTH_OUT_OF_SYNC


@pytest.mark.parametrize('bbox', [
    (5000.0, 360.0, 100.0, 100.0),   # целиком правее кадра
    (640.0, 360.0, 0.0, 100.0),      # нулевая ширина
    (640.0, 360.0, -1.0, -1.0),      # «нет бокса» из VisionEvent
    (math.nan, 360.0, 10.0, 10.0),
])
def test_bbox_outside_frame(bbox):
    est = geo.estimate_position(_depth(2000), K, bbox)
    assert est.status == geo.STATUS_BBOX_OUTSIDE_FRAME
    assert est.valid is False


def test_out_of_range_depth_is_ignored():
    depth = _depth(30000)  # 30 м — за пределами max_depth_m
    est = geo.estimate_position(depth, K, (640.0, 360.0, 200.0, 400.0))
    assert est.status == geo.STATUS_DEPTH_INVALID


# ============================================================================
# build_observation
# ============================================================================

def _person_event(**overrides):
    ev = {
        'source_camera': 'camera_color_optical_frame',
        'event_type': 'person',
        'class_name': 'person',
        'class_id': 0,
        'confidence': 0.87,
        'bbox_cx': 0.5,
        'bbox_cy': 0.5,
        'bbox_w': 0.25,
        'bbox_h': 0.5,
        'distance_m': -1.0,
        'embedding_id': '',
        'display_name': '',
        'attributes_json': '',
    }
    ev.update(overrides)
    return ev


def test_build_observation_person_with_depth():
    obs = geo.build_observation(
        _person_event(), source='oak_d', image_width=W, image_height=H,
        depth_mm=_depth(2500), intrinsics=K, depth_status=geo.STATUS_OK,
    )
    assert set(obs) == set(geo.OBSERVATION_FIELDS)
    assert obs['source'] == 'oak_d'
    assert obs['class_name'] == 'person'
    assert obs['confidence'] == pytest.approx(0.87)
    assert obs['bbox_cx_px'] == pytest.approx(640.0)
    assert obs['bbox_cy_px'] == pytest.approx(360.0)
    assert obs['bbox_w_px'] == pytest.approx(320.0)
    assert obs['bbox_h_px'] == pytest.approx(360.0)
    assert (obs['image_width'], obs['image_height']) == (W, H)
    assert obs['position_valid'] is True
    assert obs['position_status'] == 'ok'
    assert obs['position'][2] == pytest.approx(2.5)
    assert obs['distance_m'] == pytest.approx(2.5)
    assert obs['candidate_id'] == ''
    assert obs['candidate_similarity'] == -1.0


def test_build_observation_without_depth_is_honest():
    obs = geo.build_observation(
        _person_event(), source='ceiling_camera', image_width=W, image_height=H,
    )
    assert obs['position_valid'] is False
    assert obs['position_status'] == geo.STATUS_NO_DEPTH_STREAM
    assert obs['distance_m'] == -1.0
    assert all(math.isnan(v) for v in obs['position'])


@pytest.mark.parametrize('class_name', ['cup', 'chair', '', 'stub'])
def test_build_observation_skips_other_classes(class_name):
    """ADR-0130 §2.2: профили кроме person не пишутся."""
    ev = _person_event(class_name=class_name, event_type='object')
    assert geo.build_observation(
        ev, source='oak_d', image_width=W, image_height=H,
    ) is None


def test_build_observation_requires_frame_size():
    assert geo.build_observation(
        _person_event(), source='oak_d', image_width=0, image_height=0,
    ) is None


def test_face_observation_carries_candidate_without_name():
    """Инвариант ADR-0130 §5.1 п.1/п.5: имени нет, id — ссылка на улику."""
    ev = _person_event(
        event_type='face', class_name='face',
        embedding_id='4ff0ddc5-aaaa', display_name='Дэнчик',
        attributes_json=json.dumps({
            'encounter': 'start', 'person_id': '4ff0ddc5-aaaa',
            'name': 'Дэнчик', 'similarity': 0.9291,
        }, ensure_ascii=False),
    )
    obs = geo.build_observation(ev, source='oak_d', image_width=W, image_height=H)
    assert obs['class_name'] == 'face'
    assert obs['candidate_id'] == '4ff0ddc5-aaaa'
    assert obs['candidate_similarity'] == pytest.approx(0.9291)
    assert 'Дэнчик' not in json.dumps(obs, ensure_ascii=False, default=str)
    assert not any('name' in key and key != 'class_name' for key in obs)


def test_face_candidate_similarity_unknown_without_encounter_marker():
    ev = _person_event(
        event_type='face', class_name='face', embedding_id='abc',
        display_name='Маша',
    )
    obs = geo.build_observation(ev, source='oak_d', image_width=W, image_height=H)
    assert obs['candidate_id'] == 'abc'
    assert obs['candidate_similarity'] == -1.0


def test_face_candidate_similarity_ignores_foreign_marker():
    """Маркер о другой записи не приписывает её сходство этому кандидату."""
    ev = _person_event(
        event_type='face', class_name='face', embedding_id='abc',
        attributes_json=json.dumps({'person_id': 'zzz', 'similarity': 0.9}),
    )
    obs = geo.build_observation(ev, source='oak_d', image_width=W, image_height=H)
    assert obs['candidate_similarity'] == -1.0


def test_observation_fields_have_no_identity_name():
    """Python-сторона контракта не несёт имени (ADR-0130 §5.1 п.1)."""
    forbidden = {'display_name', 'name', 'person_name', 'acquaintance_name'}
    assert not forbidden & set(geo.OBSERVATION_FIELDS)
