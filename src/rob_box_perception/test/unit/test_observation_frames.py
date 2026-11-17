"""Точка наблюдения в base_link / map через tf2 (ADR-0130 §7 этап 1, ADR-0138 поправка).

Контракт observation_frames:
  1. Математика трансформа сверена с ЖИВЫМ трансформом робота
     (tf2_echo base_link camera_color_optical_frame, 03.11.2026):
     2 м перед камерой → 1.884 м вперёд от base_link на высоте 0.46 м.
  2. Каждый фрейм — свой честный статус: нет map → ``no_transform`` для
     map, base_link при этом ``ok``; невалидная глубина → ``no_position``;
     выключено → ``tf_disabled``. Точка NaN, когда статус не ``ok``.
  3. Трансформ ищется на stamp КАДРА; не дождались — ``transform_stale``,
     latest не подставляется.
  4. Детекции одного кадра ищут TF один раз на фрейм (кэш по stamp).
  5. Адаптер Tf2Lookup над фейковым tf-буфером: нет цепочки — сразу
     ``NoTransform`` без ожидания; исключения tf2 → статусы модуля.
  6. Словарь статусов описан в Observation.msg.

Запуск:
    python -m pytest src/rob_box_perception/test/unit/test_observation_frames.py -v --no-cov
"""

from __future__ import annotations

import importlib
import math
import sys
import types
from pathlib import Path
from typing import Any, List, Tuple

import pytest

_HERE = Path(__file__).resolve()
_PKG_ROOT = _HERE.parents[2]
if str(_PKG_ROOT) not in sys.path:
    sys.path.insert(0, str(_PKG_ROOT))

fr = importlib.import_module('rob_box_perception.observation_frames')

_OBSERVATION_MSG = _HERE.parents[3] / 'rob_box_perception_msgs' / 'msg' / 'Observation.msg'

#: Живой трансформ base_link ← camera_color_optical_frame (Vision Pi,
#: 03.11.2026): Translation [-0.116, -0.000, 0.460],
#: Quaternion (xyzw) [-0.500, 0.500, -0.500, 0.500].
CAMERA = fr.RigidTransform(
    translation=(-0.116, 0.0, 0.46), rotation=(-0.5, 0.5, -0.5, 0.5),
)
MAP_FROM_CAMERA = fr.RigidTransform(translation=(10.0, 5.0, 0.0), rotation=(0.0, 0.0, 0.0, 1.0))


def _approx(a, b, tol=1e-9):
    return all(abs(x - y) <= tol for x, y in zip(a, b))


def _is_nan_point(p):
    return len(p) == 3 and all(math.isnan(v) for v in p)


# ---------- RigidTransform ------------------------------------------------

def test_optical_forward_is_base_link_forward():
    assert _approx(CAMERA.apply((0.0, 0.0, 2.0)), (1.884, 0.0, 0.46))


def test_optical_right_is_base_link_minus_y_and_down_is_minus_z():
    # REP-103 optical: x вправо, y вниз; base_link: y влево, z вверх.
    assert _approx(CAMERA.apply((1.0, 0.0, 0.0)), (-0.116, -1.0, 0.46))
    assert _approx(CAMERA.apply((0.0, 1.0, 0.0)), (-0.116, 0.0, -0.54))


def test_unnormalized_quaternion_is_normalized():
    t = fr.RigidTransform(translation=(0.0, 0.0, 0.0), rotation=(-1.0, 1.0, -1.0, 1.0))
    assert _approx(t.apply((0.0, 0.0, 2.0)), (2.0, 0.0, 0.0))


def test_zero_quaternion_fails_loud():
    with pytest.raises(ValueError):
        fr.RigidTransform((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 0.0)).apply((1.0, 0.0, 0.0))


# ---------- FrameResolver -------------------------------------------------

class _FakeLookup:
    """Дерево TF: {target: RigidTransform | Exception-класс}."""

    def __init__(self, tree):
        self.tree = tree
        self.calls: List[Tuple[str, str, float, float]] = []

    def __call__(self, target, source, stamp, timeout):
        self.calls.append((target, source, stamp, timeout))
        value = self.tree[target]
        if isinstance(value, type) and issubclass(value, Exception):
            raise value(f'{target} <- {source}')
        return value


SRC = 'camera_color_optical_frame'


def test_both_frames_ok():
    lookup = _FakeLookup({'base_link': CAMERA, 'map': MAP_FROM_CAMERA})
    out = fr.FrameResolver(lookup).resolve(SRC, 100.5, (0.0, 0.0, 2.0), True)
    assert set(out) == set(fr.FRAME_FIELDS)
    assert out['base_link_status'] == fr.FRAME_STATUS_OK
    assert _approx(out['position_base_link'], (1.884, 0.0, 0.46))
    assert out['map_status'] == fr.FRAME_STATUS_OK
    assert _approx(out['position_map'], (10.0, 5.0, 2.0))


def test_no_map_is_honest_and_base_link_still_ok():
    lookup = _FakeLookup({'base_link': CAMERA, 'map': fr.NoTransform})
    out = fr.FrameResolver(lookup).resolve(SRC, 100.5, (0.0, 0.0, 2.0), True)
    assert out['base_link_status'] == fr.FRAME_STATUS_OK
    assert out['map_status'] == fr.FRAME_STATUS_NO_TRANSFORM
    assert _is_nan_point(out['position_map'])


def test_stale_tf_is_not_replaced_by_latest():
    lookup = _FakeLookup({'base_link': CAMERA, 'map': fr.TransformStale})
    out = fr.FrameResolver(lookup).resolve(SRC, 100.5, (0.0, 0.0, 2.0), True)
    assert out['map_status'] == fr.FRAME_STATUS_TRANSFORM_STALE
    assert _is_nan_point(out['position_map'])


def test_lookup_uses_frame_stamp_and_timeout():
    lookup = _FakeLookup({'base_link': CAMERA, 'map': MAP_FROM_CAMERA})
    fr.FrameResolver(lookup, timeout_sec=0.07).resolve(SRC, 123.25, (0.0, 0.0, 1.0), True)
    assert ('base_link', SRC, 123.25, 0.07) in lookup.calls
    assert ('map', SRC, 123.25, 0.07) in lookup.calls


def test_invalid_position_is_not_transformed():
    lookup = _FakeLookup({'base_link': CAMERA, 'map': MAP_FROM_CAMERA})
    nan = (math.nan, math.nan, math.nan)
    out = fr.FrameResolver(lookup).resolve(SRC, 1.0, nan, False)
    assert out['base_link_status'] == fr.FRAME_STATUS_NO_POSITION
    assert out['map_status'] == fr.FRAME_STATUS_NO_POSITION
    assert _is_nan_point(out['position_base_link'])
    assert lookup.calls == []


def test_disabled_resolver():
    resolver = fr.FrameResolver(None)
    assert not resolver.enabled
    out = resolver.resolve(SRC, 1.0, (0.0, 0.0, 2.0), True)
    assert out['base_link_status'] == fr.FRAME_STATUS_TF_DISABLED
    assert out['map_status'] == fr.FRAME_STATUS_TF_DISABLED
    assert _is_nan_point(out['position_base_link'])
    assert _is_nan_point(out['position_map'])


def test_one_lookup_per_frame_and_target():
    lookup = _FakeLookup({'base_link': CAMERA, 'map': fr.NoTransform})
    resolver = fr.FrameResolver(lookup)
    for _ in range(3):  # три детекции одного кадра
        resolver.resolve(SRC, 10.0, (0.0, 0.0, 2.0), True)
    assert len(lookup.calls) == 2
    resolver.resolve(SRC, 10.1, (0.0, 0.0, 2.0), True)  # следующий кадр
    assert len(lookup.calls) == 4


def test_negative_timeout_clamped_to_zero():
    lookup = _FakeLookup({'base_link': CAMERA, 'map': MAP_FROM_CAMERA})
    fr.FrameResolver(lookup, timeout_sec=-1.0).resolve(SRC, 1.0, (0.0, 0.0, 1.0), True)
    assert all(call[3] == 0.0 for call in lookup.calls)


# ---------- Tf2Lookup над фейковым tf-буфером ------------------------------

class _LookupExc(Exception):
    pass


class _ConnectivityExc(Exception):
    pass


class _ExtrapolationExc(Exception):
    pass


def _stamped(translation, rotation):
    t = types.SimpleNamespace(x=translation[0], y=translation[1], z=translation[2])
    r = types.SimpleNamespace(x=rotation[0], y=rotation[1], z=rotation[2], w=rotation[3])
    return types.SimpleNamespace(transform=types.SimpleNamespace(translation=t, rotation=r))


class _FakeBuffer:
    """Дублёр tf2_ros.Buffer: can_transform / lookup_transform."""

    def __init__(self, frames, lookup_raises=None):
        self.frames = frames  # {(target, source): (translation, rotation)}
        self.lookup_raises = lookup_raises
        self.can_calls: List[Any] = []
        self.lookup_calls: List[Any] = []

    def can_transform(self, target, source, time, timeout):
        self.can_calls.append((target, source, time, timeout))
        return (target, source) in self.frames

    def lookup_transform(self, target, source, time, timeout):
        self.lookup_calls.append((target, source, time, timeout))
        if self.lookup_raises is not None:
            raise self.lookup_raises('boom')
        return _stamped(*self.frames[(target, source)])


def _adapter(buffer):
    return fr.Tf2Lookup(
        buffer,
        exceptions=(_LookupExc, _ConnectivityExc, _ExtrapolationExc),
        to_time=lambda s: ('time', s),
        to_duration=lambda s: ('dur', s),
    )


def test_tf2_lookup_converts_transform_at_frame_stamp():
    buf = _FakeBuffer({('base_link', SRC): ((-0.116, 0.0, 0.46), (-0.5, 0.5, -0.5, 0.5))})
    t = _adapter(buf)('base_link', SRC, 42.5, 0.05)
    assert t == CAMERA
    # проверка цепочки — на latest без ожидания, поиск — на stamp кадра с таймаутом
    assert buf.can_calls == [('base_link', SRC, ('time', 0.0), ('dur', 0.0))]
    assert buf.lookup_calls == [('base_link', SRC, ('time', 42.5), ('dur', 0.05))]


def test_tf2_lookup_missing_chain_does_not_wait():
    buf = _FakeBuffer({})
    with pytest.raises(fr.NoTransform):
        _adapter(buf)('map', SRC, 42.5, 0.05)
    assert buf.lookup_calls == []


@pytest.mark.parametrize('exc, expected', [
    (_ExtrapolationExc, fr.TransformStale),
    (_LookupExc, fr.NoTransform),
    (_ConnectivityExc, fr.NoTransform),
])
def test_tf2_exceptions_map_to_module_errors(exc, expected):
    buf = _FakeBuffer({('map', SRC): ((0, 0, 0), (0, 0, 0, 1))}, lookup_raises=exc)
    with pytest.raises(expected):
        _adapter(buf)('map', SRC, 42.5, 0.05)


def test_resolver_over_fake_buffer_end_to_end():
    buf = _FakeBuffer({('base_link', SRC): ((-0.116, 0.0, 0.46), (-0.5, 0.5, -0.5, 0.5))})
    out = fr.FrameResolver(_adapter(buf)).resolve(SRC, 7.0, (0.0, 0.0, 2.0), True)
    assert out['base_link_status'] == fr.FRAME_STATUS_OK
    assert _approx(out['position_base_link'], (1.884, 0.0, 0.46))
    assert out['map_status'] == fr.FRAME_STATUS_NO_TRANSFORM


# ---------- словарь статусов в IDL ----------------------------------------

def test_every_status_is_documented_in_msg():
    text = _OBSERVATION_MSG.read_text(encoding='utf-8')
    for status in fr.FRAME_STATUSES:
        assert status in text, f'статус {status!r} не описан в Observation.msg'
