"""rob_box_core.tf_lookup: общий адаптер tf2 восприятия (#4304) и позы на карте (#4390).

Режим кадра (``__call__``) покрыт в ``rob_box_perception/test/unit/test_observation_frames.py``
через реэкспорт. Здесь — то, что добавил #4390: ``latest`` ждёт данных у пустого буфера
(без отказа по ``can_transform``) и ``yaw`` трансформа.
"""

import math
import types

import pytest

from rob_box_core import tf_lookup as tl


class _LookupExc(Exception):
    pass


class _ConnectivityExc(Exception):
    pass


class _ExtrapolationExc(Exception):
    pass


class _FreshBuffer:
    """Только что созданный буфер: ``can_transform`` ещё False, ``lookup_transform`` ждёт и находит."""

    def __init__(self, stamped=None, raises=None):
        self.stamped, self.raises = stamped, raises
        self.can_calls, self.lookup_calls = [], []

    def can_transform(self, *args):
        self.can_calls.append(args)
        return False

    def lookup_transform(self, target, source, time, timeout):
        self.lookup_calls.append((target, source, time, timeout))
        if self.raises is not None:
            raise self.raises("boom")
        return self.stamped


def _stamped(t, r):
    return types.SimpleNamespace(transform=types.SimpleNamespace(
        translation=types.SimpleNamespace(x=t[0], y=t[1], z=t[2]),
        rotation=types.SimpleNamespace(x=r[0], y=r[1], z=r[2], w=r[3]),
    ))


def _adapter(buffer):
    return tl.Tf2Lookup(
        buffer,
        exceptions=(_LookupExc, _ConnectivityExc, _ExtrapolationExc),
        to_time=lambda s: ("time", s),
        to_duration=lambda s: ("dur", s),
    )


def test_latest_waits_on_fresh_buffer_instead_of_refusing():
    yaw = math.radians(-30.0)
    buf = _FreshBuffer(_stamped((3.0, -1.0, 0.0), (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))))
    t = _adapter(buf).latest("map", "base_footprint", 1.5)
    assert t.translation == (3.0, -1.0, 0.0)
    assert t.yaw == pytest.approx(yaw)
    assert buf.can_calls == []
    assert buf.lookup_calls == [("map", "base_footprint", ("time", 0.0), ("dur", 1.5))]


@pytest.mark.parametrize("exc, expected", [
    (_ExtrapolationExc, tl.TransformStale),
    (_LookupExc, tl.NoTransform),
    (_ConnectivityExc, tl.NoTransform),
])
def test_latest_maps_tf2_errors(exc, expected):
    with pytest.raises(expected):
        _adapter(_FreshBuffer(raises=exc)).latest("map", "base_footprint", 1.5)


def test_frame_mode_still_refuses_missing_chain_without_waiting():
    buf = _FreshBuffer()
    with pytest.raises(tl.NoTransform):
        _adapter(buf)("map", "camera", 42.0, 0.05)
    assert buf.lookup_calls == []


@pytest.mark.parametrize("deg", [0.0, 90.0, 179.0, -179.0, -90.0])
def test_yaw_of_pure_z_rotation(deg):
    a = math.radians(deg)
    t = tl.RigidTransform((0.0, 0.0, 0.0), (0.0, 0.0, 2.0 * math.sin(a / 2), 2.0 * math.cos(a / 2)))  # ненормирован
    assert t.yaw == pytest.approx(a)


def test_perception_reexports_the_same_objects():
    """Второй реализации нет: observation_frames отдаёт классы rob_box_core."""
    fr = pytest.importorskip("rob_box_perception.observation_frames")
    assert fr.Tf2Lookup is tl.Tf2Lookup and fr.RigidTransform is tl.RigidTransform
    assert fr.NoTransform is tl.NoTransform and fr.TransformStale is tl.TransformStale
