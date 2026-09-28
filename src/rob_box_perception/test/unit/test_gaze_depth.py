"""Глубина через шов «Взгляд» (gaze.py, ADR-0138, этап 1 ADR-0130).

Что покрываем (без rclpy и без cv2):
  1. Frame: поля глубины опциональны, по умолчанию — честное «нет глубины».
  2. select_nearest_by_stamp: ближайший в допуске / вне допуска / пусто.
  3. _decode_depth_msg: 16UC1 с padding строки, big-endian, чужая кодировка.
  4. _intrinsics_from_camera_info: K → (fx, fy, cx, cy, w, h), нулевая K.
  5. OakDSource со стаб-нодой и стаб-``sensor_msgs``: подписки на depth +
     camera_info, связывание по stamp (в т.ч. depth пришёл ПОЗЖЕ RGB),
     рассинхрон, несовпадение размеров, нет camera_info, depth выключен.
  6. CeilingCameraSource/StubSource: глубины нет.

Реальная подписка ROS и реальные stamp OAK-D — не проверяются здесь
(нужен робот; см. ADR-0138 «Что не проверено»).

Запуск:
    python -m pytest src/rob_box_perception/test/unit/test_gaze_depth.py -q --no-cov
"""

from __future__ import annotations

import importlib
import sys
import types
from pathlib import Path

import pytest

np = pytest.importorskip('numpy')

_HERE = Path(__file__).resolve()
_PKG_ROOT = _HERE.parents[2]
if str(_PKG_ROOT) not in sys.path:
    sys.path.insert(0, str(_PKG_ROOT))

gaze = importlib.import_module('rob_box_perception.gaze')
geo = importlib.import_module('rob_box_perception.observation_geometry')

W, H = 64, 36


# ---------- fakes -----------------------------------------------------------

class _Stamp:
    def __init__(self, t: float) -> None:
        self.sec = int(t)
        self.nanosec = int(round((t - int(t)) * 1e9))


class _Header:
    def __init__(self, t: float, frame_id: str = 'camera_color_optical_frame'):
        self.stamp = _Stamp(t)
        self.frame_id = frame_id


def _depth_msg(t: float, value_mm: int = 1500, w: int = W, h: int = H,
               encoding: str = '16UC1', pad: int = 0, bigendian: bool = False):
    row = np.full((h, w + pad), value_mm, dtype='>u2' if bigendian else '<u2')
    return types.SimpleNamespace(
        header=_Header(t), height=h, width=w, encoding=encoding,
        is_bigendian=int(bigendian), step=(w + pad) * 2, data=row.tobytes(),
    )


def _info_msg(w: int = W, h: int = H, fx: float = 50.0):
    return types.SimpleNamespace(
        header=_Header(0.0), width=w, height=h,
        k=[fx, 0.0, w / 2.0, 0.0, fx, h / 2.0, 0.0, 0.0, 1.0],
    )


class _Logger:
    def info(self, *_a, **_k):
        pass

    def warning(self, *_a, **_k):
        pass


class _FakeNode:
    def __init__(self) -> None:
        self.subs = {}
        self.destroyed = []

    def create_subscription(self, msg_type, topic, cb, qos):
        self.subs[topic] = (msg_type, cb)
        return topic

    def destroy_subscription(self, sub):
        self.destroyed.append(sub)

    def get_logger(self):
        return _Logger()


@pytest.fixture
def sensor_msgs_stub(monkeypatch):
    pkg = types.ModuleType('sensor_msgs')
    msg = types.ModuleType('sensor_msgs.msg')
    msg.Image = type('Image', (), {})
    msg.CameraInfo = type('CameraInfo', (), {})
    msg.CompressedImage = type('CompressedImage', (), {})
    pkg.msg = msg
    monkeypatch.setitem(sys.modules, 'sensor_msgs', pkg)
    monkeypatch.setitem(sys.modules, 'sensor_msgs.msg', msg)
    return msg


def _frame(t: float, w: int = W, h: int = H):
    return gaze.Frame(
        rgb=np.zeros((h, w, 3), dtype=np.uint8), scale=1.0, pad_left=0,
        pad_top=0, original_w=w, original_h=h,
        frame_id='camera_color_optical_frame', stamp=t, source_name='oak_d',
    )


def _oak(sensor_msgs_stub, **kwargs):
    node = _FakeNode()
    return gaze.OakDSource(node, **kwargs), node


# ============================================================================
# Frame / helpers
# ============================================================================

def test_frame_depth_fields_default_to_honest_absence():
    frame = gaze.Frame(
        rgb=None, scale=1.0, pad_left=0, pad_top=0, original_w=0,
        original_h=0, frame_id='', stamp=0.0, source_name='test',
    )
    assert frame.depth_mm is None
    assert frame.intrinsics is None
    assert frame.depth_status == geo.STATUS_NO_DEPTH_STREAM


def test_select_nearest_by_stamp():
    items = [(10.0, 'a'), (10.2, 'b'), (10.4, 'c')]
    item, dt = gaze.select_nearest_by_stamp(items, 10.25, 0.15)
    assert item == 'b'
    assert dt == pytest.approx(0.05)
    item, dt = gaze.select_nearest_by_stamp(items, 11.0, 0.15)
    assert item is None
    assert dt == pytest.approx(0.6)
    assert gaze.select_nearest_by_stamp([], 1.0, 0.15) == (None, None)


def test_decode_depth_handles_row_padding_and_endianness():
    depth, status = gaze._decode_depth_msg(_depth_msg(0.0, 1234, pad=3))
    assert status == geo.STATUS_OK
    assert depth.shape == (H, W)
    assert depth.dtype == np.uint16
    assert int(depth[5, 5]) == 1234
    depth_be, status_be = gaze._decode_depth_msg(
        _depth_msg(0.0, 1234, bigendian=True)
    )
    assert status_be == geo.STATUS_OK
    assert int(depth_be[0, 0]) == 1234


def test_decode_depth_rejects_foreign_encoding():
    depth, status = gaze._decode_depth_msg(_depth_msg(0.0, encoding='32FC1'))
    assert depth is None
    assert status == geo.STATUS_DEPTH_ENCODING_UNSUPPORTED


def test_intrinsics_from_camera_info():
    assert gaze._intrinsics_from_camera_info(_info_msg()) == (
        50.0, 50.0, W / 2.0, H / 2.0, W, H,
    )
    assert gaze._intrinsics_from_camera_info(_info_msg(fx=0.0)) is None


# ============================================================================
# OakDSource
# ============================================================================

def test_oak_subscribes_depth_and_camera_info(sensor_msgs_stub):
    src, node = _oak(sensor_msgs_stub)
    assert set(node.subs) == {
        '/camera/camera/color/image_raw',
        '/camera/camera/depth/image_rect_raw',
        '/camera/camera/color/camera_info',
    }
    src.stop()
    assert set(node.destroyed) == set(node.subs)


def test_oak_depth_disabled_subscribes_only_rgb(sensor_msgs_stub):
    src, node = _oak(sensor_msgs_stub, depth_enabled=False)
    assert set(node.subs) == {'/camera/camera/color/image_raw'}
    out = src._attach_depth(_frame(1.0))
    assert out.depth_status == geo.STATUS_NO_DEPTH_STREAM
    assert out.depth_mm is None


def test_oak_attaches_nearest_depth_and_intrinsics(sensor_msgs_stub):
    src, _node = _oak(sensor_msgs_stub)
    src._on_camera_info(_info_msg())
    src._on_depth(_depth_msg(9.8, 1000))
    src._on_depth(_depth_msg(10.02, 2000))
    out = src._attach_depth(_frame(10.0))
    assert out.depth_status == geo.STATUS_OK
    assert int(out.depth_mm[0, 0]) == 2000
    assert out.intrinsics == (50.0, 50.0, W / 2.0, H / 2.0)


def test_oak_on_msg_attaches_depth(sensor_msgs_stub, monkeypatch):
    """RGB-колбэк сам связывает кадр с глубиной (путь без cv2 подменён)."""
    monkeypatch.setattr(
        gaze, '_decode_image_msg_to_rgb',
        lambda _m: np.zeros((H, W, 3), dtype=np.uint8),
    )
    src, _node = _oak(sensor_msgs_stub)
    src._on_camera_info(_info_msg())
    src._on_depth(_depth_msg(5.0, 1700))
    src._on_msg(types.SimpleNamespace(header=_Header(5.01)))
    frame = src.poll_latest()
    assert frame.depth_status == geo.STATUS_OK
    assert frame.stamp == pytest.approx(5.01)
    assert frame.frame_id == 'camera_color_optical_frame'


def test_oak_late_depth_is_attached_on_arrival(sensor_msgs_stub, monkeypatch):
    """Depth пришёл ПОСЛЕ своего RGB — связывание повторяется."""
    monkeypatch.setattr(
        gaze, '_decode_image_msg_to_rgb',
        lambda _m: np.zeros((H, W, 3), dtype=np.uint8),
    )
    src, _node = _oak(sensor_msgs_stub)
    src._on_camera_info(_info_msg())
    src._on_msg(types.SimpleNamespace(header=_Header(7.0)))
    assert src.poll_latest().depth_status == geo.STATUS_NO_DEPTH_STREAM
    src._on_depth(_depth_msg(7.03, 900))
    frame = src.poll_latest()
    assert frame.depth_status == geo.STATUS_OK
    assert int(frame.depth_mm[0, 0]) == 900


def test_oak_depth_out_of_sync(sensor_msgs_stub):
    src, _node = _oak(sensor_msgs_stub, depth_sync_tolerance_sec=0.15)
    src._on_camera_info(_info_msg())
    src._on_depth(_depth_msg(10.0))
    out = src._attach_depth(_frame(10.2))
    assert out.depth_status == geo.STATUS_DEPTH_OUT_OF_SYNC
    assert out.depth_mm is None


def test_oak_depth_size_mismatch(sensor_msgs_stub):
    src, _node = _oak(sensor_msgs_stub)
    src._on_camera_info(_info_msg())
    src._on_depth(_depth_msg(1.0, w=W // 2, h=H // 2))
    out = src._attach_depth(_frame(1.0))
    assert out.depth_status == geo.STATUS_DEPTH_SIZE_MISMATCH
    assert out.depth_mm is None


def test_oak_no_camera_info(sensor_msgs_stub):
    src, _node = _oak(sensor_msgs_stub)
    src._on_depth(_depth_msg(1.0))
    out = src._attach_depth(_frame(1.0))
    assert out.depth_status == geo.STATUS_NO_CAMERA_INFO
    assert out.intrinsics is None


def test_oak_camera_info_size_mismatch(sensor_msgs_stub):
    src, _node = _oak(sensor_msgs_stub)
    src._on_camera_info(_info_msg(w=W * 2, h=H * 2))
    src._on_depth(_depth_msg(1.0))
    out = src._attach_depth(_frame(1.0))
    assert out.depth_status == geo.STATUS_CAMERA_INFO_SIZE_MISMATCH


def test_oak_frame_with_depth_feeds_geometry(sensor_msgs_stub):
    """Сквозной путь шва: Frame с глубиной → estimate_position → 1.5 м."""
    src, _node = _oak(sensor_msgs_stub)
    src._on_camera_info(_info_msg())
    src._on_depth(_depth_msg(3.0, 1500))
    frame = src._attach_depth(_frame(3.0))
    est = geo.estimate_position(
        frame.depth_mm, frame.intrinsics, (W / 2.0, H / 2.0, 10.0, 10.0),
        upstream_status=frame.depth_status,
    )
    assert est.valid is True
    assert est.distance_m == pytest.approx(1.5)


# ============================================================================
# Остальные источники — глубины нет
# ============================================================================

def test_ceiling_camera_has_no_depth(sensor_msgs_stub, monkeypatch):
    monkeypatch.setattr(
        gaze, '_decode_compressed_to_rgb',
        lambda _d: np.zeros((H, W, 3), dtype=np.uint8),
    )
    node = _FakeNode()
    src = gaze.CeilingCameraSource(node)
    assert set(node.subs) == {'/ceiling_camera/image_raw/compressed'}
    src._on_msg(types.SimpleNamespace(header=_Header(1.0), data=b''))
    frame = src.poll_latest()
    assert frame.depth_mm is None
    assert frame.depth_status == geo.STATUS_NO_DEPTH_STREAM


def test_stub_source_has_no_depth():
    frame = gaze.StubSource(period_sec=0.0).wait_for_first_frame(0.0)
    assert frame.depth_mm is None
    assert frame.depth_status == geo.STATUS_NO_DEPTH_STREAM
