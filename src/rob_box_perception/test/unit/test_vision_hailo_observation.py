"""VisionHailoNode публикует Observation вместе с VisionEvent (ADR-0138).

Контракт этапа 1 ADR-0130 на уровне ноды:
  1. Детекция person/face из реального кадра → VisionEvent с заполненным
     ``distance_m`` (раньше всегда −1) + Observation на отдельном топике.
  2. Observation.header = stamp и frame_id КАДРА (ADR-0130 §2.5), а не
     время публикации; VisionEvent.stamp как был — время публикации.
  3. Нет глубины → VisionEvent.distance_m = −1, Observation с
     position_valid=false и причиной (capability-honest).
  4. Stub-события (ADR-0089 §2.2) в наблюдения не попадают, их
     VisionEvent не меняется. Объекты (cup и т.п.) — только VisionEvent.
  5. В Observation нет имени, даже если VisionEvent его несёт.
  6. ``_tick`` передаёт в публикацию тот же кадр, на котором был infer.

``vision_hailo_node`` импортирует ``rclpy`` на уровне модуля — тот же
shim, что в test_vision_face_log_stats.py; ноду собираем через
``__new__`` (паттерн test_vision_heartbeat.py), msg-классы подменяем
простыми дублёрами через monkeypatch.

Запуск:
    python -m pytest src/rob_box_perception/test/unit/test_vision_hailo_observation.py -q --no-cov
"""

from __future__ import annotations

import importlib
import math
import sys
import types
from pathlib import Path
from typing import Any, List

import pytest

np = pytest.importorskip('numpy')

_HERE = Path(__file__).resolve()
_PKG_ROOT = _HERE.parents[2]
if str(_PKG_ROOT) not in sys.path:
    sys.path.insert(0, str(_PKG_ROOT))

if 'rclpy' not in sys.modules:
    _rclpy = types.ModuleType('rclpy')
    _rclpy.init = lambda *a, **k: None
    _rclpy.spin = lambda *a, **k: None
    _rclpy.try_shutdown = lambda *a, **k: None
    _rclpy_node = types.ModuleType('rclpy.node')

    class _StubNode:  # pragma: no cover — только для `from rclpy.node import Node`
        def __init__(self, *a: Any, **k: Any) -> None:
            pass

    _rclpy_node.Node = _StubNode
    _rclpy.node = _rclpy_node
    sys.modules['rclpy'] = _rclpy
    sys.modules['rclpy.node'] = _rclpy_node

node_mod = importlib.import_module('rob_box_perception.vision_hailo_node')
gaze = importlib.import_module('rob_box_perception.gaze')
geo = importlib.import_module('rob_box_perception.observation_geometry')
loader_mod = importlib.import_module('rob_box_perception.vision_hailo_loader')

W, H = 128, 72
K = (100.0, 100.0, 64.0, 36.0)


# ---------- msg doubles -----------------------------------------------------

class _Time:
    def __init__(self) -> None:
        self.sec = 0
        self.nanosec = 0


class _Header:
    def __init__(self) -> None:
        self.stamp = _Time()
        self.frame_id = ''


class _Point:
    def __init__(self) -> None:
        self.x = self.y = self.z = 0.0


class _VisionEventMsg:
    pass


class _ObservationMsg:
    def __init__(self) -> None:
        self.header = _Header()
        self.position = _Point()


class _Publisher:
    def __init__(self) -> None:
        self.published: List[Any] = []

    def publish(self, msg: Any) -> None:
        self.published.append(msg)


class _Clock:
    def now(self):
        return types.SimpleNamespace(to_msg=lambda: 'publish-time')


class _Logger:
    def __getattr__(self, _name):
        return lambda *a, **k: None


class _FakeHeartbeat:
    def beat(self) -> None:
        pass


class _FakeLoader:
    def __init__(self, events) -> None:
        self.events = events
        self.images: List[Any] = []

    def infer(self, frame_id, image):
        self.images.append(image)
        return [dict(e) for e in self.events]


@pytest.fixture
def msgs(monkeypatch):
    monkeypatch.setattr(node_mod, 'VisionEventMsg', _VisionEventMsg)
    monkeypatch.setattr(node_mod, 'ObservationMsg', _ObservationMsg)


def _bare_node(loader=None, frame=None, real=True):
    node = node_mod.VisionHailoNode.__new__(node_mod.VisionHailoNode)
    node._publisher = _Publisher()
    node._observation_publisher = _Publisher()
    node.get_clock = lambda: _Clock()
    node.get_logger = lambda: _Logger()
    node.publish_when_no_input = True
    node._has_received_frame = True
    node._last_frame_id = frame.frame_id if frame else 'stub_frame'
    node._is_real_mode = real
    node._latest_frame = frame
    node._loader = loader
    node.confidence_threshold = 0.5
    node._consecutive_failures = 0
    node._degraded_logged = False
    node._max_logged_failures = 3
    node._heartbeat = _FakeHeartbeat()
    return node


def _frame(depth_mm=2000, status=geo.STATUS_OK, stamp=1727500000.25):
    depth = None if depth_mm is None else np.full((H, W), depth_mm, dtype=np.uint16)
    return gaze.Frame(
        rgb=np.zeros((H, W, 3), dtype=np.uint8), scale=1.0, pad_left=0,
        pad_top=0, original_w=W, original_h=H,
        frame_id='camera_color_optical_frame', stamp=stamp, source_name='oak_d',
        depth_mm=depth, intrinsics=K if depth is not None else None,
        depth_status=status,
    )


def _event(**overrides):
    ev = {
        'source_camera': 'camera_color_optical_frame', 'event_type': 'person',
        'class_name': 'person', 'class_id': 0, 'confidence': 0.9,
        'bbox_cx': 0.5, 'bbox_cy': 0.5, 'bbox_w': 0.3, 'bbox_h': 0.6,
        'distance_m': -1.0, 'embedding_id': '', 'display_name': '',
        'attributes_json': '',
    }
    ev.update(overrides)
    return ev


# ---------- tests -----------------------------------------------------------

def test_person_publishes_vision_event_with_distance_and_observation(msgs):
    node = _bare_node()
    frame = _frame(depth_mm=2000)
    node._publish_event(_event(), frame)

    [ve] = node._publisher.published
    assert ve.distance_m == pytest.approx(2.0)
    assert ve.stamp == 'publish-time'

    [obs] = node._observation_publisher.published
    assert obs.header.frame_id == 'camera_color_optical_frame'
    assert obs.header.stamp.sec == 1727500000
    assert obs.header.stamp.nanosec == 250_000_000
    assert obs.source == 'oak_d'
    assert obs.class_name == 'person'
    assert obs.position_valid is True
    assert obs.position_status == 'ok'
    assert obs.position.z == pytest.approx(2.0)
    assert obs.distance_m == pytest.approx(ve.distance_m)
    assert (obs.image_width, obs.image_height) == (W, H)
    assert obs.bbox_w_px == pytest.approx(0.3 * W)


def test_no_depth_keeps_minus_one_and_honest_status(msgs):
    node = _bare_node()
    frame = _frame(depth_mm=None, status=geo.STATUS_DEPTH_OUT_OF_SYNC)
    node._publish_event(_event(), frame)

    [ve] = node._publisher.published
    assert ve.distance_m == -1.0
    [obs] = node._observation_publisher.published
    assert obs.position_valid is False
    assert obs.position_status == geo.STATUS_DEPTH_OUT_OF_SYNC
    assert math.isnan(obs.position.x)
    assert obs.distance_m == -1.0


def test_face_observation_has_candidate_but_no_name(msgs):
    node = _bare_node()
    ev = _event(event_type='face', class_name='face', embedding_id='a659ddab',
                display_name='Дэнчик')
    node._publish_event(ev, _frame())

    [ve] = node._publisher.published
    assert ve.display_name == 'Дэнчик'  # VisionEvent как был (deprecated)
    [obs] = node._observation_publisher.published
    assert obs.class_name == 'face'
    assert obs.candidate_id == 'a659ddab'
    assert 'Дэнчик' not in [v for v in vars(obs).values() if isinstance(v, str)]
    assert not hasattr(obs, 'display_name')


def test_object_class_gets_no_observation(msgs):
    node = _bare_node()
    node._publish_event(
        _event(event_type='object', class_name='cup', class_id=41), _frame(),
    )
    [ve] = node._publisher.published
    assert ve.distance_m == -1.0
    assert node._observation_publisher.published == []


def test_stub_event_untouched_and_not_observed(msgs):
    node = _bare_node()
    stub_ev = loader_mod.StubHEFLoader(period_sec=0.0).infer('x', None)[0]
    node._publish_event(stub_ev, _frame())
    [ve] = node._publisher.published
    assert ve.event_type == loader_mod.STUB_EVENT_TYPE
    assert ve.distance_m == pytest.approx(stub_ev['distance_m'])
    assert node._observation_publisher.published == []


def test_no_frame_publishes_only_vision_event(msgs):
    node = _bare_node()
    node._publish_event(_event())
    assert len(node._publisher.published) == 1
    assert node._observation_publisher.published == []


def test_observation_publisher_absent_does_not_break_vision_event(msgs, monkeypatch):
    """Без сгенерированного Observation msg VisionEvent всё равно уходит."""
    monkeypatch.setattr(node_mod, 'ObservationMsg', None)
    node = _bare_node()
    node._observation_publisher = None
    node._publish_event(_event(), _frame())
    [ve] = node._publisher.published
    assert ve.distance_m == pytest.approx(2.0)


def test_tick_uses_same_frame_for_infer_and_observation(msgs):
    frame = _frame(depth_mm=3000)
    loader = _FakeLoader([_event()])
    node = _bare_node(loader=loader, frame=frame, real=True)
    node._tick()

    assert loader.images == [frame.rgb]
    [obs] = node._observation_publisher.published
    assert obs.header.stamp.sec == int(frame.stamp)
    assert obs.distance_m == pytest.approx(3.0)


def test_tick_stub_mode_publishes_no_observations(msgs):
    frame = _frame()
    loader = _FakeLoader([_event()])
    node = _bare_node(loader=loader, frame=frame, real=False)
    node._tick()
    assert loader.images == [None]
    assert len(node._publisher.published) == 1
    assert node._observation_publisher.published == []
