"""Unit-тесты для context_aggregator_node VisionEvent-интеграции (ADR-0089).

Проверяет touchpoint #4:
  - Подписка на /vision/hailo/events создаётся при наличии VisionEvent.msg.
  - on_hailo_vision_event сериализует ROS-msg в dict + кладёт в буфер.
  - publish_event публикует vision_event_count + vision_events_json.

Тесты не требуют реального rclpy/colcon-build: используется тот же shim-паттерн,
что и в test_context_aggregator.py. Запуск:
    pytest src/rob_box_perception/test/unit/test_vision_events_aggregator.py -v
"""

from __future__ import annotations

import importlib
import json
import sys
import time
import types
from typing import Any, Callable, Dict, List
from unittest.mock import MagicMock

import pytest


# ---------- rclpy / msg shim (минимальный) -------------------------------

class _FakeNode:

    def __init__(self, name: str, **kwargs: Any) -> None:
        self._name = name
        self._logger = MagicMock()
        self._publishers: Dict[str, MagicMock] = {}
        self._subs: Dict[str, Callable[[Any], None]] = {}
        self._params: Dict[str, Any] = {}
        self._timers: List[Any] = []

    def get_logger(self) -> MagicMock:
        return self._logger

    def declare_parameter(self, name: str, default: Any = None) -> MagicMock:
        self._params.setdefault(name, default)
        return MagicMock()

    def get_parameter(self, name: str) -> Any:
        class _Param:

            def __init__(self, value):
                self.value = value

        return _Param(self._params.get(name))

    def has_parameter(self, name: str) -> bool:
        return name in self._params

    def create_publisher(self, msg_type, topic, depth, **kwargs):
        pub = MagicMock()
        pub.topic = topic
        pub.published: List[Any] = []
        original = pub.publish

        def _capture(msg):
            pub.published.append(msg)
            return original(msg)

        pub.publish = _capture
        self._publishers[topic] = pub
        return pub

    def create_subscription(self, msg_type, topic, callback, qos,
                            callback_group=None):
        sub = MagicMock()
        sub.topic = topic
        sub.callback = callback
        self._subs[topic] = callback
        return sub

    def create_timer(self, period, callback, callback_group=None):
        timer = MagicMock()
        timer.period = period
        timer.callback = callback
        self._timers.append(timer)
        return timer

    def get_name(self) -> str:
        return self._name

    def destroy_node(self) -> None:
        return None

    def get_clock(self) -> MagicMock:
        clock = MagicMock()
        clock.now.return_value.to_msg.return_value = MagicMock()
        return clock


# ---- rclpy stub ----
_rclpy = types.ModuleType('rclpy')
_rclpy.init = lambda *a, **kw: None
_rclpy.shutdown = lambda *a, **kw: None
_rclpy.ok = lambda: True
sys.modules.setdefault('rclpy', _rclpy)

_rclpy_node = types.ModuleType('rclpy.node')
_rclpy_node.Node = _FakeNode
sys.modules.setdefault('rclpy.node', _rclpy_node)

_cb_mod = types.ModuleType('rclpy.callback_groups')
_cb_mod.ReentrantCallbackGroup = type('ReentrantCallbackGroup', (), {})
sys.modules.setdefault('rclpy.callback_groups', _cb_mod)

_qos_mod = types.ModuleType('rclpy.qos')
_qos_mod.HistoryPolicy = types.SimpleNamespace(KEEP_LAST='KEEP_LAST')
_qos_mod.ReliabilityPolicy = types.SimpleNamespace(RELIABLE='RELIABLE')
_qos_mod.QoSProfile = lambda *a, **kw: MagicMock()
sys.modules.setdefault('rclpy.qos', _qos_mod)

# ---- std_msgs / geometry_msgs / nav_msgs / control_msgs / rcl_interfaces stubs ----
_std_msgs = types.ModuleType('std_msgs')
_std_msgs_msg = types.ModuleType('std_msgs.msg')


class _String:

    def __init__(self, data=''):
        self.data = data


_std_msgs_msg.String = _String
_std_msgs_msg.Bool = type('Bool', (), {'data': False})
sys.modules.setdefault('std_msgs', _std_msgs)
sys.modules.setdefault('std_msgs.msg', _std_msgs_msg)

_geom = types.ModuleType('geometry_msgs')
_geom_msg = types.ModuleType('geometry_msgs.msg')


class _Vector3:

    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = x
        self.y = y
        self.z = z


class _Point:

    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = x
        self.y = y
        self.z = z


class _Quaternion:

    def __init__(self, x=0.0, y=0.0, z=0.0, w=1.0):
        self.x = x
        self.y = y
        self.z = z
        self.w = w


class _Pose:

    def __init__(self):
        self.position = _Point()
        self.orientation = _Quaternion()


class _PoseStamped:

    def __init__(self):
        self.pose = _Pose()
        self.header = MagicMock()


class _Twist:

    def __init__(self):
        self.linear = _Vector3()
        self.angular = _Vector3()


class _TwistWithCovariance:

    def __init__(self):
        self.twist = _Twist()


_geom_msg.Vector3 = _Vector3
_geom_msg.Point = _Point
_geom_msg.Quaternion = _Quaternion
_geom_msg.Pose = _Pose
_geom_msg.PoseStamped = _PoseStamped
_geom_msg.Twist = _Twist
_geom_msg.TwistWithCovariance = _TwistWithCovariance
sys.modules.setdefault('geometry_msgs', _geom)
sys.modules.setdefault('geometry_msgs.msg', _geom_msg)

_nav = types.ModuleType('nav_msgs')
_nav_msg = types.ModuleType('nav_msgs.msg')


class _Odometry:

    def __init__(self):
        self.pose = MagicMock()
        self.pose.pose = _Pose()
        self.twist = MagicMock()
        self.twist.twist = _Twist()


_nav_msg.Odometry = _Odometry
sys.modules.setdefault('nav_msgs', _nav)
sys.modules.setdefault('nav_msgs.msg', _nav_msg)

_ctrl = types.ModuleType('control_msgs')
_ctrl_msg = types.ModuleType('control_msgs.msg')


class _DynamicJointState:

    def __init__(self):
        self.joint_names = []
        self.interface_values = []


_ctrl_msg.DynamicJointState = _DynamicJointState
sys.modules.setdefault('control_msgs', _ctrl)
sys.modules.setdefault('control_msgs.msg', _ctrl_msg)

_rcl_iface = types.ModuleType('rcl_interfaces')
_rcl_iface_msg = types.ModuleType('rcl_interfaces.msg')


class _Log:

    def __init__(self):
        self.level = 0
        self.name = ''
        self.msg = ''
        self.stamp = None


_rcl_iface_msg.Log = _Log
sys.modules.setdefault('rcl_interfaces', _rcl_iface)
sys.modules.setdefault('rcl_interfaces.msg', _rcl_iface_msg)

# ---- rob_box_perception_msgs stubs ----
_perception_msgs = types.ModuleType('rob_box_perception_msgs')
_perception_msgs_msg = types.ModuleType('rob_box_perception_msgs.msg')


class _PerceptionEvent:

    def __init__(self):
        # Все поля ADR-0089 + ранее существовавшие.
        self.stamp = MagicMock()
        self.vision_context = ''
        self.pose = _Pose()
        self.velocity = _Twist()
        self.is_moving = False
        self.battery_voltage = 0.0
        self.temperature = 0.0
        self.apriltag_ids: List[int] = []
        self.system_health_status = 'HEALTHY'
        self.health_issues: List[str] = []
        self.current_time_human = ''
        self.time_period = ''
        self.time_context_json = ''
        self.internet_available = False
        self.active_nodes: List[str] = []
        self.failed_nodes: List[str] = []
        self.missing_nodes: List[str] = []
        self.equipment_summary_json = '{}'
        self.mapping_mode = 'unknown'
        self.memory_summary = ''
        self.speech_summaries = ''
        self.robot_response_summaries = ''
        self.robot_thought_summaries = ''
        self.vision_summaries = ''
        self.system_summaries = ''
        # ADR-0089 fields:
        self.vision_event_count = 0
        self.vision_events_json = ''


class _VisionEvent:

    def __init__(self):
        self.stamp = MagicMock()
        self.stamp.sec = 1700000000
        self.stamp.nanosec = 123456789
        self.source_camera = ''
        self.event_type = ''
        self.class_name = ''
        self.class_id = -1
        self.confidence = 0.0
        self.bbox_cx = -1.0
        self.bbox_cy = -1.0
        self.bbox_w = -1.0
        self.bbox_h = -1.0
        self.distance_m = -1.0
        self.embedding_id = ''
        self.display_name = ''
        self.attributes_json = ''


_perception_msgs_msg.PerceptionEvent = _PerceptionEvent
_perception_msgs_msg.VisionEvent = _VisionEvent
sys.modules.setdefault('rob_box_perception_msgs', _perception_msgs)
sys.modules.setdefault('rob_box_perception_msgs.msg', _perception_msgs_msg)

# ---- utils stubs (просто чтобы import работал) ----
_utils_pkg = types.ModuleType('rob_box_perception.utils')
_utils_pkg.__path__ = []
_internet_monitor = types.ModuleType('rob_box_perception.utils.internet_monitor')
_internet_monitor.InternetConnectivityMonitor = MagicMock
_node_monitor = types.ModuleType('rob_box_perception.utils.node_monitor')
_node_monitor.NodeAvailabilityMonitor = MagicMock
_time_provider = types.ModuleType('rob_box_perception.utils.time_provider')
_time_provider.TimeAwarenessProvider = MagicMock
sys.modules.setdefault('rob_box_perception.utils', _utils_pkg)
sys.modules.setdefault('rob_box_perception.utils.internet_monitor',
                       _internet_monitor)
sys.modules.setdefault('rob_box_perception.utils.node_monitor', _node_monitor)
sys.modules.setdefault('rob_box_perception.utils.time_provider', _time_provider)

# ---- import rob_box_perception.context_aggregator_node ----
repo_root = '/home/builder/rob_box_project/.worktrees/t_b9b6cf73'
pkg_root = f'{repo_root}/src/rob_box_perception'
if pkg_root not in sys.path:
    sys.path.insert(0, pkg_root)

# Должны уже быть в sys.modules из-за контекста — но явно импортируем.
importlib.import_module('rob_box_perception')
context_aggregator_module = importlib.import_module(
    'rob_box_perception.context_aggregator_node'
)
ContextAggregatorNode = context_aggregator_module.ContextAggregatorNode


# ---------- VisionEvent fake-helper для тестов ----------------------------


def _make_vision_event(**overrides):
    msg = _VisionEvent()
    msg.source_camera = overrides.get('source_camera', 'oak-d')
    msg.event_type = overrides.get('event_type', 'person')
    msg.class_name = overrides.get('class_name', 'person')
    msg.class_id = overrides.get('class_id', 0)
    msg.confidence = overrides.get('confidence', 0.85)
    msg.bbox_cx = overrides.get('bbox_cx', 0.5)
    msg.bbox_cy = overrides.get('bbox_cy', 0.5)
    msg.bbox_w = overrides.get('bbox_w', 0.3)
    msg.bbox_h = overrides.get('bbox_h', 0.5)
    msg.distance_m = overrides.get('distance_m', 1.2)
    msg.embedding_id = overrides.get('embedding_id', '')
    msg.display_name = overrides.get('display_name', '')
    msg.attributes_json = overrides.get('attributes_json', '{}')
    return msg


# ---------- tests ---------------------------------------------------------


def test_hailo_subscription_created():
    """Подписка на /vision/hailo/events создаётся когда VisionEvent доступен."""
    node = ContextAggregatorNode()
    assert node._hailo_events_sub is not None
    assert node._hailo_events_sub.topic == '/vision/hailo/events'
    assert callable(node._hailo_events_sub.callback)


def test_hailo_event_buffer_initialized_empty():
    """Кольцевой буфер _hailo_events стартует пустым."""
    node = ContextAggregatorNode()
    assert node._hailo_events == []


def test_on_hailo_vision_event_appends_to_buffer():
    """on_hailo_vision_event добавляет сериализованный event в буфер."""
    node = ContextAggregatorNode()
    msg = _make_vision_event(confidence=0.91, distance_m=1.5)
    node.on_hailo_vision_event(msg)
    assert len(node._hailo_events) == 1
    item = node._hailo_events[0]
    assert 'time' in item
    assert 'event' in item
    event = item['event']
    assert event['source_camera'] == 'oak-d'
    assert event['event_type'] == 'person'
    assert event['confidence'] == pytest.approx(0.91)
    assert event['distance_m'] == pytest.approx(1.5)


def test_on_hailo_vision_event_respects_memory_window():
    """События старше memory_window секунд удаляются из буфера."""
    node = ContextAggregatorNode()
    node.memory_window = 1.0  # 1 секунда для теста
    # Свежее событие
    node.on_hailo_vision_event(_make_vision_event(confidence=0.9))
    # Подделываем время: добавляем "старое" событие через backdoor
    node._hailo_events.append({
        'time': time.time() - 100.0,  # давно
        'event': {'source_camera': 'old', 'event_type': 'person'},
    })
    # Новый callback должен почистить старые
    node.on_hailo_vision_event(_make_vision_event(confidence=0.8))
    # Остаётся только 2 события (свежее + только что добавленное), старого нет.
    assert len(node._hailo_events) == 2
    sources = [item['event']['source_camera'] for item in node._hailo_events]
    assert 'old' not in sources


def test_publish_event_drops_stale_events_without_new_arrivals():
    """Человек ушёл из кадра — старые детекции не доезжают до Личности.

    Живой случай 16.09.2026: лицо пропало, новых VisionEvent не было, и
    чистка в on_hailo_vision_event не срабатывала — PerceptionEvent ещё
    6 минут нёс 88 событий `face`, робот говорил «вижу тебя» пустому кадру.
    """
    node = ContextAggregatorNode()
    node.memory_window = 1.0
    node.current_sensors = {'battery': 36.0, 'temperature': 42.0}
    node._hailo_events.append({
        'time': time.time() - 100.0,
        'event': {'source_camera': 'oak-d', 'event_type': 'face'},
    })
    node.publish_event()
    published = node.event_pub.published[0]
    assert published.vision_event_count == 0
    assert json.loads(published.vision_events_json) == []
    assert node._hailo_events == []


def test_publish_event_includes_vision_event_count_and_json():
    """publish_event публикует vision_event_count + vision_events_json."""
    node = ContextAggregatorNode()
    # current_sensors / current_vision заполняем валидным dict, чтобы
    # publish_event() мог сериализовать их в JSON.
    node.current_sensors = {'battery': 36.0, 'temperature': 42.0}
    # Добавляем 2 события в буфер.
    node.on_hailo_vision_event(_make_vision_event(confidence=0.85))
    node.on_hailo_vision_event(
        _make_vision_event(event_type='face', class_name='masha',
                           confidence=0.92, embedding_id='rowid_1',
                           display_name='Masha')
    )
    # Запускаем publish_event
    node.publish_event()
    assert node.event_pub is not None
    assert len(node.event_pub.published) == 1
    published = node.event_pub.published[0]
    # vision_event_count = 2
    assert published.vision_event_count == 2
    # vision_events_json — валидный JSON, два элемента.
    payload = json.loads(published.vision_events_json)
    assert isinstance(payload, list)
    assert len(payload) == 2
    assert payload[0]['event_type'] == 'person'
    assert payload[1]['event_type'] == 'face'
    assert payload[1]['display_name'] == 'Masha'


def test_publish_event_empty_buffer_yields_empty_json():
    """publish_event без событий публикует count=0 и пустой JSON-массив."""
    node = ContextAggregatorNode()
    node.current_sensors = {'battery': 36.0, 'temperature': 42.0}
    node.publish_event()
    assert node.event_pub is not None
    assert len(node.event_pub.published) == 1
    published = node.event_pub.published[0]
    assert published.vision_event_count == 0
    payload = json.loads(published.vision_events_json)
    assert payload == []


# ---------- приватность: stub не пересекает PerceptionEvent (issue #2532) --
#
# ADR-0089 §2.2: StubHEFLoader выдумывает "person, conf 0.92, 1 м" —
# PR #2583 дал этому честный маркер (event_type='stub'/source_camera='stub',
# vision_hailo_loader.is_stub_event). Архитектурное решение issue #2532:
# фильтрация стоит здесь, в publish_event(), в момент сборки PerceptionEvent
# — единственного продюсера этого сообщения. Личность читает
# vision_event_count/vision_events_json ТОЛЬКО из PerceptionEvent, поэтому
# граница на входе в него закрывает вопрос раз и навсегда.


def test_publish_event_real_event_reaches_perception_event():
    """Реальная детекция (без stub-маркера) доходит до PerceptionEvent."""
    node = ContextAggregatorNode()
    node.current_sensors = {'battery': 36.0, 'temperature': 42.0}
    node.on_hailo_vision_event(
        _make_vision_event(event_type='person', source_camera='oak-d',
                           confidence=0.87)
    )
    node.publish_event()
    published = node.event_pub.published[0]
    assert published.vision_event_count == 1
    payload = json.loads(published.vision_events_json)
    assert len(payload) == 1
    assert payload[0]['event_type'] == 'person'


def test_publish_event_stub_event_never_reaches_perception_event():
    """Выдуманное stub-событие НЕ должно попасть в PerceptionEvent вовсе."""
    node = ContextAggregatorNode()
    node.current_sensors = {'battery': 36.0, 'temperature': 42.0}
    node.on_hailo_vision_event(
        _make_vision_event(event_type='stub', source_camera='stub',
                           class_name='person', confidence=0.92,
                           distance_m=1.0)
    )
    node.publish_event()
    published = node.event_pub.published[0]
    assert published.vision_event_count == 0
    payload = json.loads(published.vision_events_json)
    assert payload == []


def test_publish_event_mixed_stub_and_real_keeps_only_real_with_correct_count():
    """27 выдуманных + 0 реальных -> Личность видит 0, а не 27 (issue #2532).

    Здесь — упрощённая версия смеси (2 stub + 1 real), но тот же принцип:
    счётчик и список считаются ПОСЛЕ фильтрации, не до неё.
    """
    node = ContextAggregatorNode()
    node.current_sensors = {'battery': 36.0, 'temperature': 42.0}
    node.on_hailo_vision_event(
        _make_vision_event(event_type='stub', source_camera='stub',
                           confidence=0.92)
    )
    node.on_hailo_vision_event(
        _make_vision_event(event_type='person', source_camera='oak-d',
                           confidence=0.81)
    )
    node.on_hailo_vision_event(
        _make_vision_event(event_type='person', source_camera='stub',
                           confidence=0.5)
    )
    node.publish_event()
    published = node.event_pub.published[0]
    assert published.vision_event_count == 1
    payload = json.loads(published.vision_events_json)
    assert len(payload) == 1
    assert payload[0]['source_camera'] == 'oak-d'


def test_publish_event_all_stub_yields_zero_not_stub_count():
    """Буфер целиком из выдумки -> count=0, а не len(буфера)."""
    node = ContextAggregatorNode()
    node.current_sensors = {'battery': 36.0, 'temperature': 42.0}
    for _ in range(3):
        node.on_hailo_vision_event(
            _make_vision_event(event_type='stub', source_camera='stub')
        )
    node.publish_event()
    published = node.event_pub.published[0]
    assert published.vision_event_count == 0
    assert json.loads(published.vision_events_json) == []


# ---- фикстура: гарантируем что у MagicMock-instance есть нужные методы ----

@pytest.fixture(autouse=True)
def _patch_node_monitors(monkeypatch):
    """Подменяет node_monitor / internet_monitor / time_provider на
    MagicMock-instance с заранее настроенными методами для publish_event().

    Без этого `publish_event()` падает на json.dumps(time_context) с
    TypeError (MagicMock-значения) и на get_status_summary() с
    AttributeError.
    """
    fake_node_monitor = MagicMock()
    fake_node_monitor.get_status_summary.return_value = {
        'active': 0, 'failed': 0, 'missing': 0,
        'active_list': [], 'failed_list': [], 'missing_list': [],
    }
    fake_internet_monitor = MagicMock()
    fake_internet_monitor.get_status.return_value = {'is_online': True}
    fake_time_provider = MagicMock()
    fake_time_provider.get_current_time_context.return_value = {
        'human_readable': '2026-09-14 03:00:00',
        'period': 'night',
    }

    monkeypatch.setattr(
        'rob_box_perception.context_aggregator_node.NodeAvailabilityMonitor',
        lambda _self: fake_node_monitor,
        raising=False,
    )
    monkeypatch.setattr(
        'rob_box_perception.context_aggregator_node.InternetConnectivityMonitor',
        lambda _self, **_kwargs: fake_internet_monitor,
        raising=False,
    )
    monkeypatch.setattr(
        'rob_box_perception.context_aggregator_node.TimeAwarenessProvider',
        lambda **_kwargs: fake_time_provider,
        raising=False,
    )
    yield
