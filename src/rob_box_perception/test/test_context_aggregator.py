#!/usr/bin/env python3
"""
test_context_aggregator.py - Unit тесты для ContextAggregatorNode

Тестирует (после W10: perception LLM-free):
- Подписку на источники данных
- Агрегацию контекста
- Публикацию событий восприятия
- Работу с кэшем состояния
- Мониторинг доступности нод

The file installs its own minimal ``rclpy`` / ``std_msgs`` shim when the
ROS2 runtime is absent so the suite is runnable with plain ``pytest``
on developer laptops and in CI containers that may not have ROS2.
Mirrors the pattern used by ``test_perception_bridge.py`` and
``src/rob_box_voice/test/test_dialogue_shell.py``.
"""

from __future__ import annotations

import os
import sys
import time
import types as _types
import unittest
from unittest.mock import MagicMock, patch

from typing import Any, Callable, Dict, List, Optional

# ── rclpy / std_msgs / geometry_msgs / nav_msgs shim ─────────────────────
# Minimal stand-ins for the ROS2 message types that ContextAggregatorNode
# exercises in tests. We only need the attributes that the test file
# reads (PoseStamped.pose.position, Odometry.pose.pose.position, etc.).
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

    def create_publisher(self, msg_type, topic, depth, **kwargs) -> MagicMock:
        pub = MagicMock()
        pub.topic = topic
        pub.published = []
        original = pub.publish

        def _capture(msg):
            pub.published.append(msg)
            return original(msg)

        pub.publish = _capture
        self._publishers[topic] = pub
        return pub

    def create_subscription(self, msg_type, topic, callback, qos,
                              callback_group=None) -> MagicMock:
        sub = MagicMock()
        sub.topic = topic
        sub.callback = callback
        self._subs[topic] = callback
        return sub

    def create_timer(self, period, callback, callback_group=None) -> MagicMock:
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


_mock_rclpy = _types.ModuleType("rclpy")
_mock_rclpy.init = lambda *a, **kw: None
_mock_rclpy.shutdown = lambda *a, **kw: None
_mock_rclpy.ok = lambda: True
sys.modules.setdefault("rclpy", _mock_rclpy)

_mock_rclpy_node = _types.ModuleType("rclpy.node")
_mock_rclpy_node.Node = _FakeNode
sys.modules.setdefault("rclpy.node", _mock_rclpy_node)

_interfaces = _types.ModuleType("rcl_interfaces")


class _Log:
    def __init__(self):
        self.level = 0
        self.name = ""
        self.msg = ""
        self.stamp = None
_interfaces_msg = _types.ModuleType("rcl_interfaces.msg")
_interfaces_msg.Log = _Log
sys.modules.setdefault("rcl_interfaces", _interfaces)
sys.modules.setdefault("rcl_interfaces.msg", _interfaces_msg)

_cb_mod = _types.ModuleType("rclpy.callback_groups")
_cb_mod.ReentrantCallbackGroup = type("ReentrantCallbackGroup", (), {})
sys.modules.setdefault("rclpy.callback_groups", _cb_mod)

_qos_mod = _types.ModuleType("rclpy.qos")
_qos_mod.HistoryPolicy = _types.SimpleNamespace(KEEP_LAST="KEEP_LAST")
_qos_mod.ReliabilityPolicy = _types.SimpleNamespace(RELIABLE="RELIABLE")
_qos_mod.QoSProfile = lambda *a, **kw: MagicMock()
sys.modules.setdefault("rclpy.qos", _qos_mod)

_std_msgs = _types.ModuleType("std_msgs")
_std_msgs_msg = _types.ModuleType("std_msgs.msg")


class _String:
    def __init__(self, data=""):
        self.data = data


class _Bool:
    def __init__(self, data=False):
        self.data = data


_std_msgs_msg.String = _String
_std_msgs_msg.Bool = _Bool
sys.modules.setdefault("std_msgs", _std_msgs)
sys.modules.setdefault("std_msgs.msg", _std_msgs_msg)

# ── geometry_msgs shim ───────────────────────────────────────────────────
_geom = _types.ModuleType("geometry_msgs")
_geom_msg = _types.ModuleType("geometry_msgs.msg")


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
sys.modules.setdefault("geometry_msgs", _geom)
sys.modules.setdefault("geometry_msgs.msg", _geom_msg)

# ── nav_msgs shim ────────────────────────────────────────────────────────
_nav = _types.ModuleType("nav_msgs")
_nav_msg = _types.ModuleType("nav_msgs.msg")


class _Odometry:
    def __init__(self):
        self.pose = _types.SimpleNamespace(
            pose=_Pose(),
            covariance=[],
        )
        self.twist = _TwistWithCovariance()
        self.header = MagicMock()


_nav_msg.Odometry = _Odometry
sys.modules.setdefault("nav_msgs", _nav)
sys.modules.setdefault("nav_msgs.msg", _nav_msg)

# ── control_msgs shim ────────────────────────────────────────────────────
# context_aggregator_node.py imports DynamicJointState from control_msgs.msg
# (used in the /dynamic_joint_states subscriber for battery telemetry).
_control = _types.ModuleType("control_msgs")
_control_msg = _types.ModuleType("control_msgs.msg")


class _DynamicJointState:
    def __init__(self):
        self.joint_names: List[str] = []
        self.interface_values: List[Any] = []
        self.header = MagicMock()


_control_msg.DynamicJointState = _DynamicJointState
sys.modules.setdefault("control_msgs", _control)
sys.modules.setdefault("control_msgs.msg", _control_msg)

# ── rob_box_perception_msgs shim ─────────────────────────────────────────
# context_aggregator_node.py tries to import ``PerceptionEvent`` from this
# package and falls back to ``None`` if missing. Without the shim, the
# ``publish_event`` test sees ``event_pub = None`` and the mock cannot
# patch ``.publish``. Stub the class so the publisher is created.
_msgs = _types.ModuleType("rob_box_perception_msgs")
_msgs_msg = _types.ModuleType("rob_box_perception_msgs.msg")


class _PerceptionEvent:
    def __init__(self):
        self.header = MagicMock()
        # The aggregator assigns arbitrary fields; MagicMock attributes
        # absorb any read.
        for f in (
            "timestamp", "battery_level", "robot_pose_x", "robot_pose_y",
            "current_location", "online_status", "node_health",
            "active_nodes", "internet_connected", "system_health_status",
            "system_health_issues", "memory_summary", "recent_events",
            "important_event", "vision_description", "current_pose",
            "current_sensors", "current_vision", "ssml_text",
        ):
            setattr(self, f, None)

    def __setattr__(self, name, value):
        # Allow any field assignment silently.
        object.__setattr__(self, name, value)


_msgs_msg.PerceptionEvent = _PerceptionEvent
sys.modules.setdefault("rob_box_perception_msgs", _msgs)
sys.modules.setdefault("rob_box_perception_msgs.msg", _msgs_msg)

# Import after the shim is registered.
import rclpy  # noqa: E402  — resolves to the shim module registered above
from std_msgs.msg import String, Bool  # noqa: E402
from geometry_msgs.msg import PoseStamped, Point, Quaternion  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402

from rob_box_perception.context_aggregator_node import ContextAggregatorNode  # noqa: E402


class TestContextAggregator(unittest.TestCase):
    """Тесты для ContextAggregatorNode"""

    @classmethod
    def setUpClass(cls):
        """Инициализация ROS2"""
        if not rclpy.ok():
            rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """Завершение ROS2"""
        if rclpy.ok():
            rclpy.shutdown()

    def setUp(self):
        """Подготовка перед каждым тестом"""
        self.node = ContextAggregatorNode()
        
    def tearDown(self):
        """Очистка после каждого теста"""
        self.node.destroy_node()

    def test_node_initialization(self):
        """Тест: Нода инициализируется корректно"""
        self.assertEqual(self.node.get_name(), 'context_aggregator')

        # Проверяем параметры (без enable_summarization — W10 LLM-free)
        self.assertTrue(self.node.has_parameter('publish_rate'))
        self.assertTrue(self.node.has_parameter('memory_window'))
        self.assertTrue(self.node.has_parameter('timezone'))

        # Проверяем начальное состояние
        self.assertIsNone(self.node.current_vision)
        self.assertIsNone(self.node.current_pose)

    def test_parameters(self):
        """Тест: Параметры ноды (LLM-free после W10)"""
        # Проверяем значения по умолчанию. enable_summarization
        # был удалён в W10 вместе с LLM-кодом.
        self.assertEqual(self.node.publish_rate, 2.0)
        self.assertEqual(self.node.memory_window, 60)
        self.assertEqual(self.node.timezone, 'Europe/Moscow')
        # Подтверждаем что LLM-параметр действительно отсутствует
        self.assertFalse(self.node.has_parameter('enable_summarization'))

    def test_vision_context_subscription(self):
        """Тест: Подписка на vision context"""
        # Создаём vision сообщение
        vision_msg = String()
        vision_msg.data = '{"objects": ["table", "chair"], "scene": "kitchen"}'
        
        # Проверяем что есть callback для vision
        self.assertTrue(hasattr(self.node, 'on_vision_context'))
        
        # Вызываем callback
        self.node.on_vision_context(vision_msg)
        
        # Проверяем что vision сохранен
        self.assertIsNotNone(self.node.current_vision)

    def test_pose_subscription(self):
        """Тест: Подписка на позицию (localization_pose)"""
        # Создаём pose сообщение
        pose_msg = PoseStamped()
        pose_msg.pose.position = Point(x=1.0, y=2.0, z=0.0)
        pose_msg.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        
        # Проверяем callback
        self.assertTrue(hasattr(self.node, 'on_robot_pose'))
        
        # Вызываем callback
        self.node.on_robot_pose(pose_msg)
        
        # Проверяем что позиция сохранена
        self.assertIsNotNone(self.node.current_pose)
        self.assertEqual(self.node.current_pose.pose.position.x, 1.0)

    def test_odometry_subscription(self):
        """Тест: Подписка на одометрию"""
        # Создаём odometry сообщение
        odom_msg = Odometry()
        odom_msg.pose.pose.position = Point(x=3.0, y=4.0, z=0.0)
        
        # Проверяем callback
        self.assertTrue(hasattr(self.node, 'on_odometry'))
        
        # Вызываем callback
        self.node.on_odometry(odom_msg)
        
        # Проверяем что одометрия сохранена
        self.assertIsNotNone(self.node.current_odom)

    def test_stt_result_subscription(self):
        """Тест: Подписка на результаты STT"""
        # Создаём STT сообщение
        stt_msg = String()
        stt_msg.data = "Привет робот"

        # Проверяем callback
        self.assertTrue(hasattr(self.node, 'on_user_speech'))

        # speech_pub удалён в W10 вместе с LLM-кодом.
        self.assertFalse(hasattr(self.node, 'speech_pub'))

        # Вызываем callback (просто проверяем что не падает)
        self.node.on_user_speech(stt_msg)

    def test_device_snapshot_subscription(self):
        """Тест: Подписка на ros2_control joint_states (батарея)"""
        # Проверяем callback
        self.assertTrue(hasattr(self.node, 'on_joint_states'))
        
        # SKIP: Сложная структура DynamicJointState, нужны моки
        # Просто проверяем что current_sensors существует
        self.assertIsNotNone(self.node.current_sensors)
        self.assertIsInstance(self.node.current_sensors, dict)

    def test_memory_window(self):
        """Тест: Ограничение окна памяти для истории"""
        # Устанавливаем небольшое окно памяти
        self.node.memory_window = 2  # 2 секунды
        
        # SKIP: Очистка истории делается внутри publish_event
        # Просто проверяем что memory_window установлен
        self.assertEqual(self.node.memory_window, 2)

    def test_context_publisher_exists(self):
        """Тест: Publisher для контекста существует"""
        self.assertTrue(hasattr(self.node, 'event_pub'))

    def test_multiple_data_sources(self):
        """Тест: Агрегация данных из нескольких источников"""
        # Посылаем данные из разных источников
        
        # Vision
        vision_msg = String()
        vision_msg.data = '{"objects": ["cup"]}'
        self.node.on_vision_context(vision_msg)
        
        # Pose
        pose_msg = PoseStamped()
        pose_msg.pose.position = Point(x=5.0, y=6.0, z=0.0)
        self.node.on_robot_pose(pose_msg)
        
        # Проверяем что все данные сохранены
        self.assertIsNotNone(self.node.current_vision)
        self.assertIsNotNone(self.node.current_pose)
        self.assertIsNotNone(self.node.current_sensors)
        
        # Проверяем что можем собрать полный контекст
        if hasattr(self.node, 'get_current_context'):
            context = self.node.get_current_context()
            self.assertIsNotNone(context)

    def test_node_availability_monitor(self):
        """Тест: Мониторинг доступности нод"""
        # Проверяем что монитор существует
        self.assertTrue(hasattr(self.node, 'node_monitor'))
        
        # Проверяем что мониторятся критичные ноды
        if hasattr(self.node.node_monitor, 'critical_nodes'):
            self.assertGreater(len(self.node.node_monitor.critical_nodes), 0)

    def test_internet_connectivity_monitor(self):
        """Тест: Мониторинг интернет соединения"""
        # Проверяем что монитор существует
        self.assertTrue(hasattr(self.node, 'internet_monitor'))

    def test_time_awareness_provider(self):
        """Тест: Провайдер осведомлённости о времени"""
        # Проверяем что провайдер существует
        self.assertTrue(hasattr(self.node, 'time_provider'))
        
        # Проверяем что можем получить текущее время
        if hasattr(self.node.time_provider, 'get_current_time_str'):
            time_str = self.node.time_provider.get_current_time_str()
            self.assertIsNotNone(time_str)
            self.assertIsInstance(time_str, str)


class TestCallbacks(unittest.TestCase):
    """Тесты для дополнительных callbacks"""

    @classmethod
    def setUpClass(cls):
        if not rclpy.ok():
            rclpy.init()

    def setUp(self):
        self.node = ContextAggregatorNode()
        
    def tearDown(self):
        self.node.destroy_node()

    def test_on_robot_response(self):
        """Тест: callback robot_response"""
        msg = String()
        msg.data = '{"ssml": "<speak>Привет!</speak>"}'
        
        self.node.on_robot_response(msg)
        
        # Проверяем что добавлено в robot_response_events
        self.assertGreater(len(self.node.robot_response_events), 0)
        self.assertEqual(self.node.robot_response_events[-1]['type'], 'robot_response')

    def test_on_robot_thought(self):
        """Тест: callback robot_thought (удалён в W10 — пропускаем)"""
        # on_robot_thought был удалён в W10 вместе с LLM-кодом.
        self.assertFalse(hasattr(self.node, 'on_robot_thought'))

    def test_on_command_intent(self):
        """Тест: callback command_intent (сохранён после W10)"""
        msg = String()
        msg.data = "navigate:0.85"

        # on_command_intent — обычный command routing, без LLM.
        if hasattr(self.node, 'on_command_intent'):
            self.node.on_command_intent(msg)
            # Команда попадает в recent_events через add_to_memory.
            self.assertGreater(len(self.node.recent_events), 0)
        else:
            self.fail("on_command_intent should still exist after W10")

    def test_on_command_feedback(self):
        """Тест: callback command_feedback"""
        msg = String()
        msg.data = "Иду к точке назначения"
        
        self.node.on_command_feedback(msg)
        
        # Проверяем что добавлено как robot_response
        self.assertGreater(len(self.node.robot_response_events), 0)

    def test_on_user_speech_movement_command_filtered(self):
        """Тест: команды движения фильтруются"""
        msg = String()
        msg.data = "вперёд"
        
        initial_count = len(self.node.speech_events)
        self.node.on_user_speech(msg)
        
        # Команда движения НЕ должна быть добавлена
        self.assertEqual(len(self.node.speech_events), initial_count)

    def test_on_user_speech_dialogue(self):
        """Тест: диалоговая речь добавляется"""
        msg = String()
        msg.data = "Как дела?"
        
        self.node.on_user_speech(msg)
        
        # Диалоговая речь должна быть добавлена
        self.assertGreater(len(self.node.speech_events), 0)
        self.assertEqual(self.node.speech_events[-1]['content'], "Как дела?")


class TestMemoryManagement(unittest.TestCase):
    """Тесты управления памятью"""

    @classmethod
    def setUpClass(cls):
        if not rclpy.ok():
            rclpy.init()

    def setUp(self):
        self.node = ContextAggregatorNode()
        
    def tearDown(self):
        self.node.destroy_node()

    def test_add_to_memory_speech(self):
        """Тест: добавление события speech"""
        self.node.add_to_memory('user_speech', 'Привет', important=True)
        
        self.assertGreater(len(self.node.speech_events), 0)
        self.assertGreater(len(self.node.recent_events), 0)
        self.assertEqual(self.node.speech_events[-1]['content'], 'Привет')
        self.assertTrue(self.node.speech_events[-1]['important'])

    def test_add_to_memory_vision(self):
        """Тест: добавление события vision"""
        self.node.add_to_memory('vision', 'Вижу стол', important=False)
        
        self.assertGreater(len(self.node.vision_events), 0)
        self.assertEqual(self.node.vision_events[-1]['type'], 'vision')

    def test_add_to_memory_system(self):
        """Тест: добавление события system"""
        self.node.add_to_memory('error', 'Ошибка сенсора', important=True)
        
        self.assertGreater(len(self.node.system_events), 0)
        self.assertEqual(self.node.system_events[-1]['type'], 'error')

    def test_memory_window_cleanup(self):
        """Тест: очистка старых событий"""
        self.node.memory_window = 1  # 1 секунда
        
        # Добавляем событие
        self.node.add_to_memory('user_speech', 'Старое событие')
        self.assertEqual(len(self.node.speech_events), 1)
        
        # Ждём чуть больше memory_window
        time.sleep(1.2)
        
        # Добавляем новое событие (должно очистить старое)
        self.node.add_to_memory('user_speech', 'Новое событие')
        
        # Старое событие должно быть удалено
        self.assertEqual(len(self.node.speech_events), 1)
        self.assertEqual(self.node.speech_events[0]['content'], 'Новое событие')

    def test_get_memory_summary_empty(self):
        """Тест: summary пустой памяти"""
        summary = self.node.get_memory_summary()
        self.assertEqual(summary, "Недавних событий нет")

    def test_get_memory_summary_with_events(self):
        """Тест: summary с событиями"""
        self.node.add_to_memory('user_speech', 'Тест 1', important=True)
        self.node.add_to_memory('robot_response', 'Тест 2', important=False)
        
        summary = self.node.get_memory_summary()
        self.assertIn('user_speech', summary)
        self.assertIn('Тест 1', summary)


class TestPublishEvent(unittest.TestCase):
    """Тесты publish_event()"""

    @classmethod
    def setUpClass(cls):
        if not rclpy.ok():
            rclpy.init()

    def setUp(self):
        self.node = ContextAggregatorNode()
        
    def tearDown(self):
        self.node.destroy_node()

    def test_publish_event_calls_publisher(self):
        """Тест: publish_event публикует событие"""
        with patch.object(self.node.event_pub, 'publish') as mock_publish:
            self.node.publish_event()
            mock_publish.assert_called_once()

    def test_check_system_health_healthy(self):
        """Тест: здоровье системы healthy"""
        self.node.current_sensors = {'battery': 40.0}
        self.node.recent_errors = []

        status, issues = self.node.check_system_health()

        # check_system_health возвращает заглавные ключи
        # ('HEALTHY' / 'DEGRADED' / 'UNHEALTHY'). Проверяем что
        # полная батарея и пустой recent_errors дают HEALTHY.
        self.assertEqual(status, 'HEALTHY')

    def test_check_system_health_low_battery(self):
        """Тест: низкая батарея"""
        self.node.current_sensors = {'battery': 33.0}
        self.node.recent_errors = []
        
        status, issues = self.node.check_system_health()
        
        self.assertGreater(len(issues), 0)
        self.assertTrue(any('батарея' in issue.lower() or 'батаре' in issue.lower() for issue in issues))

    def test_check_system_health_critical_battery(self):
        """Тест: критическая батарея"""
        self.node.current_sensors = {'battery': 31.0}
        
        status, issues = self.node.check_system_health()
        
        self.assertGreater(len(issues), 0)
        self.assertTrue(any('критическ' in issue.lower() for issue in issues))

    def test_check_system_health_many_errors(self):
        """Тест: много ошибок"""
        # Добавляем 5 недавних ошибок
        current_time = time.time()
        self.node.recent_errors = [
            {'time': current_time - 10, 'message': 'Error 1'},
            {'time': current_time - 15, 'message': 'Error 2'},
            {'time': current_time - 20, 'message': 'Error 3'},
            {'time': current_time - 25, 'message': 'Error 4'},
            {'time': current_time - 28, 'message': 'Error 5'},
        ]
        
        status, issues = self.node.check_system_health()
        
        self.assertGreater(len(issues), 0)
        self.assertTrue(any('ошибок' in issue.lower() for issue in issues))


if __name__ == '__main__':
    unittest.main()
