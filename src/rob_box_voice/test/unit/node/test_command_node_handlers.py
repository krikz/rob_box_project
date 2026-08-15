"""
test_command_node_handlers.py — Unit-тесты обработчиков CommandNode.

TASK-052 (#819): покрывает реальные (не-заглушечные) хендлеры:
  - handle_status     → позиция из TF (map→base_link) или кэша /odom
  - _lookup_map_pose  → TF lookup + обработка недоступности
  - odom_callback     → кэширование одометрии
  - vision_context_callback → кэширование vision context
  - handle_vision     → публикация запроса детекции + реальный ответ
  - handle_follow     → активация режима следования + stop отключает

Не требует ROS2 — rclpy/ROS-модули замоканы в conftest.py.
"""

import json
import math
from unittest.mock import MagicMock, patch

import pytest

from rob_box_voice.command_node import CommandNode
from rob_box_voice.core.command_parser import Command, IntentType


# ─────────────────────────────────────────────────────────────────────────────
#  Fixture: CommandNode без __init__ (объект через object.__new__)
# ─────────────────────────────────────────────────────────────────────────────

@pytest.fixture
def node():
    """Минимальный CommandNode без реального ROS-взаимодействия."""
    n = object.__new__(CommandNode)
    logger = MagicMock()
    n.get_logger = lambda: logger

    # Параметры
    n.enable_navigation = True
    n.enable_follow = True
    n.enable_vision = True

    # Публишеры
    n.feedback_pub = MagicMock()
    n.vision_request_pub = MagicMock()
    n.follow_pub = MagicMock()
    n.intent_pub = MagicMock()
    n.cmd_vel_pub = MagicMock()

    # Состояние
    n._last_odom = None
    n._last_vision_context = None
    n._tf_buffer = None
    n.follow_active = False
    n.current_goal_handle = None
    n.dialogue_state = 'IDLE'
    n.nav_client = MagicMock()
    n.cancel_client = MagicMock()
    n.cancel_client.wait_for_service.return_value = False
    n.create_client = MagicMock()

    return n


def _status_command() -> Command:
    return Command(intent=IntentType.STATUS, text='где ты', entities={}, confidence=0.9)


def _vision_command() -> Command:
    return Command(intent=IntentType.VISION, text='что видишь', entities={}, confidence=0.9)


def _follow_command() -> Command:
    return Command(intent=IntentType.FOLLOW, text='следуй за мной', entities={}, confidence=0.9)


# ─────────────────────────────────────────────────────────────────────────────
#  handle_status
# ─────────────────────────────────────────────────────────────────────────────

class TestHandleStatus:
    def test_status_uses_tf_pose(self, node):
        """Позиция берётся из TF map→base_link."""
        node._tf_buffer = MagicMock()
        t = MagicMock()
        t.transform.translation.x = 1.5
        t.transform.translation.y = -2.0
        # yaw = pi/2 → quaternion z=sin(pi/4), w=cos(pi/4)
        t.transform.rotation.z = math.sin(math.pi / 4)
        t.transform.rotation.w = math.cos(math.pi / 4)
        node._tf_buffer.lookup_transform.return_value = t

        mock_tf2 = MagicMock()
        mock_tf2.Time.return_value = None
        mock_tf2.Duration.return_value = None
        with patch.dict('sys.modules', {'tf2_ros': mock_tf2}):
            node.handle_status(_status_command())

        msg = node.feedback_pub.publish.call_args[0][0]
        assert '1.50' in msg.data
        assert '-2.00' in msg.data
        assert '90' in msg.data  # 90°

    def test_status_falls_back_to_odom(self, node):
        """TF недоступен → используется кэш /odom."""
        node._last_odom = {'x': 0.3, 'y': 0.4, 'theta': 0.0}

        node.handle_status(_status_command())

        msg = node.feedback_pub.publish.call_args[0][0]
        assert '0.30' in msg.data
        assert '0.40' in msg.data

    def test_status_unknown_position(self, node):
        """Нет ни TF, ни /odom → честное сообщение о недоступности."""
        node.handle_status(_status_command())

        msg = node.feedback_pub.publish.call_args[0][0]
        assert 'локализация недоступна' in msg.data

    def test_status_tf_buffer_none(self, node):
        """TF буфер не инициализирован → fallback на /odom, не падает."""
        node._tf_buffer = None
        node._last_odom = {'x': 5.0, 'y': 6.0, 'theta': 1.0}
        node.handle_status(_status_command())
        msg = node.feedback_pub.publish.call_args[0][0]
        assert '5.00' in msg.data


# ─────────────────────────────────────────────────────────────────────────────
#  odom_callback
# ─────────────────────────────────────────────────────────────────────────────

class TestOdomCallback:
    def test_odom_callback_caches_pose(self, node):
        """Кэш одометрии сохраняет x/y/yaw."""
        msg = MagicMock()
        msg.pose.pose.position.x = 1.0
        msg.pose.pose.position.y = 2.0
        msg.pose.pose.orientation.z = 0.0
        msg.pose.pose.orientation.w = 1.0

        node.odom_callback(msg)

        assert node._last_odom == {'x': 1.0, 'y': 2.0, 'theta': 0.0}


# ─────────────────────────────────────────────────────────────────────────────
#  vision_context_callback
# ─────────────────────────────────────────────────────────────────────────────

class TestVisionContextCallback:
    def test_valid_json_cached(self, node):
        msg = MagicMock()
        msg.data = json.dumps({'objects': ['стол', 'стул'], 'scene': 'kitchen'})
        node.vision_context_callback(msg)
        assert node._last_vision_context == {'objects': ['стол', 'стул'], 'scene': 'kitchen'}

    def test_invalid_json_ignored(self, node):
        msg = MagicMock()
        msg.data = 'not-json{{{'
        node.vision_context_callback(msg)
        assert node._last_vision_context is None


# ─────────────────────────────────────────────────────────────────────────────
#  handle_vision
# ─────────────────────────────────────────────────────────────────────────────

class TestHandleVision:
    def test_disabled_vision(self, node):
        node.enable_vision = False
        node.handle_vision(_vision_command())
        node.vision_request_pub.publish.assert_not_called()
        msg = node.feedback_pub.publish.call_args[0][0]
        assert 'недоступна' in msg.data

    def test_vision_publishes_request(self, node):
        """Запрос детекции уходит в OAK-D pipeline."""
        node.handle_vision(_vision_command())
        node.vision_request_pub.publish.assert_called_once()
        req = node.vision_request_pub.publish.call_args[0][0]
        assert json.loads(req.data)['action'] == 'detect_objects'

    def test_vision_reports_objects_from_context(self, node):
        node._last_vision_context = {'objects': ['чашка', 'книга']}
        node.handle_vision(_vision_command())
        msg = node.feedback_pub.publish.call_args[0][0]
        assert 'чашка' in msg.data
        assert 'книга' in msg.data

    def test_vision_no_objects(self, node):
        node._last_vision_context = {'objects': []}
        node.handle_vision(_vision_command())
        msg = node.feedback_pub.publish.call_args[0][0]
        assert 'не вижу объектов' in msg.data


# ─────────────────────────────────────────────────────────────────────────────
#  handle_follow
# ─────────────────────────────────────────────────────────────────────────────

class TestHandleFollow:
    def test_disabled_follow(self, node):
        node.enable_follow = False
        node.handle_follow(_follow_command())
        node.follow_pub.publish.assert_not_called()
        msg = node.feedback_pub.publish.call_args[0][0]
        assert 'недоступна' in msg.data

    def test_follow_activates(self, node):
        """Режим следования активируется и публикуется 'start'."""
        node.handle_follow(_follow_command())
        assert node.follow_active is True
        node.follow_pub.publish.assert_called_once()
        msg = node.follow_pub.publish.call_args[0][0]
        assert msg.data == 'start'

    def test_stop_deactivates_follow(self, node):
        """handle_stop публикует 'stop' и сбрасывает follow_active."""
        node.follow_active = True
        stop_cmd = Command(intent=IntentType.STOP, text='стоп', entities={}, confidence=0.9)

        node.handle_stop(stop_cmd)

        assert node.follow_active is False
        # stop-сообщение для follow controller
        stop_msg = node.follow_pub.publish.call_args[0][0]
        assert stop_msg.data == 'stop'
