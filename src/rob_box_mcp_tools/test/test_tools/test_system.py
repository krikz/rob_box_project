"""
test_system.py - Unit тесты для системных инструментов

Тестирует:
- GetRobotStatusTool: чтение реальных данных из /odom и /battery_state,
  graceful обработка недоступности топиков.

Паттерн моков ROS-модулей такой же, как в test_animation.py — тесты
работают и без установленного ROS 2 (локально), и в CI-контейнере
(rob-box-ci:humble), где ROS 2 доступен.
"""

import importlib.util
import sys
from unittest.mock import Mock

import pytest

# ROS-модули, которые пакет импортирует на уровне модуля (через tools/__init__).
# Если ROS 2 не установлен — подменяем их моками, чтобы пакет импортировался.
# В CI-контейнере реальные модули доступны, поэтому моки НЕ ставятся.
_ROS_MODULE_SUBMODULES = {
    "rclpy": ["action", "node", "callback_groups", "qos"],
    "geometry_msgs": ["msg"],
    "nav2_msgs": ["action"],
    "action_msgs": ["srv", "msg"],
    "std_msgs": ["msg"],
    "std_srvs": ["srv"],
    "nav_msgs": ["msg"],
    "sensor_msgs": ["msg"],
}


def _install_ros_mocks_if_needed():
    """Подменить ROS-модули моками, только если они реально не установлены."""
    for parent, submodules in _ROS_MODULE_SUBMODULES.items():
        # Модуль уже импортирован (реальный ROS или замокан другим тест-файлом) —
        # не трогаем и не вызываем find_spec (на Mock без __spec__ он падает).
        if parent in sys.modules:
            continue
        if importlib.util.find_spec(parent) is not None:
            continue  # реальный ROS-модуль доступен — не трогаем
        sys.modules.setdefault(parent, Mock())
        for sub in submodules:
            sys.modules.setdefault(f"{parent}.{sub}", Mock())


_install_ros_mocks_if_needed()

from rob_box_mcp_tools.tools.system import GetRobotStatusTool


def _make_odom_msg(x=1.5, y=-2.25, qz=0.0, qw=1.0):
    """Создать мок nav_msgs/Odometry с заданной позицией и yaw-кватернионом."""
    msg = Mock()
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.orientation.x = 0.0
    msg.pose.pose.orientation.y = 0.0
    msg.pose.pose.orientation.z = qz
    msg.pose.pose.orientation.w = qw
    return msg


def _make_battery_msg(percentage=85.0):
    """Создать мок sensor_msgs/BatteryState с заданным процентом заряда."""
    msg = Mock()
    msg.percentage = percentage
    return msg


@pytest.mark.unit
class TestGetRobotStatusTool:
    """Тесты GetRobotStatusTool."""

    def test_tool_creation(self, mock_node):
        """Тест создания инструмента и подписок на ROS-топики."""
        tool = GetRobotStatusTool(mock_node, wait_timeout_sec=0.0)

        assert tool.name == "get_robot_status"
        assert "позиция" in tool.description
        assert len(tool.parameters) == 0
        assert tool.ODOM_TOPIC == "/odom"
        assert tool.BATTERY_TOPIC == "/battery_state"

        # Подписки созданы на оба топика
        assert mock_node.get_subscription("/odom") is not None
        assert mock_node.get_subscription("/battery_state") is not None

    def test_execute_without_data_returns_error(self, mock_node):
        """Без сообщений из топиков — понятная ошибка, а не hardcoded заглушка."""
        tool = GetRobotStatusTool(mock_node, wait_timeout_sec=0.0)

        result = tool.execute()

        assert result.success is False
        assert "/odom" in result.error
        assert "/battery_state" in result.error
        # Никаких фейковых значений
        assert result.data["position"] is None
        assert result.data["battery_level"] is None

    def test_execute_with_odom_only(self, mock_node):
        """Данные /odom приходят, /battery_state нет — позиция реальная, батарея помечена недоступной."""
        tool = GetRobotStatusTool(mock_node, wait_timeout_sec=0.0)

        odom_sub = mock_node.get_subscription("/odom")
        odom_sub.callback(_make_odom_msg(x=2.0, y=3.0))

        result = tool.execute()

        assert result.success is True
        assert result.data["position"]["x"] == 2.0
        assert result.data["position"]["y"] == 3.0
        assert result.data["battery_level"] is None
        assert "/battery_state" in result.data["unavailable_topics"]

    def test_execute_with_battery_only(self, mock_node):
        """Данные /battery_state приходят, /odom нет — батарея реальная, позиция помечена недоступной."""
        tool = GetRobotStatusTool(mock_node, wait_timeout_sec=0.0)

        battery_sub = mock_node.get_subscription("/battery_state")
        battery_sub.callback(_make_battery_msg(percentage=72.5))

        result = tool.execute()

        assert result.success is True
        assert result.data["battery_level"] == 72.5
        assert result.data["position"] is None
        assert "/odom" in result.data["unavailable_topics"]

    def test_execute_full_status(self, mock_node):
        """Оба топика публикуют — возвращаются реальные позиция и батарея."""
        tool = GetRobotStatusTool(mock_node, wait_timeout_sec=0.0)

        odom_sub = mock_node.get_subscription("/odom")
        # Кватернион 90° вокруг Z: qz=qw=0.7071 → theta ≈ π/2
        odom_sub.callback(_make_odom_msg(x=1.5, y=-2.25, qz=0.7071067811865476, qw=0.7071067811865476))

        battery_sub = mock_node.get_subscription("/battery_state")
        battery_sub.callback(_make_battery_msg(percentage=42.0))

        result = tool.execute()

        assert result.success is True
        assert "unavailable_topics" not in result.data
        assert result.data["position"]["x"] == 1.5
        assert result.data["position"]["y"] == -2.25
        assert abs(result.data["position"]["theta"] - (3.141592653589793 / 2.0)) < 1e-6
        assert result.data["battery_level"] == 42.0
        assert result.data["systems"]["navigation"] == "active"

    def test_battery_unknown_percentage(self, mock_node):
        """percentage == -1.0 (неизвестно по ROS-конвенции) — батарея считается недоступной."""
        tool = GetRobotStatusTool(mock_node, wait_timeout_sec=0.0)

        battery_sub = mock_node.get_subscription("/battery_state")
        battery_sub.callback(_make_battery_msg(percentage=-1.0))

        result = tool.execute()

        assert result.data["battery_level"] is None
        assert "/battery_state" in result.data.get("unavailable_topics", [])

    def test_execute_without_node(self):
        """Инструмент без node (например, в unit-тестах) — graceful ошибка недоступности."""
        tool = GetRobotStatusTool(None, wait_timeout_sec=0.0)

        result = tool.execute()

        assert result.success is False
        assert "/odom" in result.error
