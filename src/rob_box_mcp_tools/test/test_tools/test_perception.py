"""
test_perception.py - Unit тесты для инструментов восприятия

Тестирует:
- GetPerceptionContextTool: контекст доступен/недоступен, update_context
- GetBatteryLevelTool: уровень батареи, предупреждения, update_battery

Покрытие: success + error.
"""

import sys
from unittest.mock import MagicMock, Mock

import pytest

# Mock ROS 2 модулей перед импортом tools
for _mod in [
    "rclpy",
    "rclpy.node",
    "rclpy.action",
    "rclpy.qos",
]:
    sys.modules.setdefault(_mod, MagicMock())

from rob_box_mcp_tools.tools.perception import (  # noqa: E402
    GetPerceptionContextTool,
    GetBatteryLevelTool,
)


class _FakeLogger:
    def __init__(self):
        self.messages = []

    def info(self, msg):
        self.messages.append(("info", msg))

    def warning(self, msg):
        self.messages.append(("warning", msg))

    def error(self, msg):
        self.messages.append(("error", msg))


class _FakeNode:
    def __init__(self):
        self._logger = _FakeLogger()

    def get_logger(self):
        return self._logger


@pytest.mark.unit
class TestGetPerceptionContextTool:
    def test_tool_metadata(self):
        tool = GetPerceptionContextTool(_FakeNode())
        assert tool.name == "get_perception_context"
        assert tool.execution_type.value == "medium"
        assert tool.parameters == []

    def test_execute_context_unavailable(self):
        tool = GetPerceptionContextTool(_FakeNode())

        result = tool.execute()

        assert result.success is False
        assert "недоступен" in result.error
        assert "ещё не получены" in result.message

    def test_execute_context_available(self):
        tool = GetPerceptionContextTool(_FakeNode())
        tool.update_context({"vision": "person_detected", "sensors": {"dist": 1.2}})

        result = tool.execute()

        assert result.success is True
        assert result.data["vision"] == "person_detected"
        assert "получен" in result.message

    def test_update_context_overwrites_cache(self):
        tool = GetPerceptionContextTool(_FakeNode())
        tool.update_context({"vision": "empty"})
        tool.update_context({"vision": "person"})

        result = tool.execute()

        assert result.success is True
        assert result.data["vision"] == "person"


@pytest.mark.unit
class TestGetBatteryLevelTool:
    def test_tool_metadata(self):
        tool = GetBatteryLevelTool(_FakeNode())
        assert tool.name == "get_battery_level"
        assert tool.execution_type.value == "fast"
        assert tool.parameters == []

    def test_execute_battery_unavailable(self):
        tool = GetBatteryLevelTool(_FakeNode())

        result = tool.execute()

        assert result.success is False
        assert "недоступны" in result.error

    def test_execute_normal_level(self):
        tool = GetBatteryLevelTool(_FakeNode())
        tool.update_battery(85.0)

        result = tool.execute()

        assert result.success is True
        assert result.data["battery_level"] == 85.0
        assert result.data["warning"] is False
        assert "нормальный" in result.message

    def test_execute_mid_level(self):
        tool = GetBatteryLevelTool(_FakeNode())
        tool.update_battery(30.0)

        result = tool.execute()

        assert result.success is True
        assert result.data["warning"] is False
        assert "30.0%" in result.message

    def test_execute_low_level_warns(self):
        tool = GetBatteryLevelTool(_FakeNode())
        tool.update_battery(15.0)

        result = tool.execute()

        assert result.success is True
        assert result.data["warning"] is True
        assert "Низкий заряд" in result.message

    def test_update_battery_changes_level(self):
        tool = GetBatteryLevelTool(_FakeNode())
        tool.update_battery(10.0)

        result = tool.execute()

        assert result.data["warning"] is True
