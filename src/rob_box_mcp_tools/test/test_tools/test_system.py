"""
test_system.py - Unit тесты для инструментов системы

Тестирует:
- SetVolumeTool, SetPitchTool, SetSpeedTool (ROS service calls)
- GetCurrentTimeTool (Python datetime)
- GetRobotStatusTool (stub)

Покрытие: success + error + invalid params + таймауты.
ROS 2 модули мокаются — тесты работают без робота.
"""

import sys
from unittest.mock import MagicMock, Mock, patch

import pytest

# Mock ROS 2 модулей перед импортом tools
for _mod in [
    "rclpy",
    "rclpy.node",
    "rclpy.action",
    "rclpy.qos",
    "rcl_interfaces",
    "rcl_interfaces.msg",
    "rcl_interfaces.srv",
]:
    sys.modules.setdefault(_mod, MagicMock())

from rob_box_mcp_tools.tools.system import (  # noqa: E402
    SetVolumeTool,
    SetPitchTool,
    SetSpeedTool,
    GetCurrentTimeTool,
    GetRobotStatusTool,
    _wait_future,
)

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


class _ImmediateFuture:
    """Future, который немедленно вызывает done_callback."""

    def __init__(self, result=None):
        self._result = result

    def add_done_callback(self, callback):
        callback(self)

    def result(self):
        return self._result


class _FakeLogger:
    def __init__(self):
        self.messages = []

    def info(self, msg):
        self.messages.append(("info", msg))

    def warning(self, msg):
        self.messages.append(("warning", msg))

    def error(self, msg):
        self.messages.append(("error", msg))

    def debug(self, msg):
        self.messages.append(("debug", msg))


class _FakeClient:
    def __init__(self, ready=True):
        self.ready = ready
        self._call_future = _ImmediateFuture(_FakeParamsResponse(1.0))

    def wait_for_service(self, timeout_sec):
        return self.ready

    def call_async(self, request):
        return self._call_future


class _FakeValue:
    def __init__(self, double_value):
        self.double_value = double_value


class _FakeValues:
    def __init__(self, value):
        self.values = [_FakeValue(value)]


class _FakeParamsResponse:
    """Ответ GetParameters.Response: .values[0].double_value."""

    def __init__(self, value):
        self.values = [_FakeValue(value)]


class _FakeSetResult:
    def __init__(self, successful=True):
        self.successful = successful


class _FakeSetResponse:
    def __init__(self, successful=True):
        self.results = [_FakeSetResult(successful)]


class _FakeNode:
    """Нода, у которой create_client() отдаёт клиент с настраиваемыми future.

    Инструменты (SetVolumeTool и др.) создают клиенты сами через
    ``node.create_client()`` — поэтому настройка future делается через
    ``tool.get_params_client._call_future`` / ``tool.set_params_client._call_future``.
    """

    def __init__(self, service_ready=True, current_value=0.0):
        self._logger = _FakeLogger()
        self.clients = {}
        self._service_ready = service_ready
        self._current_value = current_value

    def get_logger(self):
        return self._logger

    def create_client(self, srv_type, srv_name, **kwargs):
        client = _FakeClient(self._service_ready)
        # get_parameters клиент отвечает текущим значением параметра
        if srv_name.endswith("get_parameters"):
            client._call_future = _ImmediateFuture(_FakeParamsResponse(self._current_value))
        self.clients[srv_name] = client
        return client


# ---------------------------------------------------------------------------
# SetVolumeTool
# ---------------------------------------------------------------------------


@pytest.mark.unit
class TestSetVolumeTool:
    def test_tool_metadata(self):
        tool = SetVolumeTool(_FakeNode())
        assert tool.name == "set_volume"
        assert tool.execution_type.value == "fast"
        assert [p.name for p in tool.parameters] == ["action"]
        assert tool.parameters[0].enum == ["louder", "quieter", "max", "normal"]

    def test_execute_service_unavailable(self):
        node = _FakeNode(service_ready=False)
        tool = SetVolumeTool(node)

        result = tool.execute(action="louder")

        assert result.success is False
        assert "TTS параметры недоступны" in result.error

    def test_execute_unknown_action(self):
        tool = SetVolumeTool(_FakeNode())

        result = tool.execute(action="unknown")

        assert result.success is False
        assert "Неизвестное действие" in result.error

    def test_execute_louder_success(self):
        node = _FakeNode(current_value=1.0)
        tool = SetVolumeTool(node)
        tool.set_params_client._call_future = _ImmediateFuture(_FakeSetResponse(True))

        result = tool.execute(action="louder")

        assert result.success is True
        assert result.data["old_volume"] == 1.0
        assert result.data["new_volume"] == 4.0  # 1.0 + 3.0
        assert "громче" in result.message.lower()

    def test_execute_max_clamps(self):
        node = _FakeNode(current_value=5.0)
        tool = SetVolumeTool(node)
        tool.set_params_client._call_future = _ImmediateFuture(_FakeSetResponse(True))

        result = tool.execute(action="louder")

        assert result.success is True
        assert result.data["new_volume"] == 6.0  # min(5+3, 6)

    def test_execute_quieter_clamps_low(self):
        node = _FakeNode(current_value=-18.0)
        tool = SetVolumeTool(node)
        tool.set_params_client._call_future = _ImmediateFuture(_FakeSetResponse(True))

        result = tool.execute(action="quieter")

        assert result.success is True
        assert result.data["new_volume"] == -20.0  # max(-18-3, -20)

    def test_execute_already_max(self):
        node = _FakeNode(current_value=6.0)
        tool = SetVolumeTool(node)

        result = tool.execute(action="louder")

        assert result.success is True
        assert "уже максимальная" in result.message

    def test_execute_get_params_timeout(self):
        node = _FakeNode()
        tool = SetVolumeTool(node)
        tool.get_params_client._call_future = MagicMock()  # никогда не завершится

        result = tool.execute(action="louder")

        assert result.success is False
        assert "Не удалось получить" in result.error

    def test_execute_set_params_failure(self):
        node = _FakeNode(current_value=1.0)
        tool = SetVolumeTool(node)
        tool.set_params_client._call_future = _ImmediateFuture(_FakeSetResponse(False))

        result = tool.execute(action="louder")

        assert result.success is False
        assert "Не удалось установить громкость" in result.error


# ---------------------------------------------------------------------------
# SetPitchTool
# ---------------------------------------------------------------------------


@pytest.mark.unit
class TestSetPitchTool:
    def test_tool_metadata(self):
        tool = SetPitchTool(_FakeNode())
        assert tool.name == "set_pitch"
        assert tool.parameters[0].enum == ["higher", "lower", "normal"]

    def test_execute_service_unavailable(self):
        node = _FakeNode(service_ready=False)
        tool = SetPitchTool(node)

        result = tool.execute(action="higher")

        assert result.success is False
        assert "TTS параметры недоступны" in result.error

    def test_execute_unknown_action(self):
        tool = SetPitchTool(_FakeNode())

        result = tool.execute(action="unknown")

        assert result.success is False
        assert "Неизвестное действие" in result.error

    def test_execute_higher_success(self):
        node = _FakeNode(current_value=1.0)
        tool = SetPitchTool(node)
        tool.set_params_client._call_future = _ImmediateFuture(_FakeSetResponse(True))

        result = tool.execute(action="higher")

        assert result.success is True
        assert result.data["new_pitch"] == 1.2
        assert "выше" in result.message.lower()

    def test_execute_higher_clamps(self):
        node = _FakeNode(current_value=1.9)
        tool = SetPitchTool(node)
        tool.set_params_client._call_future = _ImmediateFuture(_FakeSetResponse(True))

        result = tool.execute(action="higher")

        assert result.success is True
        assert result.data["new_pitch"] == 2.0

    def test_execute_lower_clamps(self):
        node = _FakeNode(current_value=0.6)
        tool = SetPitchTool(node)
        tool.set_params_client._call_future = _ImmediateFuture(_FakeSetResponse(True))

        result = tool.execute(action="lower")

        assert result.success is True
        assert result.data["new_pitch"] == 0.5

    def test_execute_normal_resets(self):
        node = _FakeNode(current_value=1.5)
        tool = SetPitchTool(node)
        tool.set_params_client._call_future = _ImmediateFuture(_FakeSetResponse(True))

        result = tool.execute(action="normal")

        assert result.success is True
        assert result.data["new_pitch"] == 1.0

    def test_execute_already_max(self):
        node = _FakeNode(current_value=2.0)
        tool = SetPitchTool(node)

        result = tool.execute(action="higher")

        assert result.success is True
        assert "уже максимально высокий" in result.message

    def test_execute_set_params_failure(self):
        node = _FakeNode(current_value=1.0)
        tool = SetPitchTool(node)
        tool.set_params_client._call_future = _ImmediateFuture(_FakeSetResponse(False))

        result = tool.execute(action="higher")

        assert result.success is False
        assert "Не удалось установить pitch" in result.error


# ---------------------------------------------------------------------------
# SetSpeedTool
# ---------------------------------------------------------------------------


@pytest.mark.unit
class TestSetSpeedTool:
    def test_tool_metadata(self):
        tool = SetSpeedTool(_FakeNode())
        assert tool.name == "set_speed"
        assert tool.parameters[0].enum == ["faster", "slower", "normal"]

    def test_execute_service_unavailable(self):
        node = _FakeNode(service_ready=False)
        tool = SetSpeedTool(node)

        result = tool.execute(action="faster")

        assert result.success is False
        assert "TTS параметры недоступны" in result.error

    def test_execute_faster_success(self):
        node = _FakeNode(current_value=1.0)
        tool = SetSpeedTool(node)
        tool.set_params_client._call_future = _ImmediateFuture(_FakeSetResponse(True))

        result = tool.execute(action="faster")

        assert result.success is True
        assert result.data["new_speed"] == 1.2
        assert "быстрее" in result.message.lower()

    def test_execute_faster_clamps(self):
        node = _FakeNode(current_value=1.9)
        tool = SetSpeedTool(node)
        tool.set_params_client._call_future = _ImmediateFuture(_FakeSetResponse(True))

        result = tool.execute(action="faster")

        assert result.success is True
        assert result.data["new_speed"] == 2.0

    def test_execute_unknown_action(self):
        tool = SetSpeedTool(_FakeNode())

        result = tool.execute(action="unknown")

        assert result.success is False
        assert "Неизвестное действие" in result.error

    def test_execute_already_slowest(self):
        node = _FakeNode(current_value=0.5)
        tool = SetSpeedTool(node)

        result = tool.execute(action="slower")

        assert result.success is True
        assert "уже минимальная" in result.message

    def test_execute_normal_resets(self):
        node = _FakeNode(current_value=1.5)
        tool = SetSpeedTool(node)
        tool.set_params_client._call_future = _ImmediateFuture(_FakeSetResponse(True))

        result = tool.execute(action="normal")

        assert result.success is True
        assert result.data["new_speed"] == 1.0


# ---------------------------------------------------------------------------
# GetCurrentTimeTool
# ---------------------------------------------------------------------------


@pytest.mark.unit
class TestGetCurrentTimeTool:
    def test_tool_metadata(self):
        tool = GetCurrentTimeTool()
        assert tool.name == "get_current_time"
        assert tool.execution_type.value == "instant"
        assert tool.parameters == []

    def test_execute_returns_time_data(self):
        tool = GetCurrentTimeTool()
        result = tool.execute()

        assert result.success is True
        assert result.data["time"]  # "HH:MM"
        assert result.data["date"]
        assert result.data["weekday"] in [
            "понедельник", "вторник", "среда", "четверг",
            "пятница", "суббота", "воскресенье",
        ]
        assert result.data["period"] in ["утро", "день", "вечер", "ночь"]
        assert "iso" in result.data
        assert result.data["time"] in result.message


# ---------------------------------------------------------------------------
# GetRobotStatusTool
# ---------------------------------------------------------------------------


@pytest.mark.unit
class TestGetRobotStatusTool:
    def test_tool_metadata(self):
        tool = GetRobotStatusTool()
        assert tool.name == "get_robot_status"
        assert tool.execution_type.value == "medium"
        assert tool.parameters == []

    def test_execute_returns_status(self):
        tool = GetRobotStatusTool()
        result = tool.execute()

        assert result.success is True
        assert result.data["position"] == {"x": 0.0, "y": 0.0, "theta": 0.0}
        assert result.data["battery_level"] == 85.0
        assert result.data["systems"]["navigation"] == "active"


# ---------------------------------------------------------------------------
# _wait_future helper
# ---------------------------------------------------------------------------


@pytest.mark.unit
class TestSystemWaitFuture:
    def test_returns_true_when_future_completes(self):
        future = _ImmediateFuture()
        assert _wait_future(future, timeout_sec=1.0) is True

    def test_returns_false_on_timeout(self):
        future = MagicMock()  # add_done_callback никогда не вызывает callback
        assert _wait_future(future, timeout_sec=0.01) is False
