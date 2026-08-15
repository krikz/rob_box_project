"""
test_navigation.py - Unit тесты для инструментов навигации

Тестирует:
- NavigateToWaypointTool, NavigateToCoordinatesTool, MoveDirectionTool
- StopNavigationTool, ListWaypointsTool, SaveWaypointTool
- DeleteWaypointTool, ClearWaypointsTool, GetCurrentPoseTool

Покрытие: success + error + invalid params + таймауты.
ROS 2 модули мокаются — тесты работают без робота.
"""

import math
import sys
import threading
import types
from unittest.mock import MagicMock, Mock, patch

import pytest

# Mock ROS 2 modules before importing anything from rob_box_mcp_tools
for _mod in [
    "rclpy",
    "rclpy.node",
    "rclpy.action",
    "rclpy.qos",
    "std_msgs",
    "std_msgs.msg",
    "geometry_msgs",
    "geometry_msgs.msg",
    "nav2_msgs",
    "nav2_msgs.action",
    "action_msgs",
    "action_msgs.srv",
    "action_msgs.msg",
    "tf2_ros",
]:
    sys.modules.setdefault(_mod, MagicMock())

# tf2_ros exception classes должны быть настоящими BaseException-наследниками,
# иначе ``except (LookupException, ...)`` в _lookup_pose падает с TypeError.
class _TfException(Exception):
    """Суррогат LookupException — tf2_ros не установлен в тестовом окружении."""


sys.modules["tf2_ros"].LookupException = _TfException
sys.modules["tf2_ros"].ConnectivityException = _TfException
sys.modules["tf2_ros"].ExtrapolationException = _TfException

from rob_box_mcp_tools.tools.navigation import (  # noqa: E402
    NavigateToWaypointTool,
    NavigateToCoordinatesTool,
    MoveDirectionTool,
    StopNavigationTool,
    ListWaypointsTool,
    SaveWaypointTool,
    DeleteWaypointTool,
    ClearWaypointsTool,
    GetCurrentPoseTool,
    _lookup_pose,
    _send_nav_goal,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


class _FakeFuture:
    """Future, который немедленно вызывает done_callback (для _wait_future)."""

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


class _FakeNode:
    """Минимальная нода: logger + create_client для stop_navigation."""

    def __init__(self):
        self._logger = _FakeLogger()
        self.clients = {}

    def get_logger(self):
        return self._logger

    def create_client(self, srv_type, srv_name, **kwargs):
        client = MagicMock()
        client.srv_type = srv_type
        client.srv_name = srv_name
        client.wait_for_service.return_value = True
        self.clients[srv_name] = client
        return client

    def get_clock(self):
        clock = MagicMock()
        clock.now.return_value.to_msg.return_value = None
        return clock


class _FakeWaypointStore:
    def __init__(self, waypoints=None, active_map=None):
        self._waypoints = waypoints or {}
        self._active_map = active_map or {"name": "test_map", "map_id": "map-1"}

    def get_waypoint(self, name):
        return self._waypoints.get(name)

    def list_waypoints(self):
        return [{"name": n, "x": v["x"], "y": v["y"], "theta": v["theta"]} for n, v in self._waypoints.items()]

    def get_active_map(self):
        return self._active_map

    def save_waypoint(self, name, x, y, theta):
        self._waypoints[name] = {"x": x, "y": y, "theta": theta}
        return True

    def delete_waypoint(self, name):
        if name in self._waypoints:
            del self._waypoints[name]
            return True
        return False

    def clear_waypoints(self):
        count = len(self._waypoints)
        self._waypoints.clear()
        return count


class _FakeTfBuffer:
    """TF buffer, который либо отдаёт transform, либо кидает исключение."""

    def __init__(self, pose=None, exc=None):
        self._pose = pose
        self._exc = exc

    def lookup_transform(self, *args, **kwargs):
        if self._exc is not None:
            raise self._exc
        t = MagicMock()
        t.transform.translation.x = self._pose["x"]
        t.transform.translation.y = self._pose["y"]
        t.transform.rotation.z = math.sin(self._pose["theta"] / 2.0)
        t.transform.rotation.w = math.cos(self._pose["theta"] / 2.0)
        return t


# ---------------------------------------------------------------------------
# NavigateToWaypointTool
# ---------------------------------------------------------------------------


@pytest.mark.unit
class TestNavigateToWaypointTool:
    def test_tool_metadata(self, mock_node):
        tool = NavigateToWaypointTool(mock_node, _FakeWaypointStore())
        assert tool.name == "navigate_to_waypoint"
        assert tool.execution_type.value == "long"
        assert [p.name for p in tool.parameters] == ["waypoint"]

    def test_execute_unknown_waypoint_returns_error_with_available(self, mock_node):
        store = _FakeWaypointStore(waypoints={"кухня": {"x": 1.0, "y": 2.0, "theta": 0.0}})
        tool = NavigateToWaypointTool(mock_node, store)

        result = tool.execute(waypoint="несуществующая")

        assert result.success is False
        assert "Неизвестная точка" in result.error
        assert "кухня" in result.message

    def test_execute_unknown_waypoint_no_waypoints(self, mock_node):
        tool = NavigateToWaypointTool(mock_node, _FakeWaypointStore())

        result = tool.execute(waypoint="кухня")

        assert result.success is False
        assert "нет сохранённых точек" in result.message

    def test_execute_nav_server_unavailable(self, mock_node):
        store = _FakeWaypointStore(waypoints={"кухня": {"x": 1.0, "y": 2.0, "theta": 0.0}})
        tool = NavigateToWaypointTool(mock_node, store)
        tool.nav_client.wait_for_server.return_value = False

        result = tool.execute(waypoint="кухня")

        assert result.success is False
        assert "Nav2 action server недоступен" in result.error

    def test_execute_nav_timeout(self, mock_node):
        store = _FakeWaypointStore(waypoints={"кухня": {"x": 1.0, "y": 2.0, "theta": 0.0}})
        tool = NavigateToWaypointTool(mock_node, store)
        tool.nav_client.wait_for_server.return_value = True
        tool.nav_client.send_goal_async.return_value = _FakeFuture(None)

        result = tool.execute(waypoint="кухня")

        assert result.success is False
        assert "Nav2 не ответил на цель" in result.error


# ---------------------------------------------------------------------------
# NavigateToCoordinatesTool
# ---------------------------------------------------------------------------


@pytest.mark.unit
class TestNavigateToCoordinatesTool:
    def test_tool_metadata(self, mock_node):
        tool = NavigateToCoordinatesTool(mock_node)
        assert tool.name == "navigate_to_coordinates"
        assert [p.name for p in tool.parameters] == ["x", "y", "theta"]

    def test_execute_server_unavailable(self):
        tool = NavigateToCoordinatesTool(_FakeNode())
        tool.nav_client.wait_for_server.return_value = False

        result = tool.execute(x=1.0, y=2.0)

        assert result.success is False
        assert "Nav2 action server недоступен" in result.error

    def test_execute_success(self):
        tool = NavigateToCoordinatesTool(_FakeNode())
        tool.nav_client.wait_for_server.return_value = True

        goal_handle = MagicMock()
        goal_handle.accepted = True
        tool.nav_client.send_goal_async.return_value = _FakeFuture(goal_handle)

        goal_handle.get_result_async.return_value = _FakeFuture(MagicMock())

        result = tool.execute(x=1.5, y=2.5, theta=0.5)

        assert result.success is True
        assert result.data["x"] == 1.5
        assert result.data["y"] == 2.5
        assert "Приехал в точку" in result.message


# ---------------------------------------------------------------------------
# MoveDirectionTool
# ---------------------------------------------------------------------------


@pytest.mark.unit
class TestMoveDirectionTool:
    def test_tool_metadata(self, mock_node):
        tool = MoveDirectionTool(mock_node)
        assert tool.name == "move_direction"
        assert [p.name for p in tool.parameters] == ["direction", "distance"]
        assert "вперёд" in tool.DIRECTIONS

    def test_execute_invalid_direction(self, mock_node):
        tool = MoveDirectionTool(_FakeNode())

        result = tool.execute(direction="диагональ")

        assert result.success is False
        assert "Неизвестное направление" in result.error

    def test_execute_forward_scales_distance(self):
        tool = MoveDirectionTool(_FakeNode())
        tool.nav_client.wait_for_server.return_value = True
        goal_handle = MagicMock()
        goal_handle.accepted = True
        tool.nav_client.send_goal_async.return_value = _FakeFuture(goal_handle)
        goal_handle.get_result_async.return_value = _FakeFuture(MagicMock())

        result = tool.execute(direction="вперёд", distance=2.0)

        assert result.success is True
        assert result.data["direction"] == "вперёд"
        assert result.data["distance"] == 2.0
        assert result.data["relative_coords"]["x"] == 2.0


# ---------------------------------------------------------------------------
# StopNavigationTool
# ---------------------------------------------------------------------------


@pytest.mark.unit
class TestStopNavigationTool:
    def _make_tool(self, service_ready: bool):
        node = MagicMock()
        client = MagicMock()
        client.wait_for_service.return_value = service_ready
        node.create_client.return_value = client
        tool = StopNavigationTool(node)
        tool.cancel_client = client
        return tool

    def test_tool_metadata(self, mock_node):
        tool = StopNavigationTool(mock_node)
        assert tool.name == "stop_navigation"
        assert tool.parameters == []

    def test_execute_service_unavailable(self, mock_node):
        tool = self._make_tool(service_ready=False)

        result = tool.execute()

        assert result.success is False
        assert "Cancel service недоступен" in result.error

    def test_execute_success(self, mock_node):
        tool = self._make_tool(service_ready=True)

        result = tool.execute()

        assert result.success is True
        assert "Останавливаюсь" in result.message
        assert tool.cancel_client.call_async.called


# ---------------------------------------------------------------------------
# ListWaypointsTool
# ---------------------------------------------------------------------------


@pytest.mark.unit
class TestListWaypointsTool:
    def test_tool_metadata(self, mock_node):
        tool = ListWaypointsTool(mock_node, _FakeWaypointStore())
        assert tool.name == "list_waypoints"
        assert tool.parameters == []

    def test_execute_empty(self, mock_node):
        tool = ListWaypointsTool(mock_node, _FakeWaypointStore())
        result = tool.execute()

        assert result.success is True
        assert result.data["waypoints"] == []
        assert "Нет сохранённых точек" in result.message

    def test_execute_with_waypoints(self, mock_node):
        store = _FakeWaypointStore(waypoints={"кухня": {"x": 1.0, "y": 2.0, "theta": 0.0}})
        tool = ListWaypointsTool(mock_node, store)

        result = tool.execute()

        assert result.success is True
        assert len(result.data["waypoints"]) == 1
        assert "кухня" in result.message


# ---------------------------------------------------------------------------
# SaveWaypointTool
# ---------------------------------------------------------------------------


@pytest.mark.unit
class TestSaveWaypointTool:
    def test_execute_while_mapping_returns_error(self, mock_node):
        mapping_state = MagicMock()
        mapping_state.is_mapping.return_value = True
        tool = SaveWaypointTool(mock_node, _FakeWaypointStore(), _FakeTfBuffer(pose={"x": 1, "y": 2, "theta": 0}), mapping_state)

        result = tool.execute(name="кухня")

        assert result.success is False
        assert "картографирование" in result.error

    def test_execute_tf_unavailable(self, mock_node):
        tf = _FakeTfBuffer(exc=_TfException("no transform"))
        tool = SaveWaypointTool(mock_node, _FakeWaypointStore(), tf, None)

        result = tool.execute(name="кухня")

        assert result.success is False
        assert "Не могу определить позицию" in result.error

    def test_execute_success(self, mock_node):
        store = _FakeWaypointStore()
        tf = _FakeTfBuffer(pose={"x": 1.0, "y": 2.0, "theta": 0.5})
        tool = SaveWaypointTool(mock_node, store, tf, None)

        result = tool.execute(name="кухня")

        assert result.success is True
        assert result.data["name"] == "кухня"
        assert "Запомнил" in result.message
        assert "кухня" in store._waypoints


# ---------------------------------------------------------------------------
# DeleteWaypointTool / ClearWaypointsTool
# ---------------------------------------------------------------------------


@pytest.mark.unit
class TestDeleteWaypointTool:
    def test_execute_delete_existing(self, mock_node):
        store = _FakeWaypointStore(waypoints={"кухня": {"x": 1.0, "y": 2.0, "theta": 0.0}})
        tool = DeleteWaypointTool(mock_node, store)

        result = tool.execute(name="кухня")

        assert result.success is True
        assert "удалена" in result.message

    def test_execute_delete_missing(self, mock_node):
        tool = DeleteWaypointTool(mock_node, _FakeWaypointStore())

        result = tool.execute(name="кухня")

        assert result.success is False
        assert "не найдена" in result.error


@pytest.mark.unit
class TestClearWaypointsTool:
    def test_execute_clears(self, mock_node):
        store = _FakeWaypointStore(waypoints={"кухня": {"x": 1.0, "y": 2.0, "theta": 0.0}})
        tool = ClearWaypointsTool(mock_node, store)

        result = tool.execute()

        assert result.success is True
        assert result.data["deleted_count"] == 1
        assert "Удалено 1" in result.message

    def test_execute_empty(self, mock_node):
        tool = ClearWaypointsTool(mock_node, _FakeWaypointStore())

        result = tool.execute()

        assert result.success is True
        assert "Нет точек для удаления" in result.message


# ---------------------------------------------------------------------------
# GetCurrentPoseTool
# ---------------------------------------------------------------------------


@pytest.mark.unit
class TestGetCurrentPoseTool:
    def test_execute_tf_unavailable(self, mock_node):
        tool = GetCurrentPoseTool(mock_node, _FakeTfBuffer(exc=_TfException("no tf")))

        result = tool.execute()

        assert result.success is False
        assert "Не могу определить позицию" in result.error

    def test_execute_success(self, mock_node):
        tool = GetCurrentPoseTool(mock_node, _FakeTfBuffer(pose={"x": 1.0, "y": 2.0, "theta": math.pi / 2}))

        result = tool.execute()

        assert result.success is True
        assert abs(result.data["x"] - 1.0) < 1e-6
        assert abs(result.data["y"] - 2.0) < 1e-6
        assert abs(result.data["theta"] - math.pi / 2) < 1e-6


# ---------------------------------------------------------------------------
# Shared helpers
# ---------------------------------------------------------------------------


@pytest.mark.unit
class TestLookupPose:
    def test_returns_none_when_tf2_ros_missing(self):
        # tf2_ros отсутствует: sys.modules['tf2_ros'] = None заставляет
        # import tf2_ros внутри _lookup_pose бросить ImportError.
        with patch.dict(sys.modules, {"tf2_ros": None}):
            result = _lookup_pose(MagicMock(), _FakeLogger())
        assert result is None

    def test_returns_pose_on_success(self):
        logger = _FakeLogger()
        tf = _FakeTfBuffer(pose={"x": 3.0, "y": 4.0, "theta": 0.0})
        result = _lookup_pose(tf, logger)
        assert result is not None
        assert result["x"] == 3.0
        assert result["y"] == 4.0


@pytest.mark.unit
class TestSendNavGoal:
    def test_server_unavailable(self):
        client = MagicMock()
        client.wait_for_server.return_value = False
        node = _FakeNode()

        result = _send_nav_goal(client, node, 0, 0, 0)

        assert result.success is False
        assert "Nav2 action server недоступен" in result.error

    def test_goal_rejected(self):
        client = MagicMock()
        client.wait_for_server.return_value = True
        goal_handle = MagicMock()
        goal_handle.accepted = False
        client.send_goal_async.return_value = _FakeFuture(goal_handle)
        node = _FakeNode()

        result = _send_nav_goal(client, node, 0, 0, 0)

        assert result.success is False
        assert "Nav2 отклонил цель" in result.error

    def test_navigation_timeout(self):
        client = MagicMock()
        client.wait_for_server.return_value = True
        goal_handle = MagicMock()
        goal_handle.accepted = True
        client.send_goal_async.return_value = _FakeFuture(goal_handle)
        # result_future никогда не завершается → таймаут
        goal_handle.get_result_async.return_value = MagicMock()
        node = _FakeNode()

        result = _send_nav_goal(client, node, 0, 0, 0, timeout=0.01)

        assert result.success is False
        assert "превысила таймаут" in result.error

    def test_success(self):
        client = MagicMock()
        client.wait_for_server.return_value = True
        goal_handle = MagicMock()
        goal_handle.accepted = True
        client.send_goal_async.return_value = _FakeFuture(goal_handle)
        goal_handle.get_result_async.return_value = _FakeFuture(MagicMock())
        node = _FakeNode()

        result = _send_nav_goal(client, node, 1.0, 2.0, 0.0)

        assert result.success is True
