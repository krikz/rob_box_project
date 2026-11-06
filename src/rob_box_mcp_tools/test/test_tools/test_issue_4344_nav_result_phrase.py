"""#4344: итог поездки к точке говорит код по исходу Nav2, а не общее «Приехал.» (ADR-0148).

Живой 04.11 11:22 (TG): «Езжай на точку а» - робот уже стоял на «а», Nav2 ответил Goal succeeded за 60 мс,
в чат ушло «Приехал.» (8 знаков) без точки и без того, что ехать было не нужно. Цепочка здесь целиком, без LLM:
тул (фейк Nav2) → ``ToolResult`` → ``command_turn.owner_event`` (как в ``dialog_v2_ports`` и v1-мосте) →
``ActionResult`` → ``respond.phrase_from_result`` - та же фраза уходит и в чат Telegram, и в голос.
"""

import importlib.util
import sys
import types
from pathlib import Path
from unittest.mock import MagicMock

import pytest

from rob_box_dialog import knowledge as kn
from rob_box_dialog.execute import ActionRequest
from rob_box_dialog.respond import ActionResult, phrase_from_result
from rob_box_dialog.execute import owner_event
from rob_box_llm.provider import ToolResult
from rob_box_mcp_tools.core import action_outcome as ao

_PKG = Path(__file__).resolve().parents[2] / "rob_box_mcp_tools"


class _Future:
    def __init__(self, result):
        self._result = result

    def add_done_callback(self, cb):
        cb(self)

    def result(self):
        return self._result


class _GoalHandle:
    def __init__(self, status):
        self.accepted = True
        self._result_future = _Future(types.SimpleNamespace(status=status))

    def get_result_async(self):
        return self._result_future

    def cancel_goal_async(self):
        pass


class _NavClient:
    """Фейк Nav2: принимает цель, шлёт обратную связь ``distance_remaining`` и отвечает статусом."""

    def __init__(self, status, distances=()):
        self._handle = _GoalHandle(status)
        self._distances = distances

    def wait_for_server(self, timeout_sec=0.0):
        return True

    def send_goal_async(self, goal, feedback_callback=None):
        for d in self._distances:
            feedback_callback(types.SimpleNamespace(feedback=types.SimpleNamespace(distance_remaining=d)))
        return _Future(self._handle)


class _Clock:
    """``time.monotonic`` модуля навигации: каждый вызов - на ``step`` секунд позже."""

    def __init__(self, step):
        self.now, self.step = 0.0, step

    def __call__(self):
        self.now += self.step
        return self.now


@pytest.fixture
def nav(monkeypatch):
    for name in (
        "rclpy", "rclpy.action", "geometry_msgs", "geometry_msgs.msg", "nav2_msgs", "nav2_msgs.action",
        "action_msgs", "action_msgs.srv", "action_msgs.msg",
    ):
        monkeypatch.setitem(sys.modules, name, MagicMock())
    tools_pkg = types.ModuleType("rob_box_mcp_tools.tools")
    tools_pkg.__path__ = [str(_PKG / "tools")]
    monkeypatch.setitem(sys.modules, "rob_box_mcp_tools.tools", tools_pkg)
    monkeypatch.delitem(sys.modules, "rob_box_mcp_tools.tools.navigation", raising=False)
    spec = importlib.util.spec_from_file_location("rob_box_mcp_tools.tools.navigation", _PKG / "tools" / "navigation.py")
    module = importlib.util.module_from_spec(spec)
    monkeypatch.setitem(sys.modules, spec.name, module)
    spec.loader.exec_module(module)
    return module


class _Store:
    def __init__(self, points):
        self._points = points

    def get_waypoint(self, name):
        return self._points.get(name)

    def list_waypoints(self):
        return [{"name": n} for n in self._points]


def _no_orphan_halt(reason, rounds=2):
    return ao.CANCEL_NO_ACTIVE_GOAL  # #4435: фейк гашения контроллера (сироты нет)


def _waypoint_tool(nav, client):
    tool = nav.NavigateToWaypointTool.__new__(nav.NavigateToWaypointTool)
    tool.node = MagicMock()
    tool.nav_client = client
    tool.controller_halt = _no_orphan_halt
    tool.waypoint_store = _Store({"а": {"x": 10.459, "y": 6.685, "theta": 0.0}})
    return tool


def _coords_tool(nav, client):
    tool = nav.NavigateToCoordinatesTool.__new__(nav.NavigateToCoordinatesTool)
    tool.node = MagicMock()
    tool.nav_client = client
    tool.controller_halt = _no_orphan_halt
    return tool


def _spoken(tool_name, args, mcp_result, agent="telegram"):
    """Результат MCP-тула → фраза кода, как в живом ходе (транспорт ``core_adapter`` кладёт ``data`` как есть)."""
    request = ActionRequest("t1/1", 0, agent, tool_name, args, kn.TOOL_CLASSES[tool_name], 120.0)
    tool_result = ToolResult(tool_call_id="c1", content=mcp_result.to_json(), is_error=not mcp_result.success,
                             data=mcp_result.data, message=mcp_result.message)
    event = owner_event(request, tool_result)
    result = ActionResult(request.turn_id, 0, agent, tool_name, request.cls, event.status, event.reason, event.data)
    return phrase_from_result(result).text


# ---------------------------------------------------------------- решение «уже на месте» - код


@pytest.mark.unit
@pytest.mark.parametrize(
    "elapsed, distance, kind",
    [
        (0.06, None, kn.NAV_ALREADY_THERE),  # живой случай: Goal succeeded за 60 мс
        (0.5, 3.0, kn.NAV_ALREADY_THERE),  # быстрее секунды - не ехал
        (4.0, 0.2, kn.NAV_ALREADY_THERE),  # доворачивал на месте: путь не длиннее допуска цели
        (4.0, None, kn.NAV_ARRIVED),
        (25.0, 5.4, kn.NAV_ARRIVED),
    ],
)
def test_arrival_kind(elapsed, distance, kind):
    assert ao.arrival_kind(elapsed, distance) == kind


@pytest.mark.unit
def test_start_distance_is_the_longest_feedback_path():
    assert ao.start_distance([0.0, 5.4, 5.1]) == 5.4  # первая обратная связь до расчёта пути - ноль
    assert ao.start_distance([]) is None


@pytest.mark.unit
def test_unmeasured_time_is_not_instant():
    assert ao.nav_data(ao.NAV_SUCCEEDED)[kn.NAV_ARRIVAL_FIELD] == kn.NAV_ARRIVED


@pytest.mark.unit
def test_every_nav_failure_kind_has_its_phrase_in_dialog():
    """Контракт с диалогом: каждый вид неуспеха тула - своя фраза «Не доехал до точки …: причина»."""
    tool_kinds = {ao.nav_outcome(k).kind for k in (
        ao.NAV_ABORTED, ao.NAV_CANCELED, ao.NAV_TIMEOUT, ao.NAV_REJECTED, ao.NAV_NO_RESPONSE,
        ao.NAV_SERVER_UNAVAILABLE, ao.NAV_LOCALIZATION_LOST, "что-то новое")}
    assert tool_kinds | {kn.NAV_UNKNOWN_TARGET} == set(kn.NAV_FAILURE_OUTCOMES)


# ---------------------------------------------------------------- тул → фраза кода


@pytest.mark.unit
def test_already_on_waypoint_says_so(nav, monkeypatch):
    """Живой случай #4344: робот стоит на «а», Nav2 отвечает успехом сразу."""
    monkeypatch.setattr(nav.time, "monotonic", _Clock(0.03))
    res = _waypoint_tool(nav, _NavClient(ao.GOAL_STATUS_SUCCEEDED)).execute("а")
    assert res.success is True and res.data[kn.NAV_ARRIVAL_FIELD] == kn.NAV_ALREADY_THERE
    assert _spoken("navigate_to_waypoint", {"waypoint": "а"}, res) == "Я уже на точке а."


@pytest.mark.unit
def test_drove_to_waypoint_says_arrived_with_name(nav, monkeypatch):
    monkeypatch.setattr(nav.time, "monotonic", _Clock(12.0))
    res = _waypoint_tool(nav, _NavClient(ao.GOAL_STATUS_SUCCEEDED, distances=(0.0, 4.2, 3.9))).execute("а")
    assert res.data["start_distance_m"] == 4.2 and res.data[kn.NAV_ARRIVAL_FIELD] == kn.NAV_ARRIVED
    assert _spoken("navigate_to_waypoint", {"waypoint": "а"}, res) == "Приехал на точку а."


@pytest.mark.unit
@pytest.mark.parametrize(
    "status, said",
    [
        (ao.GOAL_STATUS_ABORTED, "Не доехал до точки а: навигация прервалась."),
        (ao.GOAL_STATUS_CANCELED, "Не доехал до точки а: движение отменили."),
        (ao.GOAL_STATUS_UNKNOWN, "Не уверен, что доехал до точки а: навигация не подтвердила прибытие."),
    ],
)
def test_not_reached_waypoint_says_why(nav, status, said):
    res = _waypoint_tool(nav, _NavClient(status)).execute("а")
    assert res.success is False
    assert _spoken("navigate_to_waypoint", {"waypoint": "а"}, res) == said


@pytest.mark.unit
def test_unknown_waypoint_is_named(nav):
    res = _waypoint_tool(nav, _NavClient(ao.GOAL_STATUS_SUCCEEDED)).execute("б")
    assert _spoken("navigate_to_waypoint", {"waypoint": "б"}, res) == "Не знаю точку б."


@pytest.mark.unit
def test_coordinates_use_the_same_phrases(nav, monkeypatch):
    monkeypatch.setattr(nav.time, "monotonic", _Clock(20.0))
    args = {"x": 10.459, "y": 6.685}
    res = _coords_tool(nav, _NavClient(ao.GOAL_STATUS_SUCCEEDED, distances=(6.0,))).execute(**args)
    assert _spoken("navigate_to_coordinates", args, res) == "Приехал на точку (10.5, 6.7)."
    res = _coords_tool(nav, _NavClient(ao.GOAL_STATUS_ABORTED)).execute(**args)
    assert _spoken("navigate_to_coordinates", args, res) == "Не доехал до точки (10.5, 6.7): навигация прервалась."


@pytest.mark.unit
def test_move_direction_failure_names_the_reason(nav):
    """Общий путь ``_send_nav_goal``: у ``move_direction`` точки нет, но причина отказа - по исходу Nav2."""
    tool = nav.MoveDirectionTool.__new__(nav.MoveDirectionTool)
    tool.node = MagicMock()
    tool.nav_client = _NavClient(ao.GOAL_STATUS_ABORTED)
    tool.controller_halt = _no_orphan_halt
    res = tool.execute("вперёд", 1.0)
    assert _spoken("move_direction", {"direction": "вперёд"}, res) == "Не доехал: навигация прервалась."


@pytest.mark.unit
def test_voice_agent_hears_the_same_phrase(nav, monkeypatch):
    monkeypatch.setattr(nav.time, "monotonic", _Clock(0.03))
    res = _waypoint_tool(nav, _NavClient(ao.GOAL_STATUS_SUCCEEDED)).execute("а")
    assert _spoken("navigate_to_waypoint", {"waypoint": "а"}, res, agent="personality") == "Я уже на точке а."
