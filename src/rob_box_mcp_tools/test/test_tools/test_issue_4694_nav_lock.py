"""#4694: голосовая езда к точке не говорит «еду»/«приехал», пока twist_mux глушит cmd_vel.

Замки ``/joystick_lock`` (priority 100) и ``/teleop_lock`` (priority 90), timeout 0.0 — липкие.
Любой True глушит Nav2 (priority 10). Сообщения ещё нет — не «свободно» и не «занято».
Цель NavigateToPose уходит только когда оба замка уже False.
"""

import importlib.util
import sys
import types
from pathlib import Path
from unittest.mock import MagicMock

import pytest

from rob_box_dialog import knowledge as kn
from rob_box_dialog.execute import ActionRequest, owner_event
from rob_box_dialog.respond import ActionResult, phrase_from_result
from rob_box_llm.provider import ToolResult
from rob_box_mcp_tools.core import action_outcome as ao

_PKG = Path(__file__).resolve().parents[2] / "rob_box_mcp_tools"
_LIE = ("приехал", "Приехал", "Я уже на точке")


@pytest.fixture
def nav(monkeypatch):
    for name in (
        "rclpy",
        "rclpy.action",
        "geometry_msgs",
        "geometry_msgs.msg",
        "nav2_msgs",
        "nav2_msgs.action",
        "action_msgs",
        "action_msgs.srv",
        "action_msgs.msg",
    ):
        monkeypatch.setitem(sys.modules, name, MagicMock())
    tools_pkg = types.ModuleType("rob_box_mcp_tools.tools")
    tools_pkg.__path__ = [str(_PKG / "tools")]
    monkeypatch.setitem(sys.modules, "rob_box_mcp_tools.tools", tools_pkg)
    monkeypatch.delitem(sys.modules, "rob_box_mcp_tools.tools.navigation", raising=False)
    spec = importlib.util.spec_from_file_location(
        "rob_box_mcp_tools.tools.navigation", _PKG / "tools" / "navigation.py"
    )
    module = importlib.util.module_from_spec(spec)
    monkeypatch.setitem(sys.modules, spec.name, module)
    spec.loader.exec_module(module)
    return module


class _Future:
    def __init__(self, result):
        self._result = result

    def add_done_callback(self, cb):
        cb(self)

    def result(self):
        return self._result


class _NavClient:
    def __init__(self):
        self.goals = []

    def wait_for_server(self, timeout_sec=0.0):
        return True

    def send_goal_async(self, goal, feedback_callback=None):
        self.goals.append(goal)
        handle = types.SimpleNamespace(
            accepted=True,
            get_result_async=lambda: _Future(types.SimpleNamespace(status=ao.GOAL_STATUS_SUCCEEDED)),
            cancel_goal_async=lambda: None,
        )
        return _Future(handle)


class _Locks:
    def __init__(self, kind):
        self.kind = kind
        self.calls = 0

    def block(self):
        self.calls += 1
        return self.kind


def _tool(nav, client, locks):
    tool = nav.NavigateToWaypointTool.__new__(nav.NavigateToWaypointTool)
    tool.node = MagicMock()
    tool.nav_client = client
    tool.controller_halt = lambda reason, rounds=2: ao.CANCEL_NO_ACTIVE_GOAL
    tool.localization_watch = None
    tool.twist_locks = locks
    tool.waypoint_store = types.SimpleNamespace(
        get_waypoint=lambda name: {"x": 1.0, "y": 2.0, "theta": 0.0, "frame_id": "map"} if name == "кухня" else None,
        list_waypoints=lambda: [{"name": "кухня"}],
    )
    return tool


def _spoken(mcp_result):
    request = ActionRequest(
        "t1/1", 0, "telegram", "navigate_to_waypoint", {"waypoint": "кухня"}, "action.motion", 120.0
    )
    tool_result = ToolResult(
        tool_call_id="c1",
        content=mcp_result.to_json(),
        is_error=not mcp_result.success,
        data=mcp_result.data,
        message=mcp_result.message,
    )
    event = owner_event(request, tool_result)
    result = ActionResult(
        request.turn_id, 0, "telegram", "navigate_to_waypoint", request.cls, event.status, event.reason, event.data
    )
    return phrase_from_result(result).text


def _not_arrived(text):
    assert text is not None
    assert not any(word in text for word in _LIE)


@pytest.mark.unit
def test_lock_true_does_not_send_goal_and_does_not_say_arrived(nav):
    """Замок True глушит cmd_vel: цель не уходит, фраза — «руль занят», не «приехал»."""
    client = _NavClient()
    locks = _Locks(ao.NAV_LOCK_HELD)
    res = _tool(nav, client, locks).execute("кухня")
    assert client.goals == []
    assert locks.calls == 1
    assert res.success is False
    assert res.data[kn.NAV_OUTCOME_FIELD] == ao.NAV_LOCK_HELD
    assert res.message == "Руль занят, не поеду"
    said = _spoken(res)
    assert said == "Руль занят, не поеду к точке кухня."
    _not_arrived(said)


@pytest.mark.unit
def test_unseen_lock_does_not_send_goal(nav):
    """Сообщения ещё нет: не «свободно». Цель не уходит, фраза — «не вижу замок»."""
    client = _NavClient()
    res = _tool(nav, client, _Locks(ao.NAV_LOCK_UNKNOWN)).execute("кухня")
    assert client.goals == []
    assert res.success is False
    assert res.data[kn.NAV_OUTCOME_FIELD] == ao.NAV_LOCK_UNKNOWN
    said = _spoken(res)
    assert said == "Не вижу замок, не поеду к точке кухня."
    _not_arrived(said)


@pytest.mark.unit
def test_both_locks_false_sends_goal(nav, monkeypatch):
    """Оба False — прежний путь: цель уходит, Nav2 SUCCEEDED → фраза прибытия, не отказ."""
    monkeypatch.setattr(nav.time, "monotonic", lambda: 0.0)  # мгновенный SUCCEEDED = уже на точке
    client = _NavClient()
    res = _tool(nav, client, _Locks(None)).execute("кухня")
    assert len(client.goals) == 1
    assert res.success is True
    assert res.data[kn.NAV_OUTCOME_FIELD] == ao.NAV_SUCCEEDED
    assert _spoken(res) == "Я уже на точке кухня."


@pytest.mark.unit
def test_first_message_timeout_is_the_status_wait_not_nav2(nav):
    """Ожидание первого сообщения — тот же потолок, что у статуса робота (2 с), не 120/200 с Nav2."""
    assert nav._LOCK_FIRST_MSG_TIMEOUT_SEC == 2.0
    assert nav._LOCK_FIRST_MSG_TIMEOUT_SEC < 120.0


@pytest.mark.unit
def test_missing_one_lock_message_is_unknown(nav):
    """Пришёл только один замок — второй неизвестен, «свободно» не выдумываем."""
    assert nav.lock_block(False, None) == ao.NAV_LOCK_UNKNOWN
    assert nav.lock_block(None, False) == ao.NAV_LOCK_UNKNOWN
    assert nav.lock_block(None, None) == ao.NAV_LOCK_UNKNOWN


@pytest.mark.unit
@pytest.mark.parametrize(
    "joy, teleop, kind",
    [
        (True, False, ao.NAV_LOCK_HELD),
        (False, True, ao.NAV_LOCK_HELD),
        (False, False, None),
        (True, True, ao.NAV_LOCK_HELD),
    ],
)
def test_block_reads_last_bool(nav, joy, teleop, kind):
    assert nav.lock_block(joy, teleop) == kind
