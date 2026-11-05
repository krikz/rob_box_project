"""#4435: после провала NavigateToPose наш слой гасит controller_server («сирота» FollowPath).

HIL 05.11: bt_navigator под нагрузкой не дождался подтверждения цели FollowPath
(``default_server_timeout`` 20 мс), бросил NavigateToPose («Goal failed»), а controller_server
ещё 108 с вёл робота по поздно принятой цели. Правило: исход NavigateToPose ≠ успех (после
того как цель ушла в Nav2) → отмена всех целей ``follow_path`` + нулевой ``cmd_vel``.
Тесты — поведение слоя на фейках action-клиента и сервиса отмены, без ROS.
"""

import importlib.util
import sys
import types
from pathlib import Path
from unittest.mock import MagicMock

import pytest

from rob_box_mcp_tools.core import action_outcome as ao

_PKG = Path(__file__).resolve().parents[2] / "rob_box_mcp_tools"


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
    spec = importlib.util.spec_from_file_location(
        "rob_box_mcp_tools.tools.navigation", _PKG / "tools" / "navigation.py"
    )
    module = importlib.util.module_from_spec(spec)
    monkeypatch.setitem(sys.modules, spec.name, module)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, "_HALT_ROUND_GAP_SEC", 0.0)
    monkeypatch.setattr(module, "_HALT_CANCEL_TIMEOUT_SEC", 0.05)
    return module


# ---------------------------------------------------------------- фейки


class _Future:
    def __init__(self, result, done=True):
        self._result, self._done = result, done

    def add_done_callback(self, cb):
        if self._done:
            cb(self)

    def result(self):
        return self._result


class _GoalHandle:
    def __init__(self, accepted=True, result=None, result_done=True):
        self.accepted = accepted
        self._result_future = _Future(result, done=result_done)
        self.cancel_calls = 0

    def get_result_async(self):
        return self._result_future

    def cancel_goal_async(self):
        self.cancel_calls += 1


class _NavClient:
    """Фейк action-клиента NavigateToPose."""

    def __init__(self, server=True, send_future=None):
        self.server, self.send_future = server, send_future

    def wait_for_server(self, timeout_sec=0.0):
        return self.server

    def send_goal_async(self, goal, feedback_callback=None):
        return self.send_future


def _status_client(status, accepted=True):
    return _NavClient(send_future=_Future(_GoalHandle(accepted=accepted, result=types.SimpleNamespace(status=status))))


class _Halt:
    """Фейк ``ControllerHalt``: запоминает вызовы."""

    def __init__(self, kind=ao.CANCEL_CONFIRMED):
        self.kind, self.calls = kind, []

    def __call__(self, reason, rounds=2):
        self.calls.append((reason, rounds))
        return self.kind


def _cancel_resp(code, canceling=()):
    return types.SimpleNamespace(return_code=code, goals_canceling=list(canceling))


class _CancelClient:
    """Фейк сервиса ``/follow_path/_action/cancel_goal``: ответы по очереди на каждый вызов."""

    def __init__(self, responses, ready=True):
        self.responses, self.ready, self.calls = list(responses), ready, 0

    def wait_for_service(self, timeout_sec=0.0):
        return self.ready

    def call_async(self, request):
        self.calls += 1
        return _Future(self.responses.pop(0))


def _real_halt(nav, cancel_client):
    node = MagicMock()
    halt = nav.ControllerHalt(node)
    halt._cancel_client = cancel_client
    return halt, node


# ---------------------------------------------------------------- _send_nav_goal: когда гасить


def _go(nav, client, halt):
    return nav._send_nav_goal(client, MagicMock(), 1.0, 2.0, 0.0, timeout=0.05, halt=halt)


@pytest.mark.unit
@pytest.mark.parametrize(
    "status, kind",
    [
        (ao.GOAL_STATUS_ABORTED, ao.NAV_ABORTED),
        (ao.GOAL_STATUS_CANCELED, ao.NAV_CANCELED),
        (ao.GOAL_STATUS_UNKNOWN, ao.NAV_UNKNOWN_STATUS),
    ],
)
def test_failed_nav_goal_halts_controller(nav, status, kind):
    """Сценарий HIL 3b: NavigateToPose ABORTED → гасим контроллер, исход гашения в данных."""
    halt = _Halt()
    res = _go(nav, _status_client(status), halt)
    assert res.success is False and res.data["outcome"] == kind
    assert halt.calls == [(kind, 2)]
    assert res.data["controller_halt"] == ao.CANCEL_CONFIRMED


@pytest.mark.unit
def test_result_timeout_cancels_nav_goal_and_halts_controller(nav):
    handle = _GoalHandle(result=None, result_done=False)
    halt = _Halt()
    res = _go(nav, _NavClient(send_future=_Future(handle)), halt)
    assert res.data["outcome"] == ao.NAV_TIMEOUT
    assert handle.cancel_calls == 1
    assert [c[0] for c in halt.calls] == [ao.NAV_TIMEOUT]


@pytest.mark.unit
def test_no_reply_to_send_goal_halts_controller(nav):
    """Цель могла дойти до Nav2 позже — гасим и в этом случае."""
    halt = _Halt(kind=ao.CANCEL_NO_ACTIVE_GOAL)
    res = _go(nav, _NavClient(send_future=_Future(None)), halt)
    assert res.data["outcome"] == ao.NAV_NO_RESPONSE
    assert [c[0] for c in halt.calls] == [ao.NAV_NO_RESPONSE]
    assert res.data["controller_halt"] == ao.CANCEL_NO_ACTIVE_GOAL


@pytest.mark.unit
@pytest.mark.parametrize(
    "client",
    [
        _status_client(ao.GOAL_STATUS_SUCCEEDED),
        _status_client(ao.GOAL_STATUS_SUCCEEDED, accepted=False),
        _NavClient(server=False),
    ],
    ids=["succeeded", "rejected", "no_server"],
)
def test_no_halt_when_nothing_was_started_or_goal_reached(nav, client):
    halt = _Halt()
    res = _go(nav, client, halt)
    assert halt.calls == []
    assert "controller_halt" not in (res.data or {})


# ---------------------------------------------------------------- ControllerHalt: что делает


@pytest.mark.unit
def test_halt_cancels_all_follow_path_goals_and_zeroes_cmd_vel(nav):
    cancel = _CancelClient([_cancel_resp(ao.CANCEL_ERROR_NONE, [object()]), _cancel_resp(ao.CANCEL_ERROR_NONE)])
    halt, node = _real_halt(nav, cancel)
    kind = halt("aborted")
    assert kind == ao.CANCEL_CONFIRMED
    assert cancel.calls == 2  # второй раунд — на позднее подтверждение цели
    pub = node.create_publisher.return_value
    assert pub.publish.call_count == 2 * nav._ZERO_TWIST_REPEATS
    for call in pub.publish.call_args_list:
        assert call.args[0] is nav.Twist.return_value  # Twist() — все скорости нулевые
    node.create_client.assert_called_once_with(nav.CancelGoal, "/follow_path/_action/cancel_goal")
    node.create_publisher.assert_called_once_with(nav.Twist, "/cmd_vel", 10)
    node.get_logger.return_value.warning.assert_called_once()


@pytest.mark.unit
def test_halt_catches_late_accepted_orphan_in_second_round(nav):
    """Первая отмена — «целей нет» (запрос FollowPath ещё в очереди), вторая гасит сироту."""
    cancel = _CancelClient(
        [_cancel_resp(ao.CANCEL_ERROR_UNKNOWN_GOAL_ID), _cancel_resp(ao.CANCEL_ERROR_NONE, [object()])]
    )
    halt, _ = _real_halt(nav, cancel)
    assert halt("aborted") == ao.CANCEL_CONFIRMED and cancel.calls == 2


@pytest.mark.unit
def test_halt_idle_controller_reports_no_active_goal(nav):
    cancel = _CancelClient([_cancel_resp(ao.CANCEL_ERROR_NONE)] * 2)
    halt, node = _real_halt(nav, cancel)
    assert halt("aborted") == ao.CANCEL_NO_ACTIVE_GOAL
    node.get_logger.return_value.info.assert_called_once()


@pytest.mark.unit
def test_halt_without_cancel_service_still_publishes_zero(nav):
    cancel = _CancelClient([], ready=False)
    halt, node = _real_halt(nav, cancel)
    assert halt("aborted", rounds=1) == ao.SVC_UNAVAILABLE
    assert cancel.calls == 0
    assert node.create_publisher.return_value.publish.call_count == nav._ZERO_TWIST_REPEATS


# ---------------------------------------------------------------- тулы: проводка


@pytest.mark.unit
@pytest.mark.parametrize("tool_name", ["NavigateToCoordinatesTool", "MoveDirectionTool"])
def test_tool_built_by_constructor_halts_on_abort(nav, tool_name):
    """Проводка в ``__init__``: настоящий ControllerHalt тула гасит контроллер при ABORTED."""
    tool = getattr(nav, tool_name)(MagicMock())
    tool.nav_client = _status_client(ao.GOAL_STATUS_ABORTED)
    cancel = _CancelClient([_cancel_resp(ao.CANCEL_ERROR_NONE, [object()]), _cancel_resp(ao.CANCEL_ERROR_NONE)])
    tool.controller_halt._cancel_client = cancel
    res = tool.execute(1.0, 2.0) if tool_name == "NavigateToCoordinatesTool" else tool.execute("вперёд", 1.0)
    assert res.success is False and res.data["outcome"] == ao.NAV_ABORTED
    assert res.data["controller_halt"] == ao.CANCEL_CONFIRMED and cancel.calls == 2


@pytest.mark.unit
def test_waypoint_tool_halts_on_abort(nav):
    store = MagicMock()
    store.get_waypoint.return_value = {"x": 1.0, "y": 2.0, "theta": 0.0, "frame_id": "map"}
    tool = nav.NavigateToWaypointTool(MagicMock(), store)
    tool.nav_client = _status_client(ao.GOAL_STATUS_ABORTED)
    halt = _Halt()
    tool.controller_halt = halt
    res = tool.execute("кухня")
    assert res.success is False and halt.calls == [(ao.NAV_ABORTED, 2)]


def _stop(nav, nav_cancel_resp, halt):
    nav._CANCEL_ALL_TIMEOUT_SEC = 0.05
    tool = nav.StopNavigationTool.__new__(nav.StopNavigationTool)
    tool.node = MagicMock()
    tool.cancel_client = _CancelClient([nav_cancel_resp])
    tool.controller_halt = halt
    return tool.execute()


@pytest.mark.unit
def test_stop_navigation_halts_orphan_without_nav_goal(nav):
    """«Стоп» при сироте: NavigateToPose целей нет, а контроллер ехал — это остановка, не «и так стою»."""
    halt = _Halt(kind=ao.CANCEL_CONFIRMED)
    res = _stop(nav, _cancel_resp(ao.CANCEL_ERROR_UNKNOWN_GOAL_ID), halt)
    assert halt.calls == [("stop_navigation", 1)]
    assert res.success is True and res.message == "Останавливаюсь"
    assert res.data == {"outcome": ao.CANCEL_CONFIRMED, "controller_halt": ao.CANCEL_CONFIRMED}


@pytest.mark.unit
def test_stop_navigation_idle_everywhere_says_already_standing(nav):
    halt = _Halt(kind=ao.CANCEL_NO_ACTIVE_GOAL)
    res = _stop(nav, _cancel_resp(ao.CANCEL_ERROR_NONE), halt)
    assert res.success is True and "и так стою" in res.message
    assert res.data["outcome"] == ao.CANCEL_NO_ACTIVE_GOAL


@pytest.mark.unit
def test_stop_navigation_halts_controller_even_if_nav_cancel_fails(nav):
    halt = _Halt()
    res = _stop(nav, _cancel_resp(ao.CANCEL_ERROR_REJECTED), halt)
    assert res.success is False and halt.calls == [("stop_navigation", 1)]
