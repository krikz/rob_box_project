"""#3789: навигация и картография не врут об успехе (ADR-0148).

Успех навигации — только ``GoalStatus.STATUS_SUCCEEDED``; aborted / canceled /
timeout / отказ в приёме цели / нет сервера — честный неуспех с причиной.
Сервисы карты: успех по ответу сервиса (ok / отказ / нет сервиса / таймаут) —
сквозные тесты тулов лежат в ``test_mapping.py``. Чистый разбор —
``core/action_outcome.py``; здесь он и ``_send_nav_goal``.
"""

import importlib.util
import sys
import types
from pathlib import Path
from unittest.mock import MagicMock

import pytest

from rob_box_mcp_tools.core import action_outcome as ao

_PKG = Path(__file__).resolve().parents[2] / "rob_box_mcp_tools"


# ---------------------------------------------------------------- чистый разбор


@pytest.mark.unit
def test_status_succeeded_is_the_only_success():
    out = ao.classify_nav_status(ao.GOAL_STATUS_SUCCEEDED)
    assert out.success is True and out.kind == ao.NAV_SUCCEEDED and out.error is None


@pytest.mark.unit
@pytest.mark.parametrize(
    "status, kind",
    [
        (ao.GOAL_STATUS_ABORTED, ao.NAV_ABORTED),
        (ao.GOAL_STATUS_CANCELED, ao.NAV_CANCELED),
        (ao.GOAL_STATUS_UNKNOWN, ao.NAV_UNKNOWN_STATUS),
        (ao.GOAL_STATUS_ACCEPTED, ao.NAV_UNKNOWN_STATUS),
        (ao.GOAL_STATUS_EXECUTING, ao.NAV_UNKNOWN_STATUS),
        (ao.GOAL_STATUS_CANCELING, ao.NAV_UNKNOWN_STATUS),
        (None, ao.NAV_UNKNOWN_STATUS),
        (99, ao.NAV_UNKNOWN_STATUS),
    ],
)
def test_every_other_status_is_failure(status, kind):
    out = ao.classify_nav_status(status)
    assert out.success is False
    assert out.kind == kind
    assert out.error and out.message
    assert "Приехал" not in out.message


@pytest.mark.unit
@pytest.mark.parametrize(
    "kind", [ao.NAV_TIMEOUT, ao.NAV_REJECTED, ao.NAV_NO_RESPONSE, ao.NAV_SERVER_UNAVAILABLE]
)
def test_transport_failures_are_failures_with_reason(kind):
    out = ao.nav_outcome(kind)
    assert out.success is False and out.kind == kind and out.error and out.message


@pytest.mark.unit
def test_service_ok_and_failure_modes():
    ok = ao.classify_service_call(ready=True, completed=True, response=object(), label="svc")
    assert ok.success is True and ok.kind == ao.SVC_OK

    unavailable = ao.classify_service_call(ready=False, completed=False, response=None, label="svc")
    assert (unavailable.success, unavailable.kind) == (False, ao.SVC_UNAVAILABLE)

    timeout = ao.classify_service_call(ready=True, completed=False, response=None, label="svc")
    assert (timeout.success, timeout.kind) == (False, ao.SVC_TIMEOUT)

    empty = ao.classify_service_call(ready=True, completed=True, response=None, label="svc")
    assert (empty.success, empty.kind) == (False, ao.SVC_NO_RESPONSE)

    raised = ao.classify_service_call(ready=True, completed=True, response=None, label="svc", error="boom")
    assert (raised.success, raised.kind) == (False, ao.SVC_NO_RESPONSE) and "boom" in raised.error


@pytest.mark.unit
def test_service_response_with_success_false_is_refusal():
    resp = types.SimpleNamespace(success=False, message="база занята")
    out = ao.classify_service_call(ready=True, completed=True, response=resp, label="svc")
    assert (out.success, out.kind) == (False, ao.SVC_ERROR)
    assert "база занята" in out.error

    resp_ok = types.SimpleNamespace(success=True, message="")
    assert ao.classify_service_call(ready=True, completed=True, response=resp_ok, label="svc").success is True


# ---------------------------------------------------------------- _send_nav_goal


class _Future:
    def __init__(self, result, done=True):
        self._result = result
        self._done = done

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
    def __init__(self, server=True, send_future=None):
        self.server = server
        self.send_future = send_future

    def wait_for_server(self, timeout_sec=0.0):
        return self.server

    def send_goal_async(self, goal):
        return self.send_future


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


def _go(nav, client, timeout=0.05):
    return nav._send_nav_goal(client, MagicMock(), 1.0, 2.0, 0.0, timeout=timeout)


def _client_with(status, accepted=True):
    handle = _GoalHandle(accepted=accepted, result=types.SimpleNamespace(status=status))
    return _NavClient(send_future=_Future(handle)), handle


@pytest.mark.unit
def test_nav_succeeded_is_success(nav):
    client, _ = _client_with(ao.GOAL_STATUS_SUCCEEDED)
    res = _go(nav, client)
    assert res.success is True and res.error is None


@pytest.mark.unit
def test_nav_aborted_is_honest_failure(nav):
    client, _ = _client_with(ao.GOAL_STATUS_ABORTED)
    res = _go(nav, client)
    assert res.success is False
    assert "ABORTED" in res.error
    assert res.data == {"outcome": ao.NAV_ABORTED}
    assert "Приехал" not in (res.message or "")


@pytest.mark.unit
def test_nav_canceled_is_honest_failure(nav):
    client, _ = _client_with(ao.GOAL_STATUS_CANCELED)
    res = _go(nav, client)
    assert res.success is False and res.data == {"outcome": ao.NAV_CANCELED}


@pytest.mark.unit
def test_nav_unknown_status_is_failure(nav):
    client, _ = _client_with(ao.GOAL_STATUS_UNKNOWN)
    res = _go(nav, client)
    assert res.success is False and res.data == {"outcome": ao.NAV_UNKNOWN_STATUS}


@pytest.mark.unit
def test_nav_timeout_is_failure_and_cancels_goal(nav):
    handle = _GoalHandle(result=None, result_done=False)
    res = _go(nav, _NavClient(send_future=_Future(handle)))
    assert res.success is False and res.data == {"outcome": ao.NAV_TIMEOUT}
    assert handle.cancel_calls == 1


@pytest.mark.unit
def test_nav_empty_result_is_timeout_failure(nav):
    handle = _GoalHandle(result=None, result_done=True)
    res = _go(nav, _NavClient(send_future=_Future(handle)))
    assert res.success is False and res.data == {"outcome": ao.NAV_TIMEOUT}


@pytest.mark.unit
def test_nav_goal_rejected_is_failure(nav):
    client, _ = _client_with(ao.GOAL_STATUS_SUCCEEDED, accepted=False)
    res = _go(nav, client)
    assert res.success is False and res.data == {"outcome": ao.NAV_REJECTED}


@pytest.mark.unit
def test_nav_no_reply_to_goal_is_failure(nav):
    res = _go(nav, _NavClient(send_future=_Future(None)))
    assert res.success is False and res.data == {"outcome": ao.NAV_NO_RESPONSE}


@pytest.mark.unit
def test_nav_server_unavailable_is_failure(nav):
    res = _go(nav, _NavClient(server=False))
    assert res.success is False and res.data == {"outcome": ao.NAV_SERVER_UNAVAILABLE}


def _coords_tool(nav, client):
    tool = nav.NavigateToCoordinatesTool.__new__(nav.NavigateToCoordinatesTool)
    tool.node = MagicMock()
    tool.nav_client = client
    return tool


@pytest.mark.unit
def test_navigate_tool_does_not_say_arrived_on_abort(nav):
    """Сквозной случай из карточки: тул не пишет «Приехал» при ABORTED."""
    client, _ = _client_with(ao.GOAL_STATUS_ABORTED)
    res = _coords_tool(nav, client).execute(1.0, 2.0)
    assert res.success is False
    assert "Приехал" not in (res.message or "")


@pytest.mark.unit
def test_navigate_tool_says_arrived_on_succeeded(nav):
    client, _ = _client_with(ao.GOAL_STATUS_SUCCEEDED)
    res = _coords_tool(nav, client).execute(1.0, 2.0)
    assert res.success is True and res.message.startswith("Приехал")


# ---------------------------------------------------------------- stop_navigation (хвост #3789)


def _cancel_resp(code, canceling=()):
    return types.SimpleNamespace(return_code=code, goals_canceling=list(canceling))


@pytest.mark.unit
def test_cancel_confirmed_only_when_server_is_canceling_goals():
    out = ao.classify_cancel_response(_cancel_resp(ao.CANCEL_ERROR_NONE, [object()]))
    assert (out.success, out.kind, out.message) == (True, ao.CANCEL_CONFIRMED, "Останавливаюсь")


@pytest.mark.unit
@pytest.mark.parametrize(
    "code",
    [ao.CANCEL_ERROR_NONE, ao.CANCEL_ERROR_UNKNOWN_GOAL_ID, ao.CANCEL_ERROR_GOAL_TERMINATING],
)
def test_cancel_with_no_goal_to_cancel_is_honest_idle_success(code):
    out = ao.classify_cancel_response(_cancel_resp(code))
    assert (out.success, out.kind) == (True, ao.CANCEL_NO_ACTIVE_GOAL)
    assert "и так стою" in out.message
    assert out.message != "Останавливаюсь"


@pytest.mark.unit
@pytest.mark.parametrize("code", [ao.CANCEL_ERROR_REJECTED, 42, None])
def test_cancel_rejected_or_unknown_code_is_failure(code):
    out = ao.classify_cancel_response(_cancel_resp(code, [object()]))
    assert (out.success, out.kind) == (False, ao.CANCEL_REFUSED)
    assert out.error and "Останавливаюсь" not in out.message


class _RaisingFuture(_Future):
    def result(self):
        raise RuntimeError("сервер упал")


class _CancelClient:
    def __init__(self, ready=True, future=None):
        self.ready = ready
        self.future = future
        self.calls = 0

    def wait_for_service(self, timeout_sec=0.0):
        return self.ready

    def call_async(self, request):
        self.calls += 1
        return self.future


def _stop(nav, client):
    nav._CANCEL_ALL_TIMEOUT_SEC = 0.05
    tool = nav.StopNavigationTool.__new__(nav.StopNavigationTool)
    tool.node = MagicMock()
    tool.cancel_client = client
    return tool.execute()


@pytest.mark.unit
def test_stop_navigation_confirmed_by_server_reply(nav):
    client = _CancelClient(future=_Future(_cancel_resp(ao.CANCEL_ERROR_NONE, [object()])))
    res = _stop(nav, client)
    assert res.success is True and res.message == "Останавливаюсь"
    assert res.data == {"outcome": ao.CANCEL_CONFIRMED} and client.calls == 1


@pytest.mark.unit
def test_stop_navigation_without_active_goal_says_so(nav):
    client = _CancelClient(future=_Future(_cancel_resp(ao.CANCEL_ERROR_UNKNOWN_GOAL_ID)))
    res = _stop(nav, client)
    assert res.success is True and "и так стою" in res.message
    assert res.data == {"outcome": ao.CANCEL_NO_ACTIVE_GOAL}


@pytest.mark.unit
def test_stop_navigation_server_rejected_cancel_is_failure(nav):
    client = _CancelClient(future=_Future(_cancel_resp(ao.CANCEL_ERROR_REJECTED)))
    res = _stop(nav, client)
    assert res.success is False and res.data == {"outcome": ao.CANCEL_REFUSED}
    assert "Останавливаюсь" not in res.message


@pytest.mark.unit
def test_stop_navigation_service_missing_is_failure_without_call(nav):
    client = _CancelClient(ready=False)
    res = _stop(nav, client)
    assert res.success is False and res.data == {"outcome": ao.SVC_UNAVAILABLE}
    assert client.calls == 0 and "Останавливаюсь" not in res.message


@pytest.mark.unit
def test_stop_navigation_no_reply_in_time_is_failure(nav):
    client = _CancelClient(future=_Future(None, done=False))
    res = _stop(nav, client)
    assert res.success is False and res.data == {"outcome": ao.SVC_TIMEOUT}
    assert "Останавливаюсь" not in res.message


@pytest.mark.unit
def test_stop_navigation_empty_reply_is_failure(nav):
    res = _stop(nav, _CancelClient(future=_Future(None)))
    assert res.success is False and res.data == {"outcome": ao.SVC_NO_RESPONSE}


@pytest.mark.unit
def test_stop_navigation_future_exception_is_failure(nav):
    res = _stop(nav, _CancelClient(future=_RaisingFuture(None)))
    assert res.success is False and res.data == {"outcome": ao.SVC_NO_RESPONSE}
    assert "сервер упал" in res.error
