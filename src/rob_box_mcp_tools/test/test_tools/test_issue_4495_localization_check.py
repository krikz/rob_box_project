"""#4495: «приехал» только если поза на карте за поездку не теряла опору (ADR-0148: решает код).

HIL Depot 06.11 (bag ``shisyun_s1``): на развороте ICP-одометрия rtabmap ушла в ``lost`` и не вернулась
(8309 кадров из 8946 с ковариацией 9999), поза поехала на колёсной одометрии, а Nav2 трижды ответил
``SUCCEEDED`` в 3.5–7.5 м от цели. Здесь — поведение модуля на синтетике по этим замерам и тула на
фейке Nav2: сторож видит потерю и тул говорит «не уверен, что доехал», а не «приехал».
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
from rob_box_mcp_tools.core import localization_check as lc

_PKG = Path(__file__).resolve().parents[2] / "rob_box_mcp_tools"
LOST = lc.LIMITS.odom_lost_covariance  # так rtabmap метит потерянный кадр /odom (замер 06.11: 1e4)
GOOD = 1e-4  # ковариация здорового кадра ICP в том же прогоне (~9e-5)


def _track(odom=(), map_to_odom=()):
    track = lc.LocalizationTrack()
    for cov in odom:
        track.on_odom_covariance(cov)
    for pose in map_to_odom:
        track.on_map_to_odom(*pose)
    return track


# ---------------------------------------------------------------- модуль


@pytest.mark.unit
def test_healthy_trip_is_confident():
    """M1/M2 06.11: потерь 0, поправка карты дрожит на сантиметры — ошибка по правде 0.17–0.25 м."""
    v = _track([GOOD] * 400, [(0.0, 0.0, 0.1), (0.003, -0.002, 0.05), (0.0, 0.01, 0.1)]).verdict()
    assert v.status == lc.LOC_CONFIDENT and v.trusted
    assert v.detail["lost_frames"] == 0 and v.detail["odom_frames"] == 400


@pytest.mark.unit
def test_odometry_lost_for_the_trip_is_not_trusted():
    """M3 06.11 без сброса одометрии: 356 потерянных кадров из 361, Nav2 SUCCEEDED, истинная ошибка 3.5 м."""
    v = _track([GOOD] * 5 + [LOST] * 356).verdict()
    assert v.status == lc.LOC_ODOM_LOST and not v.trusted
    assert v.detail["lost_frames"] == 356 and v.detail["longest_lost_run"] == 356


@pytest.mark.unit
def test_loss_with_odometry_reset_is_tolerated():
    """R4 06.11 со сбросом (Odom/ResetCountdown=1): два случая «потерян 9999 + сброс 19998» на 766 кадров,
    ошибка по правде 0.17 м — робот прав, говоря «приехал»."""
    odom = [GOOD] * 300 + [LOST, 2 * LOST] + [GOOD] * 5 + [LOST, 2 * LOST] + [GOOD] * 457
    v = _track(odom).verdict()
    assert v.status == lc.LOC_CONFIDENT
    assert v.detail["lost_frames"] == 4 and v.detail["longest_lost_run"] == 2


@pytest.mark.unit
def test_long_run_of_lost_frames_is_not_trusted_even_if_recovered():
    odom = [GOOD] * 500 + [LOST] * (lc.LIMITS.max_lost_run + 1) + [GOOD] * 500
    v = _track(odom).verdict()
    assert v.status == lc.LOC_ODOM_LOST and v.detail["longest_lost_run"] == lc.LIMITS.max_lost_run + 1


@pytest.mark.unit
def test_many_short_losses_are_not_trusted():
    """Сброс за сбросом (каждый кадр — догадка колёс) — доля потерь выше допуска, хоть серии и короткие."""
    odom = ([GOOD] * 8 + [LOST, LOST]) * 30
    v = _track(odom).verdict()
    assert v.detail["longest_lost_run"] == 2
    assert v.status == lc.LOC_ODOM_LOST


@pytest.mark.unit
def test_nan_covariance_counts_as_lost():
    v = _track([GOOD] * 10 + [float("nan")] * (lc.LIMITS.max_lost_run + 1)).verdict()
    assert v.status == lc.LOC_ODOM_LOST


@pytest.mark.unit
def test_map_jump_beyond_limit_is_not_trusted():
    """t=374.5 06.11: поправка карты прыгнула на 2.29 м и 10.5° за секунду."""
    v = _track([GOOD] * 20, [(0.90, 1.91, -15.2), (0.80, 4.20, -25.7)]).verdict()
    assert v.status == lc.LOC_MAP_JUMP and not v.trusted
    assert v.detail["map_jumps"] == 1 and v.detail["max_map_jump_m"] == pytest.approx(2.292, abs=1e-3)


@pytest.mark.unit
def test_map_yaw_jump_alone_is_not_trusted():
    v = _track([GOOD] * 20, [(0.0, 0.0, 0.0), (0.0, 0.0, lc.LIMITS.max_map_jump_deg + 1.0)]).verdict()
    assert v.status == lc.LOC_MAP_JUMP


@pytest.mark.unit
def test_yaw_jump_across_180_is_measured_the_short_way():
    v = _track([GOOD] * 20, [(0.0, 0.0, 179.0), (0.0, 0.0, -179.0)]).verdict()
    assert v.status == lc.LOC_CONFIDENT and v.detail["max_map_jump_deg"] == pytest.approx(2.0)


@pytest.mark.unit
def test_odometry_loss_wins_over_map_jump():
    v = _track([LOST] * 10, [(0.0, 0.0, 0.0), (3.0, 0.0, 0.0)]).verdict()
    assert v.status == lc.LOC_ODOM_LOST


@pytest.mark.unit
def test_no_signals_is_unchecked_and_does_not_block():
    """Нет ни кадра (робот уже стоял на точке, ответ Nav2 за 60 мс) — проверить нечем, «приехал» не блокируем."""
    v = _track().verdict()
    assert v.status == lc.LOC_UNCHECKED and v.trusted


@pytest.mark.unit
def test_map_to_odom_parsed_from_rtabmap_info_stats():
    keys = ["Loop/Id/", "Loop/MapToOdom_x/m", "Loop/MapToOdom_y/m", "Loop/MapToOdom_yaw/deg"]
    assert lc.map_to_odom_from_stats(keys, [0.0, 0.8, 4.2, -25.7]) == (0.8, 4.2, -25.7)
    assert lc.map_to_odom_from_stats(["Loop/Id/"], [3.0]) is None


# ---------------------------------------------------------------- тул: Nav2 SUCCEEDED + потерянная поза


class _Future:
    def __init__(self, result):
        self._result = result

    def add_done_callback(self, cb):
        cb(self)

    def result(self):
        return self._result


class _NavClient:
    """Фейк Nav2. ``during`` — что приходит в сторож, пока «едем» (до ответа на цель)."""

    def __init__(self, status, during=None):
        self._status, self._during = status, during

    def wait_for_server(self, timeout_sec=0.0):
        return True

    def send_goal_async(self, goal, feedback_callback=None):
        if self._during:
            self._during()
        handle = types.SimpleNamespace(
            accepted=True,
            get_result_async=lambda: _Future(types.SimpleNamespace(status=self._status)),
            cancel_goal_async=lambda: None,
        )
        return _Future(handle)


class _Watch:
    """Фейк ``LocalizationWatch``: трек из модуля, сигналы подаёт фейк Nav2 «во время поездки»."""

    def __init__(self):
        self.track, self.stopped = None, []

    def start(self):
        self.track = lc.LocalizationTrack()
        return self.track

    def stop(self, track):
        self.stopped.append(track)


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


def _waypoint_tool(nav, client, watch, halts):
    tool = nav.NavigateToWaypointTool.__new__(nav.NavigateToWaypointTool)
    tool.node = MagicMock()
    tool.nav_client = client
    tool.controller_halt = lambda reason, rounds=2: halts.append(reason) or ao.CANCEL_NO_ACTIVE_GOAL
    tool.localization_watch = watch
    tool.waypoint_store = types.SimpleNamespace(
        get_waypoint=lambda name: {"x": 14.75, "y": 3.5, "theta": 1.57, "frame_id": "map"} if name == "R1" else None,
        list_waypoints=lambda: [{"name": "R1"}],
    )
    return tool


def _spoken(tool_name, args, mcp_result):
    request = ActionRequest("t1/1", 0, "telegram", tool_name, args, kn.TOOL_CLASSES[tool_name], 120.0)
    tool_result = ToolResult(
        tool_call_id="c1",
        content=mcp_result.to_json(),
        is_error=not mcp_result.success,
        data=mcp_result.data,
        message=mcp_result.message,
    )
    event = owner_event(request, tool_result)
    result = ActionResult(
        request.turn_id, 0, "telegram", tool_name, request.cls, event.status, event.reason, event.data
    )
    return phrase_from_result(result).text


@pytest.mark.unit
def test_nav2_succeeded_with_lost_odometry_says_not_sure(nav):
    """Живой случай 06.11 (R1, прогон 1): Nav2 SUCCEEDED, истина в 6.4 м. Робот не говорит «приехал»."""
    watch = _Watch()
    halts = []
    client = _NavClient(
        ao.GOAL_STATUS_SUCCEEDED, during=lambda: [watch.track.on_odom_covariance(LOST) for _ in range(300)]
    )
    res = _waypoint_tool(nav, client, watch, halts).execute("R1")
    assert res.success is False
    assert res.data[kn.NAV_OUTCOME_FIELD] == ao.NAV_LOCALIZATION_LOST
    assert res.data["localization"] == lc.LOC_ODOM_LOST and res.data["localization_detail"]["lost_frames"] == 300
    assert watch.stopped == [watch.track]  # подписки сняты
    assert halts == []  # Nav2 уже стоит на «цели» - гасить нечего
    said = _spoken("navigate_to_waypoint", {"waypoint": "R1"}, res)
    assert said == "Не уверен, что доехал до точки R1: потерял, где нахожусь на карте."


@pytest.mark.unit
def test_nav2_succeeded_with_healthy_localization_says_arrived(nav, monkeypatch):
    clock = iter(range(0, 1000, 30))  # поездка 30 с: не «уже стоял на точке» (#4344)
    monkeypatch.setattr(nav.time, "monotonic", lambda: float(next(clock)))
    watch = _Watch()
    client = _NavClient(
        ao.GOAL_STATUS_SUCCEEDED, during=lambda: [watch.track.on_odom_covariance(GOOD) for _ in range(300)]
    )
    res = _waypoint_tool(nav, client, watch, []).execute("R1")
    assert res.success is True and res.data["localization"] == lc.LOC_CONFIDENT
    assert _spoken("navigate_to_waypoint", {"waypoint": "R1"}, res).startswith("Приехал на точку R1")


@pytest.mark.unit
def test_nav2_failure_keeps_its_own_reason_with_localization_detail(nav):
    """ABORTED остаётся ABORTED (и гасит контроллер, #4435); вердикт локализации - только в данных."""
    watch = _Watch()
    halts = []
    client = _NavClient(ao.GOAL_STATUS_ABORTED, during=lambda: watch.track.on_odom_covariance(LOST))
    res = _waypoint_tool(nav, client, watch, halts).execute("R1")
    assert res.success is False and res.data[kn.NAV_OUTCOME_FIELD] == ao.NAV_ABORTED
    assert res.data["localization"] == lc.LOC_ODOM_LOST
    assert halts == [ao.NAV_ABORTED]


@pytest.mark.unit
def test_subscriptions_are_released_when_the_trip_raises(nav):
    watch = _Watch()

    def boom():
        raise RuntimeError("rclpy упал посреди поездки")

    with pytest.raises(RuntimeError):
        _waypoint_tool(nav, _NavClient(ao.GOAL_STATUS_SUCCEEDED, during=boom), watch, []).execute("R1")
    assert watch.stopped == [watch.track]


@pytest.mark.unit
def test_every_failure_kind_including_localization_has_dialog_phrase():
    assert ao.NAV_LOCALIZATION_LOST in kn.NAV_FAILURE_OUTCOMES
    assert ao.nav_outcome(ao.NAV_LOCALIZATION_LOST).kind == ao.NAV_LOCALIZATION_LOST
