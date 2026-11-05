"""#4390: точка пишется и читается в одном кадре (map), поза — из TF map ← base_footprint, не из /odom.

HIL 04.11 23:00–23:41 UTC: ``save_waypoint`` / ``get_current_pose`` брали позу из ``/odom``, а
``navigate_to_waypoint`` слал цель в ``map``; после картографии map и odom разошлись ~3 м, «езжай на
хил-кухню» дал «Приехал…» за 2.6 с почти без движения.

Контракт:
  1. Поза — из трансформа ``map ← base_footprint`` (``MapPoseSource``), кадр ``map`` в данных и в БД.
  2. Нет трансформа / не дождались / нет tf2 — честный отказ со статусом; точка НЕ пишется,
     одометрия молча не подставляется.
  3. Цель Nav2 — в кадре точки; у точки до #4390 (``frame_id`` NULL) — ``map``, как и раньше,
     с пометкой ``frame_assumed``.
  4. Старая БД без колонки ``frame_id`` открывается, колонка добавляется, старые строки не меняются.
"""

import importlib.util
import math
import sqlite3
import sys
import types
from pathlib import Path
from unittest.mock import MagicMock

import pytest

from rob_box_core.tf_lookup import NoTransform, RigidTransform, TransformStale
from rob_box_mcp_tools import map_pose as mp
from rob_box_mcp_tools.core import action_outcome as ao
from rob_box_mcp_tools.waypoint_store import WaypointStore

_PKG = Path(__file__).resolve().parents[2] / "rob_box_mcp_tools"

#: map ← base_footprint: робот в (3, -1) на карте, курс 90°. Одометрия в это время говорила бы другое.
_YAW90 = (0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))
_MAP_FROM_BASE = RigidTransform(translation=(3.0, -1.0, 0.0), rotation=_YAW90)


class _Session:
    """Фейк сеанса TF: отдаёт трансформ или бросает ошибку; помнит запросы и закрытие."""

    def __init__(self, outcome):
        self.outcome = outcome
        self.calls = []
        self.closed = False

    def latest(self, target, source, timeout_sec):
        self.calls.append((target, source, timeout_sec))
        if isinstance(self.outcome, type) and issubclass(self.outcome, Exception):
            raise self.outcome("boom")
        return self.outcome

    def close(self):
        self.closed = True


def _source(outcome, timeout_sec=1.5):
    session = _Session(outcome)
    return mp.MapPoseSource(lambda: session, timeout_sec=timeout_sec), session


# ---------------------------------------------------------------- MapPoseSource


@pytest.mark.unit
def test_pose_comes_from_tf_map_from_base_footprint():
    source, session = _source(_MAP_FROM_BASE, timeout_sec=1.2)
    pose = source()
    assert pose.ok and pose.frame_id == "map"
    assert (pose.x, pose.y) == pytest.approx((3.0, -1.0))
    assert pose.theta == pytest.approx(math.pi / 2)
    assert session.calls == [("map", "base_footprint", 1.2)]
    assert session.closed, "слушатель /tf должен сниматься после запроса (CPU-петля 2026-08-22)"


@pytest.mark.unit
@pytest.mark.parametrize(
    "outcome, status",
    [(NoTransform, mp.POSE_STATUS_NO_TRANSFORM), (TransformStale, mp.POSE_STATUS_TRANSFORM_STALE)],
)
def test_missing_tf_is_honest_refusal(outcome, status):
    source, session = _source(outcome)
    pose = source()
    assert not pose.ok and pose.status == status
    assert pose.x is None and pose.y is None and pose.theta is None
    assert pose.reason == mp.POSE_FAILURE_REASONS[status]
    assert session.closed


@pytest.mark.unit
def test_tf2_not_available_is_refusal_not_crash():
    def broken():
        raise ImportError("No module named 'tf2_ros'")

    pose = mp.MapPoseSource(broken)()
    assert pose.status == mp.POSE_STATUS_TF_UNAVAILABLE and "tf2_ros" in pose.detail


# ---------------------------------------------------------------- тулы навигации


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
    return module


@pytest.fixture
def store():
    s = WaypointStore(db_path=":memory:")
    yield s
    s.close()


class _Future:
    def __init__(self, result):
        self._result = result

    def add_done_callback(self, cb):
        cb(self)

    def result(self):
        return self._result


class _NavClient:
    """Фейк Nav2: принимает цель, запоминает её и отвечает успехом."""

    def __init__(self):
        self.goals = []
        handle = types.SimpleNamespace(
            accepted=True,
            get_result_async=lambda: _Future(types.SimpleNamespace(status=ao.GOAL_STATUS_SUCCEEDED)),
            cancel_goal_async=lambda: None,
        )
        self._handle = handle

    def wait_for_server(self, timeout_sec=0.0):
        return True

    def send_goal_async(self, goal, feedback_callback=None):
        self.goals.append(goal)
        return _Future(self._handle)


def _tool(cls, **attrs):
    tool = cls.__new__(cls)
    tool.node = MagicMock()
    for k, v in attrs.items():
        setattr(tool, k, v)
    return tool


def _save_tool(nav, store, outcome):
    source, _ = _source(outcome)
    return _tool(nav.SaveWaypointTool, waypoint_store=store, _pose_source=source, mapping_state=None)


@pytest.mark.unit
def test_save_waypoint_writes_map_pose_with_frame(nav, store):
    res = _save_tool(nav, store, _MAP_FROM_BASE).execute("хил-кухня")
    assert res.success is True
    assert res.data["frame_id"] == "map" and (res.data["x"], res.data["y"]) == pytest.approx((3.0, -1.0))
    wp = store.get_waypoint("хил-кухня")
    assert wp["frame_id"] == "map"
    assert (wp["x"], wp["y"], wp["theta"]) == pytest.approx((3.0, -1.0, math.pi / 2))


@pytest.mark.unit
@pytest.mark.parametrize("outcome", [NoTransform, TransformStale])
def test_save_waypoint_without_tf_refuses_and_writes_nothing(nav, store, outcome):
    res = _save_tool(nav, store, outcome).execute("хил-окно")
    assert res.success is False
    assert res.error.startswith("Не могу определить позицию робота на карте: ")
    assert res.data["pose_status"] in (mp.POSE_STATUS_NO_TRANSFORM, mp.POSE_STATUS_TRANSFORM_STALE)
    assert "/odom" not in res.error
    assert store.get_waypoint("хил-окно") is None


@pytest.mark.unit
def test_get_current_pose_is_in_map(nav):
    source, _ = _source(_MAP_FROM_BASE)
    res = _tool(nav.GetCurrentPoseTool, _pose_source=source).execute()
    assert res.success is True and res.data["frame_id"] == "map"
    assert res.message == "Позиция: (3.00, -1.00), угол 90°"


@pytest.mark.unit
def test_get_current_pose_without_tf_refuses(nav):
    source, _ = _source(NoTransform)
    res = _tool(nav.GetCurrentPoseTool, _pose_source=source).execute()
    assert res.success is False and res.data["pose_status"] == mp.POSE_STATUS_NO_TRANSFORM
    assert res.error == "Не могу определить позицию робота на карте: " + mp.POSE_FAILURE_REASONS["no_transform"]


def _drive(nav, store, name):
    client = _NavClient()
    res = _tool(nav.NavigateToWaypointTool, nav_client=client, waypoint_store=store).execute(name)
    assert len(client.goals) == 1
    return res, client.goals[0]


@pytest.mark.unit
def test_saved_point_is_driven_to_in_the_same_frame(nav, store):
    """Сохранили по TF map → поехали: цель в map и ровно в те координаты, что дал TF."""
    _save_tool(nav, store, _MAP_FROM_BASE).execute("хил-вход")
    res, goal = _drive(nav, store, "хил-вход")
    assert res.success is True
    assert goal.pose.header.frame_id == "map"
    assert (goal.pose.pose.position.x, goal.pose.pose.position.y) == pytest.approx((3.0, -1.0))
    assert res.data["frame_id"] == "map" and res.data["frame_assumed"] is False


@pytest.mark.unit
def test_goal_uses_the_point_frame(nav, store):
    store.save_waypoint("в-одометрии", 1.0, 2.0, 0.0, frame_id="odom")
    _res, goal = _drive(nav, store, "в-одометрии")
    assert goal.pose.header.frame_id == "odom"


@pytest.mark.unit
def test_legacy_point_without_frame_goes_to_map_as_before_and_is_flagged(nav, store):
    store.save_waypoint("старая", 1.0, 2.0, 0.0)  # как писали до #4390: без кадра
    assert store.get_waypoint("старая")["frame_id"] is None
    res, goal = _drive(nav, store, "старая")
    assert goal.pose.header.frame_id == mp.LEGACY_WAYPOINT_FRAME == "map"
    assert res.data["frame_assumed"] is True


# ---------------------------------------------------------------- схема БД


_OLD_DDL = """
CREATE TABLE maps (id INTEGER PRIMARY KEY AUTOINCREMENT, map_id TEXT NOT NULL UNIQUE, name TEXT,
                   created_at REAL NOT NULL, is_active INTEGER NOT NULL DEFAULT 0);
CREATE TABLE waypoints (id INTEGER PRIMARY KEY AUTOINCREMENT, map_id TEXT NOT NULL, name TEXT NOT NULL,
                        x REAL NOT NULL, y REAL NOT NULL, theta REAL NOT NULL DEFAULT 0.0,
                        created_at REAL NOT NULL, updated_at REAL NOT NULL, UNIQUE(map_id, name),
                        FOREIGN KEY (map_id) REFERENCES maps(map_id) ON DELETE CASCADE);
INSERT INTO maps (map_id, name, created_at, is_active) VALUES ('m1', 'квартира', 1.0, 1);
INSERT INTO waypoints (map_id, name, x, y, theta, created_at, updated_at) VALUES ('m1', 'кухня', 4.5, -2.25, 0.5, 1, 1);
"""


@pytest.mark.unit
def test_old_db_gets_frame_column_and_keeps_rows(tmp_path):
    db = tmp_path / "tools.db"
    conn = sqlite3.connect(db)
    conn.executescript(_OLD_DDL)
    conn.commit()
    conn.close()

    s = WaypointStore(db_path=str(db))
    try:
        assert s.get_waypoint("кухня") == {"name": "кухня", "x": 4.5, "y": -2.25, "theta": 0.5, "frame_id": None}
        s.save_waypoint("кухня", 1.0, 1.0, 0.0, frame_id="map")  # перезапись обновляет и кадр
        assert s.get_waypoint("кухня")["frame_id"] == "map"
    finally:
        s.close()
    WaypointStore(db_path=str(db)).close()  # повторное открытие: ALTER не повторяется
