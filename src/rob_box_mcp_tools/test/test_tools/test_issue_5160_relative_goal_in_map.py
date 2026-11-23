"""#5160: относительная цель move_direction превращается в точку map один раз, при отправке.

Живой прогон 23.11: ``MoveDirectionTool`` слал ``NavigateToPose`` с ``frame_id="base_link"`` и
штампом момента отправки. Nav2 при каждом перепланировании ищет ``base_footprint``→``map``
(``BASE_FRAME`` в ``map_pose.py``, ``robot_base_frame`` в ``nav2_params.yaml``) на время
этого штампа; цель старше истории TF падает с ``Extrapolation Error``
(планировщик приступил через 18,4 с, две попытки упали, третья снята таймаутом тула 60 с).
Кэш TF планировщика — дефолт tf2, 10 с: ``tf_buffer_duration`` в yaml
узел не читает (#5151).

Контракт варианта A:
  1. «вперёд на метр» при позе (1.80, 0.06, yaw≈0) → цель в ``map`` ≈ (2.80, 0.06),
     ``frame_id == "map"``;
  2. повторный расчёт при сдвинувшемся роботе отправленную цель не меняет;
  3. нет трансформа — отказ тула с внятным текстом, цель не отправлена;
  4. «налево» при yaw 0 → курс цели +π/2: кватернион цели декодируется в yaw,
     знак ``yaw + theta`` проверяется, а не только координаты.

Модуль грузится через importlib, как в тесте #4390: ``tools/__init__.py`` тянет chimera,
а ей нужен ``rob_box_music``, которого в unit-окружении нет.
"""

import importlib.util
import math
import sys
import types
from pathlib import Path
from unittest.mock import MagicMock

import pytest

from rob_box_mcp_tools import map_pose as mp
from rob_box_mcp_tools.core import action_outcome as ao

_PKG = Path(__file__).resolve().parents[2] / "rob_box_mcp_tools"


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


class _PoseSource:
    """Заглушка TF: отдаёт позы по очереди и считает, сколько раз её спросили."""

    def __init__(self, poses):
        self._poses = list(poses)
        self.calls = 0

    def __call__(self):
        self.calls += 1
        return self._poses[min(self.calls - 1, len(self._poses) - 1)]


def _pose(x, y, yaw):
    return mp.MapPose(mp.POSE_STATUS_OK, mp.MAP_FRAME, x=x, y=y, theta=yaw)


def _missing():
    return mp.MapPose(mp.POSE_STATUS_NO_TRANSFORM, mp.MAP_FRAME, detail="Lookup would require extrapolation")


class _Future:
    def __init__(self, result):
        self._result = result

    def add_done_callback(self, callback):
        callback(self)

    def result(self):
        return self._result


class _NavClient:
    """Фейк Nav2: запоминает каждую отправленную цель и отвечает успехом."""

    def __init__(self):
        self.goals = []

    def wait_for_server(self, timeout_sec=0.0):
        return True

    def send_goal_async(self, goal, feedback_callback=None):
        self.goals.append(goal)
        handle = type("Handle", (), {})()
        handle.accepted = True
        handle.get_result_async = lambda: _Future(type("Result", (), {"status": ao.GOAL_STATUS_SUCCEEDED})())
        return _Future(handle)


class _Node:
    def get_logger(self):
        return type("Logger", (), {"info": lambda *a: None, "warning": lambda *a: None})()

    def get_clock(self):
        return type("Clock", (), {"now": lambda self: type("Stamp", (), {"to_msg": lambda self: 0})()})()


def _tool(nav, poses):
    source = _PoseSource(poses)
    tool = nav.MoveDirectionTool.__new__(nav.MoveDirectionTool)
    tool.node = _Node()
    tool.nav_client = _NavClient()
    tool.controller_halt = lambda *args, **kwargs: None
    tool._pose_source = source
    # name — property; log_info обращается к self.name и без подмены задел бы заглушку позы.
    tool.log_info = lambda *args, **kwargs: None
    tool.log_warning = lambda *args, **kwargs: None
    return tool, source


def _goal_xy(goal):
    return goal.pose.pose.position.x, goal.pose.pose.position.y


def _goal_yaw(goal):
    """Yaw из кватерниона цели: код кладёт ``z = sin(θ/2)``, ``w = cos(θ/2)``."""
    orientation = goal.pose.pose.orientation
    return math.atan2(2.0 * orientation.w * orientation.z, 1.0 - 2.0 * orientation.z * orientation.z)


# ---------------------------------------------------------------- вариант A


@pytest.mark.unit
def test_forward_meter_becomes_a_map_point(nav):
    """Робот (1.80, 0.06, yaw≈0), «вперёд на метр» → точка map ≈ (2.80, 0.06)."""
    tool, source = _tool(nav, [_pose(1.80, 0.06, 0.0)])

    result = tool.execute("вперёд", 1.0)

    assert result.success is True
    assert source.calls == 1, "позу спрашиваем ровно один раз, при отправке"
    assert len(tool.nav_client.goals) == 1
    goal = tool.nav_client.goals[0]
    assert goal.pose.header.frame_id == "map"
    assert _goal_xy(goal) == pytest.approx((2.80, 0.06), abs=1e-9)
    assert result.data["goal_frame_id"] == "map"
    assert (result.data["goal_x"], result.data["goal_y"]) == pytest.approx((2.80, 0.06), abs=1e-9)


@pytest.mark.unit
def test_replanning_does_not_move_the_sent_goal(nav):
    """Сдвиг робота между двумя вызовами источника позы первую цель не трогает.

    На старом коде цель жила в ``base_link`` и пересчитывалась Nav2 при каждом
    перепланировании. Здесь цель уходит в ``map`` один раз: повторный вызов
    ``relative_goal_in_map`` от уже сдвинувшейся позы даёт другую точку, а
    отправленная цель остаётся прежней.
    """
    source = _PoseSource([_pose(1.80, 0.06, 0.0), _pose(9.0, 9.0, math.pi / 2)])
    tool, _same = _tool(nav, [])
    tool._pose_source = source

    first = tool.execute("вперёд", 1.0)
    # Робот уехал. Пересчёт «как при перепланировании» — тот же код, новая поза.
    recomputed = nav.relative_goal_in_map(source, 1.0, 0.0, 0.0)

    assert first.success is True
    sent = tool.nav_client.goals
    assert len(sent) == 1, "повторный расчёт цель не отправляет"
    assert sent[0].pose.header.frame_id == "map"
    assert _goal_xy(sent[0]) == pytest.approx((2.80, 0.06), abs=1e-9)
    assert recomputed[0:2] == pytest.approx((9.0, 10.0), abs=1e-9)
    assert _goal_xy(sent[0]) != pytest.approx(recomputed[0:2])
    assert source.calls == 2


@pytest.mark.unit
def test_yaw_rotates_the_relative_point(nav):
    """Курс 90°: «вперёд на метр» уходит по +y карты, а не по +x."""
    tool, _source = _tool(nav, [_pose(1.80, 0.06, math.pi / 2)])

    result = tool.execute("вперёд", 1.0)

    assert result.success is True
    goal = tool.nav_client.goals[0]
    assert _goal_xy(goal) == pytest.approx((1.80, 1.06), abs=1e-9)
    assert goal.pose.header.frame_id == "map"


@pytest.mark.unit
def test_turn_left_keeps_the_sign_of_the_goal_heading(nav):
    """«Налево» при yaw 0 → цель (1.80, 0.06) с курсом +π/2.

    Координаты при повороте на месте не меняются, поэтому мутация
    ``yaw + theta`` → ``yaw - theta`` (navigation.py) координатами не ловится.
    Ловит знак yaw, прочитанный из кватерниона цели.
    """
    tool, source = _tool(nav, [_pose(1.80, 0.06, 0.0)])

    result = tool.execute("налево", 1.0)

    assert result.success is True
    assert source.calls == 1
    goal = tool.nav_client.goals[0]
    assert goal.pose.header.frame_id == "map"
    assert _goal_xy(goal) == pytest.approx((1.80, 0.06), abs=1e-9)
    assert _goal_yaw(goal) == pytest.approx(math.pi / 2, abs=1e-9)


@pytest.mark.unit
def test_no_transform_is_a_tool_error_and_sends_nothing(nav):
    """Нет base_footprint→map — внятный отказ, цель «как раньше» не уходит."""
    tool, source = _tool(nav, [_missing()])

    result = tool.execute("вперёд", 1.0)

    assert result.success is False
    assert tool.nav_client.goals == [], "без трансформа цель не отправляется"
    assert source.calls == 1
    assert result.error, "отказ называет причину, а не молчит"
    assert "трансформ" in result.error
    assert "map" in result.error
    assert result.message, "фразу строит код (ADR-0148)"
    assert "Не поехал" in result.message


# ---------------------------------------------------------------- фейковый TF: штамп цели
# Зеркало правила tf2::BufferCore, только для сценария. В бою этого буфера нет:
# MoveDirectionTool считает точку один раз (relative_goal_in_map), перепланирование
# делает Nav2 сам. Предмет теста — настоящая функция, фейк только показывает,
# почему кадр base_footprint на старый штамп падает, а точка map — нет.


class _Extrapolation(LookupError):
    """Текст как у tf2 в живом прогоне 23.11, не класс продакшена."""


class _TfHistory:
    """Окно истории: кадр старше cache_time вытесняется, lookup на старый штамп падает."""

    def __init__(self, cache_time_sec: float = 10.0) -> None:
        self.cache_time_sec = cache_time_sec
        self._frames = []

    def set_transform(self, stamp_sec: float, x: float, y: float, yaw: float) -> None:
        self._frames.append((stamp_sec, (x, y, yaw)))
        self._frames.sort(key=lambda frame: frame[0])
        latest = self._frames[-1][0]
        self._frames = [frame for frame in self._frames if frame[0] >= latest - self.cache_time_sec]

    def lookup(self, stamp_sec: float):
        if not self._frames:
            raise _Extrapolation("Lookup would require extrapolation: buffer is empty")
        earliest, latest = self._frames[0][0], self._frames[-1][0]
        if stamp_sec < earliest or stamp_sec > latest:
            raise _Extrapolation(
                "Lookup would require extrapolation into the past. "
                f"Requested time {stamp_sec} but the earliest data is at time {earliest}, "
                "when looking up transform from frame [base_footprint] to frame [map]"
            )
        return min(self._frames, key=lambda frame: abs(frame[0] - stamp_sec))[1]


_SEND = 1795423727.5
_REPLAN = _SEND + 18.4  # живой прогон: планировщик приступил через 18,4 с
_ROBOT = (1.80, 0.06, 0.0)


def _aged_history():
    """Кадр отправки уже вытеснен: в окне 10 с остался только кадр перепланирования."""
    history = _TfHistory(cache_time_sec=10.0)
    history.set_transform(_SEND, *_ROBOT)
    history.set_transform(_REPLAN, *_ROBOT)
    return history


@pytest.mark.unit
def test_stale_base_stamp_extrapolates_map_goal_does_not(nav):
    """Старый путь падает на фейковом буфере; предмет — relative_goal_in_map.

    Кадр ``base_footprint`` со штампом отправки к перепланированию (+18,4 с,
    буфер 10 с) вне окна → Extrapolation с текстом живого лога. Свежий штамп
    находится. Точка мира от штампа не зависит: ``relative_goal_in_map`` даёт
    кадр ``map`` и (2.80, 0.06), lookup на момент перепланирования её не двигает.
    """
    history = _aged_history()

    with pytest.raises(_Extrapolation, match="extrapolation into the past") as caught:
        history.lookup(_SEND)
    message = str(caught.value)
    assert "base_footprint" in message and "map" in message
    assert f"Requested time {_SEND}" in message
    assert f"earliest data is at time {_REPLAN}" in message

    assert history.lookup(_REPLAN) == _ROBOT

    world = nav.relative_goal_in_map(lambda: _pose(*_ROBOT), 1.0, 0.0, 0.0)
    assert world[3] == "map"
    assert world[0:2] == pytest.approx((2.80, 0.06))
    assert not hasattr(nav, "TfHistory")
    assert not hasattr(nav, "relative_goal_at_replan")
