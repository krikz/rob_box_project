"""Guard: боевой Nav2 описывает робота прямоугольником по замеру, а не кругом.

Шифу 04.11 измерил габарит рулеткой: 0.61 м вдоль хода × 0.605 м поперёк.
Круг robot_radius 0.45 в симе (шаг 6) не пускал DWB в проём 0.9 м: 5/15
против 25/25 с прямоугольником. Проверяется, что запустит Nav2 (параметры
костмапов и критики DWB), а не тексты комментариев.
"""

from __future__ import annotations

import math
from pathlib import Path

import pytest
import yaml

REPO_ROOT = Path(__file__).resolve().parents[3]
NAV2_PARAMS = REPO_ROOT / "docker" / "main" / "config" / "nav2" / "nav2_params.yaml"

LENGTH_M = 0.61  # вдоль хода (x base_footprint)
WIDTH_M = 0.605  # поперёк (y)
COSTMAPS = ("local_costmap", "global_costmap")


def _params() -> dict:
    return yaml.safe_load(NAV2_PARAMS.read_text(encoding="utf-8"))


def _costmap(cm: str) -> dict:
    return _params()[cm][cm]["ros__parameters"]


def _polygon(cm: str) -> list[list[float]]:
    raw = _costmap(cm)["footprint"]
    assert isinstance(raw, str), "Nav2 читает footprint строкой"
    pts = yaml.safe_load(raw)
    assert len(pts) == 4 and all(len(p) == 2 for p in pts), raw
    return pts


@pytest.mark.parametrize("cm", COSTMAPS)
def test_costmap_uses_measured_rectangle_not_radius(cm):
    assert "robot_radius" not in _costmap(cm), f"{cm}: robot_radius рядом с footprint вводит в заблуждение"
    xs = [p[0] for p in _polygon(cm)]
    ys = [p[1] for p in _polygon(cm)]
    assert max(xs) - min(xs) == pytest.approx(LENGTH_M, abs=1e-6)
    assert max(ys) - min(ys) == pytest.approx(WIDTH_M, abs=1e-6)
    # Центр корпуса — у base_footprint: смещение меша base_link.stl 1.4 мм вперёд.
    assert (max(xs) + min(xs)) / 2 == pytest.approx(0.0014, abs=1e-6)
    assert (max(ys) + min(ys)) / 2 == pytest.approx(0.0, abs=1e-6)


def test_local_and_global_footprint_are_the_same():
    assert _polygon("local_costmap") == _polygon("global_costmap")
    assert _costmap("local_costmap")["footprint_padding"] == _costmap("global_costmap")["footprint_padding"]


@pytest.mark.parametrize("cm", COSTMAPS)
def test_inflation_covers_corners_when_rotating(cm):
    """inflation_radius не меньше описанного радиуса (с padding) + 5 см запаса."""
    pad = _costmap(cm)["footprint_padding"]
    circumscribed = max(math.hypot(abs(x) + pad, abs(y) + pad) for x, y in _polygon(cm))
    inflation = _costmap(cm)["inflation_layer"]["inflation_radius"]
    assert inflation >= circumscribed + 0.05, (inflation, circumscribed)


def test_dwb_checks_whole_footprint():
    """BaseObstacle смотрит клетку под центром; с прямоугольником нужен ObstacleFootprint."""
    fp = _params()["controller_server"]["ros__parameters"]["FollowPath"]
    assert fp["plugin"] == "dwb_core::DWBLocalPlanner"
    assert "ObstacleFootprint" in fp["critics"]
    assert "BaseObstacle" not in fp["critics"]
    assert "ObstacleFootprint.scale" in fp


# --- разворот: проверка столкновения включена (Шифу 04.11) ---------------------


def _behavior() -> dict:
    return _params()["behavior_server"]["ros__parameters"]


def test_spin_collision_check_is_on_at_node_level():
    """Spin/BackUp (Humble) читают simulate_ahead_time без префикса плагина.

    Ключ spin.simulate_ahead_time Nav2 не читает: 0.0 там (28.02.2026) проверку не
    выключал, а только вводил в заблуждение. Значение — у behavior_server, > 0.
    """
    bs = _behavior()
    assert "simulate_ahead_time" not in bs["spin"], "spin.simulate_ahead_time Nav2 не читает"
    assert bs["simulate_ahead_time"] >= 1.0
    # 0.5 с при max_rotational_vel 0.6 рад/с — 0.3 рад впереди, ровно тормозной угол
    # с 0.6 рад/с при rotational_acc_lim 0.6 рад/с²: запаса на задержку costmap нет.
    lookahead_rad = bs["simulate_ahead_time"] * bs["max_rotational_vel"]
    braking_rad = bs["max_rotational_vel"] ** 2 / (2 * bs["rotational_acc_lim"])
    assert lookahead_rad >= 2 * braking_rad


def test_behaviors_take_pose_in_frame_of_their_costmap():
    """Spin/BackUp (Humble 1.1.20) берут позу робота в global_frame behavior_server и ищут её
    клетку в costmap из costmap_topic. Это local costmap — окно 5×5 м в odom. С global_frame
    map при сдвиге map→odom больше полуокна поза вне сетки: «Pose Goes Off Grid. Collision
    Ahead - Exiting Spin» (HIL 05.11, #4365, 2 из 2 в открытом месте).
    """
    bs = _behavior()
    assert bs["costmap_topic"].startswith("local_costmap/")
    assert bs["global_frame"] == _costmap("local_costmap")["global_frame"]


def test_dwb_path_align_looks_past_the_nose():
    """Курс по пути считается в точке перед носом корпуса, а не у центра.

    HIL 05.11 (#4365): при forward_point_distance 0.15 (нос — 0.31 м) DWB подводил корпус
    к проёму 0.8–0.9 м наискось, до 21°, и защита стопила по косяку.
    """
    fp = _params()["controller_server"]["ros__parameters"]["FollowPath"]
    nose = max(p[0] for p in _polygon("local_costmap"))
    assert fp["PathAlign.forward_point_distance"] > nose
    assert "PathAlign" in fp["critics"]


def test_dwb_rotate_to_goal_lookahead_positive_within_horizon():
    fp = _params()["controller_server"]["ros__parameters"]["FollowPath"]
    assert 0 < fp["RotateToGoal.lookahead_time"] <= fp["sim_time"]


@pytest.mark.parametrize("cm", COSTMAPS)
def test_padding_is_measured_compromise(cm):
    """Запас вокруг корпуса — из замера в симе 04.11 (3 прогона на вариант).

    0.01 — разворот в тупике 1.1 м 2/3, зазор 0.043; 0.03 — всё 3/3, зазор 0.081;
    0.05 и больше — проём 0.8 м 1/3 и меньше, разворот в тупике 0/3; 0.10 — проём 0.9 0/3.
    HIL 05.11: 0.03 в проёме 0.85 — «No valid trajectories», 0.02 — проходит (на восток).
    Корпус с запасом должен проходить проём 0.8 м с клеткой costmap с каждой стороны.
    """
    p = _costmap(cm)
    pad = p["footprint_padding"]
    assert 0.02 <= pad < 0.05, pad
    assert WIDTH_M + 2 * pad + 2 * p["resolution"] <= 0.8
