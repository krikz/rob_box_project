"""rob_box_sim.geometry2d: зазор габарита робота до коробок мира."""

from __future__ import annotations

import math
import re
from pathlib import Path

import pytest

from rob_box_sim.geometry2d import ROBOT_FOOTPRINT, Box, Pose2D, clearance, signed_distance

XACRO = Path(__file__).resolve().parents[3] / "src" / "rob_box_description" / "urdf" / "rob_box.xacro"


def _square(cx, cy, half):
    return [(cx - half, cy - half), (cx + half, cy - half), (cx + half, cy + half), (cx - half, cy + half)]


def test_signed_distance_separated_and_overlapping():
    assert signed_distance(_square(0, 0, 1), _square(3, 0, 1)) == pytest.approx(1.0)
    # по диагонали: углы (1,1) и (2,2)
    assert signed_distance(_square(0, 0, 1), _square(3, 3, 1)) == pytest.approx(math.sqrt(2))
    # перекрытие на 0.5 по X
    assert signed_distance(_square(0, 0, 1), _square(1.5, 0, 1)) == pytest.approx(-0.5)


def test_pose_compose_inverse_roundtrip():
    a = Pose2D(1.0, 2.0, 0.7)
    b = Pose2D(-0.3, 0.5, -2.0)
    back = a.inverse().compose(a.compose(b))
    assert (back.x, back.y, back.yaw) == pytest.approx((b.x, b.y, b.yaw))


def test_clearance_to_wall_ahead_and_after_turn():
    wall = Box("w", 1.0, 0.0, 0.1, 4.0, 1.0)  # передняя грань x = 0.95
    assert clearance(Pose2D(0, 0, 0), [wall]) == pytest.approx(0.95 - ROBOT_FOOTPRINT.x_max)
    # повёрнут на 90°: к стене обращён левым бортом
    assert clearance(Pose2D(0, 0, math.pi / 2), [wall]) == pytest.approx(0.95 - ROBOT_FOOTPRINT.y_max)
    assert clearance(Pose2D(0.8, 0, 0), [wall]) < 0  # въехал


def test_box_below_body_does_not_count():
    flat = Box("threshold", 0.0, 0.0, 1.0, 1.0, 0.05)
    assert clearance(Pose2D(0, 0, 0), [flat]) == math.inf
    low = Box("low", 0.0, 0.0, 1.0, 1.0, 0.3)
    assert clearance(Pose2D(0, 0, 0), [low]) < 0


def test_footprint_matches_sim_xacro():
    """Габарит — из сим-ветки URDF: коробка корпуса и наружная кромка колёс."""
    text = XACRO.read_text(encoding="utf-8")
    m = re.search(
        r'<origin xyz="([-\d.]+) 0 0.243" rpy="0 0 0"/>\s*<geometry>\s*<box size="([\d.]+) ([\d.]+) [\d.]+"/>', text
    )
    assert m, "в rob_box.xacro не найдена коробка корпуса сим-ветки"
    cx, sx, sy = (float(v) for v in m.groups())
    assert ROBOT_FOOTPRINT.x_min == pytest.approx(cx - sx / 2)
    assert ROBOT_FOOTPRINT.x_max == pytest.approx(cx + sx / 2)
    assert ROBOT_FOOTPRINT.y_max == pytest.approx(sy / 2)
    assert ROBOT_FOOTPRINT.y_min == pytest.approx(-sy / 2)
    wheel_re = r'wheel"?\s+(?:sim_center_y="[-\d.]+"\s+)?origin_xyz="[-\d.]+ ([-\d.]+) '
    wheel_y = [abs(float(y)) for y in re.findall(wheel_re, text)]
    assert len(wheel_y) == 4
    # центр колеса по y + сдвиг цилиндра 0.0375 + полширины 0.0375: колёса внутри корпуса
    assert max(wheel_y) + 0.075 < ROBOT_FOOTPRINT.y_max


def test_footprint_is_nav2_footprint_of_robot():
    """Один прямоугольник: метрики стенда и Nav2 робота (замер 04.11, 0.61 × 0.605)."""
    import yaml

    nav2 = Path(__file__).resolve().parents[3] / "docker" / "main" / "config" / "nav2" / "nav2_params.yaml"
    doc = yaml.safe_load(nav2.read_text(encoding="utf-8"))
    for cm in ("local_costmap", "global_costmap"):
        pts = yaml.safe_load(doc[cm][cm]["ros__parameters"]["footprint"])
        assert min(x for x, _ in pts) == pytest.approx(ROBOT_FOOTPRINT.x_min)
        assert max(x for x, _ in pts) == pytest.approx(ROBOT_FOOTPRINT.x_max)
        assert min(y for _, y in pts) == pytest.approx(ROBOT_FOOTPRINT.y_min)
        assert max(y for _, y in pts) == pytest.approx(ROBOT_FOOTPRINT.y_max)
