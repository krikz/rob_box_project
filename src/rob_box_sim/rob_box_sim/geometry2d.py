"""Плоская геометрия сима: позы, коробки мира, габарит робота, зазор.

Один модуль на «где стены и где робот»: им пользуются генераторы миров
(проверка, что точка спавна и цели свободны) и стенд метрик (зазор и
столкновения по истинной позе). ROS не нужен.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, Sequence

Point = tuple[float, float]


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float = 0.0

    def compose(self, other: "Pose2D") -> "Pose2D":
        """self ∘ other: поза other, заданная в системе self, — в системе родителя self."""
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return Pose2D(
            self.x + c * other.x - s * other.y,
            self.y + s * other.x + c * other.y,
            wrap_angle(self.yaw + other.yaw),
        )

    def inverse(self) -> "Pose2D":
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return Pose2D(-(c * self.x + s * self.y), s * self.x - c * self.y, wrap_angle(-self.yaw))

    def distance_to(self, other: "Pose2D") -> float:
        return math.hypot(self.x - other.x, self.y - other.y)


def wrap_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


@dataclass(frozen=True)
class Box:
    """Препятствие мира: прямоугольник в плане (центр, размеры, yaw) и высота от пола."""

    name: str
    cx: float
    cy: float
    sx: float
    sy: float
    height: float
    yaw: float = 0.0

    @property
    def pose(self) -> Pose2D:
        return Pose2D(self.cx, self.cy, self.yaw)

    def corners(self) -> list[Point]:
        return _rect_corners(self.pose, -self.sx / 2, self.sx / 2, -self.sy / 2, self.sy / 2)

    def transformed(self, tf: Pose2D) -> "Box":
        p = tf.compose(self.pose)
        return Box(self.name, p.x, p.y, self.sx, self.sy, self.height, p.yaw)


@dataclass(frozen=True)
class Footprint:
    """Габарит в плане в системе base_footprint: x вперёд, y влево."""

    x_min: float
    x_max: float
    y_min: float
    y_max: float

    def corners_at(self, pose: Pose2D) -> list[Point]:
        return _rect_corners(pose, self.x_min, self.x_max, self.y_min, self.y_max)


# Габарит РОББОКСА в симе (src/rob_box_description/urdf/rob_box.xacro, sim:=true):
#   корпус — box 0.61 × 0.605 с центром x=+0.0014 → x −0.3036…+0.3064, y ±0.3025
#            (замер рулеткой Шифу 04.11: 61 × 60.5 см; меш base_link.stl — y ±0.303);
#   колёса — наружная кромка |y| = 0.27 и по x (±0.145 ± r 0.1143) внутри корпуса.
# Тот же прямоугольник — footprint Nav2 в docker/main/config/nav2/nav2_params.yaml.
# Корпус начинается на высоте 0.109 м (base_link z = 0.1143, низ коробки −0.005):
# препятствие ниже этого не задевает корпус — такие коробки в зазор не идут.
ROBOT_FOOTPRINT = Footprint(x_min=-0.3036, x_max=0.3064, y_min=-0.3025, y_max=0.3025)
ROBOT_BODY_BOTTOM_Z = 0.109


def _rect_corners(pose: Pose2D, x0: float, x1: float, y0: float, y1: float) -> list[Point]:
    c, s = math.cos(pose.yaw), math.sin(pose.yaw)
    return [
        (pose.x + c * x - s * y, pose.y + s * x + c * y)
        for x, y in ((x0, y0), (x1, y0), (x1, y1), (x0, y1))
    ]


def _axes(poly: Sequence[Point]) -> list[Point]:
    out = []
    for i in range(len(poly)):
        x0, y0 = poly[i]
        x1, y1 = poly[(i + 1) % len(poly)]
        nx, ny = y0 - y1, x1 - x0
        n = math.hypot(nx, ny)
        if n > 0:
            out.append((nx / n, ny / n))
    return out


def _project(poly: Sequence[Point], axis: Point) -> tuple[float, float]:
    vals = [p[0] * axis[0] + p[1] * axis[1] for p in poly]
    return min(vals), max(vals)


def _point_segment(p: Point, a: Point, b: Point) -> float:
    ax, ay = a
    dx, dy = b[0] - ax, b[1] - ay
    L2 = dx * dx + dy * dy
    t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / L2))
    return math.hypot(p[0] - ax - t * dx, p[1] - ay - t * dy)


def signed_distance(a: Sequence[Point], b: Sequence[Point]) -> float:
    """Зазор между выпуклыми многоугольниками: > 0 — расстояние, ≤ 0 — глубина пересечения."""
    min_overlap = math.inf
    for axis in _axes(a) + _axes(b):
        a0, a1 = _project(a, axis)
        b0, b1 = _project(b, axis)
        overlap = min(a1, b1) - max(a0, b0)
        if overlap <= 0:
            break
        min_overlap = min(min_overlap, overlap)
    else:
        return -min_overlap
    best = math.inf
    for poly, other in ((a, b), (b, a)):
        for p in poly:
            for i in range(len(other)):
                best = min(best, _point_segment(p, other[i], other[(i + 1) % len(other)]))
    return best


def clearance(
    pose: Pose2D,
    boxes: Iterable[Box],
    footprint: Footprint = ROBOT_FOOTPRINT,
    *,
    min_height: float = ROBOT_BODY_BOTTOM_Z,
    search_radius: float = 3.0,
) -> float:
    """Наименьший зазор от габарита робота в позе ``pose`` до коробок мира.

    Отрицательное значение — габарит пересекает коробку (столкновение).
    Коробки ниже ``min_height`` пропускаются. Коробки дальше ``search_radius``
    от робота не считаются (для скорости); если ближе никого нет — ``inf``.
    """
    robot = footprint.corners_at(pose)
    best = math.inf
    for box in boxes:
        if box.height < min_height:
            continue
        reach = search_radius + math.hypot(box.sx, box.sy) / 2
        if abs(box.cx - pose.x) > reach or abs(box.cy - pose.y) > reach:
            continue
        best = min(best, signed_distance(robot, box.corners()))
    return best
