"""Сверка карты rtabmap в симе с геометрией мира.

Критерий шага 3 плана (docs/research/2026-11-04-gazebo-sim-navigation.md §6):
rtabmap в режиме картирования строит карту мира. «Построил» здесь меряется:
доля занятых клеток ``/rtabmap/map``, лежащих не дальше ``tol`` от коробки
мира (precision: нет ли на карте стен, которых нет в мире, и не съехала ли
карта). Плюс ASCII-картинка и pgm для глаз.

Карта ``map`` в начале прогона совпадает с миром: робот спавнится в (0, 0, 0).
Чистые функции тестируются без ROS; ``grab_map`` — тонкая обёртка над rclpy.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence

from rob_box_sim.geometry2d import Box, signed_distance


@dataclass(frozen=True)
class OccupancyGrid:
    """Как nav_msgs/OccupancyGrid: data построчно от origin, -1 неизвестно, 0..100."""

    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    data: Sequence[int]

    def cell_center(self, i: int) -> tuple[float, float]:
        r, c = divmod(i, self.width)
        return (self.origin_x + (c + 0.5) * self.resolution, self.origin_y + (r + 0.5) * self.resolution)

    def value_at(self, x: float, y: float) -> int | None:
        """Значение клетки под точкой (x, y) в системе карты; None — вне сетки."""
        c = math.floor((x - self.origin_x) / self.resolution)
        r = math.floor((y - self.origin_y) / self.resolution)
        if not (0 <= c < self.width and 0 <= r < self.height):
            return None
        return self.data[r * self.width + c]

    def occupied_cells(self, thresh: int = 65) -> list[int]:
        return [i for i, v in enumerate(self.data) if v >= thresh]


def _point_box_distance(p: tuple[float, float], box: Box) -> float:
    h = 1e-4
    cell = [(p[0] - h, p[1] - h), (p[0] + h, p[1] - h), (p[0] + h, p[1] + h), (p[0] - h, p[1] + h)]
    return signed_distance(cell, box.corners())


def _near_any(p: tuple[float, float], boxes: Sequence[Box], tol: float) -> bool:
    for b in boxes:
        reach = tol + math.hypot(b.sx, b.sy) / 2
        if abs(b.cx - p[0]) > reach or abs(b.cy - p[1]) > reach:
            continue
        if _point_box_distance(p, b) <= tol:
            return True
    return False


def compare(grid: OccupancyGrid, boxes: Sequence[Box], tol: float = 0.15) -> dict:
    """Метрики совпадения карты с миром (см. шапку модуля)."""
    occ = grid.occupied_cells()
    centers = [grid.cell_center(i) for i in occ]
    on_wall = sum(1 for p in centers if _near_any(p, boxes, tol))
    known = sum(1 for v in grid.data if v >= 0)
    return {
        "cells_occupied": len(occ),
        "cells_free": sum(1 for v in grid.data if 0 <= v < 65),
        "cells_unknown": len(grid.data) - known,
        "occupied_near_world_box": on_wall,
        "precision": round(on_wall / len(occ), 4) if occ else None,
        "tol_m": tol,
    }


def ascii_map(grid: OccupancyGrid, step: int = 4) -> list[str]:
    """Грубая картинка: ``#`` занято, ``.`` свободно, пробел — неизвестно; верх — +Y."""
    lines = []
    for r0 in range(grid.height - step, -1, -step):
        line = []
        for c0 in range(0, grid.width, step):
            vals = [
                grid.data[r * grid.width + c]
                for r in range(r0, min(r0 + step, grid.height))
                for c in range(c0, min(c0 + step, grid.width))
            ]
            if any(v >= 65 for v in vals):
                line.append("#")
            elif any(v >= 0 for v in vals):
                line.append(".")
            else:
                line.append(" ")
        lines.append("".join(line).rstrip())
    return lines


def write_pgm(grid: OccupancyGrid, path) -> None:
    """Карта в pgm (как map_saver: 0 занято, 254 свободно, 205 неизвестно; верх — +Y)."""
    px = bytearray()
    for r in range(grid.height - 1, -1, -1):
        for c in range(grid.width):
            v = grid.data[r * grid.width + c]
            px.append(205 if v < 0 else 0 if v >= 65 else 254)
    with open(path, "wb") as f:
        f.write(b"P5\n%d %d\n255\n" % (grid.width, grid.height))
        f.write(bytes(px))


def write_raw_pgm(grid: OccupancyGrid, path) -> None:
    """Сетка в pgm без порога: байт = значение клетки 0..100, 255 — неизвестно; верх — +Y.

    Для costmap Nav2: 99 — inscribed (центр робота ближе robot_radius к препятствию),
    100 — lethal, 1..98 — инфляция. NavFn/Smac считают 99 и выше непроходимым.
    """
    px = bytearray()
    for r in range(grid.height - 1, -1, -1):
        for c in range(grid.width):
            v = grid.data[r * grid.width + c]
            px.append(255 if v < 0 else min(int(v), 254))
    with open(path, "wb") as f:
        f.write(b"P5\n%d %d\n255\n" % (grid.width, grid.height))
        f.write(bytes(px))


def grab_map(
    node, topic: str = "/rtabmap/map", timeout_s: float = 30.0, transient_local: bool = True
) -> OccupancyGrid | None:
    """Одно сообщение карты (rclpy). Возвращает None, если за ``timeout_s`` не пришло.

    ``transient_local`` — взять последнюю уже опубликованную карту (rtabmap и costmap Nav2
    публикуют её latched): иначе стоящий робот, у которого карта не меняется, ничего не пришлёт.
    """
    import time

    import rclpy
    from nav_msgs.msg import OccupancyGrid as GridMsg
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

    got: list = []
    durability = DurabilityPolicy.TRANSIENT_LOCAL if transient_local else DurabilityPolicy.VOLATILE
    sub = node.create_subscription(
        GridMsg, topic, got.append,
        QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=durability),
    )
    deadline = time.monotonic() + timeout_s
    while not got and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    if not got:
        return None
    m = got[-1]
    return OccupancyGrid(
        m.info.width, m.info.height, m.info.resolution,
        m.info.origin.position.x, m.info.origin.position.y, list(m.data),
    )
