"""Синтетические миры сима: размеры заданы здесь и только здесь.

План: docs/research/2026-11-04-gazebo-sim-navigation.md §5, §6 шаг 3.
Во всех мирах робот спавнится в (0, 0) и смотрит по +X, поэтому система
``map`` rtabmap в начале прогона совпадает с мировой системой Gazebo.

Перегенерировать файлы ``worlds/*.sdf``::

    python3 -m rob_box_sim.worlds_catalog src/rob_box_sim/worlds

Тест tests/unit/sim/test_worlds_catalog.py сверяет файлы с этим модулем.
"""

from __future__ import annotations

import sys
from dataclasses import dataclass
from pathlib import Path

from rob_box_sim.geometry2d import ROBOT_FOOTPRINT, Box
from rob_box_sim.world_sdf import render_world

WALL_T = 0.1
WALL_H = 1.0


def wall_x(name: str, y: float, x0: float, x1: float, h: float = WALL_H) -> Box:
    """Стена вдоль X: ось стены y, от x0 до x1 (с учётом толщины — как задано)."""
    return Box(name, (x0 + x1) / 2, y, x1 - x0, WALL_T, h)


def wall_y(name: str, x: float, y0: float, y1: float, h: float = WALL_H) -> Box:
    return Box(name, x, (y0 + y1) / 2, WALL_T, y1 - y0, h)


def partition(name: str, x: float, gap: float, y_min: float, y_max: float) -> list[Box]:
    """Поперечная перегородка x=const с проёмом ``gap`` по центру (y=0).

    SDF пишется в миллиметрах (world_sdf._f). Если центр половины перегородки выходит на
    полмиллиметра (проём 0.85: (0.425 + 3.0) / 2), её дальний конец удлиняется на 1 мм —
    он и так внутри внешней стены, а край у проёма остаётся точным.
    """
    near = round(gap / 2, 3)
    ext = 0.001 if round((near + y_max) * 1000) % 2 else 0.0
    ext_min = 0.001 if round((-near - y_min) * 1000) % 2 else 0.0
    return [
        wall_y(f"{name}_left", x, near, y_max + ext),
        wall_y(f"{name}_right", x, y_min - ext_min, -near),
    ]


@dataclass(frozen=True)
class World:
    name: str
    description: str
    boxes: tuple[Box, ...]

    def sdf(self) -> str:
        return render_world(self.name, self.boxes, self.description)


# Проходы полигона: (x перегородки, ширина проёма). Проём подбирался под круг
# robot_radius 0.45 (по расчёту ≥ 0.9 м); с 04.11 у робота прямоугольник
# 0.61 × 0.605 (nav2_params.yaml) — ширина корпуса 0.605 меньше всех проёмов.
# Самый узкий — 0.85 м, а не 0.8 (решение Шифу 05.11): защита оставляет корпусу 5 см
# с каждого бока, в 0.8 м свободного зазора всего ~0.1 м на бок, и HIL упирался в стоп.
POLYGON_GAPS = ((2.0, 1.2), (4.0, 1.0), (6.0, 0.9), (8.0, 0.85))


def gap_label(gap: float) -> str:
    """1.2 → "1.2", 1.0 → "1.0", 0.85 → "0.85": один знак, если второго нет."""
    s = f"{gap:.2f}"
    return s[:-1] if s.endswith("0") else s


def polygon() -> World:
    boxes = [
        wall_x("wall_north", 3.0, -1.55, 10.55),
        wall_x("wall_south", -3.0, -1.55, 10.55),
        wall_y("wall_west", -1.5, -3.0, 3.0),
        wall_y("wall_east", 10.5, -3.0, 3.0),
    ]
    for x, gap in POLYGON_GAPS:
        boxes += partition(f"gap_{gap_label(gap)}".replace(".", "_"), x, gap, -3.0, 3.0)
    return World(
        "polygon",
        "  Полигон проходимости РОББОКСА (план docs/research/2026-11-04-gazebo-sim-navigation.md §5).\n"
        "  Арена 12×6 м (x -1.5..10.5, y ±3.0), стены высотой 1.0 м, толщиной 0.1 м.\n"
        "  Поперечные перегородки с проёмом по центру (y=0):\n"
        "    x=2.0 — 1.2 м, x=4.0 — 1.0 м, x=6.0 — 0.9 м, x=8.0 — 0.85 м.\n"
        "  Робот спавнится в (0, 0), смотрит по +X — прямо в проёмы.",
        tuple(boxes),
    )


# Г-поворот: коридор шириной TURN90_WIDTH в свету, сначала по +X, потом налево по +Y.
TURN90_WIDTH = 1.2
TURN90_INNER_X = 3.0  # внутренний угол поворота (x стены ветки B со стороны робота)


def turn90() -> World:
    w = TURN90_WIDTH
    half = w / 2
    t2 = WALL_T / 2
    x_in = TURN90_INNER_X
    x_out = x_in + w
    y_end = 5.0
    boxes = (
        wall_x("leg_a_south", -half - t2, -1.0 - WALL_T, x_out + WALL_T),
        wall_x("leg_a_north", half + t2, -1.0 - WALL_T, x_in - t2),
        wall_y("leg_a_end", -1.0 - t2, -half - WALL_T, half + WALL_T),
        wall_y("leg_b_outer", x_out + t2, -half - t2, y_end + WALL_T),
        wall_y("leg_b_inner", x_in - t2, half, y_end + WALL_T),
        wall_x("leg_b_end", y_end + t2, x_in - WALL_T, x_out + WALL_T),
    )
    return World(
        "turn90",
        f"  Г-поворот на 90° налево: коридор {w:.1f} м в свету.\n"
        f"  Ветка A: x -1.0..{x_out:.1f}, y ±{half:.1f}. Ветка B: x {x_in:.1f}..{x_out:.1f}, y до {y_end:.1f}.\n"
        "  Робот спавнится в (0, 0), смотрит по +X.",
        boxes,
    )


# Длинный гладкий коридор. Лидар видит до 10 м (lslidar N10, в симе range_max 10),
# торцы дальше 10 м от участка езды: в скане только две параллельные линии и ICP
# вдоль коридора «не за что зацепиться» — на роботе 14–16.12.2025 icp_odometry
# в таком коридоре «думал, что стоит». Проверяем, повторяет ли это сим.
CORRIDOR_WIDTH = 1.4
CORRIDOR_X = (-12.0, 24.0)


def corridor() -> World:
    half = CORRIDOR_WIDTH / 2
    t2 = WALL_T / 2
    x0, x1 = CORRIDOR_X
    boxes = (
        wall_x("side_north", half + t2, x0 - WALL_T, x1 + WALL_T),
        wall_x("side_south", -half - t2, x0 - WALL_T, x1 + WALL_T),
        wall_y("end_west", x0 - t2, -half, half),
        wall_y("end_east", x1 + t2, -half, half),
    )
    return World(
        "corridor",
        f"  Гладкий коридор {CORRIDOR_WIDTH:.1f} м в свету, x {x0:.0f}..{x1:.0f}, без выступов.\n"
        "  От участка езды x 0..10 оба торца дальше 10 м (дальность лидара): ICP вдоль\n"
        "  коридора вырожден, как в реальном коридоре 14–16.12.2025.\n"
        "  Робот спавнится в (0, 0), смотрит по +X.",
        boxes,
    )


# Препятствие ниже плоскости лидара (на роботе лидар на 0.477 м над base_link):
# лидар его не видит, а корпус (низ на 0.109 м) в него упирается.
LOW_OBSTACLE = Box("low_box", 2.5, 0.0, 0.5, 0.8, 0.3)


def low_obstacle() -> World:
    boxes = (
        wall_x("wall_north", 2.0, -1.55, 6.55),
        wall_x("wall_south", -2.0, -1.55, 6.55),
        wall_y("wall_west", -1.5, -2.0, 2.0),
        wall_y("wall_east", 6.5, -2.0, 2.0),
        LOW_OBSTACLE,
    )
    return World(
        "low_obstacle",
        "  Комната 8×4 м (x -1.5..6.5, y ±2.0). На прямой от спавна к цели (5, 0) —\n"
        f"  коробка {LOW_OBSTACLE.sx:.1f}×{LOW_OBSTACLE.sy:.1f} м высотой {LOW_OBSTACLE.height:.1f} м "
        f"в ({LOW_OBSTACLE.cx:.1f}, {LOW_OBSTACLE.cy:.1f}):\n"
        "  ниже плоскости лидара, лидар её не видит (слепая зона).\n"
        "  Робот спавнится в (0, 0), смотрит по +X.",
        boxes,
    )


# Тупик: разворот на месте у стены. Коридор DEAD_END_WIDTH в свету, робот спавнится
# лицом в торец с зазором DEAD_END_FRONT. Углы габарита 0.61×0.605 заметают круг
# радиусом 0.43 м (geometry2d.ROBOT_FOOTPRINT): до стен тупика ~0.12 м — по корпусу
# разворот возможен, а footprint_padding (Spin «Collision Ahead», DWB ObstacleFootprint)
# раздувает этот круг на padding·√2 — так видно, при каком запасе разворот отказывает.
DEAD_END_WIDTH = 1.1
DEAD_END_FRONT = 0.25
DEAD_END_LENGTH = 4.5  # от торца назад, до задней стены


def dead_end() -> World:
    half = DEAD_END_WIDTH / 2
    t2 = WALL_T / 2
    x_end = ROBOT_FOOTPRINT.x_max + DEAD_END_FRONT  # грань торца
    x_back = x_end - DEAD_END_LENGTH
    boxes = (
        wall_x("side_north", half + t2, x_back - WALL_T, x_end + WALL_T),
        wall_x("side_south", -half - t2, x_back - WALL_T, x_end + WALL_T),
        wall_y("end_wall", x_end + t2, -half, half),
        wall_y("back_wall", x_back - t2, -half, half),
    )
    return World(
        "dead_end",
        f"  Тупик: коридор {DEAD_END_WIDTH:.1f} м в свету, x {x_back:.3f}..{x_end:.3f}, y ±{half:.2f}.\n"
        f"  Робот спавнится в (0, 0) лицом в торец (+X), зазор до торца {DEAD_END_FRONT:.2f} м.\n"
        "  Сценарий: развернуться на месте у стены (DWB и Spin) и выехать.",
        boxes,
    )


CATALOG = {w.name: w for w in (polygon(), turn90(), corridor(), low_obstacle(), dead_end())}


def main(argv: list[str] | None = None) -> int:
    args = sys.argv[1:] if argv is None else argv
    if len(args) != 1:
        print("usage: python3 -m rob_box_sim.worlds_catalog <каталог worlds>", file=sys.stderr)
        return 2
    out = Path(args[0])
    for world in CATALOG.values():
        path = out / f"{world.name}.sdf"
        path.write_text(world.sdf(), encoding="utf-8", newline="\n")
        print(f"{path}: {len(world.boxes)} коробок")
    return 0


if __name__ == "__main__":
    sys.exit(main())
