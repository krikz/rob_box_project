"""Мир building (#4468): этаж учебного корпуса и точки-кабинеты.

Геометрия проверяется по коробкам, прочитанным обратно из SDF (то, что увидит Gazebo), а точки — по
файлу building.goals.yaml (то, что прочтут nav_bench и route.py). ROS не нужен.
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from collections import deque
from pathlib import Path

import pytest
import yaml

from rob_box_sim import building_floor as bf
from rob_box_sim import worlds_catalog as wc
from rob_box_sim.geometry2d import ROBOT_BODY_BOTTOM_Z, ROBOT_FOOTPRINT, Footprint, Pose2D, clearance
from rob_box_sim.grid_to_sdf import NAV_CENTER_CLEARANCE, center_clearance
from rob_box_sim.world_sdf import load_world_boxes

REPO = Path(__file__).resolve().parents[3]
WORLDS = REPO / "src" / "rob_box_sim" / "worlds"
PADDING = 0.03  # footprint_padding Nav2 (docker/main/config/nav2/nav2_params.yaml)
LIDAR_Z = 0.1143 + 0.4765  # rob_box.xacro: base_link над полом + лидар над base_link
RES = 0.05


@pytest.fixture(scope="module")
def boxes():
    return load_world_boxes(WORLDS / "building.sdf")


@pytest.fixture(scope="module")
def points():
    doc = yaml.safe_load((WORLDS / "building.goals.yaml").read_text(encoding="utf-8"))
    return doc["goals"]


def test_committed_goals_equal_generator():
    committed = (WORLDS / "building.goals.yaml").read_text(encoding="utf-8").replace("\r\n", "\n")
    assert committed == wc.CATALOG["building"].goals, f"перегенерируй: python3 -m rob_box_sim.worlds_catalog {WORLDS}"


def test_sdf_parses_and_every_box_has_collision(boxes):
    root = ET.fromstring((WORLDS / "building.sdf").read_text(encoding="utf-8"))
    walls = root.find("world/model[@name='building_walls']")
    assert walls is not None
    collisions = walls.findall("link/collision")
    visuals = walls.findall("link/visual")
    assert len(collisions) == len(visuals) == len(bf.floor_boxes()) == len(boxes)
    assert all(c.find("geometry/box/size") is not None for c in collisions)


def test_wall_constants_match_other_worlds():
    assert (bf.WALL_T, bf.WALL_H) == (wc.WALL_T, wc.WALL_H)


def test_floor_size_and_room_count(boxes):
    xs = [b.cx - b.sx / 2 for b in boxes] + [b.cx + b.sx / 2 for b in boxes]
    ys = [b.cy - b.sy / 2 for b in boxes] + [b.cy + b.sy / 2 for b in boxes]
    long_side = bf.X_EAST - bf.X_WEST
    assert 40 <= long_side <= 60
    assert max(xs) - min(xs) == pytest.approx(bf.X_EAST - bf.VESTIBULE_X0 + bf.WALL_T)
    assert max(ys) - min(ys) == pytest.approx(2 * bf.Y_OUT + bf.WALL_T)
    assert 10 <= len(bf.ROOMS) <= 20
    assert len({r.id for r in bf.ROOMS}) == len({r.label for r in bf.ROOMS}) == len(bf.ROOMS)


@pytest.mark.parametrize("side", ["north", "south"])
def test_rooms_tile_the_row_without_gaps(side):
    rooms = sorted((r for r in bf.ROOMS if r.side == side), key=lambda r: r.x0)
    assert rooms[0].x0 == bf.CORRIDOR_X0 and rooms[-1].x1 == bf.X_EAST
    for a, b in zip(rooms, rooms[1:]):
        assert a.x1 == b.x0, (a.id, b.id)


def _span_at(boxes, door: bf.Door) -> float:
    """Ширина просвета в стене двери по коробкам SDF: ближайшие коробки слева и справа от центра проёма."""
    along, across, size_along, size_across = ("cx", "cy", "sx", "sy") if door.axis == "x" else ("cy", "cx", "sy", "sx")
    spans = [
        (getattr(b, along) - getattr(b, size_along) / 2, getattr(b, along) + getattr(b, size_along) / 2)
        for b in boxes
        if abs(getattr(b, across) - door.at) < getattr(b, size_across) / 2
    ]
    c = door.center
    assert all(not (lo < c < hi) for lo, hi in spans), f"{door.name}: проём закрыт коробкой"
    left = max(hi for lo, hi in spans if hi <= c)
    right = min(lo for lo, hi in spans if lo >= c)
    return right - left


def test_every_door_is_at_least_min_width_in_sdf(boxes):
    doors = bf.all_doors()
    assert len(doors) == sum(len(r.doors) for r in bf.ROOMS) + 1
    for door in doors:
        width = _span_at(boxes, door)
        assert width >= bf.MIN_DOOR - 1e-6, (door.name, width)
        assert width == pytest.approx(door.width, abs=1e-6), door.name


def test_narrow_door_is_rejected():
    room = bf.Room("x", "x", "north", 0.0, 4.0, ((0.5, 0.9),))
    with pytest.raises(ValueError, match="уже"):
        bf.room_doors(room)


def test_furniture_is_above_lidar_plane(boxes):
    furniture = [b for b in boxes if min(b.sx, b.sy) > bf.WALL_T + 1e-6]
    assert furniture
    assert all(b.height >= bf.FURNITURE_MIN_H > LIDAR_Z for b in furniture)


def test_east_wing_repeats_every_4_m(boxes):
    """Похожие места: x 28..48 — пять одинаковых пар кабинетов, коридор между ними неразличим."""

    def window(k: int) -> list[tuple]:
        x0 = 28.0 + 4.0 * k
        return sorted(
            (round(b.cx - x0, 3), round(b.cy, 3), round(b.sx, 3), round(b.sy, 3), b.height)
            for b in boxes
            if x0 <= b.cx < x0 + 4.0 and b.sx < 4.5
        )

    first = window(0)
    assert len(first) >= 8  # перегородки, куски стен коридора, мебель двух кабинетов
    for k in range(1, 5):
        assert window(k) == first, k


def _padded(fp: Footprint, pad: float) -> Footprint:
    return Footprint(fp.x_min - pad, fp.x_max + pad, fp.y_min - pad, fp.y_max + pad)


def test_points_are_free_for_padded_footprint_at_any_yaw(boxes, points):
    fp = _padded(ROBOT_FOOTPRINT, PADDING)
    assert len(points) == len(bf.ROOMS) + len(bf.EXTRA_POINTS)
    for p in points:
        assert center_clearance(Pose2D(p["x"], p["y"]), boxes) >= NAV_CENTER_CLEARANCE, p["name"]
        for k in range(16):
            yaw = k * math.pi / 8
            assert clearance(Pose2D(p["x"], p["y"], yaw), boxes, fp) > 0, (p["name"], yaw)


def _free_cells(boxes, radius: float, bounds=(-9.0, 53.0, -8.5, 8.5)):
    """Сетка RES: клетка свободна, если её центр дальше ``radius`` от всех коробок (эрозия на радиус)."""
    x0, x1, y0, y1 = bounds
    nx, ny = int(round((x1 - x0) / RES)), int(round((y1 - y0) / RES))
    blocked = bytearray(nx * ny)
    for b in boxes:
        if b.height < ROBOT_BODY_BOTTOM_Z:
            continue
        assert abs(math.sin(b.yaw)) < 1e-9
        bx0, bx1, by0, by1 = b.cx - b.sx / 2, b.cx + b.sx / 2, b.cy - b.sy / 2, b.cy + b.sy / 2
        i0, i1 = max(0, int((bx0 - radius - x0) / RES)), min(nx, int((bx1 + radius - x0) / RES) + 1)
        j0, j1 = max(0, int((by0 - radius - y0) / RES)), min(ny, int((by1 + radius - y0) / RES) + 1)
        for i in range(i0, i1):
            cx = x0 + (i + 0.5) * RES
            dx = max(bx0 - cx, 0.0, cx - bx1)
            for j in range(j0, j1):
                cy = y0 + (j + 0.5) * RES
                dy = max(by0 - cy, 0.0, cy - by1)
                if dx * dx + dy * dy < radius * radius:
                    blocked[i * ny + j] = 1
    return blocked, (x0, y0, nx, ny)


def _reachable(boxes, radius: float):
    blocked, (x0, y0, nx, ny) = _free_cells(boxes, radius)

    def cell(x, y):
        return int((x - x0) / RES), int((y - y0) / RES)

    start = cell(0.0, 0.0)
    assert not blocked[start[0] * ny + start[1]]
    seen = bytearray(nx * ny)
    seen[start[0] * ny + start[1]] = 1
    queue = deque([start])
    while queue:
        i, j = queue.popleft()
        for a, b in ((i + 1, j), (i - 1, j), (i, j + 1), (i, j - 1)):
            if 0 <= a < nx and 0 <= b < ny and not seen[a * ny + b] and not blocked[a * ny + b]:
                seen[a * ny + b] = 1
                queue.append((a, b))
    return seen, cell, (nx, ny)


@pytest.fixture(scope="module")
def reach(boxes):
    # Эрозия на полуширину корпуса + padding: центр робота, едущего вдоль прохода, не ближе к стене.
    return _reachable(boxes, ROBOT_FOOTPRINT.y_max + PADDING)


def test_every_point_reachable_from_spawn(reach, points):
    seen, cell, (_, ny) = reach
    for p in points:
        i, j = cell(p["x"], p["y"])
        assert seen[i * ny + j], p["name"]


def test_floor_is_closed(reach):
    """Из этажа наружу не выехать: свободная область не касается края сетки."""
    seen, _, (nx, ny) = reach
    border = [(0, j) for j in range(ny)] + [(nx - 1, j) for j in range(ny)]
    border += [(i, 0) for i in range(nx)] + [(i, ny - 1) for i in range(nx)]
    assert not any(seen[i * ny + j] for i, j in border)


def test_bfs_sees_doors_too_wide_robot_stays_in_corridor(boxes, points):
    """Контроль самого теста: робот шире самой узкой двери (0.95) в кабинеты с ней не попадает."""
    seen, cell, (_, ny) = _reachable(boxes, bf.MIN_DOOR / 2 + 0.02)
    narrow = {r.id for r in bf.ROOMS if min(w for _, w in r.doors) < bf.MIN_DOOR + 0.02}
    assert narrow == {"dept_103", "wc"}
    for p in points:
        i, j = cell(p["x"], p["y"])
        if p["name"] in narrow:
            assert not seen[i * ny + j], p["name"]
    i, j = cell(50.5, 0.0)
    assert seen[i * ny + j]  # коридор 2.2 м проходим и для такого робота


def test_world_to_map_from_other_spawn():
    spawn = Pose2D(2.0, 1.0, math.pi / 2)
    at_spawn = bf.world_to_map(bf.NamedPoint("s", "s", 2.0, 1.0, math.pi / 2), spawn)
    assert (at_spawn.x, at_spawn.y, at_spawn.yaw) == pytest.approx((0.0, 0.0, 0.0), abs=1e-9)
    ahead = bf.world_to_map(bf.NamedPoint("a", "a", 2.0, 3.0, math.pi / 2), spawn)
    assert (ahead.x, ahead.y) == pytest.approx((2.0, 0.0), abs=1e-9)  # 2 м вперёд по оси робота


def test_goals_file_feeds_nav_bench_scenario():
    from rob_box_sim.nav_bench import load_scenario

    sc = load_scenario(REPO / "src" / "rob_box_sim" / "scenarios" / "baseline.yaml", "building", WORLDS)
    assert [g["name"] for g in sc["goals"]] == [p.name for p in bf.named_points()]
    assert sc["stop_on_failure"] is False


def test_route_py_reads_goals_file_in_degrees():
    """scripts/hil/video/route.py без ROS: функция load_goals из исходника читает файл точек мира."""
    import ast

    src = (REPO / "scripts" / "hil" / "video" / "route.py").read_text(encoding="utf-8")
    fn = next(n for n in ast.parse(src).body if isinstance(n, ast.FunctionDef) and n.name == "load_goals")
    ns: dict = {"math": math}
    exec(compile(ast.Module(body=[fn], type_ignores=[]), "route.py", "exec"), ns)  # noqa: S102
    goals = ns["load_goals"](WORLDS / "building.goals.yaml")
    by_name = {g[0]: g for g in goals}
    assert by_name["aud_101"] == ("aud_101", 5.6, 2.6, 90.0)
    assert by_name["entrance"] == ("entrance", -7.0, 0.0, 180.0)
    assert [g[0] for g in goals] == [p.name for p in bf.named_points()]
