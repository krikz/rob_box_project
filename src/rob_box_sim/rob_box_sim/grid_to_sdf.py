"""Мир Gazebo из 2D-карты rtabmap: занятые клетки → стены высотой 1 м.

План: docs/research/2026-11-04-gazebo-sim-navigation.md §5, §6 шаг 3.

Вход — карта в формате map_server (``.pgm`` + ``.yaml``), её даёт
``rtabmap-export --map --poses --opt 2 <копия rtabmap.db>`` (оптимизированные
позы робота из базы, без пересчёта), и файл поз ``*_poses.txt`` того же экспорта
(формат ``#timestamp x y z qx qy qz qw id``).

Что делает:
  * клетка «занята», если её вероятность > ``occupied_thresh`` (правило map_server);
  * соседние занятые клетки сливаются в прямоугольники (сначала отрезки по
    строкам, потом одинаковые отрезки соседних строк) — коробок в разы меньше,
    чем клеток;
  * весь мир сдвигается так, чтобы первая «свободная для Nav2» поза робота из
    экспорта (центр дальше NAV_CENTER_CLEARANCE от стен) стала (0, 0, 0): робот
    спавнится в начале координат, а ``map`` rtabmap в симе совпадает с миром;
  * цели — позы, где реальный робот проехал после спавна: точки не ближе
    ``goal_step`` друг к другу, по тому же правилу свободы.

Карта дома/мастерской Шифу в публичный репозиторий не кладётся: результат пишется
в ``src/rob_box_sim/worlds/local/`` (каталог в .gitignore).

    python3 -m rob_box_sim.grid_to_sdf robot_map.yaml robot_map_poses.txt \\
        src/rob_box_sim/worlds/local --name robot_map
"""

from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass
from pathlib import Path

import yaml

from rob_box_sim.geometry2d import Box, Footprint, Pose2D, clearance
from rob_box_sim.world_sdf import render_world

WALL_H = 1.0


@dataclass(frozen=True)
class Grid:
    """Занятость: ``occupied[row][col]``, строка 0 — верх картинки (как в pgm)."""

    occupied: list[list[bool]]
    resolution: float
    origin_x: float
    origin_y: float

    @property
    def height(self) -> int:
        return len(self.occupied)

    @property
    def width(self) -> int:
        return len(self.occupied[0]) if self.occupied else 0

    def occupied_count(self) -> int:
        return sum(sum(r) for r in self.occupied)


def read_pgm(data: bytes) -> tuple[int, int, int, bytes]:
    """(width, height, maxval, пиксели) из бинарного P5. Комментарии ``#`` поддерживаются."""
    tokens: list[bytes] = []
    i = 0
    while len(tokens) < 4:
        while i < len(data) and data[i : i + 1].isspace():
            i += 1
        if data[i : i + 1] == b"#":
            while i < len(data) and data[i : i + 1] not in (b"\n", b"\r"):
                i += 1
            continue
        j = i
        while j < len(data) and not data[j : j + 1].isspace():
            j += 1
        tokens.append(data[i:j])
        i = j
    if tokens[0] != b"P5":
        raise ValueError(f"нужен бинарный pgm P5, а не {tokens[0]!r}")
    width, height, maxval = (int(t) for t in tokens[1:])
    if maxval > 255:
        raise ValueError("поддерживается только 8-битный pgm")
    pixels = data[i + 1 : i + 1 + width * height]
    if len(pixels) != width * height:
        raise ValueError(f"pgm обрезан: {len(pixels)} байт вместо {width * height}")
    return width, height, maxval, pixels


def load_grid(yaml_path: str | Path) -> Grid:
    yaml_path = Path(yaml_path)
    meta = yaml.safe_load(yaml_path.read_text(encoding="utf-8"))
    origin = meta["origin"]
    if abs(float(origin[2])) > 1e-9:
        raise ValueError("origin с поворотом не поддерживается")
    w, h, maxval, px = read_pgm((yaml_path.parent / meta["image"]).read_bytes())
    negate = int(meta.get("negate", 0))
    thresh = float(meta["occupied_thresh"])
    occ = []
    for r in range(h):
        row = px[r * w : (r + 1) * w]
        if negate:
            occ.append([v / maxval > thresh for v in row])
        else:
            occ.append([(maxval - v) / maxval > thresh for v in row])
    return Grid(occ, float(meta["resolution"]), float(origin[0]), float(origin[1]))


def rectangles(occupied: list[list[bool]]) -> list[tuple[int, int, int, int]]:
    """Покрытие занятых клеток прямоугольниками ``(row0, row1, col0, col1)``, концы не включены.

    Каждая занятая клетка — ровно в одном прямоугольнике, свободные не покрыты.
    """
    out = []
    active: dict[tuple[int, int], int] = {}  # (col0, col1) → row0
    for r, row in enumerate(list(occupied) + [[]]):
        runs = set()
        c = 0
        while c < len(row):
            if row[c]:
                c0 = c
                while c < len(row) and row[c]:
                    c += 1
                runs.add((c0, c))
            else:
                c += 1
        for run in list(active):
            if run not in runs:
                out.append((active.pop(run), r, run[0], run[1]))
        for run in runs:
            active.setdefault(run, r)
    return out


def grid_boxes(grid: Grid, to_world: Pose2D, height: float = WALL_H) -> list[Box]:
    """Коробки стен в мировой системе: ``to_world`` переводит систему карты в мир."""
    res = grid.resolution
    boxes = []
    for i, (r0, r1, c0, c1) in enumerate(rectangles(grid.occupied)):
        # строка 0 — верх картинки = наибольший y карты
        cx = grid.origin_x + (c0 + c1) / 2 * res
        cy = grid.origin_y + (grid.height - (r0 + r1) / 2) * res
        box = Box(f"cell_{i}", cx, cy, (c1 - c0) * res, (r1 - r0) * res, height)
        boxes.append(box.transformed(to_world))
    return boxes


def read_poses(path: str | Path) -> list[Pose2D]:
    """Позы из ``rtabmap-export --poses`` (формат 11: ``stamp x y z qx qy qz qw id``)."""
    poses = []
    for line in Path(path).read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        v = [float(x) for x in line.split()]
        _, x, y, _, qx, qy, qz, qw = v[:8]
        yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
        poses.append(Pose2D(x, y, yaw))
    return poses


# Центр робота ближе robot_radius 0.45 (docker/main/config/nav2/nav2_params.yaml,
# круглый габарит) к препятствию — для планировщика Nav2 робот уже «в стене» и
# никуда не едет. Спавн и цели выбираются дальше: 0.45 + 0.1 м запаса.
NAV_CENTER_CLEARANCE = 0.55
_POINT = Footprint(0.0, 0.0, 0.0, 0.0)


def center_clearance(pose: Pose2D, boxes: list[Box]) -> float:
    """Расстояние от центра робота (base_footprint) до ближайшей коробки."""
    return clearance(pose, boxes, _POINT)


def pick_goals(
    poses: list[Pose2D], boxes: list[Box], step: float, max_goals: int, min_center: float
) -> list[Pose2D]:
    """Точки маршрута реального робота через ``step`` метров, свободные для Nav2 (см. выше)."""
    goals: list[Pose2D] = []
    last = poses[0] if poses else None
    for p in poses[1:]:
        if len(goals) >= max_goals:
            break
        if p.distance_to(last) < step:
            continue
        if center_clearance(p, boxes) < min_center or clearance(p, boxes) <= 0:
            continue
        goals.append(p)
        last = p
    return goals


def build(
    yaml_path: Path,
    poses_path: Path,
    out_dir: Path,
    name: str,
    goal_step: float = 2.5,
    max_goals: int = 5,
    min_center: float = NAV_CENTER_CLEARANCE,
) -> dict:
    grid = load_grid(yaml_path)
    poses = read_poses(poses_path)
    if not poses:
        raise ValueError(f"{poses_path}: нет поз")
    map_boxes = grid_boxes(grid, Pose2D(0.0, 0.0, 0.0))
    # Спавн — первая поза маршрута, где Nav2 считает робота свободным. Первая поза
    # реального робота может стоять вплотную к мебели (в карте 04.09 — 0.10 м).
    first_free = next(
        (
            i
            for i, p in enumerate(poses)
            if center_clearance(p, map_boxes) >= min_center and clearance(p, map_boxes) > 0
        ),
        None,
    )
    if first_free is None:
        raise ValueError(f"нет позы спавна: центр робота везде ближе {min_center} м к стене")
    spawn_in_map = poses[first_free]
    to_world = spawn_in_map.inverse()
    boxes = [b.transformed(to_world) for b in map_boxes]
    spawn_clear = clearance(Pose2D(0.0, 0.0, 0.0), boxes)
    world_poses = [to_world.compose(p) for p in poses[first_free:]]
    goals = pick_goals(world_poses, boxes, goal_step, max_goals, min_center)

    out_dir.mkdir(parents=True, exist_ok=True)
    description = (
        f"  Мир из карты rtabmap робота: {yaml_path.name}, {grid.width}×{grid.height} клеток "
        f"по {grid.resolution} м.\n"
        f"  Занятые клетки ({grid.occupied_count()}) слиты в {len(boxes)} коробок высотой {WALL_H} м.\n"
        f"  Мир сдвинут: поза робота №{first_free + 1} из экспорта ({spawn_in_map.x:.3f}, {spawn_in_map.y:.3f}, "
        f"yaw {spawn_in_map.yaw:.3f}) — это (0, 0, 0).\n"
        "  Генератор: python3 -m rob_box_sim.grid_to_sdf. В git не кладётся (карта дома)."
    )
    sdf = render_world(name, boxes, description, single_link=True)
    (out_dir / f"{name}.sdf").write_text(sdf, encoding="utf-8", newline="\n")
    meta = {
        "world": name,
        "source_map": str(yaml_path.name),
        "resolution": grid.resolution,
        "occupied_cells": grid.occupied_count(),
        "boxes": len(boxes),
        "spawn_in_map": [round(spawn_in_map.x, 4), round(spawn_in_map.y, 4), round(spawn_in_map.yaw, 4)],
        "spawn_pose_index": first_free,
        "spawn_clearance_m": round(spawn_clear, 3),
        "spawn_center_clearance_m": round(center_clearance(Pose2D(0.0, 0.0, 0.0), boxes), 3),
        "goals": [
            {"name": f"pose_{i + 1}", "x": round(g.x, 3), "y": round(g.y, 3), "yaw": round(g.yaw, 3)}
            for i, g in enumerate(goals)
        ],
    }
    (out_dir / f"{name}.goals.yaml").write_text(
        yaml.safe_dump(meta, allow_unicode=True, sort_keys=False), encoding="utf-8", newline="\n"
    )
    return meta


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("map_yaml", type=Path)
    ap.add_argument("poses", type=Path)
    ap.add_argument("out_dir", type=Path)
    ap.add_argument("--name", default="robot_map")
    ap.add_argument("--goal-step", type=float, default=2.5)
    ap.add_argument("--max-goals", type=int, default=5)
    args = ap.parse_args(argv)
    meta = build(args.map_yaml, args.poses, args.out_dir, args.name, args.goal_step, args.max_goals)
    print(yaml.safe_dump(meta, allow_unicode=True, sort_keys=False))
    return 0


if __name__ == "__main__":
    sys.exit(main())
