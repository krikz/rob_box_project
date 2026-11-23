"""Зоны скорости (решение Шифу 10.11): маска SpeedFilter Nav2 из карты или геометрии мира.

Скорость движения задаётся зонами: у стен и в проёмах 0.10–0.15 м/с, в открытом зале
без ограничения (базовая ``max_vel_x``). Механика — штатный ``nav2_costmap_2d::SpeedFilter``
(Humble 1.1.20): слой local_costmap читает маску (``nav_msgs/OccupancyGrid`` в кадре ``map``)
и публикует ``nav2_msgs/SpeedLimit`` в ``/speed_limit``; ``controller_server`` подписан и
пропорционально ужимает ``max_vel_x`` / ``max_speed_xy`` / ``max_vel_theta`` DWB. Маску
публикует ``nav2_map_server`` (``map_server`` + ``costmap_filter_info_server``) из PGM+YAML.

Значения маски (режим Scale, ``base: 0.0``, ``multiplier: 0.01``):

- ``0`` — нет ограничения (открытый зал, базовая скорость);
- ``10`` — теснота (свободный коридор уже ``tight_width``): 0.10 м/с;
- ``15`` — у стен (ближе ``near_dist`` к препятствию): 0.15 м/с.

Правило зон. «Теснота» — по ширине свободного коридора в клетке: лучи в четырёх осях до
ближайшей занятой клетки, ``width = min(луч_влево + луч_вправо, луч_вверх + луч_вниз)``.
Расстояние до ближайшего препятствия тут не подходит: клетка в полуметре от стены
большого зала дала бы «коридор 1.0 м» и притянула бы 0.10 м/с ко всем стенам. «У стен» —
по евклидову расстоянию (трансформ Felzenszwalb, чистый numpy). Неизвестное на карте
считается занятым (как в ``coverage_plan``: туда центр робота не пускаем) — у края карты
зона «у стен».

Генератор офлайновый: маска пересчитывается после смены карты. ROS не нужен.
"""

from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from rob_box_sim.geometry2d import Box, Pose2D
from rob_box_sim.map_check import OccupancyGrid
from rob_box_sim.world_sdf import load_world_boxes

#: Порог «занято» как у map_server / map_check.
OCCUPIED_THRESH = 65
#: Скорости и границы зон по умолчанию (решение Шифу 10.11, замеры HIL §11 research/2026-11-05).
DEFAULT_NEAR_SPEED_M = 0.15
DEFAULT_NEAR_DIST_M = 0.75
DEFAULT_TIGHT_SPEED_M = 0.10
DEFAULT_TIGHT_WIDTH_M = 1.2
#: Запас сетки маски за пределами занятого (м): вне карты/мира ограничения нет, но расстояние
#: должно «дотянуться» до дальних свободных клеток.
DEFAULT_MARGIN_M = 3.0
DEFAULT_RESOLUTION_M = 0.05

#: Пороги Scale-режима map_server (зашиты и в yaml маски).
FREE_THRESH = 0.25
OCC_THRESH = 0.65


@dataclass(frozen=True)
class ZoneParams:
    """Границы и скорости зон, м и м/с."""

    near_speed: float = DEFAULT_NEAR_SPEED_M
    near_dist: float = DEFAULT_NEAR_DIST_M
    tight_speed: float = DEFAULT_TIGHT_SPEED_M
    tight_width: float = DEFAULT_TIGHT_WIDTH_M

    def validate(self) -> None:
        for name in ("near_speed", "near_dist", "tight_speed", "tight_width"):
            if getattr(self, name) <= 0:
                raise ValueError(f"{name} должен быть > 0")
        if self.tight_speed > self.near_speed:
            raise ValueError("tight_speed не больше near_speed: теснота — самая медленная зона")
        if self.tight_width > 2 * self.near_dist:
            raise ValueError("tight_width не больше 2·near_dist: иначе зона 'у стен' пуста")


def _edt_1d(f: np.ndarray) -> np.ndarray:
    """Квадрат расстояния до ближайшего нуля вдоль оси (Felzenszwalb–Huttenlocher)."""
    n = f.shape[0]
    d = np.empty(n, dtype=np.float64)
    v = np.zeros(n, dtype=np.int64)
    z = np.empty(n + 1, dtype=np.float64)
    k = 0
    v[0] = 0
    z[0] = -np.inf
    z[1] = np.inf
    for q in range(1, n):
        s = ((f[q] + q * q) - (f[v[k]] + v[k] * v[k])) / (2.0 * q - 2.0 * v[k])
        while s <= z[k]:
            k -= 1
            s = ((f[q] + q * q) - (f[v[k]] + v[k] * v[k])) / (2.0 * q - 2.0 * v[k])
        k += 1
        v[k] = q
        z[k] = s
        z[k + 1] = np.inf
    k = 0
    for q in range(n):
        while z[k + 1] < q:
            k += 1
        d[q] = (q - v[k]) ** 2 + f[v[k]]
    return d


def distance_to_occupied_m(occupied: np.ndarray, resolution: float) -> np.ndarray:
    """Евклидово расстояние центра клетки до ближайшей занятой, метры. occupied: bool (h, w).

    Трансформ Felzenszwalb–Huttenlocher: минимум по всем занятым клеткам (q−v)² всегда достигается
    на ближайшей занятой и меньше квадрата диагонали сетки, поэтому вместо inf у «не занятых»
    достаточно большого конечного квадрата — результат точный везде, без вырожденных inf.
    """
    if occupied.dtype != bool:
        raise TypeError("occupied должен быть bool")
    if not occupied.any():
        raise ValueError("нет занятых клеток: расстояние не определено")
    h, w = occupied.shape
    big = float(h * h + w * w + 1)
    f = np.where(occupied, 0.0, big)
    f = np.apply_along_axis(_edt_1d, 1, f)
    d = np.apply_along_axis(_edt_1d, 0, f)
    return np.sqrt(d) * resolution


def _one_sided_dist_cells(occupied: np.ndarray, axis: int, forward: bool) -> np.ndarray:
    """Расстояние в клетках до ближайшей занятой строго по лучу вдоль оси (inf — занятой на луче нет)."""
    h, w = occupied.shape
    out = np.full((h, w), np.inf)
    n_lines, n_cols = (h, w) if axis == 1 else (w, h)
    for i in range(n_lines):
        line = occupied[i, :] if axis == 1 else occupied[:, i]
        idx = np.flatnonzero(line)
        if idx.size == 0:
            continue
        cols = np.arange(n_cols)
        if forward:
            pos = np.searchsorted(idx, cols, side="right")
            has = pos < idx.size
            nearest = idx[np.minimum(pos, idx.size - 1)]
            dist = np.where(has, nearest - cols, np.inf)
        else:
            pos = np.searchsorted(idx, cols, side="left")
            has = pos > 0
            nearest = idx[np.maximum(pos - 1, 0)]
            dist = np.where(has, cols - nearest, np.inf)
        if axis == 1:
            out[i, :] = dist
        else:
            out[:, i] = dist
    return out


def corridor_width_m(occupied: np.ndarray, resolution: float) -> np.ndarray:
    """Ширина свободного коридора в клетке, м: min(луч↔, луч↕) до занятых по четырём лучам."""
    left = _one_sided_dist_cells(occupied, axis=1, forward=False)
    right = _one_sided_dist_cells(occupied, axis=1, forward=True)
    up = _one_sided_dist_cells(occupied, axis=0, forward=False)
    down = _one_sided_dist_cells(occupied, axis=0, forward=True)
    width_x = (left + right) * resolution
    width_y = (up + down) * resolution
    return np.minimum(width_x, width_y)


def zones_mask(occupied: np.ndarray, resolution: float, params: ZoneParams | None = None) -> np.ndarray:
    """Маска зон: 0 — без ограничения, иначе round(100·скорость). occupied: bool (h, w)."""
    p = params or ZoneParams()
    p.validate()
    d = distance_to_occupied_m(occupied, resolution)
    width = corridor_width_m(occupied, resolution)
    mask = np.zeros(occupied.shape, dtype=np.uint8)
    near = d < p.near_dist
    tight = width < p.tight_width
    mask[near] = int(round(p.near_speed * 100))
    mask[tight] = int(round(p.tight_speed * 100))
    return mask


def grid_from_map(grid: OccupancyGrid) -> tuple[np.ndarray, float, float, float]:
    """OccupancyGrid → (occupied bool (h, w), resolution, origin_x, origin_y). Неизвестное — занято."""
    data = np.asarray(grid.data, dtype=np.int16).reshape(grid.height, grid.width)
    occupied = (data < 0) | (data >= OCCUPIED_THRESH)
    return occupied, grid.resolution, grid.origin_x, grid.origin_y


def grid_from_boxes(
    boxes: list[Box],
    resolution: float = DEFAULT_RESOLUTION_M,
    margin_m: float = DEFAULT_MARGIN_M,
    map_pose: Pose2D | None = None,
) -> tuple[np.ndarray, float, float, float]:
    """Коробки мира → occupied-сетка в кадре map. map_pose — поза начала координат map в мире
    (в HIL спавн): коробки переводятся в map как ``map_pose.inverse() ∘ box``."""
    pose = map_pose or Pose2D(0.0, 0.0, 0.0)
    inv = pose.inverse()
    tboxes: list[Box] = []
    for b in boxes:
        p = inv.compose(b.pose)
        # AABB повёрнутого прямоугольника: полуразмеры по осям после поворота.
        c, s = abs(math.cos(p.yaw)), abs(math.sin(p.yaw))
        hx = (b.sx / 2.0) * c + (b.sy / 2.0) * s
        hy = (b.sx / 2.0) * s + (b.sy / 2.0) * c
        tboxes.append(Box(b.name, p.x, p.y, 2.0 * hx, 2.0 * hy, b.height, 0.0))
    if not tboxes:
        raise ValueError("нет коробок")
    min_x = min(b.cx - b.sx / 2.0 for b in tboxes) - margin_m
    min_y = min(b.cy - b.sy / 2.0 for b in tboxes) - margin_m
    max_x = max(b.cx + b.sx / 2.0 for b in tboxes) + margin_m
    max_y = max(b.cy + b.sy / 2.0 for b in tboxes) + margin_m
    w = max(1, int(math.ceil((max_x - min_x) / resolution)))
    h = max(1, int(math.ceil((max_y - min_y) / resolution)))
    occ = np.zeros((h, w), dtype=bool)
    pad = resolution / 2.0
    for b in tboxes:
        c0 = max(0, int(math.floor((b.cx - b.sx / 2.0 - pad - min_x) / resolution)))
        c1 = min(w, int(math.ceil((b.cx + b.sx / 2.0 + pad - min_x) / resolution)))
        r0 = max(0, int(math.floor((b.cy - b.sy / 2.0 - pad - min_y) / resolution)))
        r1 = min(h, int(math.ceil((b.cy + b.sy / 2.0 + pad - min_y) / resolution)))
        occ[r0:r1, c0:c1] = True
    return occ, resolution, min_x, min_y


# ── Запись маски (map_server: Scale, free 0.25 / occupied 0.65) ──────────────


def cell_to_pixel(cell: int) -> int:
    """Значение клетки маски (0..100) → яркость PGM 8-бит (0 — чёрный = самое ограниченное)."""
    if not 0 <= cell <= 100:
        raise ValueError(f"cell {cell} вне 0..100")
    if cell == 0:
        return 255
    occ = FREE_THRESH + (OCC_THRESH - FREE_THRESH) * cell / 100.0
    return int(round(255.0 * (1.0 - occ)))


def write_mask_pgm(mask: np.ndarray, path: Path) -> None:
    """PGM (P5): строка 0 — верх карты (максимальный y), как читает map_server."""
    h, w = mask.shape
    # map_server кладёт пиксель y в строку height-1-y: переварачиваем.
    img = np.flipud(mask)
    with open(path, "wb") as fh:
        fh.write(f"P5\n{w} {h}\n255\n".encode("ascii"))
        fh.write(bytes(int(v) for v in img.reshape(-1)))


def write_mask_yaml(mask: np.ndarray, resolution: float, origin_x: float, origin_y: float, path: Path,
                    image_name: str | None = None) -> None:
    """YAML маски для map_server (mode: scale — три уровня скорости через пороги)."""
    name = image_name or (path.stem + ".pgm")
    text = (
        f"image: {name}\n"
        f"resolution: {resolution}\n"
        f"origin: [{origin_x}, {origin_y}, 0.0]\n"
        "occupied_thresh: 0.65\n"
        "free_thresh: 0.25\n"
        "negate: 0\n"
        "mode: scale\n"
    )
    path.write_text(text, encoding="utf-8")


def save_mask(mask: np.ndarray, resolution: float, origin_x: float, origin_y: float, out_base: Path) -> None:
    """<out_base>.pgm + <out_base>.yaml."""
    out_base.parent.mkdir(parents=True, exist_ok=True)
    pgm = out_base.with_suffix(".pgm")
    write_mask_pgm(mask, pgm)
    write_mask_yaml(mask, resolution, origin_x, origin_y, out_base.with_suffix(".yaml"), pgm.name)


# ── CLI ──────────────────────────────────────────────────────────────────────


def _zone_params_from_args(ap: argparse.Namespace) -> ZoneParams:
    return ZoneParams(
        near_speed=ap.near_speed,
        near_dist=ap.near_dist,
        tight_speed=ap.tight_speed,
        tight_width=ap.tight_width,
    )


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = ap.add_subparsers(dest="kind", required=True)

    pw = sub.add_parser("world", help="маска из геометрии мира (SDF)")
    pw.add_argument("sdf", type=Path)
    pw.add_argument("--map-pose", nargs=3, type=float, default=(0.0, 0.0, 0.0),
                    metavar=("X", "Y", "YAW"), help="поза начала координат map в мире (спавн)")
    pw.add_argument("--resolution", type=float, default=DEFAULT_RESOLUTION_M)
    pw.add_argument("--margin", type=float, default=DEFAULT_MARGIN_M)
    pw.add_argument("--out", type=Path, required=True, help="базовое имя: .pgm + .yaml")
    _add_zone_args(pw)

    pm = sub.add_parser("map", help="маска из снимка карты (npz map_recorder/plan_map)")
    pm.add_argument("npz", type=Path)
    pm.add_argument("--map-pose", nargs=3, type=float, default=(0.0, 0.0, 0.0),
                    metavar=("X", "Y", "YAW"), help="поза начала координат map в системе снимка (спавн сессии)")
    pm.add_argument("--out", type=Path, required=True)
    _add_zone_args(pm)

    args = ap.parse_args(argv)
    params = _zone_params_from_args(args)
    params.validate()

    if args.kind == "world":
        boxes = load_world_boxes(args.sdf)
        occupied, resolution, ox, oy = grid_from_boxes(
            boxes, resolution=args.resolution, margin_m=args.margin, map_pose=Pose2D(*args.map_pose)
        )
    else:
        from rob_box_sim.coverage_plan import load_grid_npz

        px, py, pyaw = args.map_pose
        if abs(pyaw) > 1e-9:
            raise SystemExit("map-режим: map-pose с yaw != 0 не поддержан (поворот сетки): используйте world-режим")
        occupied, resolution, ox, oy = grid_from_map(load_grid_npz(args.npz))
        # Снимок в системе сессии; map новой сессии смещена на (px, py): origin сдвигаем обратно.
        ox -= px
        oy -= py

    mask = zones_mask(occupied, resolution, params)
    near_c = int(round(params.near_speed * 100))
    tight_c = int(round(params.tight_speed * 100))
    cells = int(mask.size)
    print(
        f"mask {mask.shape[0]}x{mask.shape[1]} res={resolution} origin=({ox},{oy}) "
        f"tight({tight_c})={int((mask == tight_c).sum())} near({near_c})={int((mask == near_c).sum())} "
        f"free={int((mask == 0).sum())} of {cells}",
        flush=True,
    )
    save_mask(mask, resolution, ox, oy, args.out)
    print(f"written {args.out.with_suffix('.pgm')} {args.out.with_suffix('.yaml')}", flush=True)
    return 0


def _add_zone_args(ap: argparse.ArgumentParser) -> None:
    ap.add_argument("--near-speed", type=float, default=DEFAULT_NEAR_SPEED_M)
    ap.add_argument("--near-dist", type=float, default=DEFAULT_NEAR_DIST_M)
    ap.add_argument("--tight-speed", type=float, default=DEFAULT_TIGHT_SPEED_M)
    ap.add_argument("--tight-width", type=float, default=DEFAULT_TIGHT_WIDTH_M)


if __name__ == "__main__":
    sys.exit(main())
