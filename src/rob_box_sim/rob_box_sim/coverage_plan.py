"""Обход покрытием («пылесос», #4560): план полос змейкой по карте. Без ROS, на numpy.

Вход — ``map_check.OccupancyGrid`` (как ``nav_msgs/OccupancyGrid``), габарит
(``geometry2d.Footprint``), шаг полос и стартовая точка. Шаги:

1. **Свободное** — клетка известна и ниже ``free_max`` (как ``free_thresh`` map_server).
   Неизвестное считается занятым: туда центр робота не пускаем.
2. **Сужение** — квадрат ±``erosion_m`` вокруг центра робота не задевает занятого и неизвестного
   (по умолчанию половина большей стороны габарита + ``footprint_padding`` Nav2 = 0.336 м; полосы
   идут вдоль осей сетки, корпус на них — прямоугольник вдоль осей, квадрат его накрывает).
   Это «пространство центров»: проём уже ~0.67 м (габарит 0.605 + 2 × 0.03) в нём исчезает,
   минимальный проём 0.90 м (CONTEXT.md) остаётся. Развороты на концах полос (описанный
   радиус 0.43 м) план не гарантирует — это работа Nav2 (#4475).
3. **Ячейки** — boustrophedon-разбиение пространства центров по строкам сетки: ячейка
   продолжается, пока отрезок строки соединён с отрезком следующей строки один к одному;
   на развилке и слиянии вокруг препятствия открываются новые ячейки (граф соседства).
4. **Достижимость** — ячейки, связанные по графу с ячейкой старта; остальные помечаются
   «недоступно» и попадают в отчёт с площадью, а не пропадают молча.
5. **Полосы** — строки общей для карты сетки через ≤ ``lane_spacing_m`` (по умолчанию 0.55 м при
   ширине корпуса 0.605 м: перекрытие) плюс крайняя строка ячейки вдоль препятствия; внутри
   ячейки направление чередуется (змейка). Полосы короче ``min_lane_m`` отбрасываются и считаются.
6. **Порядок** — жадно: следующая ячейка та, у которой конец какой-нибудь полосы ближе к текущей
   точке по прямой; внутри ячейки так же выбирается следующая полоса и её направление (на ровной
   ячейке получается змейка). Решает код, не модель (ADR-0148).

Направление полос ``axis``: ``"x"``, ``"y"`` или ``"auto"`` — код строит оба плана и берёт тот,
что быстрее по оценке: путь / ``SPEED_MPS`` + ``PER_POSE_S`` на каждую позу (развороты на концах).

Пространство (маски свободного, центров, достижимого и «покрываемого» пола) считает
``coverage_space``; им же пользуется метрика ``coverage_metric``, чтобы план и прогон
мерились одной меркой.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Sequence

import numpy as np
from rob_box_sim.geometry2d import ROBOT_FOOTPRINT, Footprint
from rob_box_sim.map_check import OccupancyGrid

#: footprint_padding Nav2 (docker/main/config/nav2/nav2_params.yaml).
NAV2_PADDING_M = 0.03
#: Свободно — значение клетки 0..FREE_MAX (map_server free_thresh 0.25).
FREE_MAX = 25
#: Шаг полос по умолчанию: меньше ширины корпуса 0.605 м — полосы перекрываются.
LANE_SPACING_M = 0.55
#: Полоса короче — не едем (разворот дороже, чем покрытый кусок).
MIN_LANE_M = 0.30
#: Скорость для оценки времени, м/с, и накладные на позу (торможение у цели, разворот), с.
#: Накладные — догадка до замера в HIL; ими код выбирает направление полос при axis="auto".
SPEED_MPS = 0.3
PER_POSE_S = 5.0


def default_erosion(footprint: Footprint = ROBOT_FOOTPRINT, padding: float = NAV2_PADDING_M) -> float:
    """Половина большей стороны габарита + padding: столько центр держится от препятствия."""
    half = max(footprint.x_max, -footprint.x_min, footprint.y_max, -footprint.y_min)
    return half + padding


def inscribed_half_width(footprint: Footprint = ROBOT_FOOTPRINT) -> float:
    """Полусторона квадрата, который корпус накрывает в позе вдоль любой оси."""
    return min(footprint.x_max, -footprint.x_min, footprint.y_max, -footprint.y_min)


# ── Маски ────────────────────────────────────────────────────────────────────


def grid_array(grid: OccupancyGrid) -> np.ndarray:
    """data → массив [строка=y, столбец=x], строка 0 — у origin."""
    return np.asarray(grid.data, dtype=np.int16).reshape(grid.height, grid.width)


def free_mask(values: np.ndarray, free_max: int = FREE_MAX) -> np.ndarray:
    return (values >= 0) & (values <= free_max)


def dilate(mask: np.ndarray, half_cells: int) -> np.ndarray:
    """Клетка True, если в квадрате ±``half_cells`` (по Чебышёву) есть True. Раздельно по осям."""
    out = mask
    for axis in (0, 1):
        n = out.shape[axis]
        pad = np.pad(out, [(half_cells, half_cells) if a == axis else (0, 0) for a in (0, 1)])
        acc = np.zeros_like(out)
        for d in range(2 * half_cells + 1):
            acc |= np.take(pad, range(d, d + n), axis=axis)
        out = acc
    return out


def erode(free: np.ndarray, half_m: float, resolution: float) -> np.ndarray:
    """Пространство центров: квадрат ±``half_m`` вокруг центра не задевает ни одной несвободной
    клетки (и края сетки). Полосы идут вдоль осей сетки, поэтому корпус на полосе — прямоугольник
    вдоль осей, и квадрат с полустороной «большая полусторона габарита + padding» его накрывает.

    Клетка-препятствие — квадрат: она задевает квадрат робота, если до её центра меньше
    ``half_m`` + полклетки.
    """
    k = int(math.floor(half_m / resolution + 0.5))
    if k <= 0:
        return free.copy()
    blocked = np.pad(~free, k, constant_values=True)
    return ~dilate(blocked, k)[k:-k, k:-k]


def row_runs(row: np.ndarray) -> list[tuple[int, int]]:
    """Отрезки True в строке: [(c0, c1)] включительно."""
    d = np.diff(np.concatenate(([0], row.astype(np.int8), [0])))
    starts = np.flatnonzero(d == 1)
    ends = np.flatnonzero(d == -1) - 1
    return list(zip(starts.tolist(), ends.tolist()))


# ── Разбиение на ячейки ──────────────────────────────────────────────────────


@dataclass
class Decomposition:
    label: np.ndarray  # [i, j] → id ячейки, −1 вне пространства центров
    rows: list[list[tuple[int, int, int]]]  # по ячейке: (i, j0, j1) строка за строкой
    adjacency: list[set[int]]


def _overlaps(prev: Sequence[tuple[int, int]], cur: Sequence[tuple[int, int]]):
    up: dict[int, list[int]] = {i: [] for i in range(len(prev))}
    down: dict[int, list[int]] = {j: [] for j in range(len(cur))}
    for i, (a0, a1) in enumerate(prev):
        for j, (b0, b1) in enumerate(cur):
            if a0 <= b1 and b0 <= a1:
                up[i].append(j)
                down[j].append(i)
    return up, down


def decompose(space: np.ndarray) -> Decomposition:
    """Boustrophedon по строкам: ячейка живёт, пока отрезки строк связаны один к одному."""
    dec = Decomposition(np.full(space.shape, -1, dtype=np.int32), [], [])
    prev: list[tuple[int, int]] = []
    open_cells: dict[int, int] = {}
    for i in range(space.shape[0]):
        cur = row_runs(space[i])
        up, down = _overlaps(prev, cur)
        nxt = {}
        for j, run in enumerate(cur):
            parents = down[j]
            if len(parents) == 1 and len(up[parents[0]]) == 1:
                cid = open_cells[parents[0]]
            else:
                cid = _new_cell(dec, [open_cells[p] for p in parents])
            dec.rows[cid].append((i, run[0], run[1]))
            dec.label[i, run[0] : run[1] + 1] = cid
            nxt[j] = cid
        prev, open_cells = cur, nxt
    return dec


def _new_cell(dec: Decomposition, neighbours: Sequence[int]) -> int:
    cid = len(dec.rows)
    dec.rows.append([])
    dec.adjacency.append(set(neighbours))
    for n in neighbours:
        dec.adjacency[n].add(cid)
    return cid


def reachable_cells(dec: Decomposition, start_cell: int) -> set[int]:
    seen = {start_cell}
    stack = [start_cell]
    while stack:
        for n in dec.adjacency[stack.pop()]:
            if n not in seen:
                seen.add(n)
                stack.append(n)
    return seen


# ── Пространство покрытия ────────────────────────────────────────────────────


@dataclass
class CoverageSpace:
    """Маски одной карты в осях сетки [строка=y, столбец=x] и что из них достижимо."""

    resolution: float
    origin_x: float
    origin_y: float
    erosion_m: float
    free: np.ndarray  # свободный пол
    centers: np.ndarray  # пространство центров (после сужения)
    reachable: np.ndarray  # центры, связанные со стартом
    coverable: np.ndarray  # свободный пол, который корпус накрывает из достижимых центров
    start_cell: tuple[int, int] | None  # (строка, столбец) старта в пространстве центров
    start_snap_m: float | None  # насколько старт сдвинут до ближайшего центра

    def cell_xy(self, r: int, c: int) -> tuple[float, float]:
        return self.origin_x + (c + 0.5) * self.resolution, self.origin_y + (r + 0.5) * self.resolution

    def area(self, mask: np.ndarray) -> float:
        return float(mask.sum()) * self.resolution**2


def _snap_start(centers: np.ndarray, rc: tuple[int, int]) -> tuple[int, int] | None:
    h, w = centers.shape
    if 0 <= rc[0] < h and 0 <= rc[1] < w and centers[rc]:
        return rc
    pts = np.argwhere(centers)
    if not len(pts):
        return None
    k = int(np.argmin((pts[:, 0] - rc[0]) ** 2 + (pts[:, 1] - rc[1]) ** 2))
    return int(pts[k, 0]), int(pts[k, 1])


def coverage_space(
    grid: OccupancyGrid,
    start_xy: tuple[float, float],
    *,
    footprint: Footprint = ROBOT_FOOTPRINT,
    erosion_m: float | None = None,
    free_max: int = FREE_MAX,
) -> CoverageSpace:
    """Маски карты для плана и метрики; «покрываемое» — свободное в квадрате ±вписанной
    полустороны корпуса (0.3025 м) от достижимого центра: столько корпус накрывает на полосе."""
    res = grid.resolution
    erosion_m = default_erosion(footprint) if erosion_m is None else erosion_m
    free = free_mask(grid_array(grid), free_max)
    centers = erode(free, erosion_m, res)
    rc = (math.floor((start_xy[1] - grid.origin_y) / res), math.floor((start_xy[0] - grid.origin_x) / res))
    snapped = _snap_start(centers, rc)
    reach = np.zeros_like(centers)
    if snapped is not None:
        reach = _component(centers, snapped)
    coverable = free & dilate(reach, int(math.floor(inscribed_half_width(footprint) / res + 1e-9)))
    snap_m = math.hypot(snapped[0] - rc[0], snapped[1] - rc[1]) * res if snapped else None
    return CoverageSpace(res, grid.origin_x, grid.origin_y, erosion_m, free, centers, reach, coverable, snapped, snap_m)


def _component(centers: np.ndarray, rc: tuple[int, int]) -> np.ndarray:
    dec = decompose(centers)
    ids = reachable_cells(dec, int(dec.label[rc]))
    return np.isin(dec.label, list(ids))


# ── План ─────────────────────────────────────────────────────────────────────


@dataclass(frozen=True)
class CoveragePose:
    x: float
    y: float
    yaw: float  # рад
    cell: int
    lane: int  # номер полосы внутри ячейки по порядку проезда
    kind: str  # start | mid | end


@dataclass(frozen=True)
class CellInfo:
    id: int
    reachable: bool
    centers_m2: float  # площадь пространства центров ячейки
    lanes: int  # полос в плане (после отбрасывания коротких)
    x_min: float
    x_max: float
    y_min: float
    y_max: float


@dataclass
class CoveragePlan:
    axis: str
    lane_spacing_m: float
    erosion_m: float
    poses: list[CoveragePose] = field(default_factory=list)
    cells: list[CellInfo] = field(default_factory=list)
    lanes: int = 0
    lanes_dropped_short: int = 0
    route_m: float = 0.0  # по прямым от старта через все позы

    @property
    def unreachable(self) -> list[CellInfo]:
        return [c for c in self.cells if not c.reachable]

    def time_estimate_s(self, speed_mps: float = SPEED_MPS, per_pose_s: float = PER_POSE_S) -> float:
        return self.route_m / speed_mps + per_pose_s * len(self.poses)


class _Frame:
    """Оси полос: полосы идут вдоль столбцов массива ``a``; для ``axis="y"`` это транспонированная сетка."""

    def __init__(self, space: CoverageSpace, axis: str) -> None:
        self.space, self.axis = space, axis

    def array(self, m: np.ndarray) -> np.ndarray:
        return m if self.axis == "x" else m.T

    def xy(self, i: int, j: int) -> tuple[float, float]:
        return self.space.cell_xy(i, j) if self.axis == "x" else self.space.cell_xy(j, i)

    def yaw(self, forward: bool) -> float:
        base = 0.0 if self.axis == "x" else math.pi / 2
        return base if forward else _wrap(base + math.pi)


def _wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def _edge_lanes(space: np.ndarray, row: tuple[int, int, int], beyond: int, min_cells: float) -> list:
    """Куски крайней строки ячейки, за которыми препятствие (а не соседняя ячейка), длиной ≥ ``min_cells``."""
    i, j0, j1 = row
    if not 0 <= beyond < space.shape[0]:
        return [row]
    return [(i, j0 + a, j0 + b) for a, b in row_runs(~space[beyond, j0 : j1 + 1]) if b - a >= min_cells]


def _lane_rows(space: np.ndarray, rows: list, k: int, min_cells: float) -> list[tuple[int, int, int]]:
    """Полосы ячейки: строки общей для всей карты сетки через ``k`` (соседние ячейки не дублируют
    друг друга) плюс куски крайних строк вдоль препятствия: там корпус должен пройти у самой границы."""
    i0, i1 = rows[0][0], rows[-1][0]
    lanes = {rows[i - i0] for i in range(i0, i1 + 1) if i % k == 0}
    lanes.update(_edge_lanes(space, rows[0], i0 - 1, min_cells))
    lanes.update(_edge_lanes(space, rows[-1], i1 + 1, min_cells))
    if not lanes:  # тонкая ячейка между соседями без строки сетки — одна полоса посередине
        lanes.add(rows[(i1 - i0) // 2])
    return sorted(lanes)


def _lane_poses(frame: _Frame, lane: tuple[int, int, int], forward: bool, step_m: float) -> list[tuple]:
    i, j0, j1 = lane
    js = [j0, j1] if forward else [j1, j0]
    if step_m > 0:
        n = max(1, int(math.ceil((j1 - j0) * frame.space.resolution / step_m)))
        js = [int(round(v)) for v in np.linspace(js[0], js[1], n + 1)]
    yaw = frame.yaw(forward)
    out = []
    for k, j in enumerate(js):
        kind = "start" if k == 0 else "end" if k == len(js) - 1 else "mid"
        out.append((*frame.xy(i, j), yaw, kind))
    return out


def _cell_lanes(space: np.ndarray, rows, k: int, min_cells: float) -> tuple[list, int]:
    lanes = _lane_rows(space, rows, k, min_cells)
    kept = [ln for ln in lanes if (ln[2] - ln[1]) >= min_cells]
    return kept, len(lanes) - len(kept)


def _ends(frame: _Frame, lane: tuple[int, int, int]) -> tuple[tuple[float, float], tuple[float, float]]:
    i, j0, j1 = lane
    return frame.xy(i, j0), frame.xy(i, j1)


def _nearest_lane(frame: _Frame, lanes: list, here: tuple[float, float]) -> tuple[float, int, bool]:
    """(расстояние, индекс полосы, ехать ли вперёд) — до ближайшего конца любой полосы."""
    best = (math.inf, 0, True)
    for k, lane in enumerate(lanes):
        a, b = _ends(frame, lane)
        best = min(best, (math.dist(here, a), k, True), (math.dist(here, b), k, False))
    return best


def _bbox(frame: _Frame, rows) -> tuple[float, float, float, float]:
    pts = [frame.xy(i, j) for i, j0, j1 in rows for j in (j0, j1)]
    xs, ys = [p[0] for p in pts], [p[1] for p in pts]
    return min(xs), max(xs), min(ys), max(ys)


def _plan_axis(space: CoverageSpace, axis: str, spacing_m: float, min_lane_m: float, step_m: float) -> CoveragePlan:
    frame = _Frame(space, axis)
    plan = CoveragePlan(axis, spacing_m, space.erosion_m)
    if space.start_cell is None:
        return plan
    centers = frame.array(space.centers)
    dec = decompose(centers)
    si, sj = space.start_cell if axis == "x" else space.start_cell[::-1]
    reach = reachable_cells(dec, int(dec.label[si, sj]))
    res = space.resolution
    k = max(1, int(math.floor(spacing_m / res + 1e-9)))
    todo: dict[int, list] = {}
    for cid, rows in enumerate(dec.rows):
        lanes, dropped = _cell_lanes(centers, rows, k, min_lane_m / res)
        plan.lanes_dropped_short += dropped if cid in reach else 0
        cells_m2 = sum(j1 - j0 + 1 for _, j0, j1 in rows) * res * res
        plan.cells.append(CellInfo(cid, cid in reach, cells_m2, len(lanes) if cid in reach else 0, *_bbox(frame, rows)))
        if cid in reach and lanes:
            todo[cid] = lanes
    _order_cells(plan, frame, todo, space.cell_xy(*space.start_cell), step_m)
    return plan


def _order_cells(plan: CoveragePlan, frame: _Frame, todo: dict, here: tuple[float, float], step_m: float) -> None:
    """Жадно: следующая ячейка — с ближайшим по прямой концом полосы; внутри ячейки — так же по полосам
    (на ровной ячейке это змейка: ближайший конец — у соседней полосы с той же стороны)."""
    while todo:
        _, cid = min((_nearest_lane(frame, lanes, here)[0], cid) for cid, lanes in todo.items())
        lanes = list(todo.pop(cid))
        for k in range(len(lanes)):
            _, idx, fwd = _nearest_lane(frame, lanes, here)
            for x, y, yaw, kind in _lane_poses(frame, lanes.pop(idx), fwd, step_m):
                plan.route_m += math.dist(here, (x, y))
                here = (x, y)
                plan.poses.append(CoveragePose(x, y, yaw, cid, k, kind))
            plan.lanes += 1


def plan_coverage(
    space: CoverageSpace,
    *,
    axis: str = "auto",
    lane_spacing_m: float = LANE_SPACING_M,
    min_lane_m: float = MIN_LANE_M,
    waypoint_step_m: float = 0.0,
) -> CoveragePlan:
    """План обхода (см. шапку модуля). ``waypoint_step_m`` > 0 — промежуточные позы на полосе."""
    if axis not in ("x", "y", "auto"):
        raise ValueError(f"axis: x | y | auto, получено {axis!r}")
    axes = ("x", "y") if axis == "auto" else (axis,)
    plans = [_plan_axis(space, a, lane_spacing_m, min_lane_m, waypoint_step_m) for a in axes]
    return min(plans, key=lambda p: (p.time_estimate_s(), p.route_m))


# ── Отчёт и файлы ────────────────────────────────────────────────────────────


def plan_summary(plan: CoveragePlan, space: CoverageSpace, speed_mps: float = SPEED_MPS) -> str:
    """Строка итога плана; числа — из плана, не из модели."""
    un = plan.unreachable
    return (
        f"PLAN axis={plan.axis} spacing={plan.lane_spacing_m:.2f} erosion={plan.erosion_m:.3f} "
        f"poses={len(plan.poses)} lanes={plan.lanes} lanes_dropped_short={plan.lanes_dropped_short} "
        f"cells={len(plan.cells)} unreachable_cells={len(un)} "
        f"unreachable_centers_m2={sum(c.centers_m2 for c in un):.1f} "
        f"route_m={plan.route_m:.1f} time_est_s={plan.time_estimate_s(speed_mps):.0f}"
        f"@{speed_mps}m/s+{PER_POSE_S:.0f}s/pose "
        f"free_m2={space.area(space.free):.1f} coverable_m2={space.area(space.coverable):.1f} "
        f"start_snap_m={space.start_snap_m if space.start_snap_m is None else round(space.start_snap_m, 2)}"
    )


def plan_to_json(plan: CoveragePlan) -> dict:
    out = asdict(plan)
    out["unreachable_cells"] = [c.id for c in plan.unreachable]
    return out


def load_grid_npz(path: Path) -> OccupancyGrid:
    """Снимок карты в формате ``map_recorder`` (t, resolution, origin_x, origin_y, data[r, c])."""
    z = np.load(path)
    data = np.asarray(z["data"], dtype=np.int16)
    h, w = data.shape
    return OccupancyGrid(
        w, h, float(z["resolution"]), float(z["origin_x"]), float(z["origin_y"]), data.reshape(-1).tolist()
    )


def save_grid_npz(grid: OccupancyGrid, path: Path, t: float = 0.0) -> None:
    data = np.asarray(grid.data, dtype=np.int8).reshape(grid.height, grid.width)
    np.savez_compressed(
        path, t=t, resolution=grid.resolution, origin_x=grid.origin_x, origin_y=grid.origin_y, data=data
    )


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="План обхода покрытием по снимку карты (npz map_recorder)")
    ap.add_argument("map", type=Path)
    ap.add_argument("--start", type=float, nargs=2, default=(0.0, 0.0), metavar=("X", "Y"))
    ap.add_argument("--axis", default="auto", choices=("auto", "x", "y"))
    ap.add_argument("--spacing", type=float, default=LANE_SPACING_M)
    ap.add_argument("--erosion", type=float, default=None)
    ap.add_argument("--waypoint-step", type=float, default=0.0)
    ap.add_argument("--json", type=Path, default=None)
    args = ap.parse_args(argv)
    grid = load_grid_npz(args.map)
    space = coverage_space(grid, tuple(args.start), erosion_m=args.erosion)
    plan = plan_coverage(space, axis=args.axis, lane_spacing_m=args.spacing, waypoint_step_m=args.waypoint_step)
    print(plan_summary(plan, space))
    if args.json:
        args.json.write_text(json.dumps(plan_to_json(plan), ensure_ascii=False, indent=1), encoding="utf-8")
    return 0


if __name__ == "__main__":
    sys.exit(main())
