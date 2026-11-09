"""Обход покрытием («пылесос», #4560): план полос змейкой по карте. Без ROS, на numpy.

Вход — ``map_check.OccupancyGrid`` (как ``nav_msgs/OccupancyGrid``), габарит
(``geometry2d.Footprint``), шаг полос и стартовая точка. Шаги:

1. **Свободное** — клетка известна и ниже ``free_max`` (как ``free_thresh`` map_server).
   Неизвестное считается занятым: туда центр робота не пускаем.
2. **Сужение** — квадрат ±``erosion_m`` вокруг центра робота не задевает занятого и неизвестного
   (по умолчанию половина большей стороны габарита + ``footprint_padding`` Nav2 = 0.336 м; полосы
   идут вдоль осей сетки, корпус на них — прямоугольник вдоль осей, квадрат его накрывает).
   Это «пространство центров»: проём уже ~0.67 м (габарит 0.605 + 2 × 0.03) в нём исчезает,
   минимальный проём 0.90 м (CONTEXT.md) остаётся.
2a. **Место разворота** — клетка, от центра которой до любой несвободной клетки и до края сетки по
   прямой не меньше ``turn_clearance_m`` (по умолчанию описанный радиус корпуса 0.43 + зона «стоп»
   защиты 0.20 + padding 0.03 = 0.66 м). Там робот крутится на месте, не задевая углами стену и
   не попадая в стоп защиты. Сужение 0.336 этого не даёт: прогон Depot 09.11 (#4560) — концы полос
   в 0.34 м от стены, 58 стопов защиты за 10 мин, 2 цели из 8.
3. **Ячейки** — boustrophedon-разбиение пространства центров по строкам сетки: ячейка
   продолжается, пока отрезок строки соединён с отрезком следующей строки один к одному;
   на развилке и слиянии вокруг препятствия открываются новые ячейки (граф соседства).
4. **Достижимость** — ячейки, связанные по графу с ячейкой старта; остальные помечаются
   «недоступно» и попадают в отчёт с площадью, а не пропадают молча.
5. **Полосы** — строки общей для карты сетки через ≤ ``lane_spacing_m`` (по умолчанию 0.55 м при
   ширине корпуса 0.605 м: перекрытие) плюс крайняя строка ячейки вдоль препятствия; внутри
   ячейки направление чередуется (змейка). Полосы короче ``min_lane_m`` отбрасываются и считаются.
5a. **Концы полос** — полоса идёт по пространству центров (сбоку держит 0.336 у стеллажа и стены), а
   её концы, где робот разворачивается, подрезаются до крайних мест разворота на этой строке. Строка
   без единого места разворота (вдоль внешней стены, тупиковый проход уже 1.32 м) полосой не
   становится и считается в ``lanes_dropped_no_turn``; вместо неё в ячейку добавляется крайняя строка
   с местом разворота; кусок строки вдоль препятствия без места разворота продлевается по строке до
   ближайших мест с обеих сторон. Пол у торцевых стен (≈ 0.66 − 0.31 м), вдоль внешних стен и в тупиках
   уже 1.32 м план не покрывает — это видно в ``plan_coverage``, а не прячется.
5b. **Поперечный проход** (``cross``, по умолчанию да) — полосы другой оси, тоже от места разворота до
   места разворота, только там, где они накрывают ≥ ``CROSS_MIN_GAIN_M2`` пола, не накрытого основным
   проходом: торцы у колонн и стеллажей, где основные полосы подрезаны.
6. **Порядок** — жадно: следующая ячейка та, у которой конец какой-нибудь полосы ближе к текущей
   точке по прямой; внутри ячейки так же выбирается следующая полоса и её направление (на ровной
   ячейке получается змейка). Решает код, не модель (ADR-0148).

Направление полос ``axis``: ``"x"``, ``"y"`` или ``"auto"`` — код строит оба плана и берёт из тех, что
накрывают не меньше ``AXIS_COVER_SHARE`` лучшего, самый быстрый по оценке: путь / ``SPEED_MPS`` +
``PER_POSE_S`` на каждую позу (развороты на концах).

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
#: Потолок скорости Nav2: ``FollowPath.max_vel_x`` из nav2_params.yaml (равенство держит тест
#: test_sim_coverage_turn.py). Оценка плана — по нему; HIL Depot 09.11, попытка 3: средняя 0.06 м/с.
NAV2_MAX_VEL_X = 0.15
#: Скорость для оценки времени, м/с, и накладные на позу (торможение у цели, разворот), с.
#: Накладные — догадка до замера в HIL; ими код выбирает направление полос при axis="auto".
SPEED_MPS = NAV2_MAX_VEL_X
PER_POSE_S = 5.0
#: Зона «стоп» защиты по лидару в коридоре хода, м: зеркало ``rob_box_teleop.proximity.Thresholds.stop_m``.
#: Пакет сима едет на песочницу без teleop (sandbox_prep.sh кладёт только rob_box_sim), поэтому число здесь,
#: а равенство держит тест tests/unit/sim/test_sim_coverage_turn.py.
GUARD_STOP_M = 0.20
#: Поперечная полоса нужна, если накрывает столько покрываемого пола, не накрытого основным проходом, м².
CROSS_MIN_GAIN_M2 = 0.25
#: Ось при axis="auto": план, накрывающий меньше этой доли лучшего, не берётся, даже если быстрее.
AXIS_COVER_SHARE = 0.98


def circumscribed_radius(footprint: Footprint = ROBOT_FOOTPRINT) -> float:
    """Описанный радиус корпуса от base_footprint: столько заметают углы при развороте на месте."""
    return math.hypot(max(footprint.x_max, -footprint.x_min), max(footprint.y_max, -footprint.y_min))


def default_turn_clearance(
    footprint: Footprint = ROBOT_FOOTPRINT, guard_stop: float = GUARD_STOP_M, margin: float = NAV2_PADDING_M
) -> float:
    """До занятого в точке разворота: описанный радиус + зона «стоп» защиты + запас (padding Nav2) = 0.66 м.

    Разворот считается вокруг base_footprint. В симе центр вращения — задняя ось (−0.144 м,
    docs/research/2026-11-05-nav2-hil-tuning.md §10.2): там углам нужно 0.69 м, это ручка ``--turn-clearance``.
    """
    return circumscribed_radius(footprint) + guard_stop + margin


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


def _row_dilate(mask: np.ndarray, w: int) -> np.ndarray:
    """Клетка True, если в её строке на ±``w`` клеток есть True (накопленной суммой, без цикла по w)."""
    cs = np.cumsum(np.pad(mask, ((0, 0), (w + 1, w))), axis=1, dtype=np.int32)
    return (cs[:, 2 * w + 1 :] - cs[:, : -(2 * w + 1)]) > 0


def clear_disc(free: np.ndarray, radius_m: float, resolution: float) -> np.ndarray:
    """Клетки, от центра которых до любой несвободной клетки (её квадрата) и до края сетки ≥ ``radius_m``.

    Круг, а не квадрат, как в ``erode``: разворот на месте заметает круг. Строка смещения ``di`` задевает
    центр, если ``max(|di| − ½, 0)² + max(|dj| − ½, 0)² < (radius / res)²`` — ровно расстояние до квадрата клетки.
    """
    if radius_m <= 0:
        return free.copy()
    r = radius_m / resolution
    k = int(math.ceil(r + 0.5))
    blocked = np.pad(~free, k, constant_values=True)
    h = free.shape[0]
    acc = np.zeros(free.shape[:1] + blocked.shape[1:], dtype=bool)
    rows: dict[int, np.ndarray] = {}
    for di in range(-k, k + 1):
        dy = max(abs(di) - 0.5, 0.0)
        if dy >= r:
            continue
        w = int(math.ceil(math.sqrt(r * r - dy * dy) + 0.5)) - 1
        if w not in rows:
            rows[w] = _row_dilate(blocked, w)
        acc |= rows[w][k + di : k + di + h]
    return ~acc[:, k:-k]


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
    turn_clearance_m: float = 0.0  # 0 — места разворота не проверяются (turn = centers)
    turn: np.ndarray | None = None  # места разворота: центры, от которых до занятого ≥ turn_clearance_m
    cover_half_m: float = 0.3025  # полусторона квадрата, который корпус накрывает на полосе

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
    turn_clearance_m: float | None = None,
    free_max: int = FREE_MAX,
) -> CoverageSpace:
    """Маски карты для плана и метрики; «покрываемое» — свободное в квадрате ±вписанной
    полустороны корпуса (0.3025 м) от достижимого центра: столько корпус накрывает на полосе.
    Знаменатель метрики от места разворота не зависит: недобор у стен виден в покрытии, а не в знаменателе."""
    res = grid.resolution
    erosion_m = default_erosion(footprint) if erosion_m is None else erosion_m
    turn_m = default_turn_clearance(footprint) if turn_clearance_m is None else turn_clearance_m
    free = free_mask(grid_array(grid), free_max)
    centers = erode(free, erosion_m, res)
    rc = (math.floor((start_xy[1] - grid.origin_y) / res), math.floor((start_xy[0] - grid.origin_x) / res))
    snapped = _snap_start(centers, rc)
    reach = np.zeros_like(centers)
    if snapped is not None:
        reach = _component(centers, snapped)
    coverable = free & dilate(reach, int(math.floor(inscribed_half_width(footprint) / res + 1e-9)))
    snap_m = math.hypot(snapped[0] - rc[0], snapped[1] - rc[1]) * res if snapped else None
    turn = centers & clear_disc(free, turn_m, res)
    return CoverageSpace(
        res,
        grid.origin_x,
        grid.origin_y,
        erosion_m,
        free,
        centers,
        reach,
        coverable,
        snapped,
        snap_m,
        turn_m,
        turn,
        inscribed_half_width(footprint),
    )


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
    turn_clearance_m: float = 0.0
    lanes: int = 0
    lanes_dropped_short: int = 0
    lanes_dropped_no_turn: int = 0  # строки без единого места разворота (вдоль внешней стены, тупик)
    clipped_m: float = 0.0  # сколько срезано с концов полос до мест разворота, всего
    cross_lanes: int = 0  # полосы поперечного прохода (входят в lanes)
    swept_m2: float = 0.0  # покрываемого пола под полосами (оценка квадратом корпуса, для выбора оси)
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


def _lane_rows(space: np.ndarray, rows: list, k: int, min_cells: float, turn: np.ndarray) -> list:
    """Полосы ячейки: строки общей для всей карты сетки через ``k`` (соседние ячейки не дублируют
    друг друга) плюс куски крайних строк вдоль препятствия: там корпус должен пройти у самой границы,
    плюс крайние строки ячейки, где есть место разворота (они встают вместо строк у внешней стены)."""
    i0, i1 = rows[0][0], rows[-1][0]
    lanes = {rows[i - i0] for i in range(i0, i1 + 1) if i % k == 0}
    lanes.update(_edge_lanes(space, rows[0], i0 - 1, min_cells))
    lanes.update(_edge_lanes(space, rows[-1], i1 + 1, min_cells))
    with_turn = [r for r in rows if turn[r[0], r[1] : r[2] + 1].any()]
    if with_turn:  # крайняя строка ячейки без места разворота — её заменяет ближайшая строка с ним
        lanes.update(r for r, edge in ((with_turn[0], rows[0]), (with_turn[-1], rows[-1])) if r != edge)
    if not lanes:  # тонкая ячейка между соседями без строки сетки — одна полоса посередине
        lanes.add(rows[(i1 - i0) // 2])
    return sorted(lanes)


def _clip_to_turn(turn: np.ndarray, lane: tuple[int, int, int], run: tuple[int, int, int]):
    """Концы полосы ``lane`` — в местах разворота на её строке ``run`` (отрезок пространства центров).

    Есть места разворота внутри полосы — подрезать до крайних из них. Нет (кусок строки вдоль препятствия) —
    продлить по строке до ближайших мест разворота с обеих сторон. Нет ни там, ни там — None.
    """
    i, a, b = lane
    _, j0, j1 = run
    t = j0 + np.flatnonzero(turn[i, j0 : j1 + 1])
    inside = t[(t >= a) & (t <= b)]
    if len(inside):
        return i, int(inside[0]), int(inside[-1])
    left, right = t[t < a], t[t > b]
    return (i, int(left[-1]), int(right[0])) if len(left) and len(right) else None


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


@dataclass
class _LaneCount:
    short: int = 0
    no_turn: int = 0
    clipped_cells: int = 0


def _cell_lanes(space: np.ndarray, rows, k: int, min_cells: float, turn: np.ndarray, count: _LaneCount) -> list:
    """Полосы ячейки с концами в местах разворота; отброшенные — в ``count``."""
    kept = []
    for lane in _lane_rows(space, rows, k, min_cells, turn):
        clipped = _clip_to_turn(turn, lane, rows[lane[0] - rows[0][0]])
        if clipped is None:
            count.no_turn += 1
        elif clipped[2] - clipped[1] < min_cells:
            count.short += 1
        else:
            count.clipped_cells += max(0, (lane[2] - lane[1]) - (clipped[2] - clipped[1]))
            kept.append(clipped)
    return sorted(set(kept))


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


def _reachable(space: CoverageSpace, frame: _Frame) -> tuple[Decomposition, set[int]]:
    dec = decompose(frame.array(space.centers))
    si, sj = space.start_cell if frame.axis == "x" else space.start_cell[::-1]
    return dec, reachable_cells(dec, int(dec.label[si, sj]))


def _turn(space: CoverageSpace, frame: _Frame) -> np.ndarray:
    return frame.array(space.centers if space.turn is None else space.turn)


def _plan_axis(
    space: CoverageSpace, axis: str, spacing_m: float, min_lane_m: float, step_m: float, cross: bool
) -> CoveragePlan:
    frame = _Frame(space, axis)
    plan = CoveragePlan(axis, spacing_m, space.erosion_m, turn_clearance_m=space.turn_clearance_m)
    if space.start_cell is None:
        return plan
    centers, turn = frame.array(space.centers), _turn(space, frame)
    dec, reach = _reachable(space, frame)
    res = space.resolution
    k = max(1, int(math.floor(spacing_m / res + 1e-9)))
    todo: dict[int, list] = {}
    count = _LaneCount()
    for cid, rows in enumerate(dec.rows):
        lanes = _cell_lanes(centers, rows, k, min_lane_m / res, turn, count if cid in reach else _LaneCount())
        cells_m2 = sum(j1 - j0 + 1 for _, j0, j1 in rows) * res * res
        plan.cells.append(CellInfo(cid, cid in reach, cells_m2, len(lanes) if cid in reach else 0, *_bbox(frame, rows)))
        if cid in reach and lanes:
            todo[cid] = lanes
    plan.lanes_dropped_short, plan.lanes_dropped_no_turn = count.short, count.no_turn
    plan.clipped_m = count.clipped_cells * res
    hw = _cover_cells(space)
    swept = frame.array(_swept(centers.shape, [ln for v in todo.values() for ln in v], hw))
    here = _order_cells(plan, frame, todo, space.cell_xy(*space.start_cell), step_m)
    if cross:
        other = _Frame(space, "y" if axis == "x" else "x")
        extra = _cross_todo(space, other, swept, k, min_lane_m / res, len(plan.cells))
        swept |= other.array(_swept(other.array(swept).shape, [ln for v in extra.values() for ln in v], hw))
        lanes_before = plan.lanes
        _order_cells(plan, other, extra, here, step_m)
        plan.cross_lanes = plan.lanes - lanes_before
    plan.swept_m2 = space.area(space.coverable & swept)
    return plan


def _cover_cells(space: CoverageSpace) -> int:
    """Полусторона квадрата, который корпус накрывает на полосе, в клетках (как у ``coverable``)."""
    return int(math.floor(space.cover_half_m / space.resolution + 1e-9))


def _swept(shape: tuple[int, int], lanes, hw: int) -> np.ndarray:
    """Что корпус накрывает на полосах (оси кадра): квадрат ±``hw`` клеток вдоль каждого отрезка."""
    m = np.zeros(shape, dtype=bool)
    for i, j0, j1 in lanes:
        m[max(i - hw, 0) : i + hw + 1, max(j0 - hw, 0) : j1 + hw + 1] = True
    return m


def _cross_spans(need: np.ndarray, turn: np.ndarray, lane: tuple[int, int, int], hw: int) -> list:
    """Куски строки поперечного прохода: каждый от места разворота до места разворота и накрывает непокрытое.

    Непокрытые клетки рядом со строкой бьются на группы (разрыв больше корпуса — новая группа); группа без места
    разворота с какой-нибудь стороны (тупик: выезжать задом план не умеет) пропускается.
    """
    i, j0, j1 = lane
    col = need[max(i - hw, 0) : i + hw + 1].any(axis=0)
    useful = j0 + np.flatnonzero(_row_dilate(col[None, :], hw)[0][j0 : j1 + 1])
    t = j0 + np.flatnonzero(turn[i, j0 : j1 + 1])
    spans = []
    for grp in np.split(useful, np.flatnonzero(np.diff(useful) > 2 * hw + 1) + 1) if len(useful) else []:
        left, right = t[t <= grp[0]], t[t >= grp[-1]]
        if len(left) and len(right):
            spans.append((i, int(left[-1]), int(right[0])))
    return spans


def _cross_todo(space: CoverageSpace, frame: _Frame, swept_grid: np.ndarray, k: int, min_cells: float, cid0: int):
    """Поперечный проход: полосы другой оси с концами в местах разворота — только те, что накрывают
    ≥ ``CROSS_MIN_GAIN_M2`` покрываемого пола, не накрытого основным проходом (торцы у препятствий,
    где основные полосы подрезаны). Жадно по выигрышу; накрытое следующей полосой уже не считается."""
    dec, reach = _reachable(space, frame)
    centers, turn, hw = frame.array(space.centers), _turn(space, frame), _cover_cells(space)
    need = frame.array(space.coverable) & ~frame.array(swept_grid)
    cands = {
        (cid, sp)
        for cid in reach
        for ln in _lane_rows(centers, dec.rows[cid], k, min_cells, turn)
        for sp in _cross_spans(need, turn, dec.rows[cid][ln[0] - dec.rows[cid][0][0]], hw)
        if sp[2] - sp[1] >= min_cells
    }
    todo: dict[int, list] = {}
    min_gain = CROSS_MIN_GAIN_M2 / space.resolution**2
    while cands:
        scored = [(_gain(need, sp, hw), cid, sp) for cid, sp in cands]
        gain, cid, sp = max(scored)
        if gain < min_gain:
            break
        need &= ~_swept(need.shape, [sp], hw)
        todo.setdefault(cid0 + cid, []).append(sp)
        cands = {(c, x) for g, c, x in scored if g >= min_gain and x != sp}
    return todo


def _gain(need: np.ndarray, span: tuple[int, int, int], hw: int) -> int:
    i, j0, j1 = span
    return int(need[max(i - hw, 0) : i + hw + 1, max(j0 - hw, 0) : j1 + hw + 1].sum())


def _order_cells(plan: CoveragePlan, frame: _Frame, todo: dict, here: tuple[float, float], step_m: float) -> tuple:
    """Жадно: следующая ячейка — с ближайшим по прямой концом полосы; внутри ячейки — так же по полосам
    (на ровной ячейке это змейка: ближайший конец — у соседней полосы с той же стороны). Вернуть, где кончили."""
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
    return here


def plan_coverage(
    space: CoverageSpace,
    *,
    axis: str = "auto",
    lane_spacing_m: float = LANE_SPACING_M,
    min_lane_m: float = MIN_LANE_M,
    waypoint_step_m: float = 0.0,
    cross: bool = True,
) -> CoveragePlan:
    """План обхода (см. шапку модуля). ``waypoint_step_m`` > 0 — промежуточные позы на полосе;
    ``cross`` — поперечный проход по тому, что основной не накрыл (см. ``_cross_todo``)."""
    if axis not in ("x", "y", "auto"):
        raise ValueError(f"axis: x | y | auto, получено {axis!r}")
    axes = ("x", "y") if axis == "auto" else (axis,)
    plans = [_plan_axis(space, a, lane_spacing_m, min_lane_m, waypoint_step_m, cross) for a in axes]
    return _pick(plans)


def _pick(plans: list[CoveragePlan]) -> CoveragePlan:
    """Из планов по осям: среди тех, что накрывают не меньше ``AXIS_COVER_SHARE`` лучшего, — самый быстрый.

    С местами разворота оси покрывают по-разному (Depot-подобный зал: 72 % против 90 %), одного времени мало.
    """
    best = max(p.swept_m2 for p in plans)
    ok = [p for p in plans if p.swept_m2 >= AXIS_COVER_SHARE * best]
    return min(ok, key=lambda p: (p.time_estimate_s(), p.route_m))


# ── Отчёт и файлы ────────────────────────────────────────────────────────────


def turn_pose_clearance(space: CoverageSpace, poses: Sequence[CoveragePose]) -> float | None:
    """Наименьшее расстояние от поз разворота (start/end) до квадрата несвободной клетки или края сетки, м."""
    occ = np.argwhere(~space.free)
    h, w = space.free.shape
    best = None
    for p in poses:
        if p.kind == "mid":
            continue
        u, v = (p.x - space.origin_x) / space.resolution, (p.y - space.origin_y) / space.resolution
        d = min(u, w - u, v, h - v)
        if len(occ):
            dx = np.maximum(np.abs(occ[:, 1] + 0.5 - u) - 0.5, 0.0)
            dy = np.maximum(np.abs(occ[:, 0] + 0.5 - v) - 0.5, 0.0)
            d = min(d, float(np.sqrt((dx * dx + dy * dy).min())))
        best = d if best is None else min(best, d)
    return None if best is None else best * space.resolution


def plan_summary(plan: CoveragePlan, space: CoverageSpace, speed_mps: float = SPEED_MPS) -> str:
    """Строка итога плана; числа — из плана, не из модели."""
    un = plan.unreachable
    turn_min = turn_pose_clearance(space, plan.poses)
    return (
        f"PLAN axis={plan.axis} spacing={plan.lane_spacing_m:.2f} erosion={plan.erosion_m:.3f} "
        f"turn_clearance={plan.turn_clearance_m:.3f} "
        f"min_turn_clear_m={'n/a' if turn_min is None else f'{turn_min:.3f}'} "
        f"poses={len(plan.poses)} lanes={plan.lanes} lanes_dropped_short={plan.lanes_dropped_short} "
        f"lanes_dropped_no_turn={plan.lanes_dropped_no_turn} clipped_m={plan.clipped_m:.1f} "
        f"cross_lanes={plan.cross_lanes} "
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
    ap.add_argument("--turn-clearance", type=float, default=None, help="до занятого в точке разворота, м; 0 — нет")
    ap.add_argument("--no-cross", action="store_true", help="без поперечного прохода")
    ap.add_argument("--waypoint-step", type=float, default=0.0)
    ap.add_argument("--json", type=Path, default=None)
    args = ap.parse_args(argv)
    grid = load_grid_npz(args.map)
    space = coverage_space(grid, tuple(args.start), erosion_m=args.erosion, turn_clearance_m=args.turn_clearance)
    plan = plan_coverage(
        space,
        axis=args.axis,
        lane_spacing_m=args.spacing,
        waypoint_step_m=args.waypoint_step,
        cross=not args.no_cross,
    )
    print(plan_summary(plan, space))
    if args.json:
        args.json.write_text(json.dumps(plan_to_json(plan), ensure_ascii=False, indent=1), encoding="utf-8")
    return 0


if __name__ == "__main__":
    sys.exit(main())
