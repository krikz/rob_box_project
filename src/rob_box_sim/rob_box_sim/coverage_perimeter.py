"""Проход по периметру (#4607): добрать пол у стен, который змейка ``coverage_plan`` не накрыла. Без ROS, на numpy.

Змейка держит концы полос в местах разворота (0.66 м от занятого), поэтому полоса пола у внешних стен шириной
~0.3 м остаётся непокрытой: строку вдоль стены змейка не берёт, развернуться там негде (Depot 09.11: ~37 м²).

**Нырок полосы.** Базовая строка — любой отрезок достижимых центров (обе оси); если стена с какой-то стороны ближе
``turn_clearance``, середина строки плавно уводится к стене на ``wall_offset_m``, а концы пути встают в места разворота
на базовой строке до и после нырка (``dip_spans``). Робот заезжает и выезжает **только вперёд**: без разворота
у стены и без заднего хода. Соседние базовые строки дают почти одинаковые нырки — лишние отсеет жадный отбор.
Едет робот по этому пути целью ``FollowPath`` контроллера Nav2 (DWB, его проверка столкновений по costmap) —
путь строит код, а не планировщик Nav2 (ADR-0148).

**Отступ от стены** — ``wall_offset_m`` = половина ширины корпуса 0.3025 + стоп защиты сбоку 0.08 + padding 0.03 =
0.4125 м. Меньше нельзя: защита (``rob_box_teleop.proximity``) тормозит, если стена ближе 0.05 м к борту (коридор
хода — корпус ± 0.05 м, стоп с 0.20 м) или ближе 0.08 м к углу, к которому корпус едет. Расчёт той же функцией
защиты: центр в 0.336 м от стены, ход вдоль неё 0.15 м/с — «стоп»; в 0.38 м и дальше — «внимание».

**Профиль нырка** (на каждый столбец полосы — глубина в клетках к стене):

1. ``depth`` — на сколько строк от базовой можно сместиться к стене, не выйдя из «пространства прохода»
   (прямоугольник ±``wall_offset_m`` поперёк и ±``erosion_m`` вдоль не задевает несвободного), если край этого
   пространства — в пределах ``turn_clearance``; иначе 0 (стены рядом нет).
2. ``envelope`` — минимум ``depth`` на ±(нос 0.3064 + стоп 0.20 + padding 0.03) вдоль полосы: препятствие впереди
   (пилястра, колонна у стены) не должно попасть в коридор хода ближе стопа.
3. Склон: к стене не круче ``RAMP_GUARD_SHARE`` от того, что пускает защита: стена входит в коридор хода
   (борт + 0.05 м) не ближе ``нос + стоп`` впереди, т. е. ``tg θ ≤ (зазор − 0.3525) / 0.5064``. Тот же склон
   и от стены: путь не зависит от направления.

Каждая поза пути проверяется: корпус с padding не задевает несвободного (``footprint_hits``) и защита не дала бы
стоп при ходе вперёд (``guard_stops``: правило ``proximity.stop_limits`` по углам несвободных клеток карты). Полоса
с хоть одной плохой позой в направлении не едет; без годного направления — отброшена и посчитана.

Нырок берётся, если накрывает ≥ ``PERIM_MIN_GAIN_M2`` пола, не накрытого змейкой и уже взятыми нырками (жадно по
выигрышу). Порядок — жадно по ближайшему концу от конца змейки.

Чего здесь нет: южный блок ящиков Depot с тупиками уже 1.32 м — там выезд только задом (``BackUp``), это отдельный
шаг. Курс на склоне защита считает и по вращению корпуса; проверка здесь — при ``wz = 0`` (склон пологий).
"""

from __future__ import annotations

import argparse
import heapq
import math
import sys
from collections import Counter
from dataclasses import dataclass, field
from pathlib import Path
from typing import Sequence

import numpy as np
from rob_box_sim.coverage_metric import CoverageResult, GoalOutcome, coverage_result, covered_mask, plan_track
from rob_box_sim.coverage_plan import (
    GUARD_STOP_M,
    NAV2_PADDING_M,
    PER_POSE_S,
    SPEED_MPS,
    CoveragePlan,
    CoverageSpace,
    _Frame,
    _row_dilate,
    _turn,
    coverage_space,
    inscribed_half_width,
    load_grid_npz,
    plan_coverage,
    plan_summary,
)
from rob_box_sim.geometry2d import ROBOT_FOOTPRINT, Footprint

#: Зеркало ``rob_box_teleop.proximity.Thresholds`` (пакет сима едет на песочницу без teleop; равенство держит тест
#: tests/unit/sim/test_sim_coverage_perimeter.py): стоп сбоку, запас коридора хода, «приближается», мин. точек.
GUARD_SIDE_STOP_M = 0.08
GUARD_CORRIDOR_MARGIN_M = 0.05
GUARD_CLOSING_MPS = 0.02
GUARD_MIN_POINTS = 2
#: Доля склона, который пускает защита, — запас на ошибку слежения DWB (догадка до замера в HIL).
RAMP_GUARD_SHARE = 0.5
#: Нырок нужен, если накрывает столько пола, не накрытого змейкой, м².
PERIM_MIN_GAIN_M2 = 0.5
#: Защита и корпус проверяются по несвободным клеткам не дальше этого от позы, м.
CHECK_RADIUS_M = 1.0


def default_wall_offset(footprint: Footprint = ROBOT_FOOTPRINT) -> float:
    """Центр от стены при проходе вдоль неё: половина ширины + стоп защиты сбоку + padding = 0.4125 м."""
    return inscribed_half_width(footprint) + GUARD_SIDE_STOP_M + NAV2_PADDING_M


def guard_band(footprint: Footprint = ROBOT_FOOTPRINT) -> float:
    """Полуширина коридора хода защиты: борт + запас коридора, 0.3525 м."""
    return max(footprint.y_max, -footprint.y_min) + GUARD_CORRIDOR_MARGIN_M


def guard_reach(footprint: Footprint = ROBOT_FOOTPRINT) -> float:
    """Нос + стоп защиты: дальше этого впереди препятствие в коридоре хода не тормозит, 0.5064 м."""
    return footprint.x_max + GUARD_STOP_M


# ── Геометрия: пространство прохода и профиль нырка ──────────────────────────


def erode_rect(free: np.ndarray, half_rows_m: float, half_cols_m: float, resolution: float) -> np.ndarray:
    """Клетки, где прямоугольник ±``half_rows_m`` по строкам и ±``half_cols_m`` по столбцам не задевает несвободного
    (и края сетки). От центра до квадрата ближайшей несвободной клетки — не меньше полусторон (округление вверх)."""
    kr = max(0, int(math.ceil(half_rows_m / resolution - 0.5 - 1e-9)))
    kc = max(0, int(math.ceil(half_cols_m / resolution - 0.5 - 1e-9)))
    blocked = np.pad(~free, ((kr, kr), (kc, kc)), constant_values=True)
    acc = _row_dilate(blocked, kc) if kc else blocked
    acc = _row_dilate(acc.T, kr).T if kr else acc
    return ~acc[kr : kr + free.shape[0], kc : kc + free.shape[1]]


def dip_depth(strip: np.ndarray, lane: tuple[int, int, int], side: int, max_rows: int) -> np.ndarray:
    """По столбцам полосы ``lane`` = (строка, j0, j1): на сколько строк к стене (``side`` ±1) можно сместиться, не выйдя
    из ``strip``, — только если за этим край ``strip`` (стена близко). Нет края в ``max_rows`` строках — 0: нырять
    не к чему."""
    i, j0, j1 = lane
    depth = np.zeros(j1 - j0 + 1, dtype=np.int32)
    alive = np.ones_like(depth, dtype=bool)
    for m in range(1, max_rows + 2):
        r = i + side * m
        inside = strip[r, j0 : j1 + 1] if 0 <= r < strip.shape[0] else np.zeros_like(alive)
        edge = alive & ~inside
        depth[edge] = m - 1
        alive &= inside
        if not alive.any():
            break
    return depth


def dip_envelope(depth: np.ndarray, reach_cells: int) -> np.ndarray:
    """Минимум глубины на ±``reach_cells`` столбцов: препятствие впереди и сзади не входит в коридор хода."""
    n = len(depth)
    pad = np.pad(depth, reach_cells, mode="edge")
    win = np.lib.stride_tricks.sliding_window_view(pad, 2 * reach_cells + 1)
    return win.min(axis=1)[:n]


@dataclass(frozen=True)
class RampRule:
    """Склон нырка: сколько строк на столбец можно сместиться к стене при зазоре ``clear`` (м) до стены."""

    resolution: float
    wall_offset_m: float
    band_m: float
    reach_m: float
    share: float = RAMP_GUARD_SHARE

    def step(self, envelope_rows: float, dip_rows: float) -> float:
        clear = self.wall_offset_m + (envelope_rows - dip_rows) * self.resolution
        return max(0.0, self.share * (clear - self.band_m) / self.reach_m)


def dip_profile(envelope: np.ndarray, rule: RampRule, max_iter: int = 8) -> np.ndarray:
    """Глубина нырка по столбцам (дробная, в строках): ≤ ``envelope``, концы 0, склон ≤ ``rule.step`` в обе стороны."""
    m = envelope.astype(float).copy()
    if len(m) < 3:
        return np.zeros_like(m)
    m[0] = m[-1] = 0.0
    for _ in range(max_iter):
        before = m.copy()
        for j in range(1, len(m)):
            m[j] = min(m[j], m[j - 1] + rule.step(envelope[j], m[j - 1]))
        for j in range(len(m) - 2, -1, -1):
            m[j] = min(m[j], m[j + 1] + rule.step(envelope[j], m[j + 1]))
        if np.array_equal(before, m):
            break
    return m


# ── Проверки поз: корпус и защита ────────────────────────────────────────────


def _edge_cells(free: np.ndarray) -> np.ndarray:
    """Несвободные клетки, у которых есть свободный сосед по стороне: только их видит корпус и лидар."""
    occ = ~free
    pad = np.pad(free, 1, constant_values=False)
    near = pad[:-2, 1:-1] | pad[2:, 1:-1] | pad[1:-1, :-2] | pad[1:-1, 2:]
    return occ & near


def _local(points: np.ndarray, poses: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Точки [N, 2] в системе каждой позы [K, 3] → (u, v) формы [K, N]."""
    dx = points[None, :, 0] - poses[:, None, 0]
    dy = points[None, :, 1] - poses[:, None, 1]
    c, s = np.cos(poses[:, 2])[:, None], np.sin(poses[:, 2])[:, None]
    return c * dx + s * dy, -s * dx + c * dy


def _near(points: np.ndarray, poses: np.ndarray, radius: float) -> np.ndarray:
    lo, hi = poses[:, :2].min(axis=0) - radius, poses[:, :2].max(axis=0) + radius
    keep = (points[:, 0] >= lo[0]) & (points[:, 0] <= hi[0]) & (points[:, 1] >= lo[1]) & (points[:, 1] <= hi[1])
    return points[keep]


@dataclass
class Obstacles:
    """Края несвободного карты: центры клеток (корпус) и углы клеток (защита — по поверхности, как лидар)."""

    centers: np.ndarray
    corners: np.ndarray

    @classmethod
    def from_space(cls, space: CoverageSpace) -> "Obstacles":
        rr, cc = np.nonzero(_edge_cells(space.free))
        res = space.resolution
        x0, y0 = space.origin_x + cc * res, space.origin_y + rr * res
        centers = np.column_stack((x0 + res / 2, y0 + res / 2))
        corners = np.concatenate([np.column_stack((x0 + a * res, y0 + b * res)) for a in (0, 1) for b in (0, 1)])
        return cls(centers, np.unique(corners, axis=0))


def footprint_hits(
    obs: Obstacles, poses: np.ndarray, footprint: Footprint = ROBOT_FOOTPRINT, padding: float = NAV2_PADDING_M
) -> np.ndarray:
    """Для каждой позы [K, 3]: задевает ли корпус с ``padding`` центр несвободной клетки."""
    pts = _near(obs.centers, poses, CHECK_RADIUS_M)
    if not len(pts):
        return np.zeros(len(poses), dtype=bool)
    u, v = _local(pts, poses)
    inside = (u >= footprint.x_min - padding) & (u <= footprint.x_max + padding)
    inside &= (v >= footprint.y_min - padding) & (v <= footprint.y_max + padding)
    return inside.any(axis=1)


def guard_stops(
    obs: Obstacles, poses: np.ndarray, speed_mps: float = SPEED_MPS, footprint: Footprint = ROBOT_FOOTPRINT
) -> np.ndarray:
    """Для каждой позы [K, 3] при ходе вперёд ``speed_mps`` (``wz = 0``): дала бы защита стоп.

    Правило ``rob_box_teleop.proximity``: точка — угроза, если корпус к ней приближается быстрее ``GUARD_CLOSING_MPS``
    и она ближе своего порога (``stop_limits``: впереди в коридоре хода — ``GUARD_STOP_M``, прочие —
    ``GUARD_SIDE_STOP_M``);
    стоп — если таких точек ≥ ``GUARD_MIN_POINTS``. Точки — углы несвободных клеток: поверхность препятствия на карте.
    """
    pts = _near(obs.corners, poses, CHECK_RADIUS_M)
    if not len(pts):
        return np.zeros(len(poses), dtype=bool)
    u, v = _local(pts, poses)
    qu, qv = np.clip(u, footprint.x_min, footprint.x_max), np.clip(v, footprint.y_min, footprint.y_max)
    du, dv = u - qu, v - qv
    gap = np.hypot(du, dv)
    with np.errstate(divide="ignore", invalid="ignore"):
        closing = np.where(gap > 0, speed_mps * du / gap, 0.0)
    band = (v >= footprint.y_min - GUARD_CORRIDOR_MARGIN_M) & (v <= footprint.y_max + GUARD_CORRIDOR_MARGIN_M)
    limit = np.where(band & (u > footprint.x_max), GUARD_STOP_M, GUARD_SIDE_STOP_M)
    threat = (gap > 0) & (closing > GUARD_CLOSING_MPS) & (gap < limit)
    return threat.sum(axis=1) >= GUARD_MIN_POINTS


def _chunked(fn, obs: Obstacles, poses: np.ndarray, chunk: int = 64) -> np.ndarray:
    if not len(poses):
        return np.zeros(0, dtype=bool)
    return np.concatenate([fn(obs, poses[k : k + chunk]) for k in range(0, len(poses), chunk)])


# ── План ─────────────────────────────────────────────────────────────────────


@dataclass(frozen=True)
class PerimeterStrip:
    """Нырок одной полосы: путь от места разворота до места разворота (x, y, yaw рад), годные направления."""

    axis: str
    side: int  # +1 — к стене в сторону роста строк кадра, −1 — в другую
    path: tuple[tuple[float, float, float], ...]
    gain_m2: float  # оценка: пол, не накрытый змейкой, под нырком (квадрат корпуса)
    dip_max_m: float
    forward_ok: bool
    backward_ok: bool

    @property
    def length_m(self) -> float:
        return sum(math.dist(a[:2], b[:2]) for a, b in zip(self.path, self.path[1:]))

    def driven(self, forward: bool) -> list[tuple[float, float, float]]:
        """Путь в направлении проезда: назад — те же точки в обратном порядке, курс + π."""
        if forward:
            return list(self.path)
        return [(x, y, math.atan2(-math.sin(t), -math.cos(t))) for x, y, t in reversed(self.path)]


@dataclass
class PerimeterPlan:
    wall_offset_m: float
    strips: list[PerimeterStrip] = field(default_factory=list)
    order: list[tuple[int, bool]] = field(default_factory=list)  # (номер нырка, вперёд ли) по порядку проезда
    candidates: int = 0  # полос × сторон с ненулевым нырком
    dropped_small: int = 0  # нырок накрывает < PERIM_MIN_GAIN_M2 непокрытого
    dropped_footprint: int = 0  # корпус задевает несвободное хоть в одной позе в обоих направлениях
    dropped_guard: int = 0  # защита дала бы стоп хоть в одной позе в обоих направлениях
    route_m: float = 0.0  # переезды по прямой от конца змейки + пути нырков

    def driven(self) -> list[tuple[int, list[tuple[float, float, float]]]]:
        return [(k, self.strips[k].driven(fwd)) for k, fwd in self.order]

    def time_estimate_s(self, speed_mps: float = SPEED_MPS, per_goal_s: float = PER_POSE_S) -> float:
        """Путь / скорость + накладные на две цели нырка (подъезд к началу и сам путь)."""
        return self.route_m / speed_mps + 2 * per_goal_s * len(self.order)


def _frame_xy(frame: _Frame, row: np.ndarray, cols: np.ndarray) -> np.ndarray:
    """Дробная строка и столбцы кадра → (x, y) мира [K, 2]."""
    sp = frame.space
    a, b = (row, cols) if frame.axis == "x" else (cols, row)
    return np.column_stack((sp.origin_x + (b + 0.5) * sp.resolution, sp.origin_y + (a + 0.5) * sp.resolution))


def _path_poses(xy: np.ndarray) -> np.ndarray:
    """Курс каждой точки — по соседям (центральная разность)."""
    d = np.gradient(xy, axis=0)
    return np.column_stack((xy, np.arctan2(d[:, 1], d[:, 0])))


def dip_spans(profile: np.ndarray, turn_row: np.ndarray) -> list[tuple[int, int]]:
    """Куски строки под нырки: каждый кусок ``profile > 0`` продлевается до ближайших мест разворота на базовой строке
    (``turn_row``) слева и справа; пересёкшиеся куски сливаются. Кусок без места разворота с какой-то стороны
    пропускается: заехать или выехать можно только задом."""
    t = np.flatnonzero(turn_row)
    spans: list[tuple[int, int]] = []
    for a, b in _runs(profile > 0):
        left, right = t[t < a], t[t > b]
        if not (len(left) and len(right)):
            continue
        lo, hi = int(left[-1]), int(right[0])
        if spans and lo <= spans[-1][1]:
            lo = spans.pop()[0]
        spans.append((lo, hi))
    return spans


def _runs(mask: np.ndarray) -> list[tuple[int, int]]:
    d = np.diff(np.concatenate(([0], mask.astype(np.int8), [0])))
    return list(zip(np.flatnonzero(d == 1).tolist(), (np.flatnonzero(d == -1) - 1).tolist()))


@dataclass
class _Ctx:
    space: CoverageSpace
    obs: Obstacles
    need: np.ndarray  # непокрытый змейкой покрываемый пол (оси мира), убывает по мере выбора нырков
    wall_offset_m: float
    min_gain_m2: float
    hw: int  # полусторона квадрата корпуса в клетках


def _dip_candidates(ctx: _Ctx, frame: _Frame) -> list[tuple[_Frame, int, np.ndarray]]:
    """(кадр, сторона, точки пути [K, 2]) нырков с каждой строки пространства центров к близкой стене.

    Базовая строка — любой отрезок достижимых центров; нырок от неё к стене не глубже ``turn_clearance`` (дальше стена
    не «близко»), концы — в местах разворота на этой строке (``dip_spans``). Соседние базовые строки дают почти
    одинаковые нырки — лишние отсеет жадный отбор по выигрышу."""
    sp = ctx.space
    strip = erode_rect(frame.array(sp.free), ctx.wall_offset_m, sp.erosion_m, sp.resolution)
    strip &= frame.array(sp.reachable)
    base, turn = frame.array(sp.reachable), _turn(sp, frame)
    rule = RampRule(sp.resolution, ctx.wall_offset_m, guard_band(), guard_reach())
    reach_cells = int(math.ceil((guard_reach() + NAV2_PADDING_M) / sp.resolution))
    max_rows = int(math.ceil(sp.turn_clearance_m / sp.resolution))
    out = []
    for i in range(base.shape[0]):
        for j0, j1 in _runs(base[i]):
            for side in (1, -1):
                depth = dip_depth(strip, (i, j0, j1), side, max_rows)
                t = np.flatnonzero(turn[i, j0 : j1 + 1])
                if not depth.any() or len(t) < 2:
                    continue
                env = dip_envelope(depth, reach_cells)
                env[: t[0] + 1] = 0  # заезд и выезд — только между местами разворота
                env[t[-1] :] = 0
                m = dip_profile(env, rule)
                for a, b in dip_spans(m, turn[i, j0 : j1 + 1]):
                    cols = np.arange(j0 + a, j0 + b + 1, dtype=float)
                    out.append((frame, side, _frame_xy(frame, i + side * m[a : b + 1], cols)))
    return out


@dataclass(frozen=True)
class _Swept:
    """Оценка накрытого нырком: квадрат ±``hw`` клеток вокруг каждой точки пути — окно сетки и маска в нём."""

    r0: int
    c0: int
    mask: np.ndarray

    def window(self, grid: np.ndarray) -> np.ndarray:
        return grid[self.r0 : self.r0 + self.mask.shape[0], self.c0 : self.c0 + self.mask.shape[1]]


def _swept_world(ctx: _Ctx, xy: np.ndarray) -> _Swept:
    sp, hw = ctx.space, ctx.hw
    h, w = sp.free.shape
    cc = np.clip(np.floor((xy[:, 0] - sp.origin_x) / sp.resolution).astype(int), 0, w - 1)
    rr = np.clip(np.floor((xy[:, 1] - sp.origin_y) / sp.resolution).astype(int), 0, h - 1)
    r0, c0 = max(int(rr.min()) - hw, 0), max(int(cc.min()) - hw, 0)
    m = np.zeros((min(int(rr.max()) + hw + 1, h) - r0, min(int(cc.max()) + hw + 1, w) - c0), dtype=bool)
    for r, c in zip(rr - r0, cc - c0):
        m[max(r - hw, 0) : r + hw + 1, max(c - hw, 0) : c + hw + 1] = True
    return _Swept(r0, c0, m)


def _checked(ctx: _Ctx, frame: _Frame, side: int, xy: np.ndarray, gain_m2: float) -> PerimeterStrip | str:
    """Нырок с проверкой поз в обоих направлениях; не годен — причина ``footprint`` | ``guard``."""
    fwd = _path_poses(xy)
    back = _path_poses(xy[::-1])
    if _chunked(footprint_hits, ctx.obs, fwd).any():
        return "footprint"
    ok_f, ok_b = not _chunked(guard_stops, ctx.obs, fwd).any(), not _chunked(guard_stops, ctx.obs, back).any()
    if not (ok_f or ok_b):
        return "guard"
    d, rel = xy[-1] - xy[0], xy - xy[0]
    dip = float(np.max(np.abs(d[0] * rel[:, 1] - d[1] * rel[:, 0])) / max(float(np.hypot(*d)), 1e-9))
    path = tuple((float(x), float(y), float(t)) for x, y, t in fwd)
    return PerimeterStrip(frame.axis, side, path, gain_m2, dip, ok_f, ok_b)


def _select(plan: PerimeterPlan, ctx: _Ctx, cands: list) -> None:
    """Жадно по выигрышу: накрытое выбранным нырком другим уже не засчитывается."""
    swept = [_swept_world(ctx, xy) for _, _, xy in cands]
    min_cells = ctx.min_gain_m2 / ctx.space.resolution**2
    # Ленивый жадный отбор: выигрыш только убывает, поэтому пересчитывается лишь вершина кучи.
    heap = [(-_gain_cells(ctx, s), len(cands[k][2]), k) for k, s in enumerate(swept)]
    heapq.heapify(heap)
    while heap:
        neg, length, k = heapq.heappop(heap)
        gain = _gain_cells(ctx, swept[k])
        if gain < -neg:
            heapq.heappush(heap, (-gain, length, k))
            continue
        if gain < min_cells:
            plan.dropped_small += 1 + len(heap)
            return
        frame, side, xy = cands[k]
        got = _checked(ctx, frame, side, xy, gain * ctx.space.resolution**2)
        if isinstance(got, str):
            plan.dropped_footprint += got == "footprint"
            plan.dropped_guard += got == "guard"
            continue
        plan.strips.append(got)
        swept[k].window(ctx.need)[swept[k].mask] = False


def _gain_cells(ctx: _Ctx, s: _Swept) -> int:
    return int((s.window(ctx.need) & s.mask).sum())


def _order(plan: PerimeterPlan, here: tuple[float, float]) -> None:
    """Жадно: следующий нырок — с ближайшим по прямой концом, в годном направлении."""
    todo = set(range(len(plan.strips)))
    while todo:
        best = None
        for k in todo:
            s = plan.strips[k]
            for fwd, ok in ((True, s.forward_ok), (False, s.backward_ok)):
                if ok:
                    start = s.path[0] if fwd else s.path[-1]
                    cand = (math.dist(here, start[:2]), k, fwd)
                    best = cand if best is None or cand < best else best
        d, k, fwd = best
        todo.remove(k)
        plan.order.append((k, fwd))
        plan.route_m += d + plan.strips[k].length_m
        end = plan.strips[k].driven(fwd)[-1]
        here = (end[0], end[1])


def plan_perimeter(
    space: CoverageSpace,
    snake: CoveragePlan,
    *,
    wall_offset_m: float | None = None,
    min_gain_m2: float = PERIM_MIN_GAIN_M2,
) -> PerimeterPlan:
    """Нырки к стенам после змейки ``snake`` (см. шапку модуля). Непокрытое — по треку плана змейки (``plan_track``),
    той же меркой, что метрика."""
    offset = default_wall_offset() if wall_offset_m is None else wall_offset_m
    plan = PerimeterPlan(offset)
    if space.start_cell is None:
        return plan
    need = space.coverable & ~covered_mask(space, plan_track(snake.poses))
    hw = int(math.floor(space.cover_half_m / space.resolution + 1e-9))
    ctx = _Ctx(space, Obstacles.from_space(space), need, offset, min_gain_m2, hw)
    cands = []
    for axis in ("x", "y"):
        cands += _dip_candidates(ctx, _Frame(space, axis))
    plan.candidates = len(cands)
    _select(plan, ctx, cands)
    end = snake.poses[-1] if snake.poses else None
    _order(plan, (end.x, end.y) if end else space.cell_xy(*space.start_cell))
    return plan


def perimeter_track(plan: PerimeterPlan, speed_mps: float = SPEED_MPS) -> list[tuple[float, float, float, float]]:
    """Трек «как по плану» по путям нырков (переезды между ними не считаются), формат ``coverage_metric``."""
    out: list[tuple[float, float, float, float]] = []
    t = 0.0
    for _, path in plan.driven():
        for a, b in zip(path, path[1:]):
            out.append((t, *a))
            t += math.dist(a[:2], b[:2]) / speed_mps
        out.append((t, *path[-1]))
        t += 1e3
    return out


def perimeter_summary(plan: PerimeterPlan, space: CoverageSpace, snake: CoveragePlan) -> str:
    """Строка ``PERIMETER_PLAN``: числа из плана и метрики, не из модели."""
    base = covered_mask(space, plan_track(snake.poses))
    both = base | covered_mask(space, perimeter_track(plan))
    before, after = coverage_result(space, base), coverage_result(space, both)
    dips = [s.dip_max_m for s in plan.strips]
    return (
        f"PERIMETER_PLAN wall_offset={plan.wall_offset_m:.4f} strips={len(plan.strips)} candidates={plan.candidates} "
        f"dropped_small={plan.dropped_small} dropped_footprint={plan.dropped_footprint} "
        f"dropped_guard={plan.dropped_guard} one_way={sum(not (s.forward_ok and s.backward_ok) for s in plan.strips)} "
        f"dip_max_m={max(dips, default=0.0):.2f} path_m={sum(s.length_m for s in plan.strips):.1f} "
        f"route_m={plan.route_m:.1f} time_est_s={plan.time_estimate_s():.0f}@{SPEED_MPS}m/s+2x{PER_POSE_S:.0f}s/strip "
        f"{_delta(before, after)}"
    )


def _delta(before: CoverageResult, after: CoverageResult) -> str:
    return (
        f"gain_m2={after.covered_m2 - before.covered_m2:.1f} "
        f"covered_m2={before.covered_m2:.1f}->{after.covered_m2:.1f} "
        f"coverage={before.pct}%->{after.pct}% coverage_free={before.free_pct}%->{after.free_pct}%"
    )


def reverse_m(track: Sequence[tuple[float, float, float, float]], eps_m: float = 0.005) -> float:
    """Сколько метров трека (t, x, y, yaw рад) пройдено задом: сдвиг между соседними точками против курса.

    План нырков заднего хода не заказывает, но DWB может дать ``min_vel_x`` < 0 — это видно здесь, а не скрыто."""
    out = 0.0
    for a, b in zip(track, track[1:]):
        along = (b[1] - a[1]) * math.cos(a[3]) + (b[2] - a[2]) * math.sin(a[3])
        out += -along if along < -eps_m else 0.0
    return out


def perimeter_done(
    outcomes: Sequence[GoalOutcome], before: CoverageResult, after: CoverageResult, back_m: float = 0.0
) -> str:
    """Строка ``PERIMETER_DONE`` прогона: исходы нырков (цели ``P####a`` — подъезд, ``P####f`` — путь), путь задом по
    треку (``reverse_m``) и покрытие по треку истинной позы до периметра и после. Числа — из исходов и метрики."""
    approach = [o for o in outcomes if o.name.endswith("a")]
    follow = [o for o in outcomes if o.name.endswith("f")]
    bad = Counter(o.result for o in outcomes if not o.succeeded and o.result != "NOT_SENT")
    return (
        f"PERIMETER_DONE strips={len(follow)} approach_ok={sum(o.succeeded for o in approach)} "
        f"path_ok={sum(o.succeeded for o in follow)} not_sent={sum(o.result == 'NOT_SENT' for o in follow)} "
        f"failed=({', '.join(f'{k}={v}' for k, v in sorted(bad.items())) or '-'}) "
        f"em_episodes={sum(o.em_episodes for o in outcomes)} reverse_m={back_m:.2f} {_delta(before, after)}"
    )


def strips_to_json(plan: PerimeterPlan) -> list[dict]:
    return [
        {"strip": k, "axis": plan.strips[k].axis, "side": plan.strips[k].side, "forward": fwd, "path": path}
        for k, (path, fwd) in ((k, (plan.strips[k].driven(f), f)) for k, f in plan.order)
    ]


def main(argv: Sequence[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Проход по периметру после змейки по снимку карты (npz map_recorder)")
    ap.add_argument("map", type=Path)
    ap.add_argument("--start", type=float, nargs=2, default=(0.0, 0.0), metavar=("X", "Y"))
    ap.add_argument("--wall-offset", type=float, default=None, help="центр от стены на проходе, м (умолч. 0.4125)")
    ap.add_argument("--min-gain", type=float, default=PERIM_MIN_GAIN_M2, help="нырок нужен от стольких м²")
    args = ap.parse_args(argv)
    space = coverage_space(load_grid_npz(args.map), tuple(args.start))
    snake = plan_coverage(space)
    print(plan_summary(snake, space))
    plan = plan_perimeter(space, snake, wall_offset_m=args.wall_offset, min_gain_m2=args.min_gain)
    print(perimeter_summary(plan, space, snake))
    return 0


if __name__ == "__main__":
    sys.exit(main())
