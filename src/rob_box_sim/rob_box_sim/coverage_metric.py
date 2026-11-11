"""Метрика обхода покрытием (#4560): какая доля пола прошла под корпусом. Без ROS, на numpy.

Траектория — (t, x, y, yaw[рад]): истинная поза Gazebo (``/sim/ground_truth/odom``), строки
``TRACK`` лога ``scripts/hil/video/route.py`` / ``coverage_run.py`` или ``maprec/poses.csv``.
Знаменатель — «покрываемый» пол ``coverage_plan.CoverageSpace.coverable``: свободные клетки,
которые корпус накрывает из достижимого пространства центров. Числитель — те из них, чей
центр хоть раз оказался под прямоугольником корпуса (``geometry2d.Footprint``).

Между соседними образцами поза интерполируется с шагом ``step_m``; разрыв длиннее
``max_gap_m`` не заполняется (пропуск в логе не выдаётся за проезд).

Итоговую строку прогона (``run_summary``) строит код по исходам целей.
"""

from __future__ import annotations

import argparse
import json
import math
import re
import sys
from collections import Counter
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Sequence

import numpy as np
from rob_box_sim.coverage_plan import CoverageSpace, coverage_space, load_grid_npz
from rob_box_sim.geometry2d import ROBOT_FOOTPRINT, Footprint

TrackPoint = tuple[float, float, float, float]  # t, x, y, yaw (рад)

_TRACK_RE = re.compile(r"\bTRACK sim=(\S+) x=(\S+) y=(\S+) yaw=(\S+)")


def parse_track_log(lines: Iterable[str]) -> list[TrackPoint]:
    """Строки ``TRACK sim=… x=… y=… yaw=<град>`` → (t, x, y, yaw рад); прочие строки пропускаются."""
    out = []
    for line in lines:
        m = _TRACK_RE.search(line)
        if m:
            t, x, y, yaw = (float(v) for v in m.groups())
            out.append((t, x, y, math.radians(yaw)))
    return out


def densify(
    track: Sequence[TrackPoint], step_m: float = 0.05, max_gap_m: float = 1.0, max_dt_s: float = 5.0
) -> list[TrackPoint]:
    """Позы через ≤ ``step_m`` (и ≤ 5° по курсу) между образцами.

    Соседей дальше ``max_gap_m`` или реже ``max_dt_s`` не связываем: это пропуск в логе, а не проезд.
    """
    if not track:
        return []
    out = [track[0]]
    for a, b in zip(track, track[1:]):
        d = math.hypot(b[1] - a[1], b[2] - a[2])
        dyaw = math.atan2(math.sin(b[3] - a[3]), math.cos(b[3] - a[3]))
        if d <= max_gap_m and b[0] - a[0] <= max_dt_s:
            n = max(1, int(math.ceil(max(d / step_m, abs(dyaw) / math.radians(5)))))
            for k in range(1, n):
                u = k / n
                out.append(
                    (a[0] + u * (b[0] - a[0]), a[1] + u * (b[1] - a[1]), a[2] + u * (b[2] - a[2]), a[3] + u * dyaw)
                )
        out.append(b)
    return out


def covered_mask(
    space: CoverageSpace,
    track: Sequence[TrackPoint],
    *,
    footprint: Footprint = ROBOT_FOOTPRINT,
    step_m: float = 0.05,
    max_gap_m: float = 1.0,
    chunk: int = 2048,
) -> np.ndarray:
    """Клетки, чей центр хоть раз был под корпусом. Позы штампуются пачками (numpy), а не по одной."""
    covered = np.zeros_like(space.free)
    pts = np.asarray([p[1:] for p in densify(track, step_m, max_gap_m)], dtype=float).reshape(-1, 3)
    res, h, w = space.resolution, covered.shape[0], covered.shape[1]
    reach = math.hypot(max(footprint.x_max, -footprint.x_min), max(footprint.y_max, -footprint.y_min))
    k = int(math.ceil(reach / res)) + 1
    off = np.arange(-k, k + 1)
    for s0 in range(0, len(pts), chunk):
        x, y, yaw = (pts[s0 : s0 + chunk, i][:, None, None] for i in range(3))
        c = np.floor((x - space.origin_x) / res).astype(int) + off[None, None, :]
        r = np.floor((y - space.origin_y) / res).astype(int) + off[None, :, None]
        dx = space.origin_x + (c + 0.5) * res - x
        dy = space.origin_y + (r + 0.5) * res - y
        u = np.cos(yaw) * dx + np.sin(yaw) * dy
        v = -np.sin(yaw) * dx + np.cos(yaw) * dy
        hit = (u >= footprint.x_min) & (u <= footprint.x_max) & (v >= footprint.y_min) & (v <= footprint.y_max)
        hit &= (r >= 0) & (r < h) & (c >= 0) & (c < w)
        rr, cc = np.broadcast_arrays(r, c)
        covered[rr[hit], cc[hit]] = True
    return covered


@dataclass(frozen=True)
class CoverageResult:
    coverable_m2: float
    covered_m2: float  # покрываемого пола, прошедшего под корпусом
    free_m2: float
    free_covered_m2: float  # любого свободного (включая недоступное плану)

    @property
    def pct(self) -> float | None:
        """Доля «покрываемого» (то, до чего корпус дотягивается из достижимых центров) под корпусом, %."""
        return round(100.0 * self.covered_m2 / self.coverable_m2, 2) if self.coverable_m2 > 0 else None

    @property
    def free_pct(self) -> float | None:
        """Доля всего свободного пола карты под корпусом, %: сюда входит и то, куда корпус не пролезает."""
        return round(100.0 * self.free_covered_m2 / self.free_m2, 2) if self.free_m2 > 0 else None


def coverage_result(space: CoverageSpace, covered: np.ndarray) -> CoverageResult:
    return CoverageResult(
        space.area(space.coverable),
        space.area(space.coverable & covered),
        space.area(space.free),
        space.area(space.free & covered),
    )


def plan_track(poses: Sequence, speed_mps: float = 0.3, step_m: float = 0.05) -> list[TrackPoint]:
    """Трек «как по плану»: только отрезки внутри полос (переезды между полосами не считаем).

    Отрезки разделены паузой больше ``densify(max_dt_s)``, чтобы соседние полосы не связывались.
    """
    out: list[TrackPoint] = []
    t = 0.0
    for a, b in zip(poses, poses[1:]):
        if (a.cell, a.lane) != (b.cell, b.lane):
            continue
        n = max(1, int(math.ceil(math.hypot(b.x - a.x, b.y - a.y) / step_m)))
        for k in range(n + 1):
            u = k / n
            out.append((t + u * step_m * n / speed_mps, a.x + u * (b.x - a.x), a.y + u * (b.y - a.y), a.yaw))
        t = out[-1][0] + 1e3
    return out


# ── Итог прогона ─────────────────────────────────────────────────────────────


@dataclass(frozen=True)
class GoalOutcome:
    """Исход одной цели: позы по одной (NavigateToPose) или полоса (NavigateThroughPoses, ``poses`` > 1)."""

    name: str
    result: str  # SUCCEEDED | ABORTED | CANCELED | TIMEOUT | REJECTED | INTERRUPTED | NOT_SENT
    em_episodes: int = 0
    poses: int = 1  # сколько поз плана в цели
    passed: int | None = None  # из них пройдено; не задано — успех: все, иначе 0

    def __post_init__(self) -> None:
        if self.passed is None:
            object.__setattr__(self, "passed", self.poses if self.succeeded else 0)

    @classmethod
    def from_route(
        cls, name: str, raw: object, em_episodes: int = 0, poses: int = 1, remaining: int | None = None
    ) -> "GoalOutcome":
        """Исход ``route.run_goal`` / ``run_lane`` → короткое имя: ``STATUS_SUCCEEDED`` → ``SUCCEEDED``.

        Одно место правды: HIL Depot 09.11 (#4560) — 20 успехов подряд засчитались неудачами, потому что
        ``run_goal`` отдаёт ``STATUS_*``, а сравнение было с ``SUCCEEDED``. Лог ``GOAL_END`` не меняется.
        Пройдено поз: успех — все; иначе ``poses − remaining`` (последний feedback Nav2), без feedback — 0.
        """
        s = str(raw)
        result = s[len("STATUS_") :] if s.startswith("STATUS_") else s
        passed = None if remaining is None else min(poses, max(0, poses - remaining))
        out = cls(name, result, em_episodes, poses, passed)
        return cls(name, result, em_episodes, poses) if out.succeeded else out

    @classmethod
    def not_sent(cls, name: str, poses: int = 1) -> "GoalOutcome":
        return cls(name, "NOT_SENT", 0, poses, 0)

    @property
    def succeeded(self) -> bool:
        return self.result == "SUCCEEDED"


# ── Движение по треку ────────────────────────────────────────────────────────

#: «В движении» между соседними точками трека: быстрее по ходу или по курсу (пороги «в движении» защиты,
#: rob_box_teleop.proximity.Thresholds moving_v / moving_w). Разрыв трека длиннее — не считается.
MOVING_MPS = 0.03
MOVING_RADPS = 0.10
MOVING_MAX_DT_S = 5.0


@dataclass(frozen=True)
class MotionStats:
    moving_s: float  # время в движении по треку, с sim
    moved_m: float  # путь в движении, м

    def avg_mps(self, sim_s: float) -> float | None:
        """Средняя скорость за весь обход: путь / время обхода (с остановками и таймаутами)."""
        return self.moved_m / sim_s if sim_s > 0 else None

    def moving_mps(self) -> float | None:
        """Средняя скорость в движении."""
        return self.moved_m / self.moving_s if self.moving_s > 0 else None


def motion_stats(track: Sequence[TrackPoint]) -> MotionStats:
    """Время в движении и путь по треку истинной позы (t, x, y, yaw рад)."""
    moving_s = moved_m = 0.0
    for a, b in zip(track, track[1:]):
        dt = b[0] - a[0]
        if dt <= 0 or dt > MOVING_MAX_DT_S:
            continue
        ds = math.hypot(b[1] - a[1], b[2] - a[2])
        dyaw = abs(math.atan2(math.sin(b[3] - a[3]), math.cos(b[3] - a[3])))
        if ds / dt >= MOVING_MPS or dyaw / dt >= MOVING_RADPS:
            moving_s += dt
            moved_m += ds
    return MotionStats(moving_s, moved_m)


def _pct(v: float | None) -> str:
    return "n/a" if v is None else f"{v:.1f}%"


def _num(v: float | None, fmt: str) -> str:
    return "n/a" if v is None else format(v, fmt)


def _motion(motion: MotionStats | None, sim_s: float) -> str:
    if motion is None:
        return ""
    share = 100.0 * motion.moving_s / sim_s if sim_s > 0 else None
    return (
        f" moving_s={motion.moving_s:.1f} moving_pct={_pct(share)} moved_m={motion.moved_m:.1f} "
        f"avg_mps={_num(motion.avg_mps(sim_s), '.3f')} moving_mps={_num(motion.moving_mps(), '.3f')}"
    )


def run_summary(
    outcomes: Sequence[GoalOutcome],
    cov: CoverageResult,
    sim_s: float,
    stop_reason: str = "",
    motion: MotionStats | None = None,
) -> str:
    """Итоговая строка обхода: числа только из исходов, метрики и трека.

    ``goals`` — целей (поз или полос), ``poses``/``poses_passed`` — поз плана в них и пройдено.
    ``coverage`` — от покрываемого (``coverable_m2``), ``coverage_free`` — от всего свободного пола
    (``free_m2``). ``interrupted`` — обход прерван сигналом (``stop=interrupted_SIGINT``), итог по пройденному.
    ``moving_*`` / ``avg_mps`` — время в движении и средние скорости по треку истинной позы.
    """
    by = Counter(o.result for o in outcomes)
    ok = sum(o.succeeded for o in outcomes)
    by.pop("SUCCEEDED", None)
    not_sent = by.pop("NOT_SENT", 0)
    failed = ", ".join(f"{k}={v}" for k, v in sorted(by.items())) or "-"
    stop = stop_reason or "end_of_plan"
    return (
        f"COVERAGE_DONE goals={len(outcomes)} succeeded={ok} skipped={sum(by.values())} ({failed}) "
        f"not_sent={not_sent} poses={sum(o.poses for o in outcomes)} poses_passed={sum(o.passed for o in outcomes)} "
        f"stop={stop} interrupted={'yes' if stop.startswith('interrupted') else 'no'} "
        f"em_episodes={sum(o.em_episodes for o in outcomes)} sim_s={sim_s:.1f}{_motion(motion, sim_s)} "
        f"coverage={_pct(cov.pct)} covered_m2={cov.covered_m2:.1f} coverable_m2={cov.coverable_m2:.1f} "
        f"coverage_free={_pct(cov.free_pct)} free_covered_m2={cov.free_covered_m2:.1f} free_m2={cov.free_m2:.1f}"
    )


# ── Картинка ─────────────────────────────────────────────────────────────────

COLORS = {
    "unknown": (190, 190, 190),
    "occupied": (30, 30, 30),
    "free_lost": (240, 170, 60),  # свободно, но корпусу недоступно (узко / за узким местом / запас padding)
    "missed": (220, 50, 50),  # покрываемое, но не покрыто
    "covered": (60, 170, 80),
}


def coverage_rgb(space: CoverageSpace, covered: np.ndarray, known: np.ndarray) -> np.ndarray:
    img = np.empty(space.free.shape + (3,), dtype=np.uint8)
    img[:] = COLORS["unknown"]
    img[known & ~space.free] = COLORS["occupied"]
    img[space.free] = COLORS["free_lost"]
    img[space.coverable] = COLORS["missed"]
    img[space.coverable & covered] = COLORS["covered"]
    return img[::-1]  # верх картинки — +Y


def write_image(path: Path, rgb: np.ndarray) -> Path:
    """PNG через Pillow, если он есть; иначе PPM (P6) рядом — тот же путь с суффиксом .ppm."""
    try:
        from PIL import Image
    except ImportError:
        path = path.with_suffix(".ppm")
        with open(path, "wb") as f:
            f.write(b"P6\n%d %d\n255\n" % (rgb.shape[1], rgb.shape[0]))
            f.write(rgb.tobytes())
        return path
    Image.fromarray(rgb, "RGB").save(path)
    return path


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Покрытие по треку: карта npz (map_recorder) + лог с TRACK")
    ap.add_argument("map", type=Path)
    ap.add_argument("log", type=Path)
    ap.add_argument("--out", type=Path, default=Path("coverage.png"))
    ap.add_argument("--erosion", type=float, default=None)
    ap.add_argument(
        "--start",
        type=float,
        nargs=2,
        default=None,
        metavar=("X", "Y"),
        help="старт для достижимости; по умолчанию первая точка трека",
    )
    args = ap.parse_args(argv)
    track = parse_track_log(args.log.read_text(encoding="utf-8", errors="replace").splitlines())
    if not track:
        print(f"нет строк TRACK в {args.log}", file=sys.stderr)
        return 2
    grid = load_grid_npz(args.map)
    start = tuple(args.start) if args.start else track[0][1:3]
    space = coverage_space(grid, start, erosion_m=args.erosion)
    covered = covered_mask(space, track)
    res = coverage_result(space, covered)
    known = np.asarray(grid.data).reshape(grid.height, grid.width) >= 0
    img = write_image(args.out, coverage_rgb(space, covered, known))
    out = {**res.__dict__, "pct": res.pct, "free_pct": res.free_pct, "samples": len(track), "image": str(img)}
    print(json.dumps(out, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    sys.exit(main())
