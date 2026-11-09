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
        return round(100.0 * self.covered_m2 / self.coverable_m2, 2) if self.coverable_m2 > 0 else None


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
    name: str
    result: str  # SUCCEEDED | ABORTED | CANCELED | TIMEOUT | REJECTED | NOT_SENT
    em_episodes: int = 0


def run_summary(outcomes: Sequence[GoalOutcome], cov: CoverageResult, sim_s: float, stop_reason: str = "") -> str:
    """Итоговая строка обхода: числа только из исходов и метрики."""
    by = Counter(o.result for o in outcomes)
    ok = by.pop("SUCCEEDED", 0)
    not_sent = by.pop("NOT_SENT", 0)
    failed = ", ".join(f"{k}={v}" for k, v in sorted(by.items())) or "-"
    pct = "n/a" if cov.pct is None else f"{cov.pct:.1f}%"
    return (
        f"COVERAGE_DONE poses={len(outcomes)} succeeded={ok} skipped={sum(by.values())} ({failed}) "
        f"not_sent={not_sent} stop={stop_reason or 'end_of_plan'} "
        f"em_episodes={sum(o.em_episodes for o in outcomes)} sim_s={sim_s:.1f} "
        f"coverage={pct} covered_m2={cov.covered_m2:.1f} coverable_m2={cov.coverable_m2:.1f} "
        f"free_covered_m2={cov.free_covered_m2:.1f} free_m2={cov.free_m2:.1f}"
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
    print(json.dumps({**res.__dict__, "pct": res.pct, "samples": len(track), "image": str(img)}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    sys.exit(main())
