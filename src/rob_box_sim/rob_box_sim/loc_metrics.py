"""Метрики стенда локализации (#4346): истинная поза Gazebo против оценки ``map → base_footprint``.

Без ROS: на входе ряд замеров ``Sample`` (sim time, истина, оценка), на выходе числа одного
прогона — :func:`run_metrics`. Пишет их ``rob_box_sim.loc_bench``, сводит в таблицу
:func:`summary_markdown`.

Что считаем от момента события ``t_event`` (старт стека или «похищение»):
  * ``first_estimate_s`` — когда появилась оценка (TF map → base_footprint);
  * ``err0_m`` / ``yaw0_deg`` — ошибка первой оценки;
  * ``converged_s`` — первый момент, после которого ошибка держится в допуске
    ``pos_tol_m``/``yaw_tol_deg`` не меньше ``hold_s``, и ``converged_path_m`` — сколько
    робот проехал (по истине) до этого момента; None — не сошлось за прогон;
  * ``err_final_m`` / ``yaw_final_deg`` — медиана ошибки за последние ``final_window_s``;
  * ``jumps`` — скачки оценки: за шаг между замерами оценка сдвинулась на ``jump_m`` больше,
    чем истина (так видно релокализацию); ``bad_jumps`` — те из них, после которых ошибка выросла.
"""

from __future__ import annotations

import math
import statistics
from dataclasses import dataclass

Pose = tuple[float, float, float]  # x, y, yaw


@dataclass(frozen=True)
class Sample:
    t: float
    truth: Pose | None
    estimate: Pose | None


@dataclass(frozen=True)
class Tolerance:
    pos_tol_m: float = 0.15
    yaw_tol_deg: float = 5.0
    hold_s: float = 5.0
    final_window_s: float = 5.0
    jump_m: float = 0.25
    kidnap_m: float = 0.5  # скачок истины между замерами больше этого — телепорт


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def pose_error(truth: Pose, est: Pose) -> tuple[float, float]:
    """(ошибка положения, м; ошибка курса, градусы по модулю)."""
    return math.hypot(est[0] - truth[0], est[1] - truth[1]), abs(math.degrees(wrap(est[2] - truth[2])))


def _ok(s: Sample, tol: Tolerance) -> bool:
    if s.truth is None or s.estimate is None:
        return False
    d, a = pose_error(s.truth, s.estimate)
    return d <= tol.pos_tol_m and a <= tol.yaw_tol_deg


def converged_at(samples: list[Sample], t_event: float, tol: Tolerance) -> float | None:
    """Первый t ≥ t_event, после которого все замеры в допуске хотя бы ``hold_s`` (и до конца окна)."""
    run_start: float | None = None
    for s in samples:
        if s.t < t_event:
            continue
        if _ok(s, tol):
            if run_start is None:
                run_start = s.t
            if s.t - run_start >= tol.hold_s:
                return run_start
        else:
            run_start = None
    return None


def path_length(samples: list[Sample], t0: float, t1: float) -> float:
    pts = [s.truth for s in samples if t0 <= s.t <= t1 and s.truth is not None]
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:]))


def kidnap_time(samples: list[Sample], tol: Tolerance) -> float | None:
    """Момент телепорта: первый скачок истины между соседними замерами больше ``kidnap_m``."""
    prev = None
    for s in samples:
        if s.truth is None:
            continue
        if prev is not None and math.hypot(s.truth[0] - prev[0], s.truth[1] - prev[1]) > tol.kidnap_m:
            return s.t
        prev = s.truth
    return None


def estimate_jumps(samples: list[Sample], t_event: float, tol: Tolerance) -> tuple[int, int]:
    """(скачки оценки, из них «плохие» — ошибка после скачка больше, чем до)."""
    pairs = [s for s in samples if s.t >= t_event and s.truth is not None and s.estimate is not None]
    jumps = bad = 0
    for a, b in zip(pairs, pairs[1:]):
        de = (b.estimate[0] - a.estimate[0], b.estimate[1] - a.estimate[1])
        dt = (b.truth[0] - a.truth[0], b.truth[1] - a.truth[1])
        if math.hypot(de[0] - dt[0], de[1] - dt[1]) > tol.jump_m:
            jumps += 1
            if pose_error(b.truth, b.estimate)[0] > pose_error(a.truth, a.estimate)[0]:
                bad += 1
    return jumps, bad


def _r(v: float | None, nd: int = 3) -> float | None:
    return None if v is None else round(v, nd)


def run_metrics(samples: list[Sample], t_event: float | None = None, tol: Tolerance = Tolerance()) -> dict:
    """Метрики одного прогона от ``t_event`` (по умолчанию — первый замер)."""
    if not samples:
        return {"samples": 0}
    t_event = samples[0].t if t_event is None else t_event
    after = [s for s in samples if s.t >= t_event]
    paired = [s for s in after if s.truth is not None and s.estimate is not None]
    first = paired[0] if paired else None
    err0 = pose_error(first.truth, first.estimate) if first else (None, None)
    t_end = samples[-1].t
    tail = [pose_error(s.truth, s.estimate) for s in paired if s.t >= t_end - tol.final_window_s]
    conv = converged_at(samples, t_event, tol)
    errs = [pose_error(s.truth, s.estimate)[0] for s in paired]
    jumps, bad = estimate_jumps(samples, t_event, tol)
    return {
        "samples": len(after),
        "duration_s": _r(t_end - t_event, 1),
        "first_estimate_s": _r(first.t - t_event, 1) if first else None,
        "err0_m": _r(err0[0]),
        "yaw0_deg": _r(err0[1], 1),
        "converged_s": _r(conv - t_event, 1) if conv is not None else None,
        "converged_path_m": _r(path_length(samples, t_event, conv), 2) if conv is not None else None,
        "err_final_m": _r(statistics.median(d for d, _ in tail)) if tail else None,
        "yaw_final_deg": _r(statistics.median(a for _, a in tail), 1) if tail else None,
        "err_max_m": _r(max(errs)) if errs else None,
        "jumps": jumps,
        "bad_jumps": bad,
        "path_m": _r(path_length(samples, t_event, t_end), 2),
    }


COLUMNS = (
    ("label", "прогон"),
    ("err0_m", "ошибка в начале, м"),
    ("yaw0_deg", "курс в начале, °"),
    ("converged_s", "сошлось, с"),
    ("converged_path_m", "проехал до схождения, м"),
    ("err_final_m", "ошибка в конце, м"),
    ("yaw_final_deg", "курс в конце, °"),
    ("jumps", "скачки"),
    ("bad_jumps", "ложные скачки"),
    ("rtf", "RTF"),
)


def summary_markdown(rows: list[dict]) -> str:
    """Таблица прогонов; None → «—» (не сошлось / нет данных)."""
    head = "| " + " | ".join(t for _, t in COLUMNS) + " |"
    sep = "|" + "|".join("---" for _ in COLUMNS) + "|"
    lines = [head, sep]
    for row in rows:
        cells = ["—" if row.get(k) is None else str(row.get(k)) for k, _ in COLUMNS]
        lines.append("| " + " | ".join(cells) + " |")
    return "\n".join(lines)
