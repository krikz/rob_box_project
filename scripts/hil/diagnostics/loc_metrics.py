#!/usr/bin/env python3
"""Ошибка локализации против истины Gazebo (черновик #4568, приёмка Шифу 20.11).

Из черновика ``rob_box_sim.loc_metrics`` (ветка ``feat/4346-sim-localization-bench``,
не влита) взяты чистые функции: поза, ошибка, схождение, скачки, похищение,
сводка. Стенд, камера и прототип B из того PR сюда не переносились.

Что изменено против черновика и почему:

* курс считается в радианах, не в градусах: планка Шифу 20.11 — 0.35 рад
  (``docs/shifu-decisions.md``, «Приёмка Depot текущего окна»);
* вердикт цели решает код: позиция ≤ 0.32 м и курс ≤ 0.35 рад на одной позе,
  Nav2 сообщил успех. TIMEOUT — всегда провал, не PASS;
* нет истины или нет оценки — «нет данных», не ноль и не PASS;
* вход — строки лога заезда ``route.py`` (``MAP`` несёт ``map → base_footprint``
  и ``/sim/ground_truth/odom``, ``GOAL_END`` — итог Nav2). Стенд не нужен.

Скрипт не ходит на стенд и не запускает Gazebo.
"""

from __future__ import annotations

import argparse
import json
import math
import re
import statistics
import sys
from dataclasses import dataclass

# Планка Шифу 20.11: истина Gazebo, позиция и курс на одной позе.
POS_BAR_M = 0.32
YAW_BAR_RAD = 0.35

PASS = "PASS"
FAIL = "FAIL"
TIMEOUT = "TIMEOUT"
NO_DATA = "нет данных"

_FLOAT = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?"
_NUM_OR_NONE = rf"({_FLOAT}|none)"
_MAP_RE = re.compile(
    rf"MAP sim=({_FLOAT}) x={_NUM_OR_NONE} y={_NUM_OR_NONE} yaw={_NUM_OR_NONE} "
    rf"gt_x=({_FLOAT}) gt_y=({_FLOAT}) gt_yaw=({_FLOAT})"
)
_GOAL_END_RE = re.compile(rf"GOAL_END (\S+) result=(\S+) .*?sim=({_FLOAT})")

Pose = tuple[float, float, float]  # x, y, yaw рад


@dataclass(frozen=True)
class Sample:
    """Один замер: sim time, истина Gazebo, оценка ``map → base_footprint``.

    ``None`` — измерения не было. Ноль сюда не подставляется.
    """

    t: float
    truth: Pose | None
    estimate: Pose | None


@dataclass(frozen=True)
class Tolerance:
    """Допуски черновика #4568: схождение, окно финала, скачок, похищение.

    Курс — в радианах (в черновике было ``yaw_tol_deg``).
    """

    pos_tol_m: float = 0.15
    yaw_tol_rad: float = math.radians(5.0)
    hold_s: float = 5.0
    final_window_s: float = 5.0
    jump_m: float = 0.25
    kidnap_m: float = 0.5


def wrap(angle: float) -> float:
    """Угол в (−π, π]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def pose_error(truth: Pose, estimate: Pose) -> tuple[float, float]:
    """Ошибка положения (м) и курса (рад, по модулю) оценки против истины."""
    dist = math.hypot(estimate[0] - truth[0], estimate[1] - truth[1])
    yaw = abs(wrap(estimate[2] - truth[2]))
    return dist, yaw


def within_bar(dist_m: float, yaw_rad: float, pos_bar_m: float = POS_BAR_M, yaw_bar_rad: float = YAW_BAR_RAD) -> bool:
    """Оба допуска планки на одной позе. Один из двух — не зачёт."""
    return dist_m <= pos_bar_m and yaw_rad <= yaw_bar_rad


def _in_tolerance(sample: Sample, tol: Tolerance) -> bool:
    if sample.truth is None or sample.estimate is None:
        return False
    dist, yaw = pose_error(sample.truth, sample.estimate)
    return dist <= tol.pos_tol_m and yaw <= tol.yaw_tol_rad


def converged_at(samples: list[Sample], t_event: float, tol: Tolerance) -> float | None:
    """Первый t ≥ t_event, после которого замер в допуске не меньше ``hold_s``.

    Короткий проход через допуск — не схождение. Нет пары истина+оценка — тоже.
    """
    run_start: float | None = None
    for sample in samples:
        if sample.t < t_event:
            continue
        if _in_tolerance(sample, tol):
            if run_start is None:
                run_start = sample.t
            if sample.t - run_start >= tol.hold_s:
                return run_start
        else:
            run_start = None
    return None


def path_length(samples: list[Sample], t0: float, t1: float) -> float:
    """Путь по истине между t0 и t1 (м). Замеры без истины пропускаются."""
    points = [sample.truth for sample in samples if t0 <= sample.t <= t1 and sample.truth is not None]
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(points, points[1:]))


def kidnap_time(samples: list[Sample], tol: Tolerance) -> float | None:
    """Момент телепорта: первый скачок истины между соседними замерами больше ``kidnap_m``."""
    previous: Pose | None = None
    for sample in samples:
        if sample.truth is None:
            continue
        jumped = (
            previous is not None
            and math.hypot(sample.truth[0] - previous[0], sample.truth[1] - previous[1]) > tol.kidnap_m
        )
        if jumped:
            return sample.t
        previous = sample.truth
    return None


def estimate_jumps(samples: list[Sample], t_event: float, tol: Tolerance) -> tuple[int, int]:
    """(скачки оценки, из них плохие — ошибка после скачка выросла).

    Скачок — оценка сдвинулась на ``jump_m`` больше, чем истина. Скачок самой
    истины (телепорт) скачком оценки не считается.
    """
    paired = [s for s in samples if s.t >= t_event and s.truth is not None and s.estimate is not None]
    jumps = bad = 0
    for before, after in zip(paired, paired[1:]):
        estimate_step = (after.estimate[0] - before.estimate[0], after.estimate[1] - before.estimate[1])
        truth_step = (after.truth[0] - before.truth[0], after.truth[1] - before.truth[1])
        if math.hypot(estimate_step[0] - truth_step[0], estimate_step[1] - truth_step[1]) <= tol.jump_m:
            continue
        jumps += 1
        if pose_error(after.truth, after.estimate)[0] > pose_error(before.truth, before.estimate)[0]:
            bad += 1
    return jumps, bad


def _round(value: float | None, digits: int = 3) -> float | None:
    return None if value is None else round(value, digits)


def _metrics_row(after, paired, t_event, t_end, tol: Tolerance, converged, jumps) -> dict:
    """Собрать числа прогона. Нет пары истина+оценка — поля ошибки ``None``, не ноль."""
    first = paired[0] if paired else None
    err0 = pose_error(first.truth, first.estimate) if first else (None, None)
    tail = [pose_error(sample.truth, sample.estimate) for sample in paired if sample.t >= t_end - tol.final_window_s]
    errs = [pose_error(sample.truth, sample.estimate)[0] for sample in paired]
    return {
        "samples": len(after),
        "duration_s": _round(t_end - t_event, 1),
        "first_estimate_s": _round(first.t - t_event, 1) if first else None,
        "err0_m": _round(err0[0]),
        "yaw0_rad": _round(err0[1]),
        "converged_s": _round(converged - t_event, 1) if converged is not None else None,
        "converged_path_m": _round(path_length(after, t_event, converged), 2) if converged is not None else None,
        "err_final_m": _round(statistics.median(dist for dist, _ in tail)) if tail else None,
        "yaw_final_rad": _round(statistics.median(yaw for _, yaw in tail)) if tail else None,
        "err_max_m": _round(max(errs)) if errs else None,
        "jumps": jumps[0],
        "bad_jumps": jumps[1],
        "path_m": _round(path_length(after, t_event, t_end), 2),
    }


def run_metrics(samples: list[Sample], t_event: float | None = None, tol: Tolerance = Tolerance()) -> dict:
    """Метрики одного прогона от ``t_event`` (по умолчанию — первый замер).

    Пустой ряд — ``{"samples": 0}``, не нули. Нет пары истина+оценка — поля
    ошибки ``None``.
    """
    if not samples:
        return {"samples": 0}
    t_event = samples[0].t if t_event is None else t_event
    after = [sample for sample in samples if sample.t >= t_event]
    paired = [sample for sample in after if sample.truth is not None and sample.estimate is not None]
    return _metrics_row(
        after,
        paired,
        t_event,
        samples[-1].t,
        tol,
        converged_at(samples, t_event, tol),
        estimate_jumps(samples, t_event, tol),
    )


COLUMNS = (
    ("label", "прогон"),
    ("err0_m", "ошибка в начале, м"),
    ("yaw0_rad", "курс в начале, рад"),
    ("converged_s", "сошлось, с"),
    ("converged_path_m", "проехал до схождения, м"),
    ("err_final_m", "ошибка в конце, м"),
    ("yaw_final_rad", "курс в конце, рад"),
    ("jumps", "скачки"),
    ("bad_jumps", "ложные скачки"),
)


def summary_markdown(rows: list[dict]) -> str:
    """Таблица прогонов. ``None`` → «—»: не сошлось или нет данных, не ноль."""
    head = "| " + " | ".join(title for _, title in COLUMNS) + " |"
    sep = "|" + "|".join("---" for _ in COLUMNS) + "|"
    lines = [head, sep]
    for row in rows:
        cells = ["—" if row.get(key) is None else str(row.get(key)) for key, _ in COLUMNS]
        lines.append("| " + " | ".join(cells) + " |")
    return "\n".join(lines)


def _number_or_none(token: str) -> float | None:
    return None if token == "none" else float(token)


def parse_map_line(line: str) -> Sample | None:
    """Строка ``MAP`` лога ``route.py`` → замер. Чужая строка — ``None``.

    ``x=none`` — оценки не было: ``estimate`` остаётся ``None``, ноль не пишется.
    """
    found = _MAP_RE.search(line)
    if found is None:
        return None
    estimate = (
        _number_or_none(found.group(2)),
        _number_or_none(found.group(3)),
        _number_or_none(found.group(4)),
    )
    pose = None if None in estimate else (estimate[0], estimate[1], math.radians(estimate[2]))
    truth = (float(found.group(5)), float(found.group(6)), math.radians(float(found.group(7))))
    return Sample(float(found.group(1)), truth, pose)


def parse_goal_end(line: str) -> dict | None:
    """Строка ``GOAL_END`` → имя цели, итог Nav2, sim time. Чужая строка — ``None``."""
    found = _GOAL_END_RE.search(line)
    if found is None:
        return None
    return {"name": found.group(1), "result": found.group(2), "sim": float(found.group(3))}


def _nearest(samples: list[Sample], t: float) -> Sample | None:
    if not samples:
        return None
    return min(samples, key=lambda sample: abs(sample.t - t))


def goal_verdict(
    nav_result: str,
    sample: Sample | None,
    pos_bar_m: float = POS_BAR_M,
    yaw_bar_rad: float = YAW_BAR_RAD,
) -> dict:
    """Вердикт одной цели по планке Шифу.

    TIMEOUT — всегда провал, даже если поза внутри допуска. Нет истины или
    оценки — «нет данных», не PASS и не ноль. Оба допуска нужны на одной позе.
    """
    if "TIMEOUT" in (nav_result or ""):
        return {"verdict": TIMEOUT, "reason": f"Nav2: {nav_result} — таймаут всегда провал", "nav_result": nav_result}
    if sample is None or sample.truth is None:
        return {"verdict": NO_DATA, "reason": "нет истины Gazebo", "nav_result": nav_result}
    if sample.estimate is None:
        return {"verdict": NO_DATA, "reason": "нет оценки map → base_footprint", "nav_result": nav_result}
    dist, yaw = pose_error(sample.truth, sample.estimate)
    base = {
        "nav_result": nav_result,
        "err_m": _round(dist),
        "yaw_rad": _round(yaw),
        "pos_bar_m": pos_bar_m,
        "yaw_bar_rad": yaw_bar_rad,
    }
    if nav_result != "STATUS_SUCCEEDED":
        return {**base, "verdict": FAIL, "reason": f"Nav2: {nav_result}, не успех"}
    if within_bar(dist, yaw, pos_bar_m, yaw_bar_rad):
        return {**base, "verdict": PASS, "reason": f"поза {dist:.3f} м / {yaw:.4f} рад внутри планки"}
    return {
        **base,
        "verdict": FAIL,
        "reason": f"поза {dist:.3f} м / {yaw:.4f} рад вне планки {pos_bar_m} м / {yaw_bar_rad} рад",
    }


def verdicts_from_log(text: str, pos_bar_m: float = POS_BAR_M, yaw_bar_rad: float = YAW_BAR_RAD) -> list[dict]:
    """Вердикт каждой цели лога: ближайший замер ``MAP`` к ``GOAL_END``.

    Ближайший замер без оценки — «нет данных», не замер другого момента.
    """
    samples = [sample for line in text.splitlines() if (sample := parse_map_line(line)) is not None]
    out = []
    for line in text.splitlines():
        goal = parse_goal_end(line)
        if goal is None:
            continue
        nearest = _nearest(samples, goal["sim"])
        row = goal_verdict(goal["result"], nearest, pos_bar_m, yaw_bar_rad)
        row["name"] = goal["name"]
        row["sim"] = goal["sim"]
        out.append(row)
    return out


def run_verdict(rows: list[dict]) -> str:
    """Итог заезда. Нет целей или хоть одна без данных — «нет данных», не PASS.

    Хоть один TIMEOUT — TIMEOUT. Иначе FAIL, если хоть одна цель не PASS.
    """
    if not rows or any(row["verdict"] == NO_DATA for row in rows):
        return NO_DATA
    if any(row["verdict"] == TIMEOUT for row in rows):
        return TIMEOUT
    if all(row["verdict"] == PASS for row in rows):
        return PASS
    return FAIL


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description="Вердикт локализации против истины Gazebo по логу заезда.")
    parser.add_argument("log", help="route.log (строки MAP и GOAL_END)")
    parser.add_argument("--pos-bar-m", type=float, default=POS_BAR_M)
    parser.add_argument("--yaw-bar-rad", type=float, default=YAW_BAR_RAD)
    args = parser.parse_args(argv)
    with open(args.log, encoding="utf-8") as handle:
        rows = verdicts_from_log(handle.read(), args.pos_bar_m, args.yaw_bar_rad)
    json.dump({"verdict": run_verdict(rows), "goals": rows}, sys.stdout, ensure_ascii=False, indent=2)
    sys.stdout.write("\n")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
