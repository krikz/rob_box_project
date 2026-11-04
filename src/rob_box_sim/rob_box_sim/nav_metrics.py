"""Метрики одной цели навигации по записанным образцам. Без ROS.

План: docs/research/2026-11-04-gazebo-sim-navigation.md §6 шаг 4.

Источники (их пишет nav_bench, здесь — только счёт):
  * ``truth`` — истинная поза из Gazebo ``/sim/ground_truth/odom``;
  * ``estimate`` — что думает робот: TF ``map → base_footprint`` в тот же миг;
  * ``cmd`` — ``/cmd_vel`` (что ушло в twist_mux и колёса: после velocity_smoother,
    если он в цепочке, шаг 6);
  * ``cmd_raw`` — ``/cmd_vel_nav`` (выход контроллера до velocity_smoother; пусто,
    когда smoother не в цепочке и контроллер пишет прямо в ``/cmd_vel``);
  * ``scan_min`` — минимум ``/scan`` (что видит лидар);
  * ``recoveries`` — ``number_of_recoveries`` из feedback ``NavigateToPose``;
  * ``tilt_max_rad`` — наибольший наклон корпуса по истинной позе: при наклоне
    ~10° плоскость лидара (z ≈ 0.52 м) задевает пол в ~3 м и пишет в costmap
    препятствие, которого нет.

Зазор и столкновения считаются по геометрии: габарит робота в истинной позе
против коробок мира (geometry2d.clearance). Зазор < CONTACT_M — касание/столкновение:
физика не даёт корпусу войти в коробку, он упирается с зазором около нуля (замер
на low_obstacle: -0.002 м), а координаты мира округлены до 1 мм.
Так ловится и то, чего лидар не видит (низкое препятствие).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional, Sequence

from rob_box_sim.geometry2d import Box, Pose2D, clearance, wrap_angle

# |w| ниже этого — «ноль», смена знака через ноль не считается колебанием.
OSC_DEADBAND = 0.05
# Зазор меньше этого — касание препятствия (столкновение).
CONTACT_M = 0.01


@dataclass
class GoalRecord:
    name: str
    goal: Pose2D
    status: str  # SUCCEEDED | FAILED | CANCELED | TIMEOUT | REJECTED | SKIPPED
    t_start: float
    t_end: float
    truth: list[tuple[float, float, float, float]] = field(default_factory=list)  # t, x, y, yaw
    estimate: list[Optional[tuple[float, float, float]]] = field(default_factory=list)  # x, y, yaw | None
    cmd: list[tuple[float, float, float]] = field(default_factory=list)  # t, vx, wz
    cmd_raw: list[tuple[float, float, float]] = field(default_factory=list)  # t, vx, wz до smoother
    scan_min: list[tuple[float, float]] = field(default_factory=list)  # t, r
    recoveries: int = 0
    tilt_max_rad: float = 0.0  # наибольший наклон корпуса по истинной позе (тангаж/крен)


def sign_changes(values: Sequence[float], deadband: float = OSC_DEADBAND) -> int:
    """Сколько раз знак сменился, не считая значений в мёртвой зоне."""
    last = 0
    n = 0
    for v in values:
        if abs(v) < deadband:
            continue
        s = 1 if v > 0 else -1
        if last and s != last:
            n += 1
        last = s
    return n


def path_length(points: Sequence[tuple[float, float]]) -> float:
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(points, points[1:]))


def _r(v: Optional[float], nd: int = 3) -> Optional[float]:
    if v is None or (isinstance(v, float) and (math.isinf(v) or math.isnan(v))):
        return None
    return round(v, nd)


def goal_metrics(rec: GoalRecord, boxes: Sequence[Box]) -> dict:
    """Метрики одной цели; значения, которых нет, — None (не ноль)."""
    pts = [(x, y) for _, x, y, _ in rec.truth]
    start = Pose2D(*rec.truth[0][1:]) if rec.truth else None
    end = Pose2D(*rec.truth[-1][1:]) if rec.truth else None

    clear = [(t, clearance(Pose2D(x, y, yaw), boxes)) for t, x, y, yaw in rec.truth]
    min_clear = min((c for _, c in clear), default=None)
    collision_s = 0.0
    for (t0, c0), (t1, _) in zip(clear, clear[1:]):
        if c0 < CONTACT_M:
            collision_s += t1 - t0

    errs = []
    yaw_errs = []
    for (_, x, y, yaw), est in zip(rec.truth, rec.estimate):
        if est is None:
            continue
        errs.append(math.hypot(est[0] - x, est[1] - y))
        yaw_errs.append(abs(wrap_angle(est[2] - yaw)))
    last_est = next((e for e in reversed(rec.estimate) if e is not None), None)

    plen = path_length(pts)
    straight = start.distance_to(rec.goal) if start else None
    wz = [w for _, _, w in rec.cmd]
    wz_raw = [w for _, _, w in rec.cmd_raw]
    scan = [r for _, r in rec.scan_min if math.isfinite(r)]

    out = {
        "goal": rec.name,
        "status": rec.status,
        "success": rec.status == "SUCCEEDED",
        "time_s": _r(rec.t_end - rec.t_start, 1),
        "goal_x": rec.goal.x,
        "goal_y": rec.goal.y,
        "end_x": _r(end.x) if end else None,
        "end_y": _r(end.y) if end else None,
        "goal_err_m": _r(end.distance_to(rec.goal)) if end else None,
        "path_len_m": _r(plen),
        "straight_m": _r(straight),
        "path_ratio": _r(plen / straight, 2) if straight else None,
        "pose_err_mean_m": _r(sum(errs) / len(errs)) if errs else None,
        "pose_err_max_m": _r(max(errs)) if errs else None,
        "pose_err_end_m": _r(math.hypot(last_est[0] - end.x, last_est[1] - end.y)) if last_est and end else None,
        "yaw_err_max_rad": _r(max(yaw_errs)) if yaw_errs else None,
        "min_clearance_m": _r(min_clear),
        "collision": bool(min_clear is not None and min_clear < CONTACT_M),
        "collision_s": _r(collision_s, 1),
        "min_scan_m": _r(min(scan)) if scan else None,
        "osc_sign_changes": sign_changes(wz),
        "osc_per_m": _r(sign_changes(wz) / plen, 2) if plen > 0.05 else None,
        "osc_raw_per_m": _r(sign_changes(wz_raw) / plen, 2) if wz_raw and plen > 0.05 else None,
        "recoveries": rec.recoveries,
        "tilt_max_deg": _r(math.degrees(rec.tilt_max_rad), 1) if rec.truth else None,
        "samples_truth": len(rec.truth),
        "samples_cmd": len(rec.cmd),
    }
    if not rec.truth:  # цель не ехали (SKIPPED/REJECTED) или поза не пришла: чисел нет
        for k in ("time_s", "path_len_m", "collision", "collision_s", "osc_sign_changes", "recoveries"):
            out[k] = None
    return out


SUMMARY_COLUMNS = (
    ("world", "мир"),
    ("goal", "цель"),
    ("status", "итог"),
    ("time_s", "время, с"),
    ("goal_err_m", "до цели, м"),
    ("path_ratio", "путь/прямая"),
    ("pose_err_max_m", "ошибка позы max, м"),
    ("pose_err_end_m", "ошибка позы в конце, м"),
    ("min_clearance_m", "мин. зазор, м"),
    ("collision", "столкн."),
    ("min_scan_m", "мин. скан, м"),
    ("osc_sign_changes", "смен знака w"),
    ("osc_per_m", "смен знака w на м"),
    ("recoveries", "recovery"),
)


def _cell(v) -> str:
    if v is None:
        return "—"
    if isinstance(v, bool):
        return "да" if v else "нет"
    return str(v)


def summary_markdown(rows: Sequence[dict]) -> str:
    head = "| " + " | ".join(t for _, t in SUMMARY_COLUMNS) + " |"
    sep = "|" + "|".join("---" for _ in SUMMARY_COLUMNS) + "|"
    body = ["| " + " | ".join(_cell(r.get(k)) for k, _ in SUMMARY_COLUMNS) + " |" for r in rows]
    ok = sum(1 for r in rows if r.get("success"))
    ran = sum(1 for r in rows if r.get("status") != "SKIPPED")
    return "\n".join([head, sep, *body, "", f"Успех: {ok} из {ran} запущенных целей ({len(rows)} всего)."])
