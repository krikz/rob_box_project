#!/usr/bin/env python3
"""Обход покрытием в HIL (#4560): карта → план полос → цели NavigateToPose по одной → итог покрытия.

Рядом с route.py (тот же лог: TRACK по /sim/ground_truth/odom, GOAL_START/GOAL_END, EM_ON/EM_OFF по
/cmd_vel_emergency, RECOVERY) и запускается так же — в контейнере rob-box-hil-gazebo (runcoverage.sh).
План и метрика — чистые модули rob_box_sim.coverage_plan / coverage_metric (тесты
tests/unit/sim/test_sim_coverage.py); здесь только ROS и порядок действий.

Недостижимую позу (ABORTED/REJECTED/TIMEOUT) пропускаем и считаем; после --max-fail неудач подряд
обход останавливается (робот застрял — дальше только таймауты). Итоговую строку COVERAGE_DONE строит
coverage_metric.run_summary по исходам целей и треку истинной позы.

  python3 -u coverage_run.py --out /sim_video/coverage [--plan-only] [--spacing 0.55] [--axis auto]
"""
import argparse
import json
import math
import sys
import time
from pathlib import Path

import rclpy
from rob_box_sim.coverage_metric import (
    GoalOutcome,
    coverage_result,
    coverage_rgb,
    covered_mask,
    plan_track,
    run_summary,
    write_image,
)
from rob_box_sim.coverage_plan import coverage_space, plan_coverage, plan_summary, plan_to_json, save_grid_npz
from rob_box_sim.map_check import grab_map
from route import Mon, run_goal, ts, wait_ready


class CoverageMon(Mon):
    """Mon из route.py, который ещё копит истинную позу (t, x, y, yaw рад) для метрики."""

    def __init__(self):
        super().__init__("shidi_coverage")
        self.track = []

    def on_gt(self, m):
        super().on_gt(m)
        p = (self.simt(), self.gt[0], self.gt[1], math.radians(self.gt[2]))
        if not self.track or _moved(self.track[-1], p):
            self.track.append(p)


def _moved(a, b, d_m=0.02, d_yaw=math.radians(2), d_t=1.0):
    """Прореживание трека: новая точка, если сдвиг, поворот или пауза заметны."""
    dyaw = abs(math.atan2(math.sin(b[3] - a[3]), math.cos(b[3] - a[3])))
    return math.hypot(b[1] - a[1], b[2] - a[2]) >= d_m or dyaw >= d_yaw or b[0] - a[0] >= d_t


def parse_args(argv):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--out", type=Path, default=Path("/sim_video/coverage"))
    ap.add_argument("--map-topic", default="/rtabmap/map")
    ap.add_argument("--axis", default="auto", choices=("auto", "x", "y"))
    ap.add_argument("--spacing", type=float, default=0.55, help="шаг полос, м")
    ap.add_argument("--erosion", type=float, default=None, help="полусторона сужения, м (умолч. 0.3364)")
    ap.add_argument("--waypoint-step", type=float, default=0.0, help="промежуточные позы на полосе, м (0 — нет)")
    ap.add_argument("--timeout", type=float, default=120.0, help="таймаут на позу, с стены")
    ap.add_argument("--max-fail", type=int, default=5, help="столько неудач подряд — стоп")
    ap.add_argument("--max-poses", type=int, default=0, help="ограничить число поз (0 — все)")
    ap.add_argument("--plan-only", action="store_true", help="только план и картинка, без езды")
    return ap.parse_args(argv)


def _write_plan(out, grid, space, plan):
    out.mkdir(parents=True, exist_ok=True)
    save_grid_npz(grid, out / "plan_map.npz")
    (out / "plan.json").write_text(json.dumps(plan_to_json(plan), ensure_ascii=False, indent=1), encoding="utf-8")
    cov = covered_mask(space, plan_track(plan.poses))
    img = write_image(out / "plan.png", coverage_rgb(space, cov, _known(grid)))
    print(
        f"{ts()} {plan_summary(plan, space)} plan_coverage={coverage_result(space, cov).pct}% image={img}", flush=True
    )
    for c in plan.unreachable:
        print(
            f"{ts()} UNREACHABLE cell={c.id} centers_m2={c.centers_m2:.2f} "
            f"x=[{c.x_min:.2f},{c.x_max:.2f}] y=[{c.y_min:.2f},{c.y_max:.2f}]",
            flush=True,
        )


def _known(grid):
    import numpy as np

    return np.asarray(grid.data).reshape(grid.height, grid.width) >= 0


def drive(n, cl, poses, timeout, max_fail):
    """Позы по одной; вернуть (исходы, причина остановки). Решение «стоп» — по счёту неудач подряд."""
    outcomes, fails = [], 0
    for k, p in enumerate(poses):
        if fails >= max_fail:
            outcomes += [GoalOutcome(f"p{j:04d}", "NOT_SENT") for j in range(k, len(poses))]
            return outcomes, f"{max_fail}_failures_in_row"
        name = f"p{k:04d}_c{p.cell}l{p.lane}{p.kind[0]}"
        result, em = run_goal(n, cl, name, round(p.x, 3), round(p.y, 3), round(math.degrees(p.yaw), 1), timeout)
        outcomes.append(GoalOutcome(name, result, em))
        fails = 0 if result == "SUCCEEDED" else fails + 1
    return outcomes, ""


def main(argv=None):
    args = parse_args(argv)
    rclpy.init()
    n = CoverageMon()
    cl = None if args.plan_only else wait_ready(n)
    t_end = time.time() + 15
    while n.gt is None and time.time() < t_end:
        rclpy.spin_once(n, timeout_sec=0.1)
    grid = grab_map(n, args.map_topic, timeout_s=60.0)
    if grid is None or n.gt is None:
        print(f"{ts()} COVERAGE_FAIL нет карты {args.map_topic} или истинной позы", flush=True)
        rclpy.shutdown()
        return 2
    space = coverage_space(grid, n.gt[:2], erosion_m=args.erosion)
    plan = plan_coverage(space, axis=args.axis, lane_spacing_m=args.spacing, waypoint_step_m=args.waypoint_step)
    _write_plan(args.out, grid, space, plan)
    if args.plan_only:
        rclpy.shutdown()
        return 0
    poses = plan.poses[: args.max_poses] if args.max_poses else plan.poses
    n.track.clear()
    sim0 = n.simt()
    outcomes, stop = drive(n, cl, poses, args.timeout, args.max_fail)
    cov_mask = covered_mask(space, n.track)
    cov = coverage_result(space, cov_mask)
    img = write_image(args.out / "coverage.png", coverage_rgb(space, cov_mask, _known(grid)))
    (args.out / "track.json").write_text(json.dumps(n.track), encoding="utf-8")
    print(f"{ts()} {run_summary(outcomes, cov, n.simt() - sim0, stop)} image={img}", flush=True)
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
