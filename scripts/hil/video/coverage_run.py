#!/usr/bin/env python3
"""Обход покрытием в HIL (#4560): карта → план полос → цели NavigateToPose по одной → итог покрытия.

Рядом с route.py (тот же лог: TRACK по /sim/ground_truth/odom, GOAL_START/GOAL_END, EM_ON/EM_OFF по
/cmd_vel_emergency, RECOVERY) и запускается так же — в контейнере rob-box-hil-gazebo (runcoverage.sh).
План и метрика — чистые модули rob_box_sim.coverage_plan / coverage_metric (тесты
tests/unit/sim/test_sim_coverage.py); здесь только ROS и порядок действий.

Недостижимую позу (ABORTED/REJECTED/TIMEOUT) пропускаем и считаем; после --max-fail неудач подряд
обход останавливается (робот застрял — дальше только таймауты). Итоговую строку COVERAGE_DONE строит
coverage_metric.run_summary по исходам целей и треку истинной позы.

SIGINT/SIGTERM: текущая цель отменяется (INTERRUPTED), остаток — NOT_SENT, во вход Nav2 /cmd_vel — нулевая
скорость, COVERAGE_DONE пишется по пройденному со stop=interrupted_SIGINT. Второй сигнал — выход сразу.

  python3 -u coverage_run.py --out /sim_video/coverage [--plan-only] [--spacing 0.55] [--axis auto]
"""
import argparse
import json
import math
import signal
import sys
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
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
    ap.add_argument(
        "--turn-clearance",
        type=float,
        default=None,
        help="до занятого в точке разворота, м (умолч. 0.66: описанный радиус + стоп защиты + padding; 0 — нет)",
    )
    ap.add_argument("--no-cross", action="store_true", help="без поперечного прохода по торцам у препятствий")
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
    res = coverage_result(space, cov)
    print(
        f"{ts()} {plan_summary(plan, space)} plan_coverage={res.pct}% plan_coverage_free={res.free_pct}% image={img}",
        flush=True,
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


class StopSignal:
    """SIGINT/SIGTERM → флаг: текущую цель отменить, итог дописать по пройденному. Второй сигнал — выход сразу."""

    def __init__(self):
        self.name = ""

    def __call__(self, signum, frame=None):
        if self.name:
            raise KeyboardInterrupt
        self.name = signal.Signals(signum).name
        print(f"{ts()} STOP_SIGNAL {self.name}: отменяю цель, итог по пройденному", flush=True)

    def install(self):
        for s in (signal.SIGINT, signal.SIGTERM):
            signal.signal(s, self)
        return self


def _stop_reason(stop, fails, max_fail):
    if stop.name:
        return f"interrupted_{stop.name}"
    return f"{max_fail}_failures_in_row" if fails >= max_fail else ""


def drive(n, cl, poses, timeout, max_fail, stop=None):
    """Позы по одной; вернуть (исходы, причина остановки). «Стоп» — по счёту неудач подряд или по сигналу."""
    stop = stop or StopSignal()
    outcomes, fails = [], 0
    for k, p in enumerate(poses):
        why = _stop_reason(stop, fails, max_fail)
        if why:
            outcomes += [GoalOutcome(f"p{j:04d}", "NOT_SENT") for j in range(k, len(poses))]
            return outcomes, why
        name = f"p{k:04d}_c{p.cell}l{p.lane}{p.kind[0]}"
        xy_yaw = (round(p.x, 3), round(p.y, 3), round(math.degrees(p.yaw), 1))
        result, em = run_goal(n, cl, name, *xy_yaw, timeout, stop=lambda: bool(stop.name))
        outcomes.append(GoalOutcome.from_route(name, result, em))
        fails = 0 if outcomes[-1].succeeded else fails + 1
    return outcomes, f"interrupted_{stop.name}" if stop.name else ""


def halt(n, topic="/cmd_vel", times=5):
    """Нулевая скорость во вход Nav2 у twist_mux: после отмены цели робот не доедет по старой команде."""
    pub = n.create_publisher(Twist, topic, 10)
    for _ in range(times):
        pub.publish(Twist())
        rclpy.spin_once(n, timeout_sec=0.1)
    print(f"{ts()} HALT {topic} x{times}", flush=True)


def finish(out, n, grid, space, outcomes, stop, sim0):
    """Итог обхода по уже пройденному: картинка, трек, строка COVERAGE_DONE. Вернуть строку."""
    cov_mask = covered_mask(space, n.track)
    cov = coverage_result(space, cov_mask)
    out.mkdir(parents=True, exist_ok=True)
    img = write_image(out / "coverage.png", coverage_rgb(space, cov_mask, _known(grid)))
    (out / "track.json").write_text(json.dumps(n.track), encoding="utf-8")
    line = f"{ts()} {run_summary(outcomes, cov, n.simt() - sim0, stop)} image={img}"
    print(line, flush=True)
    return line


def _init_ros():
    """rclpy без своих обработчиков сигналов: иначе SIGINT гасит контекст, и отменить цель уже нечем."""
    try:
        from rclpy.signals import SignalHandlerOptions
    except ImportError:
        rclpy.init()
        return
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)


def main(argv=None):
    args = parse_args(argv)
    _init_ros()
    stop = StopSignal().install()
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
    space = coverage_space(grid, n.gt[:2], erosion_m=args.erosion, turn_clearance_m=args.turn_clearance)
    plan = plan_coverage(
        space,
        axis=args.axis,
        lane_spacing_m=args.spacing,
        waypoint_step_m=args.waypoint_step,
        cross=not args.no_cross,
    )
    _write_plan(args.out, grid, space, plan)
    if args.plan_only:
        rclpy.shutdown()
        return 0
    poses = plan.poses[: args.max_poses] if args.max_poses else plan.poses
    n.track.clear()
    sim0 = n.simt()
    outcomes, why = drive(n, cl, poses, args.timeout, args.max_fail, stop)
    if stop.name:
        halt(n)
    finish(args.out, n, grid, space, outcomes, why, sim0)
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
