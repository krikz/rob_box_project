#!/usr/bin/env python3
"""Обход покрытием в HIL (#4560): карта → план полос → полоса одной целью NavigateThroughPoses → итог покрытия.

Рядом с route.py (тот же лог: TRACK по /sim/ground_truth/odom, GOAL_START/GOAL_END, EM_ON/EM_OFF по
/cmd_vel_emergency, RECOVERY) и запускается так же — в контейнере rob-box-hil-gazebo (runcoverage.sh).
План и метрика — чистые модули rob_box_sim.coverage_plan / coverage_metric (тесты
tests/unit/sim/test_sim_coverage.py); здесь только ROS и порядок действий.

Полоса (позы одной полосы плана: start → промежуточные → end) уходит одной целью NavigateThroughPoses —
робот не тормозит в каждой позе (HIL Depot 09.11, попытка 3: позами по одной средняя 0.06 м/с, в движении
59 % времени). Таймаут полосы — ``--timeout`` на позу × поз + путь / потолок скорости Nav2. ``--per-pose`` —
по-старому, NavigateToPose на каждую позу.

Неудачную цель (ABORTED/REJECTED/TIMEOUT) пропускаем и считаем; после --max-fail неудач подряд
обход останавливается (робот застрял — дальше только таймауты). Итоговую строку COVERAGE_DONE строит
coverage_metric.run_summary по исходам целей и треку истинной позы (покрытие, время в движении, скорость).

SIGINT/SIGTERM: текущая цель отменяется (INTERRUPTED), остаток — NOT_SENT, во вход Nav2 /cmd_vel — нулевая
скорость, COVERAGE_DONE пишется по пройденному со stop=interrupted_SIGINT. Второй сигнал — выход сразу.

``--perimeter`` (#4607): после змейки — проход по периметру (rob_box_sim.coverage_perimeter). На каждый нырок две
цели: подъезд к началу (NavigateToPose в место разворота) и сам путь целью FollowPath контроллера Nav2 — путь строит
код, без разворота у стены и без заднего хода. Подъезд не удался — путь не шлётся (NOT_SENT): иначе DWB поехал бы
к пути оттуда, где стоит. Итог нырков — строка PERIMETER_DONE (покрытие по треку до и после периметра);
COVERAGE_DONE считает всё вместе. ``--perimeter-only`` — без езды змейкой (проба периметра).

  python3 -u coverage_run.py --out /sim_video/coverage [--plan-only] [--spacing 0.55] [--axis auto] [--perimeter]
"""
import argparse
import json
import math
import signal
import sys
import time
from pathlib import Path

import rclpy
import route
from geometry_msgs.msg import Twist
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Path as NavPath
from rclpy.action import ActionClient
from rob_box_sim.coverage_metric import (
    GoalOutcome,
    coverage_result,
    coverage_rgb,
    covered_mask,
    motion_stats,
    plan_track,
    run_summary,
    write_image,
)
from rob_box_sim.coverage_perimeter import perimeter_done, perimeter_summary, plan_perimeter, reverse_m, strips_to_json
from rob_box_sim.coverage_plan import (
    SPEED_MPS,
    coverage_space,
    plan_coverage,
    plan_summary,
    plan_to_json,
    save_grid_npz,
)
from rob_box_sim.map_check import grab_map
from route import Mon, _pose, run_goal, run_lane, through_poses_client, ts, wait_ready


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
    ap.add_argument("--timeout", type=float, default=45.0, help="таймаут на позу, с стены (полоса: × поз + путь)")
    ap.add_argument("--per-pose", action="store_true", help="NavigateToPose на каждую позу вместо полосы целиком")
    ap.add_argument("--max-fail", type=int, default=5, help="столько неудач подряд — стоп")
    ap.add_argument("--max-poses", type=int, default=0, help="ограничить число поз (0 — все)")
    ap.add_argument("--plan-only", action="store_true", help="только план и картинка, без езды")
    ap.add_argument("--perimeter", action="store_true", help="после змейки — проход по периметру (#4607)")
    ap.add_argument("--perimeter-only", action="store_true", help="только проход по периметру, змейкой не ехать")
    ap.add_argument("--wall-offset", type=float, default=None, help="центр от стены на периметре, м (умолч. 0.4125)")
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


def lanes_of(poses):
    """Позы плана → полосы: подряд идущие позы одной полосы (cell, lane) — одна группа."""
    out = []
    for p in poses:
        if out and (out[-1][-1].cell, out[-1][-1].lane) == (p.cell, p.lane):
            out[-1].append(p)
        else:
            out.append([p])
    return out


def lane_timeout(group, here, per_pose_s, speed_mps=SPEED_MPS):
    """Таймаут полосы, с: ``per_pose_s`` на позу + путь (от ``here`` через все позы) на потолке скорости Nav2."""
    pts = ([here] if here is not None else []) + [(p.x, p.y) for p in group]
    path_m = sum(math.dist(a, b) for a, b in zip(pts, pts[1:]))
    return per_pose_s * len(group) + path_m / speed_mps


def _xy_yaw(p):
    return round(p.x, 3), round(p.y, 3), round(math.degrees(p.yaw), 1)


def _send(n, cl, k, group, timeout, stop_fn, here):
    """Одна цель: поза (NavigateToPose) или полоса (NavigateThroughPoses). Вернуть GoalOutcome."""
    p = group[0]
    if len(group) == 1:
        name = f"p{k:04d}_c{p.cell}l{p.lane}{p.kind[0]}"
        raw, em = run_goal(n, cl, name, *_xy_yaw(p), timeout, stop=stop_fn)
        return GoalOutcome.from_route(name, raw, em)
    name = f"L{k:04d}_c{p.cell}l{p.lane}"
    t = lane_timeout(group, here, timeout)
    raw, em, left = run_lane(n, cl, name, [_xy_yaw(q) for q in group], t, stop=stop_fn)
    return GoalOutcome.from_route(name, raw, em, poses=len(group), remaining=left)


def drive(n, cl, poses, timeout, max_fail, stop=None, lanes=False):
    """Цели по одной; вернуть (исходы, причина остановки). «Стоп» — по счёту неудач подряд или по сигналу.

    ``lanes`` — полоса одной целью (``cl`` — клиент NavigateThroughPoses), иначе поза одной целью.
    """
    stop = stop or StopSignal()
    groups = lanes_of(poses) if lanes else [[p] for p in poses]
    outcomes, fails, here = [], 0, None
    for k, group in enumerate(groups):
        why = _stop_reason(stop, fails, max_fail)
        if why:
            outcomes += [GoalOutcome.not_sent(f"g{j:04d}", len(g)) for j, g in enumerate(groups[k:], k)]
            return outcomes, why
        outcomes.append(_send(n, cl, k, group, timeout, lambda: bool(stop.name), here))
        here = (group[-1].x, group[-1].y)
        fails = 0 if outcomes[-1].succeeded else fails + 1
    return outcomes, f"interrupted_{stop.name}" if stop.name else ""


# ── Проход по периметру (#4607) ──────────────────────────────────────────────


def follow_path_client(n):
    """Клиент /follow_path контроллера Nav2 (до 60 с)."""
    cl = ActionClient(n, FollowPath, "/follow_path")
    assert cl.wait_for_server(timeout_sec=60), "нет /follow_path"
    return cl


def run_follow(n, cl, name, path, timeout, stop=None):
    """Путь ``path`` [(x, y, yaw рад)] целью FollowPath (DWB, проверка по costmap); вернуть (итог, эпизоды защиты)."""
    g = FollowPath.Goal()
    g.path = NavPath()
    g.path.header.frame_id = "map"
    g.path.poses = [_pose(x, y, math.degrees(t)) for x, y, t in path]
    g.controller_id = "FollowPath"
    g.goal_checker_id = "general_goal_checker"
    (x0, y0, _), (x1, y1, _) = path[0], path[-1]
    target = f"path[{len(path)}] ({x0:.3f},{y0:.3f})->({x1:.3f},{y1:.3f})"
    return route._send(n, cl, name, g, target, (x1, y1), timeout, stop)


def path_length(path):
    return sum(math.dist(a[:2], b[:2]) for a, b in zip(path, path[1:]))


def _strip(n, nav, follow, name, path, timeout, stop_fn):
    """Один нырок: подъезд к началу (место разворота), затем путь. Подъезд не удался — путь NOT_SENT."""
    x, y, t = path[0]
    raw, em = run_goal(n, nav, f"{name}a", round(x, 3), round(y, 3), round(math.degrees(t), 1), timeout, stop=stop_fn)
    approach = GoalOutcome.from_route(f"{name}a", raw, em)
    if not approach.succeeded:
        return approach, GoalOutcome.not_sent(f"{name}f")
    raw, em = run_follow(n, follow, f"{name}f", path, timeout + path_length(path) / SPEED_MPS, stop=stop_fn)
    return approach, GoalOutcome.from_route(f"{name}f", raw, em)


def drive_perimeter(n, nav, follow, pplan, timeout, max_fail, stop=None):
    """Нырки по порядку плана; вернуть (исходы, причина остановки). Неудачный нырок (подъезд или путь) идёт в счёт
    ``max_fail`` подряд, как цель змейки."""
    stop = stop or StopSignal()
    strips = pplan.driven()
    outcomes, fails = [], 0
    for idx, (k, path) in enumerate(strips):
        why = _stop_reason(stop, fails, max_fail)
        if why:
            return outcomes + _not_sent_strips(pplan, idx), why
        s = pplan.strips[k]
        name = f"P{idx:04d}"
        print(
            f"{ts()} PERIM_STRIP {name} axis={s.axis} side={s.side:+d} path_m={s.length_m:.1f} "
            f"dip_m={s.dip_max_m:.2f} gain_est_m2={s.gain_m2:.2f}",
            flush=True,
        )
        pair = _strip(n, nav, follow, name, path, timeout, lambda: bool(stop.name))
        outcomes += pair
        fails = 0 if pair[1].succeeded else fails + 1
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
    line = f"{ts()} {run_summary(outcomes, cov, n.simt() - sim0, stop, motion_stats(n.track))} image={img}"
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


def _clients(n, args):
    """(NavigateToPose, клиент цели змейки, FollowPath); None — где не нужен."""
    if args.plan_only:
        return None, None, None
    nav = wait_ready(n)
    lane = nav if args.per_pose or args.perimeter_only else through_poses_client(n)
    follow = follow_path_client(n) if args.perimeter or args.perimeter_only else None
    return nav, lane, follow


def _plan_perimeter(args, space, plan):
    """План периметра, perimeter.json и строка PERIMETER_PLAN; без ``--perimeter`` — None."""
    if not (args.perimeter or args.perimeter_only):
        return None
    pplan = plan_perimeter(space, plan, wall_offset_m=args.wall_offset)
    (args.out / "perimeter.json").write_text(json.dumps(strips_to_json(pplan)), encoding="utf-8")
    print(f"{ts()} {perimeter_summary(pplan, space, plan)}", flush=True)
    return pplan


def _not_sent_strips(pplan, start=0):
    return [GoalOutcome.not_sent(f"P{j:04d}{g}") for j in range(start, len(pplan.order)) for g in "af"]


def _drive_all(n, clients, args, poses, pplan, stop):
    """Змейка, затем периметр; вернуть (исходы змейки, исходы периметра, причина остановки, точек трека до периметра).

    Змейка встала (неудачи подряд, сигнал) — нырки не едут, все NOT_SENT."""
    nav, lane, follow = clients
    snake, perim, why = [], [], ""
    if not args.perimeter_only:
        snake, why = drive(n, lane, poses, args.timeout, args.max_fail, stop, lanes=not args.per_pose)
    k0 = len(n.track)
    if pplan is not None:
        if why:
            perim = _not_sent_strips(pplan)
        else:
            perim, why = drive_perimeter(n, nav, follow, pplan, args.timeout, args.max_fail, stop)
    return snake, perim, why, k0


def _report_perimeter(n, space, perim, k0):
    """Строка PERIMETER_DONE: исходы нырков и покрытие по треку истинной позы до периметра и после."""
    before = coverage_result(space, covered_mask(space, n.track[:k0]))
    after = coverage_result(space, covered_mask(space, n.track))
    line = f"{ts()} {perimeter_done(perim, before, after, reverse_m(n.track[k0:]))}"
    print(line, flush=True)
    return line


def main(argv=None):
    args = parse_args(argv)
    _init_ros()
    stop = StopSignal().install()
    n = CoverageMon()
    clients = _clients(n, args)
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
    pplan = _plan_perimeter(args, space, plan)
    if args.plan_only:
        rclpy.shutdown()
        return 0
    poses = plan.poses[: args.max_poses] if args.max_poses else plan.poses
    n.track.clear()
    sim0 = n.simt()
    snake, perim, why, k0 = _drive_all(n, clients, args, poses, pplan, stop)
    if stop.name:
        halt(n)
    if pplan is not None:
        _report_perimeter(n, space, perim, k0)
    finish(args.out, n, grid, space, snake + perim, why, sim0)
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
