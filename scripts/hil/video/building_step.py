#!/usr/bin/env python3
"""Шаг прогона «большая карта здания» (#4468) внутри rob-box-hil-gazebo: цели, телепорт, поворот, метки.

Запускает его ``scripts/hil/building/run_building.sh`` через ``building_remote.sh step``, по одному шагу на вызов;
между шагами оркестр перезапускает стек на Main Pi и зовёт тулы реестра на Sound Pi. Лог — построчно в stdout:
``GOAL_START/GOAL_END/TRACK/EM_*`` из ``route.py`` и ``EVENT <KIND> sim=…`` (формат —
``rob_box_sim.building_run.event_line``). Решений здесь нет: порядок, цели телепорта и вердикты считает
``rob_box_sim.building_run``, шаг только исполняет и пишет.

    building_step.py goto NAME [NAME ...]                 цели NavigateToPose по точкам ROUTE_GOALS_FILE
    building_step.py mark NAME [k=v ...]                  метка с sim time (SEG_START/SEG_END — name=, kind=)
    building_step.py teleport X Y YAW_DEG                 телепорт в Gazebo (set_pose), ждать скачка истины
    building_step.py probe [--turns 1] [--wz 0.3] [--settle 20]   поворот на месте через /cmd_vel_web и выдержка
    building_step.py kidnap NAME                          похищение из building_run.KIDNAPS
    building_step.py arrive NAME [--timeout 300]          ждать, пока истина у точки (голосовой сценарий)

Переменные: ROUTE_GOALS_FILE (/sim_worlds/building.goals.yaml), BUILDING_WORLD (building), BUILDING_MODEL (rob_box).
"""
import argparse
import datetime
import math
import os
import subprocess
import sys
import time

sys.path.insert(0, "/")  # route.py кладёт рядом runvideo/run_building (docker cp … /route.py)

import rclpy  # noqa: E402
import route  # noqa: E402
from geometry_msgs.msg import Twist  # noqa: E402

from rob_box_sim import building_run as br  # noqa: E402

GOALS_FILE = os.environ.get("ROUTE_GOALS_FILE", "/sim_worlds/building.goals.yaml")
WORLD = os.environ.get("BUILDING_WORLD", "building")
MODEL = os.environ.get("BUILDING_MODEL", "rob_box")


def ts():
    return datetime.datetime.utcnow().strftime("%H:%M:%S.%f")[:-3]


def emit(n, event, **kv):
    print(f"{ts()} {br.event_line(event, n.simt(), **kv)}", flush=True)


def spin_for(n, sim_s):
    t0 = n.simt()
    while n.simt() - t0 < sim_s:
        rclpy.spin_once(n, timeout_sec=0.05)


def goals():
    return {g[0]: g for g in route.load_goals(GOALS_FILE)}


def goto(n, cl, names, stop=None):
    """Цели по одной; вернуть исход последней (route.run_goal пишет GOAL_START/GOAL_END)."""
    by = goals()
    unknown = [x for x in names if x not in by]
    assert not unknown, f"нет точек {unknown} в {GOALS_FILE}"
    result = None
    for name in names:
        _, x, y, yaw = by[name]
        result, _ = route.run_goal(n, cl, name, x, y, yaw, stop=stop)
        spin_for(n, 1.5)
    return result


def teleport(n, x, y, yaw_deg, wait_wall_s=15.0):
    """set_pose в Gazebo; ok=1, если истина прыгнула ближе 0.3 м к цели за wait_wall_s."""
    before = n.gt
    req = br.set_pose_request(MODEL, x, y, math.radians(yaw_deg))
    cmd = [
        "ign", "service", "-s", f"/world/{WORLD}/set_pose", "--reqtype", "ignition.msgs.Pose",
        "--reptype", "ignition.msgs.Boolean", "--timeout", "5000", "--req", req,
    ]  # fmt: skip
    rc = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
    t_end = time.time() + wait_wall_s
    ok = 0
    while time.time() < t_end:
        rclpy.spin_once(n, timeout_sec=0.05)
        if n.gt and math.hypot(n.gt[0] - x, n.gt[1] - y) < 0.3:
            ok = 1
            break
    fx, fy = (before[0], before[1]) if before else (float("nan"), float("nan"))
    emit(n, "TELEPORT", ok=ok, rc=rc.returncode, x=float(x), y=float(y), yaw=float(yaw_deg), from_x=fx, from_y=fy)
    return ok


def probe(n, turns=1.0, wz=0.3, settle=20.0):
    """Поворот на месте (twist_mux, вход cmd_vel_web — как телеоп), затем выдержка: лидар видит всё вокруг."""
    pub = n.create_publisher(Twist, "/cmd_vel_web", 10)
    emit(n, "PROBE_START", turns=float(turns), wz=float(wz))
    dur, t0, nxt = abs(2 * math.pi * turns / wz), n.simt(), 0.0
    msg = Twist()
    while n.simt() - t0 < dur:
        if n.simt() >= nxt:
            msg.angular.z = math.copysign(abs(wz), wz)
            pub.publish(msg)
            nxt = n.simt() + 0.1
        rclpy.spin_once(n, timeout_sec=0.02)
    msg.angular.z = 0.0
    for _ in range(5):
        pub.publish(msg)
        rclpy.spin_once(n, timeout_sec=0.05)
    spin_for(n, settle)
    emit(n, "PROBE_END")


def kidnap(n, cl, name):
    k = {x.name: x for x in br.KIDNAPS}[name]
    goto(n, cl, [k.approach])
    t0 = n.simt()
    emit(n, "KIDNAP_ARMED", goal=k.goal, after_s=k.after_s)
    goto(n, cl, [k.goal], stop=lambda: n.simt() - t0 >= k.after_s)
    if k.similar:
        p = br.similar_target(n.gt[0], n.gt[1], math.radians(n.gt[2]), k.similar)
        x, y, yaw = p.x, p.y, math.degrees(p.yaw)
    else:
        x, y, yaw = k.to
    if not teleport(n, x, y, yaw):
        emit(n, "KIDNAP_MISSING")
        return
    probe(n)
    goto(n, cl, [k.goal])


def arrive(n, name, timeout):
    _, x, y, _ = goals()[name]
    t0 = n.simt()
    while n.simt() - t0 < timeout:
        rclpy.spin_once(n, timeout_sec=0.1)
        if n.gt and math.hypot(n.gt[0] - x, n.gt[1] - y) <= br.Criteria().arrive_m:
            spin_for(n, 3.0)
            emit(n, "ARRIVE", target=name, ok=1, waited_s=n.simt() - t0)
            return
    emit(n, "ARRIVE", target=name, ok=0, waited_s=n.simt() - t0)


def main():
    ap = argparse.ArgumentParser(description="шаг прогона здания (#4468)")
    ap.add_argument("cmd", choices=("goto", "mark", "teleport", "probe", "kidnap", "arrive"))
    ap.add_argument("args", nargs="*")
    ap.add_argument("--turns", type=float, default=1.0)
    ap.add_argument("--wz", type=float, default=0.3)
    ap.add_argument("--settle", type=float, default=20.0)
    ap.add_argument("--timeout", type=float, default=300.0)
    a = ap.parse_args()
    rclpy.init()
    n = route.Mon("shidi_building")
    if a.cmd in ("goto", "kidnap"):
        cl = route.wait_ready(n)
    else:
        spin_for_wall = time.time() + 15
        while (n.gt is None or n.simt() == 0) and time.time() < spin_for_wall:
            rclpy.spin_once(n, timeout_sec=0.1)
    if a.cmd == "goto":
        goto(n, cl, a.args)
    elif a.cmd == "mark":
        event = "MARK"
        kv = dict(x.split("=", 1) for x in a.args[1:])
        if a.args[0] in ("SEG_START", "SEG_END"):
            event = a.args[0]
        else:
            kv = {"name": a.args[0], **kv}
        emit(n, event, **kv)
    elif a.cmd == "teleport":
        teleport(n, float(a.args[0]), float(a.args[1]), float(a.args[2]))
    elif a.cmd == "probe":
        probe(n, a.turns, a.wz, a.settle)
    elif a.cmd == "kidnap":
        kidnap(n, cl, a.args[0])
    else:
        arrive(n, a.args[0], a.timeout)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
