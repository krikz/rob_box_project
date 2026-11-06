"""Замер верности привода в Gazebo: разворот и прямая по команде против правды и одометрии (#4495).

Колёсная одометрия робота ``open_loop`` (интегрирует команду) с ``wheel_separation: 1.11`` —
калибровка скид-стира на полу мастерской (``controller_manager.yaml``). Сим должен на ту же
команду поворачиваться так же, как живой робот: разворот, который одометрия считает за 360°,
в правде Gazebo тоже 360° (±5 %), а 1 м по одометрии — 1 м по правде (±3 %).

Каждая проба: робот стоит, команда в ``/diff_drive_controller/cmd_vel_unstamped`` до тех пор, пока
колёсная одометрия (``/diff_drive_controller/odom``) не насчитает цель (угол или путь), потом
ноль и пауза на остановку. Сравниваются приращения одометрии и правды
(``/sim/ground_truth/odom``) от покоя до покоя.

Решение — чистые функции (:func:`turn_verdict`, :func:`line_verdict`, :class:`YawAccumulator`),
тест — tests/unit/sim/test_sim_turn_calibration.py. Обёртка с rclpy — :func:`main`, запуск в
контейнере gazebo (сим или HIL), моторы робота команду не получают: в HIL ``ros2-control``
робота остановлен, единственный контроллер — Gazebo (``rob_box_sim.hil``).

  python3 -m rob_box_sim.turn_calibration                      # набор по умолчанию
  python3 -m rob_box_sim.turn_calibration --turn 0.6 --turn -0.6 --line 0.3
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from dataclasses import asdict, dataclass

CMD_TOPIC = "/diff_drive_controller/cmd_vel_unstamped"
ODOM_TOPIC = "/diff_drive_controller/odom"
TRUTH_TOPIC = "/sim/ground_truth/odom"

TURN_TARGET_RAD = 2 * math.pi
LINE_TARGET_M = 1.0
TURN_TOL = 0.05
LINE_TOL = 0.03

DEFAULT_TURNS = (0.3, -0.3, 0.6, -0.6, 1.0, -1.0)
DEFAULT_LINES = (0.2, 0.5)


def wrap_pi(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


class YawAccumulator:
    """Сумма поворота по ряду курсов в (−π, π]: шаги между соседними отсчётами разворачиваются."""

    def __init__(self) -> None:
        self.total = 0.0
        self._last: float | None = None

    def add(self, yaw: float) -> float:
        if self._last is not None:
            self.total += wrap_pi(yaw - self._last)
        self._last = yaw
        return self.total


@dataclass(frozen=True)
class Verdict:
    kind: str
    cmd: float
    target: float
    odom: float
    truth: float
    ratio: float
    truth_err: float
    odom_err: float
    ok: bool


def _verdict(kind: str, cmd: float, target: float, odom: float, truth: float, tol: float) -> Verdict:
    ratio = truth / odom if odom else float("inf")
    truth_err = abs(truth) / target - 1.0
    odom_err = abs(odom) / target - 1.0
    ok = abs(truth_err) <= tol and abs(odom_err) <= tol
    return Verdict(kind, cmd, target, odom, truth, ratio, truth_err, odom_err, ok)


def turn_verdict(cmd: float, odom_rad: float, truth_rad: float, target: float = TURN_TARGET_RAD) -> Verdict:
    """Разворот: и одометрия, и правда в пределах ±5 % от цели (знак не важен, важна величина)."""
    return _verdict("turn", cmd, target, odom_rad, truth_rad, TURN_TOL)


def line_verdict(cmd: float, odom_m: float, truth_m: float, target: float = LINE_TARGET_M) -> Verdict:
    """Прямая: и одометрия, и правда в пределах ±3 % от цели."""
    return _verdict("line", cmd, target, odom_m, truth_m, LINE_TOL)


def format_row(v: Verdict) -> str:
    unit = "°" if v.kind == "turn" else " м"
    k = math.degrees(1.0) if v.kind == "turn" else 1.0
    return (
        f"{v.kind:4s} cmd {v.cmd:+.2f}  odom {v.odom * k:+8.2f}{unit}  truth {v.truth * k:+8.2f}{unit}  "
        f"truth/odom {v.ratio:.3f}  ошибка правды {v.truth_err * 100:+.1f} %  "
        f"{'OK' if v.ok else 'FAIL'}"
    )


# ── rclpy-обёртка ─────────────────────────────────────────────────────────────


def _yaw(q) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class _Probe:
    """Подписки на одометрию и правду; накопители поворота и пути от сброса."""

    def __init__(self, node, odom_msg) -> None:
        self.node = node
        self.last = {"odom": None, "truth": None}
        self.reset()
        node.create_subscription(odom_msg, ODOM_TOPIC, lambda m: self._on("odom", m), 50)
        node.create_subscription(odom_msg, TRUTH_TOPIC, lambda m: self._on("truth", m), 50)

    def reset(self) -> None:
        self.yaw = {"odom": YawAccumulator(), "truth": YawAccumulator()}
        self.origin = {"odom": None, "truth": None}

    def _on(self, key: str, msg) -> None:
        p = msg.pose.pose
        self.last[key] = (p.position.x, p.position.y)
        self.yaw[key].add(_yaw(p.orientation))
        if self.origin[key] is None:
            self.origin[key] = self.last[key]

    def ready(self) -> bool:
        return all(v is not None for v in self.origin.values())

    def turned(self, key: str) -> float:
        return self.yaw[key].total

    def moved(self, key: str) -> float:
        (x0, y0), (x1, y1) = self.origin[key], self.last[key]
        return math.hypot(x1 - x0, y1 - y0)


def _spin_for(rclpy, node, seconds: float, pub=None, twist=None) -> None:
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        if pub is not None:
            pub.publish(twist)
        rclpy.spin_once(node, timeout_sec=0.05)


def _run_trial(rclpy, node, probe, pub, twist_cls, kind: str, cmd: float, args) -> Verdict:
    stop = twist_cls()
    _spin_for(rclpy, node, args.settle, pub, stop)
    probe.reset()
    while not probe.ready():
        _spin_for(rclpy, node, 0.1, pub, stop)
    move = twist_cls()
    if kind == "turn":
        move.angular.z = cmd
        done = lambda: abs(probe.turned("odom")) >= args.turn_target  # noqa: E731
    else:
        move.linear.x = cmd
        done = lambda: probe.moved("odom") >= args.line_target  # noqa: E731
    deadline = time.monotonic() + args.timeout
    while not done() and time.monotonic() < deadline:
        _spin_for(rclpy, node, 0.05, pub, move)
    _spin_for(rclpy, node, args.settle, pub, stop)
    if kind == "turn":
        return turn_verdict(cmd, probe.turned("odom"), probe.turned("truth"), args.turn_target)
    return line_verdict(cmd, probe.moved("odom"), probe.moved("truth"), args.line_target)


def _parse(argv):
    ap = argparse.ArgumentParser(description="Замер разворота и прямой в Gazebo: правда против одометрии")
    ap.add_argument("--turn", type=float, action="append", help="рад/с; можно несколько")
    ap.add_argument("--line", type=float, action="append", help="м/с; можно несколько")
    ap.add_argument("--turn-target", type=float, default=TURN_TARGET_RAD)
    ap.add_argument("--line-target", type=float, default=LINE_TARGET_M)
    ap.add_argument("--settle", type=float, default=2.0, help="пауза на остановку, с")
    ap.add_argument("--timeout", type=float, default=60.0, help="предел одной пробы, с")
    ap.add_argument("--json", action="store_true", help="строка JSON на пробу")
    return ap.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = _parse(argv)
    turns = args.turn if args.turn is not None else ([] if args.line else list(DEFAULT_TURNS))
    lines = args.line if args.line is not None else ([] if args.turn else list(DEFAULT_LINES))

    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.parameter import Parameter

    rclpy.init()
    node = rclpy.create_node(
        "sim_turn_calibration", parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)]
    )
    pub = node.create_publisher(Twist, CMD_TOPIC, 10)
    probe = _Probe(node, Odometry)
    results = []
    try:
        for kind, cmds in (("turn", turns), ("line", lines)):
            for cmd in cmds:
                v = _run_trial(rclpy, node, probe, pub, Twist, kind, cmd, args)
                results.append(v)
                print(json.dumps(asdict(v)) if args.json else format_row(v), flush=True)
    finally:
        _spin_for(rclpy, node, 0.5, pub, Twist())
        node.destroy_node()
        rclpy.shutdown()
    return 0 if results and all(v.ok for v in results) else 1


if __name__ == "__main__":
    sys.exit(main())
