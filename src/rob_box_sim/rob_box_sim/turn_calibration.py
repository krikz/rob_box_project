"""Замер верности привода в Gazebo: разворот и прямая по команде против правды и одометрии (#4495).

Колёсная одометрия робота ``open_loop`` (интегрирует команду) с ``wheel_separation: 1.11`` —
калибровка скид-стира на полу мастерской (``controller_manager.yaml``). Сим должен на ту же
команду поворачиваться так же, как живой робот: разворот, который одометрия считает за 360°,
в правде Gazebo тоже 360° (±5 %), а 1 м по одометрии — 1 м по правде (±3 %).

Каждая проба: робот стоит, команда в ``/diff_drive_controller/cmd_vel_unstamped`` до тех пор, пока
колёсная одометрия (``/diff_drive_controller/odom``) не насчитает цель (угол или путь), потом
ноль и пауза на остановку. Сравниваются приращения одометрии и правды
(``/sim/ground_truth/odom``) от покоя до покоя.

Центр вращения (#4377, #4475). Разворот на месте у скид-стира идёт вокруг мгновенного центра, а
не обязательно вокруг ``base_footprint``. Колёсная одометрия ``open_loop`` считает, что
``base_footprint`` стоит, и при сдвинутом центре ошибается по позиции. В пробе разворота
:func:`rotation_center` находит центр по правде Gazebo в системе корпуса.
:func:`odom_position_error` даёт наибольшее расхождение смещения одометрии и правды. HIL 09.11 до
правки сима: центр (−0.144, 0.000) — задняя ось, ошибка одометрии 0.287 м за оборот
(docs/research/2026-11-05-nav2-hil-tuning.md §10).

Решение — чистые функции (:func:`turn_verdict`, :func:`line_verdict`, :class:`YawAccumulator`,
:func:`rotation_center`, :func:`odom_position_error`), тест — tests/unit/sim/test_sim_turn_calibration.py.
Обёртка с rclpy — :func:`main`, запуск в контейнере gazebo (сим или HIL), моторы робота команду
не получают: в HIL ``ros2-control`` робота остановлен, единственный контроллер — Gazebo (``rob_box_sim.hil``).

  python3 -m rob_box_sim.turn_calibration                      # набор по умолчанию
  python3 -m rob_box_sim.turn_calibration --turn 0.6 --turn -0.6 --line 0.3
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from dataclasses import asdict, dataclass, replace

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


Pose = tuple[float, float, float]  # x, y, yaw

#: Пары отсчётов для центра вращения: поворот между ними не меньше этого (иначе система
#: (R_i − R_j) b = p_j − p_i плохо обусловлена) и не больше π.
CENTER_MIN_DYAW = 0.5


@dataclass(frozen=True)
class RotationCenter:
    """Мгновенный центр вращения в системе base_footprint (x вперёд, y влево), м."""

    x: float
    y: float
    spread: float  # СКО оценок по парам отсчётов, м
    pairs: int


def rotation_center(poses: list[Pose], min_dyaw: float = CENTER_MIN_DYAW) -> RotationCenter | None:
    """Точка корпуса, которая за разворот стоит на месте (правда Gazebo, поза — base_footprint).

    Для пары поз i, j точка корпуса b неподвижна, если p_i + R_i b = p_j + R_j b, отсюда
    b = (R_i − R_j)⁻¹ (p_j − p_i). Пара — первая j после i с поворотом от ``min_dyaw`` до π.
    Нет таких пар (поворот мал) — ``None``.
    """
    if len(poses) < 2:
        return None
    unwrapped = [poses[0][2]]
    for prev, cur in zip(poses, poses[1:]):
        unwrapped.append(unwrapped[-1] + wrap_pi(cur[2] - prev[2]))
    xs: list[float] = []
    ys: list[float] = []
    j = 0
    for i, (xi, yi, _) in enumerate(poses):
        j = max(j, i + 1)
        while j < len(poses) and abs(unwrapped[j] - unwrapped[i]) < min_dyaw:
            j += 1
        if j >= len(poses):
            break
        if abs(unwrapped[j] - unwrapped[i]) > math.pi:
            continue
        ci, si = math.cos(unwrapped[i]), math.sin(unwrapped[i])
        cj, sj = math.cos(unwrapped[j]), math.sin(unwrapped[j])
        a11, a12, a21, a22 = ci - cj, -(si - sj), si - sj, ci - cj  # R_i − R_j
        det = a11 * a22 - a12 * a21
        dx, dy = poses[j][0] - xi, poses[j][1] - yi
        xs.append((a22 * dx - a12 * dy) / det)
        ys.append((-a21 * dx + a11 * dy) / det)
    if not xs:
        return None
    mx, my = sum(xs) / len(xs), sum(ys) / len(ys)
    spread = math.sqrt(sum((x - mx) ** 2 + (y - my) ** 2 for x, y in zip(xs, ys)) / len(xs))
    return RotationCenter(mx, my, spread, len(xs))


def odom_position_error(samples: list[tuple[Pose, Pose]]) -> float:
    """Наибольшее расхождение смещения одометрии и правды от первой пары, м.

    ``samples`` — пары (правда, одометрия) в одни моменты. Каждое смещение берётся в системе
    своей начальной позы (у правды — мир Gazebo, у одометрии — odom), поэтому начала отсчёта
    не обязаны совпадать.
    """
    if not samples:
        return 0.0

    def local(p: Pose, p0: Pose) -> tuple[float, float]:
        c, s = math.cos(-p0[2]), math.sin(-p0[2])
        dx, dy = p[0] - p0[0], p[1] - p0[1]
        return c * dx - s * dy, s * dx + c * dy

    t0, o0 = samples[0]
    worst = 0.0
    for t, o in samples:
        (tx, ty), (ox, oy) = local(t, t0), local(o, o0)
        worst = max(worst, math.hypot(tx - ox, ty - oy))
    return worst


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
    # Разворот: центр вращения в системе base_footprint и ошибка одометрии по позиции (#4377).
    center_x: float | None = None
    center_y: float | None = None
    center_spread: float | None = None
    odom_pos_err: float | None = None


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
    row = (
        f"{v.kind:4s} cmd {v.cmd:+.2f}  odom {v.odom * k:+8.2f}{unit}  truth {v.truth * k:+8.2f}{unit}  "
        f"truth/odom {v.ratio:.3f}  ошибка правды {v.truth_err * 100:+.1f} %  "
        f"{'OK' if v.ok else 'FAIL'}"
    )
    if v.center_x is not None:
        row += (
            f"  центр вращения ({v.center_x:+.3f}, {v.center_y:+.3f}) м ±{v.center_spread:.3f}"
            f"  ошибка одометрии по позиции {v.odom_pos_err:.3f} м"
        )
    return row


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
        self.pose = {"odom": None, "truth": None}
        #: (правда, одометрия) на каждый отсчёт правды — для центра вращения и ошибки по позиции.
        self.samples: list[tuple[Pose, Pose]] = []

    def _on(self, key: str, msg) -> None:
        p = msg.pose.pose
        self.last[key] = (p.position.x, p.position.y)
        self.pose[key] = (p.position.x, p.position.y, _yaw(p.orientation))
        self.yaw[key].add(self.pose[key][2])
        if self.origin[key] is None:
            self.origin[key] = self.last[key]
        if key == "truth" and self.pose["odom"] is not None:
            self.samples.append((self.pose["truth"], self.pose["odom"]))

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
        v = turn_verdict(cmd, probe.turned("odom"), probe.turned("truth"), args.turn_target)
        center = rotation_center([t for t, _ in probe.samples])
        return replace(
            v,
            center_x=center.x if center else None,
            center_y=center.y if center else None,
            center_spread=center.spread if center else None,
            odom_pos_err=odom_position_error(probe.samples),
        )
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
