#!/usr/bin/env python3
"""Гейт twist_mux до первой цели HIL (ADR-0148: вердикт строит код, не текст оператора).

Поломка (2026-11-20, два обрыва после рестарта Gazebo): twist_mux штампует вход своими
часами (use_sim_time=true) и не просрочивает сообщение, пока sim-время позади штампа.
Скачок /clock назад оставляет старый вход высокого приоритета «свежим», и он глотает
/cmd_vel (priority navigation): на выходе нули или тишина. В простое исправный mux и
mux с уже протухшей защёлкой выглядят одинаково (все входы masked, priority 0). Их
различает только инъекция.

Проверен на записях с katana (/tmp/m4_depot_20261120/): PASS на захватах 15:57, 16:01
и 16:47, FAIL на захвате обрыва. На живом стенде не проверялся — инъекцию vx=0.1
туда не слали.

Живой прогон (только HIL, opt-in) шлёт 1.5 с × 10 Гц vx=0.1 в /cmd_vel и пишет
capture.txt. Код выхода: 0 PASS, маршрут можно слать; 1 FAIL и 2 «проверка не
выполнилась» — маршрут не слать, capture.txt уже на диске.

    python3 mux_gate.py capture <файл>     # разобрать уже снятый capture.txt, ROS не нужен
    python3 mux_gate.py run <каталог>      # живая инъекция; без sentinel HIL не стартует
"""

from __future__ import annotations

import ast
import os
import re
import sys
from typing import Dict, List, Optional, Sequence, Tuple

VX = 0.1
WZ = 0.0
HZ = 10.0
SECONDS = 1.5
WARM_S = 2.0
DRAIN_S = 1.2
# Насколько выход должен совпасть с тем, что вкололи.
MATCH_TOL = 1e-3
# Столько вколотых сообщений обязаны вернуться на выходе.
MIN_FORWARDED = 10
# Входы приоритетнее navigation: любой из них unmasked во время инъекции — защёлка.
ABOVE_NAV = ("emergency_stop", "joystick", "quest", "web_ui", "voice", "yaw_at_goal")
LOCKS = ("joystick_lock", "teleop_lock")
TWIST_INPUTS = (
    "cmd_vel",
    "emergency",
    "joy",
    "quest",
    "voice",
    "web",
    "yaw",
    "mux_out",
)

# Живой прогон публикует скорость. Без этих признаков среды он не стартует:
# файл пишет hil.sh on, имя контейнера — префикс сима, не боевого стека.
HIL_LOCK_FILE = os.environ.get("HIL_LOCK_FILE", "/tmp/hil.lock")
HIL_CONTAINER = os.environ.get("HIL_MUX_GATE_CONTAINER", "rob-box-hil-gazebo")
MUX_STATUS_NAME = "twist_mux: Twist mux status"

PASS, FAIL = "PASS", "FAIL"
RC_PASS, RC_FAIL, RC_UNRUN = 0, 1, 2

_SENT_RE = re.compile(r"^sent (\d+) vx ([0-9.eE+-]+) wz ([0-9.eE+-]+)")
_CLOCK_RE = re.compile(
    r"^clock count (\d+)(?: first ([0-9.eE+-]+) last ([0-9.eE+-]+))?"
)
_COUNT_RE = re.compile(r"^(\w+) count (\d+) nonzero (\d+) sample (\[.*\])$")
# Ключ диагностики может содержать пробелы: «velocity topics.yaw_at_goal».
_DIAG_KV_RE = re.compile(r"^  (.+?) = (.*)$")
_UNMASKED_RE = re.compile(r"velocity topics\.(\w+) = +\S*unmasked")


def hil_sentinel_ok(lock_text: str, container: str) -> bool:
    """Живой прогон разрешён только в контейнере HIL при живом замке hil.sh.

    Пустой замок, чужой контейнер и имя без «hil» — отказ. Боевой стек сюда не входит.
    """
    if "hil" not in (container or ""):
        return False
    text = lock_text or ""
    return "HIL on since" in text and "sandbox=" in text


def unmasked_above_nav(sample: Sequence[Tuple[str, str]]) -> List[str]:
    """Имена входов приоритетнее navigation, которые диагностика назвала unmasked."""
    found = []
    for key, value in sample:
        for name in ABOVE_NAV:
            if key.endswith(name) and "unmasked" in value:
                found.append(name)
    return found


def _clock_advanced(count: int, first: Optional[float], last: Optional[float]) -> bool:
    return count >= 2 and first is not None and last is not None and last > first


def _latch_reason(
    prefix: str, diag: Sequence[Sequence[Tuple[str, str]]]
) -> Optional[str]:
    """Имена входов приоритетнее navigation, которые диагностика назвала unmasked, или None."""
    latched = sorted({name for sample in diag for name in unmasked_above_nav(sample)})
    if not latched:
        return None
    return prefix + ",".join(latched)


def _forward_reason(
    sent: int, vx: float, wz: float, mux_out: Sequence[Tuple[float, float]]
) -> Optional[str]:
    """None, если выход повторил вколотое не меньше MIN_FORWARDED раз."""
    forwarded = [
        m
        for m in mux_out
        if abs(m[0] - vx) <= MATCH_TOL and abs(m[1] - wz) <= MATCH_TOL
    ]
    if len(forwarded) >= MIN_FORWARDED:
        return None
    zeros = [m for m in mux_out if abs(m[0]) <= MATCH_TOL and abs(m[1]) <= MATCH_TOL]
    return (
        f"NOT_FORWARDED: injected {sent} x ({vx},{wz}), mux output {len(mux_out)} msgs, "
        f"matching {len(forwarded)}, zeros {len(zeros)} (need >= {MIN_FORWARDED} matching)"
    )


def _collect_reasons(
    sent: int,
    vx: float,
    wz: float,
    clock: Sequence[float],
    mux_out: Sequence[Tuple[float, float]],
    diag: Sequence[Sequence[Tuple[str, str]]],
    locks: Dict[str, Sequence[bool]],
    idle_diag: Optional[Sequence[Sequence[Tuple[str, str]]]],
) -> List[str]:
    """Причины FAIL. Пустой список — PASS. Каждая проверка — отдельная функция, здесь только сбор."""
    reasons = []
    if not _clock_advanced(
        len(clock), clock[0] if clock else None, clock[-1] if clock else None
    ):
        reasons.append("CLOCK_FROZEN: /clock did not advance across the inject")
    if idle_diag is not None:
        reasons += _one(
            _latch_reason(
                "IDLE_LATCH: priority>10 input unmasked before inject: ", idle_diag
            )
        )
    reasons += _one(_forward_reason(sent, vx, wz, mux_out))
    reasons += _one(
        _latch_reason("LATCH: priority>10 input unmasked during inject: ", diag)
    )
    reasons += _lock_reasons(locks)
    return reasons


def _one(reason: Optional[str]) -> List[str]:
    return [reason] if reason else []


def _lock_reasons(locks: Dict[str, Sequence[bool]]) -> List[str]:
    reasons = []
    for lock in LOCKS:
        samples = locks.get(lock, ())
        if any(samples):
            reasons.append(
                f"LOCK: {lock} data=true in {sum(samples)} of {len(samples)} samples"
            )
    return reasons


def verdict_of(
    sent: int,
    vx: float,
    wz: float,
    clock: Sequence[float],
    mux_out: Sequence[Tuple[float, float]],
    diag: Sequence[Sequence[Tuple[str, str]]],
    locks: Dict[str, Sequence[bool]],
    idle_diag: Optional[Sequence[Sequence[Tuple[str, str]]]] = None,
) -> Tuple[str, List[str]]:
    """PASS/FAIL по сырым замерам. Пустой список причин — PASS.

    ``idle_diag`` — диагностика до инъекции (живой прогон). В записанном capture её нет,
    защёлка там видна только в общем блоке DIAG.
    """
    reasons = _collect_reasons(sent, vx, wz, clock, mux_out, diag, locks, idle_diag)
    return (PASS if not reasons else FAIL), reasons


def parse_capture(text: str) -> Tuple[str, List[str]]:
    """Вердикт по уже снятому capture.txt. ROS не нужен.

    Формат — тот, что пишет живой прогон и пробы 2026-11-20: шапка ``sent``,
    ``clock count``, строки ``<имя> count N nonzero M sample [...]`` и блоки DIAG.
    Число совпавших сообщений берётся из ``nonzero``, когда sample однородный
    (в записях он такой); иначе — из самого sample, и это честно меньше порога.
    """
    sent, vx, wz = 0, VX, WZ
    clock_n, clock_first, clock_last = 0, None, None
    mux_n, mux_nz, mux_sample = 0, 0, []
    locks: Dict[str, List[bool]] = {name: [] for name in LOCKS}
    diag: List[List[Tuple[str, str]]] = []
    current: Optional[List[Tuple[str, str]]] = None

    for raw in text.splitlines():
        line = raw.rstrip()
        sent_m = _SENT_RE.match(line)
        if sent_m:
            sent, vx, wz = (
                int(sent_m.group(1)),
                float(sent_m.group(2)),
                float(sent_m.group(3)),
            )
            continue
        clock_m = _CLOCK_RE.match(line)
        if clock_m:
            clock_n = int(clock_m.group(1))
            clock_first = float(clock_m.group(2)) if clock_m.group(2) else None
            clock_last = float(clock_m.group(3)) if clock_m.group(3) else None
            continue
        if line == "DIAG" or line.startswith("DIAG "):
            current = []
            diag.append(current)
            continue
        kv_m = _DIAG_KV_RE.match(line)
        if kv_m and current is not None:
            current.append((kv_m.group(1), kv_m.group(2).strip()))
            continue
        count_m = _COUNT_RE.match(line)
        if not count_m:
            continue
        name, count, nonzero, sample_src = (
            count_m.group(1),
            int(count_m.group(2)),
            int(count_m.group(3)),
            count_m.group(4),
        )
        if name == "mux_out":
            mux_n, mux_nz, mux_sample = count, nonzero, _literal_pairs(sample_src)
        elif name in locks:
            locks[name] = [True] * nonzero + [False] * (count - nonzero)

    clock = _clock_span(clock_n, clock_first, clock_last)
    mux_out = _mux_pairs(mux_n, mux_nz, mux_sample, vx, wz)
    return verdict_of(sent, vx, wz, clock, mux_out, diag, locks)


def _literal_pairs(sample_src: str) -> List[Tuple[float, float]]:
    """Пары (vx, wz) из ``sample [(0.1, 0.0), ...]``. Кривой sample — пустой список."""
    try:
        parsed = ast.literal_eval(sample_src)
    except (SyntaxError, ValueError):
        return []
    pairs = []
    for item in parsed if isinstance(parsed, list) else []:
        if isinstance(item, (list, tuple)) and len(item) == 2:
            pairs.append((float(item[0]), float(item[1])))
    return pairs


def _clock_span(
    count: int, first: Optional[float], last: Optional[float]
) -> List[float]:
    """Две точки, если часы в шапке шли вперёд; иначе как записано (0 или 1 точка)."""
    if count >= 2 and first is not None and last is not None:
        return [first, last]
    if count == 1 and first is not None:
        return [first]
    return []


def _mux_pairs(
    count: int,
    nonzero: int,
    sample: Sequence[Tuple[float, float]],
    vx: float,
    wz: float,
) -> List[Tuple[float, float]]:
    """Восстановить выход mux из шапки capture.

    Записи хранят sample из трёх элементов. Если он однородный и совпадает со вколотым,
    все ``nonzero`` считаются совпавшими — так устроены пробы 15:57, 16:01 и 16:47.
    Иначе совпавшими считаются только пары из sample: порог честно не достигнут.
    """
    if not sample:
        return [(0.0, 0.0)] * count
    head = sample[0]
    uniform = all(
        abs(m[0] - head[0]) <= MATCH_TOL and abs(m[1] - head[1]) <= MATCH_TOL
        for m in sample
    )
    matches = abs(head[0] - vx) <= MATCH_TOL and abs(head[1] - wz) <= MATCH_TOL
    if uniform and matches:
        rest = max(count - nonzero, 0)
        return [head] * nonzero + [(0.0, 0.0)] * rest
    return list(sample)


def _format_capture(seen: Dict[str, list], sent: int) -> List[str]:
    """Шапка capture.txt из сырых замеров живого прогона. DIAG пишет вызывающий."""
    lines = [f"sent {sent} vx {VX} wz {WZ} seconds {SECONDS}"]
    clock = seen["clock"]
    if clock:
        lines.append(
            f"clock count {len(clock)} first {clock[0]:.3f} last {clock[-1]:.3f}"
        )
    else:
        lines.append("clock count 0")
    for name in TWIST_INPUTS:
        msgs = seen[name]
        nonzero = sum(1 for m in msgs if abs(m[0]) > MATCH_TOL or abs(m[1]) > MATCH_TOL)
        lines.append(f"{name} count {len(msgs)} nonzero {nonzero} sample {msgs[:3]}")
    for lock in LOCKS:
        lines.append(
            f"{lock} count {len(seen[lock])} nonzero {sum(seen[lock])} sample {seen[lock][:3]}"
        )
    if seen["gt"]:
        lines.append(f"gt first {seen['gt'][0]} last {seen['gt'][-1]}")
    lines.append(f"diag samples {len(seen['diag'])}")
    return lines


def _append_diag(lines: List[str], diag: Sequence[Sequence[Tuple[str, str]]]) -> None:
    for sample in diag:
        lines.append("DIAG")
        for key, value in sample:
            lines.append(f"  {key} = {value.strip()}")


def _write_capture(out_dir: str, lines: List[str], raw: str) -> str:
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, "capture.txt")
    with open(path, "w", encoding="utf-8") as handle:
        handle.write("\n".join(lines) + "\n")
    with open(os.path.join(out_dir, "raw.txt"), "w", encoding="utf-8") as handle:
        handle.write(raw)
    return path


def capture_main(path: str) -> int:
    """Разобрать capture.txt и напечатать вердикт. 2 — файла нет."""
    if not os.path.isfile(path):
        print(f"NO_CAPTURE: нет {path}", flush=True)
        return RC_UNRUN
    with open(path, encoding="utf-8") as handle:
        verdict, reasons = parse_capture(handle.read())
    print("VERDICT " + verdict, flush=True)
    for reason in reasons:
        print("REASON " + reason, flush=True)
    return RC_PASS if verdict == PASS else RC_FAIL


def _refuse(reason: str) -> int:
    print("VERDICT FAIL", flush=True)
    print("REASON " + reason, flush=True)
    return RC_UNRUN


def _read_lock() -> str:
    try:
        with open(HIL_LOCK_FILE, encoding="utf-8") as handle:
            return handle.read()
    except OSError:
        return ""


def run_main(out_dir: str) -> int:
    """Живая инъекция. Без sentinel HIL скорость не публикуется."""
    container = os.environ.get("HOSTNAME", "")
    if not hil_sentinel_ok(_read_lock(), container):
        return _refuse(
            "NOT_HIL: нет замка HIL "
            f"({HIL_LOCK_FILE}) или контейнер '{container}' не {HIL_CONTAINER}; скорость не публикую"
        )
    try:
        return _run_inject(out_dir)
    except (
        Exception
    ) as exc:  # noqa: BLE001 — сбой проверки не должен выглядеть как PASS
        _write_capture(
            out_dir, [f"sent 0 vx {VX} wz {WZ} seconds {SECONDS}", f"ERROR {exc!r}"], ""
        )
        return _refuse(f"UNRUN: проверка не выполнилась: {exc!r}")


def _topic_table():
    """Имя замера → (топик, тип). Импорт ROS только здесь: разбор capture его не требует."""
    from geometry_msgs.msg import Twist
    from std_msgs.msg import Bool

    return {
        "cmd_vel": ("/cmd_vel", Twist),
        "emergency": ("/cmd_vel_emergency", Twist),
        "joy": ("/cmd_vel_joy", Twist),
        "quest": ("/cmd_vel_quest", Twist),
        "voice": ("/cmd_vel_voice", Twist),
        "web": ("/cmd_vel_web", Twist),
        "yaw": ("/cmd_vel_yaw", Twist),
        "mux_out": ("/diff_drive_controller/cmd_vel_unstamped", Twist),
        "joystick_lock": ("/joystick_lock", Bool),
        "teleop_lock": ("/teleop_lock", Bool),
    }


def _subscribe(node, seen: Dict[str, list]) -> None:
    """Все входы, замки, выход, /clock, правда Gazebo и диагностика mux."""
    from diagnostic_msgs.msg import DiagnosticArray
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock

    def on_twist(name):
        return lambda msg: seen[name].append((msg.linear.x, msg.angular.z))

    for name, (topic, typ) in _topic_table().items():
        if typ is Twist:
            node.create_subscription(typ, topic, on_twist(name), 10)
        else:
            node.create_subscription(
                typ, topic, lambda msg, n=name: seen[n].append(bool(msg.data)), 10
            )
    node.create_subscription(
        Clock,
        "/clock",
        lambda msg: seen["clock"].append(msg.clock.sec + msg.clock.nanosec * 1e-9),
        10,
    )
    node.create_subscription(
        Odometry,
        "/sim/ground_truth/odom",
        lambda msg: seen["gt"].append(
            (msg.pose.pose.position.x, msg.pose.pose.position.y)
        ),
        10,
    )

    def on_diag(msg):
        for status in msg.status:
            if status.name == MUX_STATUS_NAME:
                seen["diag"].append(
                    [(value.key, value.value) for value in status.values]
                )

    node.create_subscription(DiagnosticArray, "/diagnostics", on_diag, 10)


def _spin_for(node, seconds: float, step: float) -> None:
    import time

    import rclpy

    end = time.time() + seconds
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=step)


def _publish_burst(node, pub) -> int:
    """1.5 с × 10 Гц vx=0.1 в /cmd_vel. Вернуть, сколько сообщений ушло."""
    import time

    import rclpy
    from geometry_msgs.msg import Twist

    msg = Twist()
    msg.linear.x, msg.angular.z = VX, WZ
    sent, nxt, end = 0, 0.0, time.time() + SECONDS
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.02)
        if time.time() >= nxt:
            pub.publish(msg)
            sent += 1
            nxt = time.time() + 1.0 / HZ
    return sent


def _run_inject(out_dir: str) -> int:
    """Подписки, 1.5 с vx=0.1 в /cmd_vel, capture.txt, вердикт. Вызывается только после sentinel."""
    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.node import Node

    rclpy.init()
    node = Node("mux_gate")
    seen: Dict[str, list] = {
        name: [] for name in list(_topic_table()) + ["clock", "diag", "gt"]
    }
    _subscribe(node, seen)
    pub = node.create_publisher(Twist, "/cmd_vel", 10)
    _spin_for(node, WARM_S, 0.1)
    idle_diag = list(seen["diag"])
    sent = _publish_burst(node, pub)
    _spin_for(node, DRAIN_S, 0.1)
    code = _finish(out_dir, seen, sent, idle_diag)
    node.destroy_node()
    rclpy.shutdown()
    return code


def _finish(out_dir: str, seen: Dict[str, list], sent: int, idle_diag: list) -> int:
    """Вердикт, capture.txt, печать. Код выхода — 0 PASS, 1 FAIL."""
    verdict, reasons = verdict_of(
        sent,
        VX,
        WZ,
        seen["clock"],
        seen["mux_out"],
        seen["diag"],
        seen,
        idle_diag=idle_diag,
    )
    lines = _format_capture(seen, sent)
    _append_diag(lines, seen["diag"])
    lines.append("VERDICT " + verdict)
    for reason in reasons:
        lines.append("REASON " + reason)
    path = _write_capture(out_dir, lines, repr({"sent": sent, "seen": seen}) + "\n")
    for line in lines:
        if line.startswith("DIAG"):
            break
        print(line, flush=True)
    print("CAPTURE " + path, flush=True)
    print("VERDICT " + verdict, flush=True)
    for reason in reasons:
        print("REASON " + reason, flush=True)
    return RC_PASS if verdict == PASS else RC_FAIL


def main(argv: Sequence[str]) -> int:
    if len(argv) != 3 or argv[1] not in ("capture", "run"):
        print("usage: mux_gate.py capture <файл> | run <каталог>", file=sys.stderr)
        return RC_UNRUN
    if argv[1] == "capture":
        return capture_main(argv[2])
    return run_main(argv[2])


if __name__ == "__main__":
    sys.exit(main(sys.argv))
