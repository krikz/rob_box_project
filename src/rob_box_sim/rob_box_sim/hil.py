"""HIL: имена сим-узлов и сторож графа «ровно один /controller_manager, и он в Gazebo».

План: docs/research/2026-11-04-hil-robot-on-gazebo-data.md §5.1–§5.2, эпик #4371.

В HIL Gazebo на песочнице публикует под ID робота (``ROBOT_ID=RBXU100001``), и граф
песочницы и робота — один граф. Отсюда две опасности, и обе решает этот модуль:

1. ``gz_ros2_control`` берёт URDF у узла ``/robot_state_publisher``. В HIL под этим
   именем живёт RSP Main Pi с боевым URDF (железо VESC). Поэтому сим запускает свой RSP
   в пространстве ``/sim`` (:data:`SIM_RSP_NODE`), а его ``/tf`` и ``/tf_static``
   переназначает в ``/sim/...`` (:data:`SIM_TF_REMAPS`), чтобы TF сим-URDF не попал к роботу.
2. Если ``ros2-control`` на Main Pi жив, в графе два ``/controller_manager`` и два
   ``diff_drive_controller`` на одном топике команд: команды идут и в Gazebo, и на
   настоящие моторы. Сторож (:func:`precheck`, :func:`watch_verdict`) не даёт Gazebo
   стартовать при живом контроллере робота и гасит Gazebo, если второй появился потом
   (деплой, перезагрузка Main Pi).

Решение — в коде (ADR-0148): чистые функции над списком имён узлов, тест —
tests/unit/sim/test_sim_hil.py. Обёртка с rclpy — :func:`main`, её запускает
docker/sim/hil_run.sh в контейнере gazebo.

  python3 -m rob_box_sim.hil precheck   # 0 — можно стартовать Gazebo, 2 — отказ
  python3 -m rob_box_sim.hil watch      # бесконечно; 3 — нарушение, Gazebo гасить
  python3 -m rob_box_sim.hil count      # печать: число /controller_manager и узлы
"""

from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass
from typing import Iterable

# ── Имена сим-узлов в HIL (единственный источник: launch и тесты импортируют отсюда) ──

SIM_NAMESPACE = "sim"
SIM_RSP_NODE = f"/{SIM_NAMESPACE}/robot_state_publisher"
SIM_ROBOT_DESCRIPTION_TOPIC = f"/{SIM_NAMESPACE}/robot_description"
SIM_TF_REMAPS = (("/tf", f"/{SIM_NAMESPACE}/tf"), ("/tf_static", f"/{SIM_NAMESPACE}/tf_static"))

CONTROLLER_MANAGER = "/controller_manager"
# RSP Main Pi. Виден в графе — линк песочница ↔ робот поднят и ID совпадает.
# Без этого «0 контроллеров» значило бы только «робота не видно», а не «моторы выключены».
ROBOT_LINK_PROBE = "/robot_state_publisher"

EXIT_OK = 0
EXIT_REFUSED = 2
EXIT_VIOLATION = 3


@dataclass(frozen=True)
class Verdict:
    ok: bool
    reason: str


def full_names(names_and_namespaces: Iterable[tuple[str, str]]) -> list[str]:
    """``[("controller_manager", "/")] → ["/controller_manager"]`` (как ``ros2 node list``)."""
    out = []
    for name, ns in names_and_namespaces:
        ns = ns.rstrip("/")
        out.append(f"{ns}/{name}" if ns else f"/{name}")
    return sorted(out)


def count_controller_managers(names: Iterable[str]) -> int:
    return sum(1 for n in names if n == CONTROLLER_MANAGER)


def precheck(names: Iterable[str], *, require_link: bool = True) -> Verdict:
    """Можно ли поднимать Gazebo: робот виден, а контроллеров в графе нет ни одного."""
    names = list(names)
    if require_link and ROBOT_LINK_PROBE not in names:
        return Verdict(False, f"не видно {ROBOT_LINK_PROBE} робота: нет линка Zenoh "
                              "с Main Pi или ROBOT_ID не совпадает — отказ")
    cm = count_controller_managers(names)
    if cm:
        return Verdict(False, f"в графе уже {cm} × {CONTROLLER_MANAGER} до старта Gazebo: "
                              "ros2-control робота не остановлен — отказ")
    return Verdict(True, f"{CONTROLLER_MANAGER} в графе нет, Gazebo можно поднимать")


def watch_verdict(names: Iterable[str]) -> Verdict:
    """Работа HIL: больше одного контроллера — нарушение (второй — робот)."""
    cm = count_controller_managers(names)
    if cm > 1:
        return Verdict(False, f"в графе {cm} × {CONTROLLER_MANAGER}: поднялся контроллер "
                              "робота (деплой? перезагрузка Main Pi?) — Gazebo гасится")
    return Verdict(True, f"{CONTROLLER_MANAGER}: {cm}")


# ── rclpy-обёртка (в контейнере gazebo) ──────────────────────────────────────


def _snapshot(node, settle_s: float) -> list[str]:
    import rclpy

    deadline = time.monotonic() + settle_s
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    return full_names(node.get_node_names_and_namespaces())


def _log(msg: str) -> None:
    print(f"[hil-guard {time.strftime('%H:%M:%S', time.gmtime())}Z] {msg}", flush=True)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Сторож графа HIL: /controller_manager")
    ap.add_argument("mode", choices=("precheck", "watch", "count"))
    ap.add_argument("--settle", type=float, default=5.0,
                    help="секунд на сбор графа перед решением (Zenoh liveliness)")
    ap.add_argument("--interval", type=float, default=5.0, help="период watch, с")
    ap.add_argument("--no-link-check", action="store_true",
                    help=f"precheck без проверки {ROBOT_LINK_PROBE} (только песочница без робота)")
    args = ap.parse_args(argv)

    import rclpy

    rclpy.init()
    node = rclpy.create_node("hil_guard", namespace=f"/{SIM_NAMESPACE}")
    try:
        names = _snapshot(node, args.settle)
        if args.mode == "count":
            print(count_controller_managers(names))
            for n in names:
                print(n)
            return EXIT_OK
        if args.mode == "precheck":
            verdict = precheck(names, require_link=not args.no_link_check)
            _log(("OK: " if verdict.ok else "ОТКАЗ: ") + verdict.reason)
            return EXIT_OK if verdict.ok else EXIT_REFUSED
        last = None
        while True:
            verdict = watch_verdict(names)
            if not verdict.ok:
                _log("НАРУШЕНИЕ: " + verdict.reason)
                return EXIT_VIOLATION
            if verdict.reason != last:
                _log(verdict.reason)
                last = verdict.reason
            names = _snapshot(node, args.interval)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    sys.exit(main())
