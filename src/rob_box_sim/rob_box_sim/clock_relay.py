"""HIL: прореженный /clock для робота (эпик #4371, план docs/research/2026-11-04-hil-robot-on-gazebo-data.md §8.5 п.6).

Gazebo Fortress публикует /clock на каждом шаге физики (max_step_size 1 мс → 1 кГц; замер
§8.5: 999.985 Гц). В HIL каждая sim-нода Main Pi (~10 шт.) получает 1000 сообщений/с и
обрабатывает их (/clock → TimeSource → таймеры). Первый живой прогон: load average Main Pi
30 при 4 ядрах.

Решение: мост пишет сырой 1 кГц в ``/sim/clock_raw`` (его читает только этот узел, на
роботе подписчиков нет), а этот узел отдаёт в ``/clock`` не чаще одного сообщения на
``period`` СИМ-времени (по умолчанию 10 мс = 100 Гц). Физика и RTF не меняются —
Gazebo об узле не знает. Прореживание по sim-времени, а не по wall: при RTF 0.5 робот
получит 50 Гц wall (те же 100 Гц sim), время в /clock не искажается.

Решение «публиковать или нет» — чистая функция :func:`should_publish` (ADR-0148),
тест tests/unit/sim/test_sim_clock_relay.py. Узел — тонкая rclpy-обёртка :func:`main`.

  python3 -m rob_box_sim.clock_relay --hz 100
"""

from __future__ import annotations

import argparse
import sys

RAW_TOPIC = "/sim/clock_raw"
OUT_TOPIC = "/clock"
DEFAULT_HZ = 100.0


def period_ns(hz: float) -> int:
    if not hz > 0:
        raise ValueError(f"hz должен быть > 0, получено {hz!r}")
    return int(round(1e9 / hz))


def should_publish(stamp_ns: int, last_ns: int | None, period: int) -> bool:
    """Отдавать ли отсчёт ``stamp_ns``.

    Первый отсчёт — всегда. Время назад (сброс сима) — всегда: иначе робот остался бы
    в «будущем», и /clock замолчал бы до перегона старого времени.
    """
    if last_ns is None or stamp_ns < last_ns:
        return True
    return stamp_ns - last_ns >= period


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Прореживание /clock Gazebo для HIL")
    ap.add_argument("--hz", type=float, default=DEFAULT_HZ, help="частота /clock по sim-времени, Гц")
    args = ap.parse_args(argv)
    period = period_ns(args.hz)

    import rclpy
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
    from rosgraph_msgs.msg import Clock

    rclpy.init()
    # use_sim_time False: узел сам источник времени, на /clock подписываться ему нельзя.
    node = rclpy.create_node("clock_relay", namespace="/sim")
    pub = node.create_publisher(
        Clock, OUT_TOPIC,
        QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1),
    )
    state = {"last": None, "in": 0, "out": 0}

    def on_clock(msg: Clock) -> None:
        stamp = msg.clock.sec * 1_000_000_000 + msg.clock.nanosec
        state["in"] += 1
        if should_publish(stamp, state["last"], period):
            state["last"] = stamp
            state["out"] += 1
            pub.publish(msg)

    node.create_subscription(
        Clock, RAW_TOPIC, on_clock,
        QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10),
    )

    def report() -> None:
        node.get_logger().info(
            f"/clock relay: in={state['in']} out={state['out']} за 10 с (цель {args.hz:g} Гц sim)")
        state["in"] = state["out"] = 0

    node.create_timer(10.0, report)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
