"""Одна проба lifecycle трёх узлов nav2 (#5150).

``ros2 lifecycle get`` на каждый узел — отдельный процесс. На Main Pi один
такой вызов стоит 3.5–7.7 с и жжёт ядро; три подряд не влезают в docker
Timeout 10 с. Совпадение по времени с «CRITICAL FAILURE … heartbeat for
4000 ms» — гипотеза, не доказанный механизм (см. ``healthcheck_budget``).
Здесь один процесс ``rclpy`` спрашивает три сервиса ``/<node>/get_state``
под общим дедлайном короче bond-ориентира 4 с.

Решение о коде выхода — чистые функции. ``rclpy`` импортируется только в
``probe_states``: юнит гоняет вердикт без ROS.
"""

from __future__ import annotations

import sys
import time
from typing import Callable, Mapping, Optional, Sequence

from healthcheck_budget import PROBE_DEADLINE_S

NODES = ("controller_server", "planner_server", "bt_navigator")

ACTIVE = "active"


def _service_name(node: str) -> str:
    name = node if node.startswith("/") else f"/{node}"
    return f"{name}/get_state"


def verdict(states: Mapping[str, Optional[str]], nodes: Sequence[str] = NODES) -> tuple[int, str]:
    """0, если каждый узел ответил ``active``. Иначе 1 и причина по сбоям.

    ``None`` и пустая строка — нет ответа за дедлайн (недоступен или завис).
    Любое другое состояние — не active. Причина пишется в одну строку на узел,
    чтобы docker Health.Log показывал, кто именно красный.
    """
    lines = []
    for node in nodes:
        state = states.get(node)
        if state == ACTIVE:
            continue
        if not state:
            lines.append(f"nav2 healthcheck: /{node.lstrip('/')} no answer within {PROBE_DEADLINE_S:g}s")
        else:
            lines.append(f"nav2 healthcheck: /{node.lstrip('/')} is '{state}', want active")
    if lines:
        return 1, "\n".join(lines)
    return 0, ""


def collect_states(
    nodes: Sequence[str],
    deadline_s: float,
    now: Callable[[], float],
    spin_once: Callable[[float], None],
    service_ready: Callable[[str], bool],
    call_state: Callable[[str, float], Optional[str]],
) -> dict[str, Optional[str]]:
    """Собрать состояния до общего дедлайна. Не ответивший узел — ``None``.

    Сначала ждём, пока сервисы станут видны (одна сессия, общий spin), затем
    спрашиваем готовые. Остаток дедлайна делится между ещё не спрошенными:
    зависший первый вызов не съедает бюджет остальных и не выходит за дедлайн.
    """
    end = now() + deadline_s
    states: dict[str, Optional[str]] = {node: None for node in nodes}
    pending = list(nodes)
    while pending and now() < end:
        spin_once(min(0.05, end - now()))
        ready = [node for node in pending if service_ready(node)]
        if not ready:
            continue
        for node in ready:
            pending.remove(node)
            left = end - now()
            if left <= 0:
                break
            share = left / max(1, len(pending) + 1)
            states[node] = call_state(node, min(share, left))
    return states


def _spin_until(node, future, timeout_s: float) -> None:
    import rclpy

    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_s)


def probe_states(nodes: Sequence[str] = NODES, deadline_s: float = PROBE_DEADLINE_S) -> dict[str, Optional[str]]:
    """Спросить ``get_state`` всех узлов одним процессом rclpy."""
    import rclpy
    from lifecycle_msgs.srv import GetState

    rclpy.init()
    node = rclpy.create_node("nav2_healthcheck")
    try:
        clients = {name: node.create_client(GetState, _service_name(name)) for name in nodes}

        def spin_once(timeout_s: float) -> None:
            rclpy.spin_once(node, timeout_sec=max(0.0, timeout_s))

        def call_state(name: str, timeout_s: float) -> Optional[str]:
            future = clients[name].call_async(GetState.Request())
            _spin_until(node, future, timeout_s)
            if not future.done():
                return None
            result = future.result()
            if result is None:
                return None
            return result.current_state.label

        return collect_states(
            nodes,
            deadline_s,
            time.monotonic,
            spin_once,
            lambda name: clients[name].service_is_ready(),
            call_state,
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main(argv: Optional[Sequence[str]] = None) -> int:
    nodes = tuple(argv) if argv else NODES
    try:
        states = probe_states(nodes)
    except Exception as exc:  # noqa: BLE001 — healthcheck обязан вернуть причину, не трейс
        print(f"nav2 healthcheck: probe failed: {exc}", file=sys.stderr)
        return 1
    code, reason = verdict(states, nodes)
    if reason:
        print(reason, file=sys.stderr)
    return code


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
