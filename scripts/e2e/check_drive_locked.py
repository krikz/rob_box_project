#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Предпроверка блокировки привода перед окном приёмки #5094.

На робота ничего не публикует: только читает снимки топиков и lifecycle,
которые принёс координатор (или ``--capture-cmd`` на стенде). Сам по себе
скрипт к роботу не ходит.

Что в репо реально глушит привод (отдельной защёлки «стенд» нет):

* ``cmd_vel_emergency`` priority 255, timeout 0.1 с — нулевой Twist
  ``proximity_guard_node`` (``src/rob_box_teleop/rob_box_teleop/proximity_guard_node.py:53``).
  Пока сообщения идут, twist_mux отдаёт на моторы нули.
* ``/teleop_lock`` priority 90 и ``/joystick_lock`` priority 51, timeout 0.0
  (``docker/main/config/twist_mux/twist_mux.yaml``). True глушит источники
  строго меньшего приоритета, в том числе Nav2 ``cmd_vel`` (10) и голос (25).
  Оба False — «ехать можно», это не блокировка (``lock_block``, navigation.py).
* lifecycle ``controller_server`` / ``planner_server`` / ``bt_navigator`` не
  ``active`` — целей, которые ведут робота, нет (``docker/main/scripts/nav2/healthcheck.sh``
  считает active нормой; для окна на кирпичах норма — обратная).

Честный FAIL: ни один признак не подтверждён сырьём — exit 1, даже если
часть снимков просто не пришла. Неподтверждённое не считается блокировкой.
"""

from __future__ import annotations

import argparse
import json
import shlex
import subprocess
import sys
from dataclasses import dataclass
from typing import Callable, List, Optional, Sequence

# Приоритеты — docker/main/config/twist_mux/twist_mux.yaml (не словарь: там joystick_lock ещё 100).
EMERGENCY_PRIORITY = 255
EMERGENCY_TIMEOUT_S = 0.1
JOYSTICK_LOCK_PRIORITY = 51
TELEOP_LOCK_PRIORITY = 90
NAV_CMD_VEL_PRIORITY = 10
VOICE_CMD_VEL_PRIORITY = 25

# Узлы, без которых навигации нет (healthcheck.sh). Для окна все должны быть НЕ active.
NAV_NODES = ("/controller_server", "/planner_server", "/bt_navigator")

ZERO = 1e-6

Capture = Callable[[str], str]


@dataclass
class Sign:
    """Один признак: имя, PASS/FAIL, сырьё, которое это доказало или не доказало."""

    name: str
    ok: bool
    raw: str

    def line(self) -> str:
        mark = "PASS" if self.ok else "FAIL"
        return f"{mark} {self.name}: {self.raw}"


def _float(text: str) -> Optional[float]:
    try:
        return float(text)
    except (TypeError, ValueError):
        return None


def _last_data(raw: str) -> Optional[str]:
    """Последняя строка ``data:`` из ``ros2 topic echo``. Нет строки — нет сообщения."""
    found = None
    for line in raw.splitlines():
        stripped = line.strip()
        if stripped.startswith("data:"):
            found = stripped.split(":", 1)[1].strip().strip("'\"")
    return found


def _twist_nonzero(raw: str) -> Optional[bool]:
    """True — в сырье есть ненулевая скорость. False — все числа нули. None — чисел нет."""
    saw = False
    for line in raw.splitlines():
        stripped = line.strip()
        if ":" not in stripped:
            continue
        key, _, value = stripped.partition(":")
        key = key.strip()
        if key not in ("x", "y", "z"):
            continue
        number = _float(value.strip())
        if number is None:
            continue
        saw = True
        if abs(number) > ZERO:
            return True
    if not saw:
        return None
    return False


def _lifecycle_state(raw: str) -> Optional[str]:
    """Последняя строка вида ``active [3]``. Префикс zenoh (``Using ZID``) игнорируется."""
    found = None
    for line in raw.splitlines():
        parts = line.split()
        if (
            len(parts) >= 2
            and parts[0].isalpha()
            and parts[1].startswith("[")
            and parts[1].endswith("]")
        ):
            found = parts[0]
    return found


def _bool_data(raw: str) -> Optional[bool]:
    value = _last_data(raw)
    if value is None:
        return None
    lowered = value.lower()
    if lowered in ("true", "1"):
        return True
    if lowered in ("false", "0"):
        return False
    return None


def sign_emergency(raw: str) -> Sign:
    """Нулевой Twist в cmd_vel_emergency глушит всё (priority 255, timeout 0.1 с)."""
    name = "cmd_vel_emergency"
    nonzero = _twist_nonzero(raw)
    if nonzero is None:
        return Sign(
            name,
            False,
            f"нет сообщения Twist (priority {EMERGENCY_PRIORITY} не доказан): {raw!r}",
        )
    if nonzero:
        return Sign(
            name,
            False,
            f"ненулевой Twist — стоп не держится (timeout {EMERGENCY_TIMEOUT_S} с): {raw!r}",
        )
    return Sign(
        name,
        True,
        f"нулевой Twist, priority {EMERGENCY_PRIORITY}, timeout {EMERGENCY_TIMEOUT_S} с; сырьё: {raw!r}",
    )


def sign_lock(name: str, raw: str, priority: int) -> Sign:
    """True на липком замке глушит Nav2 (10) и голос (25). False — ехать можно."""
    value = _bool_data(raw)
    if value is None:
        return Sign(
            name,
            False,
            f"нет Bool (priority {priority}, timeout 0.0 — неизвестно не значит занято): {raw!r}",
        )
    if not value:
        return Sign(
            name,
            False,
            f"data=false, priority {priority}: источники < {priority} не глушатся (Nav2 {NAV_CMD_VEL_PRIORITY}, "
            f"голос {VOICE_CMD_VEL_PRIORITY}); сырьё: {raw!r}",
        )
    return Sign(
        name,
        True,
        f"data=true, priority {priority} глушит источники < {priority} (Nav2 {NAV_CMD_VEL_PRIORITY}, "
        f"голос {VOICE_CMD_VEL_PRIORITY}); сырьё: {raw!r}",
    )


def sign_nav_inactive(node: str, raw: str) -> Sign:
    """Узел навигации не active — целей, которые ведут робота, этот узел не везёт."""
    name = f"nav2 {node}"
    state = _lifecycle_state(raw)
    if state is None:
        return Sign(
            name,
            False,
            f"lifecycle не разобран (нет строки вида 'inactive [1]'): {raw!r}",
        )
    if state == "active":
        return Sign(
            name, False, f"state=active — узел жив и может вести; сырьё: {raw!r}"
        )
    return Sign(name, True, f"state={state} (не active); сырьё: {raw!r}")


def judge(snapshots: dict) -> List[Sign]:
    """Признаки по снимкам. Ключ отсутствует — признак FAIL с пустым сырьём."""
    signs = [
        sign_emergency(snapshots.get("cmd_vel_emergency", "")),
        sign_lock(
            "joystick_lock", snapshots.get("joystick_lock", ""), JOYSTICK_LOCK_PRIORITY
        ),
        sign_lock(
            "teleop_lock", snapshots.get("teleop_lock", ""), TELEOP_LOCK_PRIORITY
        ),
    ]
    for node in NAV_NODES:
        signs.append(sign_nav_inactive(node, snapshots.get(node, "")))
    return signs


def blocked(signs: Sequence[Sign]) -> bool:
    """Привод заблокирован, только если хотя бы один признак подтверждён сырьём."""
    return any(sign.ok for sign in signs)


def render(signs: Sequence[Sign]) -> str:
    lines = [sign.line() for sign in signs]
    if blocked(signs):
        held = ", ".join(sign.name for sign in signs if sign.ok)
        lines.append(f"PASS drive locked by: {held}")
    else:
        lines.append("FAIL drive NOT locked: ни один признак не подтверждён сырьём")
    return "\n".join(lines)


def load_snapshots(path: str) -> dict:
    with open(path, encoding="utf-8") as handle:
        loaded = json.load(handle)
    if not isinstance(loaded, dict):
        raise ValueError(
            f"{path}: жду объект имя→сырьё, пришло {type(loaded).__name__}"
        )
    return {
        str(key): "" if value is None else str(value) for key, value in loaded.items()
    }


def capture_from_command(template: str) -> Capture:
    """``{name}`` в команде — имя признака. Команда только читает; скрипт её не выдумывает."""

    def capture(name: str) -> str:
        command = template.format(name=shlex.quote(name))
        completed = subprocess.run(
            command,
            shell=True,
            capture_output=True,
            text=True,
            check=False,
        )
        raw = completed.stdout
        if completed.stderr:
            raw = raw + ("\n" if raw else "") + completed.stderr
        if completed.returncode not in (0, 124):
            raw = raw + f"\nrc={completed.returncode}"
        return raw

    return capture


def snapshots_from_capture(capture: Capture) -> dict:
    names = ("cmd_vel_emergency", "joystick_lock", "teleop_lock", *NAV_NODES)
    return {name: capture(name) for name in names}


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        description="Предпроверка блокировки привода перед окном #5094. Ничего не публикует."
    )
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument(
        "--snapshots",
        help="JSON: имя признака → сырьё ros2 topic echo / ros2 lifecycle get",
    )
    source.add_argument(
        "--capture-cmd",
        help="Команда чтения одного признака, {name} подставляется. Скрипт сам к роботу не ходит.",
    )
    args = parser.parse_args(argv)
    if args.snapshots:
        snapshots = load_snapshots(args.snapshots)
    else:
        snapshots = snapshots_from_capture(capture_from_command(args.capture_cmd))
    signs = judge(snapshots)
    print(render(signs))
    return 0 if blocked(signs) else 1


if __name__ == "__main__":
    sys.exit(main())
