#!/usr/bin/env python3
"""Предохранитель #5160: цель в HIL только в Gazebo, колёса железа молчат.

Код решает до первой цели (ADR-0148). Три факта, все обязательны, иначе отказ:

* ``hil.sh status`` показывает ровно один ``/controller_manager`` и боевой
  ``ros2-control`` не в состоянии ``running``;
* командных кадров VESC нет: ``commands`` в json ``can_watch.py`` равен 0
  (команды 0..8 — управление, не статусы);
* цель ещё не отправлена (``inject.txt`` пуст, в ``probe.log`` нет ``NAV_SENT``).

Нет сырья — тоже отказ: «не проверено» не считается разрешением. Оркестр при
отказе выходит с кодом 2 и цель не шлёт. Модуль живое железо не трогает.
"""

from __future__ import annotations

import json
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Sequence, Tuple

#: Код оркестра, когда предохранитель отказал: цель не отправлялась.
REFUSED = 2
OK = 0

_CM = re.compile(r"/controller_manager в графе[^:\n]*: (\S+)")
_ROS2_CONTROL = re.compile(r"^/?ros2-control\s+(\S+)", re.MULTILINE)


@dataclass(frozen=True)
class Isolation:
    """Вердикт до цели: ``ok`` или отказ с фактами."""

    ok: bool
    facts: Tuple[str, ...]

    def line(self) -> str:
        head = "ISOLATION PASS" if self.ok else "ISOLATION REFUSE"
        return head + ": " + "; ".join(self.facts)


def _text(path: Path) -> str:
    if not path.is_file():
        return ""
    return path.read_text(encoding="utf-8", errors="replace")


def _combat_stopped(inspect: str) -> Optional[str]:
    """Факт, если docker inspect назвал ros2-control и он не running."""
    found = _ROS2_CONTROL.search(inspect)
    if found is None or found.group(1) == "running":
        return None
    return f"боевой ros2-control: {found.group(1)}"


def _gazebo_running(inspect: str, prefix: str) -> Optional[str]:
    """Факт, если контейнер gazebo песочницы running. Префикс — из hil.conf."""
    name = f"{prefix}-gazebo" if prefix else "gazebo"
    for line in inspect.splitlines():
        parts = line.split()
        if len(parts) < 2:
            continue
        got = parts[0][1:] if parts[0].startswith("/") else parts[0]
        if got == name and parts[1] == "running":
            return f"{name}: running"
    return None


def _one_controller(status: str) -> Optional[str]:
    """Факт, если в графе ровно один /controller_manager (это Gazebo, не железо)."""
    found = _CM.search(status)
    if found is None or found.group(1) != "1":
        return None
    return "/controller_manager в графе: 1"


def _vesc_commands(raw: str) -> Optional[int]:
    """Число командных кадров 0..8. Нет разбора — None, и это отказ, не ноль."""
    text = raw.strip()
    if not text:
        return None
    try:
        payload = json.loads(text)
    except json.JSONDecodeError:
        return None
    commands = payload.get("commands")
    if isinstance(commands, bool) or not isinstance(commands, int) or commands < 0:
        return None
    return commands


def _goal_not_sent(directory: Path) -> Optional[str]:
    if _text(directory / "inject.txt").strip():
        return None
    if re.search(r"(^|\s)NAV_SENT(\s|$)", _text(directory / "probe.log"), re.MULTILINE):
        return None
    return "цель ещё не отправлена"


def judge(directory: Path, gazebo_prefix: str = "") -> Isolation:
    """Сырьё каталога → вердикт. Пустой факт — отказ, цель слать нельзя."""
    status = _text(directory / "hil_status.txt")
    inspect = _text(directory / "inspect.txt")
    bad: list[str] = []
    facts: list[str] = []
    combat = _combat_stopped(inspect)
    (facts if combat else bad).append(
        combat or "боевой ros2-control не доказан остановленным"
    )
    cm = _one_controller(status)
    (facts if cm else bad).append(cm or "не Gazebo: /controller_manager не ровно 1")
    gz = _gazebo_running(inspect, gazebo_prefix)
    (facts if gz else bad).append(gz or "контейнер gazebo не running")
    commands = _vesc_commands(_text(directory / "can_watch.json"))
    if commands == 0:
        facts.append("командных кадров VESC: 0")
    elif commands is None:
        bad.append("командных кадров VESC нет в сырье (can_watch.json)")
    else:
        bad.append(f"командных кадров VESC: {commands}")
    quiet = _goal_not_sent(directory)
    (facts if quiet else bad).append(
        quiet or "цель уже отправлена — предохранитель опоздал"
    )
    if bad:
        return Isolation(False, tuple(bad + facts))
    return Isolation(True, tuple(facts))


def main(argv: Optional[Sequence[str]] = None) -> int:
    import argparse

    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("dir", type=Path)
    ap.add_argument("--gazebo-prefix", default="")
    args = ap.parse_args(argv)
    verdict = judge(args.dir, args.gazebo_prefix)
    print(verdict.line())
    return OK if verdict.ok else REFUSED


if __name__ == "__main__":
    import sys

    sys.exit(main())
