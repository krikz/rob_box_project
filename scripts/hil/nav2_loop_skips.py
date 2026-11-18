#!/usr/bin/env python3
"""Счётчик пропусков такта Nav2 по уже существующим WARN/строкам лога.

Сырой факт depot4694_20261117T1235Z (#4694, Loki nav2 12:31–13:02Z):
  Control loop missed its desired rate of 20.0000Hz     6915
  Behavior Tree tick rate 100.00 was exceeded!          1904
  Planner loop missed its desired rate of 20.0000 Hz      77

Не меняет controller_frequency / bt_loop_duration и не вводит порог вердикта:
только раскладывает уже пишущиеся Nav2-события. speed/run_scene.sh раньше
считал shell-grep ``missed its desired rate`` одним числом (controller+planner),
BT tick exceeded не видел.

    python scripts/hil/nav2_loop_skips.py <nav2.log>
    # → LOOP_SKIPS controller=N bt=N planner=N rate_missed=N
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Mapping

# Подстроки Nav2 Humble 1.1.20 (controller_server / bt_navigator / planner_server).
# «its» в controller/planner есть в upstream; старые доки иногда без — ловим оба.
CONTROLLER = "Control loop missed"
PLANNER = "Planner loop missed"
BT_TICK = "Behavior Tree tick rate"
BT_EXCEEDED = "exceeded"
# Совместимость с прежним grep -cE 'missed its desired rate' (+ вариант без its).
RATE_MISSED_MARKERS = (
    "missed its desired rate",
    "missed desired rate",
)


def count_loop_skips(text: str) -> dict[str, int]:
    """Сколько строк лога — пропуск такта controller / BT / planner.

    Одна строка учитывается в одном классе. ``rate_missed`` = controller + planner
    (то, что раньше давал shell-grep по desired rate).
    """
    controller = planner = bt = 0
    for line in text.splitlines():
        if CONTROLLER in line:
            controller += 1
        elif PLANNER in line:
            planner += 1
        elif BT_TICK in line and BT_EXCEEDED in line:
            bt += 1
    return {
        "controller": controller,
        "bt": bt,
        "planner": planner,
        "rate_missed": controller + planner,
    }


def format_loop_skips(counts: Mapping[str, int]) -> str:
    """Одна строка для route/speed лога: LOOP_SKIPS controller=… bt=… planner=… rate_missed=…"""
    return (
        f"LOOP_SKIPS controller={int(counts['controller'])} bt={int(counts['bt'])} "
        f"planner={int(counts['planner'])} rate_missed={int(counts['rate_missed'])}"
    )


def count_rate_missed_compat(text: str) -> int:
    """Как shell ``grep -cE 'missed its desired rate'``, плюс вариант без ``its``."""
    n = 0
    for line in text.splitlines():
        if any(m in line for m in RATE_MISSED_MARKERS):
            n += 1
    return n


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument(
        "log",
        nargs="?",
        type=Path,
        help="файл docker logs nav2; без аргумента — stdin",
    )
    args = p.parse_args(argv)
    if args.log is None:
        text = sys.stdin.read()
    elif args.log.is_file():
        text = args.log.read_text(encoding="utf-8", errors="replace")
    else:
        text = ""
    counts = count_loop_skips(text)
    print(format_loop_skips(counts), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
