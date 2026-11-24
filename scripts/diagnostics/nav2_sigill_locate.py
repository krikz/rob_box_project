#!/usr/bin/env python3
"""Адрес падения controller_server -> модуль и смещение (#5180).

На робота не ходит. На вход — текст dmesg и текст /proc/<pid>/maps того же
процесса. На выход — одна строка или JSON: адрес, сигнал, модуль, смещение
внутри отображения. Адрес вне всех строк maps — ближайший модуль и пробел.
Пустой ввод не бросает исключение.

Смещение считается от начала отображения (колонка start maps), не от file
offset: addr2line на Pi потом получает VMA. file_offset = maps-offset +
(ip - start), его печатает только --json.
"""

from __future__ import annotations

import argparse
import json
import re
import sys
from typing import Optional

# ip:ADDR или «at pc ADDR». Префикс 0x необязателен.
_IP = re.compile(r"(?:ip:|at pc\s+)(0x)?([0-9a-fA-F]+)")
_TRAP = re.compile(
    r"traps:\s+(\S+)\[(\d+)\]\s+" r"(general protection fault|illegal instruction|segfault|segmentation fault)",
    re.IGNORECASE,
)
_COMM_TRAP = re.compile(
    r"(\S+)\[(\d+)\]:\s+(illegal instruction|general protection fault|segfault)",
    re.IGNORECASE,
)
_MAPS = re.compile(r"^([0-9a-fA-F]+)-([0-9a-fA-F]+)\s+(\S+)\s+([0-9a-fA-F]+)\s+\S+\s+\S+\s*(.*)$")

_SIGNAL = {
    "illegal instruction": "SIGILL",
    "general protection fault": "SIGSEGV",
    "segfault": "SIGSEGV",
    "segmentation fault": "SIGSEGV",
}


def parse_dmesg(text: str) -> dict:
    """Первая строка traps с адресом. Пустой текст — пустой разбор."""
    found = {"comm": None, "pid": None, "fault": None, "signal": None, "ip": None}
    if not text or not text.strip():
        return found
    for line in text.splitlines():
        trap = _TRAP.search(line) or _COMM_TRAP.search(line)
        ip = _IP.search(line)
        if trap is None and ip is None:
            continue
        if trap is not None:
            fault = trap.group(3).lower()
            found["comm"] = trap.group(1)
            found["pid"] = int(trap.group(2))
            found["fault"] = fault
            found["signal"] = _SIGNAL.get(fault)
        if ip is not None:
            found["ip"] = int(ip.group(2), 16)
        if found["ip"] is not None and found["fault"] is not None:
            return found
    return found


def parse_maps(text: str) -> list:
    """Строки /proc/pid/maps. Пустой текст — пустой список."""
    rows = []
    if not text:
        return rows
    for line in text.splitlines():
        match = _MAPS.match(line.strip())
        if match is None:
            continue
        path = match.group(5).strip()
        rows.append(
            {
                "start": int(match.group(1), 16),
                "end": int(match.group(2), 16),
                "perms": match.group(3),
                "offset": int(match.group(4), 16),
                "path": path or None,
            }
        )
    return rows


def _module_name(path):
    if not path:
        return None
    name = path.rsplit("/", 1)[-1]
    return name or None


def locate(ip, maps):
    """Адрес -> модуль и смещение, либо пробел до ближайшего отображения."""
    if ip is None:
        return {"hit": None, "gap_to_nearest": None, "nearest": None}
    hits = [row for row in maps if row["start"] <= ip < row["end"]]
    if hits:
        exec_hits = [row for row in hits if "x" in row["perms"]]
        row = exec_hits[0] if exec_hits else hits[0]
        delta = ip - row["start"]
        return {
            "hit": {
                "module": _module_name(row["path"]),
                "path": row["path"],
                "offset": delta,
                "file_offset": row["offset"] + delta,
                "perms": row["perms"],
                "start": row["start"],
                "end": row["end"],
            },
            "gap_to_nearest": 0,
            "nearest": None,
        }
    if not maps:
        return {"hit": None, "gap_to_nearest": None, "nearest": None}
    nearest = min(
        maps,
        key=lambda row: min(abs(ip - row["start"]), abs(ip - (row["end"] - 1))),
    )
    if ip < nearest["start"]:
        gap = nearest["start"] - ip
    else:
        gap = ip - (nearest["end"] - 1)
    return {
        "hit": None,
        "gap_to_nearest": gap,
        "nearest": {
            "module": _module_name(nearest["path"]),
            "path": nearest["path"],
            "start": nearest["start"],
            "end": nearest["end"],
        },
    }


def resolve(dmesg: str, maps_text: str) -> dict:
    """Сырой текст -> один словарь. Пустой ввод не бросает исключение."""
    fault = parse_dmesg(dmesg)
    maps = parse_maps(maps_text)
    placed = locate(fault["ip"], maps)
    hit = placed["hit"]
    nearest = placed["nearest"]
    return {
        "comm": fault["comm"],
        "pid": fault["pid"],
        "fault": fault["fault"],
        "signal": fault["signal"],
        "ip": fault["ip"],
        "module": None if hit is None else hit["module"],
        "path": None if hit is None else hit["path"],
        "offset": None if hit is None else hit["offset"],
        "file_offset": None if hit is None else hit["file_offset"],
        "gap_to_nearest": placed["gap_to_nearest"],
        "nearest_module": None if nearest is None else nearest["module"],
        "in_module": hit is not None,
    }


def format_line(result: dict) -> str:
    """Одна строка. Адрес и смещение — hex."""
    ip = result["ip"]
    ip_text = "none" if ip is None else "0x%x" % ip
    signal = result["signal"] or "none"
    if result["in_module"]:
        return "%s %s %s+0x%x" % (ip_text, signal, result["module"], result["offset"])
    if result["nearest_module"] is not None:
        return "%s %s outside modules, nearest %s gap 0x%x" % (
            ip_text,
            signal,
            result["nearest_module"],
            result["gap_to_nearest"],
        )
    return "%s %s no module" % (ip_text, signal)


def main(argv: Optional[list] = None) -> int:
    parser = argparse.ArgumentParser(description="Адрес падения nav2 -> модуль и смещение")
    parser.add_argument("--dmesg", required=True, help="файл dmesg, или - для stdin")
    parser.add_argument("--maps", required=True, help="файл maps; оба входа не могут быть stdin")
    parser.add_argument("--json", action="store_true", help="полный JSON вместо одной строки")
    args = parser.parse_args(argv)
    if args.dmesg == "-" and args.maps == "-":
        print("nav2_sigill_locate: оба входа не могут быть stdin", file=sys.stderr)
        return 2

    def _read(path: str) -> str:
        if path == "-":
            return sys.stdin.read()
        with open(path, encoding="utf-8") as handle:
            return handle.read()

    result = resolve(_read(args.dmesg), _read(args.maps))
    if args.json:
        print(json.dumps(result, ensure_ascii=False, indent=2))
    else:
        print(format_line(result))
    return 0


if __name__ == "__main__":
    sys.exit(main())
