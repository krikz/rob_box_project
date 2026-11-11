"""Наблюдатель для workflow «F: Update Robot» (action: status): собрать состояния агентов трёх Pi через zenoh REST.

Только чтение: ни ssh, ни команд роботу. Сначала `get` (вернёт данные, только если в роутере есть узкое
хранилище), затем подписка SSE до `wait_s` — агенты публикуют состояние каждые 5 минут.
Итог строит код: таблица и вердикт; «всё хорошо» — только если отчитались все три узла без ошибок.
"""

from __future__ import annotations

from typing import Any

from . import state as st
from .zenoh_rest import ZenohError


def _node_of(key: str) -> str:
    return key.rstrip("/").rsplit("/", 1)[-1]


def collect(zenoh: Any, robot_id: str, wait_s: float, nodes: tuple[str, ...] = st.NODES) -> tuple[dict, list[str]]:
    """({узел: запись}, заметки о том, как собирали)."""
    selector = st.state_selector(robot_id)
    found: dict[str, dict[str, Any]] = {}
    notes: list[str] = []
    try:
        for key, val in zenoh.get(selector):
            if _node_of(key) in nodes and isinstance(val, dict):
                found[_node_of(key)] = val
        notes.append(f"GET {selector}: узлов {len(found)}")
    except ZenohError as exc:
        notes.append(f"GET {selector}: ошибка {exc}")
    if len(found) < len(nodes) and wait_s > 0:
        try:
            for key, val in zenoh.subscribe(selector, wait_s):
                if _node_of(key) in nodes and isinstance(val, dict):
                    found[_node_of(key)] = val
                if len(found) == len(nodes):
                    break
            notes.append(f"подписка {selector} до {int(wait_s)} с: узлов {len(found)}")
        except ZenohError as exc:
            notes.append(f"подписка {selector}: ошибка {exc}")
    return found, notes


def verdict(found: dict[str, dict], release: str | None, nodes: tuple[str, ...] = st.NODES) -> tuple[bool, list[str]]:
    problems = [f"{n}: нет отчёта агента" for n in nodes if n not in found]
    for node, rec in sorted(found.items()):
        if rec.get("state") in {s.value for s in st.ERROR_STATES}:
            problems.append(f"{node}: {rec.get('state')} — {rec.get('reason')}")
        if release and release not in (rec.get("available"), rec.get("staged")):
            problems.append(f"{node}: релиз {release} не виден (available={rec.get('available')})")
    releases = {rec.get("available") for rec in found.values()}
    if len(releases) > 1:
        problems.append(f"узлы видят разные релизы: {sorted(map(str, releases))}")
    return not problems, problems


_COLUMNS = ("state", "available", "staged", "current", "mode", "hil_locked", "dirty", "ts", "reason")


def render(robot_id: str, found: dict[str, dict], ok: bool, problems: list[str], notes: list[str]) -> str:
    lines = [f"## chuangong: {robot_id}", ""]
    lines.append("| node | " + " | ".join(_COLUMNS) + " |")
    lines.append("|" + "---|" * (len(_COLUMNS) + 1))
    for node in st.NODES:
        rec = found.get(node)
        cells = [str(rec.get(c)) if rec else "—" for c in _COLUMNS]
        lines.append(f"| {node} | " + " | ".join(c.replace("|", "/") for c in cells) + " |")
    lines += ["", "**Итог:** " + ("все узлы отчитались, ошибок нет" if ok else "есть проблемы"), ""]
    lines += [f"- {p}" for p in problems]
    lines += ["", "<details><summary>как собирали</summary>", ""] + [f"- {n}" for n in notes] + ["", "</details>"]
    return "\n".join(lines) + "\n"
