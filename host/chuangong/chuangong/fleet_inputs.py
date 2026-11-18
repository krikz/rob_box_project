"""Входы пульта F: Update Robot: robot_id, release, action; пустые — отказ (ADR-0168, #4375).

Команда и отчёт делят один operation_id (run_id workflow). Сети, ssh и compose здесь нет.
status — единственный wired remote-action (zenoh read-only). update/rollback/cancel после
валидных полей — явный not_wired (exit != 0), не зелёный «успех без дела».
Cancel на Pi — host/chuangong cancel (локальный staged).
"""

from __future__ import annotations

from typing import Any

ACTIONS = ("status", "update", "rollback", "cancel")
# Единственный remote-путь, реально выполняемый workflow (status job → zenoh).
WIRED_ACTIONS = frozenset({"status"})
# update/rollback — по имени релиза; status может сверять опционально; cancel — локальный staged.
RELEASE_REQUIRED = frozenset({"update", "rollback"})


class FleetInputError(ValueError):
    """Пустой, неизвестный или ещё не wired вход пульта. reason — стабильный код для CI."""

    def __init__(self, reason: str, detail: str = "") -> None:
        super().__init__(f"{reason}: {detail}" if detail else reason)
        self.reason = reason
        self.detail = detail


def validate_inputs(
    *,
    action: str,
    robot_id: str,
    release: str = "",
    operation_id: str = "",
) -> dict[str, Any]:
    """Вернуть нормализованные входы или FleetInputError.

    Порядок отказов: empty/bad поля → not_wired для update/rollback/cancel.
    robot_id пустой — всегда отказ. release пустой — отказ для update и rollback.
    """
    act = (action or "").strip()
    rid = (robot_id or "").strip()
    rel = (release or "").strip()
    op = (operation_id or "").strip()

    if not act:
        raise FleetInputError("empty_action")
    if act not in ACTIONS:
        raise FleetInputError("bad_action", act)
    if not rid:
        raise FleetInputError("empty_robot_id")
    if act in RELEASE_REQUIRED and not rel:
        raise FleetInputError("empty_release", act)
    if act not in WIRED_ACTIONS:
        raise FleetInputError(
            "not_wired",
            f"{act}: remote path not connected yet (no ssh; cancel on Pi — chuangong cancel)",
        )

    return {
        "action": act,
        "robot_id": rid,
        "release": rel if rel else None,
        "operation_id": op if op else None,
        "release_required": act in RELEASE_REQUIRED,
        "wired": True,
    }


def render_summary(record: dict[str, Any]) -> str:
    """Markdown для GITHUB_STEP_SUMMARY при принятых (wired) входах."""
    rel = record.get("release") or "—"
    op = record.get("operation_id") or "—"
    lines = [
        "### F: Update Robot — входы",
        "",
        f"- action: `{record['action']}`",
        f"- robot_id: `{record['robot_id']}`",
        f"- release: `{rel}`",
        f"- operation_id: `{op}`",
        "",
    ]
    return "\n".join(lines)


def render_error_summary(
    *,
    reason: str,
    detail: str = "",
    action: str = "",
    robot_id: str = "",
    release: str = "",
    operation_id: str = "",
) -> str:
    """Markdown отказа: пустые поля или not_wired — job не зелёный."""
    lines = [
        "### F: Update Robot — отказ",
        "",
        f"- reason: `{reason}`",
    ]
    if detail:
        lines.append(f"- detail: `{detail}`")
    if action:
        lines.append(f"- action: `{action}`")
    if robot_id:
        lines.append(f"- robot_id: `{robot_id}`")
    if release:
        lines.append(f"- release: `{release}`")
    if operation_id:
        lines.append(f"- operation_id: `{operation_id}`")
    lines.append("")
    if reason == "not_wired":
        lines.append(
            "Удалённый update/rollback/cancel ещё не подключен. "
            "SSH нет. Cancel на Pi — `chuangong cancel`. Job exit ≠ 0 (не тихий успех)."
        )
        lines.append("")
    return "\n".join(lines)
