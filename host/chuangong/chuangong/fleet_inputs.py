"""Входы пульта F: Update Robot: robot_id, release, action; пустые — отказ (ADR-0168, #4375).

Команда и отчёт делят один operation_id (run_id workflow). Сети, ssh и compose здесь нет:
только проверка полей до status/update/rollback/cancel. Cancel на Pi — host/chuangong cancel.
"""

from __future__ import annotations

from typing import Any

ACTIONS = ("status", "update", "rollback", "cancel")
# update/rollback — по имени релиза; status может сверять опционально; cancel — локальный staged.
RELEASE_REQUIRED = frozenset({"update", "rollback"})


class FleetInputError(ValueError):
    """Пустой или неизвестный вход пульта. reason — стабильный код для CI и Job Summary."""

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

    return {
        "action": act,
        "robot_id": rid,
        "release": rel if rel else None,
        "operation_id": op if op else None,
        "release_required": act in RELEASE_REQUIRED,
    }


def render_summary(record: dict[str, Any]) -> str:
    """Markdown для GITHUB_STEP_SUMMARY: одни поля, без «успеха обновления»."""
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
    if record["action"] != "status":
        lines.append(
            "Входы приняты. Удалённый путь update/rollback/cancel в этом workflow не выполняется "
            "(без ssh; cancel на Pi — `chuangong cancel`)."
        )
        lines.append("")
    return "\n".join(lines)
