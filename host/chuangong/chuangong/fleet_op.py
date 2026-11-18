"""Координация Vision → Sound → Main: детерминированный ход операции без сети.

Слот 4 (фаза C, ADR-0003): Main-координатор пишет ход операции и исходный релиз в state.json,
при сбое помечает failed и строит откат уже затронутых ролей. Compose/zenoh/сеть не трогает.

Протокол role_switch (слот 3; если `role_switch.py` ещё нет — контракт здесь минимально):
  ROLE_ORDER = ("vision", "sound", "main")
  switch(role: str, release: str) -> None   # успех; иначе исключение / ненулевой код
  Координатор вызывает switch по ROLE_ORDER; этот модуль хранит ход, исходный релиз и план отката.
  Исполнитель switch — не здесь (слот 3 / activation); сеть и compose этому модулю запрещены.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

from . import state as st

ROLE_ORDER: tuple[str, ...] = ("vision", "sound", "main")

STATUS_RUNNING = "running"
STATUS_FAILED = "failed"
STATUS_COMPLETED = "completed"

_SCHEMA = 1
_KIND = "fleet_op"
_STATE_NAME = "state.json"


class FleetOpError(RuntimeError):
    def __init__(self, reason: str, detail: str = "") -> None:
        super().__init__(f"{reason}: {detail}" if detail else reason)
        self.reason = reason
        self.detail = detail


def state_path(state_dir: Path) -> Path:
    return state_dir / _STATE_NAME


def load(state_dir: Path) -> dict[str, Any] | None:
    """Прочитать ход операции из state.json. Сети нет."""
    raw = st.load(state_path(state_dir))
    if raw is None:
        return None
    if raw.get("kind") != _KIND:
        raise FleetOpError("not_fleet_op", str(state_path(state_dir)))
    return raw


def start(
    state_dir: Path,
    operation_id: str,
    target_release: str,
    origin_release: str,
) -> dict[str, Any]:
    """Старт операции: исходный релиз и порядок vision, sound, main.

    Повтор с тем же operation_id не начинает вторую операцию — возвращает текущий ход.
    """
    if not operation_id:
        raise FleetOpError("bad_operation_id", repr(operation_id))
    if not target_release:
        raise FleetOpError("bad_target_release", repr(target_release))
    if not origin_release:
        raise FleetOpError("bad_origin_release", repr(origin_release))
    if target_release == origin_release:
        raise FleetOpError("same_release", target_release)

    with st.write_lock(state_dir):
        existing = _load_unlocked(state_dir)
        if existing is not None and existing.get("operation_id") == operation_id:
            return existing
        if existing is not None and existing.get("status") == STATUS_RUNNING:
            raise FleetOpError(
                "operation_in_progress",
                f"{existing.get('operation_id')} status={existing.get('status')}",
            )
        record = {
            "schema": _SCHEMA,
            "kind": _KIND,
            "operation_id": operation_id,
            "status": STATUS_RUNNING,
            "target_release": target_release,
            "origin_release": origin_release,
            "order": list(ROLE_ORDER),
            "completed_roles": [],
            "current_role": ROLE_ORDER[0],
            "failed_role": None,
            "fail_detail": None,
            "rollback": [],
        }
        st.save(state_path(state_dir), record)
        return record


def report_role_ok(state_dir: Path, role: str) -> dict[str, Any]:
    """Роль переключена на target. Двигает курсор; все три — status=completed."""
    with st.write_lock(state_dir):
        record = _require_running(state_dir)
        _expect_current(record, role)
        completed = list(record["completed_roles"])
        completed.append(role)
        record["completed_roles"] = completed
        nxt = _next_role(completed)
        record["current_role"] = nxt
        if nxt is None:
            record["status"] = STATUS_COMPLETED
        st.save(state_path(state_dir), record)
        return record


def report_role_failed(state_dir: Path, role: str, detail: str = "") -> dict[str, Any]:
    """Сбой роли: status=failed, откат уже завершённых ролей к origin_release (обратный порядок)."""
    with st.write_lock(state_dir):
        record = _require_running(state_dir)
        _expect_current(record, role)
        completed = list(record["completed_roles"])
        record["status"] = STATUS_FAILED
        record["failed_role"] = role
        record["fail_detail"] = detail
        record["current_role"] = None
        record["rollback"] = _build_rollback(completed, record["origin_release"])
        st.save(state_path(state_dir), record)
        return record


def _build_rollback(
    completed_roles: list[str], origin_release: str
) -> list[dict[str, str]]:
    """Откат затронутых ролей: reverse completed → origin_release. Незатронутые не входят."""
    return [
        {"op": "rollback", "role": role, "release": origin_release}
        for role in reversed(completed_roles)
    ]


def _next_role(completed: list[str]) -> str | None:
    done = set(completed)
    for role in ROLE_ORDER:
        if role not in done:
            return role
    return None


def _load_unlocked(state_dir: Path) -> dict[str, Any] | None:
    path = state_path(state_dir)
    raw = st.load(path)
    if raw is None:
        return None
    if raw.get("kind") != _KIND:
        raise FleetOpError("not_fleet_op", str(path))
    return raw


def _require_running(state_dir: Path) -> dict[str, Any]:
    record = _load_unlocked(state_dir)
    if record is None:
        raise FleetOpError("no_operation", str(state_path(state_dir)))
    if record.get("status") != STATUS_RUNNING:
        raise FleetOpError("not_running", f"status={record.get('status')}")
    return record


def _expect_current(record: dict[str, Any], role: str) -> None:
    if role not in ROLE_ORDER:
        raise FleetOpError("bad_role", role)
    current = record.get("current_role")
    if role != current:
        raise FleetOpError("role_out_of_order", f"expected={current} got={role}")
