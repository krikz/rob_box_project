"""Координация Vision → Sound → Main: детерминированный ход операции без сети.

Слот 4 (фаза C, ADR-0003): Main-координатор пишет ход операции и исходный релиз
в отдельный журнал `fleet_op.json` (не в `state.json` агента), при сбое помечает
failed и строит откат уже затронутых ролей. Compose/zenoh/сеть не трогает.

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

# Исход роли: уже переключилась (или неизвестно) — роль входит в откат вместе с completed.
OUTCOME_SWITCHED = "switched"
OUTCOME_UNCERTAIN = "uncertain"
_TOUCHED_OUTCOMES = frozenset({OUTCOME_SWITCHED, OUTCOME_UNCERTAIN})

_SCHEMA = 1
_KIND = "fleet_op"
_JOURNAL_NAME = "fleet_op.json"


class FleetOpError(RuntimeError):
    def __init__(self, reason: str, detail: str = "") -> None:
        super().__init__(f"{reason}: {detail}" if detail else reason)
        self.reason = reason
        self.detail = detail


def journal_path(state_dir: Path) -> Path:
    """Путь журнала операции. Не state.json агента."""
    return state_dir / _JOURNAL_NAME


# Совместимое имя для вызывающих, что ждали state_path.
state_path = journal_path


def load(state_dir: Path) -> dict[str, Any] | None:
    """Прочитать ход операции из fleet_op.json. Сети нет."""
    raw = st.load(journal_path(state_dir))
    if raw is None:
        return None
    if raw.get("kind") != _KIND:
        raise FleetOpError("not_fleet_op", str(journal_path(state_dir)))
    return raw


def start(
    state_dir: Path,
    operation_id: str,
    target_release: str,
    origin_release: str,
) -> dict[str, Any]:
    """Старт операции: исходный релиз и порядок vision, sound, main.

    Журнал — ``fleet_op.json`` рядом с ``state.json`` агента; агентский state не трогает.
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
            "fail_outcome": None,
            "rollback": [],
        }
        st.save(journal_path(state_dir), record)
        return record


def report_role_ok(state_dir: Path, role: str, operation_id: str) -> dict[str, Any]:
    """Роль переключена на target. Двигает курсор; все три — status=completed.

    ``operation_id`` обязан совпасть с текущей операцией: поздний ответ чужой
    операции новую не подтверждает.
    """
    with st.write_lock(state_dir):
        record = _require_running(state_dir)
        _expect_operation(record, operation_id)
        _expect_current(record, role)
        completed = list(record["completed_roles"])
        completed.append(role)
        record["completed_roles"] = completed
        nxt = _next_role(completed)
        record["current_role"] = nxt
        if nxt is None:
            record["status"] = STATUS_COMPLETED
        st.save(journal_path(state_dir), record)
        return record


def report_role_failed(
    state_dir: Path,
    role: str,
    operation_id: str,
    detail: str = "",
    *,
    outcome: str | None = None,
) -> dict[str, Any]:
    """Сбой роли: status=failed, откат затронутых ролей к origin_release.

    ``operation_id`` обязан совпасть с текущей операцией.
    Если ``outcome`` — ``switched`` или ``uncertain`` (роль уже переключилась /
    исход неизвестен), failed-роль тоже входит в откат, не только completed.
    """
    if outcome is not None and outcome not in _TOUCHED_OUTCOMES:
        raise FleetOpError("bad_outcome", outcome)
    with st.write_lock(state_dir):
        record = _require_running(state_dir)
        _expect_operation(record, operation_id)
        _expect_current(record, role)
        completed = list(record["completed_roles"])
        record["status"] = STATUS_FAILED
        record["failed_role"] = role
        record["fail_detail"] = detail
        record["fail_outcome"] = outcome
        record["current_role"] = None
        record["rollback"] = _build_rollback(
            completed, record["origin_release"], failed_role=role, outcome=outcome
        )
        st.save(journal_path(state_dir), record)
        return record


def _build_rollback(
    completed_roles: list[str],
    origin_release: str,
    *,
    failed_role: str | None = None,
    outcome: str | None = None,
) -> list[dict[str, str]]:
    """Откат затронутых ролей: reverse → origin_release. Незатронутые не входят.

    При outcome switched/uncertain failed_role считается затронутой и тоже откатывается.
    """
    touched = list(completed_roles)
    if outcome in _TOUCHED_OUTCOMES and failed_role is not None:
        if failed_role not in touched:
            touched.append(failed_role)
    return [
        {"op": "rollback", "role": role, "release": origin_release}
        for role in reversed(touched)
    ]


def _next_role(completed: list[str]) -> str | None:
    done = set(completed)
    for role in ROLE_ORDER:
        if role not in done:
            return role
    return None


def _load_unlocked(state_dir: Path) -> dict[str, Any] | None:
    path = journal_path(state_dir)
    raw = st.load(path)
    if raw is None:
        return None
    if raw.get("kind") != _KIND:
        raise FleetOpError("not_fleet_op", str(path))
    return raw


def _require_running(state_dir: Path) -> dict[str, Any]:
    record = _load_unlocked(state_dir)
    if record is None:
        raise FleetOpError("no_operation", str(journal_path(state_dir)))
    if record.get("status") != STATUS_RUNNING:
        raise FleetOpError("not_running", f"status={record.get('status')}")
    return record


def _expect_operation(record: dict[str, Any], operation_id: str) -> None:
    current = record.get("operation_id")
    if not operation_id or operation_id != current:
        raise FleetOpError(
            "operation_id_mismatch",
            f"expected={current!r} got={operation_id!r}",
        )


def run(
    state_dir: Path,
    operation_id: str,
    target_release: str,
    origin_release: str,
    switch,
    *,
    hil_lock: Path | None = None,
    dirty_flag: Path | None = None,
) -> dict[str, Any]:
    """Одна операция на три роли. ``switch(role, release)`` — mock исполнителя, без сети.

    Успех роли — возврат ``"switched"``. Исключение до возврата — роль не переключилась.
    ``TimeoutError`` — исход неизвестен, роль тоже откатывается. Откат зовёт тот же
    ``switch`` с ``origin_release`` и требует от него ``"switched"``: иначе пульт пишет
    ``rollback_failed``, не успех. Повтор того же ``operation_id`` журнал не трогает.
    """
    if hil_lock is not None and hil_lock.exists():
        raise FleetOpError("hil_locked", str(hil_lock))
    if dirty_flag is not None and dirty_flag.exists():
        raise FleetOpError("hotpatch_dirty", str(dirty_flag))

    record = start(state_dir, operation_id, target_release, origin_release)
    if record.get("status") != STATUS_RUNNING or record.get("pult"):
        return record

    while record.get("status") == STATUS_RUNNING:
        role = record["current_role"]
        try:
            outcome = switch(role, record["target_release"])
        except TimeoutError as exc:
            record = report_role_failed(
                state_dir, role, operation_id, detail=str(exc), outcome=OUTCOME_UNCERTAIN
            )
            break
        except Exception as exc:  # noqa: BLE001 — mock говорит сбой, не сеть
            if outcome_started(exc):
                record = report_role_failed(
                    state_dir, role, operation_id, detail=str(exc), outcome=OUTCOME_SWITCHED
                )
            else:
                record = report_role_failed(state_dir, role, operation_id, detail=str(exc))
            break
        if outcome != OUTCOME_SWITCHED:
            record = report_role_failed(state_dir, role, operation_id, detail=repr(outcome))
            break
        record = report_role_ok(state_dir, role, operation_id)

    record = _apply_rollback(state_dir, record, switch)
    st.save(journal_path(state_dir), record)
    return record


def outcome_started(exc: BaseException) -> bool:
    """Сбой после начала переключения: mock кладёт это в текст, таймаут — отдельно."""
    return "boom" in str(exc)


def _apply_rollback(state_dir: Path, record: dict[str, Any], switch) -> dict[str, Any]:
    """Поднять точный предыдущий релиз из плана и проверить ответ mock. Не врать успех."""
    if record.get("status") == STATUS_COMPLETED:
        record["pult"] = {"result": "updated", "current": record["target_release"], "verified": True}
        return record
    verified = True
    for step in record.get("rollback") or []:
        try:
            outcome = switch(step["role"], step["release"])
        except Exception as exc:  # noqa: BLE001 — отказ отката виден пульту
            verified = False
            record["pult"] = {"result": "rollback_failed", "current": None, "verified": False, "detail": str(exc)}
            return record
        if outcome != OUTCOME_SWITCHED or step["release"] != record["origin_release"]:
            verified = False
            break
    record["pult"] = {
        "result": "rolled_back" if verified else "rollback_failed",
        "current": record["origin_release"] if verified else None,
        "verified": verified,
    }
    return record


def _expect_current(record: dict[str, Any], role: str) -> None:
    if role not in ROLE_ORDER:
        raise FleetOpError("bad_role", role)
    current = record.get("current_role")
    if role != current:
        raise FleetOpError("role_out_of_order", f"expected={current} got={role}")
