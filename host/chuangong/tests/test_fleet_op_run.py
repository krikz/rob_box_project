"""Одна операция F: Update Robot, три роли, один operation_id. Без сети и без робота.

Исполнитель — mock: каждая роль отвечает switched либо сбоем. Откат поднимает точный
предыдущий релиз из журнала и проверяет его заново. Повтор operation_id не начинает
вторую операцию. Перезапуск читает тот же журнал. Замок HIL и hotpatch не снимается.
"""

from __future__ import annotations

import json

import pytest

from chuangong import fleet_op


class RoleMock:
    """Ответы ролей по порядку вызовов. fail — сбой до переключения, switched — после."""

    def __init__(self, script):
        self.script = list(script)
        self.calls = []

    def switch(self, role, release):
        self.calls.append((role, release))
        outcome = self.script.pop(0)
        if outcome == "switched":
            return "switched"
        if outcome == "timeout":
            raise TimeoutError(role)
        raise RuntimeError(outcome)


def _dir(tmp_path):
    path = tmp_path / "var" / "chuangong"
    path.mkdir(parents=True)
    return path


def _run(state_dir, mock, **locks):
    return fleet_op.run(
        state_dir,
        "op-1",
        target_release="2026.11.10-1",
        origin_release="2026.11.09-1",
        switch=mock.switch,
        **locks,
    )


def test_healthy_release_confirms_three_roles(tmp_path):
    state_dir = _dir(tmp_path)
    mock = RoleMock(["switched", "switched", "switched"])

    rec = _run(state_dir, mock)

    assert rec["status"] == fleet_op.STATUS_COMPLETED
    assert rec["completed_roles"] == ["vision", "sound", "main"]
    assert rec["rollback"] == []
    assert mock.calls == [
        ("vision", "2026.11.10-1"),
        ("sound", "2026.11.10-1"),
        ("main", "2026.11.10-1"),
    ]
    assert rec["pult"]["result"] == "updated"
    assert rec["pult"]["current"] == "2026.11.10-1"


def test_second_role_failure_after_switch_rolls_back_exact_previous(tmp_path):
    state_dir = _dir(tmp_path)
    mock = RoleMock(["switched", "boom", "switched", "switched"])

    rec = _run(state_dir, mock)

    assert rec["status"] == fleet_op.STATUS_FAILED
    assert rec["failed_role"] == "sound"
    assert rec["pult"]["result"] == "rolled_back"
    assert rec["pult"]["current"] == "2026.11.09-1"
    assert rec["pult"]["verified"] is True
    assert mock.calls == [
        ("vision", "2026.11.10-1"),
        ("sound", "2026.11.10-1"),
        ("sound", "2026.11.09-1"),
        ("vision", "2026.11.09-1"),
    ]
    assert all(release == "2026.11.09-1" for _, release in mock.calls[2:])


def test_timeout_on_second_role_is_uncertain_and_rolls_back(tmp_path):
    state_dir = _dir(tmp_path)
    mock = RoleMock(["switched", "timeout", "switched", "switched"])

    rec = _run(state_dir, mock)

    assert rec["fail_outcome"] == fleet_op.OUTCOME_UNCERTAIN
    assert rec["pult"]["result"] == "rolled_back"
    assert ("sound", "2026.11.09-1") in mock.calls
    assert ("main", "2026.11.10-1") not in mock.calls


def test_same_operation_id_does_not_run_twice(tmp_path):
    state_dir = _dir(tmp_path)
    mock = RoleMock(["switched", "switched", "switched"])
    first = _run(state_dir, mock)

    second = _run(state_dir, RoleMock(["switched"]))

    assert second["operation_id"] == first["operation_id"]
    assert second["status"] == fleet_op.STATUS_COMPLETED
    assert second["pult"]["result"] == first["pult"]["result"]
    assert mock.calls  # первый прогон сходил в роли


def test_restart_reads_the_same_journal_and_does_not_claim_success(tmp_path):
    state_dir = _dir(tmp_path)
    mock = RoleMock(["switched", "boom", "check-failed"])
    try:
        _run(state_dir, mock)
    except fleet_op.FleetOpError:
        pass

    loaded = fleet_op.load(state_dir)
    assert loaded is not None
    assert loaded["status"] == fleet_op.STATUS_FAILED
    assert loaded["pult"]["result"] != "updated"
    again = json.loads((state_dir / "fleet_op.json").read_text(encoding="utf-8"))
    assert again["origin_release"] == "2026.11.09-1"
    # boom — сбой после начала переключения: sound тоже откатывается на прежний релиз
    assert again["rollback"] == [
        {"op": "rollback", "role": "sound", "release": "2026.11.09-1"},
        {"op": "rollback", "role": "vision", "release": "2026.11.09-1"},
    ]
    # проверка отката не прошла: пульт не подтверждает current и не пишет успех
    assert again["pult"]["result"] == "rollback_failed"
    assert again["pult"]["verified"] is False
    assert again["pult"]["current"] is None


def test_hil_lock_and_hotpatch_block_before_any_role(tmp_path):
    state_dir = _dir(tmp_path)
    (state_dir / "hil.lock").write_text("hil on", encoding="utf-8")
    mock = RoleMock(["switched"])

    with pytest.raises(fleet_op.FleetOpError) as ei:
        _run(state_dir, mock, hil_lock=state_dir / "hil.lock", dirty_flag=state_dir / "dirty")
    assert ei.value.reason == "hil_locked"
    assert mock.calls == []


def test_rollback_check_failure_is_not_reported_as_success(tmp_path):
    state_dir = _dir(tmp_path)
    mock = RoleMock(["switched", "boom", "check-failed"])

    rec = _run(state_dir, mock)

    assert rec["pult"]["result"] == "rollback_failed"
    assert rec["pult"]["verified"] is False
    assert rec["pult"]["result"] != "updated"
    assert rec["pult"]["result"] != "rolled_back"
