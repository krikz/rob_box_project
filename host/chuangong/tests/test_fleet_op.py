"""Координация Vision → Sound → Main: ход операции в fleet_op.json без сети и без compose."""

from __future__ import annotations

import json

import pytest

from chuangong import fleet_op


def _dir(tmp_path):
    return tmp_path / "var" / "chuangong"


def _start(state_dir, op_id="op-1", target="2026.11.10-1", origin="2026.11.09-1"):
    return fleet_op.start(
        state_dir, op_id, target_release=target, origin_release=origin
    )


def test_start_writes_origin_release_and_role_order(tmp_path):
    state_dir = _dir(tmp_path)
    rec = _start(state_dir, "op-1")

    assert rec["operation_id"] == "op-1"
    assert rec["status"] == fleet_op.STATUS_RUNNING
    assert rec["origin_release"] == "2026.11.09-1"
    assert rec["target_release"] == "2026.11.10-1"
    assert rec["order"] == ["vision", "sound", "main"]
    assert rec["order"] == list(fleet_op.ROLE_ORDER)
    assert rec["completed_roles"] == []
    assert rec["current_role"] == "vision"
    assert rec["rollback"] == []

    journal = state_dir / "fleet_op.json"
    on_disk = json.loads(journal.read_text(encoding="utf-8"))
    assert on_disk["origin_release"] == "2026.11.09-1"
    assert on_disk["order"] == ["vision", "sound", "main"]
    assert on_disk["kind"] == "fleet_op"
    assert not (state_dir / "state.json").exists()


def test_start_alongside_existing_agent_state_json(tmp_path):
    """Журнал операции — отдельный файл: agent state.json не блокирует start."""
    state_dir = _dir(tmp_path)
    state_dir.mkdir(parents=True)
    agent_state = {
        "schema": 1,
        "agent_version": "0.2.3",
        "robot_id": "RBXU100001",
        "node": "main",
        "channel": "dev",
        "mode": "observe",
        "state": "available",
        "available": "2026.11.09-1",
        "current": None,
        "previous": None,
        "staged": None,
    }
    (state_dir / "state.json").write_text(
        json.dumps(agent_state, ensure_ascii=False, indent=2), encoding="utf-8"
    )

    rec = _start(state_dir, "op-agent")

    assert rec["kind"] == "fleet_op"
    assert rec["operation_id"] == "op-agent"
    assert rec["status"] == fleet_op.STATUS_RUNNING
    on_disk_agent = json.loads((state_dir / "state.json").read_text(encoding="utf-8"))
    assert on_disk_agent["state"] == "available"
    assert on_disk_agent.get("kind") is None
    assert "operation_id" not in on_disk_agent
    journal = json.loads((state_dir / "fleet_op.json").read_text(encoding="utf-8"))
    assert journal["operation_id"] == "op-agent"
    assert journal["kind"] == "fleet_op"


def test_failure_on_second_role_builds_rollback_for_first(tmp_path):
    state_dir = _dir(tmp_path)
    _start(state_dir, "op-fail")
    fleet_op.report_role_ok(state_dir, "vision", "op-fail")

    rec = fleet_op.report_role_failed(
        state_dir, "sound", "op-fail", detail="health: unhealthy"
    )

    assert rec["status"] == fleet_op.STATUS_FAILED
    assert rec["failed_role"] == "sound"
    assert rec["fail_detail"] == "health: unhealthy"
    assert rec["fail_outcome"] is None
    assert rec["completed_roles"] == ["vision"]
    assert rec["current_role"] is None
    assert rec["origin_release"] == "2026.11.09-1"
    assert rec["rollback"] == [
        {"op": "rollback", "role": "vision", "release": "2026.11.09-1"},
    ]
    on_disk = fleet_op.load(state_dir)
    assert on_disk is not None
    assert on_disk["status"] == "failed"
    assert on_disk["rollback"] == rec["rollback"]


@pytest.mark.parametrize("outcome", ["switched", "uncertain"])
def test_sound_fail_after_switched_rolls_back_sound_and_vision(tmp_path, outcome):
    """Сбой Sound после переключения (switched/uncertain) — в откате Sound и Vision."""
    state_dir = _dir(tmp_path)
    _start(state_dir, "op-sw", origin="2026.11.08-1")
    fleet_op.report_role_ok(state_dir, "vision", "op-sw")

    rec = fleet_op.report_role_failed(
        state_dir,
        "sound",
        "op-sw",
        detail="health: unhealthy after switch",
        outcome=outcome,
    )

    assert rec["status"] == fleet_op.STATUS_FAILED
    assert rec["failed_role"] == "sound"
    assert rec["fail_outcome"] == outcome
    assert rec["completed_roles"] == ["vision"]
    assert rec["rollback"] == [
        {"op": "rollback", "role": "sound", "release": "2026.11.08-1"},
        {"op": "rollback", "role": "vision", "release": "2026.11.08-1"},
    ]
    on_disk = fleet_op.load(state_dir)
    assert on_disk is not None
    assert on_disk["rollback"] == rec["rollback"]
    assert on_disk["fail_outcome"] == outcome


def test_late_response_foreign_operation_id_does_not_confirm(tmp_path):
    """Поздний ответ роли с чужим operation_id новую операцию не подтверждает."""
    state_dir = _dir(tmp_path)
    _start(state_dir, "op-new")

    with pytest.raises(fleet_op.FleetOpError, match="operation_id_mismatch") as ei:
        fleet_op.report_role_ok(state_dir, "vision", "op-old-late")
    assert ei.value.reason == "operation_id_mismatch"

    loaded = fleet_op.load(state_dir)
    assert loaded is not None
    assert loaded["operation_id"] == "op-new"
    assert loaded["completed_roles"] == []
    assert loaded["current_role"] == "vision"
    assert loaded["status"] == fleet_op.STATUS_RUNNING

    with pytest.raises(fleet_op.FleetOpError, match="operation_id_mismatch"):
        fleet_op.report_role_failed(state_dir, "vision", "op-old-late", detail="stale")

    loaded2 = fleet_op.load(state_dir)
    assert loaded2 is not None
    assert loaded2["status"] == fleet_op.STATUS_RUNNING
    assert loaded2["completed_roles"] == []
    assert loaded2["failed_role"] is None
    assert loaded2["rollback"] == []


def test_same_operation_id_does_not_start_second_operation(tmp_path):
    state_dir = _dir(tmp_path)
    first = _start(state_dir, "op-same")
    fleet_op.report_role_ok(state_dir, "vision", "op-same")

    again = fleet_op.start(
        state_dir,
        "op-same",
        target_release="2026.11.99-1",
        origin_release="2026.11.00-1",
    )

    assert again["operation_id"] == "op-same"
    assert again["origin_release"] == "2026.11.09-1"
    assert again["target_release"] == "2026.11.10-1"
    assert again["completed_roles"] == ["vision"]
    assert again["current_role"] == "sound"
    assert again["status"] == fleet_op.STATUS_RUNNING
    assert again == fleet_op.load(state_dir)
    assert first["operation_id"] == again["operation_id"]


def test_restart_reads_progress_and_keeps_origin_release(tmp_path):
    state_dir = _dir(tmp_path)
    _start(state_dir, "op-restart", origin="2026.11.08-1")
    fleet_op.report_role_ok(state_dir, "vision", "op-restart")
    fleet_op.report_role_ok(state_dir, "sound", "op-restart")

    # «Перезапуск координатора»: новый процесс только читает журнал
    loaded = fleet_op.load(state_dir)

    assert loaded is not None
    assert loaded["operation_id"] == "op-restart"
    assert loaded["origin_release"] == "2026.11.08-1"
    assert loaded["target_release"] == "2026.11.10-1"
    assert loaded["completed_roles"] == ["vision", "sound"]
    assert loaded["current_role"] == "main"
    assert loaded["status"] == fleet_op.STATUS_RUNNING
    assert loaded["order"] == ["vision", "sound", "main"]


def test_all_roles_ok_marks_completed(tmp_path):
    state_dir = _dir(tmp_path)
    _start(state_dir, "op-ok")
    fleet_op.report_role_ok(state_dir, "vision", "op-ok")
    fleet_op.report_role_ok(state_dir, "sound", "op-ok")
    rec = fleet_op.report_role_ok(state_dir, "main", "op-ok")

    assert rec["status"] == fleet_op.STATUS_COMPLETED
    assert rec["completed_roles"] == ["vision", "sound", "main"]
    assert rec["current_role"] is None
    assert rec["rollback"] == []
    assert rec["origin_release"] == "2026.11.09-1"


def test_failure_on_first_role_rollback_empty(tmp_path):
    state_dir = _dir(tmp_path)
    _start(state_dir, "op-first")
    rec = fleet_op.report_role_failed(
        state_dir, "vision", "op-first", detail="stage missing"
    )

    assert rec["status"] == fleet_op.STATUS_FAILED
    assert rec["failed_role"] == "vision"
    assert rec["completed_roles"] == []
    assert rec["rollback"] == []


def test_failure_on_first_role_switched_rolls_back_self(tmp_path):
    state_dir = _dir(tmp_path)
    _start(state_dir, "op-first-sw", origin="2026.11.07-1")
    rec = fleet_op.report_role_failed(
        state_dir,
        "vision",
        "op-first-sw",
        detail="health after switch",
        outcome="switched",
    )

    assert rec["rollback"] == [
        {"op": "rollback", "role": "vision", "release": "2026.11.07-1"},
    ]
    assert rec["fail_outcome"] == "switched"


def test_failure_on_third_rolls_back_sound_then_vision(tmp_path):
    state_dir = _dir(tmp_path)
    _start(state_dir, "op-third", origin="2026.11.07-1")
    fleet_op.report_role_ok(state_dir, "vision", "op-third")
    fleet_op.report_role_ok(state_dir, "sound", "op-third")
    rec = fleet_op.report_role_failed(
        state_dir, "main", "op-third", detail="compose up failed"
    )

    assert rec["rollback"] == [
        {"op": "rollback", "role": "sound", "release": "2026.11.07-1"},
        {"op": "rollback", "role": "vision", "release": "2026.11.07-1"},
    ]


def test_role_out_of_order_rejected(tmp_path):
    state_dir = _dir(tmp_path)
    _start(state_dir, "op-order")
    with pytest.raises(fleet_op.FleetOpError, match="role_out_of_order"):
        fleet_op.report_role_ok(state_dir, "sound", "op-order")


def test_no_network_and_no_compose(tmp_path, monkeypatch):
    def boom(*_a, **_k):
        raise AssertionError("сеть/compose запрещены")

    monkeypatch.setattr("urllib.request.urlopen", boom)
    monkeypatch.setattr("socket.create_connection", boom)

    state_dir = _dir(tmp_path)
    compose = tmp_path / "opt" / "rob_box" / "docker-compose.yaml"
    compose.parent.mkdir(parents=True)
    compose.write_text("stay\n", encoding="utf-8")

    _start(state_dir, "op-local")
    fleet_op.report_role_ok(state_dir, "vision", "op-local")
    fleet_op.report_role_failed(state_dir, "sound", "op-local", detail="x")
    loaded = fleet_op.load(state_dir)

    assert loaded is not None
    assert loaded["origin_release"] == "2026.11.09-1"
    assert compose.read_text(encoding="utf-8") == "stay\n"
    assert not (tmp_path / "opt" / "rob_box" / "current").exists()
