"""Координация Vision → Sound → Main: ход операции в state.json без сети и без compose."""

from __future__ import annotations

import json

import pytest
from chuangong import fleet_op


def _dir(tmp_path):
    return tmp_path / "var" / "chuangong"


def test_start_writes_origin_release_and_role_order(tmp_path):
    state_dir = _dir(tmp_path)
    rec = fleet_op.start(
        state_dir, "op-1", target_release="2026.11.10-1", origin_release="2026.11.09-1"
    )

    assert rec["operation_id"] == "op-1"
    assert rec["status"] == fleet_op.STATUS_RUNNING
    assert rec["origin_release"] == "2026.11.09-1"
    assert rec["target_release"] == "2026.11.10-1"
    assert rec["order"] == ["vision", "sound", "main"]
    assert rec["order"] == list(fleet_op.ROLE_ORDER)
    assert rec["completed_roles"] == []
    assert rec["current_role"] == "vision"
    assert rec["rollback"] == []

    on_disk = json.loads((state_dir / "state.json").read_text(encoding="utf-8"))
    assert on_disk["origin_release"] == "2026.11.09-1"
    assert on_disk["order"] == ["vision", "sound", "main"]
    assert on_disk["kind"] == "fleet_op"


def test_failure_on_second_role_builds_rollback_for_first(tmp_path):
    state_dir = _dir(tmp_path)
    fleet_op.start(
        state_dir,
        "op-fail",
        target_release="2026.11.10-1",
        origin_release="2026.11.09-1",
    )
    fleet_op.report_role_ok(state_dir, "vision")

    rec = fleet_op.report_role_failed(state_dir, "sound", detail="health: unhealthy")

    assert rec["status"] == fleet_op.STATUS_FAILED
    assert rec["failed_role"] == "sound"
    assert rec["fail_detail"] == "health: unhealthy"
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


def test_same_operation_id_does_not_start_second_operation(tmp_path):
    state_dir = _dir(tmp_path)
    first = fleet_op.start(
        state_dir,
        "op-same",
        target_release="2026.11.10-1",
        origin_release="2026.11.09-1",
    )
    fleet_op.report_role_ok(state_dir, "vision")

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
    fleet_op.start(
        state_dir,
        "op-restart",
        target_release="2026.11.10-1",
        origin_release="2026.11.08-1",
    )
    fleet_op.report_role_ok(state_dir, "vision")
    fleet_op.report_role_ok(state_dir, "sound")

    # «Перезапуск координатора»: новый процесс только читает state.json
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
    fleet_op.start(
        state_dir, "op-ok", target_release="2026.11.10-1", origin_release="2026.11.09-1"
    )
    fleet_op.report_role_ok(state_dir, "vision")
    fleet_op.report_role_ok(state_dir, "sound")
    rec = fleet_op.report_role_ok(state_dir, "main")

    assert rec["status"] == fleet_op.STATUS_COMPLETED
    assert rec["completed_roles"] == ["vision", "sound", "main"]
    assert rec["current_role"] is None
    assert rec["rollback"] == []
    assert rec["origin_release"] == "2026.11.09-1"


def test_failure_on_first_role_rollback_empty(tmp_path):
    state_dir = _dir(tmp_path)
    fleet_op.start(
        state_dir,
        "op-first",
        target_release="2026.11.10-1",
        origin_release="2026.11.09-1",
    )
    rec = fleet_op.report_role_failed(state_dir, "vision", detail="stage missing")

    assert rec["status"] == fleet_op.STATUS_FAILED
    assert rec["failed_role"] == "vision"
    assert rec["completed_roles"] == []
    assert rec["rollback"] == []


def test_failure_on_third_rolls_back_sound_then_vision(tmp_path):
    state_dir = _dir(tmp_path)
    fleet_op.start(
        state_dir,
        "op-third",
        target_release="2026.11.10-1",
        origin_release="2026.11.07-1",
    )
    fleet_op.report_role_ok(state_dir, "vision")
    fleet_op.report_role_ok(state_dir, "sound")
    rec = fleet_op.report_role_failed(state_dir, "main", detail="compose up failed")

    assert rec["rollback"] == [
        {"op": "rollback", "role": "sound", "release": "2026.11.07-1"},
        {"op": "rollback", "role": "vision", "release": "2026.11.07-1"},
    ]


def test_role_out_of_order_rejected(tmp_path):
    state_dir = _dir(tmp_path)
    fleet_op.start(
        state_dir,
        "op-order",
        target_release="2026.11.10-1",
        origin_release="2026.11.09-1",
    )
    with pytest.raises(fleet_op.FleetOpError, match="role_out_of_order"):
        fleet_op.report_role_ok(state_dir, "sound")


def test_no_network_and_no_compose(tmp_path, monkeypatch):
    def boom(*_a, **_k):
        raise AssertionError("сеть/compose запрещены")

    monkeypatch.setattr("urllib.request.urlopen", boom)
    monkeypatch.setattr("socket.create_connection", boom)

    state_dir = _dir(tmp_path)
    compose = tmp_path / "opt" / "rob_box" / "docker-compose.yaml"
    compose.parent.mkdir(parents=True)
    compose.write_text("stay\n", encoding="utf-8")

    fleet_op.start(
        state_dir,
        "op-local",
        target_release="2026.11.10-1",
        origin_release="2026.11.09-1",
    )
    fleet_op.report_role_ok(state_dir, "vision")
    fleet_op.report_role_failed(state_dir, "sound", detail="x")
    loaded = fleet_op.load(state_dir)

    assert loaded is not None
    assert loaded["origin_release"] == "2026.11.09-1"
    assert compose.read_text(encoding="utf-8") == "stay\n"
    assert not (tmp_path / "opt" / "rob_box" / "current").exists()
