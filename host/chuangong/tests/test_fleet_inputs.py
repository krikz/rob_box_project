"""Входы F: Update Robot: пустые — отказ; update/rollback/cancel — not_wired; status OK."""

from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

import pytest
from chuangong.fleet_inputs import (
    ACTIONS,
    WIRED_ACTIONS,
    FleetInputError,
    render_error_summary,
    render_summary,
    validate_inputs,
)

ROOT = Path(__file__).resolve().parents[1]


def test_status_accepts_robot_and_optional_release():
    rec = validate_inputs(action="status", robot_id="RBXU100001", release="2026.11.18-1", operation_id="42")
    assert rec == {
        "action": "status",
        "robot_id": "RBXU100001",
        "release": "2026.11.18-1",
        "operation_id": "42",
        "release_required": False,
        "wired": True,
    }
    st = validate_inputs(action="status", robot_id="RBXU100001", release="")
    assert st["release"] is None and st["wired"] is True


def test_update_rollback_cancel_not_wired_even_with_valid_fields():
    """Валидные поля не дают зелёный успех: remote mutate ещё не подключен."""
    for action in ("update", "rollback"):
        with pytest.raises(FleetInputError) as ei:
            validate_inputs(action=action, robot_id="RBXU100001", release="2026.11.18-1", operation_id="7")
        assert ei.value.reason == "not_wired"
        assert action in ei.value.detail
    with pytest.raises(FleetInputError) as ei:
        validate_inputs(action="cancel", robot_id="RBXU100001", release="")
    assert ei.value.reason == "not_wired"
    assert "cancel" in ei.value.detail
    assert WIRED_ACTIONS == frozenset({"status"})


def test_empty_robot_id_rejected_before_not_wired():
    for action in ACTIONS:
        with pytest.raises(FleetInputError) as ei:
            validate_inputs(action=action, robot_id="  ", release="2026.11.18-1")
        assert ei.value.reason == "empty_robot_id"


def test_empty_release_rejected_only_for_update_rollback_before_not_wired():
    for action in ("update", "rollback"):
        with pytest.raises(FleetInputError) as ei:
            validate_inputs(action=action, robot_id="RBXU100001", release="")
        assert ei.value.reason == "empty_release"
        assert action in ei.value.detail


def test_bad_action_rejected():
    with pytest.raises(FleetInputError) as ei:
        validate_inputs(action="deploy", robot_id="RBXU100001", release="x")
    assert ei.value.reason == "bad_action"


def test_strips_whitespace_and_keeps_one_operation_id():
    rec = validate_inputs(
        action=" status ",
        robot_id=" RBXU100001 ",
        release=" 2026.11.18-1 ",
        operation_id=" 99 ",
    )
    assert rec["action"] == "status"
    assert rec["robot_id"] == "RBXU100001"
    assert rec["release"] == "2026.11.18-1"
    assert rec["operation_id"] == "99"
    text = render_summary(rec)
    assert "operation_id: `99`" in text
    assert "robot_id: `RBXU100001`" in text


def test_render_error_summary_not_wired_is_explicit():
    text = render_error_summary(
        reason="not_wired",
        detail="update: remote path not connected yet",
        action="update",
        robot_id="RBXU100001",
        release="2026.11.18-1",
        operation_id="7",
    )
    assert "reason: `not_wired`" in text
    assert "ещё не подключен" in text
    assert "exit ≠ 0" in text
    assert "action: `update`" in text


def test_cli_validate_inputs_exit_codes(tmp_path):
    out = tmp_path / "out"
    summary = tmp_path / "sum.md"
    env = {**dict(**{k: v for k, v in __import__("os").environ.items()}), "PYTHONPATH": str(ROOT)}

    def run(argv: list[str]) -> subprocess.CompletedProcess:
        return subprocess.run(
            [sys.executable, "-m", "chuangong", "validate-inputs", *argv],
            cwd=ROOT,
            env=env,
            capture_output=True,
            text=True,
            encoding="utf-8",
        )

    ok = run(
        [
            "--action",
            "status",
            "--robot-id",
            "RBXU100001",
            "--release",
            "2026.11.18-1",
            "--operation-id",
            "7",
            "--github-output",
            str(out),
            "--summary",
            str(summary),
        ]
    )
    assert ok.returncode == 0, ok.stderr
    payload = json.loads(ok.stdout)
    assert payload["operation_id"] == "7" and payload["action"] == "status"
    assert payload.get("wired") is True
    assert "operation_id=7" in out.read_text(encoding="utf-8")
    assert "operation_id: `7`" in summary.read_text(encoding="utf-8")

    bad_empty = run(["--action", "rollback", "--robot-id", "RBXU100001", "--release", ""])
    assert bad_empty.returncode == 2
    err = json.loads(bad_empty.stdout)
    assert err["reason"] == "empty_release"

    unwired_sum = tmp_path / "unwired.md"
    for action, extra in (
        ("update", ["--release", "2026.11.18-1"]),
        ("rollback", ["--release", "2026.11.18-1"]),
        ("cancel", []),
    ):
        bad = run(
            [
                "--action",
                action,
                "--robot-id",
                "RBXU100001",
                *extra,
                "--operation-id",
                "9",
                "--summary",
                str(unwired_sum),
            ]
        )
        assert bad.returncode == 2, (action, bad.stdout, bad.stderr)
        body = json.loads(bad.stdout)
        assert body["ok"] is False and body["reason"] == "not_wired"
        assert action in body["detail"]

    text = unwired_sum.read_text(encoding="utf-8")
    assert "reason: `not_wired`" in text
    assert "ещё не подключен" in text
