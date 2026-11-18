"""Входы F: Update Robot: пустые robot_id/release — отказ; operation_id общий."""

from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

import pytest
from chuangong.fleet_inputs import ACTIONS, FleetInputError, render_summary, validate_inputs

ROOT = Path(__file__).resolve().parents[1]


def test_update_and_rollback_require_robot_and_release():
    for action in ("update", "rollback"):
        rec = validate_inputs(action=action, robot_id="RBXU100001", release="2026.11.18-1", operation_id="42")
        assert rec == {
            "action": action,
            "robot_id": "RBXU100001",
            "release": "2026.11.18-1",
            "operation_id": "42",
            "release_required": True,
        }


def test_empty_robot_id_rejected_for_every_action():
    for action in ACTIONS:
        with pytest.raises(FleetInputError) as ei:
            validate_inputs(action=action, robot_id="  ", release="2026.11.18-1")
        assert ei.value.reason == "empty_robot_id"


def test_empty_release_rejected_only_for_update_rollback():
    for action in ("update", "rollback"):
        with pytest.raises(FleetInputError) as ei:
            validate_inputs(action=action, robot_id="RBXU100001", release="")
        assert ei.value.reason == "empty_release"
        assert action in ei.value.detail

    st = validate_inputs(action="status", robot_id="RBXU100001", release="")
    assert st["release"] is None and st["release_required"] is False
    can = validate_inputs(action="cancel", robot_id="RBXU100001", release="  ")
    assert can["release"] is None and can["action"] == "cancel"


def test_bad_action_rejected():
    with pytest.raises(FleetInputError) as ei:
        validate_inputs(action="deploy", robot_id="RBXU100001", release="x")
    assert ei.value.reason == "bad_action"


def test_strips_whitespace_and_keeps_one_operation_id():
    rec = validate_inputs(
        action=" update ",
        robot_id=" RBXU100001 ",
        release=" 2026.11.18-1 ",
        operation_id=" 99 ",
    )
    assert rec["action"] == "update"
    assert rec["robot_id"] == "RBXU100001"
    assert rec["release"] == "2026.11.18-1"
    assert rec["operation_id"] == "99"
    text = render_summary(rec)
    assert "operation_id: `99`" in text
    assert "robot_id: `RBXU100001`" in text
    assert "без ssh" in text


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
            "update",
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
    assert payload["operation_id"] == "7" and payload["action"] == "update"
    assert "operation_id=7" in out.read_text(encoding="utf-8")
    assert "operation_id: `7`" in summary.read_text(encoding="utf-8")

    bad = run(["--action", "rollback", "--robot-id", "RBXU100001", "--release", ""])
    assert bad.returncode == 2
    err = json.loads(bad.stdout)
    assert err["reason"] == "empty_release"
