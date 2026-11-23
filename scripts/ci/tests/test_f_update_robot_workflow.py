"""Гард F: Update Robot — status wired; update/rollback/cancel → not_wired; без ssh."""

from __future__ import annotations

from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parents[3]
WF = REPO_ROOT / ".github" / "workflows" / "F-Update-Robot.yml"


def _wf() -> dict:
    return yaml.safe_load(WF.read_text(encoding="utf-8"))


def _on(data: dict) -> dict:
    # PyYAML: ключ `on:` становится bool True.
    return data["on"] if "on" in data else data[True]


def test_dispatch_only_no_push_pr_schedule():
    on = _on(_wf())
    assert set(on) == {"workflow_dispatch"}
    assert "schedule" not in on and "push" not in on and "pull_request" not in on


def test_inputs_include_robot_release_update_rollback_cancel():
    inputs = _on(_wf())["workflow_dispatch"]["inputs"]
    assert "robot_id" in inputs and "release" in inputs and "action" in inputs
    actions = inputs["action"]["options"]
    assert actions == ["status", "update", "rollback", "cancel"]
    assert "RBXU100001" in inputs["robot_id"]["options"]
    # Меню хранит mutate-actions, но job не зелёный: not_wired в validate.
    desc = inputs["action"]["description"].lower()
    assert "not_wired" in desc or "not wired" in desc


def test_validate_job_calls_chuangong_validate_inputs_and_no_ssh():
    jobs = _wf()["jobs"]
    assert "validate" in jobs
    steps = jobs["validate"]["steps"]
    runs = "\n".join(str(s.get("run", "")) for s in steps)
    assert "python3 -m chuangong validate-inputs" in runs
    assert "--robot-id" in runs and "--release" in runs and "--action" in runs
    assert "--operation-id" in runs
    jobs_body = WF.read_text(encoding="utf-8").split("jobs:", 1)[1].lower()
    for forbidden in ("ssh ", "sshpass", "scp "):
        assert forbidden not in runs.lower(), forbidden
        assert forbidden not in jobs_body, forbidden
    assert "status" in jobs
    needs = jobs["status"].get("needs")
    assert needs == "validate" or "validate" in (needs or [])
    # status — единственный job к роботу; mutate-actions не имеют success-path job.
    assert list(jobs) == ["validate", "status"]
    assert jobs["status"].get("if") == "${{ inputs.action == 'status' }}"
    # Комментарий/описание: not_wired, не «успех validate = зелёный update».
    full = WF.read_text(encoding="utf-8").lower()
    assert "not_wired" in full
    assert "ssh" not in jobs_body or "no ssh" in full or "ни ssh" in full
