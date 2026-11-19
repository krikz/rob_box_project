"""Фаза C: activate/rollback по умолчанию — план (execute=false, файлы не меняются);
``execute=True`` зовёт исполнителя роли. Подпись и registry не нужны: релиз передаётся
напрямую, исполнитель — заглушка, которая пишет факт вызова."""

from __future__ import annotations

import json

import pytest

from chuangong.activation import ActivationMachine
from chuangong.agent import Config, Deps
from chuangong.state import Mode


class _Health:
    def __init__(self, status="not_verified", detail="health not executed in dry-run"):
        self.status = status
        self.detail = detail

    def check(self, role, release, dry_run):
        from chuangong.activation import HealthResult

        return HealthResult(self.status, self.detail)


class _Executor:
    """Заглушка role_switch.execute_role: пишет вызов, файлы не трогает."""

    def __init__(self, state="role_switched"):
        self.calls = []
        self.state = state

    def __call__(self, release, role, *, docker, project_dir, state_dir, previous=None, health_after=None):
        self.calls.append(
            {
                "release": release["release"],
                "role": role,
                "previous": previous,
                "project_dir": str(project_dir),
                "state_dir": str(state_dir),
            }
        )
        return {"state": self.state, "execute": True, "role": role, "release": release["release"]}


def _release(name):
    from chuangong import release_format as rf

    digest = "sha256:" + "ab" * 32
    roles = {}
    for role in rf.ROLES:
        roles[role] = {
            "bundle": {"file": rf.bundle_file_name(role, name), "sha256": "cd" * 32, "size": 1},
            "images": {f"{role}-svc": f"10.1.1.249:5000/krikz/rob_box@{digest}"},
            "external_images": [],
            "resource_pack": None,
            "migrations": [],
        }
    return {
        "schema": 1,
        "release": name,
        "channel": "dev",
        "git": "abc1234",
        "created": "2026-11-09T12:00:00Z",
        "image_tag": "dev-abc1234",
        "docker_tag": "dev",
        "roles": roles,
        "compat": {"min_previous": None},
    }


def _cfg(tmp_path):
    return Config(
        robot_id="RBXU100001",
        node="vision",
        channel="dev",
        registry_url="http://registry.invalid",
        pubkey=tmp_path / "release.pub",
        state_dir=tmp_path / "state",
        releases_root=tmp_path / "releases",
        legacy_root=tmp_path / "legacy",
        zenoh_url="http://127.0.0.1:9",
        mode=Mode.OBSERVE,
        dry_run=True,
        hil_lock=tmp_path / "hil.lock",
    )


class _Docker:
    def hotpatched_containers(self):
        return []

    def running_images(self):
        return []


def _machine(cfg, executor):
    return ActivationMachine(
        cfg, Deps(registry=None, docker=_Docker(), zenoh=None), health=_Health(), executor=executor
    )


def test_activate_dry_run_plans_without_touching_files_or_executor(tmp_path, monkeypatch):
    cfg = _cfg(tmp_path)
    cfg.state_dir.mkdir()
    (cfg.state_dir / "state.json").write_text(
        json.dumps({"current": "2026.11.08-1", "previous": None}), encoding="utf-8"
    )
    before = (cfg.state_dir / "state.json").read_bytes()
    executor = _Executor()
    monkeypatch.setattr(
        "chuangong.activation.fetch_release", lambda *args, **kwargs: _release("2026.11.09-1")
    )
    monkeypatch.setattr(
        "chuangong.activation.ActivationMachine._preflight",
        lambda self, record, release: (object(), {}),
    )

    rec = _machine(cfg, executor).activate("2026.11.09-1", execute=False)

    assert rec["state"] == "activation_planned"
    assert rec["plan"] and all(step["execute"] is False for step in rec["plan"])
    assert executor.calls == []
    assert (cfg.state_dir / "state.json").read_bytes() == before


def test_activate_execute_calls_executor_for_each_role(tmp_path, monkeypatch):
    cfg = _cfg(tmp_path)
    executor = _Executor()
    monkeypatch.setattr(
        "chuangong.activation.fetch_release", lambda *args, **kwargs: _release("2026.11.09-1")
    )
    monkeypatch.setattr(
        "chuangong.activation.ActivationMachine._preflight",
        lambda self, record, release: (object(), {}),
    )
    (cfg.state_dir).mkdir()
    (cfg.state_dir / "state.json").write_text(
        json.dumps({"current": "2026.11.08-1", "previous": None}), encoding="utf-8"
    )

    rec = _machine(cfg, executor).activate("2026.11.09-1", execute=True)

    assert rec["state"] == "activation_executed"
    assert [call["role"] for call in executor.calls] == ["vision", "sound", "main"]
    assert all(call["release"] == "2026.11.09-1" for call in executor.calls)
    assert all(call["previous"] == "2026.11.08-1" for call in executor.calls)


def test_rollback_execute_calls_executor_with_previous(tmp_path, monkeypatch):
    cfg = _cfg(tmp_path)
    executor = _Executor()
    cfg.state_dir.mkdir()
    (cfg.state_dir / "state.json").write_text(
        json.dumps({"current": "2026.11.10-1", "previous": "2026.11.09-1"}), encoding="utf-8"
    )
    monkeypatch.setattr(
        "chuangong.activation.fetch_release", lambda *args, **kwargs: _release("2026.11.09-1")
    )
    monkeypatch.setattr(
        "chuangong.activation.ActivationMachine._preflight",
        lambda self, record, release: (object(), {}),
    )

    rec = _machine(cfg, executor).rollback(execute=True)

    assert rec["state"] == "rollback_executed"
    assert [call["role"] for call in executor.calls] == ["vision", "sound", "main"]
    assert all(call["release"] == "2026.11.09-1" for call in executor.calls)


@pytest.mark.parametrize(
    "state", ("blocked", "role_rolled_back", "rollback_failed", "role_switch_planned", "rollback_planned")
)
def test_execute_requires_explicit_role_switched(tmp_path, monkeypatch, state):
    cfg = _cfg(tmp_path)
    executor = _Executor(state)
    monkeypatch.setattr("chuangong.activation.fetch_release", lambda *args, **kwargs: _release("2026.11.09-1"))
    monkeypatch.setattr(
        "chuangong.activation.ActivationMachine._preflight",
        lambda self, record, release: (object(), {}),
    )
    cfg.state_dir.mkdir()
    cfg.state_file.write_text(json.dumps({"current": "2026.11.08-1", "previous": None}), encoding="utf-8")

    rec = _machine(cfg, executor).activate("2026.11.09-1", execute=True)

    assert rec["state"] == "blocked"
    assert rec["reason"] == "role_switch_failed"
    assert rec["executed"][-1]["state"] == state
    assert [call["role"] for call in executor.calls] == ["vision"]


@pytest.mark.parametrize("cmd", ("activate", "rollback"))
def test_cli_execute_stays_blocked_without_safe_executor(tmp_path, monkeypatch, capsys, cmd):
    from chuangong.__main__ import main

    cfg = _cfg(tmp_path)
    cfg.state_dir.mkdir()
    cfg.state_file.write_text(json.dumps({"current": "2026.11.10-1", "previous": "2026.11.09-1"}), encoding="utf-8")
    original = cfg.state_file.read_bytes()
    called = []

    def execute(*args, **kwargs):
        called.append((args, kwargs))
        return {"state": "role_switched"}

    monkeypatch.setattr("chuangong.__main__._config", lambda args: cfg)
    monkeypatch.setattr("chuangong.__main__._deps", lambda cfg: Deps(registry=None, docker=_Docker(), zenoh=None))
    monkeypatch.setattr("chuangong.role_switch.execute_role", execute)
    monkeypatch.setattr("chuangong.activation.fetch_release", lambda *args, **kwargs: _release("2026.11.09-1"))
    monkeypatch.setattr(
        "chuangong.activation.ActivationMachine._preflight",
        lambda self, record, release: (object(), {}),
    )

    args = [cmd, "--robot-id", cfg.robot_id, "--node", cfg.node, "--execute"]
    if cmd == "activate":
        args.append("2026.11.09-1")
    assert main(args) == 1
    rec = json.loads(capsys.readouterr().out)
    assert rec["state"] == "blocked"
    assert rec["reason"] == "executor_not_wired"
    assert called == []
    assert cfg.state_file.read_bytes() == original
