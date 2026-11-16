from __future__ import annotations

import subprocess
import types
from pathlib import Path

from rob_box_sim import controller_spawner as cs

LAUNCH = (
    Path(__file__).resolve().parents[3]
    / "src"
    / "rob_box_sim"
    / "launch"
    / "sim.launch.py"
)


def _fake_ign(monkeypatch, services, state_result="true"):
    events = []

    def run(cmd, **kwargs):
        if cmd[:3] == ["ign", "service", "-l"]:
            events.append(("list",))
            return types.SimpleNamespace(returncode=0, stdout=services)
        assert cmd[:4] == ["ign", "service", "-s", "/world/depot/state"]
        events.append(("request", cmd[3]))
        return types.SimpleNamespace(returncode=0, stdout=state_result)

    monkeypatch.setattr(subprocess, "run", run)
    return events


def test_launch_starts_jsb_only_after_gazebo_state():
    text = LAUNCH.read_text(encoding="utf-8")
    assert "jsb = ExecuteProcess(" in text
    assert '"rob_box_sim.controller_spawner"' in text
    assert '"--startup-timeout"' in text
    assert '"--clock-topic"' not in text


def test_spawner_execs_only_after_gazebo_state(monkeypatch):
    events = _fake_ign(monkeypatch, "/world/depot/state\n")
    monkeypatch.setattr(
        cs.os,
        "execvp",
        lambda executable, argv: events.append(("exec", executable, argv)),
    )

    cs.main(
        [
            "joint_state_broadcaster",
            "--startup-timeout",
            "60",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "60",
        ]
    )

    assert events[0] == ("list",)
    assert ("request", "/world/depot/state") in events
    assert events.index(("request", "/world/depot/state")) < next(
        i for i, event in enumerate(events) if event[0] == "exec"
    )
    assert events[-1] == (
        "exec",
        "ros2",
        [
            "ros2",
            "run",
            "controller_manager",
            "spawner",
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "60",
        ],
    )


def test_spawner_does_not_exec_without_gazebo_state_service(monkeypatch):
    events = _fake_ign(monkeypatch, "/world/depot/scene/info\n")
    monkeypatch.setattr(cs.os, "execvp", lambda *args: events.append(("exec",)))

    rc = cs.main(["joint_state_broadcaster", "--startup-timeout", "0"])

    assert rc == 1
    assert not any(event[0] in ("request", "exec") for event in events)


def test_spawner_does_not_exec_when_gazebo_state_request_fails(monkeypatch):
    events = _fake_ign(monkeypatch, "/world/depot/state\n", state_result="false")
    monkeypatch.setattr(cs.os, "execvp", lambda *args: events.append(("exec",)))

    rc = cs.main(["joint_state_broadcaster", "--startup-timeout", "0.01"])

    assert rc == 1
    assert ("request", "/world/depot/state") in events
    assert not any(event[0] == "exec" for event in events)
