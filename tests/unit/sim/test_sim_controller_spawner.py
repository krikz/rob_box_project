from __future__ import annotations

import subprocess
import types
from pathlib import Path

import pytest

from rob_box_sim import controller_spawner as cs

LAUNCH = (
    Path(__file__).resolve().parents[3]
    / "src"
    / "rob_box_sim"
    / "launch"
    / "sim.launch.py"
)

VALID_STATE = """stats {
  iterations: 42
}
state {
}
"""


def _result(stdout="", stderr="", returncode=0):
    return types.SimpleNamespace(stdout=stdout, stderr=stderr, returncode=returncode)


def _fake_ign(
    monkeypatch, services="/world/depot/state\n", list_results=None, state_results=None
):
    events = []
    if list_results is None:
        list_results = [_result(stdout=services)]
    if state_results is None:
        state_results = [_result(stdout=VALID_STATE)]

    def take(results):
        result = results.pop(0) if len(results) > 1 else results[0]
        if isinstance(result, BaseException):
            raise result
        return result

    def run(cmd, **kwargs):
        if cmd[:3] == ["ign", "service", "-l"]:
            events.append(("list",))
            return take(list_results)
        assert cmd[:4] == ["ign", "service", "-s", "/world/depot/state"]
        events.append(("request", cmd[3]))
        return take(state_results)

    monkeypatch.setattr(subprocess, "run", run)
    return events


def _capture_exec(monkeypatch, events):
    monkeypatch.setattr(
        cs.os,
        "execvp",
        lambda executable, argv: events.append(("exec", executable, argv)),
    )


def _run_main():
    return cs.main(
        [
            "joint_state_broadcaster",
            "--startup-timeout",
            "1",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "60",
        ]
    )


def test_launch_starts_jsb_only_after_gazebo_state():
    text = LAUNCH.read_text(encoding="utf-8")
    assert "jsb = ExecuteProcess(" in text
    assert '"rob_box_sim.controller_spawner"' in text
    assert '"--startup-timeout"' in text
    assert '"--clock-topic"' not in text


def test_spawner_execs_only_after_gazebo_state(monkeypatch):
    events = _fake_ign(monkeypatch)
    _capture_exec(monkeypatch, events)

    assert _run_main() == 0

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


def test_spawner_selects_exact_world_state_service(monkeypatch):
    events = _fake_ign(
        monkeypatch,
        services=(
            "/world/depot/control/state\n"
            "/world/depot/scene/state\n"
            "/world//state\n"
            "/world/depot/state\n"
            "/world/depot/state/nested\n"
        ),
    )
    _capture_exec(monkeypatch, events)

    assert _run_main() == 0

    assert [event for event in events if event[0] == "request"] == [
        ("request", "/world/depot/state")
    ]


@pytest.mark.parametrize(
    "state_result",
    [
        _result(stdout="Service call failed\n"),
        _result(stderr="Service call timed out\n"),
    ],
)
def test_spawner_does_not_exec_on_failed_state_contract(monkeypatch, state_result):
    events = _fake_ign(monkeypatch, state_results=[state_result])
    monkeypatch.setattr(cs.os, "execvp", lambda *args: events.append(("exec",)))

    rc = cs.main(["joint_state_broadcaster", "--startup-timeout", "0.01"])

    assert rc == 1
    assert ("request", "/world/depot/state") in events
    assert not any(event[0] == "exec" for event in events)


def test_spawner_rejects_true_nested_in_state_without_stats(monkeypatch):
    result = _result(stdout="state {\n  has_one_time_component_changes: true\n}\n")
    events = _fake_ign(monkeypatch, state_results=[result])
    monkeypatch.setattr(cs.os, "execvp", lambda *args: events.append(("exec",)))

    rc = cs.main(["joint_state_broadcaster", "--startup-timeout", "0.01"])

    assert rc == 1
    assert not any(event[0] == "exec" for event in events)


def test_spawner_does_not_exec_without_gazebo_state_service(monkeypatch):
    events = _fake_ign(monkeypatch, services="/world/depot/scene/info\n")
    monkeypatch.setattr(cs.os, "execvp", lambda *args: events.append(("exec",)))

    rc = cs.main(["joint_state_broadcaster", "--startup-timeout", "0"])

    assert rc == 1
    assert not any(event[0] in ("request", "exec") for event in events)


@pytest.mark.parametrize("command", ["list", "request"])
@pytest.mark.parametrize(
    "error",
    [
        subprocess.TimeoutExpired("ign", 1),
        FileNotFoundError("ign"),
        OSError("transport unavailable"),
    ],
)
def test_spawner_retries_subprocess_errors_within_budget(monkeypatch, command, error):
    kwargs = {}
    if command == "list":
        kwargs["list_results"] = [error, _result(stdout="/world/depot/state\n")]
    else:
        kwargs["state_results"] = [error, _result(stdout=VALID_STATE)]
    events = _fake_ign(monkeypatch, **kwargs)
    _capture_exec(monkeypatch, events)
    monkeypatch.setattr(cs.time, "sleep", lambda seconds: None)

    assert _run_main() == 0
    assert any(event[0] == "exec" for event in events)


def test_spawner_retries_nonzero_service_list_within_budget(monkeypatch):
    events = _fake_ign(
        monkeypatch,
        list_results=[
            _result(stderr="transport unavailable\n", returncode=1),
            _result(stdout="/world/depot/state\n"),
        ],
    )
    _capture_exec(monkeypatch, events)
    monkeypatch.setattr(cs.time, "sleep", lambda seconds: None)

    assert _run_main() == 0
    assert events.count(("list",)) == 2
    assert any(event[0] == "exec" for event in events)
