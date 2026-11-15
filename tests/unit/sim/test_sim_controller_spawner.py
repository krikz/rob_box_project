from __future__ import annotations

import sys
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


def _fake_ros(monkeypatch, deliver_clock):
    events = []

    class Future:
        def __init__(self):
            self._done = False

        def done(self):
            return self._done

        def set_result(self, result):
            self._done = True

    class Logger:
        def error(self, message):
            events.append(("error", message))

    class Node:
        def __init__(self):
            self.callback = None

        def create_subscription(self, msg_type, topic, callback, qos):
            events.append(("subscribe", topic))
            self.callback = callback

        def get_logger(self):
            return Logger()

        def destroy_node(self):
            events.append(("destroy",))

    rclpy = types.ModuleType("rclpy")
    node = Node()
    rclpy.init = lambda: events.append(("init",))
    rclpy.create_node = lambda name: node
    rclpy.try_shutdown = lambda: events.append(("shutdown",))

    def spin_until_future_complete(actual_node, future, timeout_sec):
        events.append(("spin", timeout_sec))
        if deliver_clock:
            actual_node.callback(object())

    rclpy.spin_until_future_complete = spin_until_future_complete
    task = types.ModuleType("rclpy.task")
    task.Future = Future
    qos = types.ModuleType("rclpy.qos")
    qos.HistoryPolicy = types.SimpleNamespace(KEEP_LAST=1)
    qos.ReliabilityPolicy = types.SimpleNamespace(BEST_EFFORT=1)
    qos.QoSProfile = lambda **kwargs: kwargs
    rosgraph_msgs = types.ModuleType("rosgraph_msgs")
    rosgraph_msg = types.ModuleType("rosgraph_msgs.msg")
    rosgraph_msg.Clock = object
    monkeypatch.setitem(sys.modules, "rclpy", rclpy)
    monkeypatch.setitem(sys.modules, "rclpy.task", task)
    monkeypatch.setitem(sys.modules, "rclpy.qos", qos)
    monkeypatch.setitem(sys.modules, "rosgraph_msgs", rosgraph_msgs)
    monkeypatch.setitem(sys.modules, "rosgraph_msgs.msg", rosgraph_msg)
    return events


def test_launch_starts_jsb_only_after_gazebo_clock():
    text = LAUNCH.read_text(encoding="utf-8")
    assert "jsb = ExecuteProcess(" in text
    assert '"rob_box_sim.controller_spawner"' in text
    assert '"--clock-topic"' in text
    assert "CLOCK_RAW_TOPIC" in text


def test_spawner_execs_only_after_clock(monkeypatch):
    events = _fake_ros(monkeypatch, deliver_clock=True)
    execs = []
    monkeypatch.setattr(
        cs.os, "execvp", lambda executable, argv: execs.append((executable, argv))
    )

    cs.main(
        [
            "joint_state_broadcaster",
            "--clock-topic",
            "/sim/clock_raw",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "60",
        ]
    )

    assert ("subscribe", "/sim/clock_raw") in events
    assert events.index(("spin", 60.0)) < events.index(("shutdown",))
    assert execs == [
        (
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
    ]


def test_spawner_does_not_exec_without_clock(monkeypatch):
    events = _fake_ros(monkeypatch, deliver_clock=False)
    execs = []
    monkeypatch.setattr(
        cs.os, "execvp", lambda executable, argv: execs.append((executable, argv))
    )

    rc = cs.main(["joint_state_broadcaster", "--clock-timeout", "0.1"])

    assert rc == 1
    assert execs == []
    assert any(event[0] == "error" and "0.1" in event[1] for event in events)
