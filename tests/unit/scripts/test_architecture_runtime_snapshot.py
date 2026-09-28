"""Tests for tools/architecture_runtime_snapshot.py.

Run 36422274238 stored every endpoint with the namespace of the *previous*
endpoint: ``ros2 topic info --verbose`` prints ``Node name:`` before
``Node namespace:``, and the old parser closed the endpoint on ``Node name:``.
288 of 581 endpoints in runtime-main.json had a wrong namespace.

The fixture is the real ``/scan`` output from that run.
"""

from __future__ import annotations

import importlib.util
import json
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[3]
TOOL = REPO / "tools" / "architecture_runtime_snapshot.py"
FIXTURE = Path(__file__).parent / "fixtures" / "ros2_topic_info_verbose_scan.txt"

spec = importlib.util.spec_from_file_location("architecture_runtime_snapshot", TOOL)
snapshot = importlib.util.module_from_spec(spec)
sys.modules["architecture_runtime_snapshot"] = snapshot
spec.loader.exec_module(snapshot)


def test_namespace_belongs_to_its_own_endpoint():
    publishers, subscribers = snapshot.parse_topic_info(FIXTURE.read_text(encoding="utf-8"))

    assert [(e["namespace"], e["node"]) for e in publishers] == [("/", "lslidar_driver_node")]
    assert [(e["namespace"], e["node"]) for e in subscribers] == [
        ("/global_costmap", "global_costmap"),
        ("/local_costmap", "local_costmap"),
        ("/", "quest_node"),
        ("/rtabmap", "rtabmap"),
        ("/rtabmap", "icp_odometry"),
    ]


def test_type_and_qos_are_kept_per_endpoint():
    publishers, subscribers = snapshot.parse_topic_info(FIXTURE.read_text(encoding="utf-8"))

    assert publishers[0] == {
        "node": "lslidar_driver_node",
        "namespace": "/",
        "type": "sensor_msgs/msg/LaserScan",
        "endpoint_type": "PUBLISHER",
        "reliability": "RELIABLE",
        "durability": "VOLATILE",
    }
    assert [e["reliability"] for e in subscribers] == [
        "BEST_EFFORT",
        "BEST_EFFORT",
        "RELIABLE",
        "RELIABLE",
        "RELIABLE",
    ]
    assert {e["endpoint_type"] for e in subscribers} == {"SUBSCRIPTION"}


def test_empty_and_zero_count_output():
    assert snapshot.parse_topic_info("") == ([], [])
    raw = "Type: std_msgs/msg/String\n\nPublisher count: 0\n\nSubscription count: 0\n"
    assert snapshot.parse_topic_info(raw) == ([], [])


# --------------------------------------------------------------------------- batch mode
# One ssh + one docker exec for all topics (222 topics took ~5 min one ssh at a time).

ENV_ONLY = "for t in; do"


def fake_batch_output(blocks, env=None):
    out = "".join(f"=====ENV {k}={v}=====\n" for k, v in (env or {}).items())
    for topic, (raw, rc) in blocks.items():
        out += f"=====TOPIC {topic}=====\n{raw}=====RC {rc}=====\n"
    return out


class FakeRunner:
    """Replays canned results and records every call instead of running ssh."""

    def __init__(self, batch_stdout, batch_rc=0, single=None, env_only_stdout=""):
        self.batch_stdout = batch_stdout
        self.batch_rc = batch_rc
        self.single = single or {}
        self.env_only_stdout = env_only_stdout
        self.calls = []

    def __call__(self, args, timeout=30, **kwargs):
        self.calls.append(args)
        if args[:2] == ["bash", "-c"]:
            if ENV_ONLY in args[2]:
                return {"returncode": 0, "stdout": self.env_only_stdout, "stderr": ""}
            return {"returncode": self.batch_rc, "stdout": self.batch_stdout, "stderr": "boom" if self.batch_rc else ""}
        if args[:3] == ["ros2", "topic", "info"]:
            return {"returncode": 0, "stdout": self.single[args[3]], "stderr": ""}
        lists = {"node": "/a\n", "topic": "/scan\n/x\n", "service": "", "action": ""}
        return {"returncode": 0, "stdout": lists[args[1]], "stderr": ""}


def test_split_batch_keeps_raw_bytes_and_env():
    scan = FIXTURE.read_text(encoding="utf-8")
    empty = "Type: std_msgs/msg/String\n\nPublisher count: 0\n\nSubscription count: 0\n\n"
    stdout = fake_batch_output(
        {"/scan": (scan, 0), "/x": (empty, 0)},
        {"ROS_DOMAIN_ID": "0", "RMW_IMPLEMENTATION": "rmw_zenoh_cpp"},
    )

    env, blocks = snapshot.split_batch(stdout)

    assert env == {"ROS_DOMAIN_ID": "0", "RMW_IMPLEMENTATION": "rmw_zenoh_cpp"}
    assert blocks["/scan"] == {"returncode": 0, "stdout": scan}
    assert blocks["/x"] == {"returncode": 0, "stdout": empty}
    assert snapshot.parse_topic_info(blocks["/scan"]["stdout"]) == snapshot.parse_topic_info(scan)


def test_split_batch_failed_empty_and_truncated_blocks():
    stdout = (
        "=====TOPIC /bad=====\n=====RC 1=====\n"
        "=====TOPIC /noeol=====\nType: x=====RC 0=====\n"
        "=====TOPIC /cut=====\nType: partial\n"
        "=====TOPIC /last=====\nType: y\n=====RC 0=====\n"
    )

    env, blocks = snapshot.split_batch(stdout)

    assert env == {}
    assert blocks["/bad"] == {"returncode": 1, "stdout": ""}
    assert blocks["/noeol"] == {"returncode": 0, "stdout": "Type: x"}
    assert "/cut" not in blocks
    assert blocks["/last"] == {"returncode": 0, "stdout": "Type: y\n"}


def test_batch_script_quotes_topics_and_reads_env_in_container():
    script = snapshot.batch_script(["/scan", "/we ird'topic"])

    assert "for t in /scan '/we ird'\"'\"'topic'; do" in script
    assert 'ros2 topic info "$t" --verbose' in script
    assert '"$ROS_DOMAIN_ID"' in script and '"$RMW_IMPLEMENTATION"' in script
    assert ENV_ONLY in snapshot.batch_script([])


def run_main(monkeypatch, tmp_path, runner, *extra):
    monkeypatch.setattr(snapshot, "remote_run", runner)
    monkeypatch.setattr(snapshot.time, "sleep", lambda _: None)
    monkeypatch.setenv("ROS_DOMAIN_ID", "99")  # the runner's env must not leak into the snapshot
    out = tmp_path / "runtime.json"
    argv = ["snapshot", "--output", str(out), "--remote-host", "pi", "--remote-container", "nav2", *extra]
    monkeypatch.setattr(sys, "argv", argv)
    snapshot.main()
    return json.loads(out.read_text(encoding="utf-8"))


def test_main_batch_mode_retries_only_failed_topics(monkeypatch, tmp_path):
    scan = FIXTURE.read_text(encoding="utf-8")
    runner = FakeRunner(
        fake_batch_output({"/scan": (scan, 0), "/x": ("", 1)}, {"ROS_DOMAIN_ID": "0"}),
        single={"/x": "Type: std_msgs/msg/String\n\nPublisher count: 0\n\nSubscription count: 0\n"},
    )

    data = run_main(monkeypatch, tmp_path, runner)

    assert data["ros_domain_id"] == "0"
    assert data["rmw_implementation"] is None
    assert data["topic_info"]["/scan"]["raw"] == scan
    assert data["topic_info"]["/scan"]["present"] is True
    assert data["topic_info"]["/x"]["present"] is True
    assert [c for c in runner.calls if c[:3] == ["ros2", "topic", "info"]] == [
        ["ros2", "topic", "info", "/x", "--verbose"]
    ]
    assert sum(c[:2] == ["bash", "-c"] for c in runner.calls) == 1


def test_main_batch_failure_falls_back_to_per_topic(monkeypatch, tmp_path):
    scan = FIXTURE.read_text(encoding="utf-8")
    runner = FakeRunner("", batch_rc=255, single={"/scan": scan, "/x": "Type: t\n"}, env_only_stdout="")

    data = run_main(monkeypatch, tmp_path, runner)

    assert data["ros_domain_id"] is None
    assert data["topic_info"]["/scan"]["raw"] == scan
    assert sorted(c[3] for c in runner.calls if c[:3] == ["ros2", "topic", "info"]) == ["/scan", "/x"]


def test_main_per_topic_flag_still_reads_env_from_container(monkeypatch, tmp_path):
    runner = FakeRunner(
        "", single={"/scan": "Type: a\n", "/x": "Type: b\n"}, env_only_stdout="=====ENV ROS_DOMAIN_ID=0=====\n"
    )

    data = run_main(monkeypatch, tmp_path, runner, "--per-topic")

    assert data["ros_domain_id"] == "0"
    assert data["topic_info"]["/x"]["raw"] == "Type: b\n"
    batch_calls = [c for c in runner.calls if c[:2] == ["bash", "-c"]]
    assert len(batch_calls) == 1 and ENV_ONLY in batch_calls[0][2]
