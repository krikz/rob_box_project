"""Tests for tools/architecture_runtime_snapshot.py.

Run 36422274238 stored every endpoint with the namespace of the *previous*
endpoint: ``ros2 topic info --verbose`` prints ``Node name:`` before
``Node namespace:``, and the old parser closed the endpoint on ``Node name:``.
288 of 581 endpoints in runtime-main.json had a wrong namespace.

The fixture is the real ``/scan`` output from that run.
"""

from __future__ import annotations

import importlib.util
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
