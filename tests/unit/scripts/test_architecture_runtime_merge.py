"""Tests for tools/architecture_runtime_merge.py.

The merge used to be inline Python in ``L: Architecture Audit``; the tool must
keep the runtime.json shape its consumers read (``nodes`` as strings,
``captures`` with ``pi``/``host``). On ROB-BOX both Pis see one shared Zenoh
graph (run 36422274238: 52 == 52 == 52 nodes), so identical inputs must merge
to the same graph, and a single capture must be enough.
"""

from __future__ import annotations

import argparse
import copy
import importlib.util
import json
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[3]
TOOL = REPO / "tools" / "architecture_runtime_merge.py"

spec = importlib.util.spec_from_file_location("architecture_runtime_merge", TOOL)
merge_tool = importlib.util.module_from_spec(spec)
sys.modules["architecture_runtime_merge"] = merge_tool
spec.loader.exec_module(merge_tool)


def ep(node, namespace):
    return {"node": node, "namespace": namespace}


MAIN = {
    "captured_at_unix": 100.0,
    "ros_domain_id": None,
    "rmw_implementation": "rmw_zenoh_cpp",
    "nodes": ["/b", "/a"],
    "topics": ["/scan", "/x"],
    "services": ["/s1"],
    "actions": [],
    "topic_info": {
        "/scan": {"present": True, "publishers": [ep("lidar", "/")], "subscribers": [], "raw": "MAIN"},
        "/x": {"present": False},
    },
}
VISION = {
    "captured_at_unix": 200.0,
    "ros_domain_id": "0",
    "rmw_implementation": "rmw_fastrtps_cpp",
    "nodes": ["/a", "/c"],
    "topics": ["/scan", "/x", "/y"],
    "services": ["/s1", "/s2"],
    "actions": ["/nav"],
    "topic_info": {
        "/scan": {"present": True, "publishers": [ep("lidar", "/"), ep("cam", "/oak")], "subscribers": [], "raw": "V"},
        "/x": {"present": True, "publishers": [], "subscribers": [ep("q", "/")], "raw": "X"},
    },
}


def capture(pi, host, container, name):
    return {"pi": pi, "host": host, "container": container, "path": Path("architecture") / name}


CAPTURES = [
    capture("Main Pi", "10.1.1.20", "nav2", "runtime-main.json"),
    capture("Vision Pi", "10.1.1.21", "oak-d", "runtime-vision.json"),
]


def test_two_captures_union_and_consumer_fields():
    merged = merge_tool.merge(CAPTURES, [MAIN, VISION])

    assert merged["schema_version"] == 4
    assert merged["captured_from"] == "remote:MainPi+VisionPi"
    assert merged["captured_at_unix"] == 200.0
    assert merged["ros_domain_id"] == "0"
    assert merged["rmw_implementation"] == "rmw_zenoh_cpp"
    assert merged["nodes"] == ["/a", "/b", "/c"]
    assert merged["topics"] == ["/scan", "/x", "/y"]
    assert merged["services"] == ["/s1", "/s2"]
    assert merged["actions"] == ["/nav"]
    assert merged["captures"] == [
        {"host": "10.1.1.20", "pi": "Main Pi", "container": "nav2", "snapshot": "runtime-main.json"},
        {"host": "10.1.1.21", "pi": "Vision Pi", "container": "oak-d", "snapshot": "runtime-vision.json"},
    ]
    scan = merged["topic_info"]["/scan"]
    assert scan["raw"] == "MAIN"
    assert scan["publishers"] == [ep("lidar", "/"), ep("cam", "/oak")]
    assert merged["topic_info"]["/x"] == {"present": True, "subscribers": [ep("q", "/")]}


def test_identical_inputs_merge_to_the_same_graph():
    merged = merge_tool.merge(CAPTURES, [MAIN, copy.deepcopy(MAIN)])
    single = merge_tool.merge(CAPTURES[:1], [MAIN])

    for key in ("nodes", "topics", "services", "actions", "topic_info"):
        assert merged[key] == single[key]
    assert merged["topic_info"]["/scan"]["publishers"] == [ep("lidar", "/")]


def test_single_capture():
    merged = merge_tool.merge(CAPTURES[:1], [MAIN])

    assert merged["captured_from"] == "remote:MainPi"
    assert merged["nodes"] == ["/a", "/b"]
    assert [c["pi"] for c in merged["captures"]] == ["Main Pi"]


def test_inputs_are_not_mutated():
    before = copy.deepcopy(MAIN)
    merge_tool.merge(CAPTURES, [MAIN, VISION])
    assert MAIN == before


def test_parse_capture():
    assert merge_tool.parse_capture("Main Pi|10.1.1.20|nav2|a/b.json") == {
        "pi": "Main Pi",
        "host": "10.1.1.20",
        "container": "nav2",
        "path": Path("a/b.json"),
    }
    with pytest.raises(argparse.ArgumentTypeError):
        merge_tool.parse_capture("10.1.1.20|nav2|a.json")
    with pytest.raises(argparse.ArgumentTypeError):
        merge_tool.parse_capture("Main Pi||nav2|a.json")


def test_cli_writes_runtime_json(tmp_path, capsys):
    main_path = tmp_path / "runtime-main.json"
    main_path.write_text(json.dumps(MAIN), encoding="utf-8")
    output = tmp_path / "out" / "runtime.json"

    merge_tool.main(["--output", str(output), "--capture", f"Main Pi|10.1.1.20|nav2|{main_path}"])

    data = json.loads(output.read_text(encoding="utf-8"))
    assert data["captures"][0]["snapshot"] == "runtime-main.json"
    assert json.loads(capsys.readouterr().out) == {"Main Pi nodes": 2, "union_nodes": 2, "union_topics": 2}
