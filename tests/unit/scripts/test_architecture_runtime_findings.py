"""Tests for tools/architecture_runtime_findings.py (live ROS graph review candidates).

Every finding type here was seen on the robot in run 36422274238: /odom read as
std_msgs/String by avatar_arbiter, /device/snapshot with no publisher,
/cmd_vel_smoothed with no subscriber, four writers on /voice/tts/control,
five /panel_image publishers inside voice_animation_player.
"""

from __future__ import annotations

import importlib.util
import json
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[3]
TOOL = REPO / "tools" / "architecture_runtime_findings.py"

spec = importlib.util.spec_from_file_location("architecture_runtime_findings", TOOL)
findings = importlib.util.module_from_spec(spec)
sys.modules["architecture_runtime_findings"] = findings
spec.loader.exec_module(findings)


def endpoint(node, kind, msg_type="std_msgs/msg/String", namespace="/", reliability="RELIABLE", durability="VOLATILE"):
    return (
        f"Node name: {node}\nNode namespace: {namespace}\nTopic type: {msg_type}\nEndpoint type: {kind}\n"
        f"GID: 00\nQoS profile:\n  Reliability: {reliability}\n  History (Depth): KEEP_LAST (10)\n"
        f"  Durability: {durability}\n  Lifespan: Infinite\n\n"
    )


def topic(pubs=(), subs=(), msg_type="std_msgs/msg/String"):
    raw = f"Type: {msg_type}\n\nPublisher count: {len(pubs)}\n\n"
    raw += "".join(endpoint(*p) if isinstance(p, tuple) else endpoint(p, "PUBLISHER") for p in pubs)
    raw += f"Subscription count: {len(subs)}\n\n"
    raw += "".join(endpoint(*s) if isinstance(s, tuple) else endpoint(s, "SUBSCRIPTION") for s in subs)
    return {"present": True, "raw": raw}


RUNTIME = {
    "captured_from": "test",
    "topic_info": {
        "/odom": topic(
            pubs=[("icp_odometry", "PUBLISHER", "nav_msgs/msg/Odometry", "/rtabmap")],
            subs=[
                ("avatar_arbiter", "SUBSCRIPTION", "std_msgs/msg/String"),
                ("bt_navigator", "SUBSCRIPTION", "nav_msgs/msg/Odometry"),
            ],
        ),
        "/device/snapshot": topic(subs=["avatar_arbiter", "quest_node"]),
        "/cmd_vel_smoothed": topic(pubs=["velocity_smoother"]),
        "/voice/tts/control": topic(pubs=["audio_node", "dialogue_node", "quest_node", "stt_node"], subs=["tts_node"]),
        "/panel_image": topic(pubs=["voice_animation_player"] * 5, subs=["led_matrix_compositor"]),
        "/avatar/tts/audio": topic(
            pubs=[("tts_node", "PUBLISHER", "std_msgs/msg/String", "/", "BEST_EFFORT")],
            subs=[("quest_node", "SUBSCRIPTION", "std_msgs/msg/String", "/", "RELIABLE")],
        ),
        # Noise that must never produce findings.
        "/tf": topic(pubs=["robot_state_publisher"]),
        "/behavior_server/transition_event": topic(pubs=["behavior_server"]),
        "/fine": topic(pubs=["a", "_ros2cli_daemon_0_92e553f0f73144f5995d659a226896f6"], subs=["b"]),
    },
}


def by_type(report):
    out = {}
    for item in report["findings"]:
        out.setdefault(item["type"], []).append((item["topic"], item["detail"]))
    return out


def test_every_finding_type_on_robot_shaped_graph():
    report = findings.build_report(RUNTIME, own_nodes={"avatar_arbiter", "quest_node", "tts_node"})
    found = by_type(report)

    assert found["type_mismatch"] == [("/odom", "nav_msgs/msg/Odometry vs std_msgs/msg/String")]
    assert found["dead_input"] == [("/device/snapshot", "")]
    assert found["dead_output"] == [("/cmd_vel_smoothed", "")]
    assert found["multiple_writers"] == [("/voice/tts/control", "")]
    assert found["duplicate_publishers"] == [("/panel_image", "/voice_animation_player")]
    assert found["qos_incompatible"] == [("/avatar/tts/audio", "/tts_node -> /quest_node")]
    assert report["summary"]["findings"] == 6


def test_infra_topics_and_helper_nodes_are_ignored():
    topics = {item["topic"] for item in findings.analyze(RUNTIME)}
    assert "/tf" not in topics
    assert "/behavior_server/transition_event" not in topics
    assert "/fine" not in topics  # the ros2cli daemon is not a second writer


def test_scope_marks_our_nodes_and_declared_topics():
    inventory = {"topics": [{"name": "/cmd_vel_smoothed"}]}
    report = findings.build_report(RUNTIME, inventory, own_nodes={"avatar_arbiter"})
    scope = {(f["type"], f["topic"]): f["scope"] for f in report["findings"]}

    assert scope[("dead_input", "/device/snapshot")] == "repo"  # our subscriber
    assert scope[("dead_output", "/cmd_vel_smoothed")] == "repo"  # declared in code
    assert scope[("multiple_writers", "/voice/tts/control")] == "external"


def test_namespaces_come_from_raw_not_from_shifted_legacy_fields():
    legacy = {"topic_info": {"/scan": dict(topic(pubs=["a"]), publishers=[{"node": "a", "namespace": "/wrong"}])}}
    (item,) = findings.analyze(legacy)
    assert item["evidence"] == {"publishers": ["/a"]}


def test_legacy_snapshot_without_raw_still_works():
    legacy = {"topic_info": {"/x": {"publishers": [{"node": "a", "namespace": "/"}], "subscribers": []}}}
    assert [f["type"] for f in findings.analyze(legacy)] == ["dead_output"]


def test_baseline_marks_new_and_resolved():
    known = {"dead_input|/device/snapshot|", "dead_output|/gone|"}
    report = findings.build_report(RUNTIME, baseline=known)

    new = {f["key"] for f in report["findings"] if f["new"]}
    assert "dead_input|/device/snapshot|" not in new
    assert "dead_output|/cmd_vel_smoothed|" in new
    assert report["resolved"] == ["dead_output|/gone|"]
    assert report["summary"]["resolved_since_baseline"] == 1


def test_cli_writes_reports_baseline_and_fails_on_new(tmp_path):
    runtime = tmp_path / "runtime.json"
    runtime.write_text(json.dumps(RUNTIME), encoding="utf-8")
    base = tmp_path / "baseline.json"
    out = ["--json", str(tmp_path / "f.json"), "--markdown", str(tmp_path / "f.md"), "--repo-root", str(REPO)]

    first = subprocess.run([sys.executable, str(TOOL), str(runtime), "--write-baseline", str(base), *out])
    assert first.returncode == 0
    assert len(json.loads(base.read_text())["known"]) == 6

    same = subprocess.run([sys.executable, str(TOOL), str(runtime), "--baseline", str(base), "--fail-on-new", *out])
    assert same.returncode == 0

    runtime_new = json.loads(runtime.read_text())
    runtime_new["topic_info"]["/new/dead"] = topic(subs=["x"])
    runtime.write_text(json.dumps(runtime_new), encoding="utf-8")
    worse = subprocess.run([sys.executable, str(TOOL), str(runtime), "--baseline", str(base), "--fail-on-new", *out])
    assert worse.returncode == 1
    markdown = (tmp_path / "f.md").read_text()
    assert "🆕" in markdown and "`/new/dead`" in markdown
