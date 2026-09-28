"""Unit tests for ``tools/architecture_audit.py`` ROS-interface scanning.

Issue #3108: mcp_server tools publish through
``shared_publisher(node, Type, "topic", qos)`` instead of a direct
``node.create_publisher(Type, "topic", qos)``. The audit must still count
those as ``publish`` interfaces (topic = arg 2, type = arg 1), otherwise the
inventory silently loses publishers.
"""

from __future__ import annotations

import importlib.util
from pathlib import Path

REPO = Path(__file__).resolve().parents[3]
TOOL = REPO / "tools" / "architecture_audit.py"

spec = importlib.util.spec_from_file_location("architecture_audit", TOOL)
audit = importlib.util.module_from_spec(spec)
assert spec and spec.loader
spec.loader.exec_module(audit)


def _scan(tmp_path: Path, rel: str, content: str) -> list[dict]:
    src = tmp_path / "src"
    path = src / rel
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(content, encoding="utf-8")
    _files, _classes, interfaces = audit.scan_python(src)
    return interfaces


def test_create_publisher_still_counted(tmp_path):
    interfaces = _scan(
        tmp_path,
        "pkg/pkg/node.py",
        "class N(Node):\n"
        "    def __init__(self):\n"
        "        self.create_publisher(String, '/a', 10)\n"
        "        self.create_subscription(String, '/b', self.cb, 10)\n",
    )
    got = sorted((i["kind"], i["name"], i["type"], i["node_class"]) for i in interfaces)
    assert got == [("publish", "/a", "String", "N"), ("subscribe", "/b", "String", "N")]


def test_shared_publisher_bare_and_attribute_calls_count_as_publish(tmp_path):
    interfaces = _scan(
        tmp_path,
        "rob_box_mcp_tools/rob_box_mcp_tools/tools/t.py",
        "from ..base import shared_publisher\n"
        "from .. import base\n"
        "class Tool:\n"
        "    def __init__(self, node):\n"
        "        self.a = shared_publisher(node, String, '/voice/animation/request', 10)\n"
        "        self.b = base.shared_publisher(node, std_msgs.msg.String, '/voice/tts/set_provider', 10)\n",
    )
    got = sorted((i["kind"], i["name"], i["type"]) for i in interfaces)
    assert got == [
        ("publish", "/voice/animation/request", "String"),
        ("publish", "/voice/tts/set_provider", "std_msgs.msg.String"),
    ]


def test_shared_publisher_helper_body_and_short_calls_are_ignored(tmp_path):
    # The helper's own ``node.create_publisher(msg_type, topic, qos)`` has a
    # non-literal topic and must not produce an entry; a malformed call with
    # too few args must not crash the scan.
    interfaces = _scan(
        tmp_path,
        "rob_box_mcp_tools/rob_box_mcp_tools/base.py",
        "def shared_publisher(node, msg_type, topic, qos):\n"
        "    return node.create_publisher(msg_type, topic, qos)\n"
        "def f(node):\n"
        "    shared_publisher(node, String)\n",
    )
    assert interfaces == []
