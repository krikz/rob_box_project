"""Tests for tools/architecture_audit.py and tools/architecture_runtime_diff.py.

Before: 580 of 942 scanned files and 1939 of 2577 classes were tests, the
compose list named a missing docker/quest file and skipped docker/monitoring,
and runtime_diff wrote runtime-diff.json to a fixed path.
"""

from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[3]
AUDIT = REPO / "tools" / "architecture_audit.py"
DIFF = REPO / "tools" / "architecture_runtime_diff.py"

NODE_SRC = """
from rclpy.node import Node


class {name}(Node):
    def __init__(self):
        super().__init__("{name}")
        self.create_publisher(String, "/{topic}", 10)
"""


def write(path, text=""):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")


def make_repo(root):
    write(root / "src/pkg/package.xml", "<package/>")
    write(root / "src/pkg/pkg/real_node.py", NODE_SRC.format(name="RealNode", topic="real"))
    for index, rel in enumerate(
        (
            "src/pkg/test/test_real.py",
            "src/pkg/tests/helpers.py",
            "src/pkg/test_top.py",
            "src/pkg/conftest.py",
            "src/pkg/scripts/test_manual.py",
            "src/pkg/scripts/example_usage.py",
        )
    ):
        write(root / rel, NODE_SRC.format(name=f"Fake{index}", topic="fake"))
    write(root / "src/pkg/scripts/tool.py", "class Helper:\n    pass\n")
    write(root / "docker/main/docker-compose.yaml", "services:\n  nav2:\n    image: a\n")
    write(root / "docker/monitoring/docker-compose.yaml", "services:\n  grafana:\n    image: b\n")
    write(root / "docker/vision/docker-compose.yml", "services:\n  oak-d:\n    image: c\n")
    write(root / "docker/main/docker-compose.override.yaml", "services:\n  extra:\n    image: d\n")
    write(root / "docker/main/Dockerfile", "FROM x\n")


def run_audit(root):
    out = subprocess.run(
        [
            sys.executable,
            str(AUDIT),
            "--root",
            str(root),
            "--output",
            str(root / "out/inventory.json"),
            "--markdown",
            str(root / "out/inventory.md"),
        ],
        capture_output=True,
        text=True,
        check=True,
    )
    return json.loads((root / "out/inventory.json").read_text(encoding="utf-8")), json.loads(out.stdout)


def test_tests_and_examples_are_skipped_and_counted(tmp_path):
    make_repo(tmp_path)

    inventory, summary = run_audit(tmp_path)

    assert [c["name"] for c in inventory["classes"]] == ["RealNode", "Helper"]
    assert [i["name"] for i in inventory["ros_interfaces"]] == ["/real"]
    assert sorted(f["path"] for f in inventory["python"]) == ["src/pkg/pkg/real_node.py", "src/pkg/scripts/tool.py"]
    assert summary["skipped_test_files"] == 6
    assert summary["node_classes"] == 1
    assert inventory["schema_version"] == 2


def test_compose_files_are_globbed(tmp_path):
    make_repo(tmp_path)

    inventory, summary = run_audit(tmp_path)

    assert [c["file"] for c in inventory["containers"]] == [
        "docker/main/docker-compose.override.yaml",
        "docker/main/docker-compose.yaml",
        "docker/monitoring/docker-compose.yaml",
        "docker/vision/docker-compose.yml",
    ]
    assert summary["containers"] == 4


def test_runtime_diff_writes_json_where_asked(tmp_path):
    inventory = tmp_path / "inventory.json"
    runtime = tmp_path / "runtime.json"
    write(inventory, json.dumps({"topics": [{"name": "/a"}, {"name": "/b"}]}))
    write(runtime, json.dumps({"topics": ["/b", "/c"]}))

    subprocess.run(
        [
            sys.executable,
            str(DIFF),
            str(inventory),
            str(runtime),
            "--json",
            str(tmp_path / "x/diff.json"),
            "--markdown",
            str(tmp_path / "x/diff.md"),
        ],
        cwd=tmp_path,
        capture_output=True,
        text=True,
        check=True,
    )

    result = json.loads((tmp_path / "x/diff.json").read_text(encoding="utf-8"))
    assert result["static_not_runtime"] == ["/a"] and result["runtime_not_static"] == ["/c"]
    assert not (tmp_path / "architecture").exists()
