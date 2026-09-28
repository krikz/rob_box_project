"""Tests for tools/architecture_class_metrics.py (complexity, cohesion, god classes)."""

from __future__ import annotations

import importlib.util
import json
import sys
import textwrap
from pathlib import Path

REPO = Path(__file__).resolve().parents[3]
TOOL = REPO / "tools" / "architecture_class_metrics.py"

spec = importlib.util.spec_from_file_location("architecture_class_metrics", TOOL)
metrics = importlib.util.module_from_spec(spec)
sys.modules["architecture_class_metrics"] = metrics
spec.loader.exec_module(metrics)

TWO_JOBS = textwrap.dedent("""
    class TwoJobs(Node):
        def __init__(self):
            self.pub = self.create_publisher(String, "/a", 10)
            self.create_subscription(String, "/b", self.on_b, 10)
            self.db = {}

        def on_b(self, msg):
            if msg.data and msg.data != "x":
                self.pub.publish(msg)

        def send(self, text):
            for part in text.split():
                self.pub.publish(part)

        def save(self, key, value):
            self.db[key] = value

        def load(self, key):
            return self.db.get(key)
    """)


def write_repo(root: Path) -> None:
    pkg = root / "src" / "pkg" / "pkg"
    pkg.mkdir(parents=True)
    (pkg / "two_jobs.py").write_text(TWO_JOBS, encoding="utf-8")
    tests = root / "src" / "pkg" / "test"
    tests.mkdir()
    (tests / "test_two_jobs.py").write_text("from pkg.two_jobs import TwoJobs\n", encoding="utf-8")
    # Classes defined in tests are evidence only, never measured.
    (tests / "test_fake.py").write_text("class FakeNode:\n    pass\n", encoding="utf-8")


def only_class(data):
    (cls,) = data["classes"]
    return cls


def test_cyclomatic_complexity_counts_decisions():
    fn = metrics.ast.parse("def f(a, b):\n    if a and b:\n        return [x for x in a if x]\n    return 0\n").body[0]
    # 1 base + if + and + comprehension + comprehension-if
    assert metrics.cyclomatic_complexity(fn) == 5


def test_class_metrics_measure_size_complexity_and_responsibilities(tmp_path):
    write_repo(tmp_path)
    cls = only_class(metrics.collect(tmp_path))

    assert cls["name"] == "TwoJobs"
    assert cls["file"] == "src/pkg/pkg/two_jobs.py"
    assert cls["methods"] == 5
    assert cls["wmc"] == 1 + 3 + 2 + 1 + 1
    assert (cls["max_cc"], cls["max_cc_method"]) == (3, "on_b")
    assert cls["attributes"] == 2
    assert cls["ros_endpoints"] == 2
    # pub-side methods and db-side methods never share state: two responsibilities.
    assert cls["responsibilities"] == 2
    assert cls["responsibility_groups"] == [["load", "save"], ["on_b", "send"]]
    assert cls["tcc"] == round(2 / 6, 3)
    assert cls["god_class"] is False


def test_tests_are_references_not_measured_classes(tmp_path):
    write_repo(tmp_path)
    data = metrics.collect(tmp_path)

    assert [c["name"] for c in data["classes"]] == ["TwoJobs"]
    assert data["summary"]["test_files"] == 2
    cls = only_class(data)
    assert cls["test_refs"] == 1
    assert cls["test_ref_files"] == ["src/pkg/test/test_two_jobs.py"]
    assert cls["line_coverage"] is None


def test_coverage_json_maps_lines_to_class(tmp_path):
    write_repo(tmp_path)
    report = {
        "files": {
            "src/pkg/pkg/two_jobs.py": {"executed_lines": [2, 3, 4, 5, 6], "missing_lines": [8, 9, 10, 12, 13]},
        }
    }
    cov = tmp_path / "coverage.json"
    cov.write_text(json.dumps(report), encoding="utf-8")

    cls = only_class(metrics.collect(tmp_path, cov))
    assert cls["line_coverage"] == 0.5


def test_methods_without_shared_state_are_separate_responsibilities():
    methods = "\n".join(
        f"    def m{i}(self, x):\n        self.f{i} = x\n" + "        if x:\n            pass\n" * 12 for i in range(4)
    )
    src = "class Big:\n" + methods + "\n" * 400
    tree = metrics.ast.parse(src)
    cls = tree.body[0]
    own = [n for n in cls.body if isinstance(n, metrics.ast.FunctionDef)]
    wmc = sum(metrics.cyclomatic_complexity(m) for m in own)
    tcc, responsibilities, _, stateless = metrics.cohesion({m.name: metrics.self_usage(m) for m in own})
    assert wmc == 4 * 13
    assert tcc == 0.0
    assert responsibilities == 4
    assert stateless == 0


def test_protocol_stubs_are_stateless_not_responsibilities():
    src = "class Port(Protocol):\n    def a(self): ...\n    def b(self): ...\n    def c(self):\n        return 1\n"
    cls = metrics.ast.parse(src).body[0]
    usage = {m.name: metrics.self_usage(m) for m in cls.body}
    assert metrics.cohesion(usage)[1:] == (0, [], 3)


def test_markdown_lists_god_classes_and_untested(tmp_path):
    write_repo(tmp_path)
    data = metrics.collect(tmp_path)
    data["classes"][0].update(god_class=True, wmc=60, test_refs=0)
    text = metrics.render_markdown(data)

    assert "## God classes" in text
    assert "| TwoJobs | src/pkg/pkg/two_jobs.py:2 |" in text
    assert "without any test reference" in text
    assert "proxy, not coverage" in text
