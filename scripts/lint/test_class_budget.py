#!/usr/bin/env python3
"""Self-tests for scripts/lint/class_budget.py (ADR-0145 class-size budget).

Run:
    python -m unittest scripts.lint.test_class_budget -v
"""

from __future__ import annotations

import contextlib
import io
import json
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import class_budget  # noqa: E402


def _cls(name: str, methods: int, cc_each: int = 1, indent: str = "") -> str:
    """Class source with ``methods`` methods each of CC=``cc_each``."""
    lines = [f"{indent}class {name}:"]
    for i in range(methods):
        lines.append(f"{indent}    def m{i}(self, x):")
        lines.extend(f"{indent}        if x == {j}:\n{indent}            x += 1" for j in range(cc_each - 1))
        lines.append(f"{indent}        return x")
    return "\n".join(lines) + "\n"


class ClassBudgetTest(unittest.TestCase):
    def setUp(self) -> None:
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        self.root = Path(self._tmp.name).resolve()
        self.baseline = self.root / "baseline.json"

    def _write(self, name: str, source: str) -> Path:
        path = self.root / name
        path.write_text(source, encoding="utf-8")
        return path

    def _key(self, path: Path, cls: str) -> str:
        return f"{path.resolve().as_posix()}:{cls}"

    def _set_baseline(self, classes: dict[str, dict[str, int]], **extra: object) -> None:
        data = {"version": 1, "created": "2026-01-01", "base_sha": "x", "classes": classes}
        data.update(extra)
        self.baseline.write_text(json.dumps(data), encoding="utf-8")

    def _run(self, *argv: str) -> tuple[int, str]:
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            code = class_budget.main(["--baseline", str(self.baseline), *argv])
        return code, buf.getvalue()

    # ---------------------------------------------------------------- metrics
    def test_metrics_and_nested_class_counted_separately(self) -> None:
        # Put Inner inside Outer's body.
        source = _cls("Outer", 3, cc_each=4) + _cls("Inner", 2, cc_each=2, indent="    ")
        path = self._write("m.py", source)
        stats = class_budget.measure_file(path)
        self.assertEqual(stats["Outer"], {"wmc": 12, "methods": 3})
        self.assertEqual(stats["Outer.Inner"], {"wmc": 4, "methods": 2})

    def test_async_methods_counted(self) -> None:
        path = self._write("m.py", "class A:\n    async def a(self):\n        pass\n    def b(self):\n        pass\n")
        self.assertEqual(class_budget.measure_file(path)["A"], {"wmc": 2, "methods": 2})

    def test_nested_big_class_is_a_separate_entry(self) -> None:
        source = _cls("Outer", 1) + _cls("Inner", 41, indent="    ")
        path = self._write("m.py", source)
        self._set_baseline({})
        code, out = self._run(str(path))
        self.assertEqual(code, 1, out)
        self.assertIn(":Outer.Inner", out)
        self.assertNotIn(":Outer ", out)

    # ------------------------------------------------------------------ check
    def test_no_big_classes_passes(self) -> None:
        path = self._write("m.py", _cls("Small", 5))
        self._set_baseline({})
        code, out = self._run(str(path))
        self.assertEqual(code, 0, out)
        self.assertIn("class_budget: 0 big classes", out)

    def test_new_big_class_fails_by_methods(self) -> None:
        path = self._write("m.py", _cls("God", 41))
        self._set_baseline({})
        code, out = self._run(str(path))
        self.assertEqual(code, 1, out)
        self.assertIn("new big class", out)

    def test_new_big_class_fails_by_wmc(self) -> None:
        path = self._write("m.py", _cls("Heavy", 5, cc_each=17))  # WMC=85, 5 methods
        self._set_baseline({})
        code, out = self._run(str(path))
        self.assertEqual(code, 1, out)

    def test_threshold_boundaries_are_not_big(self) -> None:
        path = self._write("m.py", _cls("Edge", 40) + _cls("Edge2", 8, cc_each=10))  # 40 methods; WMC 80
        self._set_baseline({})
        code, out = self._run(str(path))
        self.assertEqual(code, 0, out)

    def test_ok_when_equal_to_baseline(self) -> None:
        path = self._write("m.py", _cls("God", 41))
        self._set_baseline({self._key(path, "God"): {"wmc": 41, "methods": 41}})
        code, out = self._run(str(path))
        self.assertEqual(code, 0, out)
        self.assertIn("class_budget: 1 big classes (sum WMC=41, methods=41)", out)

    def test_growth_fails(self) -> None:
        path = self._write("m.py", _cls("God", 42))
        self._set_baseline({self._key(path, "God"): {"wmc": 41, "methods": 41}})
        code, out = self._run(str(path))
        self.assertEqual(code, 1, out)
        self.assertIn("grew", out)

    def test_wmc_growth_alone_fails(self) -> None:
        path = self._write("m.py", _cls("God", 41, cc_each=2))
        self._set_baseline({self._key(path, "God"): {"wmc": 82, "methods": 41}})
        self.assertEqual(self._run(str(path))[0], 0)
        self._set_baseline({self._key(path, "God"): {"wmc": 81, "methods": 41}})
        code, out = self._run(str(path))
        self.assertEqual(code, 1, out)
        self.assertIn("grew", out)

    def test_shrink_fails_with_refresh_hint(self) -> None:
        path = self._write("m.py", _cls("God", 41))
        self._set_baseline({self._key(path, "God"): {"wmc": 90, "methods": 45}})
        code, out = self._run(str(path))
        self.assertEqual(code, 1, out)
        self.assertIn("--update-baseline", out)

    def test_no_longer_big_fails(self) -> None:
        path = self._write("m.py", _cls("God", 5))
        self._set_baseline({self._key(path, "God"): {"wmc": 90, "methods": 45}})
        code, out = self._run(str(path))
        self.assertEqual(code, 1, out)
        self.assertIn("no longer big", out)

    def test_phantom_fails(self) -> None:
        path = self._write("m.py", _cls("Small", 2))
        self._set_baseline({self._key(path, "Gone"): {"wmc": 90, "methods": 45}})
        code, out = self._run(str(path))
        self.assertEqual(code, 1, out)
        self.assertIn("phantom", out)

    def test_subset_run_has_no_false_phantom(self) -> None:
        scanned = self._write("a.py", _cls("God", 41))
        other = self._write("b.py", _cls("Other", 50))  # not scanned, still exists
        self._set_baseline(
            {
                self._key(scanned, "God"): {"wmc": 41, "methods": 41},
                self._key(other, "Other"): {"wmc": 99, "methods": 50},
            }
        )
        code, out = self._run(str(scanned))
        self.assertEqual(code, 0, out)
        self.assertNotIn("phantom", out)

    def test_deleted_file_entry_is_phantom_in_default_scope_run(self) -> None:
        scanned = self._write("a.py", _cls("God", 41))
        gone = self.root / "deleted.py"  # never created == deleted
        self._set_baseline(
            {
                self._key(scanned, "God"): {"wmc": 41, "methods": 41},
                self._key(gone, "Old"): {"wmc": 99, "methods": 50},
            }
        )
        code, out = self._run(str(scanned))
        self.assertEqual(code, 1, out)
        self.assertIn("file no longer exists", out)

    def test_update_drops_entries_of_deleted_files(self) -> None:
        scanned = self._write("a.py", _cls("God", 41))
        gone = self.root / "deleted.py"
        self._set_baseline({self._key(gone, "Old"): {"wmc": 99, "methods": 50}})
        code, out = self._run("--update-baseline", str(scanned))
        self.assertEqual(code, 0, out)
        data = json.loads(self.baseline.read_text(encoding="utf-8"))
        self.assertNotIn(self._key(gone, "Old"), data["classes"])
        self.assertIn("[drop]", out)

    # ----------------------------------------------------------------- update
    def test_update_snapshots_and_preserves_metadata_and_unscanned(self) -> None:
        scanned = self._write("a.py", _cls("God", 41) + _cls("Small", 2))
        other = self._write("b.py", _cls("Other", 50))
        self._set_baseline(
            {self._key(other, "Other"): {"wmc": 99, "methods": 50}},
            _refactor_cards={"k": "#1 (card)"},
        )
        code, out = self._run("--update-baseline", str(scanned))
        self.assertEqual(code, 0, out)
        data = json.loads(self.baseline.read_text(encoding="utf-8"))
        self.assertEqual(data["limits"], {"wmc": 80, "methods": 40})
        self.assertEqual(data["classes"][self._key(scanned, "God")], {"wmc": 41, "methods": 41})
        self.assertNotIn(self._key(scanned, "Small"), data["classes"])
        self.assertIn(self._key(other, "Other"), data["classes"])
        self.assertEqual(data["_refactor_cards"], {"k": "#1 (card)"})
        self.assertEqual(self._run(str(scanned))[0], 0)


if __name__ == "__main__":
    unittest.main()
