#!/usr/bin/env python3
"""Self-tests for scripts/lint/cc_budget.py (ADR-0021 R1, ADR-0021-r1).

Same convention as ``test_cc_budget_refs.py``: the guard's own correctness is
protected by tests, so a future refactor cannot silently re-open a hole.

| Defect | Test |
|---|---|
| D1 over-limit method dropped below baseline stays ``[ok]`` | test_over_limit_shrink_fails |
| D2 --update-baseline wipes _refactor_cards/_legacy_acknowledged | test_update_preserves_metadata |
| D2 legacy growth must be refused | test_update_refuses_legacy_growth |
| D2 R-1e new exemption CC>30 needs _adr_reference | test_update_refuses_new_hard_exempt_without_adr |
| D3 subset run reports phantoms for unscanned files | test_subset_run_has_no_false_phantom |
| D3 subset update deletes unscanned exemptions | test_subset_update_keeps_unscanned_exemptions |

Run:
    python -m unittest scripts.lint.test_cc_budget -v
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
import cc_budget  # noqa: E402


def _src(**funcs: int) -> str:
    """Python source with module-level functions of the requested CC."""
    out = []
    for name, cc in funcs.items():
        body = "".join(f"    if x == {i}:\n        x += 1\n" for i in range(cc - 1)) or "    pass\n"
        out.append(f"def {name}(x):\n{body}    return x\n")
    return "\n".join(out)


class CcBudgetTest(unittest.TestCase):
    def setUp(self) -> None:
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        self.root = Path(self._tmp.name).resolve()
        self._orig = cc_budget.BASELINE_FILE
        cc_budget.BASELINE_FILE = self.root / "baseline.json"
        self.addCleanup(setattr, cc_budget, "BASELINE_FILE", self._orig)

    def _write(self, name: str, **funcs: int) -> Path:
        path = self.root / name
        path.write_text(_src(**funcs), encoding="utf-8")
        return path

    def _key(self, path: Path) -> str:
        return path.resolve().as_posix()

    def _set_baseline(self, data: dict) -> None:
        base = {"version": 1, "created": "2026-01-01", "base_sha": "x", "limits": {"method": 15, "init": 20}}
        base.update(data)
        cc_budget.BASELINE_FILE.write_text(json.dumps(base), encoding="utf-8")

    def _baseline(self) -> dict:
        return json.loads(cc_budget.BASELINE_FILE.read_text(encoding="utf-8"))

    def _run(self, *argv: str) -> tuple[int, str]:
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            code = cc_budget.main(list(argv))
        return code, buf.getvalue()

    # ------------------------------------------------------------------ check
    def test_ok_when_equal_to_baseline(self) -> None:
        path = self._write("a.py", big=20)
        self._set_baseline({"exemptions": {self._key(path): {"big": 20}}})
        code, out = self._run(str(path))
        self.assertEqual(code, 0, out)

    def test_over_limit_shrink_fails(self) -> None:
        """D1: CC=18 > limit 15 but < baseline 25 must FAIL (ratchet)."""
        path = self._write("a.py", big=18)
        self._set_baseline({"exemptions": {self._key(path): {"big": 25}}})
        code, out = self._run(str(path))
        self.assertEqual(code, 1, out)
        self.assertIn("dropped below baseline 25", out)

    def test_below_limit_shrink_fails(self) -> None:
        path = self._write("a.py", big=10)
        self._set_baseline({"exemptions": {self._key(path): {"big": 25}}})
        code, out = self._run(str(path))
        self.assertEqual(code, 1, out)
        self.assertIn("recovered below baseline", out)

    def test_growth_fails(self) -> None:
        path = self._write("a.py", big=22)
        self._set_baseline({"exemptions": {self._key(path): {"big": 20}}})
        code, out = self._run(str(path))
        self.assertEqual(code, 1, out)
        self.assertIn("grew past baseline", out)

    def test_new_violation_fails(self) -> None:
        path = self._write("a.py", big=18)
        self._set_baseline({"exemptions": {}})
        code, _ = self._run(str(path))
        self.assertEqual(code, 1)

    def test_phantom_in_scanned_file_fails(self) -> None:
        path = self._write("a.py", small=3)
        self._set_baseline({"exemptions": {self._key(path): {"gone": 20}}})
        code, out = self._run(str(path))
        self.assertEqual(code, 1, out)
        self.assertIn("phantom", out)

    def test_deleted_file_entry_is_phantom_in_default_scope_run(self) -> None:
        """Baseline entry of a deleted file is a phantom even if the file is not scanned."""
        scanned = self._write("a.py", big=20)
        gone = self.root / "deleted.py"  # never created == deleted
        self._set_baseline({"exemptions": {self._key(scanned): {"big": 20}, self._key(gone): {"old": 30}}})
        code, out = self._run(str(scanned))
        self.assertEqual(code, 1, out)
        self.assertIn("phantom", out)
        self.assertIn("file no longer exists", out)

    def test_subset_run_has_no_false_phantom(self) -> None:
        """D3: baseline entries of files outside the scanned set are ignored."""
        scanned = self._write("a.py", big=20)
        other = self._write("not_scanned.py", whatever=33)
        self._set_baseline({"exemptions": {self._key(scanned): {"big": 20}, self._key(other): {"whatever": 33}}})
        code, out = self._run(str(scanned))
        self.assertEqual(code, 0, out)
        self.assertNotIn("phantom", out)

    # ----------------------------------------------------------------- update
    def test_update_preserves_metadata(self) -> None:
        """D2: _refactor_cards / _legacy_acknowledged / _adr_reference survive."""
        path = self._write("a.py", big=20, other=18)
        key = self._key(path)
        self._set_baseline(
            {
                "exemptions": {key: {"big": 25, "other": 18}},
                "_refactor_cards": {f"{key}:other": "#1 (card)"},
                "_legacy_acknowledged": [
                    {"path": key, "method": "big", "cc": 25, "since": "s"},
                    {"path": key, "method": "other", "cc": 18, "since": "s"},
                ],
                "_adr_reference": {},
                "_custom": {"k": 1},
            }
        )
        code, out = self._run("--update-baseline", str(path))
        self.assertEqual(code, 0, out)
        data = self._baseline()
        self.assertEqual(data["exemptions"][key], {"big": 20, "other": 18})
        self.assertEqual(data["_refactor_cards"], {f"{key}:other": "#1 (card)"})
        legacy = {e["method"]: e["cc"] for e in data["_legacy_acknowledged"]}
        self.assertEqual(legacy, {"big": 20, "other": 18})  # lowered in sync, R-1a equality
        self.assertEqual(data["_custom"], {"k": 1})
        self.assertIn("_adr_reference", data)

    def test_update_drops_entries_no_longer_over_limit(self) -> None:
        path = self._write("a.py", big=20, fixed=5)
        key = self._key(path)
        self._set_baseline(
            {
                "exemptions": {key: {"big": 20, "fixed": 18}},
                "_refactor_cards": {f"{key}:fixed": "#2 (card)"},
                "_legacy_acknowledged": [{"path": key, "method": "fixed", "cc": 18, "since": "s"}],
            }
        )
        code, out = self._run("--update-baseline", str(path))
        self.assertEqual(code, 0, out)
        data = self._baseline()
        self.assertEqual(data["exemptions"][key], {"big": 20})
        self.assertEqual(data["_refactor_cards"], {})
        self.assertEqual(data["_legacy_acknowledged"], [])
        self.assertIn("[drop]", out)

    def test_update_refuses_legacy_growth(self) -> None:
        path = self._write("a.py", big=27)
        key = self._key(path)
        self._set_baseline(
            {
                "exemptions": {key: {"big": 25}},
                "_legacy_acknowledged": [{"path": key, "method": "big", "cc": 25, "since": "s"}],
            }
        )
        before = cc_budget.BASELINE_FILE.read_text(encoding="utf-8")
        code, out = self._run("--update-baseline", str(path))
        self.assertEqual(code, 1, out)
        self.assertIn("R-1a", out)
        self.assertEqual(cc_budget.BASELINE_FILE.read_text(encoding="utf-8"), before)

    def test_update_refuses_new_hard_exempt_without_adr(self) -> None:
        """D2/R-1e: new exemption with CC>30 needs an _adr_reference entry."""
        path = self._write("a.py", monster=35)
        key = self._key(path)
        self._set_baseline({"exemptions": {}})
        code, out = self._run("--update-baseline", str(path))
        self.assertEqual(code, 1, out)
        self.assertIn("R-1e", out)

        self._set_baseline({"exemptions": {}, "_adr_reference": {f"{key}:monster": "docs/adr/0145-x.md"}})
        code, out = self._run("--update-baseline", str(path))
        self.assertEqual(code, 0, out)
        self.assertEqual(self._baseline()["exemptions"][key], {"monster": 35})
        self.assertIn(f"{key}:monster", self._baseline()["_adr_reference"])

    def test_update_drops_entries_of_deleted_files(self) -> None:
        scanned = self._write("a.py", big=20)
        gone = self._key(self.root / "deleted.py")
        self._set_baseline(
            {
                "exemptions": {gone: {"old": 30}},
                "_legacy_acknowledged": [{"path": gone, "method": "old", "cc": 30, "since": "s"}],
                "_refactor_cards": {f"{gone}:old": "#4 (card)"},
            }
        )
        code, out = self._run("--update-baseline", str(scanned))
        self.assertEqual(code, 0, out)
        data = self._baseline()
        self.assertNotIn(gone, data["exemptions"])
        self.assertEqual(data["_legacy_acknowledged"], [])
        self.assertEqual(data["_refactor_cards"], {})
        self.assertIn("[drop]", out)

    def test_subset_update_keeps_unscanned_exemptions(self) -> None:
        """D3: update with a subset of paths must not delete other files' entries."""
        scanned = self._write("a.py", big=20)
        other = self._write("other.py", keep=30)
        self._set_baseline(
            {
                "exemptions": {self._key(other): {"keep": 30}},
                "_legacy_acknowledged": [{"path": self._key(other), "method": "keep", "cc": 30, "since": "s"}],
                "_refactor_cards": {f"{self._key(other)}:keep": "#3 (card)"},
            }
        )
        code, out = self._run("--update-baseline", str(scanned))
        self.assertEqual(code, 0, out)
        data = self._baseline()
        self.assertEqual(data["exemptions"][self._key(other)], {"keep": 30})
        self.assertEqual(data["exemptions"][self._key(scanned)], {"big": 20})
        self.assertEqual(len(data["_legacy_acknowledged"]), 1)
        self.assertEqual(len(data["_refactor_cards"]), 1)


if __name__ == "__main__":
    unittest.main()
