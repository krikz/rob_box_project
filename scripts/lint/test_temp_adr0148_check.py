#!/usr/bin/env python3
"""Self-tests для scripts/lint/temp_adr0148_check.py (ADR-0148 / ADR-0150 A8, #3684).

Run:
    python -m unittest scripts.lint.test_temp_adr0148_check -v
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
import temp_adr0148_check as guard  # noqa: E402


class TempAdr0148Test(unittest.TestCase):
    def setUp(self) -> None:
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        self.root = Path(self._tmp.name).resolve()
        self.src = self.root / "src"
        self.baseline = self.root / "baseline.json"

    def _write(self, rel: str, text: str) -> None:
        path = self.src / rel
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text, encoding="utf-8")

    def _run(self, *args: str) -> tuple:
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            code = guard.main(["--src", str(self.src), "--baseline", str(self.baseline), *args])
        return code, out.getvalue()

    def _baseline(self, count: int) -> None:
        self.baseline.write_text(json.dumps({"count": count}), encoding="utf-8")

    def test_counts_marks_with_and_without_closing_paren(self) -> None:
        self._write("pkg/a.py", "# TEMP(ADR-0148) — #1\nx = 1\n\"\"\"TEMP(ADR-0148): doc\"\"\"\n")
        self.assertEqual(guard.find_marks(self.src), [("pkg/a.py", 1), ("pkg/a.py", 3)])

    def test_two_marks_on_one_line_count_twice(self) -> None:
        self._write("pkg/a.py", "# TEMP(ADR-0148) и ещё раз TEMP(ADR-0148)\n")
        self.assertEqual(len(guard.find_marks(self.src)), 2)

    def test_tests_are_excluded(self) -> None:
        self._write("pkg/test_x.py", "# TEMP(ADR-0148)\n")
        self._write("pkg/tests/helper.py", "# TEMP(ADR-0148)\n")
        self._write("pkg/test/helper.py", "# TEMP(ADR-0148)\n")
        self._write("pkg/conftest.py", "# TEMP(ADR-0148)\n")
        self.assertEqual(guard.find_marks(self.src), [])

    def test_growth_fails(self) -> None:
        self._write("pkg/a.py", "# TEMP(ADR-0148)\n# TEMP(ADR-0148)\n")
        self._baseline(1)
        code, out = self._run()
        self.assertEqual(code, 1)
        self.assertIn("FAIL", out)

    def test_equal_passes(self) -> None:
        self._write("pkg/a.py", "# TEMP(ADR-0148)\n")
        self._baseline(1)
        self.assertEqual(self._run()[0], 0)

    def test_shrink_passes_with_ratchet_hint(self) -> None:
        self._baseline(3)
        code, out = self._run()
        self.assertEqual(code, 0)
        self.assertIn("--update-baseline", out)

    def test_update_baseline_writes_current_count(self) -> None:
        self._write("pkg/a.py", "# TEMP(ADR-0148)\n")
        self._baseline(9)
        self.assertEqual(self._run("--update-baseline")[0], 0)
        self.assertEqual(guard.load_baseline(self.baseline), 1)

    def test_committed_baseline_matches_repo(self) -> None:
        """Живой baseline в репо не отстаёт от кода: рост поймает основной шаг CI, этот тест — рассинхрон вниз."""
        count = len(guard.find_marks(guard.REPO / "src"))
        self.assertLessEqual(count, guard.load_baseline())


if __name__ == "__main__":
    unittest.main()
