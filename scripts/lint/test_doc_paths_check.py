#!/usr/bin/env python3
"""Self-tests для scripts/lint/doc_paths_check.py (ADR-0166).

Run:
    python -m unittest scripts.lint.test_doc_paths_check -v
"""

from __future__ import annotations

import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import doc_paths_check as guard  # noqa: E402


class DocPathsCheckTest(unittest.TestCase):
    def setUp(self) -> None:
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        self.root = Path(tmp.name).resolve()
        old = guard.ROOT
        guard.ROOT = self.root
        self.addCleanup(setattr, guard, "ROOT", old)
        (self.root / "scripts").mkdir()
        (self.root / "scripts" / "real.sh").write_text("x", encoding="utf-8")

    def _check(self, text: str) -> list[str]:
        md = self.root / "docs" / "guide.md"
        md.parent.mkdir(exist_ok=True)
        md.write_text(text, encoding="utf-8")
        return [p for _, p in guard.check_file(md)]

    def test_existing_path_is_ok(self) -> None:
        self.assertEqual(self._check("запусти `scripts/real.sh` и `scripts/real.sh:12`"), [])

    def test_missing_path_is_reported(self) -> None:
        self.assertEqual(self._check("запусти `scripts/gone.sh`"), ["scripts/gone.sh"])

    def test_templates_and_actions_are_skipped(self) -> None:
        self.assertEqual(self._check("`scripts/*.sh` `docker/<svc>/x` `docker/build-push-action@v5`"), [])

    def test_runtime_files_are_skipped(self) -> None:
        self.assertEqual(self._check("секреты в `docker/studio/.env.secrets`, карты `docker/main/maps/`"), [])

    def test_line_mark_and_file_mark(self) -> None:
        self.assertEqual(self._check("`scripts/gone.sh` <!-- stale-paths: ignore -->"), [])
        self.assertEqual(self._check("<!-- stale-paths: ignore-file -->\n`scripts/gone.sh`"), [])

    def test_historical_section_is_skipped(self) -> None:
        text = "## Живое\n`scripts/real.sh`\n## Историческое\n`scripts/gone.sh`\n## Дальше\n`scripts/gone2.sh`\n"
        self.assertEqual(self._check(text), ["scripts/gone2.sh"])

    def test_tree_lines_are_skipped(self) -> None:
        self.assertEqual(self._check("```\n├── scripts/gone.sh\n```"), [])

    def test_path_relative_to_doc_dir(self) -> None:
        pkg = self.root / "pkg"
        (pkg / "src" / "scene").mkdir(parents=True)
        md = pkg / "README.md"
        md.write_text("см. `src/scene/`", encoding="utf-8")
        self.assertEqual(guard.check_file(md), [])


if __name__ == "__main__":
    unittest.main()
