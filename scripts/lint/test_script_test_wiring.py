"""Self-test гарда script_test_wiring (issue #5176).

Мутация списка — единственное доказательство, что гард ловит дыру, а не
молчит. Три случая: новый файл без упоминания красный; тот же файл,
названный в workflow, зелёный; тот же файл в allowlist с причиной зелёный;
пустая причина — ошибка загрузки, не молчаливый пропуск.
"""

from __future__ import annotations

import json
import unittest
from pathlib import Path
from tempfile import TemporaryDirectory
from unittest.mock import patch

from scripts.lint import script_test_wiring as guard

REPO = Path(__file__).resolve().parents[2]


class WiringGuardTest(unittest.TestCase):
    def setUp(self) -> None:
        self.tmp = TemporaryDirectory()
        self.root = Path(self.tmp.name)
        self.scripts = self.root / "tests" / "unit" / "scripts"
        self.scripts.mkdir(parents=True)
        self.wf = self.root / ".github" / "workflows"
        self.wf.mkdir(parents=True)
        (self.wf / "G-Lint Code.yml").write_text("jobs: {}\n", encoding="utf-8")
        self.allow = self.root / "allow.json"
        self.allow.write_text("{}\n", encoding="utf-8")

    def tearDown(self) -> None:
        self.tmp.cleanup()

    def _patch_tree(self, on_ref: set[str], ref_workflows: str = ""):
        return patch.multiple(
            guard,
            REPO=self.root,
            WF_DIR=self.wf,
            names_at=lambda ref: on_ref,
            workflow_text_at=lambda ref: ref_workflows,
            _ref_exists=lambda ref: False,
        )

    def test_new_file_not_in_workflow_fails(self) -> None:
        (self.scripts / "test_issue_9999_new.py").write_text("def test_x():\n    pass\n", encoding="utf-8")
        with self._patch_tree(set()):
            missing = guard.unwired("origin/develop", self.root, self.allow)
        self.assertEqual(missing, ["test_issue_9999_new.py"])

    def test_file_unwired_on_develop_is_grandfathered(self) -> None:
        (self.scripts / "test_old.py").write_text("def test_x():\n    pass\n", encoding="utf-8")
        with self._patch_tree({"test_old.py"}, ref_workflows="jobs: {}\n"):
            missing = guard.unwired("origin/develop", self.root, self.allow)
        self.assertEqual(missing, [])

    def test_dropping_a_file_wired_on_ref_fails(self) -> None:
        (self.scripts / "test_issue_5170_hil_off_ready.py").write_text("def test_x():\n    pass\n", encoding="utf-8")
        ref_workflows = "pytest tests/unit/scripts/test_issue_5170_hil_off_ready.py\n"
        with self._patch_tree({"test_issue_5170_hil_off_ready.py"}, ref_workflows):
            missing = guard.unwired("origin/develop", self.root, self.allow)
        self.assertEqual(missing, ["test_issue_5170_hil_off_ready.py"])

    def test_rollback_after_merge_fails(self) -> None:
        # HEAD уже содержит гард и называет файл; рабочее дерево строку убрало.
        name = "test_issue_5170_hil_off_ready.py"
        (self.scripts / name).write_text("def test_x():\n    pass\n", encoding="utf-8")
        head_workflows = f"pytest {name}\npython scripts/lint/script_test_wiring.py\n"

        def text_at(ref: str) -> str:
            return "" if ref == "origin/develop" else head_workflows

        with patch.multiple(
            guard,
            REPO=self.root,
            WF_DIR=self.wf,
            names_at=lambda ref: {name},
            workflow_text_at=text_at,
            _ref_exists=lambda ref: True,
        ):
            missing = guard.unwired("origin/develop", self.root, self.allow)
        self.assertEqual(missing, [name])

    def test_new_file_named_in_workflow_passes(self) -> None:
        (self.scripts / "test_issue_9999_new.py").write_text("def test_x():\n    pass\n", encoding="utf-8")
        (self.wf / "G-Lint Code.yml").write_text(
            "python -m pytest tests/unit/scripts/test_issue_9999_new.py\n",
            encoding="utf-8",
        )
        with self._patch_tree(set()):
            missing = guard.unwired("origin/develop", self.root, self.allow)
        self.assertEqual(missing, [])

    def test_allowlist_with_reason_passes(self) -> None:
        (self.scripts / "test_issue_9999_new.py").write_text("def test_x():\n    pass\n", encoding="utf-8")
        self.allow.write_text(
            json.dumps({"test_issue_9999_new.py": "историческое, гоняется вручную на роботе"}),
            encoding="utf-8",
        )
        with self._patch_tree(set()):
            missing = guard.unwired("origin/develop", self.root, self.allow)
        self.assertEqual(missing, [])

    def test_empty_reason_is_load_error(self) -> None:
        self.allow.write_text(json.dumps({"test_issue_9999_new.py": "   "}), encoding="utf-8")
        with self.assertRaises(ValueError):
            guard.load_allow(self.allow)

    def test_live_tree_has_no_unwired_new_files(self) -> None:
        code, message = guard.check("origin/develop")
        self.assertEqual(code, 0, message)
        self.assertIn("OK", message)

    def test_wired_files_are_named_in_workflow(self) -> None:
        # Доказательство подключения не зависит от того, влит ли файл:
        # имя обязано стоять в workflow, а не только лежать в каталоге.
        text = guard.workflow_text()
        for name in (
            "test_issue_5170_hil_off_ready.py",
            "test_issue_5077_series_marker.py",
        ):
            self.assertIn(name, text, name)


if __name__ == "__main__":
    unittest.main()
