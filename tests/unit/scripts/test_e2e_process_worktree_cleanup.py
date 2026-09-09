"""Regression test for issue #1707.

`scripts/agent_flow/agent-flow-e2e-process.sh` создавал /tmp/agent-flow-e2e-<PID>/
на каждый round, но cleanup() через `trap cleanup EXIT INT TERM` не срабатывал
для SIGKILL/OOM/reboot. 437+ worktree'ов скопились и забили 87 ГБ на /tmp.

Этот тест пинит contract cleanup-логики:

1. Скрипт должен содержать функции `_wt_disk_check`, `_wt_sweep_orphans`,
   `_wt_sweep_ttl`, `_wt_remove_single`, `_wt_is_pid_alive` — реализующие
   защиту от переполнения /tmp.

2. Эти функции должны вызываться ДО flock (либо сразу после него) — до
   `ensure_worktree`, чтобы диск/cleanup проверялись на КАЖДОМ тике, а не
   только при первом.

3. Артефакты (`artifact_dir`) должны жить в `$ARTIFACTS_DIR`, а НЕ в
   `$WORKTREE_DIR/.e2e-artifacts/` — иначе `cleanup()` через trap удалил бы
   их вместе с worktree, а issue-комментарии уже содержат ссылки на файлы.

4. TTL/порог диска должны быть настраиваемыми через env:
   - `E2E_WT_TTL_DAYS` (default 7)
   - `E2E_DISK_MIN_GB` (default 20)

5. `cleanup()` (через `trap EXIT INT TERM`) должен дополнительно вызывать
   `git worktree prune` — чистка orphaned refs в main clone.

Зачем Python (не чистый bash): cleanup-логика распределена по разным местам
скрипта (функции + trap + вызовы + переменные). Source-level гарантии
(«функция определена ДО её использования», «переменная объявлена ДО trap»)
проще пинить в pytest, чем в bash-test.

Тест НЕ запускает скрипт целиком (нужен gh/репо) — только проверяет
source-level контракт.
"""
from __future__ import annotations

import pathlib
import re
import subprocess
import unittest

REPO_ROOT = pathlib.Path(__file__).resolve().parents[3]
SCRIPT_PATH = REPO_ROOT / "scripts" / "agent_flow" / "agent-flow-e2e-process.sh"


def _script_text() -> str:
    return SCRIPT_PATH.read_text()


def _line_of(text: str, needle_regex: str) -> int | None:
    """Return 1-indexed line number of first regex match, or None.

    Uses re.MULTILINE so `^` and `$` anchor at line boundaries.
    """
    pat = re.compile(needle_regex, re.MULTILINE)
    for i, line in enumerate(text.splitlines(), start=1):
        if pat.search(line):
            return i
    return None


def _function_defined(text: str, fn_name: str) -> int | None:
    """Return line where 'fn_name() {' is defined."""
    return _line_of(text, rf"^{re.escape(fn_name)}\s*\(\)\s*\{{")


class CleanupContractTest(unittest.TestCase):
    """Pin the cleanup contract added by issue #1707."""

    @classmethod
    def setUpClass(cls) -> None:
        if not SCRIPT_PATH.exists():
            raise unittest.SkipTest(f"script not present at {SCRIPT_PATH}")

    def test_script_parses(self) -> None:
        """Bash syntax check — no regressions from our edits."""
        r = subprocess.run(
            ["bash", "-n", str(SCRIPT_PATH)],
            capture_output=True,
            text=True,
        )
        self.assertEqual(
            r.returncode,
            0,
            f"bash -n failed:\nstderr: {r.stderr}\nstdout: {r.stdout}",
        )

    def test_disk_check_function_defined(self) -> None:
        text = _script_text()
        line = _function_defined(text, "_wt_disk_check")
        self.assertIsNotNone(line, "_wt_disk_check must be defined")
        # _wt_disk_check returns 1 если free < min → caller делает exit 0.
        self.assertIn("disk check FAIL", text, "_wt_disk_check must log failure")

    def test_sweep_orphans_function_defined(self) -> None:
        text = _script_text()
        line = _function_defined(text, "_wt_sweep_orphans")
        self.assertIsNotNone(line, "_wt_sweep_orphans must be defined")
        # Should call _wt_remove_single + handle own PID (skip self).
        self.assertIn("_wt_self_pid", text, "_wt_self_pid guard required")
        # Should reference kill -0 (PID alive check).
        self.assertIn("kill -0", text, "kill -0 used for PID alive check")

    def test_sweep_ttl_function_defined(self) -> None:
        text = _script_text()
        line = _function_defined(text, "_wt_sweep_ttl")
        self.assertIsNotNone(line, "_wt_sweep_ttl must be defined")
        # Should use find -mtime (TTL mechanism).
        self.assertIn("-mtime", text, "find -mtime required for TTL")
        self.assertIn("E2E_WT_TTL_DAYS", text, "TTL must read env var")

    def test_remove_single_function_defined(self) -> None:
        text = _script_text()
        line = _function_defined(text, "_wt_remove_single")
        self.assertIsNotNone(line, "_wt_remove_single must be defined")
        # Should call git worktree remove.
        self.assertIn("worktree remove --force", text)

    def test_disk_check_called_in_main_path(self) -> None:
        """_wt_disk_check must be invoked under flock, before any heavy work."""
        text = _script_text()
        # Ищем КОНКРЕТНЫЕ top-level вызовы (не определения, не комментарии).
        # Anchor: ровно 0 пробелов в начале + 'exec 9>' (lock acquisition).
        flock_line = _line_of(text, r"^exec 9>")
        # Вызовы функций cleanup'а: в начале строки ровно 0 пробелов + имя.
        sweep_line = _line_of(text, r"^_wt_sweep_orphans \|\| true$")
        check_line = _line_of(text, r"^_wt_disk_check \|\| exit 0$")
        self.assertIsNotNone(check_line, "_wt_disk_check must be invoked")
        self.assertIsNotNone(flock_line, "flock line must exist")
        self.assertIsNotNone(sweep_line, "_wt_sweep_orphans must be invoked")
        assert check_line is not None
        assert flock_line is not None
        assert sweep_line is not None
        self.assertGreater(check_line, flock_line,
                           "_wt_disk_check must be after flock")
        self.assertGreater(sweep_line, flock_line,
                           "_wt_sweep_orphans must be after flock")

    def test_artifact_dir_outside_worktree(self) -> None:
        """Issue #1707: artifacts must NOT live inside worktree (cleanup wipes them)."""
        text = _script_text()
        # Must declare ARTIFACTS_DIR env override.
        self.assertIn("E2E_ARTIFACTS_DIR", text)
        # Must use $ARTIFACTS_DIR for artifact_dir, not $WORKTREE_DIR.
        # Old code: artifact_dir="${WORKTREE_DIR}/.e2e-artifacts/${number}"
        # New code: artifact_dir="${ARTIFACTS_DIR}/${number}"
        bad = re.search(
            r'artifact_dir=.*\$\{?WORKTREE_DIR\}?/\.e2e-artifacts',
            text,
        )
        self.assertIsNone(bad,
                          "artifact_dir still inside WORKTREE_DIR — would "
                          "be deleted by cleanup() trap (issue #1707)")
        good = re.search(r'artifact_dir="\$\{ARTIFACTS_DIR\}', text)
        self.assertIsNotNone(good,
                             "artifact_dir must reference $ARTIFACTS_DIR")

    def test_cleanup_calls_worktree_prune(self) -> None:
        """cleanup() must additionally call `git worktree prune`."""
        text = _script_text()
        # Find cleanup() definition block — ends at next ^} at column 0.
        m = re.search(
            r'^cleanup\(\)\s*\{(.*?)\n\}\n',
            text,
            flags=re.MULTILINE | re.DOTALL,
        )
        self.assertIsNotNone(m, "cleanup() function not found")
        assert m is not None  # narrow for Pyright
        body = m.group(1)
        self.assertIn("worktree prune", body,
                      "cleanup() must call `git worktree prune` "
                      "(issue #1707)")

    def test_ttl_default_is_seven_days(self) -> None:
        """TTL default = 7 days (acceptance criterion)."""
        text = _script_text()
        m = re.search(r'E2E_WT_TTL_DAYS:=\s*(\d+)', text)
        self.assertIsNotNone(m, "E2E_WT_TTL_DAYS default not set")
        assert m is not None  # narrow for Pyright
        self.assertEqual(m.group(1), "7",
                         "TTL default must be 7 days (issue #1707)")

    def test_disk_min_default_is_20gb(self) -> None:
        """Disk-space fail-fast default = 20 GB."""
        text = _script_text()
        m = re.search(r'E2E_DISK_MIN_GB:=\s*(\d+)', text)
        self.assertIsNotNone(m, "E2E_DISK_MIN_GB default not set")
        assert m is not None  # narrow for Pyright
        self.assertEqual(m.group(1), "20",
                         "Disk min default must be 20 GB (issue #1707)")

    def test_documentation_has_cleanup_section(self) -> None:
        """Header must contain `Cleanup contract` section."""
        text = _script_text()
        self.assertIn("Cleanup contract", text,
                      "Header must document cleanup behavior "
                      "(acceptance criterion)")
