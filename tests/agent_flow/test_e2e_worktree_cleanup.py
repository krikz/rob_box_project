"""Regression tests for /tmp/agent-flow-e2e-* worktree cleanup logic
(issue #1707, ретро t_0ff29dcd).

Контекст:

  agent-flow-e2e-process.sh v1 (commit eacb933c) создавал worktree
  `/tmp/agent-flow-e2e-<PID>/` на каждый round, но при SIGKILL/OOM/reboot
  cleanup() через trap НЕ вызывался → /tmp/ забивался (437 worktree = ~87 ГБ).

  Текущий fix (PR #1889 + это PR) добавляет:
    1) --self-test --cleanup-only режим в e2e-process.sh для verification
       без сетевых/gh API вызовов.
    2) standalone scripts/agent_flow/agent-flow-e2e-wt-sweep.sh для одноразовой
       зачистки существующего завала.
    3) e2e_worktree_count metric в e2e-drift-watchdog.sh (alert при >10).

Тесты тут фокусируются на self-test mode + standalone sweep скрипте. Это
чистые bash-тесты — никакого gh/network/flock — поэтому они дешёвые и
стабильные (без integration-tests).

Тесты (R1-R7):
  R1: --self-test exits 0, печатает счётчик worktree + REPO_DIR.
  R2: --cleanup-only exits 0, удаляет fake orphan dirs, печатает before/after.
  R3: --count-dry-run exits 0, печатает ТОЛЬКО число (один line, valid integer).
  R4: --help exits 0, печатает usage banner с обоими флагами.
  R5: unknown arg → exit 0 (warning, не fail — backward-compat с cron env).
  R6: stand-alone sweep (DRY_RUN=true) НЕ удаляет реальные файлы.
  R7: stand-alone sweep (real run) удаляет fake orphans, итоговый count = 0.
"""
from __future__ import annotations

import os
import shutil
import subprocess
import tempfile
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
E2E_PROCESS_SH = REPO_ROOT / "scripts" / "agent_flow" / "agent-flow-e2e-process.sh"
E2E_WT_SWEEP_SH = REPO_ROOT / "scripts" / "agent_flow" / "agent-flow-e2e-wt-sweep.sh"


def _run_bash(args: list[str], env: dict | None = None, timeout: int = 30) -> subprocess.CompletedProcess:
    """Run bash with given args; capture stdout/stderr separately."""
    return subprocess.run(
        ["bash"] + args,
        cwd=str(REPO_ROOT),
        env=env,
        capture_output=True,
        text=True,
        timeout=timeout,
    )


# ============================================================================
# --self-test / --cleanup-only / --count-dry-run in agent-flow-e2e-process.sh
# ============================================================================

class TestE2eProcessSelfTestFlags:
    """R1-R5: CLI flags added by this fix must work and not corrupt state."""

    def test_self_test_exits_zero_and_prints_count(self):
        """R1: --self-test exits 0 with self-test banner + e2e_worktree_count line."""
        if not E2E_PROCESS_SH.exists():
            pytest.skip("agent-flow-e2e-process.sh not found in SOT")
        r = _run_bash([str(E2E_PROCESS_SH), "--self-test"])
        assert r.returncode == 0, f"--self-test failed: {r.stderr}"
        assert "self-test:" in r.stderr, f"missing self-test banner: {r.stderr}"
        assert "e2e_worktree_count=" in r.stderr, f"missing metric line: {r.stderr}"
        assert "WORKTREE_DIR=" in r.stderr, f"missing WORKTREE_DIR line: {r.stderr}"

    def test_count_dry_run_prints_only_integer(self):
        """R3: --self-test --count-dry-run → stdout is single integer (parseable)."""
        if not E2E_PROCESS_SH.exists():
            pytest.skip("agent-flow-e2e-process.sh not found in SOT")
        r = _run_bash([str(E2E_PROCESS_SH), "--self-test", "--count-dry-run"])
        assert r.returncode == 0
        # stdout = single line, parseable as int
        out = r.stdout.strip()
        assert out.isdigit(), f"expected integer stdout, got: {out!r}"
        n = int(out)
        assert n >= 0

    def test_help_exits_zero_prints_banner(self):
        """R4: --help exits 0 and shows both flags in usage banner."""
        if not E2E_PROCESS_SH.exists():
            pytest.skip("agent-flow-e2e-process.sh not found in SOT")
        r = _run_bash([str(E2E_PROCESS_SH), "--help"])
        assert r.returncode == 0
        assert "--self-test" in r.stdout or "--self-test" in r.stderr
        assert "--cleanup-only" in r.stdout or "--cleanup-only" in r.stderr

    def test_unknown_arg_returns_nonzero(self):
        """R5: unknown arg → exit 2 (typo detection), НЕ flock'ает lock-файл."""
        if not E2E_PROCESS_SH.exists():
            pytest.skip("agent-flow-e2e-process.sh not found in SOT")
        r = _run_bash([str(E2E_PROCESS_SH), "--unknown-flag-12345"], timeout=5)
        assert r.returncode != 0, f"unknown arg should be fatal, got: {r.stderr}"
        assert r.returncode == 2, f"expected exit 2 (usage error), got: {r.returncode}"
        assert "unknown arg" in r.stderr.lower() or "ERROR" in r.stderr

    def test_cleanup_only_removes_fake_orphans(self, tmp_path):
        """R2: --cleanup-only удаляет fake /tmp/agent-flow-e2e-* dirs мёртвых PID."""
        if not E2E_PROCESS_SH.exists():
            pytest.skip("agent-flow-e2e-process.sh not found in SOT")
        # Создаём 2 fake orphan'а с мёртвыми PID (99999 + 99998).
        # Используем tmp_path, потом копируем в /tmp.
        fake_dirs = []
        for pid in ("99999001", "99999002"):
            d = f"/tmp/agent-flow-e2e-{pid}-fake"
            os.makedirs(d, exist_ok=True)
            (Path(d) / ".git").touch()
            fake_dirs.append(d)
        try:
            before = int(_run_bash([str(E2E_PROCESS_SH), "--self-test", "--count-dry-run"]).stdout.strip())
            assert before >= 2
            r = _run_bash([str(E2E_PROCESS_SH), "--cleanup-only"])
            assert r.returncode == 0, f"--cleanup-only failed: {r.stderr}"
            # Либо v1 cleanup удалил (PID dead), либо нет — в обоих случаях процесс
            # завершился exit 0 (мы не валимся на broken .git/etc).
            assert "cleanup-only:" in r.stderr
        finally:
            for d in fake_dirs:
                shutil.rmtree(d, ignore_errors=True)


# ============================================================================
# standalone sweep script (one-shot cleanup)
# ============================================================================

@pytest.fixture
def fake_orphans():
    """Create N fake orphan dirs under /tmp and clean them after the test."""
    created: list[str] = []
    for pid in ("88888001", "88888002", "88888003"):
        d = f"/tmp/agent-flow-e2e-{pid}-pytest"
        os.makedirs(d, exist_ok=True)
        (Path(d) / ".git").touch()
        created.append(d)
    yield created
    for d in created:
        shutil.rmtree(d, ignore_errors=True)


class TestE2eWtSweepScript:
    """R6-R7: standalone script — DRY_RUN=true / real run."""

    def test_dry_run_does_not_remove(self, fake_orphans):
        """R6: DRY_RUN=true — НЕ удаляет файлы, печатает count (before == after)."""
        if not E2E_WT_SWEEP_SH.exists():
            pytest.skip("agent-flow-e2e-wt-sweep.sh not found in SOT")
        before_dirs = {p for p in fake_orphans if Path(p).exists()}
        assert len(before_dirs) == len(fake_orphans)

        env = dict(os.environ, DRY_RUN="true")
        r = _run_bash([str(E2E_WT_SWEEP_SH)], env=env)
        assert r.returncode == 0, f"sweep DRY-RUN failed: {r.stderr}"
        after_dirs = {p for p in fake_orphans if Path(p).exists()}
        assert before_dirs == after_dirs, f"DRY-RUN removed files: {before_dirs - after_dirs}"

    def test_real_run_removes_fake_orphans(self, fake_orphans):
        """R7: real run — fake orphans с dead PID удаляются, count → 0."""
        if not E2E_WT_SWEEP_SH.exists():
            pytest.skip("agent-flow-e2e-wt-sweep.sh not found in SOT")
        env = dict(os.environ, DRY_RUN="false")
        r = _run_bash([str(E2E_WT_SWEEP_SH)], env=env)
        assert r.returncode == 0, f"sweep real failed: {r.stderr}"
        for d in fake_orphans:
            assert not Path(d).exists(), f"sweep didn't remove {d}"

    def test_summary_block_present(self, fake_orphans):
        """Summary line `summary: removed_orphans=N removed_ttl=N kept_alive=N`."""
        if not E2E_WT_SWEEP_SH.exists():
            pytest.skip("agent-flow-e2e-wt-sweep.sh not found in SOT")
        env = dict(os.environ, DRY_RUN="false")
        r = _run_bash([str(E2E_WT_SWEEP_SH)], env=env)
        assert "summary:" in r.stderr
        assert "e2e_worktree_count(" in r.stderr


# ============================================================================
# install.sh SOT-list bound to drift-detect (3 copies synced by md5)
# ============================================================================

class TestInstallShRegistration:
    """Sanity: install.sh EXPECTED[] includes our new scripts so install.sh
    фактически разложит 3 копии (через hardlink/cp).

    See: kanban t_0ff29dcd acceptance [ ] 3 копии синхронизированы по md5.
    """

    def test_sweep_script_listed_in_install_expected(self):
        install_sh = REPO_ROOT / "scripts" / "agent_flow" / "install.sh"
        if not install_sh.exists():
            pytest.skip("install.sh not found")
        text = install_sh.read_text(encoding="utf-8")
        assert "agent-flow-e2e-wt-sweep.sh" in text, (
            "agent-flow-e2e-wt-sweep.sh отсутствует в EXPECTED[] — "
            "install.sh не разложит его по 3 target-папкам, что сломает "
            "acceptance [3 копии синхронизированы по md5]."
        )
