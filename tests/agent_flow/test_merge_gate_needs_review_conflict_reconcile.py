"""Regression tests for needs_review_conflict_reconcile_all() and the related
PR-side label transitions in agent-flow-merge-gate.sh.

Контекст (ретро 02.09 t_4869a1f7 / PR #1863):

PR #1863 (`z-backend/t_ad97d944-fix-dramaturgy`) был открыт с mergeable=MERGEABLE
+ CLEAN → merge-gate поставил метку `needs-review` (lint path, e2e-done reconcile
или clean-pr-sweep). Через несколько часов develop убежал вперёд (CI прогон
от develop принёс новые коммиты ef525468e, 96bb39302, 02c25bef8, 7ca4ed6cd) →
PR стал CONFLICTING+DIRTY. Но метка `needs-review` осталась.

Шифу видит `needs-review` в review queue → открывает PR → merge-ui показывает
«Pull request is not mergeable: This branch has conflicts that must be resolved».
Тратит время на разбор, потом пишет воркеру 'rebase'. PR не вливается, лаг.

`stale_conflicting_scan_all` (существующий, ретро 24.08 t_cd32788f) сканирует
ТОЛЬКО PR с меткой `needs-e2e` → PR с `needs-review + CONFLICTING` остаётся
серой зоной.

Фикс (t_4869a1f7):
  - Новая функция `needs_review_conflict_reconcile_all()` сканирует OPEN PR с
    меткой `needs-review` и reconcile'ит:
      * mergeable=CONFLICTING ИЛИ mergeStateStatus=DIRTY → снимает needs-review,
        ставит merge-conflict, постит rebase-инструкцию (24ч dedup).
      * mergeable=MERGEABLE + mergeStateStatus=CLEAN → если merge-conflict
        остался с прошлого CONFLICTING, снимает (recovery).
  - Вызывается в `scan-all-prs` блоке рядом с `stale_conflicting_scan_all`.

Тесты (R1-R5):
  R1: PR #1863 — needs-review + CONFLICTING + DIRTY (без merge-conflict)
      → ожидаем: gh pr edit --remove-label needs-review + whoami add merge-conflict
        + gh pr comment (rebase-инструкция).
  R2: PR #1867 — needs-review + MERGEABLE + CLEAN + merge-conflict (recovery).
      → ожидаем: gh pr edit --remove-label merge-conflict (recovery).
  R3: PR #X — needs-review + MERGEABLE + CLEAN, БЕЗ merge-conflict.
      → ожидаем: ничего не делаем (нормальный review state).
  R4: PR #X — needs-review + UNKNOWN (CI calc in progress).
      → ожидаем: ничего не делаем (false positive дороже чем пропуск).
  R5: PR с needs-review + CONFLICTING, но user_unlabel guard сработал
      (Шифу снял needs-review руками) → НЕ ставим merge-conflict.

Mock strategy: как в test_merge_gate_stale_branch_process_scripts.py — извлекаем
функцию через brace-counting, source'им в обёртке с PATH shim'ом для `gh`,
которая логирует все вызовы. Проверяем что были вызваны нужные команды.
"""

from __future__ import annotations

import os
import shlex
import subprocess
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT = REPO_ROOT / "scripts" / "agent_flow" / "agent-flow-merge-gate.sh"


# --------------------------------------------------------------------------- #
# gh shim: log all calls into a file we can inspect.
# --------------------------------------------------------------------------- #

GH_SHIM_TEMPLATE = r"""#!/usr/bin/env bash
# test shim for gh — see tests/agent_flow/test_merge_gate_needs_review_conflict_reconcile.py
# Args after `gh` are parsed: cmd=$1, subcmd=$2.
# Returns canned JSON for pr list (with needs-review label), no-op for api/edit/comment.
# Logs each call to ${CALL_LOG_FILE} (line-format: cmd|subcmd|args).
CALL_LOG="${CALL_LOG_FILE:-/tmp/gh_calls.log}"
echo "CALL|$1|$2|${*:3}" >> "$CALL_LOG"

cmd="$1"; shift || true
subcmd="$1"; shift || true

case "$cmd $subcmd" in
  "pr list")
    # If FIXTURE_FILE points to a JSON file with PR list, return it.
    if [ -n "${FIXTURE_FILE:-}" ] && [ -f "${FIXTURE_FILE}" ]; then
      cat "${FIXTURE_FILE}"
    else
      printf '[]'
    fi
    ;;
  "api")
    # gh api repos/.../issues/N/comments?since=... — return empty dedup-check JSON.
    case "$*" in
      *"comments?since="*) printf '[]' ;;
      *) printf '[]' ;;
    esac
    ;;
  "pr edit"|"pr comment"|"pr create")
    # Real call would touch GitHub. For tests we no-op (success exit).
    exit 0
    ;;
  "auth")
    printf 'mock-token'
    ;;
  *)
    # No-op for unknown commands (avoid test breakage on lib_*.sh helpers).
    exit 0
    ;;
esac
"""


def _make_gh_shim(bin_dir: Path) -> Path:
    shim = bin_dir / "gh"
    shim.write_text(GH_SHIM_TEMPLATE)
    shim.chmod(0o755)
    return shim


def _extract_function(name: str) -> str:
    """Extract a single function from merge-gate.sh by brace-counting.

    Returns the function text ready for `source` (no main-script state, no
    library dependencies — those are stubbed in the wrapper).
    """
    content = SCRIPT.read_text()
    lines = content.split("\n")
    start_idx = None
    for i, ln in enumerate(lines):
        if ln.startswith(f"{name}()"):
            start_idx = i
            break
    if start_idx is None:
        raise RuntimeError(f"function {name}() not found in script")
    depth = 0
    end_idx = None
    for j in range(start_idx, len(lines)):
        depth += lines[j].count("{") - lines[j].count("}")
        if depth == 0 and j > start_idx:
            end_idx = j
            break
    if end_idx is None:
        raise RuntimeError(f"could not find end of {name}()")
    return "\n".join(lines[start_idx:end_idx + 1]) + "\n"


# Mock helpers for sourced libraries.
_STUB_LIB = r"""
# Stub for hermes_github.sh whoami_add_label (we don't want real GitHub writes).
whoami_add_label() {
    echo "[whoami_add_label] pr=$1 label=$2 reason=$3" >&2
    return 0
}

# Stub for lib_user_unlabel_check.sh — default: user did NOT remove the label.
user_removed_label_recently() { return 1; }
user_unlabel_log_skip() { :; }
user_unlabel_should_notify() { return 1; }
user_unlabel_mark_notified() { return 0; }

# Stub log function (real one writes to stderr with timestamp).
log() { printf '[log] %s\n' "$*" >&2; }
"""


def _run_reconcile(
    *,
    bin_dir: Path,
    pr_fixture: Path,
    user_unlabel_removed: bool = False,
) -> subprocess.CompletedProcess[str]:
    """Run needs_review_conflict_reconcile_all() against a PR fixture.

    Args:
        bin_dir: temp directory containing the `gh` shim (must be on PATH).
        pr_fixture: JSON file with PR list (label-filtered to needs-review).
        user_unlabel_removed: if True, mock user_removed_label_recently to
            return 0 (true) instead of 1 (false). Tests the user-unlabel guard.
    """
    fn_text = _extract_function("needs_review_conflict_reconcile_all")
    wrapper = bin_dir / "_funcs.sh"
    user_unlabel_override = ""
    if user_unlabel_removed:
        user_unlabel_override = "user_removed_label_recently() { return 0; }\n"
    wrapper.write_text(_STUB_LIB + "\n" + user_unlabel_override + fn_text)
    wrapper.chmod(0o755)

    env = os.environ.copy()
    env["PATH"] = f"{bin_dir}{os.pathsep}{env['PATH']}"
    env["GH_REPO"] = "krikz/test-repo"
    env["FIXTURE_FILE"] = str(pr_fixture)
    env["CALL_LOG_FILE"] = str(bin_dir / "gh_calls.log")
    # Required env vars for the function (mirrors merge-gate.sh defensive defaults).
    env["NEEDS_REVIEW_LABEL"] = "needs-review"
    env["MERGE_CONFLICT_LABEL"] = "merge-conflict"
    env["NEEDS_REVIEW_CONFLICT_DEDUP_HOURS"] = "24"
    env["KANBAN_BOARD"] = "robbox"
    env["DEVELOP_BRANCH"] = "develop"

    bash_cmd = (
        f"source {shlex.quote(str(wrapper))}; "
        f"needs_review_conflict_reconcile_all"
    )
    return subprocess.run(
        ["bash", "-c", bash_cmd],
        capture_output=True,
        text=True,
        env=env,
        timeout=30,
    )


def _calls_for(call_log: Path, cmd: str, subcmd: str) -> list[str]:
    """Filter gh-call log for a specific cmd+subcmd combination."""
    if not call_log.exists():
        return []
    out = []
    for line in call_log.read_text().splitlines():
        parts = line.split("|", 4)
        if len(parts) >= 4 and parts[1] == cmd and parts[2] == subcmd:
            out.append(parts[3])
    return out


@pytest.fixture()
def shim_dir(tmp_path: Path) -> Path:
    bin_dir = tmp_path / "bin"
    bin_dir.mkdir()
    _make_gh_shim(bin_dir)
    return bin_dir


def _write_pr_fixture(bin_dir: Path, prs: list[dict]) -> Path:
    fx = bin_dir / ".fixture_prs.json"
    fx.write_text(
        # Compact JSON (no whitespace) to match gh output
        # and avoid shell quoting issues in shim.
        __import__("json").dumps(prs)
    )
    return fx


# --------------------------------------------------------------------------- #
# Test cases
# --------------------------------------------------------------------------- #


def test_r1_pr_1863_conflcting_swap_labels(shim_dir: Path) -> None:
    """R1: PR #1863 — needs-review + CONFLICTING + DIRTY.

    Ожидаем:
    - gh pr edit --remove-label needs-review
    - whoami_add_label merge-conflict
    - gh pr comment (rebase-инструкция)
    """
    fx = _write_pr_fixture(shim_dir, [
        {
            "number": 1863,
            "headRefName": "z-backend/t_ad97d944-fix-dramaturgy",
            "mergeable": "CONFLICTING",
            "mergeStateStatus": "DIRTY",
            "labels": [{"name": "needs-review"}],
        }
    ])
    proc = _run_reconcile(bin_dir=shim_dir, pr_fixture=fx)
    assert proc.returncode == 0, f"stderr: {proc.stderr!r}"

    log = shim_dir / "gh_calls.log"
    edits = _calls_for(log, "pr", "edit")
    assert any("--remove-label" in e and "needs-review" in e for e in edits), (
        f"Ожидаем gh pr edit --remove-label needs-review для #1863. "
        f"Got edits: {edits}"
    )
    # whoami_add_label is a bash function (sourced stub), not a gh call.
    assert "whoami_add_label] pr=1863 label=merge-conflict" in proc.stderr, (
        f"whoami_add_label не вызван для #1863/merge-conflict. "
        f"stderr: {proc.stderr!r}"
    )
    comments = _calls_for(log, "pr", "comment")
    assert len(comments) >= 1, "Ожидаем gh pr comment для #1863"
    assert "needs-review + CONFLICTING" in comments[0], (
        f"Коммент должен быть про needs-review+CONFLICTING. Got: {comments[0]!r}"
    )


def test_r2_pr_1867_mergerable_recovery(shim_dir: Path) -> None:
    """R2: PR уже MERGEABLE+CLEAN, висит merge-conflict с прошлого CONFLICTING.

    Ожидаем:
    - gh pr edit --remove-label merge-conflict (cleanup)
    - НЕ трогаем needs-review (он уже там, user-unlabel guard не активен).
    - gh pr comment (recovery «✅ needs-review conflict RECOVERED»).
    """
    fx = _write_pr_fixture(shim_dir, [
        {
            "number": 1867,
            "headRefName": "fix/dj-plan-rewrite",
            "mergeable": "MERGEABLE",
            "mergeStateStatus": "CLEAN",
            "labels": [
                {"name": "needs-review"},
                {"name": "merge-conflict"},
            ],
        }
    ])
    proc = _run_reconcile(bin_dir=shim_dir, pr_fixture=fx)
    assert proc.returncode == 0, f"stderr: {proc.stderr!r}"

    log = shim_dir / "gh_calls.log"
    edits = _calls_for(log, "pr", "edit")
    assert any("--remove-label" in e and "merge-conflict" in e for e in edits), (
        f"Ожидаем gh pr edit --remove-label merge-conflict для #1867. "
        f"Got edits: {edits}"
    )
    # needs-review уже там — НЕ добавляем заново.
    assert not any("--add-label" in e and "needs-review" in e for e in edits), (
        f"needs-review уже был на #1867, не должны add. Got edits: {edits}"
    )
    comments = _calls_for(log, "pr", "comment")
    assert any("RECOVERED" in c for c in comments), (
        f"Ожидаем recovery-коммент для #1867. Got: {comments}"
    )


def test_r3_pr_mergerable_no_conflict_noop(shim_dir: Path) -> None:
    """R3: PR MERGEABLE+CLEAN, без merge-conflict — нормальный review state.

    Ожидаем: НИЧЕГО не делаем (нет ни remove-label, ни add-label, ни comment).
    """
    fx = _write_pr_fixture(shim_dir, [
        {
            "number": 9999,
            "headRefName": "feature/some-fix",
            "mergeable": "MERGEABLE",
            "mergeStateStatus": "CLEAN",
            "labels": [{"name": "needs-review"}],
        }
    ])
    proc = _run_reconcile(bin_dir=shim_dir, pr_fixture=fx)
    assert proc.returncode == 0, f"stderr: {proc.stderr!r}"

    log = shim_dir / "gh_calls.log"
    edits = _calls_for(log, "pr", "edit")
    comments = _calls_for(log, "pr", "comment")
    assert len(edits) == 0, f"Не должно быть gh pr edit. Got: {edits}"
    assert len(comments) == 0, f"Не должно быть gh pr comment. Got: {comments}"


def test_r4_pr_unknown_skip(shim_dir: Path) -> None:
    """R4: PR MERGEABLE=UNKNOWN (CI calc in progress).

    Ожидаем: skip (не трогаем ни labels, ни comment) — UNKNOWN может
    быть временным состоянием, false positive дороже чем пропуск.
    """
    fx = _write_pr_fixture(shim_dir, [
        {
            "number": 8888,
            "headRefName": "wip/in-progress",
            "mergeable": "UNKNOWN",
            "mergeStateStatus": "BLOCKED",
            "labels": [{"name": "needs-review"}],
        }
    ])
    proc = _run_reconcile(bin_dir=shim_dir, pr_fixture=fx)
    assert proc.returncode == 0, f"stderr: {proc.stderr!r}"

    log = shim_dir / "gh_calls.log"
    edits = _calls_for(log, "pr", "edit")
    comments = _calls_for(log, "pr", "comment")
    assert len(edits) == 0, (
        f"UNKNOWN → skip, никаких edits. Got: {edits}"
    )
    assert len(comments) == 0, (
        f"UNKNOWN → skip, никаких comments. Got: {comments}"
    )


def test_r5_user_unlabel_guard(shim_dir: Path) -> None:
    """R5: PR CONFLICTING + needs-review, НО Шифу руками снял needs-review.

    Ожидаем: НЕ ставим merge-conflict (он сам решил проблему). user-unlabel
    guard защищает от автоматического re-add после ручного решения.
    """
    fx = _write_pr_fixture(shim_dir, [
        {
            "number": 7777,
            "headRefName": "feature/whatever",
            "mergeable": "CONFLICTING",
            "mergeStateStatus": "DIRTY",
            # needs-review СНЯТ Шифу (его нет в labels), но по guard мы
            # должны check user_removed_label_recently и skip.
            "labels": [],
        }
    ])
    proc = _run_reconcile(
        bin_dir=shim_dir, pr_fixture=fx, user_unlabel_removed=True
    )
    assert proc.returncode == 0, f"stderr: {proc.stderr!r}"

    # whoami_add_label не должен быть вызван (мы skip).
    assert "whoami_add_label] pr=7777" not in proc.stderr, (
        f"user-unlabel guard должен skip — whoami_add_label не должен "
        f"сработать для #7777. stderr: {proc.stderr!r}"
    )
    log = shim_dir / "gh_calls.log"
    comments = _calls_for(log, "pr", "comment")
    assert not any("needs-review + CONFLICTING" in c for c in comments), (
        f"Не должно быть CONFLICTING-коммента при user-unlabel guard. "
        f"Got: {comments}"
    )


def test_r6_mixed_batch(shim_dir: Path) -> None:
    """R6: Смешанный batch — несколько PR в одной выборке.

    Подтверждаем что function обрабатывает каждый PR независимо.
    PR A (CONFLICTING): swap. PR B (CLEAN): no-op. PR C (CLEAN + conflict): recovery.
    """
    fx = _write_pr_fixture(shim_dir, [
        # A: CONFLICTING + needs-review → swap
        {
            "number": 1001,
            "headRefName": "z-agent/1001-broken",
            "mergeable": "CONFLICTING",
            "mergeStateStatus": "DIRTY",
            "labels": [{"name": "needs-review"}],
        },
        # B: CLEAN + needs-review, без merge-conflict → no-op
        {
            "number": 1002,
            "headRefName": "z-agent/1002-clean",
            "mergeable": "MERGEABLE",
            "mergeStateStatus": "CLEAN",
            "labels": [{"name": "needs-review"}],
        },
        # C: CLEAN + needs-review + merge-conflict (recovery)
        {
            "number": 1003,
            "headRefName": "z-agent/1003-recovered",
            "mergeable": "MERGEABLE",
            "mergeStateStatus": "CLEAN",
            "labels": [
                {"name": "needs-review"},
                {"name": "merge-conflict"},
            ],
        },
    ])
    proc = _run_reconcile(bin_dir=shim_dir, pr_fixture=fx)
    assert proc.returncode == 0, f"stderr: {proc.stderr!r}"

    # A: remove needs-review + whoami merge-conflict
    assert "whoami_add_label] pr=1001 label=merge-conflict" in proc.stderr
    log = shim_dir / "gh_calls.log"
    edits = _calls_for(log, "pr", "edit")
    assert any("1001" in e and "--remove-label" in e and "needs-review" in e
               for e in edits)
    # B: ничего
    assert not any("1002" in e for e in edits)
    assert not any("1002" in c for c in _calls_for(log, "pr", "comment"))
    # C: remove merge-conflict + recovery comment
    assert any("1003" in e and "--remove-label" in e and "merge-conflict" in e
               for e in edits)
    c1003_comments = [c for c in _calls_for(log, "pr", "comment") if "1003" in c]
    assert any("RECOVERED" in c for c in c1003_comments), (
        f"Recovery-коммент для #1003 не найден. Got: {c1003_comments}"
    )
