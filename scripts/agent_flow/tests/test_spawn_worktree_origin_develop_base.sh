#!/bin/bash
# ============================================================================
# test_spawn_worktree_origin_develop_base.sh — base ref for new worktree
#
# Issue #1571: _ensure_git_worktree создавал новые worktree-ветки от
# локального HEAD главного worktree, который мог отставать от origin/develop
# на 48-50+ коммитов. Симптом — drift в каждом новом PR.
#
# Этот тест проверяет фикс:
#
#   spawn_worktree_uses_origin_develop_when_fresh
#       _ensure_git_worktree при создании НОВОЙ ветки делает
#       ``git fetch origin develop`` и использует ``origin/develop``
#       как base. После этого ветка обязана указывать на тот же commit,
#       что и origin/develop (а не на локальный HEAD).
#       Acceptance #1+#2.
#
#   spawn_worktree_warns_on_drift_above_threshold
#       Если HEAD отстаёт от origin/develop на >10 коммитов,
#       в stderr должен уйти WORKTREE_BASE_DRIFT marker с числом
#       коммитов. Acceptance #4 (warning часть).
#
#   spawn_worktree_fetch_fail_falls_back_to_head
#       Если ``git fetch origin develop`` падает (нет remote),
#       _resolve_worktree_base_ref должен вернуть "HEAD" и напечатать
#       WORKTREE_BASE_FETCH_FAILED в stderr. Acceptance #1 (graceful
#       degradation).
#
#   spawn_worktree_branch_exists_uses_existing
#       Когда branch_name уже существует, _ensure_git_worktree НЕ
#       должен вызывать fetch — base уже зафиксирован, fetch не нужен.
#       Backward-compat.
#
# Применяет vendor-патч hermes-agent-spawn-base-origin-develop.patch
# к чистому origin/main hermes-agent в worktree, и гоняет
# функциональные сценарии через python3 -c.
#
# Invocation:
#   bash tests/test_spawn_worktree_origin_develop_base.sh
# Returns 0 on all-pass, non-zero on first failure.
# ============================================================================
set -euo pipefail

TEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$TEST_DIR/.." && pwd)"
PATCH="$REPO_ROOT/vendor/hermes-agent-z-spawn-base-origin-develop.patch"
HA_SRC="${HERMES_AGENT_SRC:-/home/builder/.hermes/hermes-agent}"
WORKTREE="$(mktemp -d /tmp/ha-origin-base.XXXXXX)"

cleanup() {
    git -C "$HA_SRC" worktree remove --force "$WORKTREE" >/dev/null 2>&1 || rm -rf "$WORKTREE"
}
trap cleanup EXIT

fail() { echo "FAIL: $*" >&2; exit 1; }
pass() { echo "  ok: $*"; }

[ -s "$PATCH" ] || fail "vendor patch missing or empty: $PATCH"
pass "vendor patch present ($(wc -l < "$PATCH") lines)"

git -C "$HA_SRC" fetch origin main --quiet 2>/dev/null || fail "cannot fetch origin/main"
git -C "$HA_SRC" worktree add --detach "$WORKTREE" origin/main >/dev/null 2>&1 \
    || fail "cannot create worktree at origin/main"

# Apply precheck patch FIRST (it defines WorktreeBranchBusyError which
# this patch's anchor lives near — z-prefixed patch must apply second).
PRECHECK_PATCH="$REPO_ROOT/vendor/hermes-agent-spawn-worktree-precheck.patch"
[ -s "$PRECHECK_PATCH" ] || fail "precheck patch missing: $PRECHECK_PATCH"
( cd "$WORKTREE" && git apply --check "$PRECHECK_PATCH" ) \
    || fail "precheck patch does not apply cleanly to origin/main"
( cd "$WORKTREE" && git apply "$PRECHECK_PATCH" ) \
    || fail "precheck git apply failed"
pass "precheck patch applies cleanly to origin/main"

# Apply the patch under test — but DON'T commit. We want a clean
# working tree after the test so subsequent test runs don't see
# stale state.
( cd "$WORKTREE" && git apply --check "$PATCH" ) \
    || fail "patch does not apply cleanly to origin/main+precheck"
( cd "$WORKTREE" && git apply "$PATCH" ) \
    || fail "git apply failed"
pass "patch applies cleanly to origin/main+precheck"

# Idempotency: re-applying (reverse-check) must be clean after apply.
( cd "$WORKTREE" && git apply --reverse --check "$PATCH" ) \
    || fail "patch not idempotent"
pass "patch idempotent (reverse-check clean)"

# Symbols anchored: confirm the new public symbols exist in the
# patched file.
grep -q "^def _resolve_worktree_base_ref" "$WORKTREE/hermes_cli/kanban_db.py" \
    || fail "helper _resolve_worktree_base_ref not found in patched kanban_db.py"
pass "helper _resolve_worktree_base_ref present"

grep -q "^def _warn_worktree_base_drift" "$WORKTREE/hermes_cli/kanban_db.py" \
    || fail "helper _warn_worktree_base_drift not found in patched kanban_db.py"
pass "helper _warn_worktree_base_drift present"

# Functional scenarios — drive the patched module via python3.
export WORKTREE
WORKTREE="$WORKTREE" PYTHONPATH="$WORKTREE" python3 - <<'PY' || fail "functional scenario failed"
import os, subprocess, sys, tempfile, io, contextlib
from pathlib import Path

# WORKTREE is the patched hermes-agent source tree (env from bash).
worktree = os.environ['WORKTREE']
sys.path.insert(0, worktree)

from hermes_cli.kanban_db import (
    _ensure_git_worktree,
    _resolve_worktree_base_ref,
    _warn_worktree_base_drift,
)


def make_repo_with_origin():
    """Create a bare 'origin' + a working clone with develop branch.

    Returns (working_repo, bare_origin). The working repo has 'origin'
    pointing at the bare, on the 'develop' branch, with N commits
    ahead of HEAD (configurable per scenario).
    """
    base = tempfile.mkdtemp(prefix='wt-base-')
    bare = os.path.join(base, 'origin.git')
    working = os.path.join(base, 'work')
    # bare repo with develop branch
    subprocess.run(['git', 'init', '--bare', '-q', '-b', 'develop', bare], check=True)
    subprocess.run(['git', 'clone', '-q', bare, working], check=True)
    for cmd in [
        ['config', 'user.email', 't@x'],
        ['config', 'user.name', 'T'],
        ['checkout', '-q', '-b', 'develop'],
    ]:
        subprocess.run(['git', '-C', working] + cmd, check=True)
    Path(working, 'f').write_text('hi')
    subprocess.run(['git', '-C', working, 'add', 'f'], check=True)
    subprocess.run(['git', '-C', working, 'commit', '-q', '-m', 'init'], check=True)
    subprocess.run(['git', '-C', working, 'push', '-q', '-u', 'origin', 'develop'], check=True)
    return working, bare, base


def cleanup(*paths):
    for p in paths:
        subprocess.run(['rm', '-rf', p])


def make_extra_commits(repo, n):
    """Add ``n`` commits ahead of current HEAD on develop branch."""
    for i in range(n):
        Path(repo, f'f{i}').write_text(f'data-{i}')
        subprocess.run(['git', '-C', repo, 'add', f'f{i}'], check=True)
        subprocess.run(['git', '-C', repo, 'commit', '-q', '-m', f'c{i}'], check=True)
    subprocess.run(['git', '-C', repo, 'push', '-q', 'origin', 'develop'], check=True)


def capture_stderr(callable_):
    """Run ``callable_`` and capture its stderr output and return value.

    Returns ``(stderr_text, return_value)`` so callers can both assert
    on warning markers AND verify the function's actual return value
    — otherwise ``print(..., file=sys.stderr)`` clobbers what should
    have been the return value.
    """
    buf = io.StringIO()
    with contextlib.redirect_stderr(buf):
        ret = callable_()
    return buf.getvalue(), ret


def branch_sha(repo, branch):
    r = subprocess.run(
        ['git', '-C', repo, 'rev-parse', branch],
        capture_output=True, text=True, check=True,
    )
    return r.stdout.strip()


def list_worktree_count(repo):
    r = subprocess.run(
        ['git', '-C', repo, 'worktree', 'list', '--porcelain'],
        capture_output=True, text=True, check=True,
    )
    return r.stdout.count('worktree ')


# ---- Scenario A: fresh origin/develop as base ----------------------------
# Setup: 5 commits on origin/develop. Working repo HEAD = same as origin.
# Calling _ensure_git_worktree with NEW branch name must:
#   1) Fetch origin develop (so origin/develop stays current).
#   2) Use origin/develop as the base for the new branch.
# After call: new branch sha == origin/develop sha (== working HEAD here).
print('== A: uses_origin_develop_when_fresh ==')
working, bare, base = make_repo_with_origin()
try:
    wt_new = os.path.join(working, '.worktrees', 'wt-new')
    os.makedirs(os.path.dirname(wt_new), exist_ok=True)
    _ensure_git_worktree(Path(working), Path(wt_new), 'feature-x')
    # The new branch's HEAD must point at origin/develop (== working HEAD).
    new_sha = branch_sha(working, 'feature-x')
    od_sha = branch_sha(working, 'origin/develop')
    head_sha = branch_sha(working, 'HEAD')
    assert new_sha == od_sha == head_sha, (
        f'feature-x ({new_sha}) must equal origin/develop ({od_sha}) '
        f'and HEAD ({head_sha})'
    )
    # The new worktree path checks out the new branch.
    wt_branch = subprocess.run(
        ['git', '-C', wt_new, 'rev-parse', '--abbrev-ref', 'HEAD'],
        capture_output=True, text=True, check=True,
    ).stdout.strip()
    assert wt_branch == 'feature-x', f'wt must checkout feature-x, got {wt_branch}'
    assert list_worktree_count(working) == 2, (
        f'expected 2 worktrees (main + wt-new), got {list_worktree_count(working)}'
    )
    print(f'  PASS uses_origin_develop_when_fresh (sha={new_sha[:7]})')
finally:
    cleanup(base)


# ---- Scenario B: drift warning when HEAD lags origin/develop > threshold --
# Setup: working repo HEAD = 15 commits behind origin/develop
# (after fetch). Calling _ensure_git_worktree with NEW branch name
# must emit WORKTREE_BASE_DRIFT warning to stderr.
print('== B: warns_on_drift_above_threshold ==')
working, bare, base = make_repo_with_origin()
try:
    make_extra_commits(working, 15)
    # Reset local develop back to origin's parent so HEAD is now 15 behind.
    od_parent = subprocess.run(
        ['git', '-C', working, 'rev-parse', 'origin/develop~15'],
        capture_output=True, text=True, check=True,
    ).stdout.strip()
    subprocess.run(['git', '-C', working, 'reset', '--hard', '-q', od_parent], check=True)
    # Sanity: HEAD..origin/develop = 15 commits.
    sanity = subprocess.run(
        ['git', '-C', working, 'rev-list', '--count', 'HEAD..origin/develop'],
        capture_output=True, text=True, check=True,
    ).stdout.strip()
    assert sanity == '15', f'expected HEAD..origin/develop=15, got {sanity}'
    wt_new = os.path.join(working, '.worktrees', 'wt-drift')
    os.makedirs(os.path.dirname(wt_new), exist_ok=True)
    err, _ = capture_stderr(lambda: _ensure_git_worktree(
        Path(working), Path(wt_new), 'drifted-branch',
    ))
    assert 'WORKTREE_BASE_DRIFT' in err, (
        f'expected WORKTREE_BASE_DRIFT in stderr, got: {err!r}'
    )
    assert '15' in err, f'expected drift count "15" in stderr, got: {err!r}'
    assert 'origin/develop' in err, (
        f'expected origin/develop in stderr, got: {err!r}'
    )
    # The new branch was still created (from origin/develop despite drift).
    new_sha = branch_sha(working, 'drifted-branch')
    od_sha = branch_sha(working, 'origin/develop')
    assert new_sha == od_sha, (
        f'drifted-branch ({new_sha}) must equal origin/develop ({od_sha}) '
        f'even when HEAD is behind'
    )
    print(f'  PASS warns_on_drift_above_threshold (15 commits behind)')
finally:
    cleanup(base)


# ---- Scenario C: fetch failure falls back to HEAD with warning -----------
# Setup: repo without 'origin' remote so fetch fails. _resolve_worktree_base_ref
# must return "HEAD" and print WORKTREE_BASE_FETCH_FAILED to stderr.
print('== C: fetch_fail_falls_back_to_head ==')
tmp = tempfile.mkdtemp(prefix='wt-nofetch-')
try:
    subprocess.run(['git', '-C', tmp, 'init', '-q', '-b', 'main'], check=True)
    for cmd in [['config', 'user.email', 't@x'], ['config', 'user.name', 'T']]:
        subprocess.run(['git', '-C', tmp] + cmd, check=True)
    Path(tmp, 'f').write_text('hi')
    subprocess.run(['git', '-C', tmp, 'add', 'f'], check=True)
    subprocess.run(['git', '-C', tmp, 'commit', '-q', '-m', 'init'], check=True)
    # No 'origin' remote — fetch must fail.
    err_text, ref = capture_stderr(lambda: _resolve_worktree_base_ref(Path(tmp)))
    assert ref == 'HEAD', f'expected HEAD fallback, got {ref!r}'
    err_text, ref2 = capture_stderr(lambda: _resolve_worktree_base_ref(Path(tmp)))
    assert ref2 == 'HEAD', f'expected HEAD fallback (2nd call), got {ref2!r}'
    assert 'WORKTREE_BASE_FETCH_FAILED' in err_text, (
        f'expected WORKTREE_BASE_FETCH_FAILED in stderr, got: {err_text!r}'
    )
    print('  PASS fetch_fail_falls_back_to_head')
finally:
    subprocess.run(['rm', '-rf', tmp])


# ---- Scenario D: existing branch checkout — no fetch, no base change ----
# Setup: branch 'existing' is already on origin. Calling
# _ensure_git_worktree for 'existing' (branch DOES exist as ref) must NOT
# call fetch — the existing branch is checked out as-is. Backward-compat.
print('== D: existing_branch_no_fetch ==')
working, bare, base = make_repo_with_origin()
try:
    # Pre-create the branch as a ref (not checked out anywhere).
    subprocess.run(
        ['git', '-C', working, 'branch', 'pre-existing', 'develop'],
        check=True, capture_output=True, text=True,
    )
    wt_target = os.path.join(working, '.worktrees', 'wt-existing')
    os.makedirs(os.path.dirname(wt_target), exist_ok=True)
    err, _ = capture_stderr(lambda: _ensure_git_worktree(
        Path(working), Path(wt_target), 'pre-existing',
    ))
    # No drift warning when checking out an existing branch (base already
    # fixed). Fetch is also skipped.
    assert 'WORKTREE_BASE_DRIFT' not in err, (
        f'existing-branch checkout must NOT emit drift warning, got: {err!r}'
    )
    assert 'WORKTREE_BASE_FETCH_FAILED' not in err, (
        f'existing-branch checkout must NOT emit fetch warning, got: {err!r}'
    )
    wt_branch = subprocess.run(
        ['git', '-C', wt_target, 'rev-parse', '--abbrev-ref', 'HEAD'],
        capture_output=True, text=True, check=True,
    ).stdout.strip()
    assert wt_branch == 'pre-existing', f'got {wt_branch!r}'
    print('  PASS existing_branch_no_fetch')
finally:
    cleanup(base)


# ---- Scenario E: dispatch log includes warning when _ensure_git_worktree -
# is wired from a hypothetical drift-15 scenario (sanity: the warning text
# is grep-friendly so merge-gate / completion-check can surface it).
print('== E: warning_format_is_grep_friendly ==')
working, bare, base = make_repo_with_origin()
try:
    make_extra_commits(working, 12)
    od_parent = subprocess.run(
        ['git', '-C', working, 'rev-parse', 'origin/develop~12'],
        capture_output=True, text=True, check=True,
    ).stdout.strip()
    subprocess.run(['git', '-C', working, 'reset', '--hard', '-q', od_parent], check=True)
    wt_new = os.path.join(working, '.worktrees', 'wt-fmt')
    os.makedirs(os.path.dirname(wt_new), exist_ok=True)
    err, _ = capture_stderr(lambda: _ensure_git_worktree(
        Path(working), Path(wt_new), 'format-branch',
    ))
    # Marker format: WORKTREE_BASE_DRIFT: HEAD..origin/develop = N commits ...
    # Must be a single line so grep -F "WORKTREE_BASE_DRIFT" matches.
    matching = [ln for ln in err.splitlines() if 'WORKTREE_BASE_DRIFT' in ln]
    assert len(matching) == 1, (
        f'expected exactly one WORKTREE_BASE_DRIFT line, got {len(matching)}: '
        f'{err!r}'
    )
    line = matching[0]
    # Format must include numeric count and reference name.
    assert '12 commits' in line, f'expected "12 commits" in: {line!r}'
    assert 'origin/develop' in line, f'expected "origin/develop" in: {line!r}'
    assert 'threshold 10' in line, f'expected "threshold 10" in: {line!r}'
    # Issue #1571 reference for triage.
    assert 'issue #1571' in line or '#1571' in line, (
        f'expected issue ref in: {line!r}'
    )
    print(f'  PASS warning_format_is_grep_friendly')
finally:
    cleanup(base)


print()
print('ALL ORIGIN-DEVELOP-BASE WORKTREE TESTS PASSED')
PY

# Final cleanup: revert the patches in REVERSE order so the worktree
# returns to origin/main cleanly.
( cd "$WORKTREE" && git apply --reverse "$PATCH" ) \
    || fail "could not reverse z-patch after run"
( cd "$WORKTREE" && git apply --reverse "$PRECHECK_PATCH" ) \
    || fail "could not reverse precheck patch after run"
pass "patches cleanly reverted"

echo "ALL ORIGIN-DEVELOP-BASE WORKTREE TESTS PASSED"