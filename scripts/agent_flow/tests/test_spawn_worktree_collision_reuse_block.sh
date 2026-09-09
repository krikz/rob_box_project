#!/bin/bash
# ============================================================================
# test_spawn_worktree_collision_reuse_block.sh — pre-spawn worktree guard
#
# Ретро t_695f5138 / task t_bc3ac7fb: при повторном spawn карточки с
# workspace_kind=worktree, если её branch_name уже чекаутнута в чужом
# worktree, ``git worktree add`` падает с fatal "already checked out".
# До фикса это вызывало spawn_failed × 2 → gave_up → blocked, и пока
# карточка висела в blocked, triage успевал наплодить ещё несколько
# дублей на ту же ветку.
#
# Этот тест проверяет функциональный фикс:
#
#   spawn_worktree_collision_reuse_idle
#       _ensure_git_worktree вызывается с target, который уже
#       существует и совпадает с чекаутом branch. Не должно быть
#       повторного git worktree add (early return по
#       target_exists+common_dir). Acceptance #1.
#
#   spawn_worktree_collision_block_busy
#       _ensure_git_worktree вызывается с target, который НЕ
#       совпадает с уже-чекаутнутым branch. Должно быть поднято
#       WorktreeBranchBusyError с маркером WORKTREE_BRANCH_BUSY и
#       путями existing и target в сообщении. Acceptance #2.
#
#   spawn_worktree_collision_no_branch
#       Если branch вообще не существует как ref, helper
#       _find_worktree_for_branch возвращает None и
#       _ensure_git_worktree нормально создаёт новый worktree.
#       Backward-compat (acceptance #3).
#
# Применяет vendor-патч hermes-agent-spawn-worktree-precheck.patch
# к чистому origin/main hermes-agent в worktree, и гоняет
# функциональные сценарии через python3 -c.
#
# Invocation:
#   bash tests/test_spawn_worktree_collision_reuse_block.sh
# Returns 0 on all-pass, non-zero on first failure.
# ============================================================================
set -euo pipefail

TEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$TEST_DIR/.." && pwd)"
PATCH="$REPO_ROOT/vendor/hermes-agent-spawn-worktree-precheck.patch"
HA_SRC="${HERMES_AGENT_SRC:-/home/builder/.hermes/hermes-agent}"
WORKTREE="$(mktemp -d /tmp/ha-spawn-precheck.XXXXXX)"

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

# Apply the patch — but DON'T commit. We want a clean working tree after
# the test so subsequent test runs don't see stale state.
( cd "$WORKTREE" && git apply --check "$PATCH" ) \
    || fail "patch does not apply cleanly to origin/main"
( cd "$WORKTREE" && git apply "$PATCH" ) \
    || fail "git apply failed"
pass "patch applies cleanly to origin/main"

# Idempotency: re-applying (reverse-check) must be clean after apply.
( cd "$WORKTREE" && git apply --reverse --check "$PATCH" ) \
    || fail "patch not idempotent"
pass "patch idempotent (reverse-check clean)"

# Symbols anchored: confirm the two new public symbols exist in the
# patched file.
grep -q "^def _find_worktree_for_branch" "$WORKTREE/hermes_cli/kanban_db.py" \
    || fail "helper _find_worktree_for_branch not found in patched kanban_db.py"
pass "helper _find_worktree_for_branch present"

grep -q "^class WorktreeBranchBusyError" "$WORKTREE/hermes_cli/kanban_db.py" \
    || fail "class WorktreeBranchBusyError not found in patched kanban_db.py"
pass "class WorktreeBranchBusyError present"

# Functional scenarios — drive the patched module via python3.
export WORKTREE
WORKTREE="$WORKTREE" PYTHONPATH="$WORKTREE" python3 - <<'PY' || fail "functional scenario failed"
import os, subprocess, sys, tempfile
from pathlib import Path

# WORKTREE is the patched hermes-agent source tree (env from bash).
worktree = os.environ['WORKTREE']
sys.path.insert(0, worktree)

from hermes_cli.kanban_db import (
    _ensure_git_worktree,
    _find_worktree_for_branch,
    WorktreeBranchBusyError,
)


def make_repo():
    tmp = tempfile.mkdtemp(prefix='wt-precheck-')
    subprocess.run(['git', '-C', tmp, 'init', '-q', '-b', 'main'], check=True)
    subprocess.run(['git', '-C', tmp, 'config', 'user.email', 't@x'], check=True)
    subprocess.run(['git', '-C', tmp, 'config', 'user.name', 'T'], check=True)
    Path(tmp, 'f').write_text('hi')
    subprocess.run(['git', '-C', tmp, 'add', 'f'], check=True)
    subprocess.run(['git', '-C', tmp, 'commit', '-q', '-m', 'init'], check=True)
    return tmp


def cleanup(tmp):
    subprocess.run(['rm', '-rf', tmp])


# ---- Scenario A: spawn_worktree_collision_reuse_idle -----------------
# Existing checkout at target. Calling _ensure_git_worktree again with
# the SAME target must NOT shell out to ``git worktree add`` — the
# existing-target early return + common-dir check catches it.
print('== A: reuse_idle ==')
tmp = make_repo()
try:
    wt_dir = os.path.join(tmp, '.worktrees', 'wt-existing')
    os.makedirs(os.path.dirname(wt_dir), exist_ok=True)
    subprocess.run(
        ['git', '-C', tmp, 'worktree', 'add', '-b', 'reused', wt_dir, 'main'],
        check=True, capture_output=True, text=True,
    )
    # Sanity: helper finds the existing checkout.
    found = _find_worktree_for_branch(Path(tmp), 'reused')
    assert found is not None, 'helper must find the existing checkout'
    assert Path(found).resolve() == Path(wt_dir).resolve(), 'wrong path returned'
    # Call again: must not raise (path-equals-existing catches it
    # via the target.exists()+common_dir check at the top of
    # _ensure_git_worktree).
    _ensure_git_worktree(Path(tmp), Path(wt_dir), 'reused')
    print('  PASS reuse_idle')
finally:
    cleanup(tmp)

# ---- Scenario B: spawn_worktree_collision_block_busy -----------------
# Existing checkout at path A; dispatcher tries to claim the SAME
# branch at a DIFFERENT path B. Pre-fix: git worktree add fatal, retry
# storm. Post-fix: WorktreeBranchBusyError raised immediately, with
# WORKTREE_BRANCH_BUSY marker and both paths in the message.
print('== B: block_busy ==')
tmp = make_repo()
try:
    wt_busy = os.path.join(tmp, '.worktrees', 'wt-busy')
    wt_target = os.path.join(tmp, '.worktrees', 'wt-target')
    os.makedirs(os.path.dirname(wt_busy), exist_ok=True)
    os.makedirs(os.path.dirname(wt_target), exist_ok=True)
    subprocess.run(
        ['git', '-C', tmp, 'worktree', 'add', '-b', 'busy', wt_busy, 'main'],
        check=True, capture_output=True, text=True,
    )
    # Now the SAME branch is checked out at wt_busy. Dispatcher tries
    # to spawn a card whose workspace_path is wt_target.
    raised = False
    try:
        _ensure_git_worktree(Path(tmp), Path(wt_target), 'busy')
    except WorktreeBranchBusyError as exc:
        raised = True
        msg = str(exc)
        assert 'WORKTREE_BRANCH_BUSY' in msg, f'missing marker: {msg}'
        assert wt_busy in msg, f'busy path missing from message: {msg}'
        assert wt_target in msg, f'target path missing from message: {msg}'
        # The exception MUST be a RuntimeError subclass — the dispatch
        # loop catches ``Exception`` but the new branch catches the
        # specific subclass for force_trip=True routing.
        assert isinstance(exc, RuntimeError), 'must subclass RuntimeError'
    assert raised, 'WorktreeBranchBusyError not raised'
    # No extra worktree was created (we never reached the git shell-out).
    porcelain = subprocess.run(
        ['git', '-C', tmp, 'worktree', 'list', '--porcelain'],
        capture_output=True, text=True, check=True,
    )
    assert porcelain.stdout.count('worktree ') == 2, (
        f'expected 2 worktrees (main + busy), got {porcelain.stdout}'
    )
    print('  PASS block_busy')
finally:
    cleanup(tmp)

# ---- Scenario C: spawn_worktree_collision_no_branch ------------------
# Backward-compat: if the branch does NOT exist anywhere, the helper
# returns None and _ensure_git_worktree proceeds to create a fresh
# worktree exactly as before the patch. Acceptance #3.
print('== C: no_branch (backward-compat) ==')
tmp = make_repo()
try:
    # Create the branch as a ref, but DO NOT check it out anywhere.
    subprocess.run(
        ['git', '-C', tmp, 'branch', 'fresh-branch', 'main'],
        check=True, capture_output=True, text=True,
    )
    found = _find_worktree_for_branch(Path(tmp), 'fresh-branch')
    assert found is None, (
        f'helper must return None for un-checked-out branch, got {found}'
    )
    # _ensure_git_worktree should now create a new worktree at target.
    wt_new = os.path.join(tmp, '.worktrees', 'wt-new')
    os.makedirs(os.path.dirname(wt_new), exist_ok=True)
    _ensure_git_worktree(Path(tmp), Path(wt_new), 'fresh-branch')
    porcelain = subprocess.run(
        ['git', '-C', tmp, 'worktree', 'list', '--porcelain'],
        capture_output=True, text=True, check=True,
    )
    # main + wt-new = 2 worktrees. The branch existed as a ref before
    # the call; after the call it exists as a checked-out worktree at
    # wt-new. The branch-as-ref row in ``git worktree list`` was never
    # there to begin with — refs without a worktree don't show up.
    assert porcelain.stdout.count('worktree ') == 2, (
        f'expected 2 worktrees (main + wt-new), got {porcelain.stdout}'
    )
    # And the new worktree is the one that checks out fresh-branch.
    found = _find_worktree_for_branch(Path(tmp), 'fresh-branch')
    assert found is not None
    assert Path(found).resolve() == Path(wt_new).resolve(), (
        f'helper should locate the new checkout at {wt_new}, got {found}'
    )
    print('  PASS no_branch')
finally:
    cleanup(tmp)

# ---- Scenario D: dispatch_once wires WorktreeBranchBusyError to ------
# force_trip=True (verifies the second hunk of the patch is wired).
print('== D: dispatch_once catches WorktreeBranchBusyError ==')
# Read the patched source directly — inspect.getsource() requires the
# module to be importable under a name that survives the sys.path
# rewrite we did at the top, which is fragile. The on-disk file is
# the canonical artifact.
src = open(os.path.join(worktree, 'hermes_cli', 'kanban_db.py')).read()
assert 'WorktreeBranchBusyError' in src, (
    'patched kanban_db.py lost WorktreeBranchBusyError'
)
# The exact branch we're verifying: ``except WorktreeBranchBusyError
# as exc`` followed by ``force_trip=True``. Count occurrences to be
# precise — there should be one per ``try`` block in dispatch_once
# (ready loop + review loop).
import re
handles = re.findall(
    r'except WorktreeBranchBusyError as exc:.*?force_trip=True',
    src,
    re.DOTALL,
)
assert len(handles) >= 2, (
    f'expected >=2 force_trip=True handlers for WorktreeBranchBusyError '
    f'(ready + review dispatch), got {len(handles)}'
)
print(f'  PASS dispatch_once_wired ({len(handles)} handler sites)')
PY

# Final cleanup: revert the patch so subsequent runs start clean.
( cd "$WORKTREE" && git apply --reverse "$PATCH" ) \
    || fail "could not reverse patch after run"
pass "patch cleanly reverted"

echo "ALL SPAWN-WORKTREE-PRECHECK TESTS PASSED"
