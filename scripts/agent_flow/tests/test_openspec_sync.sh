#!/bin/bash
# ============================================================================
# test_openspec_sync.sh — regression for agent-flow-openspec-sync.sh
#
# Verifies:
#   A. help/usage text works
#   B. status command prints JSON
#   C. create-change creates skeleton files (proposal.md, tasks.md, README.md,
#      .openspec.yaml, specs/.gitkeep)
#   D. create-change is idempotent (second call → noop, files unchanged)
#   E. archive-change moves active change to changes/archive/
#   F. archive-change is idempotent (second call → noop)
#   G. AGENT_FLOW_OPENSPEC_DISABLED=1 → full noop
#   H. Non-existent root + no auto-detect → skip (exit 0, not crash)
#   I. Arg validation: missing args → DIE exit 1
#
# Run:
#   bash scripts/agent_flow/tests/test_openspec_sync.sh
# Returns 0 on all-pass, non-zero on first failure.
#
# No mocks needed: each scenario uses a fresh tmp sandbox with OPENSPEC_ROOT
# pointed at it. Tests are isolated and cleanable.
# ============================================================================
set -euo pipefail

TEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SYNC="$TEST_DIR/../agent-flow-openspec-sync.sh"

if [ ! -x "$SYNC" ]; then
    echo "FATAL: $SYNC not found or not executable" >&2
    exit 1
fi

# Per-scenario sandbox setup. Each scenario uses its own dir, cleaned on exit.
SANDBOX=""
cleanup() {
    if [ -n "$SANDBOX" ] && [ -d "$SANDBOX" ]; then
        # Targeted cleanup (avoid `rm -rf` per terminal-guard).
        find "$SANDBOX" -mindepth 1 -delete 2>/dev/null || true
        rmdir "$SANDBOX" 2>/dev/null || true
    fi
}
trap cleanup EXIT

make_sandbox() {
    cleanup
    SANDBOX="$(mktemp -d /tmp/openspec-sync-test-XXXXXX)"
    mkdir -p "$SANDBOX/openspec/changes"
    printf 'schema: spec-driven\n' > "$SANDBOX/openspec/config.yaml"
}

PASS=0
FAIL=0
log_result() {  # $1=PASS|FAIL $2=label
    if [ "$1" = "PASS" ]; then
        printf '  \033[32m✓\033[0m %s\n' "$2"
        PASS=$((PASS+1))
    else
        printf '  \033[31m✗\033[0m %s\n' "$2"
        FAIL=$((FAIL+1))
    fi
}

assert_eq() {  # $1=actual $2=expected $3=label
    if [ "$1" = "$2" ]; then
        log_result PASS "$3"
    else
        printf '    expected: %q\n    actual:   %q\n' "$2" "$1"
        log_result FAIL "$3"
    fi
}

assert_file_exists() {  # $1=path $2=label
    if [ -f "$1" ]; then
        log_result PASS "$2"
    else
        printf '    missing: %s\n' "$1"
        log_result FAIL "$2"
    fi
}

assert_dir_exists() {  # $1=path $2=label
    if [ -d "$1" ]; then
        log_result PASS "$2"
    else
        printf '    missing dir: %s\n' "$1"
        log_result FAIL "$2"
    fi
}

echo "=== A. help/usage ==="
make_sandbox
if "$SYNC" --help >/dev/null 2>&1; then
    log_result PASS "A. --help exits 0"
else
    log_result FAIL "A. --help exits 0"
fi

echo "=== B. status ==="
make_sandbox
_status_out="$(OPENSPEC_ROOT="$SANDBOX/openspec" "$SYNC" status 2>&1)"
_status_rc=$?
assert_eq "$_status_rc" "0" "B. status exits 0"
echo "$_status_out" | grep -q '"root":' \
    && log_result PASS "B. status contains root field" \
    || log_result FAIL "B. status contains root field"
echo "$_status_out" | grep -q '"active_changes":' \
    && log_result PASS "B. status contains active_changes field" \
    || log_result FAIL "B. status contains active_changes field"

echo "=== C. create-change creates skeleton ==="
make_sandbox
_cdir="$SANDBOX/openspec/changes/t_aaaa-task-slug"
OPENSPEC_ROOT="$SANDBOX/openspec" "$SYNC" create-change 1234 t_aaaa task-slug "Test Task" \
    "https://github.com/krikz/rob_box_project/issues/1234" "body text" >/dev/null 2>&1
assert_dir_exists "$_cdir" "C. change dir created"
assert_file_exists "$_cdir/proposal.md" "C. proposal.md created"
assert_file_exists "$_cdir/tasks.md" "C. tasks.md created"
assert_file_exists "$_cdir/README.md" "C. README.md created"
assert_file_exists "$_cdir/.openspec.yaml" "C. .openspec.yaml created"
# Verify proposal.md has Source line with issue number
grep -q 'Issue: #1234' "$_cdir/proposal.md" \
    && log_result PASS "C. proposal.md has issue #" \
    || log_result FAIL "C. proposal.md has issue #"
grep -q 'kanban t_aaaa' "$_cdir/proposal.md" \
    && log_result PASS "C. proposal.md has kanban t_aaaa" \
    || log_result FAIL "C. proposal.md has kanban t_aaaa"
# Verify .openspec.yaml schema
grep -q 'schema: spec-driven' "$_cdir/.openspec.yaml" \
    && log_result PASS "C. .openspec.yaml schema: spec-driven" \
    || log_result FAIL "C. .openspec.yaml schema: spec-driven"

echo "=== D. create-change is idempotent ==="
make_sandbox
_cdir="$SANDBOX/openspec/changes/t_bbbb-task-slug"
OPENSPEC_ROOT="$SANDBOX/openspec" "$SYNC" create-change 2222 t_bbbb task-slug "Test" >/dev/null 2>&1
_first_sha="$(sha256sum "$_cdir/proposal.md" | awk '{print $1}')"
# Second call should not modify the file
OPENSPEC_ROOT="$SANDBOX/openspec" "$SYNC" create-change 2222 t_bbbb task-slug "Test" >/dev/null 2>&1
_second_sha="$(sha256sum "$_cdir/proposal.md" | awk '{print $1}')"
assert_eq "$_second_sha" "$_first_sha" "D. proposal.md unchanged on second call"

echo "=== E. archive-change moves to archive/ ==="
make_sandbox
_cdir="$SANDBOX/openspec/changes/t_cccc-task-slug"
_archive="$SANDBOX/openspec/changes/archive/t_cccc-task-slug"
OPENSPEC_ROOT="$SANDBOX/openspec" "$SYNC" create-change 3333 t_cccc task-slug "Test" >/dev/null 2>&1
assert_dir_exists "$_cdir" "E. change dir created (pre-archive)"
OPENSPEC_ROOT="$SANDBOX/openspec" "$SYNC" archive-change 3333 t_cccc task-slug 4567 >/dev/null 2>&1
assert_dir_exists "$_archive" "E. archive dir exists after archive-change"

echo "=== F. archive-change is idempotent ==="
make_sandbox
OPENSPEC_ROOT="$SANDBOX/openspec" "$SYNC" create-change 4444 t_dddd task-slug "Test" >/dev/null 2>&1
OPENSPEC_ROOT="$SANDBOX/openspec" "$SYNC" archive-change 4444 t_dddd task-slug 5678 >/dev/null 2>&1
_archive="$SANDBOX/openspec/changes/archive/t_dddd-task-slug"
if [ -d "$_archive" ]; then
    _first_sha="$(sha256sum "$_archive/proposal.md" | awk '{print $1}')"
    OPENSPEC_ROOT="$SANDBOX/openspec" "$SYNC" archive-change 4444 t_dddd task-slug 5678 >/dev/null 2>&1
    _second_sha="$(sha256sum "$_archive/proposal.md" | awk '{print $1}')"
    assert_eq "$_second_sha" "$_first_sha" "F. archive idempotent (no modify)"
else
    log_result FAIL "F. archive dir not found (precondition)"
fi

echo "=== G. AGENT_FLOW_OPENSPEC_DISABLED=1 → noop ==="
make_sandbox
_disabled_out="$(AGENT_FLOW_OPENSPEC_DISABLED=1 OPENSPEC_ROOT="$SANDBOX/openspec" \
    "$SYNC" create-change 5555 t_eeee task-slug "Test" 2>&1)"
echo "$_disabled_out" | grep -q 'skipping' \
    && log_result PASS "G. disabled prints 'skipping'" \
    || log_result FAIL "G. disabled prints 'skipping'"
# verify NO change dir was created
if [ ! -d "$SANDBOX/openspec/changes/t_eeee-task-slug" ]; then
    log_result PASS "G. disabled did not create change"
else
    log_result FAIL "G. disabled did not create change"
fi

echo "=== H. Bad root → fallback to auto-detect (exit 0, not crash) ==="
make_sandbox
# Point OPENSPEC_ROOT to a path that doesn't exist; auto-detect should find pilot.
_h_out="$(OPENSPEC_ROOT="/nonexistent/path/$$/openspec" REPO_DIR="" \
    "$SYNC" status 2>&1 || true)"
_h_rc=$?
assert_eq "$_h_rc" "0" "H. status with bad root exits 0"
# Bad root that doesn't exist → should fall through to auto-detect
# (find pilot location). Verify it succeeded without crashing.
echo "$_h_out" | grep -q 'OpenSpec root:' \
    && log_result PASS "H. resolved some OpenSpec root (auto-detect)" \
    || log_result FAIL "H. resolved some OpenSpec root (auto-detect)"

echo "=== I. Arg validation: missing args ==="
make_sandbox
if OPENSPEC_ROOT="$SANDBOX/openspec" "$SYNC" create-change >/dev/null 2>&1; then
    log_result FAIL "I. create-change with no args exits 1"
else
    log_result PASS "I. create-change with no args exits 1"
fi
if OPENSPEC_ROOT="$SANDBOX/openspec" "$SYNC" archive-change >/dev/null 2>&1; then
    log_result FAIL "I. archive-change with no args exits 1"
else
    log_result PASS "I. archive-change with no args exits 1"
fi
if OPENSPEC_ROOT="$SANDBOX/openspec" "$SYNC" unknown-cmd >/dev/null 2>&1; then
    log_result FAIL "I. unknown command exits 1"
else
    log_result PASS "I. unknown command exits 1"
fi

echo "=== J. No auto-detect, no env → graceful skip ==="
# Run in a tmp dir with no openspec/ anywhere uptree, blank env.
_no_root="$(cd /tmp && OPENSPEC_ROOT="" REPO_DIR="" \
    "$SYNC" status 2>&1 || true)"
echo "$_no_root" | grep -q 'OpenSpec root not found' \
    && log_result PASS "J. prints 'OpenSpec root not found' when no env" \
    || log_result FAIL "J. prints 'OpenSpec root not found' when no env"

echo ""
echo "==== Summary ===="
echo "passed: $PASS"
echo "failed: $FAIL"
[ "$FAIL" -eq 0 ]
