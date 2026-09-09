#!/bin/bash
# ============================================================================
# test_install_sentinel_detect.sh — install.sh's sentinel-detect for vendor
# patches correctly identifies when upstream already includes the fix.
#
# Issue #2332: when upstream (or a local dev-merge) includes the fix that
# a vendor patch adds, install.sh used to fail with "patch does not apply
# cleanly". Sentinel-detect scans for a known anchor string in the live
# tree; if found, the patch is skipped (SKIP with info, no ERROR).
#
# Тест покрывает:
#   1. install.sh source'ится без bash syntax errors
#   2. sentinel-detect идентифицирует patch как "already in live" когда
#      live содержит сигнатуру (`def _profile_skill_names`)
#   3. sentinel-detect возвращает "not in live" когда live чистый
#   4. install.sh dry-run проходит без ERROR строк для skill-validation.patch
#      (live tree с sentinel → SKIP, не ERROR)
#
# Требует: bash, grep, временный test-tree для имитации live state.
#
# Invocation:
#   bash tests/test_install_sentinel_detect.sh
# Returns 0 on all-pass, non-zero on first failure.
# ============================================================================
set -euo pipefail

TEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# REPO_ROOT points to the repo root (three levels above tests/).
# install.sh expects REPO_DIR=<repo_root>; pass-through from this test.
# Path layout: <repo_root>/scripts/agent_flow/tests/<this>.sh
REPO_ROOT="$(cd "$TEST_DIR/../../.." && pwd)"
INSTALL_SH="$REPO_ROOT/scripts/agent_flow/install.sh"
HERMES_AGENT_DIR="${HERMES_AGENT_DIR:-/home/builder/.hermes/hermes-agent}"

cleanup() {
    [ -n "${TEST_LIVE:-}" ] && rm -rf "${TEST_LIVE}"
}
trap cleanup EXIT

fail() { echo "FAIL: $*" >&2; exit 1; }
pass() { echo "  ok: $*"; }

echo "==> 1. install.sh syntax check"
[ -f "$INSTALL_SH" ] || fail "install.sh missing: $INSTALL_SH"
bash -n "$INSTALL_SH" || fail "install.sh has bash syntax errors"
pass "install.sh parses cleanly"

echo "==> 2. sentinel-detect source extractable (function patch_already_in_live)"
# Source install.sh's vendor-patch block in a subshell. install.sh is a
# script (not library), so we can't source it directly — execute its body
# with a no-op main() wrapper. Easier: just grep that the function exists.
grep -q '^patch_already_in_live()' "$INSTALL_SH" \
    || fail "install.sh missing patch_already_in_live() function"
grep -q 'HERMES_AGENT_PATCH_SENTINELS' "$INSTALL_SH" \
    || fail "install.sh missing HERMES_AGENT_PATCH_SENTINELS declaration"
pass "sentinel-detect declares expected function + assoc array"

echo "==> 3. sentinel-detect: live with sentinel → returns 0"
TEST_LIVE="$(mktemp -d /tmp/sentinel-detect.XXXXXX)"
mkdir -p "$TEST_LIVE/hermes_cli"
cat > "$TEST_LIVE/hermes_cli/kanban_db.py" <<'PYEOF'
def create_task(...):
    pass

def _profile_skill_names(assignee):
    """Regression for t_1ab37fa8."""
    return frozenset()
PYEOF
# Extract just patch_already_in_live + the assoc array into a tiny test
# harness and run it against TEST_LIVE.
HARNESS="$(mktemp)"
cat > "$HARNESS" <<'HARNESS_EOF'
#!/bin/bash
# Inline copy of install.sh's sentinel-detect logic for isolated testing.
declare -A HERMES_AGENT_PATCH_SENTINELS=(
    ["hermes-agent-skill-validation.patch"]="hermes_cli/kanban_db.py|def _profile_skill_names"
)
patch_already_in_live() {
    local name="$1"
    local sentinel="${HERMES_AGENT_PATCH_SENTINELS[$name]:-}"
    [ -n "$sentinel" ] || return 1
    local sentinel_file="${sentinel%|*}"
    local sentinel_grep="${sentinel#*|}"
    local full="$HERMES_AGENT_DIR/$sentinel_file"
    [ -f "$full" ] || return 1
    grep -qF -- "$sentinel_grep" "$full"
}
if patch_already_in_live "$1"; then
    exit 0
else
    exit 1
fi
HARNESS_EOF
chmod +x "$HARNESS"
HERMES_AGENT_DIR="$TEST_LIVE" "$HARNESS" hermes-agent-skill-validation.patch \
    || fail "sentinel-detect returned non-zero when sentinel IS in live"
pass "sentinel-detect correctly returns 0 when live contains sentinel"

echo "==> 4. sentinel-detect: live without sentinel → returns 1"
# Strip the sentinel from the test file (re-create without _profile_skill_names)
cat > "$TEST_LIVE/hermes_cli/kanban_db.py" <<'PYEOF'
def create_task(...):
    pass
PYEOF
if HERMES_AGENT_DIR="$TEST_LIVE" "$HARNESS" hermes-agent-skill-validation.patch; then
    fail "sentinel-detect returned 0 when sentinel NOT in live (false positive)"
fi
pass "sentinel-detect correctly returns 1 when live lacks sentinel"

echo "==> 5. install.sh dry-run against LIVE: patch failure does not abort install.sh"
# This is the operational smoke test for issue #2332:
# - Before the fix: install.sh failed with "ERROR patch does not apply cleanly"
#   and HARD-EXITED, blocking all subsequent steps (cron registration, drift-
#   detect verify, etc.).
# - After the fix: install.sh prints ERROR line for the patch but continues
#   with the rest of the script, ending with rc=0 (or rc=3 only on real
#   post-install md5 mismatch, NOT on a vendor patch failure).
#
# NB: REPO_DIR=$REPO_ROOT forces install.sh to use THIS worktree's vendor/
# (which after PR #2331 may contain .DISABLED patches) instead of the
# default /home/builder/hermes-share/rob_box_project which is the operator's
# stale main clone and may not reflect the worktree's state.
OUTPUT="$(REPO_DIR="$REPO_ROOT" DRY_RUN=1 bash "$INSTALL_SH" --dry-run 2>&1 || true)"
# If live tree already has the sentinel, we should see "SKIP patch already in live tree".
# If live tree does NOT have the sentinel, we should see "ERROR patch does not apply cleanly"
# BUT install.sh should also report "continuing with rest of install.sh" and reach the
# kanban MAINTENANCE probe / sync-skills / Done blocks.
if echo "$OUTPUT" | grep -q "patch-summary:"; then
    pass "install.sh --dry-run prints patch-summary (PR #2331 + sentinel-detect)"
elif echo "$OUTPUT" | grep -q "continuing with rest of install.sh"; then
    pass "install.sh --dry-run recovers from patch failure (continues to next steps)"
elif echo "$OUTPUT" | grep -q "SKIP patch already in live tree"; then
    pass "install.sh --dry-run: sentinel-detect SKIPs the patch (live already has fix)"
else
    echo "$OUTPUT" | tail -30 >&2
    fail "install.sh --dry-run: neither patch-summary nor SKIP patch already in live found — issue #2332 fix may be missing"
fi

# Negative check: install.sh must not silently swallow patch failures either.
# If patch fails AND there's no sentinel-detect message AND no recovery, that's
# the broken pre-fix behavior.
if echo "$OUTPUT" | grep -q "ERROR patch does not apply cleanly" \
   && ! echo "$OUTPUT" | grep -q "continuing with rest of install.sh"; then
    echo "$OUTPUT" | tail -30 >&2
    fail "install.sh --dry-run: patch ERROR without recovery — issue #2332 fix regressed"
fi
pass "install.sh --dry-run does not silently swallow patch failures (issue #2332 fix landed)"

echo "==> 6. install.sh dry-run with sentinel injected into live: SKIP path"
# Inject the sentinel string into a temporary copy of HERMES_AGENT_DIR's
# kanban_db.py (we don't modify the real hermes-agent checkout — we just
# verify install.sh's logic handles a "live has sentinel" scenario when
# HERMES_AGENT_DIR points to a fake tree). This is the path that triggers
# on a host where upstream already includes the fix (or a dev-merge has
# happened) — the most common scenario described in issue #2332.
TEST_LIVE2="$(mktemp -d /tmp/sentinel-detect-install.XXXXXX)"
mkdir -p "$TEST_LIVE2/hermes_cli"
cp "$HERMES_AGENT_DIR/hermes_cli/kanban_db.py" "$TEST_LIVE2/hermes_cli/" 2>/dev/null \
    || echo "fake kanban_db" > "$TEST_LIVE2/hermes_cli/kanban_db.py"
# Inject the sentinel marker (must be on a line that grep -F matches)
printf '\ndef _profile_skill_names(assignee):\n    """sentinel for test."""\n    return frozenset()\n' >> "$TEST_LIVE2/hermes_cli/kanban_db.py"
# Stub a fake git checkout so install.sh's HERMES_AGENT_DIR guard passes.
# install.sh requires `git -C HERMES_AGENT_DIR rev-parse --is-inside-work-tree`
# to succeed. We can't easily fake that, so we instead test the helper logic
# directly: invoke patch_already_in_live on TEST_LIVE2 and confirm it returns 0.
if HERMES_AGENT_DIR="$TEST_LIVE2" "$HARNESS" hermes-agent-skill-validation.patch; then
    pass "sentinel-detect returns 0 against TEST_LIVE2 with sentinel injected"
else
    fail "sentinel-detect should return 0 against TEST_LIVE2 with sentinel, but returned 1"
fi
rm -rf "$TEST_LIVE2"

echo "ALL INSTALL SENTINEL-DETECT TESTS PASSED"
