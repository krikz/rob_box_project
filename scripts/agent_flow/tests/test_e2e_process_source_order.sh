#!/bin/bash
# ============================================================================
# test_e2e_process_source_order.sh — ретро t_df4fff46, issue #1586
#
# Regression: ``. hermes_github.sh`` MUST appear ABOVE the earliest top-level
# ``whoami_*`` call site in agent-flow-e2e-process.sh. Otherwise bash
# resolves the function name at call time → "command not found" crash →
# entire e2e rotation freezes (ретро t_df4fff46: 3+ hours of crashes on
# 24.08 06:00-09:30 UTC, until a host-only hotfix landed).
#
# Scenarios:
#   A. structural order — every ``. hermes_github.sh`` line precedes the
#      first top-level ``whoami_*`` call site
#   B. bash -n parse — script must still parse cleanly after refactors
#   C. unit heredoc — the early source block MUST be the ONLY source block
#      (no duplicate ``_LIB_DIR_HERE=`` further down that drifts apart on
#      host installs)
#
# Run:
#   bash scripts/agent_flow/tests/test_e2e_process_source_order.sh
# Returns 0 on all-pass, non-zero on first failure.
# ============================================================================
set -euo pipefail

TEST_LIB_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TEST_ROOT_DIR="$(cd "$TEST_LIB_DIR/.." && pwd)"
E2E_PROCESS="$TEST_ROOT_DIR/agent-flow-e2e-process.sh"

if [ ! -f "$E2E_PROCESS" ]; then
    echo "FATAL: e2e-process script not found at $E2E_PROCESS" >&2
    exit 2
fi

# Colors (matches sibling tests)
if [ -t 1 ]; then
    RED=$'\033[31m'; GRN=$'\033[32m'; YEL=$'\033[33m'; END=$'\033[0m'
else
    RED=''; GRN=''; YEL=''; END=''
fi

pass=0; fail=0
report() {
    local name="$1" status="$2" detail="${3:-}"
    if [ "$status" = "PASS" ]; then
        printf '  %s✓%s %s\n' "$GRN" "$END" "$name"
        pass=$((pass+1))
    else
        printf '  %s✗%s %s — %s\n' "$RED" "$END" "$name" "$detail"
        fail=$((fail+1))
    fi
}

echo "[test_e2e_process_source_order] running against $E2E_PROCESS"

# --- A. structural order -----------------------------------------------------
# Find every . hermes_github.sh line and the earliest top-level whoami_* call.
source_lines=$(grep -nE '^\s*\.\s+"\$_LIB_DIR_HERE/hermes_github\.sh"\s*$' "$E2E_PROCESS" | cut -d: -f1 || true)
whoami_line=$(grep -nE '^[^#]*\bwhoami_(add_label|remove_label|close_issue|reopen_issue|set_assignee|close_pr)\b' "$E2E_PROCESS" | head -1 | cut -d: -f1 || true)

if [ -z "$source_lines" ]; then
    report "A1. . hermes_github.sh present" FAIL "no source line found — script lost its whoami helper"
elif [ -z "$whoami_line" ]; then
    report "A1. whoami_* call site present" FAIL "no top-level whoami_* call found — drop the source line together with the dependency"
else
    # Earliest source line:
    earliest_source=$(printf '%s\n' "$source_lines" | sort -n | head -1)
    if [ "$earliest_source" -lt "$whoami_line" ]; then
        report "A. . hermes_github.sh (line $earliest_source) precedes whoami_* (line $whoami_line)" PASS
    else
        report "A. . hermes_github.sh precedes whoami_*" FAIL "source at line $earliest_source, earliest whoami_* call at line $whoami_line — bash will resolve call BEFORE function definition"
    fi
fi

# --- B. bash -n parse --------------------------------------------------------
if bash -n "$E2E_PROCESS" 2>/dev/null; then
    report "B. bash -n parse" PASS
else
    report "B. bash -n parse" FAIL "syntax error introduced — run bash -n $E2E_PROCESS to inspect"
fi

# --- C. single source block --------------------------------------------------
# The old buggy layout had source-order blocks both at top and at line ~1100.
# Once we move the bootstrap up, there should be exactly ONE _LIB_DIR_HERE=.
# Count occurrences.
lib_decl=$(grep -cE '^_LIB_DIR_HERE=' "$E2E_PROCESS" || true)
if [ "$lib_decl" -eq 1 ]; then
    report "C. exactly one _LIB_DIR_HERE= declaration" PASS
else
    report "C. exactly one _LIB_DIR_HERE= declaration" FAIL "found $lib_decl occurrences — multiple declarations drift apart on host installs (ретро t_df4fff46)"
fi

# Also count how many . hermes_github.sh lines we have. Should be exactly 1
# to keep the dependency graph auditable; multiple would mean a fall-back
# that masks source-order bugs in the primary block.
github_sources=$(grep -cE '^\s*\.\s+"\$_LIB_DIR_HERE/hermes_github\.sh"\s*$' "$E2E_PROCESS" || true)
if [ "$github_sources" -eq 1 ]; then
    report "C2. exactly one . hermes_github.sh source line" PASS
else
    report "C2. exactly one . hermes_github.sh source line" FAIL "found $github_sources occurrences — multiple sources invite drift"
fi

# --- summary -----------------------------------------------------------------
echo
if [ "$fail" -eq 0 ]; then
    printf '%sPASS%s: %d checks\n' "$GRN" "$END" "$pass"
    exit 0
else
    printf '%sFAIL%s: %d failed / %d passed\n' "$RED" "$END" "$fail" "$pass"
    exit 1
fi
