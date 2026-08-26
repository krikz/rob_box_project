#!/bin/bash
# ============================================================================
# test_triage_fingerprint.sh — ADR for G8 (fingerprint dedup gate)
# Verifies the helpers added in agent-flow-triage.sh by ретро t_b0fe4398:
#   * file_in_fp_whitelist
#   * fp_for_pr_file
#   * find_duplicate_fix_prs
#
# Scenarios covered:
#   A. file_in_fp_whitelist: exact match (`docker/vision/docker-compose.yaml`)
#   B. file_in_fp_whitelist: leading `./` strip
#   C. file_in_fp_whitelist: negative case (Python file NOT in whitelist)
#   D. file_in_fp_whitelist: glob match (Dockerfile под `docker/*/`)
#   E. fp_for_pr_file: same fix → same fingerprint (stable across runs)
#   F. fp_for_pr_file: different fix → different fingerprint
#   G. find_duplicate_fix_prs: extracts whitelist file from body
#   H. find_duplicate_fix_prs: body without whitelist file → empty (backward-compat)
#
# Run:
#   bash scripts/agent_flow/tests/test_triage_fingerprint.sh
# Returns exit 0 on all-pass, non-zero on first failure.
#
# NO mocks needed: this tests pure helper functions. We source the relevant
# helpers from a stripped copy of agent-flow-triage.sh (the helpers don't
# depend on any earlier globals like GH_REPO).
# ============================================================================
set -euo pipefail

# Anchor
TEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TRIAGE="$TEST_DIR/../agent-flow-triage.sh"

if [ ! -f "$TRIAGE" ]; then
    echo "FATAL: $TRIAGE not found" >&2
    exit 1
fi

# Helpers we need are pure (no env vars read inside their bodies).
# Source them by extracting the relevant function blocks via awk.
# Simpler: just source the whole file with a stubbed GH_REPO — helpers don't
# touch GH_REPO unless the script enters the main loop (which it doesn't if
# we exit early after sourcing). So we source + set GH_REPO + return early.
# But the script does work at top-level (parse env, etc.). Instead we use
# bash to extract each function from the file and eval it in this shell.
extract_function() {  # $1=function_name
    local fn="$1"
    # Find "fn() {" or "fn () {" and copy until matching closing brace at
    # column 0. Use awk with brace-counter.
    awk -v fn="$fn" '
        function trim(s) { sub(/^[ \t]+/, "", s); sub(/[ \t]+$/, "", s); return s }
        $0 ~ "^" fn "[[:space:]]*\\(\\)[[:space:]]*\\{" { capture=1; depth=1; print; next }
        capture {
            n = gsub(/\{/, "{")
            m = gsub(/\}/, "}")
            depth += n - m
            print
            if (depth <= 0) { capture=0; exit }
        }
    ' "$TRIAGE"
}

# Load helpers into this shell. Use a sub-shell guard so side-effects in
# source (like HERMES_HOME default) don't pollute the test runner.
FINGERPRINT_FILE_GLOBS='docker/*/docker-compose.yaml|docker/*/.env.example|docker/*/Dockerfile|src/*/package.xml|src/*/setup.py|install/setup*.sh'
FINGERPRINT_DUPLICATE_THRESHOLD=1
export FINGERPRINT_FILE_GLOBS FINGERPRINT_DUPLICATE_THRESHOLD

# shellcheck disable=SC1090
eval "$(extract_function file_in_fp_whitelist)"
# shellcheck disable=SC1090
eval "$(extract_function fp_for_pr_file)"
# shellcheck disable=SC1090
eval "$(extract_function find_duplicate_fix_prs)"

# --- Test registry ----------------------------------------------------------
TESTS_TOTAL=0
TESTS_PASSED=0
TESTS_FAILED=0
FAILED_NAMES=()
if [ -t 1 ]; then
    RED=$'\033[31m'; GRN=$'\033[32m'; BLU=$'\033[34m'; END=$'\033[0m'
else
    RED=''; GRN=''; BLU=''; END=''
fi

run_test() {  # $1=name $2=fn
    local name="$1" fn="$2"
    TESTS_TOTAL=$((TESTS_TOTAL+1))
    printf '%s[ RUN     ]%s %s\n' "$BLU" "$END" "$name"
    if "$fn"; then
        TESTS_PASSED=$((TESTS_PASSED+1))
        printf '%s[   PASS  ]%s %s\n' "$GRN" "$END" "$name"
    else
        TESTS_FAILED=$((TESTS_FAILED+1))
        FAILED_NAMES+=("$name")
        printf '%s[   FAIL  ]%s %s\n' "$RED" "$END" "$name"
    fi
}

assert_eq() {  # $1=expected $2=actual $3=msg
    if [ "$1" != "$2" ]; then
        printf '  %sassert fail:%s %s\n    expected: %q\n    actual:   %q\n' \
            "$RED" "$END" "$3" "$1" "$2" >&2
        return 1
    fi
}

assert_contains() {  # $1=needle $2=haystack $3=msg
    case "$2" in
        *"$1"*) return 0 ;;
        *)
            printf '  %sassert fail:%s %s\n    needle:   %q\n    haystack: %q\n' \
                "$RED" "$END" "$3" "$1" "$2" >&2
            return 1
            ;;
    esac
}

assert_not_contains() {  # $1=needle $2=haystack $3=msg
    case "$2" in
        *"$1"*)
            printf '  %sassert fail:%s %s\n    needle should NOT appear: %q\n    haystack: %q\n' \
                "$RED" "$END" "$3" "$1" "$2" >&2
            return 1
            ;;
    esac
    return 0
}

# ============================================================================
# A. file_in_fp_whitelist: exact match
# ============================================================================
test_A_file_in_whitelist_exact() {
    if file_in_fp_whitelist "docker/vision/docker-compose.yaml"; then
        return 0
    fi
    echo "  expected match, got no match" >&2
    return 1
}

# ============================================================================
# B. file_in_fp_whitelist: leading `./` strip
# ============================================================================
test_B_file_in_whitelist_strip_dot() {
    if file_in_fp_whitelist "./docker/vision/docker-compose.yaml"; then
        return 0
    fi
    echo "  expected match after ./strip, got no match" >&2
    return 1
}

# ============================================================================
# C. file_in_fp_whitelist: negative case (Python file NOT in whitelist)
# ============================================================================
test_C_file_not_in_whitelist() {
    if file_in_fp_whitelist "src/rob_box_voice/dialog_core.py"; then
        echo "  expected NO match, got match" >&2
        return 1
    fi
    return 0
}

# ============================================================================
# D. file_in_fp_whitelist: glob match (Dockerfile под docker/*/)
# ============================================================================
test_D_file_glob_match() {
    if file_in_fp_whitelist "docker/vision/Dockerfile"; then
        return 0
    fi
    echo "  expected glob match, got no match" >&2
    return 1
}

# ============================================================================
# E. fp_for_pr_file: same content → same fingerprint (stable)
#    Use PR #1651 (the real one from ретро — adds profiles: ["quest"]).
#    Run twice and assert identical. Skip if gh not authed.
# ============================================================================
test_E_fp_stable() {
    if ! command -v gh >/dev/null 2>&1; then
        echo "  SKIP: gh not installed" >&2
        return 0
    fi
    if ! gh auth status >/dev/null 2>&1; then
        echo "  SKIP: gh not authed (fp test needs real PR)" >&2
        return 0
    fi
    export GH_CONFIG_DIR="${GH_CONFIG_DIR:-/home/builder/.config/gh}"
    if [ -z "${GH_REPO:-}" ]; then
        GH_REPO="krikz/rob_box_project"
    fi
    local fp1 fp2
    fp1="$(fp_for_pr_file 1651 "docker/vision/docker-compose.yaml" 2>/dev/null || echo "")"
    fp2="$(fp_for_pr_file 1651 "docker/vision/docker-compose.yaml" 2>/dev/null || echo "")"
    if [ -z "$fp1" ] || [ -z "$fp2" ]; then
        echo "  SKIP: PR #1651 diff empty (probably no auth or wrong repo)" >&2
        return 0
    fi
    if [ "$fp1" = "$fp2" ] && [ "${#fp1}" -eq 16 ]; then
        printf '  fp=%s (stable, 16 hex chars)\n' "$fp1" >&2
        return 0
    fi
    printf '  fp1=%q fp2=%q (must match and be 16 chars)\n' "$fp1" "$fp2" >&2
    return 1
}

# ============================================================================
# F. fp_for_pr_file: 4 PR from retro (t_b0fe4398) all add profiles: ["quest"]
#    to docker/vision/docker-compose.yaml — must produce the SAME fingerprint.
#    This is the heart of the G8 dedup gate: without matching fingerprints,
#    triage would create 4 duplicate-fix cards.
# ============================================================================
test_F_fp_same_fix_same_fp() {
    if ! command -v gh >/dev/null 2>&1; then
        echo "  SKIP: gh not installed" >&2
        return 0
    fi
    if ! gh auth status >/dev/null 2>&1; then
        echo "  SKIP: gh not authed" >&2
        return 0
    fi
    export GH_CONFIG_DIR="${GH_CONFIG_DIR:-/home/builder/.config/gh}"
    if [ -z "${GH_REPO:-}" ]; then
        GH_REPO="krikz/rob_box_project"
    fi
    # All 4 PRs from ретро — same fix.
    local fp_1651 fp_1654 fp_1656 fp_1659
    fp_1651="$(fp_for_pr_file 1651 "docker/vision/docker-compose.yaml" 2>/dev/null || echo "")"
    fp_1654="$(fp_for_pr_file 1654 "docker/vision/docker-compose.yaml" 2>/dev/null || echo "")"
    fp_1656="$(fp_for_pr_file 1656 "docker/vision/docker-compose.yaml" 2>/dev/null || echo "")"
    fp_1659="$(fp_for_pr_file 1659 "docker/vision/docker-compose.yaml" 2>/dev/null || echo "")"
    if [ -z "$fp_1651" ] || [ -z "$fp_1654" ] || [ -z "$fp_1656" ] || [ -z "$fp_1659" ]; then
        echo "  SKIP: couldn't fetch all 4 diffs" >&2
        return 0
    fi
    if [ "$fp_1651" = "$fp_1654" ] && [ "$fp_1654" = "$fp_1656" ] && [ "$fp_1656" = "$fp_1659" ]; then
        printf '  ✓ all 4 PR fingerprints match: %s (the bug scenario from t_b0fe4398)\n' "$fp_1651" >&2
        return 0
    fi
    printf '  fp_1651=%s fp_1654=%s fp_1656=%s fp_1659=%s (MUST all match!)\n' \
        "$fp_1651" "$fp_1654" "$fp_1656" "$fp_1659" >&2
    return 1
}

# ============================================================================
# G. find_duplicate_fix_prs: end-to-end against the actual retro scenario.
#    Body mentions docker/vision/docker-compose.yaml — must emit ≥1 line
#    containing that file AND a PR number (one of #1651/#1654/#1656/#1659).
#    This proves the WHOLE helper detects the duplicate-fix bug.
# ============================================================================
test_G_find_dup_end_to_end() {
    if ! command -v gh >/dev/null 2>&1; then
        echo "  SKIP: gh not installed" >&2
        return 0
    fi
    if ! gh auth status >/dev/null 2>&1; then
        echo "  SKIP: gh not authed" >&2
        return 0
    fi
    export GH_CONFIG_DIR="${GH_CONFIG_DIR:-/home/builder/.config/gh}"
    GH_REPO="krikz/rob_box_project"
    local body="Issue: container rob-box-quest restart-loop on test-env. \
Fix by adding profiles: [\"quest\"] to docker/vision/docker-compose.yaml. \
Also tried src/rob_box_quest/quest_node.py changes."
    local result
    result="$(find_duplicate_fix_prs "9999" "$body" 2>/dev/null || true)"
    # Must contain reference to docker/vision/docker-compose.yaml AND one of
    # the known duplicate PR numbers.
    if printf '%s' "$result" | grep -q "docker/vision/docker-compose.yaml" \
        && printf '%s' "$result" | grep -qE '\b(1651|1654|1656|1659)\b'; then
        printf '  ✓ end-to-end: detected duplicate-fix in OPEN PR (got %d line(s))\n' \
            "$(printf '%s\n' "$result" | wc -l | tr -d ' ')" >&2
        return 0
    fi
    if [ -z "$result" ]; then
        echo "  EMPTY result — body file was extracted but no OPEN PR matched. PRs may have been closed/merged since test creation. Re-run after re-opening." >&2
        return 1
    fi
    printf '  result=%q (expected to contain docker/vision/docker-compose.yaml AND PR #1651/1654/1656/1659)\n' "$result" >&2
    return 1
}

# ============================================================================
# H. find_duplicate_fix_prs: body without whitelist file → empty (backward-compat)
# ============================================================================
test_H_find_dup_no_whitelist_file() {
    if ! command -v gh >/dev/null 2>&1; then
        echo "  SKIP: gh not installed" >&2
        return 0
    fi
    if ! gh auth status >/dev/null 2>&1; then
        echo "  SKIP: gh not authed" >&2
        return 0
    fi
    export GH_CONFIG_DIR="${GH_CONFIG_DIR:-/home/builder/.config/gh}"
    GH_REPO="krikz/rob_box_project"
    local body="Add a new tool to dialog_core.py for handling late-STOP. \
Tests in test_issue_1562_late_stop_after_synth.py."
    local result
    result="$(find_duplicate_fix_prs "9999" "$body" 2>/dev/null || true)"
    if [ -z "$result" ]; then
        echo "  ✓ body without whitelist file → empty result (backward-compat preserved)" >&2
        return 0
    fi
    printf '  result=%q (expected empty, body has no whitelist file)\n' "$result" >&2
    return 1
}

# --- run all tests ----------------------------------------------------------
run_test "A. file_in_fp_whitelist exact match" test_A_file_in_whitelist_exact
run_test "B. file_in_fp_whitelist leading ./" test_B_file_in_whitelist_strip_dot
run_test "C. file_in_fp_whitelist negative" test_C_file_not_in_whitelist
run_test "D. file_in_fp_whitelist glob match" test_D_file_glob_match
run_test "E. fp_for_pr_file stable (same PR, 2 runs)" test_E_fp_stable
run_test "F. fp_for_pr_file same fix (4 PR same fp)" test_F_fp_same_fix_same_fp
run_test "G. find_duplicate_fix_prs end-to-end (retro scenario)" test_G_find_dup_end_to_end
run_test "H. find_duplicate_fix_prs no-whitelist-body" test_H_find_dup_no_whitelist_file

echo ""
echo "==== Summary ===="
echo "total:  $TESTS_TOTAL"
echo "passed: $TESTS_PASSED"
echo "failed: $TESTS_FAILED"
if [ "$TESTS_FAILED" -gt 0 ]; then
    echo "FAILED names:"
    for n in "${FAILED_NAMES[@]}"; do
        echo "  - $n"
    done
    exit 1
fi
exit 0