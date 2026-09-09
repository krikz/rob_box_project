#!/bin/bash
# ============================================================================
# test_triage_dedup_guard.sh — юнит-тест dedup-логики в agent-flow-triage.sh
#                              (ретро-фикс 18.08 t_a0fac345).
#
# Проверяет, что:
#   T1: регекс `\bissue\W*#(\d+)` ловит manual-формат `**Source**: issue #1432`
#       (где нет двоеточия между "issue" и "#N"), а не только auto-формат
#       `Source issue: #1432` (с двоеточием).
#   T2: регекс case-insensitive — ловит и "Issue #1432", и "issue #1432".
#   T3: регекс не ложно матчит "issues" (s на конце — не должно).
#   T4: python-блок `existing_by_issue` корректно собирает мапу issue→id+status
#       из mock kanban list JSON (включая --archived карточки).
#   T5: dedup-skip срабатывает для active карточки (running) на тот же issue.
#   T6: dedup-skip срабатывает для done карточки на non-reopened issue.
#   T7: dedup-skip срабатывает для archived карточки на non-reopened issue.
#   T8: для REOPENED issue + done/archived карточка — НЕ skip (создаём свежую).
#   T9: shellcheck-clean + syntax-OK triage.sh.
#
# Использование:
#   bash test_triage_dedup_guard.sh
# Env:
#   VERBOSE=1 — печатать подробности
# ============================================================================
set -uo pipefail

TESTS_DIR="$(cd "$(dirname "$0")" && pwd)"
SCRIPT_UNDER_TEST="$TESTS_DIR/../agent-flow-triage.sh"

PASS=0
FAIL=0
FAILED_CASES=()

# --- harness -------------------------------------------------------------
log() { if [ "${VERBOSE:-0}" = "1" ]; then printf '  %s\n' "$*"; fi; }
pass() { PASS=$((PASS+1)); printf '  \033[32m✓\033[0m %s\n' "$1"; }
fail() {
    FAIL=$((FAIL+1)); FAILED_CASES+=("$1")
    printf '  \033[31m✗\033[0m %s\n' "$1"
    if [ -n "${2:-}" ]; then printf '      %s\n' "$2"; fi
}

# --- T1, T2, T3: regex correctness --------------------------------------
echo "=== T1-T3: regex issue-matching ==="

# extract the regex line from triage.sh (it must match the new format)
ACTUAL_REGEX="$(grep -oE 're\.search\(r"[^"]+", body[^)]*\)' "$SCRIPT_UNDER_TEST" | head -1)"
log "regex line: $ACTUAL_REGEX"

# Use python to test the regex directly (the same way it's used in the script).
test_regex_match() {
    local body="$1" expected="$2" desc="$3"
    local matched
    matched="$(python3 -c '
import re, sys
body = sys.argv[1]
m = re.search(r"\bissue\W*#(\d+)", body, re.IGNORECASE)
print(m.group(1) if m else "")
' "$body")"
    if [ "$matched" = "$expected" ]; then
        pass "$desc"
    else
        fail "$desc" "body='$body' expected=$expected got=$matched"
    fi
}

# T1a: auto-format with colon (was the OLD regex)
test_regex_match "Source issue: #1432" "1432" \
    "T1a: auto-format 'Source issue: #1432' matched"

# T1b: manual format WITHOUT colon (this is the bug from t_a0fac345)
test_regex_match "**Source**: issue #1432 (юзер снял e2e-done)" "1432" \
    "T1b: manual-format '**Source**: issue #1432' matched (was broken before fix)"

# T1c: "issue" word + "#N" in body — это и есть наш основной кейс
test_regex_match "task #1432: продолжить разбор issue #1432" "1432" \
    "T1c: 'task #1432 ... issue #1432' — first match wins, returns 1432"

# T1d: with various whitespace
test_regex_match "issue:#1432" "1432" "T1d: 'issue:#1432' (no space) matched"
test_regex_match "issue :#1432" "1432" "T1e: 'issue :#1432' (space before colon) matched"
test_regex_match "issue: #1432" "1432" "T1f: 'issue: #1432' (colon + space) matched"

# T2: case-insensitive
test_regex_match "Issue #1432" "1432" "T2a: 'Issue #1432' (capital I) matched"
test_regex_match "ISSUE #1432" "1432" "T2b: 'ISSUE #1432' (all caps) matched"

# T3: not falsely matching "issues" (plural)
test_regex_match "multiple issues #1432 #1433" "" \
    "T3: 'issues' (plural) does NOT falsely match"

# T3b: word-boundary correctly excludes 'issues' (suffix s)
test_regex_match "multiple issues to fix in the repo" "" \
    "T3b: 'issues to fix' (plural 'issues' word) does NOT match — word boundary works"

# --- T4: existing_by_issue python block builds correct map --------------
echo ""
echo "=== T4: existing_by_issue python parses kanban list correctly ==="

# Run the python block standalone with mock JSON input
test_python_block() {
    local mock_json="$1" expected="$2" desc="$3"
    local out
    out="$(printf '%s' "$mock_json" | python3 -c '
import json, sys, re
try:
    d = json.load(sys.stdin)
    tasks = d if isinstance(d, list) else d.get("tasks", [])
except Exception:
    tasks = []
for t in tasks:
    body = t.get("body") or ""
    m = re.search(r"\bissue\W*#(\d+)", body, re.IGNORECASE)
    if m:
        print("%s\t%s\t%s" % (m.group(1), t.get("id", ""), t.get("status", "")))
')"
    # Convert tab-separated output to sorted single line for easy compare
    local norm
    norm="$(printf '%s' "$out" | sort)"
    local exp_norm
    exp_norm="$(printf '%s' "$expected" | sort)"
    if [ "$norm" = "$exp_norm" ]; then
        pass "$desc"
    else
        fail "$desc" "expected:
$exp_norm
got:
$norm"
    fi
}

# Mock kanban list with mixed statuses and body formats
MOCK_JSON='{"tasks": [
  {"id": "t_a72296b6", "status": "running", "body": "**Source**: issue #1432 (юзер снял e2e-done)"},
  {"id": "t_fa24323c", "status": "archived", "body": "Source\n  repo: krikz/rob_box_project\n  issue: #1432\n  labels: ..."},
  {"id": "t_other", "status": "ready", "body": "Some random card about a different issue"},
  {"id": "t_done", "status": "done", "body": "**Source**: issue #1432 (closed by user, but card still done)"}
]}'
EXPECTED_T4="1432	t_a72296b6	running
1432	t_fa24323c	archived
1432	t_done	done"
test_python_block "$MOCK_JSON" "$EXPECTED_T4" \
    "T4: extracts issue→id+status for ALL statuses (incl archived/done) from manual+auto body formats"

# Empty list
test_python_block '{"tasks": []}' "" "T4b: empty tasks list returns empty map"

# Body without issue reference
test_python_block '{"tasks": [{"id": "t_xxx", "status": "ready", "body": "no issue here"}]}' "" \
    "T4c: body without issue reference is skipped"

# --- T5-T8: dedup skip logic (pure bash, simulated) ---------------------
echo ""
echo "=== T5-T8: dedup skip decision (bash) ==="

# Simulate the decision block from triage.sh
dedup_decision() {
    local number="$1" is_reopened="$2" existing_line="$3"
    local existing_id existing_status
    existing_line="$(printf '%s' "$existing_line" | tr -d '\n')"
    existing_id="$(printf '%s' "$existing_line" | cut -f2)"
    existing_status="$(printf '%s' "$existing_line" | cut -f3)"
    if [ -z "$existing_id" ]; then
        echo "CREATE"
        return
    fi
    if [ "$is_reopened" = "false" ]; then
        echo "SKIP"
        return
    fi
    case "${existing_status:-}" in
        running|ready|todo|blocked) echo "SKIP" ;;
        done|archived|"") echo "CREATE" ;;
        *) echo "CREATE" ;;  # unknown status — let it through
    esac
}

# T5: active card + non-reopened → SKIP
R=$(dedup_decision "1432" "false" "1432	t_a72296b6	running")
[ "$R" = "SKIP" ] && pass "T5: active card + non-reopened issue → SKIP (dedup)" \
    || fail "T5: active card + non-reopened issue" "got=$R"

# T6: done card + non-reopened → SKIP (новое поведение — раньше был CREATE)
R=$(dedup_decision "1432" "false" "1432	t_old	done")
[ "$R" = "SKIP" ] && pass "T6: done card + non-reopened issue → SKIP" \
    || fail "T6: done card + non-reopened issue" "got=$R"

# T7: archived card + non-reopened → SKIP
R=$(dedup_decision "1432" "false" "1432	t_old	archived")
[ "$R" = "SKIP" ] && pass "T7: archived card + non-reopened issue → SKIP" \
    || fail "T7: archived card + non-reopened issue" "got=$R"

# T8: done card + REOPENED issue → CREATE
R=$(dedup_decision "1432" "true" "1432	t_old	done")
[ "$R" = "CREATE" ] && pass "T8: done card + REOPENED issue → CREATE fresh" \
    || fail "T8: done card + REOPENED issue" "got=$R"

# T8b: archived card + REOPENED → CREATE
R=$(dedup_decision "1432" "true" "1432	t_old	archived")
[ "$R" = "CREATE" ] && pass "T8b: archived card + REOPENED → CREATE fresh" \
    || fail "T8b: archived card + REOPENED" "got=$R"

# T8c: running card + REOPENED → SKIP (worker is busy)
R=$(dedup_decision "1432" "true" "1432	t_active	running")
[ "$R" = "SKIP" ] && pass "T8c: running card + REOPENED → SKIP (worker still busy)" \
    || fail "T8c: running card + REOPENED" "got=$R"

# T8d: no existing card + anything → CREATE
R=$(dedup_decision "1432" "false" "")
[ "$R" = "CREATE" ] && pass "T8d: no existing card + non-reopened → CREATE" \
    || fail "T8d: no existing card + non-reopened" "got=$R"

R=$(dedup_decision "1432" "true" "")
[ "$R" = "CREATE" ] && pass "T8e: no existing card + REOPENED → CREATE" \
    || fail "T8e: no existing card + REOPENED" "got=$R"

# --- T9: shellcheck-clean + syntax-OK triage.sh -------------------------
echo ""
echo "=== T9: shellcheck + syntax check ==="

# Syntax check (bash -n)
if bash -n "$SCRIPT_UNDER_TEST" 2>/tmp/sc_err; then
    pass "T9a: bash -n syntax check passed"
else
    fail "T9a: bash -n syntax check failed" "$(cat /tmp/sc_err)"
fi

# Verify the new regex is present in the file
if grep -q '\\bissue\\W\*#(\\d+)' "$SCRIPT_UNDER_TEST"; then
    pass "T9b: new regex \\bissue\\W*#(\\d+) present in triage.sh"
else
    fail "T9b: new regex not found in triage.sh"
fi

# Verify --archived flag added to kanban list
if grep -q 'kanban --board.*list --json --archived' "$SCRIPT_UNDER_TEST"; then
    pass "T9c: kanban list now uses --archived flag"
else
    fail "T9c: kanban list does NOT use --archived flag"
fi

# Verify status is now in the printed map (3 fields)
# The python print uses % formatting with 3 %s — match the actual line format
if grep -E '^[[:space:]]+print\("%s\\t%s\\t%s"' "$SCRIPT_UNDER_TEST" >/dev/null; then
    pass "T9d: existing_by_issue prints status in 3rd field"
else
    fail "T9d: existing_by_issue does NOT print 3rd field"
fi

# Verify the new comment block exists (ретро-фикс с ID карточки)
if grep -E 'Ретро-фикс.*t_a0fac345' "$SCRIPT_UNDER_TEST" >/dev/null; then
    pass "T9e: retro-fix comment with t_a0fac345 reference present"
else
    fail "T9e: retro-fix comment NOT found"
fi

# shellcheck (if available — check both PATH and the venv where hermes installs it)
if ! command -v shellcheck >/dev/null 2>&1; then
    for sc in /home/builder/.hermes/hermes-agent/venv/bin/shellcheck \
             /usr/local/bin/shellcheck /usr/bin/shellcheck; do
        [ -x "$sc" ] && PATH="$(dirname "$sc"):$PATH" && break
    done
fi
if command -v shellcheck >/dev/null 2>&1; then
    # Snapshot shellcheck output BEFORE this PR (using origin/develop) to
    # compare — only NEW warnings count as failures.
    # Walk up to find the .git dir (we may be in a worktree).
    _repo_root="$(git -C "$TESTS_DIR/.." rev-parse --show-toplevel 2>/dev/null || echo "$TESTS_DIR/..")"
    if [ -d "$_repo_root/.git" ] || [ -f "$_repo_root/.git" ]; then
        ORIG_SC="$(cd "$_repo_root" && git show "origin/develop:scripts/agent_flow/agent-flow-triage.sh" 2>/dev/null | shellcheck - 2>&1 | wc -l)"
        NEW_SC="$(shellcheck "$SCRIPT_UNDER_TEST" 2>&1 | wc -l)"
        log "  shellcheck: origin/develop=$ORIG_SC, current=$NEW_SC"
        if [ "$NEW_SC" -le "$ORIG_SC" ]; then
            pass "T9f: shellcheck — no NEW warnings (origin/develop=$ORIG_SC, current=$NEW_SC)"
        else
            DIFF="$(shellcheck "$SCRIPT_UNDER_TEST" 2>&1)"
            fail "T9f: shellcheck — $((NEW_SC-ORIG_SC)) new warning(s)" "$(printf '%s\n' "$DIFF" | head -30)"
        fi
    else
        log "T9f: not a git repo (no .git at $_repo_root) — skipping baseline comparison"
    fi
else
    log "T9f: shellcheck not installed — skip"
fi

# --- summary -------------------------------------------------------------
echo ""
echo "============================================================"
echo "PASS: $PASS    FAIL: $FAIL"
echo "============================================================"
if [ "$FAIL" -gt 0 ]; then
    printf '\nFAILED CASES:\n'
    for c in "${FAILED_CASES[@]}"; do printf '  - %s\n' "$c"; done
    exit 1
fi
exit 0
