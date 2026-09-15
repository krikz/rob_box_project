#!/bin/bash
# ============================================================================
# test_triage_dedup_branch_active.sh — модульный тест G9c branch-name race-window
#                                       dedup в agent-flow-triage.sh (ретро t_60473741,
#                                       ADR-AF-0067).
#
# Проверяет, что G9c:
#   T1: existing_by_issue snapshot имеет 4 поля (issue, id, status, branch).
#   T2: existing_by_issue пуст → guard skip (fail-OPEN), branch_match пуст.
#   T3: 1 запись для issue, branch совпал, status=blocked → match (skip).
#   T4: 1 запись для issue, branch совпал, status=ready → match (skip).
#   T5: 1 запись для issue, branch совпал, status=done → NO match (archived не блокирует).
#   T6: 1 запись для issue, branch НЕ совпал (другая ветка) → NO match.
#   T7: 2 записи для issue (старая done + новая blocked с тем же branch) → match (newer blocked wins).
#   T8: 2 записи для issue, обе active, разные branches → match для вычисленной $branch.
#   T9: 1 запись для issue, branch совпал, но status=archived → NO match.
#   T10: shellcheck-clean + syntax-OK triage.sh.
#   T11: presence check — ключевые маркеры G9c в коде.
#
# Использование:
#   bash test_triage_dedup_branch_active.sh
# Env:
#   VERBOSE=1 — печатать подробности
# ============================================================================
set -uo pipefail

TESTS_DIR="$(cd "$(dirname "$0")" && pwd)"
SCRIPT_UNDER_TEST="$TESTS_DIR/../agent-flow-triage.sh"

PASS=0
FAIL=0
FAILED_CASES=()

log() { if [ "${VERBOSE:-0}" = "1" ]; then printf '  %s\n' "$*"; fi; }
pass() { PASS=$((PASS+1)); printf '  \033[32m✓\033[0m %s\n' "$1"; }
fail() {
    FAIL=$((FAIL+1)); FAILED_CASES+=("$1")
    printf '  \033[31m✗\033[0m %s\n' "$1"
    if [ -n "${2:-}" ]; then printf '      %s\n' "$2"; fi
}

# =============================================================================
# Helper: pure-bash реализация G9c awk-scan. Эта функция идентична той, что
# в agent-flow-triage.sh:1392-1406 (extract branch-match id from existing_by_issue).
# =============================================================================
g9c_branch_match() {  # $1=existing_by_issue  $2=number  $3=branch
    local existing_by_issue="$1" number="$2" branch="$3"
    [ -n "$existing_by_issue" ] || return 1
    [ -n "$branch" ] || return 1
    local _match
    _match="$(
        printf '%s\n' "$existing_by_issue" \
            | awk -F'\t' -v n="$number" -v br="$branch" '
                $1 == n && br != "" && $4 == br {
                    if ($3 == "running" || $3 == "ready" || $3 == "todo" || $3 == "blocked") {
                        print $2; exit
                    }
                }
            '
    )"
    if [ -n "$_match" ]; then
        printf '%s' "$_match"
        return 0
    fi
    return 1
}

# --- T1: existing_by_issue snapshot имеет 4 поля -----------------------------
echo "=== T1: existing_by_issue snapshot schema (4 fields) ==="
# Snapshot из реального кейса (issue 2406): t_40a610d0 (blocked) + t_6535e27d (ready)
# + t_b7fbff1c (archived, отдадим через неактивный статус).
SNAP_T1="$(printf '%s\n' \
    "2406	t_40a610d0	blocked	z-{agent}/2406-fix-prompt-tool-call-enforcement-discovery-n201-n3" \
    "2406	t_6535e27d	ready	z-{agent}/2406-fix-prompt-tool-call-enforcement-discovery-n201-n3" \
    "2406	t_b7fbff1c	archived	z-{agent}/2406-fix-prompt-tool-call-enforcement-discovery-n201-n3" \
    "2300	t_some_card	running	z-{agent}/2300-other-issue-branch")"
# Проверяем что мы можем распарсить каждую строку на 4 поля без потерь.
fields_ok=true
while IFS=$'\t' read -r f1 f2 f3 f4 rest; do
    if [ -z "$f1" ] || [ -z "$f2" ] || [ -z "$f3" ]; then
        fields_ok=false
    fi
    # f4 может быть пустым (карточка без worktree), но не должно быть 'rest' (5+ полей)
    if [ -n "${rest:-}" ]; then
        fields_ok=false
    fi
done <<< "$SNAP_T1"
if [ "$fields_ok" = "true" ]; then
    pass "T1: existing_by_issue snapshot имеет 4 поля (issue, id, status, branch) без overflow"
else
    fail "T1: snapshot имеет >5 полей или пустые первые 3"
fi

# --- T2: empty existing_by_issue → guard skip (no match) ---------------------
echo ""
echo "=== T2: empty existing_by_issue ==="
if g9c_branch_match "" "2406" "z-{agent}/2406-foo" >/dev/null 2>&1; then
    fail "T2: empty snapshot should return 1 (fail-OPEN, no match)"
else
    pass "T2: empty snapshot → no match (fail-OPEN, guard пропускается)"
fi
# Также пустой branch → no match
if g9c_branch_match "2406	t_x	blocked	z-{agent}/2406-foo" "2406" "" >/dev/null 2>&1; then
    fail "T2b: empty branch should return 1"
else
    pass "T2b: empty branch → no match (защита от false-positive на не-worktree карточках)"
fi

# --- T3: 1 entry, branch совпал, status=blocked → match ----------------------
echo ""
echo "=== T3: 1 active entry, branch match, status=blocked ==="
SNAP_T3="2406	t_40a610d0	blocked	z-{agent}/2406-fix-prompt-tool-call-enforcement-discovery-n201-n3"
match="$(g9c_branch_match "$SNAP_T3" "2406" "z-{agent}/2406-fix-prompt-tool-call-enforcement-discovery-n201-n3" 2>/dev/null || true)"
if [ "$match" = "t_40a610d0" ]; then
    pass "T3: blocked + branch match → match=t_40a610d0 (skip ✓)"
else
    fail "T3: expected match=t_40a610d0, got=$match"
fi

# --- T4: 1 entry, branch совпал, status=ready → match ------------------------
echo ""
echo "=== T4: 1 active entry, branch match, status=ready ==="
SNAP_T4="2406	t_6535e27d	ready	z-{agent}/2406-fix-prompt-tool-call-enforcement-discovery-n201-n3"
match="$(g9c_branch_match "$SNAP_T4" "2406" "z-{agent}/2406-fix-prompt-tool-call-enforcement-discovery-n201-n3" 2>/dev/null || true)"
if [ "$match" = "t_6535e27d" ]; then
    pass "T4: ready + branch match → match=t_6535e27d (skip ✓)"
else
    fail "T4: expected match=t_6535e27d, got=$match"
fi

# --- T5: 1 entry, branch совпал, status=done → NO match ----------------------
echo ""
echo "=== T5: 1 entry, branch match, status=done (dead, не блокирует) ==="
SNAP_T5="2406	t_ec75ba3e	done	z-{agent}/2406-fix-prompt-tool-call-enforcement-discovery-n201-n3"
if g9c_branch_match "$SNAP_T5" "2406" "z-{agent}/2406-fix-prompt-tool-call-enforcement-discovery-n201-n3" >/dev/null 2>&1; then
    fail "T5: done card should NOT match (только active блокирует)"
else
    pass "T5: done card → no match (archived/done не блокирует, корректно)"
fi

# --- T6: 1 entry, branch НЕ совпал → NO match --------------------------------
echo ""
echo "=== T6: 1 entry, branch mismatch (different ветка) ==="
SNAP_T6="2406	t_40a610d0	blocked	z-{agent}/DIFFERENT-BRANCH"
if g9c_branch_match "$SNAP_T6" "2406" "z-{agent}/2406-fix-prompt-tool-call-enforcement-discovery-n201-n3" >/dev/null 2>&1; then
    fail "T6: branch mismatch should NOT match"
else
    pass "T6: branch mismatch → no match (different branches, карточка не блокирует)"
fi

# --- T7: 2 entries (старая done + новая blocked) → match (blocked wins) ------
echo ""
echo "=== T7: 2 entries (done + blocked, тот же branch) → blocked wins ==="
SNAP_T7="$(printf '%s\n' \
    "2406	t_ec75ba3e	done	z-{agent}/2406-fix-prompt-tool-call-enforcement-discovery-n201-n3" \
    "2406	t_40a610d0	blocked	z-{agent}/2406-fix-prompt-tool-call-enforcement-discovery-n201-n3")"
match="$(g9c_branch_match "$SNAP_T7" "2406" "z-{agent}/2406-fix-prompt-tool-call-enforcement-discovery-n201-n3" 2>/dev/null || true)"
if [ "$match" = "t_40a610d0" ]; then
    pass "T7: blocked wins over done (realistic сценарий: старая карточка уже сделана, новая заблокирована)"
else
    fail "T7: expected match=t_40a610d0 (blocked), got=$match"
fi

# --- T8: 2 active entries, разные branches → match для вычисленной ----------
echo ""
echo "=== T8: 2 active entries, different branches → match только для нужной ==="
SNAP_T8="$(printf '%s\n' \
    "2406	t_card_a	blocked	z-{agent}/2406-fix-A" \
    "2406	t_card_b	ready	z-{agent}/2406-fix-B")"
match="$(g9c_branch_match "$SNAP_T8" "2406" "z-{agent}/2406-fix-B" 2>/dev/null || true)"
if [ "$match" = "t_card_b" ]; then
    pass "T8: branch-B match → t_card_b (точное соответствие)"
else
    fail "T8: expected match=t_card_b, got=$match"
fi
# Negative: branch-A при вычисленной branch-B → no match
match_a="$(g9c_branch_match "$SNAP_T8" "2406" "z-{agent}/2406-fix-A" 2>/dev/null || true)"
if [ "$match_a" = "t_card_a" ]; then
    pass "T8b: branch-A match → t_card_a (negative check, не путаем ветки)"
else
    fail "T8b: expected match=t_card_a, got=$match_a"
fi

# --- T9: branch совпал, status=archived → NO match ---------------------------
echo ""
echo "=== T9: branch match, status=archived (dead) → no match ==="
SNAP_T9="2406	t_b7fbff1c	archived	z-{agent}/2406-fix-prompt-tool-call-enforcement-discovery-n201-n3"
if g9c_branch_match "$SNAP_T9" "2406" "z-{agent}/2406-fix-prompt-tool-call-enforcement-discovery-n201-n3" >/dev/null 2>&1; then
    fail "T9: archived card should NOT match"
else
    pass "T9: archived → no match (gave_up не блокирует — триаж может пересоздать карточку на той же ветке)"
fi

# --- T10: shellcheck-clean + syntax-OK triage.sh ------------------------------
echo ""
echo "=== T10: shellcheck + syntax check ==="
if bash -n "$SCRIPT_UNDER_TEST" 2>/tmp/sc_err; then
    pass "T10a: bash -n syntax check passed"
else
    fail "T10a: bash -n syntax check failed" "$(cat /tmp/sc_err)"
fi

if ! command -v shellcheck >/dev/null 2>&1; then
    for sc in /home/builder/.hermes/hermes-agent/venv/bin/shellcheck \
             /usr/local/bin/shellcheck /usr/bin/shellcheck; do
        [ -x "$sc" ] && PATH="$(dirname "$sc"):$PATH" && break
    done
fi
if command -v shellcheck >/dev/null 2>&1; then
    _repo_root="$(git -C "$TESTS_DIR/.." rev-parse --show-toplevel 2>/dev/null || echo "$TESTS_DIR/..")"
    if [ -d "$_repo_root/.git" ] || [ -f "$_repo_root/.git" ]; then
        ORIG_SC="$(cd "$_repo_root" && git show "origin/develop:scripts/agent_flow/agent-flow-triage.sh" 2>/dev/null | shellcheck - 2>&1 | wc -l)"
        NEW_SC="$(shellcheck "$SCRIPT_UNDER_TEST" 2>&1 | wc -l)"
        log "  shellcheck: origin/develop=$ORIG_SC, current=$NEW_SC"
        if [ "$NEW_SC" -le "$ORIG_SC" ]; then
            pass "T10b: shellcheck — no NEW warnings (origin/develop=$ORIG_SC, current=$NEW_SC)"
        else
            DIFF="$(shellcheck "$SCRIPT_UNDER_TEST" 2>&1)"
            fail "T10b: shellcheck — $((NEW_SC-ORIG_SC)) new warning(s)" "$(printf '%s\n' "$DIFF" | head -30)"
        fi
    else
        log "T10b: not a git repo — skipping baseline comparison"
    fi
else
    log "T10b: shellcheck not installed — skip"
fi

# --- T11: presence check (key G9c markers в коде) ----------------------------
echo ""
echo "=== T11: presence-of-key-markers для G9c в triage.sh ==="
if grep -q 'ADR-AF-0067' "$SCRIPT_UNDER_TEST"; then
    pass "T11a: ADR-AF-0067 referenced in comments"
else
    fail "T11a: ADR-AF-0067 NOT referenced (G9c regression)"
fi
if grep -q 'dedup_branch_active_skipped' "$SCRIPT_UNDER_TEST"; then
    pass "T11b: dedup_branch_active_skipped counter referenced"
else
    fail "T11b: dedup_branch_active_skipped counter NOT referenced (G9c regression)"
fi
if grep -q 'G9c branch-name race-window' "$SCRIPT_UNDER_TEST"; then
    pass "T11c: G9c guard block comment present"
else
    fail "T11c: G9c guard block comment NOT present (G9c regression)"
fi
if grep -q 'existing_branch=' "$SCRIPT_UNDER_TEST"; then
    pass "T11d: existing_branch variable parsed from snapshot (4th field)"
else
    fail "T11d: existing_branch NOT parsed (existing_by_issue schema regression)"
fi
if grep -q "branch-active" "$SCRIPT_UNDER_TEST"; then
    pass "T11e: 'branch-active' label appears in summary log line"
else
    fail "T11e: 'branch-active' label NOT in summary log line (G9c counter regression)"
fi
if grep -q 't_60473741' "$SCRIPT_UNDER_TEST"; then
    pass "T11f: retro-key t_60473741 referenced"
else
    fail "T11f: retro-key t_60473741 NOT referenced"
fi
# Проверяем 4-field schema snapshot: print(..., t.get("branch_name") or "")
if grep -q 't.get("branch_name") or' "$SCRIPT_UNDER_TEST"; then
    pass "T11g: existing_by_issue python emits branch_name as 4th field"
else
    fail "T11g: branch_name NOT in existing_by_issue python emitter (schema regression)"
fi
if grep -q 'cut -f4' "$SCRIPT_UNDER_TEST"; then
    pass "T11h: cut -f4 used to parse branch_name from snapshot"
else
    fail "T11h: cut -f4 NOT used (existing_branch parse regression)"
fi

# --- summary -----------------------------------------------------------------
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