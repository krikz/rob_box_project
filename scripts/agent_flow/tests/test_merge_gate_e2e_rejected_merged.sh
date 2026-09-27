#!/bin/bash
# ============================================================================
# test_merge_gate_e2e_rejected_merged.sh — регресс-гард для merge-gate
# case 0.1c (retro 16.09 t_4a242e15, issue #2487, PR #2495).
#
# Сценарии:
#   R1. PR MERGED into develop, branch ЖИВА, issue OPEN с меткой
#       `e2e:rejected`, PR-body содержит "Refs:#N" (БЕЗ Closes/Fixes
#       keyword) → 0.1c auto-close: strip label + close issue.
#   R2. ... то же, но issue УЖЕ CLOSED → 0.1c skip (idempotent).
#   R3. ... то же, но branch УДАЛЕНА → defer to Q22-orphan path
#       (fallback skip).
#   R4. ... то же, но issue имеет whitelist `user-reopened-this` →
#       0.1c skip (ADR-0014 #1391).
#   R5. ... то же, но issue имеет `e2e-done` label → 0.1c skip
#       (e2e-done path is primary, retro 13.08 t_92ec94f3).
#   R6. ... то же, но issue имеет `no-e2e-required` label → 0.1c skip
#       (0.1a path takes over).
#
# Run:
#   bash scripts/agent_flow/tests/test_merge_gate_e2e_rejected_merged.sh
# ============================================================================
set -euo pipefail

RECENT_MERGED_AT="$(date -u -d '7 days ago' +%Y-%m-%dT%H:%M:%SZ)"

TEST_LIB_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/mock_env.sh
. "$TEST_LIB_DIR/lib/mock_env.sh"

# Title используется merge-gate'ом для derive branch = z-{agent}/<issue>-<slugify(title)>.
# Title ОБЯЗАН совпадать с тем что подставляется в ISSUE_LIST_JSON, иначе
# canonical PR_HEAD_<branch>_JSON lookup не подхватит fixture.
TITLE_FIXED_R1="e2e-rejected merged auto-close"
TITLE_FIXED_R2="e2e-rejected already closed"
TITLE_FIXED_R3="e2e-rejected branch deleted"
TITLE_FIXED_R4="e2e-rejected whitelist label"
TITLE_FIXED_R5="e2e-rejected but e2e-done primary"
TITLE_FIXED_R6="e2e-rejected but no-e2e-required primary"

# Helper: базовый fixture для case 0.1c (e2e:rejected + MERGED PR).
# $1=issue $2=pr $3=branch $4=pr_body $5=labels (space-separated)
# $6=issue_title (тот же что в ISSUE_LIST_JSON — slugify от него)
fixture_e2e_rejected_merged() {
    local issue="$1" pr="$2" branch="$3" pr_body="$4" labels="$5" title="$6"
    set_state ISSUE_LIST_JSON '[]'
    set_state PR_LIST_MERGED_JSON '[]'
    # Основной PR, который merge-gate видит через scan-all-prs.
    set_state "PR_HEAD_${branch}_JSON" "[{\"number\":${pr},\"state\":\"MERGED\",\"baseRefName\":\"develop\",\"mergedAt\":\"${RECENT_MERGED_AT}\",\"mergeable\":\"MERGEABLE\",\"mergeStateStatus\":\"CLEAN\",\"statusCheckRollup\":[{\"conclusion\":\"SUCCESS\"}],\"title\":\"fix #${issue} via e2e-rejected-merged path\",\"labels\":[]}]"
    # Issue в hermes list — обязательно содержит kanban-marker `kanban: t_<id>`
    # в body для того, чтобы merge-gate подхватил её в основной цикл
    # (см. agent-flow-merge-gate.sh line ~2818).
    set_state ISSUE_LIST_JSON "[{\"number\":${issue},\"title\":\"${title}\",\"labels\":[{\"name\":\"hermes\"}],\"body\":\"kanban: t_er${issue}\"}]"
    # Build labels JSON for ISSUE_<N>_LABELS_JSON (re-read в 0.1).
    local labels_json='['
    local first=1
    for lab in $labels; do
        if [ "$first" = "1" ]; then
            labels_json="${labels_json}{\"name\":\"${lab}\"}"
            first=0
        else
            labels_json="${labels_json},{\"name\":\"${lab}\"}"
        fi
    done
    labels_json="${labels_json}]"
    set_state "ISSUE_${issue}_LABELS_JSON" "{\"labels\":${labels_json}}"
    set_state "ISSUE_${issue}_STATE_JSON" '{"state":"OPEN"}'
    set_state "ISSUE_${issue}_COMMENTS_JSON" "{\"comments\":[{\"body\":\"kanban: t_er${issue}\"}]}"
    set_state "ISSUE_${issue}_COMMENTS_SINCE_JSON" '[]'
    set_state "ISSUE_${issue}_TIMELINE_JSON" '[]'
    set_state "PR_${pr}_COMMITS_JSON" '[]'
    set_state "PR_${pr}_VIEW_JSON" "{\"body\":\"${pr_body}\"}"
    set_state PR_LIST_ALL_OPEN_JSON '[]'
    set_state PR_FOLLOWUP_JSON "[{\"number\":${pr},\"state\":\"MERGED\",\"baseRefName\":\"develop\",\"mergedAt\":\"${RECENT_MERGED_AT}\",\"mergeable\":\"MERGEABLE\",\"mergeStateStatus\":\"CLEAN\",\"statusCheckRollup\":[{\"conclusion\":\"SUCCESS\"}],\"title\":\"fix #${issue} via e2e-rejected-merged path\",\"headRefName\":\"${branch}\",\"labels\":[],\"additions\":1,\"deletions\":0,\"commits\":[]}]"
    set_state RATE_LIMIT_JSON '{"resources":{"core":{"remaining":5000}}}'
    set_state KANBAN_LIST_JSON '[]'
    set_state "BRANCH_PRESENT_${branch}" 1
}

slugify_branch() {  # $1=issue $2=title
    local issue="$1" title="$2" slug
    slug="$(printf '%s' "$title" | tr '[:upper:]' '[:lower:]' \
        | sed -E 's/[^a-z0-9]+/-/g; s/^-+|-+$//g; s/-{2,}/-/g' \
        | cut -c1-50)"
    printf 'z-{agent}/%s-%s' "$issue" "$slug"
}

# ============================================================================
# R1. e2e:rejected + MERGED + Refs (no Closes keyword) → 0.1c auto-close.
# ============================================================================
test_R1_e2e_rejected_merged_refs_closes_issue() {
    new_test
    local issue=2487 pr=2495 title="${TITLE_FIXED_R1}"
    local branch
    branch="$(slugify_branch "$issue" "$title")"
    # PR-body uses Refs:#N (NOT Closes) — exactly the bug scenario.
    fixture_e2e_rejected_merged "$issue" "$pr" "$branch" \
        "Refs: #${issue} (no closes/fixes keyword in body — squash-merge loses it anyway)" \
        "hermes e2e:rejected" "$title"

    run_merge_gate
    local journal
    # Retro 16.09 t_4a242e15: tests grep и для gh-вызовов (journal) И для
    # log() merge-gate (stderr → stderr.log). log() пишет в stderr
    # (LOG_PREFIX redirect, agent-flow-merge-gate.sh:269), поэтому чистый
    # `$GH_JOURNAL` не содержит log-сообщений — gh mock пишет туда ТОЛЬКО
    # свой собственный journal, а log() merge-gate идёт в stderr. Склеиваем
    # оба потока в один grep-input.
    journal="$(cat "$GH_JOURNAL" "$TEST_TMP/stderr.log" 2>/dev/null)"

    # 0.1c must close the issue.
    local close_calls
    close_calls="$(printf '%s' "$journal" | grep -c "gh issue close ${issue} --reason completed" || true)"
    assert_eq "1" "$close_calls" "0.1c closes issue via gh issue close (retro 16.09 t_4a242e15)"

    # Audit-comment with the unique 0.1c marker.
    local audit
    audit="$(printf '%s' "$journal" | grep -Fc "🔁 e2e:rejected auto-close (retro 16.09 t_4a242e15," || true)"
    assert_eq "1" "$audit" "0.1c audit comment published (unique marker, not confused with 0.1b fallback)"

    # Must NOT have triggered 0.1b fallback (different marker).
    local fb_audit
    fb_audit="$(printf '%s' "$journal" | grep -Fc "🔁 fallback auto-close (ADR-AF-0063 §4.1)" || true)"
    assert_eq "0" "$fb_audit" "no 0.1b fallback audit (this is 0.1c, not 0.1b)"

    # Strip e2e:rejected label via pr_label_sweep_after_merge.
    local label_strip
    label_strip="$(printf '%s' "$journal" | grep -c "gh pr edit ${pr} .*remove-label e2e:rejected" || true)"
    assert_ge "$label_strip" "1" "0.1c strips e2e:rejected from PR (ретро 01.09 t_fd604461)"

    # State flips to CLOSED.
    local state_now
    state_now="$(grep -E "^ISSUE_${issue}_STATE_JSON=" "$GH_STATE" | sed "s/^ISSUE_${issue}_STATE_JSON=//")"
    assert_contains '"CLOSED"' "$state_now" "issue state flipped to CLOSED"
}

# ============================================================================
# R2. Issue already CLOSED → 0.1c skip (idempotent guard).
# ============================================================================
test_R2_already_closed_idempotent_skip() {
    new_test
    local issue=2488 pr=2496 title="${TITLE_FIXED_R2}"
    local branch
    branch="$(slugify_branch "$issue" "$title")"
    fixture_e2e_rejected_merged "$issue" "$pr" "$branch" \
        "Refs: #${issue}" "hermes e2e:rejected" "$title"
    set_state "ISSUE_${issue}_STATE_JSON" '{"state":"CLOSED"}'

    run_merge_gate
    local journal
    # Retro 16.09 t_4a242e15: tests grep и для gh-вызовов (journal) И для
    # log() merge-gate (stderr → stderr.log). log() пишет в stderr
    # (LOG_PREFIX redirect, agent-flow-merge-gate.sh:269), поэтому чистый
    # `$GH_JOURNAL` не содержит log-сообщений — gh mock пишет туда ТОЛЬКО
    # свой собственный journal, а log() merge-gate идёт в stderr. Склеиваем
    # оба потока в один grep-input.
    journal="$(cat "$GH_JOURNAL" "$TEST_TMP/stderr.log" 2>/dev/null)"

    # No close call (issue already CLOSED → outer guard skips 0.1c).
    local close_calls
    close_calls="$(printf '%s' "$journal" | grep -c "gh issue close ${issue} --reason completed" || true)"
    assert_eq "0" "$close_calls" "no close when issue already CLOSED (outer guard)"
}

# ============================================================================
# R3. Branch deleted on remote → defer to Q22-orphan path (skip 0.1c).
# ============================================================================
test_R3_branch_deleted_defer_to_q22() {
    new_test
    local issue=2489 pr=2497 title="${TITLE_FIXED_R3}"
    local branch
    branch="$(slugify_branch "$issue" "$title")"
    fixture_e2e_rejected_merged "$issue" "$pr" "$branch" \
        "Refs: #${issue}" "hermes e2e:rejected" "$title"
    set_state "BRANCH_PRESENT_${branch}" 0

    run_merge_gate
    local journal
    # Retro 16.09 t_4a242e15: tests grep и для gh-вызовов (journal) И для
    # log() merge-gate (stderr → stderr.log). log() пишет в stderr
    # (LOG_PREFIX redirect, agent-flow-merge-gate.sh:269), поэтому чистый
    # `$GH_JOURNAL` не содержит log-сообщений — gh mock пишет туда ТОЛЬКО
    # свой собственный journal, а log() merge-gate идёт в stderr. Склеиваем
    # оба потока в один grep-input.
    journal="$(cat "$GH_JOURNAL" "$TEST_TMP/stderr.log" 2>/dev/null)"

    # 0.1c must log defer-to-Q22 (NOT 0.1c audit-comment).
    local defer_log
    defer_log="$(printf '%s' "$journal" | grep -Fc "e2e-rejected+Merged path (retro 16.09 t_4a242e15) — branch ${branch} deleted on remote, defer to Q22-orphan path" || true)"
    assert_eq "1" "$defer_log" "logs defer-to-Q22 when branch deleted on remote (0.1c skips, falls to Q22)"

    # 0.1c audit-comment must NOT have fired (we deferred).
    local r01c_audit
    r01c_audit="$(printf '%s' "$journal" | grep -Fc "🔁 e2e:rejected auto-close (retro 16.09 t_4a242e15," || true)"
    assert_eq "0" "$r01c_audit" "no 0.1c audit-comment when branch deleted"

    # Close IS happening — but via Q22-orphan path, NOT 0.1c (см. existing
    # behavior of 0.1b's branch-deleted deferral — issue #2123). The test
    # verifies the deferral works correctly: Q22 takes over and closes.
    local close_calls
    close_calls="$(printf '%s' "$journal" | grep -c "gh issue close ${issue} --reason completed" || true)"
    assert_ge "$close_calls" "1" "Q22-orphan path closes issue (deferred from 0.1c)"

    # The Q22 close uses a distinct log marker.
    local q22_log
    q22_log="$(printf '%s' "$journal" | grep -c "MERGED без e2e-done, ветка ${branch} удалена" || true)"
    assert_eq "1" "$q22_log" "close happens via Q22 user-merge path (not 0.1c)"
}

# ============================================================================
# R4. Whitelist label `user-reopened-this` → 0.1c skip.
# ============================================================================
test_R4_whitelist_label_blocks_0_1c() {
    new_test
    local issue=2490 pr=2498 title="${TITLE_FIXED_R4}"
    local branch
    branch="$(slugify_branch "$issue" "$title")"
    fixture_e2e_rejected_merged "$issue" "$pr" "$branch" \
        "Refs: #${issue}" "hermes e2e:rejected user-reopened-this" "$title"

    run_merge_gate
    local journal
    # Retro 16.09 t_4a242e15: tests grep и для gh-вызовов (journal) И для
    # log() merge-gate (stderr → stderr.log). log() пишет в stderr
    # (LOG_PREFIX redirect, agent-flow-merge-gate.sh:269), поэтому чистый
    # `$GH_JOURNAL` не содержит log-сообщений — gh mock пишет туда ТОЛЬКО
    # свой собственный journal, а log() merge-gate идёт в stderr. Склеиваем
    # оба потока в один grep-input.
    journal="$(cat "$GH_JOURNAL" "$TEST_TMP/stderr.log" 2>/dev/null)"

    # Must log whitelist skip.
    local skip_log
    skip_log="$(printf '%s' "$journal" | grep -Fc "e2e-rejected+Merged path (retro 16.09 t_4a242e15), whitelist user-reopened-this → skip auto-close" || true)"
    assert_eq "1" "$skip_log" "user-reopened-this whitelist blocks 0.1c"

    # No close.
    local close_calls
    close_calls="$(printf '%s' "$journal" | grep -c "gh issue close ${issue} --reason completed" || true)"
    assert_eq "0" "$close_calls" "no close when whitelist active"
}

# ============================================================================
# R5. e2e-done label present → 0.1c skip (e2e-done path primary).
# ============================================================================
test_R5_e2e_done_label_skips_0_1c() {
    new_test
    local issue=2491 pr=2499 title="${TITLE_FIXED_R5}"
    local branch
    branch="$(slugify_branch "$issue" "$title")"
    # Both labels present — e2e-done must win (reconcile path).
    fixture_e2e_rejected_merged "$issue" "$pr" "$branch" \
        "Closes: #${issue}" "hermes e2e-done e2e:rejected" "$title"

    run_merge_gate
    local journal
    # Retro 16.09 t_4a242e15: tests grep и для gh-вызовов (journal) И для
    # log() merge-gate (stderr → stderr.log). log() пишет в stderr
    # (LOG_PREFIX redirect, agent-flow-merge-gate.sh:269), поэтому чистый
    # `$GH_JOURNAL` не содержит log-сообщений — gh mock пишет туда ТОЛЬКО
    # свой собственный journal, а log() merge-gate идёт в stderr. Склеиваем
    # оба потока в один grep-input.
    journal="$(cat "$GH_JOURNAL" "$TEST_TMP/stderr.log" 2>/dev/null)"

    # e2e-done path closes (not 0.1c).
    local close_calls
    close_calls="$(printf '%s' "$journal" | grep -c "gh issue close ${issue} --reason completed" || true)"
    assert_eq "1" "$close_calls" "e2e-done path closes issue (reconcile path)"

    # 0.1c audit must NOT have fired.
    local r01c_audit
    r01c_audit="$(printf '%s' "$journal" | grep -Fc "🔁 e2e:rejected auto-close (retro 16.09 t_4a242e15," || true)"
    assert_eq "0" "$r01c_audit" "no 0.1c audit when e2e-done path closes"
}

# ============================================================================
# R6. no-e2e-required label → 0.1c skip (0.1a path primary).
# ============================================================================
test_R6_no_e2e_required_label_skips_0_1c() {
    new_test
    local issue=2492 pr=2500 title="${TITLE_FIXED_R6}"
    local branch
    branch="$(slugify_branch "$issue" "$title")"
    fixture_e2e_rejected_merged "$issue" "$pr" "$branch" \
        "Closes: #${issue}" "hermes e2e:rejected no-e2e-required" "$title"

    run_merge_gate
    local journal
    # Retro 16.09 t_4a242e15: tests grep и для gh-вызовов (journal) И для
    # log() merge-gate (stderr → stderr.log). log() пишет в stderr
    # (LOG_PREFIX redirect, agent-flow-merge-gate.sh:269), поэтому чистый
    # `$GH_JOURNAL` не содержит log-сообщений — gh mock пишет туда ТОЛЬКО
    # свой собственный journal, а log() merge-gate идёт в stderr. Склеиваем
    # оба потока в один grep-input.
    journal="$(cat "$GH_JOURNAL" "$TEST_TMP/stderr.log" 2>/dev/null)"

    # 0.1a path closes (no-e2e-required).
    local close_calls
    close_calls="$(printf '%s' "$journal" | grep -c "gh issue close ${issue} --reason completed" || true)"
    assert_eq "1" "$close_calls" "no-e2e-required path closes issue (0.1a)"

    # 0.1c audit must NOT have fired.
    local r01c_audit
    r01c_audit="$(printf '%s' "$journal" | grep -Fc "🔁 e2e:rejected auto-close (retro 16.09 t_4a242e15," || true)"
    assert_eq "0" "$r01c_audit" "no 0.1c audit when 0.1a path closes"
}

# Run all.
run_test "R1_e2e_rejected_merged_refs_closes_issue" test_R1_e2e_rejected_merged_refs_closes_issue
run_test "R2_already_closed_idempotent_skip" test_R2_already_closed_idempotent_skip
run_test "R3_branch_deleted_defer_to_q22" test_R3_branch_deleted_defer_to_q22
run_test "R4_whitelist_label_blocks_0_1c" test_R4_whitelist_label_blocks_0_1c
run_test "R5_e2e_done_label_skips_0_1c" test_R5_e2e_done_label_skips_0_1c
run_test "R6_no_e2e_required_label_skips_0_1c" test_R6_no_e2e_required_label_skips_0_1c
summary