#!/bin/bash
# ============================================================================
# test_merge_gate_fallback_keyword.sh — ADR-AF-0063 §4.1 (issue #2123)
#
# Сценарии:
#   F1. PR MERGED into develop, branch ЖИВА, issue OPEN без process-меток,
#       PR-body содержит "closes #N" → fallback auto-close.
#   F2. ... то же, но PR-body содержит только "#N" reference (НЕ keyword) →
#       no auto-close (ADR-AF-0063 §6 by-design limitation).
#   F3. ... то же, но branch удалена с remote → defer к Q22-orphan path
#       (не fallback); проверяем что fallback НЕ вызвал close.
#   F4. ... то же, но issue имеет user-reopened-this метку → fallback skip
#       (whitelist guard, ADR-0014 #1391).
#   F5. ... то же, но issue имеет e2e-done label → fallback skip (е2е путь
#       главнее).
#   F6. ... то же, но issue уже CLOSED → fallback skip (idempotent).
#
# Run:
#   bash scripts/agent_flow/tests/test_merge_gate_fallback_keyword.sh
# ============================================================================
set -euo pipefail

RECENT_MERGED_AT="$(date -u -d '7 days ago' +%Y-%m-%dT%H:%M:%SZ)"

TEST_LIB_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/mock_env.sh
. "$TEST_LIB_DIR/lib/mock_env.sh"

slugify_branch() {  # $1=issue $2=title
    local issue="$1" title="$2" slug
    slug="$(printf '%s' "$title" | tr '[:upper:]' '[:lower:]' \
        | sed -E 's/[^a-z0-9]+/-/g; s/^-+|-+$//g; s/-{2,}/-/g' \
        | cut -c1-50)"
    printf 'z-{agent}/%s-%s' "$issue" "$slug"
}

# Title используется merge-gate'ом для derive branch = z-{agent}/<issue>-<slugify(title)>.
# Title ОБЯЗАН совпадать с тем что подставляется в ISSUE_LIST_JSON, иначе
# canonical PR_HEAD_<branch>_JSON lookup не подхватит fixture (см. mock_env.sh line 540-557).
TITLE_FIXED_F1="fallback closes test"
TITLE_FIXED_F2="reference only test"
TITLE_FIXED_F3="branch deleted defer"
TITLE_FIXED_F4="whitelist label test"
TITLE_FIXED_F5="e2e done present"
TITLE_FIXED_F6="already closed test"

# Helper: базовый fixture для ADR-AF-0063 §4.1 (branch-жива + PR-body keyword).
# $1=issue $2=pr $3=branch $4=pr_body $5=labels (space-separated, or "" for no labels)
# $6=issue_title (тот же что в ISSUE_LIST_JSON — slugify от него)
fixture_fallback() {
    local issue="$1" pr="$2" branch="$3" pr_body="$4" labels="$5" title="$6"
    # hermes-issues пуст, retro-path тоже пуст — fallback работает в своём окне.
    set_state ISSUE_LIST_JSON '[]'
    set_state PR_LIST_MERGED_JSON '[]'
    # Основной PR, который merge-gate видит через scan-all-prs (PR_HEAD_<branch>).
    set_state "PR_HEAD_${branch}_JSON" "[{\"number\":${pr},\"state\":\"MERGED\",\"baseRefName\":\"develop\",\"mergedAt\":\"${RECENT_MERGED_AT}\",\"mergeable\":\"MERGEABLE\",\"mergeStateStatus\":\"CLEAN\",\"statusCheckRollup\":[{\"conclusion\":\"SUCCESS\"}],\"title\":\"fix #${issue} via fallback\",\"labels\":[]}]"
    # Issue в hermes list (чтобы merge-gate его подобрал).
    if [ -z "$labels" ]; then
        set_state ISSUE_LIST_JSON "[{\"number\":${issue},\"title\":\"${title}\",\"labels\":[{\"name\":\"hermes\"}],\"body\":\"kanban: t_fb${issue}\"}]"
        set_state "ISSUE_${issue}_LABELS_JSON" "{\"labels\":[{\"name\":\"hermes\"}]}"
    else
        # Build labels JSON
        local labels_json='['
        for lab in $labels; do
            if [ "$labels_json" = "[" ]; then
                labels_json="${labels_json}{\"name\":\"${lab}\"}"
            else
                labels_json="${labels_json},{\"name\":\"${lab}\"}"
            fi
        done
        labels_json="${labels_json}]"
        set_state ISSUE_LIST_JSON "[{\"number\":${issue},\"title\":\"${title}\",\"labels\":[{\"name\":\"hermes\"}],\"body\":\"kanban: t_fb${issue}\"}]"
        set_state "ISSUE_${issue}_LABELS_JSON" "{\"labels\":${labels_json}}"
    fi
    set_state "ISSUE_${issue}_STATE_JSON" '{"state":"OPEN"}'
    # Merge-gate main-cycle требует kanban-marker `kanban: t_<id>` в issue
    # comments (см. agent-flow-merge-gate.sh line 2818-2827). Без него
    # issue skip'ается до PR-lookup — fallback никогда не сработает.
    set_state "ISSUE_${issue}_COMMENTS_JSON" "{\"comments\":[{\"body\":\"kanban: t_fb${issue}\\n\"}]}"
    set_state "ISSUE_${issue}_COMMENTS_SINCE_JSON" '[]'
    set_state "ISSUE_${issue}_TIMELINE_JSON" '[]'
    set_state "PR_${pr}_COMMITS_JSON" '[]'
    set_state "PR_${pr}_VIEW_JSON" "{\"body\":\"${pr_body}\"}"
    set_state PR_LIST_ALL_OPEN_JSON '[]'
    # Mock: gh pr list --state all --search "<N> in:title" берёт из
    # PR_FOLLOWUP_JSON (см. mock_env.sh line 562-565, fallback branch lookup).
    # Наш PR — канонический z-{agent}/<n>-slug, попадёт в headRefName-match.
    set_state PR_FOLLOWUP_JSON "[{\"number\":${pr},\"state\":\"MERGED\",\"baseRefName\":\"develop\",\"mergedAt\":\"${RECENT_MERGED_AT}\",\"mergeable\":\"MERGEABLE\",\"mergeStateStatus\":\"CLEAN\",\"statusCheckRollup\":[{\"conclusion\":\"SUCCESS\"}],\"title\":\"fix #${issue} via fallback\",\"headRefName\":\"${branch}\",\"labels\":[],\"additions\":1,\"deletions\":0,\"commits\":[]}]"
    set_state RATE_LIMIT_JSON '{"resources":{"core":{"remaining":5000}}}'
    # Канбан-карточка: archive_merged_card ищет её в списке. Пустой список — fallback skip-cleanup OK.
    set_state KANBAN_LIST_JSON '[]'
    # Branch жива по умолчанию; тест может переопределить.
    set_state "BRANCH_PRESENT_${branch}" 1
}

# ============================================================================
# F1. PR-body keyword + branch жива + no process labels → fallback auto-close.
# ============================================================================
test_F1_fallback_keyword_closes_issue() {
    new_test
    local issue=2123 pr=2200 title="${TITLE_FIXED_F1}"
    # Branch — derived от issue title (как делает merge-gate main-cycle).
    local slug
    slug="$(printf '%s' "$title" | tr '[:upper:]' '[:lower:]' \
        | sed -E 's/[^a-z0-9]+/-/g; s/^-+|-+$//g; s/-{2,}/-/g' \
        | cut -c1-50)"
    local branch="z-{agent}/${issue}-${slug}"
    fixture_fallback "$issue" "$pr" "$branch" "closes #${issue}" "" "$title"

    run_merge_gate
    local journal
    journal="$(cat "$GH_JOURNAL")"

    # Fallback close вызван.
    local close_calls
    close_calls="$(printf '%s\n' "$journal" | grep -c "gh issue close ${issue} --reason completed" || true)"
    assert_eq "1" "$close_calls" "fallback path closes issue via 'gh issue close' (ADR-AF-0063 §4.1)"

    # Audit comment with ADR-AF-0063 §4.1 marker.
    local audit
    audit="$(printf '%s\n' "$journal" | grep -c "🔁 fallback auto-close (ADR-AF-0063 §4.1)" || true)"
    assert_eq "1" "$audit" "fallback audit comment published"

    # State flips to CLOSED.
    local state_now
    state_now="$(grep -E "^ISSUE_${issue}_STATE_JSON=" "$GH_STATE" | sed "s/^ISSUE_${issue}_STATE_JSON=//")"
    assert_contains '"CLOSED"' "$state_now" "issue state flipped to CLOSED"
}

# ============================================================================
# F2. PR-body только reference (#N без keyword) → НЕ close (ADR-AF-0063 §6).
# ============================================================================
test_F2_reference_only_no_close() {
    new_test
    local issue=2124 pr=2201 title="${TITLE_FIXED_F2}"
    local slug
    slug="$(printf '%s' "$title" | tr '[:upper:]' '[:lower:]' \
        | sed -E 's/[^a-z0-9]+/-/g; s/^-+|-+$//g; s/-{2,}/-/g' \
        | cut -c1-50)"
    local branch="z-{agent}/${issue}-${slug}"
    fixture_fallback "$issue" "$pr" "$branch" "fix related to #${issue} discussion" "" "$title"

    run_merge_gate
    local journal
    journal="$(cat "$GH_JOURNAL")"

    # Fallback НЕ должен закрывать.
    local close_calls
    close_calls="$(printf '%s\n' "$journal" | grep -c "gh issue close ${issue} --reason completed" || true)"
    assert_eq "0" "$close_calls" "reference-only PR (no closes/fixes/resolves keyword) → NO auto-close (ADR-AF-0063 §6)"

    # Нет audit-коммента fallback.
    local audit
    audit="$(printf '%s\n' "$journal" | grep -c "🔁 fallback auto-close" || true)"
    assert_eq "0" "$audit" "no fallback audit comment for reference-only PR"

    # Issue осталась OPEN.
    local state_now
    state_now="$(grep -E "^ISSUE_${issue}_STATE_JSON=" "$GH_STATE" | sed "s/^ISSUE_${issue}_STATE_JSON=//")"
    assert_contains '"OPEN"' "$state_now" "issue stays OPEN for reference-only PR"
}

# ============================================================================
# F3. Branch удалена с remote → defer to Q22-orphan path (fallback skip).
# ============================================================================
test_F3_branch_deleted_defer_to_q22() {
    new_test
    local issue=2125 pr=2202 title="${TITLE_FIXED_F3}"
    local slug
    slug="$(printf '%s' "$title" | tr '[:upper:]' '[:lower:]' \
        | sed -E 's/[^a-z0-9]+/-/g; s/^-+|-+$//g; s/-{2,}/-/g' \
        | cut -c1-50)"
    local branch="z-{agent}/${issue}-${slug}"
    fixture_fallback "$issue" "$pr" "$branch" "closes #${issue}" "" "$title"
    # Branch УДАЛЕНА → fallback skip.
    set_state "BRANCH_PRESENT_${branch}" 0

    run_merge_gate
    local journal
    journal="$(cat "$GH_JOURNAL")"

    # Fallback не должен закрывать (defer к Q22).
    local fb_log
    fb_log="$(printf '%s\n' "$journal" | grep -c "fallback path (ADR-AF-0063 §4.1) — branch ${branch} deleted on remote, defer to Q22-orphan path" || true)"
    assert_eq "1" "$fb_log" "fallback logs defer-to-Q22 message when branch deleted"

    # Audit-коммент fallback НЕ публикуется (т.к. skip).
    local fb_audit
    fb_audit="$(printf '%s\n' "$journal" | grep -c "🔁 fallback auto-close" || true)"
    assert_eq "0" "$fb_audit" "no fallback audit comment when branch deleted"
}

# ============================================================================
# F4. user-reopened-this whitelist → fallback skip (ADR-0014 #1391 supplement).
# ============================================================================
test_F4_whitelist_label_blocks_fallback() {
    new_test
    local issue=2126 pr=2203 title="${TITLE_FIXED_F4}"
    local slug
    slug="$(printf '%s' "$title" | tr '[:upper:]' '[:lower:]' \
        | sed -E 's/[^a-z0-9]+/-/g; s/^-+|-+$//g; s/-{2,}/-/g' \
        | cut -c1-50)"
    local branch="z-{agent}/${issue}-${slug}"
    fixture_fallback "$issue" "$pr" "$branch" "closes #${issue}" "hermes user-reopened-this" "$title"

    run_merge_gate
    local journal
    journal="$(cat "$GH_JOURNAL")"

    # Whitelist skip log.
    local skip_log
    skip_log="$(printf '%s\n' "$journal" | grep -c "fallback path (ADR-AF-0063 §4.1), whitelist user-reopened-this → skip auto-close" || true)"
    assert_eq "1" "$skip_log" "user-reopened-this whitelist blocks fallback"

    # No close.
    local close_calls
    close_calls="$(printf '%s\n' "$journal" | grep -c "gh issue close ${issue} --reason completed" || true)"
    assert_eq "0" "$close_calls" "no close when whitelist active"
}

# ============================================================================
# F5. Issue имеет e2e-done → fallback skip (е2е-путь главнее).
# ============================================================================
test_F5_e2e_done_label_skips_fallback() {
    new_test
    local issue=2127 pr=2204 title="${TITLE_FIXED_F5}"
    local slug
    slug="$(printf '%s' "$title" | tr '[:upper:]' '[:lower:]' \
        | sed -E 's/[^a-z0-9]+/-/g; s/^-+|-+$//g; s/-{2,}/-/g' \
        | cut -c1-50)"
    local branch="z-{agent}/${issue}-${slug}"
    fixture_fallback "$issue" "$pr" "$branch" "closes #${issue}" "hermes e2e-done" "$title"

    run_merge_gate
    local journal
    journal="$(cat "$GH_JOURNAL")"

    # e2e-done путь закрывает, не fallback.
    local close_calls
    close_calls="$(printf '%s\n' "$journal" | grep -c "gh issue close ${issue} --reason completed" || true)"
    assert_eq "1" "$close_calls" "e2e-done path closes issue"

    # Audit-коммент e2e-done, не fallback.
    local fb_audit
    fb_audit="$(printf '%s\n' "$journal" | grep -c "🔁 fallback auto-close" || true)"
    assert_eq "0" "$fb_audit" "no fallback audit when e2e-done path closes"
}

# ============================================================================
# F6. Issue уже CLOSED → fallback idempotent skip.
# ============================================================================
test_F6_already_closed_idempotent_skip() {
    new_test
    local issue=2128 pr=2205 title="${TITLE_FIXED_F6}"
    local slug
    slug="$(printf '%s' "$title" | tr '[:upper:]' '[:lower:]' \
        | sed -E 's/[^a-z0-9]+/-/g; s/^-+|-+$//g; s/-{2,}/-/g' \
        | cut -c1-50)"
    local branch="z-{agent}/${issue}-${slug}"
    fixture_fallback "$issue" "$pr" "$branch" "closes #${issue}" "" "$title"
    set_state "ISSUE_${issue}_STATE_JSON" '{"state":"CLOSED"}'

    run_merge_gate
    local journal
    journal="$(cat "$GH_JOURNAL")"

    # No close вызов (issue уже closed → gh issue close call идёт в fallback, но case "0.2 CLOSED" срабатывает первым? нет, fallback 0.1b срабатывает ПЕРВЫМ.
    # Spec: fallback triggers only when state=OPEN. CLOSED state = fallback skip via outer guard.
    # Но _issue_state = 'CLOSED' значит внешний if [ "$_issue_state" = "OPEN" ] = false → fallback НЕ запускается.
    local close_calls
    close_calls="$(printf '%s\n' "$journal" | grep -c "gh issue close ${issue} --reason completed" || true)"
    assert_eq "0" "$close_calls" "no close call when issue already CLOSED (outer guard)"

    # No fallback audit.
    local fb_audit
    fb_audit="$(printf '%s\n' "$journal" | grep -c "🔁 fallback auto-close" || true)"
    assert_eq "0" "$fb_audit" "no fallback audit when issue already CLOSED"
}

# Run all.
run_test "F1_fallback_keyword_closes_issue" test_F1_fallback_keyword_closes_issue
run_test "F2_reference_only_no_close" test_F2_reference_only_no_close
run_test "F3_branch_deleted_defer_to_q22" test_F3_branch_deleted_defer_to_q22
run_test "F4_whitelist_label_blocks_fallback" test_F4_whitelist_label_blocks_fallback
run_test "F5_e2e_done_label_skips_fallback" test_F5_e2e_done_label_skips_fallback
run_test "F6_already_closed_idempotent_skip" test_F6_already_closed_idempotent_skip
summary
