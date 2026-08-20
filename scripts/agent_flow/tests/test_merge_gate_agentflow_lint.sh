#!/bin/bash
# ============================================================================
# test_merge_gate_agentflow_lint.sh — ретро 13.08 t_de63be1f
#
# Два регресс-гарда для конвейерных фиксов:
#
# J. PR с title-префиксом `fix(agent-flow` / `fix(agent_flow` → lint
#    (needs-review на PR, НЕ needs-e2e). Раньше detect_pr_kind классифицировал
#    их как functional → фиксы конвейера (#1189/#1190) уходили в e2e-очередь
#    и застревали: ротация жгла build+deploy на заведомо непрофильный
#    сценарий, Шифу не видел PR в ревью.
#
# K. Issue с needs-review + needs-e2e одновременно (#942) → needs-e2e снят,
#    issue вне e2e-ротации (skip). Раньше merge-gate скипал такую issue как
#    «triage not finished» (нет kanban-маркера) и метки не чистил.
#
# Run:
#   bash scripts/agent_flow/tests/test_merge_gate_agentflow_lint.sh
# ============================================================================
set -euo pipefail

TEST_LIB_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/mock_env.sh
. "$TEST_LIB_DIR/lib/mock_env.sh"

# ---------------------------------------------------------------------------
# J. fix(agent-flow ...) → lint → needs-review на PR, needs-e2e НЕ ставится.
# ---------------------------------------------------------------------------
test_J_agentflow_fix_is_lint() {
    new_test
    local issue=3101 pr=3102
    local title="fix(agent-flow e2e): python-пайпы неубиваемые — try/except BrokenPipeError"
    local slug
    slug="$(printf '%s' "$title" | tr '[:upper:]' '[:lower:]' \
        | sed -E 's/[^a-z0-9]+/-/g; s/^-+|-+$//g; s/-{2,}/-/g' \
        | cut -c1-40)"
    local branch="z-{agent}/${issue}-${slug}"
    set_state ISSUE_LIST_JSON "[{\"number\":${issue},\"title\":\"${title}\",\"labels\":[{\"name\":\"hermes\"}],\"body\":\"kanban: t_dead${issue}\"}]"
    set_state "ISSUE_${issue}_LABELS_JSON" '{"labels":[{"name":"hermes"}]}'
    set_state "ISSUE_${issue}_STATE_JSON" '{"state":"OPEN"}'
    set_state "ISSUE_${issue}_COMMENTS_JSON" "{\"comments\":[{\"body\":\"kanban: t_dead${issue}\\n\"}]}"
    set_state "ISSUE_${issue}_COMMENTS_SINCE_JSON" '[]'
    set_state "ISSUE_${issue}_TIMELINE_JSON" '[]'
    set_state "PR_HEAD_${branch}_JSON" "[{\"number\":${pr},\"state\":\"OPEN\",\"baseRefName\":\"develop\",\"mergeable\":\"MERGEABLE\",\"mergeStateStatus\":\"CLEAN\",\"statusCheckRollup\":[{\"conclusion\":\"SUCCESS\"}],\"title\":\"${title}\",\"labels\":[{\"name\":\"agent:devops\"}],\"additions\":50,\"commits\":[{},{}]}]"
    set_state "PR_${pr}_COMMITS_JSON" '[]'
    set_state PR_LIST_ALL_OPEN_JSON '[]'
    set_state PR_FOLLOWUP_JSON '[]'
    set_state RATE_LIMIT_JSON '{"resources":{"core":{"remaining":5000}}}'
    set_state "BRANCH_PRESENT_${branch}" 1

    run_merge_gate

    local journal
    journal="$(cat "$GH_JOURNAL")"

    local needs_review_calls
    needs_review_calls="$(printf '%s\n' "$journal" | grep -c "gh pr edit ${pr} --repo .* --add-label needs-review" || true)"
    assert_eq "1" "$needs_review_calls" "fix(agent-flow) PR → needs-review set on PR"

    local needs_e2e_calls
    needs_e2e_calls="$(printf '%s\n' "$journal" | grep -c "gh issue edit ${issue} --add-label needs-e2e" || true)"
    assert_eq "0" "$needs_e2e_calls" "fix(agent-flow) PR → needs-e2e NOT set on issue"
}

# ---------------------------------------------------------------------------
# K. Issue с needs-review + needs-e2e (#942) → needs-e2e снят, skip.
# ---------------------------------------------------------------------------
test_K_conflicting_labels_cleared() {
    new_test
    local issue=3103 pr=3104
    local title="fix #${issue} demo"
    local slug
    slug="$(printf '%s' "$title" | tr '[:upper:]' '[:lower:]' \
        | sed -E 's/[^a-z0-9]+/-/g; s/^-+|-+$//g; s/-{2,}/-/g' \
        | cut -c1-40)"
    local branch="z-{agent}/${issue}-${slug}"
    # Issue уже с needs-review + needs-e2e (конфликт, #942) и БЕЗ kanban-маркера.
    set_state ISSUE_LIST_JSON "[{\"number\":${issue},\"title\":\"${title}\",\"labels\":[{\"name\":\"hermes\"},{\"name\":\"needs-e2e\"},{\"name\":\"needs-review\"}],\"body\":\"\"}]"
    set_state "ISSUE_${issue}_LABELS_JSON" '{"labels":[{"name":"hermes"},{"name":"needs-e2e"},{"name":"needs-review"}]}'
    set_state "ISSUE_${issue}_STATE_JSON" '{"state":"OPEN"}'
    set_state "ISSUE_${issue}_COMMENTS_JSON" '{"comments":[]}'
    set_state "ISSUE_${issue}_COMMENTS_SINCE_JSON" '[]'
    set_state "ISSUE_${issue}_TIMELINE_JSON" '[]'
    set_state PR_LIST_ALL_OPEN_JSON '[]'
    set_state PR_FOLLOWUP_JSON '[]'
    set_state RATE_LIMIT_JSON '{"resources":{"core":{"remaining":5000}}}'

    run_merge_gate

    local journal
    journal="$(cat "$GH_JOURNAL")"

    local remove_calls
    remove_calls="$(printf '%s\n' "$journal" | grep -c "gh issue edit ${issue} --remove-label needs-e2e" || true)"
    assert_eq "1" "$remove_calls" "конфликт needs-review+needs-e2e → needs-e2e removed"

    # И главное: merge-gate НЕ должен повторно навесить needs-e2e (иначе
    # конфликт вернётся следующим тиком). При needs-review на issue — skip.
    local add_calls
    add_calls="$(printf '%s\n' "$journal" | grep -c "gh issue edit ${issue} --add-label needs-e2e" || true)"
    assert_eq "0" "$add_calls" "issue с needs-review → needs-e2e НЕ навешивается повторно"
}

# ===========================================================================
# Run all tests.
# ===========================================================================
run_test "J. fix(agent-flow) → lint (needs-review, no e2e)" test_J_agentflow_fix_is_lint
run_test "K. needs-review+needs-e2e конфликт → needs-e2e снят" test_K_conflicting_labels_cleared

summary
