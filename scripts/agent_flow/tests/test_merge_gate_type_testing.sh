#!/bin/bash
# ============================================================================
# test_merge_gate_type_testing.sh — ретро 22.08 t_944df2c5 (issue #1506)
#
# Проверяет type:testing gate в signal 3 (return-path из e2e:rejected):
#   Для type:testing задач sibling e2e-done PR (инфра-фикс) НЕ валидирует
#   acceptance исходной задачи. Generic signal 3 вернул бы issue в needs-e2e
#   и «разблокировал» её инфра-прогоном → «красивый PASS вместо честного
#   FAIL» (ADR-0018). Вместо этого: needs-review на data-only PR + warning,
#   issue остаётся e2e:rejected до ручной верификации Шифу.
#
# Scenarios:
#   L1. issue type:testing + e2e:rejected + data-only PR e2e:rejected +
#       sibling PR e2e-done → needs-review на data-only PR (НЕ на sibling),
#       e2e:rejected с issue НЕ снят, warning-комментарий есть.
#   L2. регрессия: та же конфигурация, но БЕЗ type:testing → старое поведение
#       signal 3 (e2e:rejected снят с issue, needs-review на sibling) —
#       покрыто test_merge_gate_e2e_done_data_only_pr.sh, здесь smoke.
#
# Run:
#   bash scripts/agent_flow/tests/test_merge_gate_type_testing.sh
# ============================================================================
set -euo pipefail

TEST_LIB_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/mock_env.sh
. "$TEST_LIB_DIR/lib/mock_env.sh"

slugify_branch() {  # $1=issue $2=title
    local issue="$1" title="$2" slug
    slug="$(printf '%s' "$title" | tr '[:upper:]' '[:lower:]' \
        | sed -E 's/[^a-z0-9]+/-/g; s/^-+|-+$//g; s/-{2,}/-/g' \
        | cut -c1-40)"
    printf 'z-{agent}/%s-%s' "$issue" "$slug"
}

# ---------------------------------------------------------------------------
# Fixture: issue #1506 c type:testing + e2e:rejected + needs-e2e, data-only PR
# #1507 (e2e:rejected) + sibling PR #1508 (infra-fix, e2e-done, OPEN).
# ---------------------------------------------------------------------------
fixture_1506_type_testing_with_infra_fix() {
    local issue=1506 pr_data=1507 pr_infra=1508
    local title='task(voice) e2e-валидация голосовых фич'
    local branch_canon
    branch_canon="$(slugify_branch "$issue" "$title")"

    set_state ISSUE_LIST_JSON "[{\"number\":${issue},\"title\":\"${title}\",\"labels\":[{\"name\":\"hermes\"},{\"name\":\"needs-e2e\"},{\"name\":\"e2e:rejected\"},{\"name\":\"type:testing\"}],\"body\":\"kanban: t_944df2c5\"}]"
    set_state "ISSUE_${issue}_LABELS_JSON" '{"labels":[{"name":"hermes"},{"name":"needs-e2e"},{"name":"e2e:rejected"},{"name":"type:testing"}]}'
    set_state "ISSUE_${issue}_STATE_JSON" '{"state":"OPEN"}'
    set_state "ISSUE_${issue}_COMMENTS_JSON" '{"comments":[{"body":"kanban: t_944df2c5\n"}]}'
    set_state "ISSUE_${issue}_COMMENTS_SINCE_JSON" '[]'
    set_state "ISSUE_${issue}_TIMELINE_JSON" "[{\"event\":\"labeled\",\"label\":{\"name\":\"e2e:rejected\"},\"created_at\":\"2026-08-21T22:12:00Z\"}]"

    # Канонический PR (data-only): ветка z-{agent}/1506-task-voice-...
    # метки e2e:rejected, MERGEABLE/CLEAN.
    set_state "PR_HEAD_${branch_canon}_JSON" "[{\"number\":${pr_data},\"state\":\"OPEN\",\"baseRefName\":\"develop\",\"mergeable\":\"MERGEABLE\",\"mergeStateStatus\":\"CLEAN\",\"statusCheckRollup\":[{\"conclusion\":\"SUCCESS\"}],\"title\":\"e2e #1506: voice_core_suite + acceptance (data-only)\",\"labels\":[{\"name\":\"e2e:rejected\"}],\"commits\":[],\"additions\":0,\"deletions\":0}]"
    set_state "PR_${pr_data}_COMMITS_JSON" '[]'

    # Sibling OPEN PR (infra-fix): ветка НЕ каноническая, метка e2e-done.
    set_state "PR_FOLLOWUP_JSON" "[{\"number\":${pr_infra},\"headRefName\":\"z-devops/t_a8fb9137-e2e-upload-artifact-question-mark-legacy-collect\",\"mergeStateStatus\":\"CLEAN\",\"labels\":[{\"name\":\"e2e-done\"}]}]"

    set_state PR_LIST_ALL_OPEN_JSON '[]'
    set_state PR_LIST_MERGED_JSON '[]'
    set_state RATE_LIMIT_JSON '{"resources":{"core":{"remaining":5000}}'
    set_state "KANBAN_LIST_JSON" "[{\"id\":\"t_944df2c5\",\"status\":\"running\"}]"
}

# ===========================================================================
# L1. type:testing + sibling e2e-done → needs-review на data-only PR,
#     e2e:rejected с issue НЕ снят, warning-комментарий.
# ===========================================================================
test_L1_type_testing_sibling_not_validate_acceptance() {
    new_test
    fixture_1506_type_testing_with_infra_fix

    run_merge_gate
    local journal
    journal="$(cat "$GH_JOURNAL")"

    # 1) needs-review поставлен на data-only PR #1507.
    local add_review_data
    add_review_data="$(printf '%s\n' "$journal" | grep -cE "gh pr edit 1507 .*--add-label needs-review" || true)"
    assert_eq "1" "$add_review_data" "needs-review added to data-only PR #1507" || return 1

    # 2) needs-review НЕ поставлен на sibling PR #1508 (в отличие от generic signal 3).
    local add_review_sib
    add_review_sib="$(printf '%s\n' "$journal" | grep -cE "gh pr edit 1508 .*--add-label needs-review" || true)"
    assert_eq "0" "$add_review_sib" "needs-review NOT added to sibling PR #1508" || return 1

    # 3) e2e:rejected снят с data-only PR #1507.
    local rm_rej_pr
    rm_rej_pr="$(printf '%s\n' "$journal" | grep -cE "gh pr edit 1507 .*--remove-label e2e:rejected" || true)"
    assert_eq "1" "$rm_rej_pr" "e2e:rejected removed from data-only PR #1507" || return 1

    # 4) e2e:rejected НЕ снят с issue #1506 (остаётся до ручной верификации).
    local rm_rej_issue
    rm_rej_issue="$(printf '%s\n' "$journal" | grep -c "gh issue edit 1506 --remove-label e2e:rejected" || true)"
    assert_eq "0" "$rm_rej_issue" "e2e:rejected NOT removed from issue #1506" || return 1

    # 5) needs-e2e НЕ поставлен на issue (не возвращаем в ротацию).
    local add_needs
    add_needs="$(printf '%s\n' "$journal" | grep -c "gh issue edit 1506 --add-label needs-e2e" || true)"
    assert_eq "0" "$add_needs" "needs-e2e NOT added to issue #1506" || return 1

    # 6) warning-комментарий постится (type:testing + НЕ валидирует acceptance).
    local cmt_count
    cmt_count="$(printf '%s\n' "$journal" | grep -c 'type:testing.*НЕ валидирует acceptance' || true)"
    assert_eq "1" "$cmt_count" "type:testing warning comment posted" || return 1

    # 7) issue НЕ закрыта.
    local close_calls
    close_calls="$(printf '%s\n' "$journal" | grep -c 'gh issue close' || true)"
    assert_eq "0" "$close_calls" "issue stays OPEN" || return 1
}

# ===========================================================================
# L2. регрессия: БЕЗ type:testing → старое поведение signal 3
#     (e2e:rejected снят с issue, needs-e2e поставлен, needs-review на sibling).
#     Фикстура без type:testing в метках.
# ===========================================================================
fixture_1506_no_type_testing_with_infra_fix() {
    local issue=1506 pr_data=1507 pr_infra=1508
    local title='task(voice) e2e-валидация голосовых фич'
    local branch_canon
    branch_canon="$(slugify_branch "$issue" "$title")"

    set_state ISSUE_LIST_JSON "[{\"number\":${issue},\"title\":\"${title}\",\"labels\":[{\"name\":\"hermes\"},{\"name\":\"needs-e2e\"},{\"name\":\"e2e:rejected\"}],\"body\":\"kanban: t_944df2c5\"}]"
    set_state "ISSUE_${issue}_LABELS_JSON" '{"labels":[{"name":"hermes"},{"name":"needs-e2e"},{"name":"e2e:rejected"}]}'
    set_state "ISSUE_${issue}_STATE_JSON" '{"state":"OPEN"}'
    set_state "ISSUE_${issue}_COMMENTS_JSON" '{"comments":[{"body":"kanban: t_944df2c5\n"}]}'
    set_state "ISSUE_${issue}_COMMENTS_SINCE_JSON" '[]'
    set_state "ISSUE_${issue}_TIMELINE_JSON" "[{\"event\":\"labeled\",\"label\":{\"name\":\"e2e:rejected\"},\"created_at\":\"2026-08-21T22:12:00Z\"}]"
    set_state "PR_HEAD_${branch_canon}_JSON" "[{\"number\":${pr_data},\"state\":\"OPEN\",\"baseRefName\":\"develop\",\"mergeable\":\"MERGEABLE\",\"mergeStateStatus\":\"CLEAN\",\"statusCheckRollup\":[{\"conclusion\":\"SUCCESS\"}],\"title\":\"e2e #1506: voice_core_suite + acceptance (data-only)\",\"labels\":[{\"name\":\"e2e:rejected\"}],\"commits\":[],\"additions\":0,\"deletions\":0}]"
    set_state "PR_${pr_data}_COMMITS_JSON" '[]'
    set_state "PR_FOLLOWUP_JSON" "[{\"number\":${pr_infra},\"headRefName\":\"z-devops/t_a8fb9137-e2e-upload-artifact-question-mark-legacy-collect\",\"mergeStateStatus\":\"CLEAN\",\"labels\":[{\"name\":\"e2e-done\"}]}]"
    set_state PR_LIST_ALL_OPEN_JSON '[]'
    set_state PR_LIST_MERGED_JSON '[]'
    set_state RATE_LIMIT_JSON '{"resources":{"core":{"remaining":5000}}'
    set_state "KANBAN_LIST_JSON" "[{\"id\":\"t_944df2c5\",\"status\":\"running\"}]"
}

test_L2_no_type_testing_keeps_old_signal3() {
    new_test
    fixture_1506_no_type_testing_with_infra_fix

    run_merge_gate
    local journal
    journal="$(cat "$GH_JOURNAL")"

    # e2e:rejected снят с issue (generic signal 3).
    local rm_rejected
    rm_rejected="$(printf '%s\n' "$journal" | grep -c "gh issue edit 1506 --remove-label e2e:rejected" || true)"
    assert_eq "1" "$rm_rejected" "L2 regression: e2e:rejected removed from issue" || return 1

    # needs-e2e поставлен на issue.
    local add_needs
    add_needs="$(printf '%s\n' "$journal" | grep -c "gh issue edit 1506 --add-label needs-e2e" || true)"
    assert_eq "1" "$add_needs" "L2 regression: needs-e2e added to issue" || return 1

    # needs-review на sibling PR #1508 (generic signal 3).
    local add_review_sib
    add_review_sib="$(printf '%s\n' "$journal" | grep -cE "gh pr edit 1508 .*--add-label needs-review" || true)"
    assert_eq "1" "$add_review_sib" "L2 regression: needs-review added to sibling PR #1508" || return 1
}

# --- run --------------------------------------------------------------------
run_test "L1_type_testing_sibling_not_validate_acceptance" test_L1_type_testing_sibling_not_validate_acceptance
run_test "L2_no_type_testing_keeps_old_signal3"             test_L2_no_type_testing_keeps_old_signal3
summary
