#!/bin/bash
# ============================================================================
# test_merge_gate_rebase_card_contract_drift.sh
#
# Ретро t_527e1231 → process-fix t_58c69473, блок B («decompose-on-rebase»).
#
# ПРОБЛЕМА (кейс t_002aae48, 01.09): PR #1857 = MERGEABLE + UNSTABLE, у него
# НЕТ hermes-issue → основной цикл (с его classification unit_lint →
# diagnostic-карточка) до него не доходит, а scan-all-prs создаёт карточку
# «🔀 rebase PR #1857 … на develop» с assignee=default. Но падал **Unit Tests**
# — то есть contract drift ВНУТРИ PR, а rebase бессилен. Default-воркер без
# скиллов повисел 1.6ч и ничего не сделал.
#
# ФИКС: в scan-all-prs для UNSTABLE PR (НЕ CONFLICTING):
#   1. Классифицировать failing checks по имени:
#      contract_drift   — Unit Tests / Build / Lint / pytest / mypy / ruff …
#      rebase_candidate — integration / e2e / deploy / docker / smoke …
#   2. contract_drift → assignee **backend** (вместо default), если метки issue
#      не дали конкретного владельца.
#   3. В body карточки ОБЯЗАТЕЛЬНО вставить блок `## Contract-drift pre-check`
#      со ссылкой на CI run и списком failing jobs.
#   4. rebase_candidate / unknown → поведение как раньше (assignee=default).
#
# Сценарии:
#   B1. UNSTABLE + failing «Unit Tests (ROS2 Humble)» → assignee=backend
#       + блок `## Contract-drift pre-check` + имя job'а в body.
#   B2. UNSTABLE + failing «Integration Tests» → assignee=default (как раньше).
#   B3. CONFLICTING PR → contract-drift-блок НЕ добавляется (это реальный
#       конфликт, rebase — правильный ответ).
#   B4. UNSTABLE + failing «Unit Tests», но у issue стоит agent:devops →
#       assignee=devops (явная метка сильнее эвристики).
#   B5. UNSTABLE, rollup недоступен (unknown) → assignee=default, без блока.
#
# Run:
#   bash scripts/agent_flow/tests/test_merge_gate_rebase_card_contract_drift.sh
# ============================================================================
set -euo pipefail

TEST_LIB_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/mock_env.sh
. "$TEST_LIB_DIR/lib/mock_env.sh"

# ---------------------------------------------------------------------------
# Fixture: один OPEN PR без карточки и без hermes-issue → scan-all-prs создаёт
# rebase/recovery-карточку сам (ровно путь t_002aae48).
# $1=pr $2=head $3=mergeable $4=mergeStateStatus
# ---------------------------------------------------------------------------
fixture_open_pr() {  # $1=pr $2=head $3=mergeable $4=merge_state
    local pr="$1" head="$2" mergeable="$3" merge_state="$4"
    set_state ISSUE_LIST_JSON '[]'
    set_state PR_LIST_MERGED_JSON '[]'
    set_state PR_FOLLOWUP_JSON '[]'
    set_state PR_LIST_ALL_OPEN_REST_JSON '[]'
    set_state KANBAN_LIST_JSON '[]'
    set_state RATE_LIMIT_JSON '{"resources":{"core":{"remaining":5000}}}'
    set_state PR_LIST_ALL_OPEN_JSON \
        "[{\"number\":${pr},\"headRefName\":\"${head}\",\"mergeable\":\"${mergeable}\",\"mergeStateStatus\":\"${merge_state}\",\"title\":\"docs(adr-0043): provider-chain sync contract\",\"labels\":[],\"baseRefName\":\"develop\",\"state\":\"OPEN\",\"isDraft\":false}]"
    set_state "PR_${pr}_VIEW_JSON" \
        "{\"state\":\"OPEN\",\"headRefName\":\"${head}\",\"number\":${pr}}"
}

create_line_for_pr() {  # $1=pr → весь журнал (body карточки многострочный,
                        # поэтому grep построчно не годится: title и --assignee
                        # оказываются в разных строках одной create-команды).
                        # В каждом тесте ровно один PR → журнала достаточно.
    cat "$GH_JOURNAL" 2>/dev/null || true
}

# ===========================================================================
# B1. UNSTABLE + Unit Tests FAILURE → assignee=backend + contract-drift блок.
# ===========================================================================
test_B1_unit_tests_failure_routes_to_backend() {
    new_test
    local pr=1857 head='z-architect/t_5d93c7b1-adr-0040-provider-sync'
    fixture_open_pr "$pr" "$head" MERGEABLE UNSTABLE
    set_state "PR_${pr}_ROLLUP_JSON" \
        '{"statusCheckRollup":[{"name":"Unit Tests (ROS2 Humble)","conclusion":"FAILURE","detailsUrl":"https://github.com/krikz/rob_box_project/actions/runs/33503553426/job/99842198694"},{"name":"Shell Scripts","conclusion":"SUCCESS","detailsUrl":""}]}'

    run_merge_gate
    local created
    created="$(create_line_for_pr "$pr")"
    assert_contains "assignee backend" "$created" \
        "UNSTABLE + Unit Tests failure → assignee=backend (B1)"
    assert_contains "Contract-drift pre-check" "$created" \
        "body contains Contract-drift pre-check block (B1)"
    assert_contains "Unit Tests (ROS2 Humble)" "$created" \
        "body lists the failing job name (B1)"
    assert_contains "99842198694" "$created" \
        "body links the failing CI run/job (B1)"
}

# ===========================================================================
# B2. UNSTABLE + только Integration Tests FAILURE → assignee=default (как раньше),
#     contract-drift блока нет (develop-фиксы реально могут починить).
# ===========================================================================
test_B2_integration_failure_keeps_default() {
    new_test
    local pr=1861 head='z-agent/t_1234abcd-integration-flake'
    fixture_open_pr "$pr" "$head" MERGEABLE UNSTABLE
    set_state "PR_${pr}_ROLLUP_JSON" \
        '{"statusCheckRollup":[{"name":"Integration Tests","conclusion":"FAILURE","detailsUrl":"https://example/job/1"}]}'

    run_merge_gate
    local created
    created="$(create_line_for_pr "$pr")"
    assert_contains "assignee default" "$created" \
        "UNSTABLE + integration failure → assignee stays default (B2)"
    assert_not_contains "Contract-drift pre-check" "$created" \
        "no contract-drift block for integration-only failures (B2)"
}

# ===========================================================================
# B3. CONFLICTING PR → contract-drift блок НЕ добавляем (rebase корректен).
# ===========================================================================
test_B3_conflicting_no_contract_drift_block() {
    new_test
    local pr=1862 head='z-agent/t_5678beef-real-conflict'
    fixture_open_pr "$pr" "$head" CONFLICTING DIRTY
    set_state "PR_${pr}_ROLLUP_JSON" \
        '{"statusCheckRollup":[{"name":"Unit Tests (ROS2 Humble)","conclusion":"FAILURE","detailsUrl":"https://example/job/2"}]}'

    run_merge_gate
    local created
    created="$(create_line_for_pr "$pr")"
    assert_not_contains "Contract-drift pre-check" "$created" \
        "CONFLICTING PR → no contract-drift block (B3)"
}

# ===========================================================================
# B4. Явная метка agent:devops сильнее эвристики contract_drift.
# ===========================================================================
test_B4_explicit_label_wins() {
    new_test
    local pr=1863 head='z-devops/t_99991111-ci-fix'
    fixture_open_pr "$pr" "$head" MERGEABLE UNSTABLE
    # scan-all-prs берёт issue_num из title (#N) — задаём его и метку.
    set_state PR_LIST_ALL_OPEN_JSON \
        "[{\"number\":${pr},\"headRefName\":\"${head}\",\"mergeable\":\"MERGEABLE\",\"mergeStateStatus\":\"UNSTABLE\",\"title\":\"ci: fix workflow #1799\",\"labels\":[],\"baseRefName\":\"develop\",\"state\":\"OPEN\",\"isDraft\":false}]"
    set_state ISSUE_1799_LABELS_JSON '{"labels":[{"name":"agent:devops"},{"name":"ci"}]}'
    set_state ISSUE_1799_STATE_JSON '{"state":"OPEN"}'
    set_state "PR_${pr}_ROLLUP_JSON" \
        '{"statusCheckRollup":[{"name":"Lint Summary","conclusion":"FAILURE","detailsUrl":"https://example/job/3"}]}'

    run_merge_gate
    local created
    created="$(create_line_for_pr "$pr")"
    assert_contains "assignee devops" "$created" \
        "explicit agent:devops label wins over contract-drift heuristic (B4)"
    assert_contains "Contract-drift pre-check" "$created" \
        "contract-drift block still added for lint failure (B4)"
}

# ===========================================================================
# B5. Rollup недоступен → unknown → assignee=default, блока нет (fail-open).
# ===========================================================================
test_B5_unknown_rollup_fail_open() {
    new_test
    local pr=1864 head='z-agent/t_aaaabbbb-no-rollup'
    fixture_open_pr "$pr" "$head" MERGEABLE UNSTABLE
    # PR_<n>_ROLLUP_JSON намеренно НЕ задан.

    run_merge_gate
    local created
    created="$(create_line_for_pr "$pr")"
    assert_contains "assignee default" "$created" \
        "unknown classification → assignee=default (B5)"
    assert_not_contains "Contract-drift pre-check" "$created" \
        "unknown classification → no contract-drift block (B5)"
}

run_test "B1 Unit Tests failure → backend + block"  test_B1_unit_tests_failure_routes_to_backend
run_test "B2 integration failure → default"         test_B2_integration_failure_keeps_default
run_test "B3 CONFLICTING → no block"                test_B3_conflicting_no_contract_drift_block
run_test "B4 explicit label wins"                   test_B4_explicit_label_wins
run_test "B5 unknown rollup → fail-open"            test_B5_unknown_rollup_fail_open

summary
