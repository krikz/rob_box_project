#!/bin/bash
# ============================================================================
# test_merge_gate_conflicting.sh — ретро 12.08 t_618208c0 acceptance tests
#
# СЕРАЯ ЗОНА merge-gate: open PR с hermes-issue, mergeable=CONFLICTING (или
# mergeStateStatus=DIRTY) БЕЗ карточки-воркера — раньше merge-gate молча
# пропускал такой PR (needs-e2e ставится только при clean; конфликт-карточка
# e2e-process создавалась только на round-merge, а PR в очередь не попадал).
# Кейс: PR #1165 / issue #1160 (карточка t_43d02968 done, конфликт-карточки
# нет, PR висит в лимбе).
#
# Теперь (ретро t_618208c0): scan-all-prs для CONFLICTING/DIRTY PR без
# карточки:
#   - коммент на САМ PR (gh pr comment, dedup 24h): «конфликт с develop,
#     нужен rebase»
#   - создание конфликт-карточки (идемпотентно: title-lookup по PR/ветке,
#     requeue done/archived, unblock blocked, create если нет), assignee из
#     метки issue (agent:backend → backend)
#
# Scenarios:
#   A. CONFLICTING PR + issue БЕЗ kanban-marker (нет карточки) → конфликт-
#      карточка создаётся + PR comment постится.
#   B. mergeStateStatus=DIRTY (mergeable=UNKNOWN, async GitHub) → то же
#      поведение (DIRTY = конфликт, не UNSTABLE).
#   C. Идемпотентность: конфликт-карточка УЖЕ есть (done) → requeue, дубликат
#      НЕ создаётся; PR comment дедуплицируется (24h).
#   D. UNSTABLE PR без карточки → карточка НЕ создаётся, PR comment НЕ
#      постится (регрессия: UNSTABLE остаётся на main-cycle).
#
# Run:
#   bash scripts/agent_flow/tests/test_merge_gate_conflicting.sh
# ============================================================================
set -euo pipefail

TEST_LIB_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/mock_env.sh
. "$TEST_LIB_DIR/lib/mock_env.sh"

# ---------------------------------------------------------------------------
# Helper: PR fixture для scan-all-prs (без kanban-marker в issue = нет карточки)
# $1=pr $2=issue $3=branch $4=mergeable $5=merge_state $6=title
# ---------------------------------------------------------------------------
fixture_conflict_pr() {
    local pr="$1" issue="$2" branch="$3" mergeable="$4" merge_state="$5" title="$6"
    # Основной цикл (hermes issues) пропускаем — тестируем scan-all-prs.
    set_state ISSUE_LIST_JSON '[]'
    set_state PR_LIST_ALL_OPEN_JSON "[{\"number\":${pr},\"title\":\"${title}\",\"headRefName\":\"${branch}\",\"mergeable\":\"${mergeable}\",\"mergeStateStatus\":\"${merge_state}\",\"labels\":[]}]"
    # Issue БЕЗ kanban-marker (кандидаты пусты → task_id остаётся пустым)
    set_state "ISSUE_${issue}_COMMENTS_JSON" '{"comments":[]}'
    set_state "ISSUE_${issue}_LABELS_JSON" '{"labels":[{"name":"agent:backend"}]}'
    # PR-коммент dedup: по умолчанию пусто → коммент постится
    set_state "ISSUE_${pr}_COMMENTS_SINCE_JSON" '[]'
    # scan-all-prs stale-branch guard: нет MERGED PR на этой ветке
    set_state "PR_MERGED_HEAD_${branch}_JSON" ''
    set_state PR_LIST_MERGED_JSON '[]'
    set_state PR_FOLLOWUP_JSON '[]'
    set_state RATE_LIMIT_JSON '{"resources":{"core":{"remaining":5000}}}'
}

# ===========================================================================
# A. CONFLICTING PR без карточки → конфликт-карточка + PR comment
# ===========================================================================
test_A_conflicting_no_card_creates_card_and_comments_pr() {
    new_test
    local branch="z-{agent}/4201-fix-conflict-demo"
    fixture_conflict_pr 4201 4201 "$branch" CONFLICTING DIRTY "fix #4201 conflict demo"
    set_state KANBAN_LIST_JSON '[]'

    run_merge_gate

    assert_contains "gh pr comment 4201 --repo" "$(journal_grep 'gh pr comment 4201')" \
        "A: PR comment должен поститься на сам PR"
    assert_contains "hermes kanban --board robbox create" "$(journal_grep 'hermes kanban --board robbox create')" \
        "A: конфликт-карточка должна создаваться"
    assert_contains "--assignee backend" "$(cat "$GH_JOURNAL")" \
        "A: assignee из метки agent:backend"
    assert_contains "🔀 rebase PR #4201" "$(cat "$GH_JOURNAL")" \
        "A: title конфликт-карточки содержит PR-номер"
}

# ===========================================================================
# B. DIRTY merge_state (mergeable=UNKNOWN) → то же, что CONFLICTING
# ===========================================================================
test_B_dirty_state_treated_as_conflict() {
    new_test
    local branch="z-{agent}/4202-fix-conflict-dirty"
    fixture_conflict_pr 4202 4202 "$branch" UNKNOWN DIRTY "fix #4202 dirty demo"
    set_state KANBAN_LIST_JSON '[]'

    run_merge_gate

    assert_contains "gh pr comment 4202 --repo" "$(journal_grep 'gh pr comment 4202')" \
        "B: DIRTY → PR comment постится (это конфликт, не UNSTABLE)"
    assert_contains "hermes kanban --board robbox create" "$(journal_grep 'hermes kanban --board robbox create')" \
        "B: DIRTY → конфликт-карточка создаётся"
    assert_contains "🔀 rebase PR #4202" "$(cat "$GH_JOURNAL")" \
        "B: title содержит PR-номер"
    # UNSTABLE-reminder НЕ должен появляться (текст «CI UNSTABLE detected»)
    assert_not_contains "CI UNSTABLE detected" "$(journal_grep 'hermes kanban --board robbox comment')" \
        "B: DIRTY не должен получать UNSTABLE-reminder"
}

# ===========================================================================
# C. Идемпотентность: конфликт-карточка уже есть (done) → requeue, без дубля;
#    PR comment уже есть (24h) → dedup, без повтора
# ===========================================================================
test_C_existing_conflict_card_requeued_and_comment_dedup() {
    new_test
    local branch="z-{agent}/4203-fix-conflict-again"
    fixture_conflict_pr 4203 4203 "$branch" CONFLICTING DIRTY "fix #4203 conflict again"
    # Конфликт-карточка УЖЕ существует (done)
    set_state KANBAN_LIST_JSON "[{\"id\":\"t_conflict4203\",\"title\":\"🔀 rebase PR #4203 (\`${branch}\`) на develop — конфликт (issue 4203)\",\"status\":\"done\"}]"
    # PR comment уже был за 24h
    set_state "ISSUE_4203_COMMENTS_SINCE_JSON" '[{"body":"🔀 **merge conflict** (merge-gate, ретро 12.08 t_618208c0): PR #4203 ..."}]'

    run_merge_gate

    assert_not_contains "hermes kanban --board robbox create" "$(journal_grep 'hermes kanban --board robbox create')" \
        "C: дубликат конфликт-карточки НЕ создаётся"
    assert_contains "hermes kanban --board robbox requeue t_conflict4203" "$(journal_grep 'hermes kanban --board robbox requeue')" \
        "C: существующая done-карточка requeue'ится"
    assert_not_contains "gh pr comment 4203" "$(journal_grep 'gh pr comment 4203')" \
        "C: PR comment дедуплицируется (уже был за 24h)"
}

# ===========================================================================
# D. UNSTABLE PR без карточки → НЕ создаём конфликт-карточку, НЕ комментим PR
#    (регрессия: UNSTABLE остаётся на main-cycle — needs-e2e поставится при clean)
# ===========================================================================
test_D_unstable_without_card_no_new_behavior() {
    new_test
    local branch="z-{agent}/4204-fix-unstable"
    fixture_conflict_pr 4204 4204 "$branch" MERGEABLE UNSTABLE "fix #4204 unstable demo"
    set_state KANBAN_LIST_JSON '[]'

    run_merge_gate

    assert_not_contains "gh pr comment 4204" "$(journal_grep 'gh pr comment 4204')" \
        "D: UNSTABLE не комментит сам PR"
    assert_not_contains "hermes kanban --board robbox create" "$(journal_grep 'hermes kanban --board robbox create')" \
        "D: UNSTABLE не создаёт конфликт-карточку"
}

# ---------------------------------------------------------------------------
# Registry
# ---------------------------------------------------------------------------
run_test "A_conflicting_no_card_creates_card_and_comments_pr" test_A_conflicting_no_card_creates_card_and_comments_pr
run_test "B_dirty_state_treated_as_conflict" test_B_dirty_state_treated_as_conflict
run_test "C_existing_conflict_card_requeued_and_comment_dedup" test_C_existing_conflict_card_requeued_and_comment_dedup
run_test "D_unstable_without_card_no_new_behavior" test_D_unstable_without_card_no_new_behavior

summary
