#!/bin/bash
# ============================================================================
# test_merge_gate_retro_card_archive.sh — ретро-архив done-карточек
# по MERGED PR (ретро 14.08 t_36c9ac4e)
#
# ПРОБЛЕМА (надзор 13.08 22:20Z): ретро/recovery-карточки (маркер retro-key,
# БЕЗ issue-линка) после merge PR остаются done НАВСЕГДА. Archive-путь
# основного цикла (стр. 774-783) находит карточку ТОЛЬКО по issue-номеру в
# body (kanban list --json -> t.get("issue")), которого в hermes v0.20.0
# НЕТ ВООБЩЕ → card_id пуст → archive не выполняется никогда. Даже
# t_41fec39e (issue #1217 есть, kanban-маркер есть) не архивировалась: Q22
# orphan-close делал continue ДО archive-блока.
#
# Сценарии нового прохода retro-card-archive:
#   A. done ретро-карточка, id которой (t_<hex>) есть в head-ветке MERGED PR
#      (z-devops/t_fe266643-... / PR #1221) → archive.
#   B. done recovery-карточка, title которой упоминает смерженный PR/branch
#      ("🔀 rebase PR #1212 (`z-devops/t_da3e0bd5-...`)") → archive.
#   C. done issue-карточка, чей СОБСТВЕННЫЙ branch_name (exact) == head MERGED
#      PR, но issue уже CLOSED (t_41fec39e / PR #1220) → archive.
#   D. done карточка, чей branch_name == head CLOSED-но-НЕ-merged PR
#      (t_cc9cc56d / PR #1155 CLOSED) → НЕ архивируется (exact-match guard).
#   E. карточка НЕ done (running/ready) с подходящим MERGED PR → НЕ архивируется.
#   F. done карточка без совпадения с MERGED PR → НЕ архивируется.
#
# Run:
#   bash scripts/agent_flow/tests/test_merge_gate_retro_card_archive.sh
# ============================================================================
set -euo pipefail

TEST_LIB_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/mock_env.sh
. "$TEST_LIB_DIR/lib/mock_env.sh"

# ---------------------------------------------------------------------------
# Helper: базовый fixture для прохода retro-card-archive.
# $1=pr_number, $2=head_branch, $3=mergedAt
# Нет hermes-issues, ретро-PR БЕЗ ссылок на issues (иначе retro-path
# вмешается), карточки приходят через KANBAN_LIST_JSON.
# ---------------------------------------------------------------------------
fixture_archive() {  # $1=pr $2=head $3=mergedAt
    local pr="$1" head="$2" merged_at="$3"
    set_state ISSUE_LIST_JSON '[]'
    # PR без #N-ссылок на чужие issues (ретро-стиль): retro-path молчит.
    set_state PR_LIST_MERGED_JSON "[{\"number\":${pr},\"title\":\"fix(agent-flow t_retro): archive pass\",\"body\":\"\",\"headRefName\":\"${head}\",\"mergedAt\":\"${merged_at}\"}]"
    set_state PR_LIST_ALL_OPEN_JSON '[]'
    set_state PR_FOLLOWUP_JSON '[]'
    set_state RATE_LIMIT_JSON '{"resources":{"core":{"remaining":5000}}}'
}

# ===========================================================================
# A. done ретро-карточка: id в head-ветке MERGED PR (t_<hex>) → archive.
# ===========================================================================
test_A_retro_card_archived_by_branch_task_id() {
    new_test
    local pr=1221 head='z-devops/t_fe266643-empty-round-sweep-order'
    fixture_archive "$pr" "$head" '2026-08-13T21:25:25Z'
    set_state KANBAN_LIST_JSON "[{\"id\":\"t_fe266643\",\"status\":\"done\",\"branch_name\":\"\",\"title\":\"ретро: пустая round-104\"}]"

    run_merge_gate
    local journal
    journal="$(cat "$GH_JOURNAL")"

    local archive_calls
    archive_calls="$(printf '%s\n' "$journal" | grep -c 'kanban --board robbox archive t_fe266643' || true)"
    assert_eq "1" "$archive_calls" "retro done card archived by branch t_<hex> (A)"
}

# ===========================================================================
# B. done recovery-карточка: title упоминает MERGED PR/branch → archive.
# ===========================================================================
test_B_recovery_card_archived_by_title_ref() {
    new_test
    local pr=1212 head='z-devops/t_da3e0bd5-e2e-timeout-cancel-dedup'
    fixture_archive "$pr" "$head" '2026-08-13T21:56:47Z'
    set_state KANBAN_LIST_JSON "[{\"id\":\"t_e9e09694\",\"status\":\"done\",\"branch_name\":\"\",\"title\":\"🔀 rebase PR #1212 (z-devops/t_da3e0bd5-e2e-timeout-cancel-dedup) на develop — конфликт/CI\"}]"

    run_merge_gate
    local journal
    journal="$(cat "$GH_JOURNAL")"

    local archive_calls
    archive_calls="$(printf '%s\n' "$journal" | grep -c 'kanban --board robbox archive t_e9e09694' || true)"
    assert_eq "1" "$archive_calls" "recovery done card archived by title ref (B)"
}

# ===========================================================================
# C. done issue-карточка: СОБСТВЕННЫЙ branch_name (exact) == head MERGED PR,
#    issue уже CLOSED (t_41fec39e кейс) → archive.
# ===========================================================================
test_C_issue_card_archived_by_own_branch() {
    new_test
    local pr=1220 head='z-{agent}/1217-e2e-40-deepseek'
    fixture_archive "$pr" "$head" '2026-08-13T21:24:57Z'
    set_state KANBAN_LIST_JSON "[{\"id\":\"t_41fec39e\",\"status\":\"done\",\"branch_name\":\"z-{agent}/1217-e2e-40-deepseek\",\"title\":\"e2e 40-кейс прогон: deepseek пустой ответ\"}]"

    run_merge_gate
    local journal
    journal="$(cat "$GH_JOURNAL")"

    local archive_calls
    archive_calls="$(printf '%s\n' "$journal" | grep -c 'kanban --board robbox archive t_41fec39e' || true)"
    assert_eq "1" "$archive_calls" "issue done card archived by own branch exact match (C)"
}

# ===========================================================================
# D. done карточка с branch == CLOSED-но-НЕ-merged PR → НЕ архивируется
#    (exact-match не срабатывает: в MERGED-PR списке такого head нет).
# ===========================================================================
test_D_closed_not_merged_pr_not_archived() {
    new_test
    # В MERGED-PR списке НЕТ PR с head z-{agent}/1041-... (он CLOSED, #1155).
    local pr=1109 head='z-{agent}/1105-atomic-e2e-harness'
    fixture_archive "$pr" "$head" '2026-08-11T14:51:11Z'
    set_state KANBAN_LIST_JSON "[{\"id\":\"t_cc9cc56d\",\"status\":\"done\",\"branch_name\":\"z-{agent}/1041-fix-l-build-vision-pi-docker-tag-local-g\",\"title\":\"fix(L-Build Vision Pi): DOCKER_TAG=local\"}]"

    run_merge_gate
    local journal
    journal="$(cat "$GH_JOURNAL")"

    local archive_calls
    archive_calls="$(printf '%s\n' "$journal" | grep -c 'kanban --board robbox archive t_cc9cc56d' || true)"
    assert_eq "0" "$archive_calls" "done card with CLOSED-not-merged PR NOT archived (D)"
}

# ===========================================================================
# E. карточка НЕ done (running) с подходящим MERGED PR → НЕ архивируется.
# ===========================================================================
test_E_non_done_card_not_archived() {
    new_test
    local pr=1221 head='z-devops/t_fe266643-empty-round-sweep-order'
    fixture_archive "$pr" "$head" '2026-08-13T21:25:25Z'
    set_state KANBAN_LIST_JSON "[{\"id\":\"t_fe266643\",\"status\":\"running\",\"branch_name\":\"\",\"title\":\"ретро: пустая round-104\"}]"

    run_merge_gate
    local journal
    journal="$(cat "$GH_JOURNAL")"

    local archive_calls
    archive_calls="$(printf '%s\n' "$journal" | grep -c 'kanban --board robbox archive t_fe266643' || true)"
    assert_eq "0" "$archive_calls" "running card NOT archived (E)"
}

# ===========================================================================
# G. Q22-path archive (ретро 14.08 t_36c9ac4e, кейс t_41fec39e): issue OPEN,
#    PR MERGED, ветка УДАЛЕНА, e2e-done НЕТ → Q22 orphan-close ЗАКРЫВАЕТ issue
#    и архивирует done-карточку ПРЯМО в Q22-ветке (раньше continue до
#    archive-блока стр. 774-783 → карточка оставалась done НАВСЕГДА).
# ===========================================================================
test_G_q22_orphan_close_archives_card() {
    new_test
    local issue=1217 branch='z-{agent}/1217-e2e-40-deepseek'
    set_state ISSUE_LIST_JSON "[{\"number\":${issue},\"title\":\"e2e 40 deepseek\",\"labels\":[{\"name\":\"hermes\"},{\"name\":\"needs-e2e\"}],\"body\":\"issue: #${issue}\"}]"
    set_state "ISSUE_${issue}_LABELS_JSON" '{"labels":[{"name":"hermes"},{"name":"needs-e2e"}]}'
    set_state "ISSUE_${issue}_STATE_JSON" '{"state":"OPEN"}'
    set_state "ISSUE_${issue}_COMMENTS_JSON" "{\"comments\":[{\"body\":\"kanban: t_41fec39e\\\\n\"}]}"
    set_state "ISSUE_${issue}_COMMENTS_SINCE_JSON" '[]'
    set_state "ISSUE_${issue}_TIMELINE_JSON" "[]"
    set_state "PR_HEAD_${branch}_JSON" "[{\"number\":1220,\"state\":\"MERGED\",\"baseRefName\":\"develop\",\"mergedAt\":\"2026-08-13T21:24:57Z\",\"mergeable\":\"MERGEABLE\",\"mergeStateStatus\":\"CLEAN\",\"statusCheckRollup\":[{\"conclusion\":\"SUCCESS\"}],\"title\":\"fix(#${issue}): corrective retry\",\"labels\":[]}]"
    set_state PR_1220_COMMITS_JSON '[]'
    set_state PR_LIST_ALL_OPEN_JSON '[]'
    set_state PR_FOLLOWUP_JSON '[]'
    set_state RATE_LIMIT_JSON '{"resources":{"core":{"remaining":5000}}}'
    # ВАЖНО: BRANCH_PRESENT_<branch> НЕ ставим → ветка удалена → Q22-путь.
    # Карточка t_41fec39e в kanban list — status=done (mock kanban_card_status
    # читает из KANBAN_DB, но archive-вызов идёт в журнал через mock hermes).
    set_state KANBAN_LIST_JSON "[{\"id\":\"t_41fec39e\",\"status\":\"done\",\"branch_name\":\"z-{agent}/1217-e2e-40-deepseek\",\"title\":\"e2e 40-кейс прогон\"}]"

    run_merge_gate
    local journal
    journal="$(cat "$GH_JOURNAL")"

    # 1) Issue закрыта (Q22 orphan-close).
    local close_calls
    close_calls="$(printf '%s\n' "$journal" | grep -c "gh issue close ${issue} --reason completed" || true)"
    assert_eq "1" "$close_calls" "Q22: issue closed (user-merge, e2e impossible)"

    # 2) Карточка t_41fec39e заархивирована в Q22-ветке.
    local archive_calls
    archive_calls="$(printf '%s\n' "$journal" | grep -c 'kanban --board robbox archive t_41fec39e' || true)"
    assert_eq "1" "$archive_calls" "Q22: done card archived right in orphan branch (t_41fec39e)"
}

# ===========================================================================
# F. done карточка без совпадения с MERGED PR → НЕ архивируется.
# ===========================================================================
test_F_unmatched_done_card_not_archived() {
    new_test
    local pr=1221 head='z-devops/t_fe266643-empty-round-sweep-order'
    fixture_archive "$pr" "$head" '2026-08-13T21:25:25Z'
    # Карточка с ДРУГИМ id — не в ветке, в title нет ref на PR/ветку.
    set_state KANBAN_LIST_JSON "[{\"id\":\"t_other000\",\"status\":\"done\",\"branch_name\":\"\",\"title\":\"какая-то посторонняя done-карточка\"}]"

    run_merge_gate
    local journal
    journal="$(cat "$GH_JOURNAL")"

    local archive_calls
    archive_calls="$(printf '%s\n' "$journal" | grep -c 'kanban --board robbox archive t_other000' || true)"
    assert_eq "0" "$archive_calls" "unmatched done card NOT archived (F)"
}

# ===========================================================================
# H. Ретро 14.08 t_0bd15be9: blocked-карточка, чей id (t_<hex>) есть в
#    head-ветке MERGED PR (кейс t_36c9ac4e: фикс #1224 влит, карточка
#    timeout×2 → blocked) → unblock + complete + archive. Раньше проход
#    скипал status!=done → blocked-карточка висела вечно.
# ===========================================================================
test_H_blocked_retro_card_unblock_complete_archive() {
    new_test
    local pr=1224 head='z-devops/t_36c9ac4e-merge-gate-archive-retro-cards'
    fixture_archive "$pr" "$head" '2026-08-14T03:05:42Z'
    set_state KANBAN_LIST_JSON "[{\"id\":\"t_36c9ac4e\",\"status\":\"blocked\",\"branch_name\":\"\",\"title\":\"ретро: merge-gate не архивирует ретро-карточки\"}]"

    run_merge_gate
    local journal
    journal="$(cat "$GH_JOURNAL")"

    # 1) Unblock вызван с reason «фикс влит, критерий выполнен».
    local unblock_calls
    unblock_calls="$(printf '%s\n' "$journal" | grep -c 'kanban --board robbox unblock --reason .* t_36c9ac4e' || true)"
    assert_eq "1" "$unblock_calls" "blocked retro card: unblock called with фикс влит reason"

    # 2) Complete вызван с summary.
    local complete_calls
    complete_calls="$(printf '%s\n' "$journal" | grep -c 'kanban --board robbox complete --summary .* t_36c9ac4e' || true)"
    assert_eq "1" "$complete_calls" "blocked retro card: complete called"

    # 3) Archive вызван ПОСЛЕ unblock+complete.
    local archive_calls
    archive_calls="$(printf '%s\n' "$journal" | grep -c 'kanban --board robbox archive t_36c9ac4e' || true)"
    assert_eq "1" "$archive_calls" "blocked retro card: archived after unblock+complete"
}

# ===========================================================================
# Run
# ===========================================================================
run_test "A. retro-card-archive: done card by branch t_<hex> → archived" test_A_retro_card_archived_by_branch_task_id
run_test "B. retro-card-archive: recovery card by title ref → archived" test_B_recovery_card_archived_by_title_ref
run_test "C. retro-card-archive: issue card by own branch exact → archived" test_C_issue_card_archived_by_own_branch
run_test "D. retro-card-archive: CLOSED-not-merged PR → NOT archived" test_D_closed_not_merged_pr_not_archived
run_test "E. retro-card-archive: non-done card → NOT archived" test_E_non_done_card_not_archived
run_test "F. retro-card-archive: unmatched done card → NOT archived" test_F_unmatched_done_card_not_archived
run_test "G. Q22 orphan-close → done card archived (t_41fec39e)" test_G_q22_orphan_close_archives_card
run_test "H. MERGED PR + blocked retro card → unblock+complete+archive (t_0bd15be9)" test_H_blocked_retro_card_unblock_complete_archive

summary
