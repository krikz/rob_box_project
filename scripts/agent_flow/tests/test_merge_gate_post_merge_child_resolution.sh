#!/bin/bash
# ============================================================================
# test_merge_gate_post_merge_child_resolution.sh
#
# Ретро t_527e1231 → process-fix t_58c69473 («orphan-parent pattern»).
#
# ПРОБЛЕМА: карточка остаётся в `todo`/`ready` НАВСЕГДА, хотя её работа уже
# влита в develop через MERGED PR. Наблюдаемые кейсы 01.09:
#   - t_e2ae0c29 (todo, assignee=agent-flow, >сутки): реализация ADR-0035
#     влита через PR #1849 (MERGED 01.09 06:10:45Z). Никто не закрыл.
#   - issue #1810/#1811: fix в develop (ae170b717 + 0237fbdb5), issue OPEN.
#
# Почему существующие проходы НЕ ловят:
#   - main-cycle archive-путь: работает по issue-линку карточки; ретро/
#     decomposer-карточки issue не имеют.
#   - retro-card-archive (ретро 14.08 t_36c9ac4e): обрабатывает ТОЛЬКО
#     status ∈ {done, blocked}. Карточка в `todo`/`ready` не подпадает.
#   - scan-all-prs: смотрит OPEN PR, а тут PR уже MERGED.
#
# НОВЫЙ ПРОХОД: post-merge-child-resolution.
# Для недавно смерженных PR (base=develop, окно RETRO_MERGED_DAYS) ищет
# НЕ-терминальные карточки (todo/ready) и завершает их:
#   S1 (marker):      body карточки содержит `parent-pr:<N>` — явный контракт
#                     (auto-decomposer / архитектор ставят маркер).
#   S2 (branch-token): id карточки (t_<hex>) — токен head-ветки смерженного PR
#                     И на этом токене НЕТ открытого PR (иначе работа ещё идёт).
#
# Сценарии:
#   P1. todo + body `parent-pr:1849` + MERGED PR #1849 → complete (1 раз).
#   P2. todo + id-токен в head MERGED PR, открытых PR на токене нет → complete.
#   P3. то же, но есть OPEN PR с тем же токеном → НЕ complete (работа идёт).
#   P4. running карточка с `parent-pr:<N>` → НЕ complete (живой воркер).
#   P5. done карточка → НЕ complete этим проходом (владеет retro-card-archive).
#   P6. нет совпадения (чужой PR) → НЕ complete.
#   P7. DRY_RUN=true → complete НЕ вызывается, только лог.
#
# Run:
#   bash scripts/agent_flow/tests/test_merge_gate_post_merge_child_resolution.sh
# ============================================================================
set -euo pipefail

TEST_LIB_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/mock_env.sh
. "$TEST_LIB_DIR/lib/mock_env.sh"

# ---------------------------------------------------------------------------
# Базовый fixture: один смерженный PR (без #N-ссылок на issues, чтобы
# retro-path молчал), пустой issue-список, никаких open PR по умолчанию.
# $1=pr_number $2=head_branch $3=mergedAt
# ---------------------------------------------------------------------------
fixture_merged_pr() {  # $1=pr $2=head $3=mergedAt
    local pr="$1" head="$2" merged_at="$3"
    set_state ISSUE_LIST_JSON '[]'
    set_state PR_LIST_MERGED_JSON \
        "[{\"number\":${pr},\"title\":\"feat(merge-gate): post-merge child resolution\",\"body\":\"\",\"headRefName\":\"${head}\",\"mergedAt\":\"${merged_at}\"}]"
    set_state PR_LIST_ALL_OPEN_JSON '[]'
    set_state PR_LIST_ALL_OPEN_REST_JSON '[]'
    set_state PR_FOLLOWUP_JSON '[]'
    set_state RATE_LIMIT_JSON '{"resources":{"core":{"remaining":5000}}}'
}

# Сколько раз merge-gate позвал `kanban ... complete` для конкретной карточки.
complete_calls_for() {  # $1=card_id
    local card="$1"
    grep -F 'kanban --board robbox complete' "$GH_JOURNAL" 2>/dev/null \
        | grep -cF -- "$card" || true
}

# ===========================================================================
# P1. todo-карточка с явным маркером `parent-pr:<N>` + MERGED PR → complete.
# ===========================================================================
test_P1_marker_card_completed() {
    new_test
    fixture_merged_pr 1849 'z-backend/t_e2ae0c29-stale-diag-auto-detect-impl' '2026-09-01T06:10:45Z'
    set_state KANBAN_LIST_JSON \
        '[{"id":"t_cafe0001","status":"todo","branch_name":"","title":"ретро: merge-gate auto-detect stale-after-upstream-fix","body":"## Контекст\nРеализация ADR-0035.\nparent-pr:1849\n"}]'

    run_merge_gate
    assert_eq "1" "$(complete_calls_for t_cafe0001)" \
        "todo card with parent-pr:1849 marker completed once (P1)"
    assert_contains "post-merge-child-resolution" "$(cat "$TEST_TMP/stderr.log")" \
        "pass logged its name (P1)"
}

# ===========================================================================
# P2. todo-карточка, id которой — токен head-ветки MERGED PR, открытых PR на
#     этом токене нет → complete.
# ===========================================================================
test_P2_branch_token_card_completed() {
    new_test
    fixture_merged_pr 1849 'z-backend/t_aaaa1111-stale-diag-impl' '2026-09-01T06:10:45Z'
    set_state KANBAN_LIST_JSON \
        '[{"id":"t_aaaa1111","status":"todo","branch_name":"","title":"impl ADR-0035","body":"## Контекст\nбез маркера\n"}]'

    run_merge_gate
    assert_eq "1" "$(complete_calls_for t_aaaa1111)" \
        "todo card matched by t_<hex> token in merged head completed (P2)"
}

# ===========================================================================
# P3. То же, что P2, но на том же токене есть OPEN PR → работа продолжается,
#     карточку НЕ закрываем (guard от убийства живой работы).
# ===========================================================================
test_P3_open_pr_on_token_not_completed() {
    new_test
    fixture_merged_pr 1849 'z-backend/t_aaaa1111-stale-diag-impl' '2026-09-01T06:10:45Z'
    # Открытый PR на ТОЙ ЖЕ ветке (продолжение работы, кейс PR #1853).
    set_state PR_LIST_ALL_OPEN_JSON \
        '[{"number":1853,"headRefName":"z-backend/t_aaaa1111-stale-diag-impl","mergeable":"MERGEABLE","mergeStateStatus":"CLEAN","title":"feat: продолжение","labels":[],"baseRefName":"develop","state":"OPEN"}]'
    set_state KANBAN_LIST_JSON \
        '[{"id":"t_aaaa1111","status":"todo","branch_name":"","title":"impl ADR-0035","body":"без маркера"}]'

    run_merge_gate
    assert_eq "0" "$(complete_calls_for t_aaaa1111)" \
        "card with OPEN PR on same token NOT completed (P3)"
}

# ===========================================================================
# P4. running-карточка (живой воркер) с маркером → НЕ complete.
# ===========================================================================
test_P4_running_card_not_completed() {
    new_test
    fixture_merged_pr 1849 'z-devops/t_bbbb2222-impl' '2026-09-01T06:10:45Z'
    set_state KANBAN_LIST_JSON \
        '[{"id":"t_bbbb2222","status":"running","branch_name":"","title":"impl","body":"parent-pr:1849"}]'

    run_merge_gate
    assert_eq "0" "$(complete_calls_for t_bbbb2222)" \
        "running card NOT completed (P4)"
}

# ===========================================================================
# P5. done-карточка → НЕ complete этим проходом (её путь — retro-card-archive:
#     archive, а не повторный complete).
# ===========================================================================
test_P5_done_card_not_completed_here() {
    new_test
    fixture_merged_pr 1849 'z-devops/t_cccc3333-impl' '2026-09-01T06:10:45Z'
    set_state KANBAN_LIST_JSON \
        '[{"id":"t_cccc3333","status":"done","branch_name":"","title":"impl","body":"parent-pr:1849"}]'

    run_merge_gate
    assert_eq "0" "$(complete_calls_for t_cccc3333)" \
        "done card NOT re-completed by this pass (P5)"
    # Регрессия: retro-card-archive по-прежнему архивирует её.
    assert_contains "archive t_cccc3333" "$(cat "$GH_JOURNAL")" \
        "retro-card-archive still archives the done card (P5 regression)"
}

# ===========================================================================
# P6. Нет совпадения: маркер на другой PR, токен не совпадает → НЕ complete.
# ===========================================================================
test_P6_no_match_not_completed() {
    new_test
    fixture_merged_pr 1849 'z-devops/t_dddd4444-impl' '2026-09-01T06:10:45Z'
    set_state KANBAN_LIST_JSON \
        '[{"id":"t_eeee5555","status":"todo","branch_name":"","title":"другая работа","body":"parent-pr:1700"}]'

    run_merge_gate
    assert_eq "0" "$(complete_calls_for t_eeee5555)" \
        "unrelated card NOT completed (P6)"
}

# ===========================================================================
# P7. DRY_RUN → никаких complete-вызовов, только лог.
# ===========================================================================
test_P7_dry_run_no_side_effects() {
    new_test
    fixture_merged_pr 1849 'z-backend/t_ffff6666-impl' '2026-09-01T06:10:45Z'
    set_state KANBAN_LIST_JSON \
        '[{"id":"t_ffff6666","status":"ready","branch_name":"","title":"impl","body":"parent-pr:1849"}]'

    DRY_RUN=true run_merge_gate
    assert_eq "0" "$(complete_calls_for t_ffff6666)" \
        "DRY_RUN performs no complete call (P7)"
    assert_contains "DRY-RUN would complete card t_ffff6666" "$(cat "$TEST_TMP/stderr.log")" \
        "DRY_RUN logs the intent (P7)"
}

# ===========================================================================
# P8. todo-карточка требует promote ПЕРЕД complete: реальный
#     kanban_db.complete_task() принимает только running|ready|blocked|review
#     и требует satisfied parents. Мок раньше это скрывал — `complete` на todo
#     молча «проходил» в тесте и падал в прод-CLI с «cannot complete <id>
#     (unknown id or terminal state)». Проверено вживую на t_e2ae0c29 /
#     t_4019c107 (01.09).
# ===========================================================================
test_P8_todo_promoted_before_complete() {
    new_test
    fixture_merged_pr 1849 'z-backend/t_1111aaaa-impl' '2026-09-01T06:10:45Z'
    set_state KANBAN_LIST_JSON \
        '[{"id":"t_1111aaaa","status":"todo","branch_name":"","title":"impl","body":"parent-pr:1849"}]'

    run_merge_gate
    local j; j="$(cat "$GH_JOURNAL")"
    # Порядок аргументов важен: reason ПОЗИЦИОННЫЙ и идёт ДО --force
    # (иначе прод-CLI падает «unrecognized arguments: <reason>» —
    # проверено вживую 01.09).
    assert_contains "promote t_1111aaaa " "$j" \
        "todo card is promoted before complete (P8)"
    assert_contains "--force" "$j" \
        "promote uses --force (parents may be open; work is already merged) (P8)"
    assert_eq "1" "$(complete_calls_for t_1111aaaa)" \
        "todo card still completed exactly once after promote (P8)"
}

# ===========================================================================
# P9. ready-карточка НЕ нуждается в promote (complete_task её принимает).
# ===========================================================================
test_P9_ready_completed_without_promote() {
    new_test
    fixture_merged_pr 1849 'z-backend/t_2222bbbb-impl' '2026-09-01T06:10:45Z'
    set_state KANBAN_LIST_JSON \
        '[{"id":"t_2222bbbb","status":"ready","branch_name":"","title":"impl","body":"parent-pr:1849"}]'

    run_merge_gate
    local j; j="$(cat "$GH_JOURNAL")"
    assert_not_contains "promote t_2222bbbb" "$j" \
        "ready card is NOT promoted (complete accepts ready) (P9)"
    assert_eq "1" "$(complete_calls_for t_2222bbbb)" \
        "ready card completed once (P9)"
}

run_test "P1 marker card completed"              test_P1_marker_card_completed
run_test "P2 branch-token card completed"        test_P2_branch_token_card_completed
run_test "P3 open PR on token → not completed"   test_P3_open_pr_on_token_not_completed
run_test "P4 running card → not completed"       test_P4_running_card_not_completed
run_test "P5 done card → not completed here"     test_P5_done_card_not_completed_here
run_test "P6 no match → not completed"           test_P6_no_match_not_completed
run_test "P7 DRY_RUN → no side effects"          test_P7_dry_run_no_side_effects
run_test "P8 todo → promote --force then complete" test_P8_todo_promoted_before_complete
run_test "P9 ready → complete without promote"   test_P9_ready_completed_without_promote

summary
