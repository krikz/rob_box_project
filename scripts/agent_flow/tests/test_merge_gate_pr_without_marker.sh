#!/bin/bash
# ============================================================================
# test_merge_gate_pr_without_marker.sh — ретро 25.08 t_1a4f3275 / issue #1624
#
# Verifies pr_without_marker_scan_all в agent-flow-merge-gate.sh:
#   Сценарий: воркер открыл PR с process-меткой (agent-flow / agent-flow-error
#   / needs-e2e / needs-review) в обход процесса — без kanban-marker в issue
#   body и без kanban-карточки. Основной цикл merge-gate ходит по issues →
#   ищет kanban-marker → не находит → skip. PR висит в CONFLICTING/UNSTABLE
#   без внимания процесса.
#
# Guard: pr_without_marker_scan_all сканирует ВСЕ open PR с process-меткой,
# определяет связанный issue (по #NNNN в title или z-{agent}/NNNN-* в head),
# проверяет kanban-marker в issue comments (REST API /repos/X/issues/N/comments).
# Если marker'а нет — коммент (24h dedup) на PR + на issue. Если marker есть —
# skip. PR без process-меток — skip (не шумим на black-label PR).
#
# Acceptance:
#   W1. PR с process-меткой + issue без marker → коммент на PR + на issue.
#   W2. PR с process-меткой + issue с marker → skip (нет комментов).
#   W3. PR БЕЗ process-метки → skip (вне scope, не шумим).
#   W4. PR с process-меткой, но без #NNNN и без z-{agent}/NNNN- → skip.
#   W5. Дубликат-коммент в течение 24h → skip (dedup по substring тела).
#
# Run:
#   bash scripts/agent_flow/tests/test_merge_gate_pr_without_marker.sh
# ============================================================================
set -euo pipefail

TEST_LIB_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/mock_env.sh
. "$TEST_LIB_DIR/lib/mock_env.sh"

# ---------------------------------------------------------------------------
# Fixture: открытый PR + базовый issue/PR для mock_env.
# Мок читает:
#   - PR_LIST_ALL_OPEN_JSON   — список PR для `gh pr list --state open`
#                               (включая scan-all-prs блок)
#   - ISSUE_<n>_COMMENTS_JSON — комментарии issue (для `gh issue view N --comments`)
#                               И для `gh api repos/.../issues/N/comments` (без since)
#                               mock_env использует один ключ ISSUE_N_COMMENTS_SINCE_JSON
#                               для gh api .../issues/N/comments* — для нашего guard'а
#                               (мы зовём gh api .../comments?per_page=100) мок вернёт то
#                               же значение, и в нём должен быть kanban-marker (или его
#                               отсутствие) для теста.
#   - PR_<n>_VIEW_JSON        — для `gh pr view N --json headRefName`
# ---------------------------------------------------------------------------
fixture_wm_pr() {
    local pr="$1" head="$2" title="$3" labels_csv="$4" issue="${5:-}"
    local labels_json
    labels_json="$(printf '%s' "$labels_csv" | tr ',' '\n' | sed 's/.*/{"name":"&"}/' | paste -sd, -)"
    [ -z "$labels_json" ] && labels_json='[]'
    set_state ISSUE_LIST_JSON '[]'
    set_state PR_LIST_ALL_OPEN_JSON "[{\"number\":${pr},\"headRefName\":\"${head}\",\"title\":\"${title}\",\"mergeStateStatus\":\"CONFLICTING\",\"isDraft\":false,\"labels\":[${labels_json}]}]"
    set_state RATE_LIMIT_JSON '{"resources":{"core":{"remaining":5000}}}'
    # gh pr view N --json headRefName/baseRefName/mergeStateStatus идёт через
    # общую PR_N_VIEW_JSON (default-branch мока).
    set_state "PR_${pr}_VIEW_JSON" "{\"headRefName\":\"${head}\",\"baseRefName\":\"develop\",\"mergeStateStatus\":\"CONFLICTING\"}"
    if [ -n "$issue" ]; then
        # Для дедупа — стартовый пустой (нет «уже висевшего» marker-комментария).
        # REST-массив (как у реального gh api .../issues/N/comments).
        set_state "ISSUE_${issue}_COMMENTS_JSON" '[]'
        set_state "ISSUE_${issue}_COMMENTS_SINCE_JSON" '[]'
    fi
}

# Подкладывает kanban-marker (или нет) в комментарии issue. Использует оба ключа,
# потому что gh api .../comments* в моке читает COMMENTS_SINCE_JSON, а
# gh issue view N --comments — COMMENTS_JSON (для основного цикла).
#
# Реальный REST GET /repos/X/issues/N/comments возвращает JSON-МАССИВ
# объектов [{user,body,created_at,...}], не {"comments":[...]}. Поэтому
# фикстура хранит массив, как у реального GitHub.
set_issue_marker() {
    local issue="$1" marker="$2"  # marker: t_<id> или пусто
    local body
    if [ -n "$marker" ]; then
        body="[{\"body\":\"kanban: ${marker}\\nbranch: z-{agent}/${issue}-slug\\nrole: devops\"}]"
    else
        body='[{"body":"какой-то обычный коммент без marker"}]'
    fi
    set_state "ISSUE_${issue}_COMMENTS_JSON" "$body"
    set_state "ISSUE_${issue}_COMMENTS_SINCE_JSON" "$body"
}

# ============================================================================
# W1. PR с process-меткой + issue без marker → комменты на PR + на issue.
#     Это кейс PR #1623 / issue #1600 (agent-flow не использовался,
#     PR открыт через gh api напрямую).
# ============================================================================
test_W1_pr_without_marker_gets_alert() {
    new_test
    fixture_wm_pr 3501 "z-{agent}/1600-av-6-supervisor-node-rebased-v2" \
        "feat(supervisor): supervisor_node.py в monitor-режиме" "needs-e2e" 1600
    set_issue_marker 1600 ""  # БЕЗ kanban-marker

    run_merge_gate

    local journal
    journal="$(cat "$GH_JOURNAL")"
    assert_eq "1" "$(printf '%s\n' "$journal" | grep -c 'gh pr comment 3501 ' || true)" \
        "PR без kanban-marker → gh pr comment" || return 1
    assert_eq "1" "$(printf '%s\n' "$journal" | grep -c 'gh issue comment 1600 ' || true)" \
        "PR без kanban-marker → gh issue comment на связанной issue" || return 1
    # log() пишет в stderr (не в journal) — проверим через stderr.log.
    local stderr_log
    stderr_log="$(cat "$TEST_TMP/stderr.log")"
    assert_contains "pr-without-marker-scan: PR #3501" "$stderr_log" \
        "лог 'pr-without-marker-scan: PR #3501' присутствует" || return 1
}

# ============================================================================
# W2. PR с process-меткой + issue С marker → skip (нет комментов).
# ============================================================================
test_W2_pr_with_marker_skipped() {
    new_test
    fixture_wm_pr 3502 "z-{agent}/3502-fix-thing" \
        "fix(voice #3502): demo" "needs-e2e" 3502
    set_issue_marker 3502 "t_dead3502"  # marker есть

    run_merge_gate

    local journal
    journal="$(cat "$GH_JOURNAL")"
    assert_eq "0" "$(printf '%s\n' "$journal" | grep -c 'gh pr comment 3502 ' || true)" \
        "PR с kanban-marker → НЕ комментим PR" || return 1
    assert_eq "0" "$(printf '%s\n' "$journal" | grep -c 'gh issue comment 3502 ' || true)" \
        "PR с kanban-marker → НЕ комментим issue" || return 1
    assert_eq "0" "$(printf '%s\n' "$journal" | grep -c 'pr-without-marker-scan: PR #3502' || true)" \
        "PR с kanban-marker → НЕТ лога pr-without-marker-scan" || return 1
}

# ============================================================================
# W3. PR БЕЗ process-метки → skip (вне scope, не шумим на black-label PR).
# ============================================================================
test_W3_pr_without_process_labels_skipped() {
    new_test
    fixture_wm_pr 3503 "feature/some-branch" \
        "fix: random fix" "" 3503
    set_issue_marker 3503 ""

    run_merge_gate

    local journal
    journal="$(cat "$GH_JOURNAL")"
    assert_eq "0" "$(printf '%s\n' "$journal" | grep -c 'gh pr comment 3503 ' || true)" \
        "PR без process-метки → НЕ комментим (не шумим)" || return 1
    assert_eq "0" "$(printf '%s\n' "$journal" | grep -c 'gh issue comment 3503 ' || true)" \
        "PR без process-метки → НЕ комментим issue" || return 1
}

# ============================================================================
# W4. PR с process-меткой, но без #NNNN в title и без z-{agent}/NNNN- в head
#     → skip (нет точки приложения для проверки).
# ============================================================================
test_W4_pr_without_issue_ref_skipped() {
    new_test
    fixture_wm_pr 3504 "feature/random-branch" \
        "chore: nothing" "needs-e2e" 3504
    # Issue #3504 задан, но PR не указывает на неё (нет #3504 в title, нет
    # z-{agent}/3504-* в head). Guard не должен ничего делать.

    run_merge_gate

    local journal
    journal="$(cat "$GH_JOURNAL")"
    assert_eq "0" "$(printf '%s\n' "$journal" | grep -c 'gh pr comment 3504 ' || true)" \
        "PR без issue-ref → НЕ комментим (нет сопоставления)" || return 1
}

# ============================================================================
# W5. Дубликат-коммент в течение 24h → skip (dedup по substring тела).
# ============================================================================
test_W5_dedup_substring_24h() {
    new_test
    fixture_wm_pr 3505 "z-{agent}/3505-fix" \
        "fix(test #3505): demo" "needs-e2e" 3505
    set_issue_marker 3505 ""
    # Подложим «уже висевший» comment с подстрокой "process marker missing"
    # для PR-dedup + "process marker missing on PR" для issue-dedup — guard
    # должен dedup'нуть обе ветки через 24h substring-проверку. Мок для gh api
    # .../issues/3505/comments?since=... читает ISSUE_3505_COMMENTS_SINCE_JSON.
    # Реальный REST возвращает массив, не объект.
    set_state "ISSUE_3505_COMMENTS_SINCE_JSON" \
        '[{"body":"⚠️ **process marker missing on PR** (merge-gate, прошлый тик)"},{"body":"⚠️ **process marker missing** (merge-gate, прошлый тик)"}]'
    # Для gh pr comment mock просто journal — нам важно, что guard ВНУТРИ
    # себя делает gh api на PR (тот же mock → вернёт предыдущий marker-коммент).
    # Но PR-номер = 3505 — мок gh api .../issues/3505/comments?since=... опять
    # вернёт marker. Это OK: оба пути увидят dedup.

    run_merge_gate

    local journal
    journal="$(cat "$GH_JOURNAL")"
    assert_eq "0" "$(printf '%s\n' "$journal" | grep -c 'gh pr comment 3505 ' || true)" \
        "dedup на substring → НЕ шлём gh pr comment повторно" || return 1
    assert_eq "0" "$(printf '%s\n' "$journal" | grep -c 'gh issue comment 3505 ' || true)" \
        "dedup на substring → НЕ шлём gh issue comment повторно" || return 1
}

# ============================================================================
# Run all tests.
# ============================================================================
run_test "W1. PR без marker → alert на PR + issue" test_W1_pr_without_marker_gets_alert
run_test "W2. PR с marker → skip" test_W2_pr_with_marker_skipped
run_test "W3. PR без process-метки → skip (вне scope)" test_W3_pr_without_process_labels_skipped
run_test "W4. PR без issue-ref → skip" test_W4_pr_without_issue_ref_skipped
run_test "W5. dedup 24h substring → skip" test_W5_dedup_substring_24h

# print_summary — финальный счёт (TESTS_TOTAL / TESTS_PASSED / TESTS_FAILED).
printf '\n%d tests, %d passed, %d failed\n' "$TESTS_TOTAL" "$TESTS_PASSED" "$TESTS_FAILED"
if [ "$TESTS_FAILED" -ne 0 ]; then
    printf 'FAILED: %s\n' "${FAILED_NAMES[*]}"
    exit 1
fi
exit 0
