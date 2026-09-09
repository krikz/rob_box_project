#!/bin/bash
# ============================================================================
# test_merge_gate_human_close.sh — human-close propagation (ретро 22.08, PR #1516)
#
# Шифу закрыл PR вручную (комментарий + close) — раньше это ни на что не
# влияло: issue оставалась OPEN с hermes/needs-e2e, воркер открывал новый PR
# (#1507→#1516), задача возвращалась бесконечно. Фикс: merge-gate читает
# последний комментарий закрытого PR, и если закрытие ручное (не process-маркер)
# — выводит задачу из автоматического цикла (снимает hermes + needs-e2e).
#
# Scenarios:
#   H1. PR CLOSED + issue OPEN (hermes+needs-e2e) + ручной комментарий
#       → сняты hermes + needs-e2e, комментарий «закрыт вручную» в issue.
#   H2. PR CLOSED + process-маркер (orphan-dead) → НЕ трогаем (без propagation).
#   H3. PR OPEN → НЕ трогаем (обычный путь).
#   H4. PR CLOSED, но issue без hermes/needs-e2e → НЕ трогаем.
#
# Run:
#   bash scripts/agent_flow/tests/test_merge_gate_human_close.sh
# ============================================================================
set -euo pipefail

TEST_LIB_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/mock_env.sh
. "$TEST_LIB_DIR/lib/mock_env.sh"

# ---------------------------------------------------------------------------
# Fixture: issue в hermes-списке + канонический PR в заданном state.
# $1=issue_num $2=pr_num $3=pr_state $4=reason(последний комментарий PR)
# $5=issue_labels_csv
# ---------------------------------------------------------------------------
fixture_human_close() {
    local issue_num="$1" pr_num="$2" pr_state="$3" reason="$4" issue_labels_csv="$5"
    local branch="z-{agent}/${issue_num}-voice-e2e"
    local labels_json="[$(printf '%s' "$issue_labels_csv" | tr ',' '\n' | sed 's/.*/{"name":"&"}/' | paste -sd, -)]"
    set_state ISSUE_LIST_JSON "[{\"number\":${issue_num},\"title\":\"voice-e2e\",\"labels\":${labels_json},\"body\":\"\"}]"
    # kanban-маркер — иначе merge-gate скипнет «triage not finished».
    set_state "ISSUE_${issue_num}_COMMENTS_JSON" '{"comments":[{"body":"kanban: t_abc1234"}]}'
    # Канонический PR по head-ветке (mock: gh pr list --head <branch>).
    set_state "PR_HEAD_${branch}_JSON" "[{\"number\":${pr_num},\"mergeable\":\"MERGEABLE\",\"mergeStateStatus\":\"CLEAN\",\"statusCheckRollup\":[],\"baseRefName\":\"develop\",\"state\":\"${pr_state}\",\"mergedAt\":null,\"title\":\"e2e\",\"labels\":[],\"additions\":0,\"deletions\":0,\"commits\":[]}]"
    # Последний комментарий PR (mock: gh pr view <n> --comments --json comments).
    set_state "PR_${pr_num}_VIEW_JSON" "{\"comments\":[{\"body\":\"${reason}\"}]}"
    # Нет merged PR с тем же head → stale-branch guard молчит.
    set_state "PR_MERGED_HEAD_${branch}_JSON" ''
    set_state RATE_LIMIT_JSON '{"resources":{"core":{"remaining":5000}}}'
}

# ===========================================================================
# H1. Ручное закрытие → propagation: сняты hermes + needs-e2e, коммент в issue.
# ===========================================================================
test_H1_human_close_propagates() {
    new_test
    fixture_human_close 1506 1516 'CLOSED' \
        'Это специальные сценарии без вейк слова, нужны для тестирования беклога!! Не нужно их удалять!!' \
        'hermes,needs-e2e'

    run_merge_gate

    local journal
    journal="$(cat "$GH_JOURNAL")"
    assert_eq "1" "$(printf '%s\n' "$journal" | grep -c 'gh issue edit 1506 --remove-label hermes' || true)" \
        "ручное закрытие → hermes снят с issue" || return 1
    assert_eq "1" "$(printf '%s\n' "$journal" | grep -c 'gh issue edit 1506 --remove-label needs-e2e' || true)" \
        "ручное закрытие → needs-e2e снят с issue" || return 1
    assert_eq "1" "$(printf '%s\n' "$journal" | grep -c 'gh issue comment 1506 .*закрыт вручную' || true)" \
        "ручное закрытие → комментарий «закрыт вручную» в issue" || return 1
}

# ===========================================================================
# H2. Process-закрытие (orphan-dead) → НЕ трогаем issue.
# ===========================================================================
test_H2_process_close_skipped() {
    new_test
    fixture_human_close 1506 1516 'CLOSED' \
        'Закрываю как orphan-dead после ретро t_944df2c5: diff vs develop = только 6 voice .ogg.' \
        'hermes,needs-e2e'

    run_merge_gate

    local journal
    journal="$(cat "$GH_JOURNAL")"
    assert_eq "0" "$(printf '%s\n' "$journal" | grep -c 'gh issue edit 1506 --remove-label hermes' || true)" \
        "process-закрытие → hermes НЕ снимается" || return 1
    assert_eq "0" "$(printf '%s\n' "$journal" | grep -c 'gh issue edit 1506 --remove-label needs-e2e' || true)" \
        "process-закрытие → needs-e2e НЕ снимается" || return 1
    assert_eq "0" "$(printf '%s\n' "$journal" | grep -c 'gh issue comment 1506 .*закрыт вручную' || true)" \
        "process-закрытие → комментарий «закрыт вручную» НЕ пишется" || return 1
}

# ===========================================================================
# H3. PR OPEN → обычный путь, propagation не срабатывает.
# ===========================================================================
test_H3_open_pr_no_propagation() {
    new_test
    fixture_human_close 1506 1516 'OPEN' \
        'любой комментарий' 'hermes,needs-e2e'

    run_merge_gate

    local journal
    journal="$(cat "$GH_JOURNAL")"
    assert_eq "0" "$(printf '%s\n' "$journal" | grep -c 'gh issue edit 1506 --remove-label hermes' || true)" \
        "OPEN PR → hermes НЕ снимается" || return 1
    assert_eq "0" "$(printf '%s\n' "$journal" | grep -c 'gh issue comment 1506 .*закрыт вручную' || true)" \
        "OPEN PR → комментарий «закрыт вручную» НЕ пишется" || return 1
}

# ===========================================================================
# H4. PR CLOSED, но issue вне цикла (без hermes/needs-e2e) → НЕ трогаем.
# ===========================================================================
test_H4_closed_pr_issue_not_in_flow() {
    new_test
    fixture_human_close 1506 1516 'CLOSED' \
        'Это специальные сценарии, не удалять' 'type:testing'

    run_merge_gate

    local journal
    journal="$(cat "$GH_JOURNAL")"
    assert_eq "0" "$(printf '%s\n' "$journal" | grep -c 'gh issue edit 1506 --remove-label hermes' || true)" \
        "issue без hermes/needs-e2e → hermes НЕ снимается" || return 1
    assert_eq "0" "$(printf '%s\n' "$journal" | grep -c 'gh issue edit 1506 --remove-label needs-e2e' || true)" \
        "issue без hermes/needs-e2e → needs-e2e НЕ снимается" || return 1
    assert_eq "0" "$(printf '%s\n' "$journal" | grep -c 'gh issue comment 1506 .*закрыт вручную' || true)" \
        "issue без hermes/needs-e2e → комментарий «закрыт вручную» НЕ пишется" || return 1
}

# ===========================================================================
# Run all tests.
# ===========================================================================
run_test "H1. ручное закрытие → propagation" test_H1_human_close_propagates
run_test "H2. process-закрытие → skip" test_H2_process_close_skipped
run_test "H3. OPEN PR → no propagation" test_H3_open_pr_no_propagation
run_test "H4. issue вне цикла → no propagation" test_H4_closed_pr_issue_not_in_flow

summary
