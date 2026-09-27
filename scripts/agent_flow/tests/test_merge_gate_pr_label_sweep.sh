#!/bin/bash
# ============================================================================
# test_merge_gate_pr_label_sweep.sh — ретро 01.09 t_fd604461
#
# PR-label-sweep после MERGED (auto-cleanup process-меток):
#   - needs-e2e / needs-review / e2e-done / e2e:rejected / no-e2e-required /
#     agent-flow-error / stale-conflicting / stale-branch-reuse должны
#     сниматься с PR как только PR становится MERGED (state=MERGED).
#   - Backstop: сканирование MERGED PR за последние RETRO_MERGED_DAYS дней
#     с process-метками → снять их.
#
# Использует run_merge_gate (полный тик merge-gate.sh). Backstop
# pr_label_sweep_merged_pass_all вызывается в самом конце тика, поэтому
# достаточно проверить journal после run_merge_gate.
#
# Сценарии:
#   A. Backstop: MERGED PR с needs-e2e в PR_LIST_MERGED_JSON (в окне) →
#      needs-e2e снята (вызов gh pr edit).
#   B. Backstop: MERGED PR с 8 process-метками → все 8 сняты.
#   C. Backstop: PR без process-меток → no-op.
#   D. Backstop: PR старше RETRO_MERGED_DAYS → НЕ тронут.
#   E. Backstop: state=OPEN → НЕ тронут.
#   F. Backstop: только process-метки в whitelist (bug/voice сохраняются).
#
# Run:
#   bash scripts/agent_flow/tests/test_merge_gate_pr_label_sweep.sh
# ============================================================================
set -euo pipefail

# CRITICAL: GH_REPO должен совпадать с тем, что использует mock_env в gh pr edit
# (mock передаёт $GH_REPO напрямую из родительского shell). Без export
# run_merge_gate устанавливает default krikz/test-repo, но mock видит реальный
# env, который может быть krikz/rob_box_project. Делаем единообразно.
export GH_REPO="${GH_REPO:-krikz/rob_box_project}"

TEST_LIB_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/mock_env.sh
. "$TEST_LIB_DIR/lib/mock_env.sh"

# Helper: минимальный fixture для теста — нет ни одного issue/PR в основных
# циклах, только PR_LIST_MERGED_JSON для backstop. Не зависит от retropath.
fixture_backstop_only() {
    local pr_num="$1" pr_state="$2" labels_csv="$3" merged_at="$4"
    local labels_json_arr="[]"
    if [ -n "$labels_csv" ]; then
        labels_json_arr="$(printf '%s' "$labels_csv" | python3 -c '
import json, sys
labels = [l.strip() for l in sys.stdin.read().split(",") if l.strip()]
print(json.dumps([{"name": l} for l in labels]))
')"
    fi
    # PR_LIST_MERGED_JSON — для backstop-скана.
    set_state "PR_LIST_MERGED_JSON" "[{\"number\":${pr_num},\"title\":\"fix #${pr_num}\",\"body\":\"\",\"headRefName\":\"z-agent/t_${pr_num}-fix\",\"mergedAt\":\"${merged_at}\",\"labels\":${labels_json_arr}}]"
    # PR view для state-проверки (backstop читает state через gh pr view).
    set_state "PR_${pr_num}_VIEW_JSON" "{\"state\":\"${pr_state}\",\"labels\":${labels_json_arr}}"
    # Остальные fixture'ы — пустые (не запускаем основной цикл).
    set_state ISSUE_LIST_JSON '[]'
    set_state PR_LIST_ALL_OPEN_JSON '[]'
    set_state PR_FOLLOWUP_JSON '[]'
    set_state RATE_LIMIT_JSON '{"resources":{"core":{"remaining":5000}}}'
    set_state "ISSUE_${pr_num}_LABELS_JSON" '{"labels":[]}'
    set_state "ISSUE_${pr_num}_STATE_JSON" '{"state":"CLOSED"}'
}

# ============================================================================
# A. Backstop: MERGED PR с needs-e2e (в окне) → needs-e2e снята.
# ============================================================================
test_A_backstop_removes_needs_e2e_on_merged_pr() {
    new_test
    local now_ts
    now_ts="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
    fixture_backstop_only 1843 "MERGED" "needs-e2e" "${now_ts}"

    run_merge_gate

    local journal
    journal="$(cat "$GH_JOURNAL")"
    local rm_calls
    rm_calls="$(printf '%s\n' "$journal" | grep -c "gh pr edit 1843 --repo ${GH_REPO} --remove-label needs-e2e" || true)"
    assert_eq "1" "$rm_calls" "backstop: needs-e2e снята с MERGED PR"
}

# ============================================================================
# B. Backstop: MERGED PR с 8 process-метками → все 8 сняты.
# ============================================================================
test_B_backstop_removes_all_process_labels() {
    new_test
    local now_ts
    now_ts="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
    fixture_backstop_only 1849 "MERGED" \
        "needs-e2e,needs-review,e2e-done,e2e:rejected,no-e2e-required,agent-flow-error,stale-conflicting,stale-branch-reuse" \
        "${now_ts}"

    run_merge_gate

    local journal
    journal="$(cat "$GH_JOURNAL")"
    local expected=(
        "needs-e2e"
        "needs-review"
        "e2e-done"
        "e2e:rejected"
        "no-e2e-required"
        "agent-flow-error"
        "stale-conflicting"
        "stale-branch-reuse"
    )
    for lbl in "${expected[@]}"; do
        local n
        n="$(printf '%s\n' "$journal" | grep -c "gh pr edit 1849 --repo ${GH_REPO} --remove-label ${lbl}\$" || true)"
        assert_eq "1" "$n" "${lbl} снята"
    done
}

# ============================================================================
# C. Backstop: PR без process-меток → no-op.
# ============================================================================
test_C_backstop_noop_when_no_process_labels() {
    new_test
    local now_ts
    now_ts="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
    fixture_backstop_only 3000 "MERGED" "bug,voice,priority:high" "${now_ts}"

    run_merge_gate

    local journal
    journal="$(cat "$GH_JOURNAL")"
    local rm_calls
    rm_calls="$(printf '%s\n' "$journal" | grep -c 'gh pr edit 3000 --remove-label' || true)"
    assert_eq "0" "$rm_calls" "MERGED без process-меток — no-op"
}

# ============================================================================
# D. Backstop: PR старше RETRO_MERGED_DAYS (14 дней) → НЕ тронут.
# ============================================================================
test_D_backstop_skips_pr_outside_window() {
    new_test
    local _old_ts
    _old_ts="$(date -u -d '30 days ago' +%Y-%m-%dT%H:%M:%SZ 2>/dev/null || date -u +%Y-%m-%dT%H:%M:%SZ)"
    fixture_backstop_only 4000 "MERGED" "needs-e2e" "${_old_ts}"

    run_merge_gate

    local journal
    journal="$(cat "$GH_JOURNAL")"
    local rm_calls
    rm_calls="$(printf '%s\n' "$journal" | grep -c 'gh pr edit 4000 --remove-label' || true)"
    assert_eq "0" "$rm_calls" "PR за окном RETRO_MERGED_DAYS — НЕ тронут"
}

# ============================================================================
# E. Backstop: PR state=OPEN (не MERGED) → НЕ тронут.
# ============================================================================
test_E_backstop_skips_open_pr() {
    new_test
    local now_ts
    now_ts="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
    fixture_backstop_only 5000 "OPEN" "needs-e2e" "${now_ts}"

    run_merge_gate

    local journal
    journal="$(cat "$GH_JOURNAL")"
    local rm_calls
    rm_calls="$(printf '%s\n' "$journal" | grep -c 'gh pr edit 5000 --remove-label' || true)"
    assert_eq "0" "$rm_calls" "OPEN PR (не MERGED) — НЕ тронут"
}

# ============================================================================
# F. Backstop: пользовательские метки (bug/voice/agent:*) НЕ трогаем.
# ============================================================================
test_F_backstop_preserves_user_labels() {
    new_test
    local now_ts
    now_ts="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
    fixture_backstop_only 6000 "MERGED" "bug,voice,agent:backend,priority:high" "${now_ts}"

    run_merge_gate

    local journal
    journal="$(cat "$GH_JOURNAL")"
    local rm_calls
    rm_calls="$(printf '%s\n' "$journal" | grep -c 'gh pr edit 6000 --remove-label' || true)"
    assert_eq "0" "$rm_calls" "MERGED с user-метками — no-op (только process-метки)"
}

# ============================================================================
# Run
# ============================================================================
run_test "A. Backstop: needs-e2e на MERGED PR → снята" test_A_backstop_removes_needs_e2e_on_merged_pr
run_test "B. Backstop: 8 process-меток на MERGED PR → все сняты" test_B_backstop_removes_all_process_labels
run_test "C. Backstop: MERGED без process-меток → no-op" test_C_backstop_noop_when_no_process_labels
run_test "D. Backstop: PR старше 14 дней → НЕ тронут" test_D_backstop_skips_pr_outside_window
run_test "E. Backstop: PR state=OPEN → НЕ тронут" test_E_backstop_skips_open_pr
run_test "F. Backstop: user-метки НЕ тронуты" test_F_backstop_preserves_user_labels

# ============================================================================
# Summary
# ============================================================================
echo
echo "==== Summary ===="
echo "total:  $TESTS_TOTAL"
echo "passed: $TESTS_PASSED"
echo "failed: $TESTS_FAILED"
if [ "$TESTS_FAILED" -gt 0 ]; then
    echo "failures:"
    for name in "${FAILED_NAMES[@]}"; do
        echo "  - $name"
    done
    exit 1
fi
exit 0
