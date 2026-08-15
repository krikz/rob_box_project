#!/bin/bash
# ============================================================================
# test_merge_gate_pr_orphan_needs_e2e.sh — PR-side needs-e2e сироты → needs-review
#
# Ретро 15.08 t_5cf0162b (надзор PR #1263 / issue #1246): merge-gate ставит
# needs-e2e на PR best-effort propagation, но обратного хода нет:
#   - e2e-process строит ротацию ТОЛЬКО по issues с needs-e2e — если связанная
#     issue уже обработана (e2e-done) и CLOSED, а на PR осталась needs-e2e —
#     PR НИКОГДА не попадёт в round (PR #1263, ветка z-devops/t_feca724c-*);
#   - clean-pr-sweep сканирует только PR БЕЗ process-меток → PR с needs-e2e
#     пропущен;
#   - post-round sweep (e2e-process) лейблит только ISSUES, не PR.
#
# Фикс: pr-orphan-reconcile — для OPEN PR с needs-e2e, у которых НЕТ связанной
# OPEN issue с needs-e2e:
#   - CI-only (все файлы .github/, scripts/agent_flow/, docs/) → needs-review
#     на PR + снять needs-e2e (e2e не нужен);
#   - functional + есть OPEN issue → вернуть needs-e2e на issue;
#   - functional + issue CLOSED/нет → needs-review на PR + снять needs-e2e
#     (e2e невозможен; Шифу решает, НЕ close автоматически).
#
# Scenarios:
#   O1. CI-only PR с needs-e2e, issue CLOSED → needs-review на PR, needs-e2e
#       снят (кейс PR #1263).
#   O2. functional PR с needs-e2e, issue OPEN (без needs-e2e) → needs-e2e
#       возвращается на issue.
#   O3. functional PR с needs-e2e, issues нет → needs-review на PR, needs-e2e
#       снят.
#   O4. PR с needs-e2e + живая issue (OPEN + needs-e2e) → skip (не сирота).
#   O5. PR с needs-e2e + UNSTABLE → skip (зона scan-all-prs).
#   O6. PR с needs-e2e, head уже влита через ДРУГОЙ PR → skip
#       (зона stale_branch_scan_all, ретро 14.08 t_28afb585).
#
# Run:
#   bash scripts/agent_flow/tests/test_merge_gate_pr_orphan_needs_e2e.sh
# ============================================================================
set -euo pipefail

TEST_LIB_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/mock_env.sh
. "$TEST_LIB_DIR/lib/mock_env.sh"

# ---------------------------------------------------------------------------
# Fixture: PR с needs-e2e в PR_LIST_ALL_OPEN_JSON + связанная issue.
# $1=pr $2=head $3=title $4=merge_state $5=files_csv $6=issue_num (optional)
# ---------------------------------------------------------------------------
fixture_orphan_pr() {
    local pr="$1" head="$2" title="$3" merge_state="$4" files_csv="$5" issue_num="${6:-}"
    set_state ISSUE_LIST_JSON '[]'
    local files_json="[]"
    if [ -n "$files_csv" ]; then
        files_json="[$(printf '%s' "$files_csv" | tr ',' '\n' | sed 's/.*/{"path":"&"}/' | paste -sd, -)]"
    fi
    local body_json=""
    if [ -n "$issue_num" ]; then
        body_json=',"body":"closes #'${issue_num}'"'
    fi
    local labels_json='[{"name":"needs-e2e"}]'
    set_state PR_LIST_ALL_OPEN_JSON "[{\"number\":${pr},\"headRefName\":\"${head}\",\"title\":\"${title}\",\"mergeStateStatus\":\"${merge_state}\",\"isDraft\":false,\"labels\":${labels_json},\"files\":${files_json}${body_json}}]"
    set_state RATE_LIMIT_JSON '{"resources":{"core":{"remaining":5000}}}'
    # Нет merged PR с этим head → stale-branch guard молчит.
    set_state "PR_MERGED_HEAD_${head}_JSON" ''
    set_state "BRANCH_PRESENT_${head}" 1
}

# ===========================================================================
# O1. CI-only PR с needs-e2e, issue CLOSED → needs-review на PR + снят needs-e2e
#     (кейс PR #1263: CI-only workflow, issue #1246 e2e-done + CLOSED).
# ===========================================================================
test_O1_ci_only_orphan_gets_needs_review() {
    new_test
    fixture_orphan_pr 3401 'z-devops/t_feca724c-deploy-issues-hermes-label' \
        'fix(ci/deploy #1246): метки' CLEAN \
        '.github/workflows/L-Deploy and Verify.yml' 1246
    # Issue #1246 CLOSED (обработана: e2e-done + закрыта) — сирота.
    set_state "ISSUE_1246_STATE_JSON" '{"state":"CLOSED"}'
    set_state "ISSUE_1246_LABELS_JSON" '{"labels":[{"name":"e2e-done"}]}'

    run_merge_gate

    local journal
    journal="$(cat "$GH_JOURNAL")"
    assert_eq "1" "$(printf '%s\n' "$journal" | grep -c 'gh pr edit 3401 .*--remove-label needs-e2e' || true)" \
        "CI-only orphan PR → needs-e2e снят" || return 1
    assert_eq "1" "$(printf '%s\n' "$journal" | grep -c 'gh pr edit 3401 .*--add-label needs-review' || true)" \
        "CI-only orphan PR → needs-review поставлен" || return 1
    assert_eq "0" "$(printf '%s\n' "$journal" | grep -c 'gh issue edit 1246 .*--add-label needs-e2e' || true)" \
        "CI-only orphan PR → needs-e2e НЕ возвращается на CLOSED issue" || return 1
}

# ===========================================================================
# O2. functional PR с needs-e2e, issue OPEN (без needs-e2e) → needs-e2e
#     возвращается на issue (e2e-process возьмёт её в ротацию).
# ===========================================================================
test_O2_functional_orphan_returns_needs_e2e_to_issue() {
    new_test
    fixture_orphan_pr 3402 'z-{agent}/3402-voice-fix' \
        'fix(voice #3402): демо' CLEAN 'dialogue_node.py' 3402
    # Issue OPEN, но без needs-e2e (метка потеряна) — возвращаем.
    set_state "ISSUE_3402_STATE_JSON" '{"state":"OPEN"}'
    set_state "ISSUE_3402_LABELS_JSON" '{"labels":[{"name":"hermes"}]}'

    run_merge_gate

    local journal
    journal="$(cat "$GH_JOURNAL")"
    assert_eq "1" "$(printf '%s\n' "$journal" | grep -c 'gh issue edit 3402 .*--add-label needs-e2e' || true)" \
        "functional orphan PR + OPEN issue → needs-e2e возвращён на issue" || return 1
    assert_eq "0" "$(printf '%s\n' "$journal" | grep -c 'gh pr edit 3402 .*--add-label needs-review' || true)" \
        "functional orphan PR + OPEN issue → needs-review НЕ ставится" || return 1
}

# ===========================================================================
# O3. functional PR с needs-e2e, issues нет → needs-review на PR, needs-e2e
#     снят (e2e невозможен без issue; Шифу решает, close НЕ зовём).
# ===========================================================================
test_O3_functional_no_issue_gets_needs_review() {
    new_test
    fixture_orphan_pr 3403 'z-devops/t_5cf0162b-fix' \
        'fix(agent-flow): фикс конвейера' CLEAN 'scripts/agent_flow/agent-flow-merge-gate.sh' ''

    run_merge_gate

    local journal
    journal="$(cat "$GH_JOURNAL")"
    assert_eq "1" "$(printf '%s\n' "$journal" | grep -c 'gh pr edit 3403 .*--remove-label needs-e2e' || true)" \
        "functional orphan PR без issue → needs-e2e снят" || return 1
    assert_eq "1" "$(printf '%s\n' "$journal" | grep -c 'gh pr edit 3403 .*--add-label needs-review' || true)" \
        "functional orphan PR без issue → needs-review поставлен" || return 1
    assert_eq "0" "$(printf '%s\n' "$journal" | grep -c 'gh issue close' || true)" \
        "functional orphan PR без issue → issue НЕ закрывается (решение Шифу)" || return 1
}

# ===========================================================================
# O4. PR с needs-e2e + живая issue (OPEN + needs-e2e) → skip (не сирота).
# ===========================================================================
test_O4_live_issue_skipped() {
    new_test
    fixture_orphan_pr 3404 'z-{agent}/3404-voice-demo' \
        'fix(voice #3404): demo' CLEAN 'dialogue_node.py' 3404
    set_state "ISSUE_3404_STATE_JSON" '{"state":"OPEN"}'
    set_state "ISSUE_3404_LABELS_JSON" '{"labels":[{"name":"needs-e2e"}]}'

    run_merge_gate

    local journal
    journal="$(cat "$GH_JOURNAL")"
    assert_eq "0" "$(printf '%s\n' "$journal" | grep -c 'gh pr edit 3404 .*--add-label' || true)" \
        "PR с needs-e2e + живая issue → pr-orphan не трогает" || return 1
    assert_eq "0" "$(printf '%s\n' "$journal" | grep -c 'gh pr edit 3404 .*--remove-label' || true)" \
        "PR с needs-e2e + живая issue → needs-e2e НЕ снимается" || return 1
}

# ===========================================================================
# O5. PR с needs-e2e + UNSTABLE → skip (зона scan-all-prs: карточка + reminder).
# ===========================================================================
test_O5_unstable_skipped() {
    new_test
    fixture_orphan_pr 3405 'z-{agent}/3405-voice-demo' \
        'fix(voice #3405): demo' UNSTABLE 'dialogue_node.py' 3405
    set_state "ISSUE_3405_STATE_JSON" '{"state":"CLOSED"}'
    set_state "ISSUE_3405_LABELS_JSON" '{"labels":[{"name":"e2e-done"}]}'

    run_merge_gate

    local journal
    journal="$(cat "$GH_JOURNAL")"
    assert_eq "0" "$(printf '%s\n' "$journal" | grep -c 'gh pr edit 3405 .*--add-label needs-review' || true)" \
        "UNSTABLE PR → needs-review НЕ ставится (scan-all-prs зона)" || return 1
}

# ===========================================================================
# O6. PR с needs-e2e, head уже влита через ДРУГОЙ PR → skip (зона
#     stale_branch_scan_all, ретро 14.08 t_28afb585).
# ===========================================================================
test_O6_merged_branch_head_skipped() {
    new_test
    fixture_orphan_pr 3406 'z-architect/voice-selection-proposal' \
        'fix(teleop #3406): Dockerfile' CLEAN 'docker/main/teleop/Dockerfile' 3406
    set_state "ISSUE_3406_STATE_JSON" '{"state":"CLOSED"}'
    set_state "ISSUE_3406_LABELS_JSON" '{"labels":[{"name":"e2e-done"}]}'
    # head уже влита через PR #1218 (другой PR) → stale-branch.
    set_state "PR_MERGED_HEAD_z-architect/voice-selection-proposal_JSON" '[{"number":1218,"state":"MERGED"}]'

    run_merge_gate

    local journal
    journal="$(cat "$GH_JOURNAL")"
    assert_eq "0" "$(printf '%s\n' "$journal" | grep -c 'gh pr edit 3406 .*--add-label needs-review' || true)" \
        "PR с needs-e2e на влитой ветке → pr-orphan НЕ ставит needs-review (stale_branch_scan_all)" || return 1
}

# ===========================================================================
# Run all tests.
# ===========================================================================
run_test "O1. CI-only orphan → needs-review" test_O1_ci_only_orphan_gets_needs_review
run_test "O2. functional orphan + OPEN issue → needs-e2e на issue" test_O2_functional_orphan_returns_needs_e2e_to_issue
run_test "O3. functional orphan без issue → needs-review" test_O3_functional_no_issue_gets_needs_review
run_test "O4. живая issue → skip" test_O4_live_issue_skipped
run_test "O5. UNSTABLE → skip" test_O5_unstable_skipped
run_test "O6. влитая ветка → skip" test_O6_merged_branch_head_skipped

summary
