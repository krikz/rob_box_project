#!/bin/bash
# ============================================================================
# test_merge_gate_clean_pr_sweep.sh — CLEAN-не-e2e PR → needs-review
#
# Ретро 13.08 t_a3f170fe (надзор #1194/#1197/#1198): CLEAN/MERGEABLE OPEN PR
# без process-меток выпадали из ВСЕХ путей merge-gate:
#   - основной цикл: только OPEN issues с меткой `hermes` + КАНОНИЧЕСКАЯ
#     ветка z-{agent}/<id>-<slug> → issue CLOSED (#1194/#1198), неканоническая
#     ветка (#1197/#1201), ретро-ветки без issue (#1202) — невидимы;
#   - scan-all-prs: только UNSTABLE/DIRTY/CONFLICTING — CLEAN не трогает;
#   - retro-path: только MERGED PR.
# Итог: PR висит в лимбо с 0 меток, очередь ревью товарища Шифу пуста.
#
# Фикс: clean-PR sweep — сканирует ВСЕ open PR (base=develop, не draft,
# CLEAN/MERGEABLE, без process-меток) и классифицирует:
#   - lint / CI-only (все файлы .github/, scripts/agent_flow/, docs/) →
#     needs-review на PR напрямую (e2e не нужен);
#   - functional + OPEN issue → needs-e2e на issue (стандартный путь);
#   - functional + issue CLOSED/нет → needs-review на PR (e2e невозможен).
#
# Scenarios:
#   C1. CI-only CLEAN PR без меток (docs) → needs-review на PR, needs-e2e НЕ
#       ставится.
#   C2. functional CLEAN PR + OPEN issue → needs-e2e на issue + PR.
#   C3. functional CLEAN PR + issue CLOSED → needs-review на PR (e2e
#       невозможен), close НЕ вызывается.
#   C4. PR с needs-e2e меткой → sweep пропускает (идемпотентность).
#
# Run:
#   bash scripts/agent_flow/tests/test_merge_gate_clean_pr_sweep.sh
# ============================================================================
set -euo pipefail

TEST_LIB_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/mock_env.sh
. "$TEST_LIB_DIR/lib/mock_env.sh"

# ---------------------------------------------------------------------------
# Fixture: clean open PR (CLEAN, no labels) in PR_LIST_ALL_OPEN_JSON + no
# hermes issues. Sweep (как scan-all) читает PR_LIST_ALL_OPEN_JSON.
# $1=pr $2=head $3=title $4=merge_state $5=files_csv $6=pr_labels_csv (optional)
# ---------------------------------------------------------------------------
fixture_clean_pr() {
    local pr="$1" head="$2" title="$3" merge_state="$4" files_csv="$5" pr_labels_csv="${6:-}"
    set_state ISSUE_LIST_JSON '[]'
    local files_json="[]"
    if [ -n "$files_csv" ]; then
        files_json="[$(printf '%s' "$files_csv" | tr ',' '\n' | sed 's/.*/{"path":"&"}/' | paste -sd, -)]"
    fi
    local labels_json="[]"
    if [ -n "$pr_labels_csv" ]; then
        labels_json="[$(printf '%s' "$pr_labels_csv" | tr ',' '\n' | sed 's/.*/{"name":"&"}/' | paste -sd, -)]"
    fi
    set_state PR_LIST_ALL_OPEN_JSON "[{\"number\":${pr},\"headRefName\":\"${head}\",\"title\":\"${title}\",\"mergeStateStatus\":\"${merge_state}\",\"isDraft\":false,\"labels\":${labels_json},\"files\":${files_json}}]"
    set_state RATE_LIMIT_JSON '{"resources":{"core":{"remaining":5000}}}'
    # Нет merged PR с этим head → stale-branch guard молчит.
    set_state "PR_MERGED_HEAD_${head}_JSON" ''
    set_state "BRANCH_PRESENT_${head}" 1
}

# ===========================================================================
# C1. CI-only (docs) CLEAN PR без меток → needs-review на PR.
# ===========================================================================
test_C1_ci_only_clean_pr_gets_needs_review() {
    new_test
    fixture_clean_pr 3301 'z-{agent}/968-tool-intent-based-execution-fire-and-for' \
        'docs(968/w7): план интеграции scheduler (#968)' CLEAN \
        'docs/design/SCHEDULER_DESIGN.md,docs/design/W7_INTEGRATION_PLAN.md'

    run_merge_gate

    local journal
    journal="$(cat "$GH_JOURNAL")"
    assert_eq "1" "$(printf '%s\n' "$journal" | grep -c 'gh pr edit 3301 .*--add-label needs-review' || true)" \
        "CI-only CLEAN PR → needs-review set on PR" || return 1
    assert_eq "0" "$(printf '%s\n' "$journal" | grep -c -- '--add-label needs-e2e' || true)" \
        "CI-only CLEAN PR → needs-e2e NOT set" || return 1
}

# ===========================================================================
# C2. functional CLEAN PR + OPEN issue → needs-e2e на issue + PR.
# ===========================================================================
test_C2_functional_clean_pr_open_issue_gets_needs_e2e() {
    new_test
    local issue=3302 pr=3303
    fixture_clean_pr "$pr" "z-{agent}/${issue}-voice-fix-demo" \
        "fix(voice #${issue}): демо" CLEAN 'dialogue_node.py'
    # Issue OPEN (не CLOSED) — sweep читает state и labels.
    set_state "ISSUE_${issue}_STATE_JSON" '{"state":"OPEN"}'
    set_state "ISSUE_${issue}_LABELS_JSON" '{"labels":[{"name":"hermes"}]}'

    run_merge_gate

    local journal
    journal="$(cat "$GH_JOURNAL")"
    assert_eq "1" "$(printf '%s\n' "$journal" | grep -c "gh issue edit ${issue} --add-label needs-e2e" || true)" \
        "functional CLEAN PR + OPEN issue → needs-e2e on issue" || return 1
    assert_eq "1" "$(printf '%s\n' "$journal" | grep -c "gh pr edit ${pr} .*--add-label needs-e2e" || true)" \
        "functional CLEAN PR + OPEN issue → needs-e2e on PR" || return 1
    assert_eq "0" "$(printf '%s\n' "$journal" | grep -c "gh pr edit ${pr} .*--add-label needs-review" || true)" \
        "functional CLEAN PR + OPEN issue → needs-review NOT set" || return 1
}

# ===========================================================================
# C3. functional CLEAN PR + issue CLOSED → needs-review на PR, close НЕ зовём.
# ===========================================================================
test_C3_functional_clean_pr_closed_issue_gets_needs_review() {
    new_test
    local issue=3304 pr=3305
    fixture_clean_pr "$pr" "z-{agent}/${issue}-voice-fix-demo" \
        "fix(voice #${issue}): демо" CLEAN 'dialogue_node.py'
    set_state "ISSUE_${issue}_STATE_JSON" '{"state":"CLOSED"}'
    set_state "ISSUE_${issue}_LABELS_JSON" '{"labels":[]}'

    run_merge_gate

    local journal
    journal="$(cat "$GH_JOURNAL")"
    assert_eq "1" "$(printf '%s\n' "$journal" | grep -c "gh pr edit ${pr} .*--add-label needs-review" || true)" \
        "functional CLEAN PR + CLOSED issue → needs-review on PR" || return 1
    assert_eq "0" "$(printf '%s\n' "$journal" | grep -c -- '--add-label needs-e2e' || true)" \
        "functional CLEAN PR + CLOSED issue → needs-e2e NOT set" || return 1
    assert_eq "0" "$(printf '%s\n' "$journal" | grep -c "gh issue close" || true)" \
        "functional CLEAN PR + CLOSED issue → НЕ закрываем issue (решение Шифу)" || return 1
}

# ===========================================================================
# C4. PR с process-меткой (needs-e2e) → sweep пропускает (идемпотентность).
# ===========================================================================
test_C4_labeled_pr_skipped() {
    new_test
    fixture_clean_pr 3306 'z-{agent}/3306-voice-demo' \
        'fix(voice #3306): demo' CLEAN 'dialogue_node.py' 'needs-e2e'

    run_merge_gate

    local journal
    journal="$(cat "$GH_JOURNAL")"
    assert_eq "0" "$(printf '%s\n' "$journal" | grep -c 'gh pr edit 3306 .*--add-label' || true)" \
        "PR с needs-e2e → sweep не трогает" || return 1
}

# ===========================================================================
# C5. CLEAN PR без меток, НО head-ветка уже влита через ДРУГОЙ PR → sweep
#     НЕ ставит needs-review (переиспользование ветки влитого PR, ретро
#     14.08 t_28afb585 — сценарий #1238/#1218). Обрабатывает
#     stale_branch_scan_all, а не clean-pr-sweep.
# ===========================================================================
test_C5_merged_branch_head_pr_skipped() {
    new_test
    fixture_clean_pr 3307 'z-architect/voice-selection-proposal' \
        'fix(teleop #3307): Dockerfile BASE_IMAGE' CLEAN 'docker/main/teleop/Dockerfile'
    # head-ветка уже влита через PR #1218 (другой PR) → stale-branch.
    set_state "PR_MERGED_HEAD_z-architect/voice-selection-proposal_JSON" '[{"number":1218,"state":"MERGED"}]'

    run_merge_gate

    local journal
    journal="$(cat "$GH_JOURNAL")"

    local review_calls
    review_calls="$(printf '%s\n' "$journal" | grep -c 'gh pr edit 3307 .*--add-label needs-review' || true)"
    assert_eq "0" "$review_calls" "CLEAN PR с head на влитой ветке → needs-review NOT set (обрабатывает stale_branch_scan_all)" || return 1

    local e2e_calls
    e2e_calls="$(printf '%s\n' "$journal" | grep -c -- '--add-label needs-e2e' || true)"
    assert_eq "0" "$e2e_calls" "CLEAN PR с head на влитой ветке → needs-e2e NOT set" || return 1
}

# ===========================================================================
# Run all tests.
# ===========================================================================
run_test "C1. CI-only CLEAN PR → needs-review" test_C1_ci_only_clean_pr_gets_needs_review
run_test "C2. functional + OPEN issue → needs-e2e" test_C2_functional_clean_pr_open_issue_gets_needs_e2e
run_test "C3. functional + CLOSED issue → needs-review" test_C3_functional_clean_pr_closed_issue_gets_needs_review
run_test "C4. labeled PR → skipped" test_C4_labeled_pr_skipped
run_test "C5. merged-branch head PR → skipped" test_C5_merged_branch_head_pr_skipped

summary
