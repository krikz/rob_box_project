#!/bin/bash
# ============================================================================
# test_merge_gate_pr_backfill.sh — REST-based backfill scan (ретро 15.08 t_2c814334)
#
# Инцидент: PR #1282/#1284/#1286 (process-only фиксы, CI green) созданы
# 07:38-07:55Z в peak-окно и висели без меток 5.5ч — merge-gate их не
# разметил. Причина: clean-pr-sweep/pr-orphan-reconcile используют `gh pr
# list` (GraphQL); при graphql rate-limit=0 (а core жив — квоты РАЗНЫЕ) они
# МОЛЧА возвращают [] → merge-gate «слепо» сканирует пустоту.
#
# Фикс: REST-based pr-backfill-scan (gh api pulls — core-квота) для open PR
# без process-меток старше BACKFILL_AGE_MINUTES=30 мин:
#   - process-only (все файлы .github/, scripts/agent_flow/, docs/) →
#     needs-review на PR (e2e не нужен);
#   - functional + OPEN issue → needs-e2e на issue (+ PR);
#   - functional + issue CLOSED/нет → needs-review на PR (e2e невозможен).
# G3: при graphql=0 лог-предупреждение, тик НЕ прерывается (REST продолжит).
#
# Scenarios:
#   B1. graphql=0 + core жив + process-only REST PR старше 30 мин →
#       needs-review на PR (главный сценарий инцидента).
#   B2. Свежий PR (< 30 мин) → НЕ размечается (возрастной порог).
#   B3. functional REST PR + OPEN issue → needs-e2e на issue + PR.
#   B4. REST PR с process-меткой → пропускается (идемпотентность).
#   B5. G3 лог graphql=0 (тик продолжается, backfill работает).
#
# Run:
#   bash scripts/agent_flow/tests/test_merge_gate_pr_backfill.sh
# ============================================================================
set -euo pipefail

TEST_LIB_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/mock_env.sh
. "$TEST_LIB_DIR/lib/mock_env.sh"

# ---------------------------------------------------------------------------
# Fixture: open REST PR (clean, mergeable, created X min ago) + graphql=0.
# $1=pr $2=head $3=title $4=created_iso $5=mergeable_state $6=pr_labels_csv
# $7=files_csv (optional) $8=mergeable (optional, default true)
# ---------------------------------------------------------------------------
fixture_backfill_pr() {
    local pr="$1" head="$2" title="$3" created="$4" merge_state="$5" pr_labels_csv="${6:-}" files_csv="${7:-}" mergeable="${8:-true}"
    set_state ISSUE_LIST_JSON '[]'
    # GraphQL-пути «слепы» (rate-limit=0) — gh pr list возвращает []:
    # ровно сценарий инцидента (G3 раньше смотрел только core и молча
    # продолжал со слепыми GraphQL-сканами). PR_LIST_ALL_OPEN_JSON — это
    # ответ GraphQL `gh pr list --state open` (scan-all-prs / clean-pr-sweep
    # читают его) → пустой.
    set_state PR_LIST_ALL_OPEN_JSON '[]'
    # graphql=0, core жив — ровно сценарий инцидента (G3 раньше смотрел
    # только core и молча продолжал со слепыми GraphQL-сканами).
    set_state RATE_LIMIT_JSON '{"resources":{"core":{"remaining":5000},"graphql":{"remaining":0}}}'
    local labels_json="[]"
    if [ -n "$pr_labels_csv" ]; then
        labels_json="[$(printf '%s' "$pr_labels_csv" | tr ',' '\n' | sed 's/.*/{"name":"&"}/' | paste -sd, -)]"
    fi
    # REST-объект pull (head.ref, mergeable, mergeable_state, labels, created_at).
    set_state PR_LIST_ALL_OPEN_REST_JSON "[{\"number\":${pr},\"title\":\"${title}\",\"head\":{\"ref\":\"${head}\"},\"mergeable\":${mergeable},\"mergeable_state\":\"${merge_state}\",\"draft\":false,\"labels\":${labels_json},\"created_at\":\"${created}\"}]"
    # Файлы PR — отдельный REST-вызов pulls/N/files (fixture PR_<n>_FILES_JSON).
    if [ -n "$files_csv" ]; then
        local files_json="[$(printf '%s' "$files_csv" | tr ',' '\n' | sed 's/.*/{"filename":"&","sha":"abc"}/' | paste -sd, -)]"
        set_state "PR_${pr}_FILES_JSON" "$files_json"
    else
        set_state "PR_${pr}_FILES_JSON" '[]'
    fi
    # Нет merged PR с этим head → stale-branch guard молчит.
    set_state "PR_MERGED_HEAD_${head}_JSON" ''
    set_state "BRANCH_PRESENT_${head}" 1
}

iso_minutes_ago() {  # $1=minutes
    date -u -d "$1 minutes ago" +%Y-%m-%dT%H:%M:%SZ 2>/dev/null \
        || python3 -c "from datetime import datetime,timedelta,timezone; print((datetime.now(timezone.utc)-timedelta(minutes=$1)).strftime('%Y-%m-%dT%H:%M:%SZ'))"
}

# ===========================================================================
# B1. graphql=0 + process-only REST PR → needs-review на PR (главный кейс).
# ===========================================================================
test_B1_graphql_dead_process_only_pr_gets_needs_review() {
    new_test
    # fix(agent-flow...) + только scripts/agent_flow/ → lint → needs-review.
    fixture_backfill_pr 3401 'z-devops/t_16325ddd-pr-state-race-stale-cards' \
        'fix(agent-flow nadzor): merge-gate/e2e-fail (ретро t_16325ddd)' \
        "$(iso_minutes_ago 90)" clean '' \
        'scripts/agent_flow/agent-flow-merge-gate.sh,scripts/agent_flow/agent-flow-e2e-process.sh'

    run_merge_gate

    local journal
    journal="$(cat "$GH_JOURNAL")"
    assert_eq "1" "$(printf '%s\n' "$journal" | grep -c 'gh pr edit 3401 .*--add-label needs-review' || true)" \
        "graphql=0 + process-only REST PR → needs-review set on PR" || return 1
    assert_eq "0" "$(printf '%s\n' "$journal" | grep -c -- '--add-label needs-e2e' || true)" \
        "process-only REST PR → needs-e2e NOT set" || return 1
}

# ===========================================================================
# B2. Свежий PR (< 30 мин) → НЕ размечается (возрастной порог).
# ===========================================================================
test_B2_fresh_pr_skipped() {
    new_test
    fixture_backfill_pr 3402 'z-devops/t_99999999-fresh-fix' \
        'fix(agent-flow): свежий фикс (ретро t_99999999)' \
        "$(iso_minutes_ago 5)" clean '' \
        'scripts/agent_flow/agent-flow-merge-gate.sh'

    run_merge_gate

    local journal
    journal="$(cat "$GH_JOURNAL")"
    assert_eq "0" "$(printf '%s\n' "$journal" | grep -c 'gh pr edit 3402 .*--add-label' || true)" \
        "свежий PR (<30 мин) → не размечается" || return 1
}

# ===========================================================================
# B3. functional REST PR + OPEN issue → needs-e2e на issue + PR.
# ===========================================================================
test_B3_functional_rest_pr_open_issue_gets_needs_e2e() {
    new_test
    local issue=3403 pr=3404
    fixture_backfill_pr "$pr" "z-{agent}/${issue}-voice-fix-demo" \
        "fix(voice #${issue}): демо" "$(iso_minutes_ago 60)" clean '' 'dialogue_node.py'
    set_state "ISSUE_${issue}_STATE_JSON" '{"state":"OPEN"}'
    set_state "ISSUE_${issue}_LABELS_JSON" '{"labels":[{"name":"hermes"}]}'

    run_merge_gate

    local journal
    journal="$(cat "$GH_JOURNAL")"
    assert_eq "1" "$(printf '%s\n' "$journal" | grep -c "gh issue edit ${issue} --add-label needs-e2e" || true)" \
        "functional REST PR + OPEN issue → needs-e2e on issue" || return 1
    assert_eq "1" "$(printf '%s\n' "$journal" | grep -c "gh pr edit ${pr} .*--add-label needs-e2e" || true)" \
        "functional REST PR + OPEN issue → needs-e2e on PR" || return 1
}

# ===========================================================================
# B4. REST PR с process-меткой → пропускается (идемпотентность).
# ===========================================================================
test_B4_labeled_rest_pr_skipped() {
    new_test
    fixture_backfill_pr 3405 'z-devops/t_55555555-labeled' \
        'fix(agent-flow): уже размечен (ретро t_55555555)' \
        "$(iso_minutes_ago 60)" clean 'needs-review' \
        'scripts/agent_flow/agent-flow-merge-gate.sh'

    run_merge_gate

    local journal
    journal="$(cat "$GH_JOURNAL")"
    assert_eq "0" "$(printf '%s\n' "$journal" | grep -c 'gh pr edit 3405 .*--add-label' || true)" \
        "PR с process-меткой → backfill не трогает" || return 1
}

# ===========================================================================
# B5. G3 лог graphql=0 — тик продолжается (backfill сработал), не exit.
# ===========================================================================
test_B5_g3_logs_graphql_warning_and_continues() {
    new_test
    fixture_backfill_pr 3406 'z-devops/t_66666666-g3-warn' \
        'fix(agent-flow): G3 warn (ретро t_66666666)' \
        "$(iso_minutes_ago 120)" clean '' \
        'scripts/agent_flow/agent-flow-merge-gate.sh'

    run_merge_gate

    local stderr_log
    stderr_log="$(cat "$TEST_TMP/stderr.log" 2>/dev/null || true)"
    assert_contains "GraphQL rate-limit exhausted" "$stderr_log" \
        "G3 логирует graphql=0 предупреждение" || return 1
    local journal
    journal="$(cat "$GH_JOURNAL")"
    assert_eq "1" "$(printf '%s\n' "$journal" | grep -c 'gh pr edit 3406 .*--add-label needs-review' || true)" \
        "тик НЕ прерван graphql=0 — backfill разметил PR" || return 1
}

# ===========================================================================
# Run all tests.
# ===========================================================================
run_test "B1. graphql=0 + process-only REST PR → needs-review" test_B1_graphql_dead_process_only_pr_gets_needs_review
run_test "B2. свежий PR (<30 мин) → skip" test_B2_fresh_pr_skipped
run_test "B3. functional + OPEN issue → needs-e2e" test_B3_functional_rest_pr_open_issue_gets_needs_e2e
run_test "B4. PR с меткой → skip" test_B4_labeled_rest_pr_skipped
run_test "B5. G3 лог graphql=0, тик продолжается" test_B5_g3_logs_graphql_warning_and_continues

summary
