#!/bin/bash
# ============================================================================
# test_merge_gate_retro_path_orphan_needs_e2e.sh
#   Ретро 01.09 t_365de06c: needs-e2e orphan cleanup (post-merge, без hermes).
#
# ПРОБЛЕМА (наблюдение архитектора 01.09 ~07:10Z, issue #1824):
#   triage создал карточку t_e1a9613d (done) для issue #1824; PR #1843 залит
#   в develop, но issue до сих пор OPEN с одной меткой `needs-e2e` (БЕЗ
#   `hermes`). Причины: (a) triage пометил, потом пользователь/процесс
#   снял hermes; (b) hermes никогда не выставлялся. В обоих случаях:
#     - main-cycle (по hermes) issue НЕ видит;
#     - retro-path skip'ает как «уже в process-цикле» (anti-loop guard);
#     - e2e-process не может подобрать (нет живой PR-ветки или ветка уже
#       влита) → зависшее состояние.
#
# РЕШЕНИЕ (process-fix, ретро-key merge-gate-no-needs-e2e-cleanup-after-merge):
#   В retro-path, после стандартного skip-блока (4125-4136), отдельная
#   ветка для orphan: `has(needs-e2e) && !has(hermes) && state=OPEN` →
#     - снять needs-e2e (orphan больше не претендует на e2e-ротацию);
#     - тот же PASS-check (e2e run SUCCESS / CI-only + green CI):
#       * PASS → close + comment «post-merge needs-e2e cleanup, retro-path»
#         (reason «post-merge needs-e2e cleanup, retro-path»);
#       * no PASS → оставить issue как есть (audit-коммент «merge без PASS,
#         ручной разбор» — issue вернётся в орбиту юзера, e2e-процесс её
#         больше НЕ возьмёт).
#
# ВАЖНО: не снимать needs-e2e при наличии hermes — это territory e2e-process
# (test_O_retro_hermes_with_needs_e2e_still_skips в test_merge_gate_retro_path.sh).
#
# Scenarios:
#   OR1. needs-e2e без hermes + e2e run SUCCESS → снять needs-e2e + close
#        + comment «post-merge needs-e2e cleanup, retro-path» (e2e PASS).
#   OR2. needs-e2e без hermes + CI-only PR + green CI → снять needs-e2e +
#        close (e2e не нужен).
#   OR3. needs-e2e без hermes + no PASS → снять needs-e2e + audit-коммент
#        (НЕ close, НЕ needs-e2e re-add).
#   OR4. (regression guard) needs-e2e + hermes → всё ещё skip
#        (test_O_retro_hermes_with_needs_e2e_still_skips — основной
#        skip-блок 4125-4136 должен срабатывать первым).
#
# Run:
#   bash scripts/agent_flow/tests/test_merge_gate_retro_path_orphan_needs_e2e.sh
# ============================================================================
set -euo pipefail

TEST_LIB_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/mock_env.sh
. "$TEST_LIB_DIR/lib/mock_env.sh"

# ---------------------------------------------------------------------------
# Fixture для orphan-cleanup: merged PR ссылается на issue с needs-e2e без
# hermes (state=OPEN). PASS-доказательство настраивается вызывающим.
#   $1=issue, $2=pr, $3=head, $4=mergedAt
# ---------------------------------------------------------------------------
fixture_orphan_needs_e2e() {
    local issue="$1" pr="$2" head="$3" merged_at="$4"
    set_state ISSUE_LIST_JSON '[]'  # main-cycle пуст (нет hermes → не видит).
    set_state PR_LIST_MERGED_JSON "[{\"number\":${pr},\"title\":\"fix #${issue} orphan\",\"body\":\"closes #${issue}\n\",\"headRefName\":\"${head}\",\"mergedAt\":\"${merged_at}\"}]"
    # needs-e2e БЕЗ hermes — зависший orphan.
    set_state "ISSUE_${issue}_LABELS_JSON" '{"labels":[{"name":"needs-e2e"}]}'
    set_state "ISSUE_${issue}_STATE_JSON" '{"state":"OPEN"}'
    set_state "ISSUE_${issue}_COMMENTS_JSON" '{"comments":[]}'
    set_state "ISSUE_${issue}_COMMENTS_SINCE_JSON" '[]'
    set_state "ISSUE_${issue}_TIMELINE_JSON" '[]'
    set_state PR_LIST_ALL_OPEN_JSON '[]'
    set_state PR_FOLLOWUP_JSON '[]'
    set_state RATE_LIMIT_JSON '{"resources":{"core":{"remaining":5000}}}'
}

# ===========================================================================
# OR1. needs-e2e без hermes + e2e run SUCCESS → снять needs-e2e + close.
#      Аналог кейса issue #1824 (если бы e2e на ветке был).
# ===========================================================================
test_OR1_orphan_needs_e2e_e2e_pass_closes() {
    new_test
    local issue=1824 pr=1843 head='z-agent/1824-fix-triage-unknown-assignee-rollup'
    fixture_orphan_needs_e2e "$issue" "$pr" "$head" '2026-09-01T04:42:23Z'
    # e2e run SUCCESS на ветке merged PR → PASS-доказательство есть.
    set_state "RUN_LIST_${head}_JSON" '[{"conclusion":"success"}]'

    run_merge_gate
    local journal
    journal="$(cat "$GH_JOURNAL")"

    # needs-e2e снят (orphan больше не претендует на e2e).
    local remove_needs_e2e
    remove_needs_e2e="$(printf '%s\n' "$journal" | grep -c "gh issue edit ${issue} --remove-label needs-e2e" || true)"
    assert_eq "1" "$remove_needs_e2e" "needs-e2e removed from orphan before close" || return 1

    # Issue закрыта с reason completed.
    local close_calls
    close_calls="$(printf '%s\n' "$journal" | grep -c "gh issue close ${issue} --reason completed" || true)"
    assert_eq "1" "$close_calls" "orphan needs-e2e + e2e PASS → close" || return 1

    # Audit-комментарий «post-merge needs-e2e cleanup, retro-path» опубликован.
    local audit_comment
    audit_comment="$(printf '%s\n' "$journal" | grep -c 'post-merge needs-e2e cleanup, retro-path' || true)"
    assert_eq "1" "$audit_comment" "audit comment with retro-path reason present" || return 1

    # needs-e2e НЕ ставится повторно.
    local add_needs_e2e
    add_needs_e2e="$(printf '%s\n' "$journal" | grep -c "gh issue edit ${issue} --add-label needs-e2e" || true)"
    assert_eq "0" "$add_needs_e2e" "needs-e2e NOT re-added (no loop)" || return 1

    # State flips to CLOSED.
    local state_now
    state_now="$(grep -E "^ISSUE_${issue}_STATE_JSON=" "$GH_STATE" | sed "s/^ISSUE_${issue}_STATE_JSON=//")"
    assert_contains '"CLOSED"' "$state_now" "issue state flipped to CLOSED" || return 1
}

# ===========================================================================
# OR2. needs-e2e без hermes + CI-only PR (.github/) + green CI → close.
#      Кейс process-fix: фикс в merge-gate сам по себе — CI-only.
# ===========================================================================
test_OR2_orphan_needs_e2e_ci_only_green_closes() {
    new_test
    local issue=1824 pr=1843 head='z-devops/t_365de06c-orphan-cleanup'
    fixture_orphan_needs_e2e "$issue" "$pr" "$head" '2026-09-01T05:00:00Z'
    # e2e run нет — fallback на CI-only.
    set_state "RUN_LIST_${head}_JSON" '[]'
    # PR меняет только scripts/agent_flow/ → CI-only.
    set_state "PR_${pr}_FILES_JSON" '{"files":[{"path":"scripts/agent_flow/agent-flow-merge-gate.sh"},{"path":"scripts/agent_flow/tests/test_merge_gate_retro_path_orphan_needs_e2e.sh"}]}'
    # CI зелёный.
    set_state "PR_${pr}_ROLLUP_JSON" '{"statusCheckRollup":[{"conclusion":"SUCCESS"}]}'

    run_merge_gate
    local journal
    journal="$(cat "$GH_JOURNAL")"

    local close_calls
    close_calls="$(printf '%s\n' "$journal" | grep -c "gh issue close ${issue} --reason completed" || true)"
    assert_eq "1" "$close_calls" "orphan needs-e2e + CI-only green → close" || return 1

    local remove_needs_e2e
    remove_needs_e2e="$(printf '%s\n' "$journal" | grep -c "gh issue edit ${issue} --remove-label needs-e2e" || true)"
    assert_eq "1" "$remove_needs_e2e" "needs-e2e removed before close" || return 1
}

# ===========================================================================
# OR3. needs-e2e без hermes + no PASS (функциональный код, нет e2e SUCCESS) →
#      снять needs-e2e + audit-коммент. Close НЕ вызывается (нет PASS).
#      Это основной «спасательный» сценарий: issue #1824 как раз такой
#      (функциональный код робота, e2e не было).
# ===========================================================================
test_OR3_orphan_needs_e2e_no_pass_audit() {
    new_test
    local issue=1824 pr=1843 head='z-agent/1824-fix-triage-unknown-assignee-rollup'
    fixture_orphan_needs_e2e "$issue" "$pr" "$head" '2026-09-01T04:42:23Z'
    # e2e run нет.
    set_state "RUN_LIST_${head}_JSON" '[]'
    # PR меняет код робота (НЕ CI-only) + CI green, но для orphan-CI-only не
    # работает, e2e run нет → PASS нет.
    set_state "PR_${pr}_FILES_JSON" '{"files":[{"path":"scripts/agent_flow/agent-flow-triage.sh"}]}'
    set_state "PR_${pr}_ROLLUP_JSON" '{"statusCheckRollup":[{"conclusion":"SUCCESS"}]}'

    run_merge_gate
    local journal
    journal="$(cat "$GH_JOURNAL")"

    # needs-e2e снят (orphan вышел из e2e-ротации).
    local remove_needs_e2e
    remove_needs_e2e="$(printf '%s\n' "$journal" | grep -c "gh issue edit ${issue} --remove-label needs-e2e" || true)"
    assert_eq "1" "$remove_needs_e2e" "needs-e2e removed from orphan (no PASS)" || return 1

    # Audit-коммент про ручной разбор (НЕ про PASS-cleanup).
    local audit_comment
    audit_comment="$(printf '%s\n' "$journal" | grep -c 'merge без PASS\|нужен ручной разбор' || true)"
    assert_eq "1" "$audit_comment" "audit comment for orphan no-PASS published" || return 1

    # Close НЕ вызывается (нет PASS-доказательства).
    local close_calls
    close_calls="$(printf '%s\n' "$journal" | grep -c "gh issue close ${issue}" || true)"
    assert_eq "0" "$close_calls" "no close without PASS evidence" || return 1

    # State остаётся OPEN (issue вернётся в орбиту юзера для ручного разбора).
    local state_now
    state_now="$(grep -E "^ISSUE_${issue}_STATE_JSON=" "$GH_STATE" | sed "s/^ISSUE_${issue}_STATE_JSON=//")"
    assert_contains '"OPEN"' "$state_now" "issue stays OPEN (manual review needed)" || return 1
}

# ===========================================================================
# OR4. Regression guard: needs-e2e + hermes → всё ещё skip (e2e-process owns).
#      Это уже покрыто test_O_retro_hermes_with_needs_e2e_still_skips в
#      test_merge_gate_retro_path.sh; здесь — отдельная копия для локализации
#      отказа: если orphan-cleanup зацепит hermes-issue, тест упадёт.
# ===========================================================================
test_OR4_regression_hermes_needs_e2e_still_skipped() {
    new_test
    local issue=1421 pr=1425 head='z-devops/t_8d6b7268-fix-scenario-file'
    set_state ISSUE_LIST_JSON '[]'
    set_state PR_LIST_MERGED_JSON "[{\"number\":${pr},\"title\":\"fix #${issue}\",\"body\":\"closes #${issue}\",\"headRefName\":\"${head}\",\"mergedAt\":\"2026-08-18T17:51:37Z\"}]"
    # hermes + needs-e2e — основной skip-блок должен сработать первым.
    set_state "ISSUE_${issue}_LABELS_JSON" '{"labels":[{"name":"hermes"},{"name":"needs-e2e"}]}'
    set_state "ISSUE_${issue}_STATE_JSON" '{"state":"OPEN"}'
    set_state "ISSUE_${issue}_COMMENTS_JSON" '{"comments":[]}'
    set_state "ISSUE_${issue}_COMMENTS_SINCE_JSON" '[]'
    set_state "ISSUE_${issue}_TIMELINE_JSON" '[]'
    # Даже если бы orphan-cleanup добрался, PASS-доказательство есть —
    # НЕ должно сработать.
    set_state "RUN_LIST_${head}_JSON" '[{"conclusion":"success"}]'
    set_state PR_LIST_ALL_OPEN_JSON '[]'
    set_state PR_FOLLOWUP_JSON '[]'
    set_state RATE_LIMIT_JSON '{"resources":{"core":{"remaining":5000}}}'

    run_merge_gate
    local journal
    journal="$(cat "$GH_JOURNAL")"

    # needs-e2e НЕ снимается (это territory e2e-process).
    local remove_needs_e2e
    remove_needs_e2e="$(printf '%s\n' "$journal" | grep -c "gh issue edit ${issue} --remove-label needs-e2e" || true)"
    assert_eq "0" "$remove_needs_e2e" "hermes+needs-e2e NOT touched by orphan-cleanup" || return 1

    # Close не вызывается.
    local close_calls
    close_calls="$(printf '%s\n' "$journal" | grep -c "gh issue close ${issue}" || true)"
    assert_eq "0" "$close_calls" "hermes+needs-e2e NOT closed by orphan-cleanup" || return 1
}

# ===========================================================================
# Run
# ===========================================================================
run_test "OR1. orphan needs-e2e + e2e PASS → remove needs-e2e + close" test_OR1_orphan_needs_e2e_e2e_pass_closes
run_test "OR2. orphan needs-e2e + CI-only green → remove needs-e2e + close" test_OR2_orphan_needs_e2e_ci_only_green_closes
run_test "OR3. orphan needs-e2e + no PASS → remove needs-e2e + audit, no close" test_OR3_orphan_needs_e2e_no_pass_audit
run_test "OR4. regression: hermes+needs-e2e still skipped (orphan-cleanup не цепляет)" test_OR4_regression_hermes_needs_e2e_still_skipped

summary
