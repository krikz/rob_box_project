#!/bin/bash
# ============================================================================
# test_merge_gate_retro_path.sh — ретро-путь (12.08 t_68607832)
#
# Сценарии (ADR-0014 gap):
#   A. merged PR ссылается на issue БЕЗ меток + e2e run SUCCESS на ветке PR
#      → issue закрывается (ретро-путь).
#   B. merged PR ссылается на issue БЕЗ меток, e2e нет, но PR CI-only (только
#      .github/scripts/docs) и CI зелёный → issue закрывается (e2e не нужен).
#   C. merged PR ссылается на issue БЕЗ меток, PASS-доказательства нет →
#      ставится needs-e2e (e2e-process возьмёт в ротацию), close НЕ вызывается.
#   D. merged PR ссылается на issue, но у issue уже есть needs-e2e → skip
#      (не закрываем, не трогаем).
#   E. merged PR ссылается на issue, но issue уже CLOSED → skip.
#   F. merged PR старше окна (RETRO_MERGED_DAYS) → не рассматривается.
#   G. self-reference: PR ссылается на свой собственный номер (#1142 в body)
#      → не считается issue-reference.
#   H. (12.08 t_061d466e) issue с e2e:rejected + merged CI-only PR с зелёным
#      CI → e2e:rejected снимается, issue закрывается (петля #1041).
#   I. (12.08 t_061d466e) issue с e2e:rejected + merged PR, но PASS-
#      доказательства НЕТ → needs-e2e НЕ ставится (иначе e2e-process
#      зациклится), rejected остаётся, close не вызывается.
#
# Run:
#   bash scripts/agent_flow/tests/test_merge_gate_retro_path.sh
# ============================================================================
set -euo pipefail

TEST_LIB_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/mock_env.sh
. "$TEST_LIB_DIR/lib/mock_env.sh"

# ---------------------------------------------------------------------------
# Helper: базовый fixture для ретро-пути.
# $1=issue_number, $2=pr_number, $3=head_branch, $4=mergedAt
# По умолчанию: issue OPEN без меток, merged PR ссылается на issue,
# e2e run SUCCESS на ветке (evidence). Тест может переопределить state.
# ---------------------------------------------------------------------------
fixture_retro() {  # $1=issue $2=pr $3=head $4=mergedAt
    local issue="$1" pr="$2" head="$3" merged_at="$4"
    # Нет hermes-issues: основной цикл пуст, работает только ретро-путь.
    set_state ISSUE_LIST_JSON '[]'
    set_state PR_LIST_MERGED_JSON "[{\"number\":${pr},\"title\":\"fix #${issue} retro\",\"body\":\"closes #${issue}\\n\",\"headRefName\":\"${head}\",\"mergedAt\":\"${merged_at}\"}]"
    set_state "ISSUE_${issue}_LABELS_JSON" '{"labels":[]}'
    set_state "ISSUE_${issue}_STATE_JSON" '{"state":"OPEN"}'
    set_state "ISSUE_${issue}_COMMENTS_JSON" '{"comments":[]}'
    set_state "ISSUE_${issue}_COMMENTS_SINCE_JSON" '[]'
    set_state "ISSUE_${issue}_TIMELINE_JSON" '[]'
    set_state "RUN_LIST_${head}_JSON" '[{"conclusion":"success"}]'
    set_state PR_LIST_ALL_OPEN_JSON '[]'
    set_state PR_FOLLOWUP_JSON '[]'
    set_state RATE_LIMIT_JSON '{"resources":{"core":{"remaining":5000}}}'
}

# ===========================================================================
# A. merged PR + issue без меток + e2e run SUCCESS → close.
# ===========================================================================
test_A_retro_e2e_pass_closes() {
    new_test
    fixture_retro 1138 1143 'z-devops/t_4e592534-e2e-validator-fix' '2026-08-12T14:14:05Z'

    run_merge_gate
    local journal
    journal="$(cat "$GH_JOURNAL")"

    local close_calls
    close_calls="$(printf '%s\n' "$journal" | grep -c 'gh issue close 1138 --reason completed' || true)"
    assert_eq "1" "$close_calls" "retro-path closes unlabeled issue with e2e PASS evidence"

    # Комментарий с доказательством опубликован.
    local evidence_comment
    evidence_comment="$(printf '%s\n' "$journal" | grep -c '✅ ретро-путь' || true)"
    assert_eq "1" "$evidence_comment" "retro-path publishes evidence comment"

    # needs-e2e НЕ ставится (PASS есть — сразу close).
    local add_needs_e2e
    add_needs_e2e="$(printf '%s\n' "$journal" | grep -c 'gh issue edit 1138 --add-label needs-e2e' || true)"
    assert_eq "0" "$add_needs_e2e" "no needs-e2e when PASS evidence closes"

    # State flips to CLOSED.
    local state_now
    state_now="$(grep -E '^ISSUE_1138_STATE_JSON=' "$GH_STATE" | sed 's/^ISSUE_1138_STATE_JSON=//')"
    assert_contains '"CLOSED"' "$state_now" "issue state flipped to CLOSED"
}

# ===========================================================================
# B. merged PR + issue без меток + CI-only PR с зелёным CI (e2e не нужен)
#    → close.
# ===========================================================================
test_B_retro_ci_only_green_closes() {
    new_test
    local issue=1139 pr=1142 head='z-devops/t_cd9ea383-sha-tags-no-commit'
    set_state ISSUE_LIST_JSON '[]'
    set_state PR_LIST_MERGED_JSON "[{\"number\":${pr},\"title\":\"fix(ci): SHA-теги (#${issue})\",\"body\":\"closes #${issue}\",\"headRefName\":\"${head}\",\"mergedAt\":\"2026-08-12T14:14:40Z\"}]"
    set_state "ISSUE_${issue}_LABELS_JSON" '{"labels":[]}'
    set_state "ISSUE_${issue}_STATE_JSON" '{"state":"OPEN"}'
    set_state "ISSUE_${issue}_COMMENTS_JSON" '{"comments":[]}'
    set_state "ISSUE_${issue}_COMMENTS_SINCE_JSON" '[]'
    set_state "ISSUE_${issue}_TIMELINE_JSON" '[]'
    # e2e run НЕТ (пусто) — fallback на CI-only.
    set_state "RUN_LIST_${head}_JSON" '[]'
    # PR меняет только .github/ → CI-only.
    set_state "PR_${pr}_FILES_JSON" '{"files":[{"path":".github/workflows/L-Build Main Pi Services.yml"}]}'
    # CI зелёный: нет FAILURE/CANCELLED/TIMED_OUT.
    set_state "PR_${pr}_ROLLUP_JSON" '{"statusCheckRollup":[{"conclusion":"SUCCESS"},{"conclusion":"SUCCESS"}]}'
    set_state PR_LIST_ALL_OPEN_JSON '[]'
    set_state PR_FOLLOWUP_JSON '[]'
    set_state RATE_LIMIT_JSON '{"resources":{"core":{"remaining":5000}}}'

    run_merge_gate
    local journal
    journal="$(cat "$GH_JOURNAL")"

    local close_calls
    close_calls="$(printf '%s\n' "$journal" | grep -c "gh issue close ${issue} --reason completed" || true)"
    assert_eq "1" "$close_calls" "retro-path closes CI-only issue with green CI (e2e not required)"
}

# ===========================================================================
# C. merged PR + issue без меток + НЕТ PASS-доказательства → needs-e2e,
#    close НЕ вызывается.
# ===========================================================================
test_C_retro_no_evidence_labels_needs_e2e() {
    new_test
    local issue=2001 pr=2002 head='z-devops/t_2001-no-evidence'
    set_state ISSUE_LIST_JSON '[]'
    set_state PR_LIST_MERGED_JSON "[{\"number\":${pr},\"title\":\"fix #${issue} unverified\",\"body\":\"closes #${issue}\",\"headRefName\":\"${head}\",\"mergedAt\":\"2026-08-12T14:14:40Z\"}]"
    set_state "ISSUE_${issue}_LABELS_JSON" '{"labels":[]}'
    set_state "ISSUE_${issue}_STATE_JSON" '{"state":"OPEN"}'
    set_state "ISSUE_${issue}_COMMENTS_JSON" '{"comments":[]}'
    set_state "ISSUE_${issue}_COMMENTS_SINCE_JSON" '[]'
    set_state "ISSUE_${issue}_TIMELINE_JSON" '[]'
    set_state "RUN_LIST_${head}_JSON" '[]'  # e2e нет
    # PR меняет код робота (не CI-only) → CI green не считается PASS для e2e.
    set_state "PR_${pr}_FILES_JSON" '{"files":[{"path":"src/robot/voice.py"}]}'
    set_state "PR_${pr}_ROLLUP_JSON" '{"statusCheckRollup":[{"conclusion":"SUCCESS"}]}'
    set_state PR_LIST_ALL_OPEN_JSON '[]'
    set_state PR_FOLLOWUP_JSON '[]'
    set_state RATE_LIMIT_JSON '{"resources":{"core":{"remaining":5000}}}'

    run_merge_gate
    local journal
    journal="$(cat "$GH_JOURNAL")"

    local add_needs_e2e
    add_needs_e2e="$(printf '%s\n' "$journal" | grep -c "gh issue edit ${issue} --add-label needs-e2e" || true)"
    assert_eq "1" "$add_needs_e2e" "no PASS evidence → needs-e2e added (e2e-process takes over)"

    local close_calls
    close_calls="$(printf '%s\n' "$journal" | grep -c "gh issue close ${issue}" || true)"
    assert_eq "0" "$close_calls" "no close without PASS evidence"
}

# ===========================================================================
# D. issue уже имеет needs-e2e → ретро-путь skip (не закрывает, не дублирует).
# ===========================================================================
test_D_retro_skips_labeled_issue() {
    new_test
    local issue=2003 pr=2004 head='z-devops/t_2003-labeled'
    set_state ISSUE_LIST_JSON '[]'
    set_state PR_LIST_MERGED_JSON "[{\"number\":${pr},\"title\":\"fix #${issue} labeled\",\"body\":\"closes #${issue}\",\"headRefName\":\"${head}\",\"mergedAt\":\"2026-08-12T14:14:40Z\"}]"
    set_state "ISSUE_${issue}_LABELS_JSON" '{"labels":[{"name":"needs-e2e"}]}'
    set_state "ISSUE_${issue}_STATE_JSON" '{"state":"OPEN"}'
    set_state "ISSUE_${issue}_COMMENTS_JSON" '{"comments":[]}'
    set_state "ISSUE_${issue}_COMMENTS_SINCE_JSON" '[]'
    set_state "ISSUE_${issue}_TIMELINE_JSON" '[]'
    set_state "RUN_LIST_${head}_JSON" '[{"conclusion":"success"}]'
    set_state PR_LIST_ALL_OPEN_JSON '[]'
    set_state PR_FOLLOWUP_JSON '[]'
    set_state RATE_LIMIT_JSON '{"resources":{"core":{"remaining":5000}}}'

    run_merge_gate
    local journal
    journal="$(cat "$GH_JOURNAL")"

    local close_calls
    close_calls="$(printf '%s\n' "$journal" | grep -c "gh issue close ${issue}" || true)"
    assert_eq "0" "$close_calls" "issue with needs-e2e is owned by e2e-process — no close"

    local add_calls
    add_calls="$(printf '%s\n' "$journal" | grep -c "gh issue edit ${issue} --add-label" || true)"
    assert_eq "0" "$add_calls" "no label ops on already-labeled issue"
}

# ===========================================================================
# E. issue уже CLOSED → ретро-путь skip.
# ===========================================================================
test_E_retro_skips_closed_issue() {
    new_test
    local issue=2005 pr=2006 head='z-devops/t_2005-closed'
    set_state ISSUE_LIST_JSON '[]'
    set_state PR_LIST_MERGED_JSON "[{\"number\":${pr},\"title\":\"fix #${issue} closed\",\"body\":\"closes #${issue}\",\"headRefName\":\"${head}\",\"mergedAt\":\"2026-08-12T14:14:40Z\"}]"
    set_state "ISSUE_${issue}_LABELS_JSON" '{"labels":[]}'
    set_state "ISSUE_${issue}_STATE_JSON" '{"state":"CLOSED"}'
    set_state "ISSUE_${issue}_COMMENTS_JSON" '{"comments":[]}'
    set_state "ISSUE_${issue}_COMMENTS_SINCE_JSON" '[]'
    set_state "ISSUE_${issue}_TIMELINE_JSON" '[]'
    set_state "RUN_LIST_${head}_JSON" '[{"conclusion":"success"}]'
    set_state PR_LIST_ALL_OPEN_JSON '[]'
    set_state PR_FOLLOWUP_JSON '[]'
    set_state RATE_LIMIT_JSON '{"resources":{"core":{"remaining":5000}}}'

    run_merge_gate
    local journal
    journal="$(cat "$GH_JOURNAL")"

    local close_calls
    close_calls="$(printf '%s\n' "$journal" | grep -c "gh issue close ${issue}" || true)"
    assert_eq "0" "$close_calls" "already-CLOSED issue untouched"
}

# ===========================================================================
# F. merged PR старше окна RETRO_MERGED_DAYS → не рассматривается.
# ===========================================================================
test_F_retro_skips_old_pr() {
    new_test
    local issue=2007 pr=2008 head='z-devops/t_2007-old'
    set_state ISSUE_LIST_JSON '[]'
    # mergedAt старше 14 дней.
    set_state PR_LIST_MERGED_JSON "[{\"number\":${pr},\"title\":\"fix #${issue} old\",\"body\":\"closes #${issue}\",\"headRefName\":\"${head}\",\"mergedAt\":\"2026-01-01T00:00:00Z\"}]"
    set_state "ISSUE_${issue}_LABELS_JSON" '{"labels":[]}'
    set_state "ISSUE_${issue}_STATE_JSON" '{"state":"OPEN"}'
    set_state "ISSUE_${issue}_COMMENTS_JSON" '{"comments":[]}'
    set_state "ISSUE_${issue}_COMMENTS_SINCE_JSON" '[]'
    set_state "ISSUE_${issue}_TIMELINE_JSON" '[]'
    set_state "RUN_LIST_${head}_JSON" '[{"conclusion":"success"}]'
    set_state PR_LIST_ALL_OPEN_JSON '[]'
    set_state PR_FOLLOWUP_JSON '[]'
    set_state RATE_LIMIT_JSON '{"resources":{"core":{"remaining":5000}}}'

    run_merge_gate
    local journal
    journal="$(cat "$GH_JOURNAL")"

    local close_calls
    close_calls="$(printf '%s\n' "$journal" | grep -c "gh issue close ${issue}" || true)"
    assert_eq "0" "$close_calls" "old merged PR outside window ignored"
}

# ===========================================================================
# G. self-reference: PR #1142 ссылается на #1142 в body → не issue-ref.
# ===========================================================================
test_G_retro_ignores_self_reference() {
    new_test
    local issue=1139 pr=1142 head='z-devops/t_self-ref'
    set_state ISSUE_LIST_JSON '[]'
    # В body PR упоминает СВОЙ номер (#1142) и #1139. Должен обработаться
    # только #1139 (self-reference #1142 отфильтрован).
    set_state PR_LIST_MERGED_JSON "[{\"number\":${pr},\"title\":\"fix(ci): SHA tags (#${issue})\",\"body\":\"PR: #${pr}\\ncloses #${issue}\",\"headRefName\":\"${head}\",\"mergedAt\":\"2026-08-12T14:14:40Z\"}]"
    set_state "ISSUE_${issue}_LABELS_JSON" '{"labels":[]}'
    set_state "ISSUE_${issue}_STATE_JSON" '{"state":"OPEN"}'
    set_state "ISSUE_${issue}_COMMENTS_JSON" '{"comments":[]}'
    set_state "ISSUE_${issue}_COMMENTS_SINCE_JSON" '[]'
    set_state "ISSUE_${issue}_TIMELINE_JSON" '[]'
    set_state "RUN_LIST_${head}_JSON" '[{"conclusion":"success"}]'
    set_state PR_LIST_ALL_OPEN_JSON '[]'
    set_state PR_FOLLOWUP_JSON '[]'
    set_state RATE_LIMIT_JSON '{"resources":{"core":{"remaining":5000}}}'

    run_merge_gate
    local journal
    journal="$(cat "$GH_JOURNAL")"

    # #1139 закрыта.
    local close_calls
    close_calls="$(printf '%s\n' "$journal" | grep -c "gh issue close ${issue} --reason completed" || true)"
    assert_eq "1" "$close_calls" "self-reference filtered, real issue closed"
}

# ===========================================================================
# H. (12.08 t_061d466e) issue с e2e:rejected + merged CI-only PR с зелёным
#    CI → e2e:rejected снимается, issue закрывается (петля #1041).
# ===========================================================================
test_H_retro_rejected_ci_only_green_closes() {
    new_test
    local issue=1041 pr=1161 head='z-{agent}/1041-fix-l-build-dockertag-clean'
    set_state ISSUE_LIST_JSON '[]'
    set_state PR_LIST_MERGED_JSON "[{\"number\":${pr},\"title\":\"fix(ci #${issue}): DOCKER_TAG=latest для refs/tags/v*\",\"body\":\"closes #${issue}\",\"headRefName\":\"${head}\",\"mergedAt\":\"2026-08-12T16:45:52Z\"}]"
    # issue имеет hermes + e2e:rejected (как #1041) — ретро-путь должен
    # обработать её через PASS-доказательство, а не скипнуть.
    set_state "ISSUE_${issue}_LABELS_JSON" '{"labels":[{"name":"hermes"},{"name":"e2e:rejected"}]}'
    set_state "ISSUE_${issue}_STATE_JSON" '{"state":"OPEN"}'
    set_state "ISSUE_${issue}_COMMENTS_JSON" '{"comments":[]}'
    set_state "ISSUE_${issue}_COMMENTS_SINCE_JSON" '[]'
    set_state "ISSUE_${issue}_TIMELINE_JSON" '[]'
    # e2e run НЕТ (пусто) — fallback на CI-only.
    set_state "RUN_LIST_${head}_JSON" '[]'
    # PR меняет только .github/ + docs → CI-only.
    set_state "PR_${pr}_FILES_JSON" '{"files":[{"path":".github/workflows/L-Build Vision Pi Services.yml"},{"path":"docs/process/HOTFIX.md"}]}'
    # CI зелёный: нет FAILURE/CANCELLED/TIMED_OUT.
    set_state "PR_${pr}_ROLLUP_JSON" '{"statusCheckRollup":[{"conclusion":"SUCCESS"},{"conclusion":"SUCCESS"},{"conclusion":"SKIPPED"}]}'
    set_state PR_LIST_ALL_OPEN_JSON '[]'
    set_state PR_FOLLOWUP_JSON '[]'
    set_state RATE_LIMIT_JSON '{"resources":{"core":{"remaining":5000}}}'

    run_merge_gate
    local journal
    journal="$(cat "$GH_JOURNAL")"

    local close_calls
    close_calls="$(printf '%s\n' "$journal" | grep -c "gh issue close ${issue} --reason completed" || true)"
    assert_eq "1" "$close_calls" "rejected issue with merged CI-only PR is closed"

    local remove_rejected
    remove_rejected="$(printf '%s\n' "$journal" | grep -c "gh issue edit ${issue} --remove-label e2e:rejected" || true)"
    assert_eq "1" "$remove_rejected" "e2e:rejected removed before close"

    local add_needs_e2e
    add_needs_e2e="$(printf '%s\n' "$journal" | grep -c "gh issue edit ${issue} --add-label needs-e2e" || true)"
    assert_eq "0" "$add_needs_e2e" "no needs-e2e when PASS evidence closes"
}

# ===========================================================================
# I. (12.08 t_061d466e) issue с e2e:rejected + merged PR, но PASS-
#    доказательства НЕТ → needs-e2e НЕ ставится, rejected остаётся,
#    close не вызывается (иначе e2e-process зациклится).
# ===========================================================================
test_I_retro_rejected_no_evidence_no_loop() {
    new_test
    local issue=3001 pr=3002 head='z-devops/t_3001-rejected-no-evidence'
    set_state ISSUE_LIST_JSON '[]'
    set_state PR_LIST_MERGED_JSON "[{\"number\":${pr},\"title\":\"fix #${issue} rejected unverified\",\"body\":\"closes #${issue}\",\"headRefName\":\"${head}\",\"mergedAt\":\"2026-08-12T14:14:40Z\"}]"
    set_state "ISSUE_${issue}_LABELS_JSON" '{"labels":[{"name":"e2e:rejected"}]}'
    set_state "ISSUE_${issue}_STATE_JSON" '{"state":"OPEN"}'
    set_state "ISSUE_${issue}_COMMENTS_JSON" '{"comments":[]}'
    set_state "ISSUE_${issue}_COMMENTS_SINCE_JSON" '[]'
    set_state "ISSUE_${issue}_TIMELINE_JSON" '[]'
    set_state "RUN_LIST_${head}_JSON" '[]'  # e2e нет
    # PR меняет код робота (не CI-only) → CI green не считается PASS для e2e.
    set_state "PR_${pr}_FILES_JSON" '{"files":[{"path":"src/robot/voice.py"}]}'
    set_state "PR_${pr}_ROLLUP_JSON" '{"statusCheckRollup":[{"conclusion":"SUCCESS"}]}'
    set_state PR_LIST_ALL_OPEN_JSON '[]'
    set_state PR_FOLLOWUP_JSON '[]'
    set_state RATE_LIMIT_JSON '{"resources":{"core":{"remaining":5000}}}'

    run_merge_gate
    local journal
    journal="$(cat "$GH_JOURNAL")"

    local close_calls
    close_calls="$(printf '%s\n' "$journal" | grep -c "gh issue close ${issue}" || true)"
    assert_eq "0" "$close_calls" "no close without PASS evidence even when rejected"

    local add_needs_e2e
    add_needs_e2e="$(printf '%s\n' "$journal" | grep -c "gh issue edit ${issue} --add-label needs-e2e" || true)"
    assert_eq "0" "$add_needs_e2e" "no needs-e2e on rejected issue without PASS (no e2e-process loop)"
}

# ===========================================================================
# Run
# ===========================================================================
run_test "A. retro-path: e2e PASS evidence → close unlabeled issue" test_A_retro_e2e_pass_closes
run_test "B. retro-path: CI-only PR green → close (e2e not required)" test_B_retro_ci_only_green_closes
run_test "C. retro-path: no PASS evidence → needs-e2e, no close" test_C_retro_no_evidence_labels_needs_e2e
run_test "D. retro-path: issue with needs-e2e → skip" test_D_retro_skips_labeled_issue
run_test "E. retro-path: CLOSED issue → skip" test_E_retro_skips_closed_issue
run_test "F. retro-path: old PR outside window → skip" test_F_retro_skips_old_pr
run_test "G. retro-path: self-reference ignored" test_G_retro_ignores_self_reference
run_test "H. retro-path: e2e:rejected + merged CI-only green → close + remove rejected" test_H_retro_rejected_ci_only_green_closes
run_test "I. retro-path: e2e:rejected + merged no PASS → no needs-e2e loop" test_I_retro_rejected_no_evidence_no_loop

summary
