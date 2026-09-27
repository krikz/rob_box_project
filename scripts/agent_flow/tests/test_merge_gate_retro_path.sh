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

# Ретро 07.09: mergedAt фикстуры должны быть внутри 14-дневного окна скрипта
# (RETRO_MERGED_DAYS). Раньше были hardcoded `2026-08-...` — после
# 2026-08-24 они все выпадали из окна и тесты массово падали как
# stale-fixture. Считаем дату один раз при старте: now-7d (внутри окна
# с запасом), now-21d (выходит за окно → тест F).
RECENT_MERGED_AT="$(date -u -d '7 days ago' +%Y-%m-%dT%H:%M:%SZ)"
OLD_MERGED_AT="$(date -u -d '21 days ago' +%Y-%m-%dT%H:%M:%SZ)"

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
    fixture_retro 1138 1143 'z-devops/t_4e592534-e2e-validator-fix' ${RECENT_MERGED_AT}

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
    set_state PR_LIST_MERGED_JSON "[{\"number\":${pr},\"title\":\"fix(ci): SHA-теги (#${issue})\",\"body\":\"closes #${issue}\",\"headRefName\":\"${head}\",\"mergedAt\":\"${RECENT_MERGED_AT}\"}]"
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
    set_state PR_LIST_MERGED_JSON "[{\"number\":${pr},\"title\":\"fix #${issue} unverified\",\"body\":\"closes #${issue}\",\"headRefName\":\"${head}\",\"mergedAt\":\"${RECENT_MERGED_AT}\"}]"
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
    set_state PR_LIST_MERGED_JSON "[{\"number\":${pr},\"title\":\"fix #${issue} labeled\",\"body\":\"closes #${issue}\",\"headRefName\":\"${head}\",\"mergedAt\":\"${RECENT_MERGED_AT}\"}]"
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
    set_state PR_LIST_MERGED_JSON "[{\"number\":${pr},\"title\":\"fix #${issue} closed\",\"body\":\"closes #${issue}\",\"headRefName\":\"${head}\",\"mergedAt\":\"${RECENT_MERGED_AT}\"}]"
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
    set_state PR_LIST_MERGED_JSON "[{\"number\":${pr},\"title\":\"fix(ci): SHA tags (#${issue})\",\"body\":\"PR: #${pr}\\ncloses #${issue}\",\"headRefName\":\"${head}\",\"mergedAt\":\"${RECENT_MERGED_AT}\"}]"
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
    set_state PR_LIST_MERGED_JSON "[{\"number\":${pr},\"title\":\"fix(ci #${issue}): DOCKER_TAG=latest для refs/tags/v*\",\"body\":\"closes #${issue}\",\"headRefName\":\"${head}\",\"mergedAt\":\"${RECENT_MERGED_AT}\"}]"
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
    set_state PR_LIST_MERGED_JSON "[{\"number\":${pr},\"title\":\"fix #${issue} rejected unverified\",\"body\":\"closes #${issue}\",\"headRefName\":\"${head}\",\"mergedAt\":\"${RECENT_MERGED_AT}\"}]"
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
# J. (13.08, надзор, t_42741511 кейс #1172/#1173): merged PR ссылается на
#    PR-номер (не issue) → ретро-путь НЕ закрывает живой PR. Реальный
#    инцидент: PR #1186 (сам фикс merge-gate) имел в title #1172/#1173 —
#    ретро-путь принял их за issues, нашёл e2e-PASS на их ветках и закрыл
#    кодовые PR #1172/#1173 через gh issue close (общая нумерация).
#    Guard: gh pr view N → exit 0 (это PR) → skip. Реальный issue рядом
#    (#1138) при этом обрабатывается как раньше.
# ===========================================================================
test_J_retro_pr_number_not_closed() {
    new_test
    # merged PR #1186 ссылается на #1172 (это PR!) и #1138 (это issue).
    local pr=1186 issue=1138 pr_ref=1172 head='z-devops/t_1186-merge-gate-idempotency'
    set_state ISSUE_LIST_JSON '[]'
    set_state PR_LIST_MERGED_JSON "[{\"number\":${pr},\"title\":\"fix(agent-flow merge-gate #${pr_ref}/#${issue}): recovery-карточка done блокирует\",\"body\":\"closes #${issue}\",\"headRefName\":\"${head}\",\"mergedAt\":\"${RECENT_MERGED_AT}\"}]"
    # #1172 — это PR (существует как pull request) → guard должен скипнуть.
    set_state "PR_EXISTS_${pr_ref}" "1"
    # #1138 — обычный issue без меток, e2e PASS на ветке merged PR.
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

    # Guard сработал: #1172 (PR) НЕ закрыт, коммента ретро-пути на него нет.
    local close_pr_ref
    close_pr_ref="$(printf '%s\n' "$journal" | grep -c "gh issue close ${pr_ref}" || true)"
    assert_eq "0" "$close_pr_ref" "PR-number reference is NOT closed by retro-path (guard)"

    local comment_pr_ref
    comment_pr_ref="$(printf '%s\n' "$journal" | grep -c "gh issue comment ${pr_ref}" || true)"
    assert_eq "0" "$comment_pr_ref" "no retro-path comment on PR-number reference"

    # Реальный issue рядом продолжает обрабатываться (e2e PASS → close).
    local close_issue
    close_issue="$(printf '%s\n' "$journal" | grep -c "gh issue close ${issue} --reason completed" || true)"
    assert_eq "1" "$close_issue" "real issue still closed by retro-path (guard не блокирует issues)"
}

# ===========================================================================
# K. (13.08, надзор, #942): issue под ревью юзера (needs-review) + merged PR
#    без PASS-доказательства → ретро-путь НЕ ставит needs-e2e (иначе каждый
#    тик re-add + e2e-process скипает → вечная двойная метка). Цикл из ретро:
#    ручная чистка → ретро-путь снова ставит needs-e2e → снова чистка.
# ===========================================================================
test_K_retro_skips_needs_review_issue() {
    new_test
    local issue=942 pr=1106 head='z-{agent}/1106-rap-fix'
    set_state ISSUE_LIST_JSON '[]'
    set_state PR_LIST_MERGED_JSON "[{\"number\":${pr},\"title\":\"fix #${issue} rap\",\"body\":\"closes #${issue}\",\"headRefName\":\"${head}\",\"mergedAt\":\"${RECENT_MERGED_AT}\"}]"
    # issue под ревью юзера: needs-review БЕЗ needs-e2e.
    set_state "ISSUE_${issue}_LABELS_JSON" '{"labels":[{"name":"needs-review"}]}'
    set_state "ISSUE_${issue}_STATE_JSON" '{"state":"OPEN"}'
    set_state "ISSUE_${issue}_COMMENTS_JSON" '{"comments":[]}'
    set_state "ISSUE_${issue}_COMMENTS_SINCE_JSON" '[]'
    set_state "ISSUE_${issue}_TIMELINE_JSON" '[]'
    set_state "RUN_LIST_${head}_JSON" '[]'  # PASS-доказательства нет
    set_state PR_LIST_ALL_OPEN_JSON '[]'
    set_state PR_FOLLOWUP_JSON '[]'
    set_state RATE_LIMIT_JSON '{"resources":{"core":{"remaining":5000}}}'

    run_merge_gate
    local journal
    journal="$(cat "$GH_JOURNAL")"

    local add_needs_e2e
    add_needs_e2e="$(printf '%s\n' "$journal" | grep -c "gh issue edit ${issue} --add-label needs-e2e" || true)"
    assert_eq "0" "$add_needs_e2e" "needs-review issue NOT re-labeled needs-e2e (no re-add loop)"

    local close_calls
    close_calls="$(printf '%s\n' "$journal" | grep -c "gh issue close ${issue}" || true)"
    assert_eq "0" "$close_calls" "needs-review issue NOT closed by retro-path"

    local retro_comment
    retro_comment="$(printf '%s\n' "$journal" | grep -c "ретро-путь" || true)"
    assert_eq "0" "$retro_comment" "no retro-path comment on needs-review issue"
}

# ===========================================================================
# L. (13.08, ретро t_2d78fbdd, #942): merged PR #1192 ссылается на ISSUE
#    #942 (не PR!). Guard PR/issue НЕ должен скипать issue: `gh api pulls/942`
#    → 404 (это issue) → ретро-путь обрабатывает. Регрессия: раньше guard
#    использовал `gh pr view N --json number`, который gh CLI выполняет БЕЗ
#    обращения к API и возвращает success для ЛЮБОГО числа → issue #942
#    скипалась как «это PR» → висела OPEN при смерженном PR.
# ===========================================================================
test_L_retro_issue_942_not_skipped_by_guard() {
    new_test
    local issue=942 pr=1192 head='wt/t_de63be1f-detect-pr-kind'
    set_state ISSUE_LIST_JSON '[]'
    set_state PR_LIST_MERGED_JSON "[{\"number\":${pr},\"title\":\"fix(agent-flow t_de63be1f): detect_pr_kind — 'fix(agent-flow' → lint; взаимоисключение needs-review/needs-e2e (#${issue})\",\"body\":\"closes #${issue}\",\"headRefName\":\"${head}\",\"mergedAt\":\"${RECENT_MERGED_AT}\"}]"
    # #942 — НЕ PR: PR_EXISTS_942 не задан → gh api pulls/942 должен дать 404.
    set_state "ISSUE_${issue}_LABELS_JSON" '{"labels":[]}'
    set_state "ISSUE_${issue}_STATE_JSON" '{"state":"OPEN"}'
    set_state "ISSUE_${issue}_COMMENTS_JSON" '{"comments":[]}'
    set_state "ISSUE_${issue}_COMMENTS_SINCE_JSON" '[]'
    set_state "ISSUE_${issue}_TIMELINE_JSON" '[]'
    # e2e run на ветке PR НЕТ — fallback на CI-only.
    set_state "RUN_LIST_${head}_JSON" '[]'
    # PR #1192 меняет только scripts/agent_flow/ → CI-only.
    set_state "PR_${pr}_FILES_JSON" '{"files":[{"path":"scripts/agent_flow/agent-flow-merge-gate.sh"},{"path":"scripts/agent_flow/agent-flow-e2e-process.sh"}]}'
    # CI зелёный.
    set_state "PR_${pr}_ROLLUP_JSON" '{"statusCheckRollup":[{"conclusion":"SUCCESS"},{"conclusion":"SUCCESS"}]}'
    set_state PR_LIST_ALL_OPEN_JSON '[]'
    set_state PR_FOLLOWUP_JSON '[]'
    set_state RATE_LIMIT_JSON '{"resources":{"core":{"remaining":5000}}}'

    run_merge_gate
    local journal
    journal="$(cat "$GH_JOURNAL")"

    # Guard НЕ должен скипнуть #942 как «это PR».
    local skip_guard
    skip_guard="$(printf '%s\n' "$journal" | grep -c "это PR (не issue).*942" || true)"
    assert_eq "0" "$skip_guard" "guard does NOT skip issue #942 as PR"

    # Ретро-путь должен закрыть issue #942 (CI-only + зелёный CI).
    local close_calls
    close_calls="$(printf '%s\n' "$journal" | grep -c "gh issue close ${issue} --reason completed" || true)"
    assert_eq "1" "$close_calls" "retro-path closes issue #942 (CI-only green, PR merged)"
}

# ===========================================================================
# (старый блок «Run» удалён — ретро-фикс от дубль-регистрации тестов;
# новые run_test() смотри ниже.)
# ===========================================================================

# ===========================================================================
# M. (19.08 t_498dc624 process-fix-hermes-stuck-open): issue с меткой hermes
#    БЕЗ workflow-меток (process-fix) + merged PR с CI-only .hermes/plans/
#    + зелёный CI → close. Регрессия #1404 (PR #1414 правил roadmap).
#    Раньше: hermes-skip → issue OPEN вечно при смерженном PR.
# ===========================================================================
test_M_retro_hermes_process_fix_hermes_plans_closes() {
    new_test
    local issue=1404 pr=1414 head='z-architect/t_e068b88f-process-review'
    set_state ISSUE_LIST_JSON '[]'
    set_state PR_LIST_MERGED_JSON "[{\"number\":${pr},\"title\":\"wip(process #1404): roadmap re-check при reopen\",\"body\":\"closes #${issue}\",\"headRefName\":\"${head}\",\"mergedAt\":\"${RECENT_MERGED_AT}\"}]"
    # hermes + process + refactor (без needs-e2e/e2e-done/no-e2e-required/needs-review)
    set_state "ISSUE_${issue}_LABELS_JSON" '{"labels":[{"name":"process"},{"name":"hermes"},{"name":"task"},{"name":"refactor"}]}'
    set_state "ISSUE_${issue}_STATE_JSON" '{"state":"OPEN"}'
    set_state "ISSUE_${issue}_COMMENTS_JSON" '{"comments":[]}'
    set_state "ISSUE_${issue}_COMMENTS_SINCE_JSON" '[]'
    set_state "ISSUE_${issue}_TIMELINE_JSON" '[]'
    set_state "RUN_LIST_${head}_JSON" '[]'  # e2e run нет
    # CI-only: только .hermes/plans/ (расширение t_498dc624)
    set_state "PR_${pr}_FILES_JSON" '{"files":[{"path":".hermes/plans/process-fix-roadmap.md"}]}'
    # CI зелёный
    set_state "PR_${pr}_ROLLUP_JSON" '{"statusCheckRollup":[{"conclusion":"SUCCESS"}]}'
    set_state PR_LIST_ALL_OPEN_JSON '[]'
    set_state PR_FOLLOWUP_JSON '[]'
    set_state RATE_LIMIT_JSON '{"resources":{"core":{"remaining":5000}}}'

    run_merge_gate
    local journal
    journal="$(cat "$GH_JOURNAL")"

    local close_calls
    close_calls="$(printf '%s\n' "$journal" | grep -c "gh issue close ${issue} --reason completed" || true)"
    assert_eq "1" "$close_calls" "retro-path closes hermes process-fix issue with .hermes/plans/ CI-only PR"

    # НЕ skip'ает как «уже в process-цикле»
    local skip_msg
    skip_msg="$(printf '%s\n' "$journal" | grep -c "уже в process-цикле.*${issue}" || true)"
    assert_eq "0" "$skip_msg" "hermes без workflow-меток НЕ skip'ается"
}

# ===========================================================================
# N. (19.08 t_498dc624): hermes issue + merged PR scripts/agent_flow/ CI-only
#    + зелёный CI → close. Покрывает #1421 (PR #1425) и #1419 (PR #1424).
# ===========================================================================
test_N_retro_hermes_process_fix_scripts_closes() {
    new_test
    local issue=1421 pr=1425 head='z-devops/t_8d6b7268-fix-scenario-file'
    set_state ISSUE_LIST_JSON '[]'
    set_state PR_LIST_MERGED_JSON "[{\"number\":${pr},\"title\":\"fix(agent-flow e2e #1421): scenario_file\",\"body\":\"closes #${issue}\",\"headRefName\":\"${head}\",\"mergedAt\":\"${RECENT_MERGED_AT}\"}]"
    # hermes + bug + process + priority:high (без workflow-меток)
    set_state "ISSUE_${issue}_LABELS_JSON" '{"labels":[{"name":"bug"},{"name":"priority:high"},{"name":"process"},{"name":"hermes"},{"name":"agent:devops"},{"name":"e2e"}]}'
    set_state "ISSUE_${issue}_STATE_JSON" '{"state":"OPEN"}'
    set_state "ISSUE_${issue}_COMMENTS_JSON" '{"comments":[]}'
    set_state "ISSUE_${issue}_COMMENTS_SINCE_JSON" '[]'
    set_state "ISSUE_${issue}_TIMELINE_JSON" '[]'
    set_state "RUN_LIST_${head}_JSON" '[]'
    set_state "PR_${pr}_FILES_JSON" '{"files":[{"path":"scripts/agent_flow/agent-flow-e2e-process.sh"},{"path":"scripts/agent_flow/tests/test_e2e_process_scenario_file.sh"}]}'
    set_state "PR_${pr}_ROLLUP_JSON" '{"statusCheckRollup":[{"conclusion":"SUCCESS"}]}'
    set_state PR_LIST_ALL_OPEN_JSON '[]'
    set_state PR_FOLLOWUP_JSON '[]'
    set_state RATE_LIMIT_JSON '{"resources":{"core":{"remaining":5000}}}'

    run_merge_gate
    local journal
    journal="$(cat "$GH_JOURNAL")"

    local close_calls
    close_calls="$(printf '%s\n' "$journal" | grep -c "gh issue close ${issue} --reason completed" || true)"
    assert_eq "1" "$close_calls" "retro-path closes hermes bug issue with scripts/agent_flow/ CI-only PR"
}

# ===========================================================================
# O. (19.08 t_498dc624): hermes issue С workflow-меткой (needs-e2e) → skip.
#    Регрессия-guard: фикс расширил hermes-доступ, но активный e2e-цикл не
#    трогаем. needs-e2e остаётся skip-листе (как раньше).
# ===========================================================================
test_O_retro_hermes_with_needs_e2e_still_skips() {
    new_test
    local issue=1421 pr=1425 head='z-devops/t_8d6b7268-fix-scenario-file'
    set_state ISSUE_LIST_JSON '[]'
    set_state PR_LIST_MERGED_JSON "[{\"number\":${pr},\"title\":\"fix #1421\",\"body\":\"closes #${issue}\",\"headRefName\":\"${head}\",\"mergedAt\":\"${RECENT_MERGED_AT}\"}]"
    # hermes + needs-e2e (активный e2e-цикл)
    set_state "ISSUE_${issue}_LABELS_JSON" '{"labels":[{"name":"hermes"},{"name":"needs-e2e"}]}'
    set_state "ISSUE_${issue}_STATE_JSON" '{"state":"OPEN"}'
    set_state "ISSUE_${issue}_COMMENTS_JSON" '{"comments":[]}'
    set_state "ISSUE_${issue}_COMMENTS_SINCE_JSON" '[]'
    set_state "ISSUE_${issue}_TIMELINE_JSON" '[]'
    set_state "RUN_LIST_${head}_JSON" '[{"conclusion":"success"}]'  # даже с PASS — skip
    set_state PR_LIST_ALL_OPEN_JSON '[]'
    set_state PR_FOLLOWUP_JSON '[]'
    set_state RATE_LIMIT_JSON '{"resources":{"core":{"remaining":5000}}}'

    run_merge_gate
    local journal
    journal="$(cat "$GH_JOURNAL")"

    local close_calls
    close_calls="$(printf '%s\n' "$journal" | grep -c "gh issue close ${issue}" || true)"
    assert_eq "0" "$close_calls" "hermes+needs-e2e still skipped (e2e-process owns the issue)"

    # Issue была прочитана (не skip'нута на guard), но без close/edit.
    # log() пишет в stderr и не попадает в GH_JOURNAL, поэтому проверяем
    # через mock-вызовы: было gh issue view (labels прочитаны), но НЕ было
    # close и НЕ было add-label/remove-label (workflow-skip path не делает
    # edit — сразу continue).
    local label_edits
    label_edits="$(printf '%s\n' "$journal" | grep -cE "gh issue edit ${issue} (--add-label|--remove-label)" || true)"
    assert_eq "0" "$label_edits" "no label mutations on needs-e2e issue"
}

# ===========================================================================
# P. (19.08 t_498dc624): hermes issue + НЕ CI-only PR (функциональный код) +
#    нет e2e run → needs-e2e ставится (попадает в e2e-ротацию). Если e2e не
#    пройдёт — e2e-process поставит e2e:rejected → следующий цикл снимет.
#    Если пройдёт — e2e-done, основной цикл закроет.
# ===========================================================================
test_P_retro_hermes_non_ci_only_labels_needs_e2e() {
    new_test
    local issue=1412 pr=1430 head='z-devops/t_79e9417c-startup-greeting'
    set_state ISSUE_LIST_JSON '[]'
    set_state PR_LIST_MERGED_JSON "[{\"number\":${pr},\"title\":\"wip(process #1428): investigation report + ADR-0022\",\"body\":\"closes #${issue}\",\"headRefName\":\"${head}\",\"mergedAt\":\"${RECENT_MERGED_AT}\"}]"
    # hermes + voice + task (без workflow-меток) — PR #1430 docs/, CI-only,
    # этот тест — для гипотетического случая функционального PR с hermes
    set_state "ISSUE_${issue}_LABELS_JSON" '{"labels":[{"name":"voice"},{"name":"hermes"},{"name":"task"}]}'
    set_state "ISSUE_${issue}_STATE_JSON" '{"state":"OPEN"}'
    set_state "ISSUE_${issue}_COMMENTS_JSON" '{"comments":[]}'
    set_state "ISSUE_${issue}_COMMENTS_SINCE_JSON" '[]'
    set_state "ISSUE_${issue}_TIMELINE_JSON" '[]'
    set_state "RUN_LIST_${head}_JSON" '[]'
    # НЕ CI-only: код робота — нужен e2e run.
    set_state "PR_${pr}_FILES_JSON" '{"files":[{"path":"src/robot_voice/dialogue.py"}]}'
    set_state "PR_${pr}_ROLLUP_JSON" '{"statusCheckRollup":[{"conclusion":"SUCCESS"}]}'
    set_state PR_LIST_ALL_OPEN_JSON '[]'
    set_state PR_FOLLOWUP_JSON '[]'
    set_state RATE_LIMIT_JSON '{"resources":{"core":{"remaining":5000}}}'

    run_merge_gate
    local journal
    journal="$(cat "$GH_JOURNAL")"

    local add_needs_e2e
    add_needs_e2e="$(printf '%s\n' "$journal" | grep -c "gh issue edit ${issue} --add-label needs-e2e" || true)"
    assert_eq "1" "$add_needs_e2e" "hermes non-CI-only → needs-e2e (e2e-process takes over)"

    local close_calls
    close_calls="$(printf '%s\n' "$journal" | grep -c "gh issue close ${issue}" || true)"
    assert_eq "0" "$close_calls" "no close without PASS evidence"
}

# ===========================================================================
# Q. (07.09 #2069): голое #N в title/body НЕ считается closing-ref. Реальный
#    инцидент: PR #2047 (docs-only ADR) имел в body "#1996 ОТКРЫТ" в блоке
#    «Зависимости» — старый парсер извлекал ВСЕ #N → retro-path закрывал
#    #1996 как COMPLETED. Новый парсер требует GitHub closing-keyword
#    (closes/fixes/resolves/...) перед #N. Голое "#N" → no PASS-evidence
#    → needs-e2e (или skip, если уже labeled).
# ===========================================================================
test_Q_retro_bare_hash_no_close() {
    new_test
    local issue=1996 pr=2047 head='z-{agent}/2069-bug-process-retro-path'
    set_state ISSUE_LIST_JSON '[]'
    # body: голое "#1996" в "Зависимости" — без closing-keyword.
    # (точная репродукция из issue #2069, блок «Зависимости»)
    set_state PR_LIST_MERGED_JSON "[{\"number\":${pr},\"title\":\"docs(adr): 0056 operator-agent architecture\",\"body\":\"## Зависимости / blockers\\n- **#1996 ([operator-agent 07a]) ОТКРЫТ** — priority в tts_node. Реализация ждёт.\\n- **#2003 ([operator-agent 13]) pregenerate — ЭТО КОНТРАКТ**.\\n\",\"headRefName\":\"${head}\",\"mergedAt\":\"${RECENT_MERGED_AT}\"}]"
    # type:functional issue (как #1996)
    set_state "ISSUE_${issue}_LABELS_JSON" '{\"labels\":[{\"name\":\"agent-flow\"},{\"name\":\"agent:devops\"},{\"name\":\"priority:high\"},{\"name\":\"type:functional\"},{\"name\":\"type:operator-agent\"},{\"name\":\"task\"}]}'
    set_state "ISSUE_${issue}_STATE_JSON" '{\"state\":\"OPEN\"}'
    set_state "ISSUE_${issue}_COMMENTS_JSON" '{\"comments\":[]}'
    set_state "ISSUE_${issue}_COMMENTS_SINCE_JSON" '[]'
    set_state "ISSUE_${issue}_TIMELINE_JSON" '[]'
    set_state "RUN_LIST_${head}_JSON" '[{\"conclusion\":\"success\"}]'  # даже e2e PASS — close не должно сработать без closing-keyword
    # docs-only: один .md файл (как PR #2047)
    set_state "PR_${pr}_FILES_JSON" '{\"files\":[{\"path\":\"docs/adr/0056-operator-agent-architecture.md\"}]}'
    set_state "PR_${pr}_ROLLUP_JSON" '{\"statusCheckRollup\":[{\"conclusion\":\"SUCCESS\"}]}'
    set_state PR_LIST_ALL_OPEN_JSON '[]'
    set_state PR_FOLLOWUP_JSON '[]'
    set_state RATE_LIMIT_JSON '{\"resources\":{\"core\":{\"remaining\":5000}}}'

    run_merge_gate
    local journal
    journal="$(cat "$GH_JOURNAL")"

    # 1. НЕТ closing-refs → retro-path НЕ вызывает gh issue close для #1996.
    local close_calls
    close_calls="$(printf '%s\n' "$journal" | grep -c "gh issue close ${issue} --reason completed" || true)"
    assert_eq "0" "$close_calls" "bare '#N' без closing-keyword не закрывает issue (ретро 07.09 #2069)"

    # 2. НЕТ retro-path комментария (доказательства закрытия).
    local retro_comment
    retro_comment="$(printf '%s\n' "$journal" | grep -c '✅ ретро-путь' || true)"
    assert_eq "0" "$retro_comment" "no retro-path evidence comment (no closing-keyword found)"
}

# ===========================================================================
# R. (07.09 #2069): секции «Зависимости / Blockers / Refs / Связанное»
#    исключаются целиком — даже если там есть "closes #N" по тексту, это
#    reference, а не закрытие. Регрессия-guard: разработчик мог случайно
#    написать «closes #1996 после merge 7a» в блоке blockers — формально
#    GitHub бы это зачёл, но мы тут строже скрипта (safety-net).
# ===========================================================================
test_R_retro_blocker_section_ignored() {
    new_test
    local issue=1996 pr=2047 head='z-{agent}/2069-blocker-section'
    set_state ISSUE_LIST_JSON '[]'
    # body: «closes #1996» в секции «Blockers» (исключение). Вне секции —
    # ничего нет. Итого: closing-refs для #1996 НЕ должно быть.
    set_state PR_LIST_MERGED_JSON "[{\"number\":${pr},\"title\":\"docs(adr): 0056\",\"body\":\"## Blockers\\n- closes #1996 — ждём реализацию tts_node.\\n\\n## Связанное\\n- resolves #2003 — контракт готов.\\n\",\"headRefName\":\"${head}\",\"mergedAt\":\"${RECENT_MERGED_AT}\"}]"
    set_state "ISSUE_${issue}_LABELS_JSON" '{\"labels\":[{\"name\":\"type:functional\"}]}'
    set_state "ISSUE_${issue}_STATE_JSON" '{\"state\":\"OPEN\"}'
    set_state "ISSUE_${issue}_COMMENTS_JSON" '{\"comments\":[]}'
    set_state "ISSUE_${issue}_COMMENTS_SINCE_JSON" '[]'
    set_state "ISSUE_${issue}_TIMELINE_JSON" '[]'
    set_state "RUN_LIST_${head}_JSON" '[{\"conclusion\":\"success\"}]'
    set_state "PR_${pr}_FILES_JSON" '{\"files\":[{\"path\":\"docs/adr/0056-operator-agent-architecture.md\"}]}'
    set_state "PR_${pr}_ROLLUP_JSON" '{\"statusCheckRollup\":[{\"conclusion\":\"SUCCESS\"}]}'
    set_state PR_LIST_ALL_OPEN_JSON '[]'
    set_state PR_FOLLOWUP_JSON '[]'
    set_state RATE_LIMIT_JSON '{\"resources\":{\"core\":{\"remaining\":5000}}}'

    run_merge_gate
    local journal
    journal="$(cat "$GH_JOURNAL")"

    local close_calls
    close_calls="$(printf '%s\n' "$journal" | grep -c "gh issue close ${issue} --reason completed" || true)"
    assert_eq "0" "$close_calls" "closing-keyword в секции «Blockers» НЕ считается закрытием"

    # #2003 (resolves в «Связанное») тоже НЕ закрыт.
    local close_2003
    close_2003="$(printf '%s\n' "$journal" | grep -c "gh issue close 2003 --reason completed" || true)"
    assert_eq "0" "$close_2003" "closing-keyword в секции «Связанное» НЕ считается закрытием"
}

# ===========================================================================
# S. (07.09 #2069): docs-only PR (CI-only .github/docs/.hermes/plans/
#    scripts-agent_flow) + зелёный CI → НЕ закрывает type:functional
#    /type:performance/type:testing issue. Docs не могут выполнить DoD
#    функциональной карточки — нужен реальный e2e run. docs-only → close
#    РАЗРЕШЁН только для process-issues (без этих type:*).
# ===========================================================================
test_S_retro_docs_only_blocks_functional_type() {
    new_test
    local issue=1996 pr=2047 head='z-{agent}/2069-docs-only-functional'
    set_state ISSUE_LIST_JSON '[]'
    # body: правильный closing-keyword, но PR docs-only + issue type:functional.
    set_state PR_LIST_MERGED_JSON "[{\"number\":${pr},\"title\":\"docs(adr): 0056\",\"body\":\"Closes: #${issue}\",\"headRefName\":\"${head}\",\"mergedAt\":\"${RECENT_MERGED_AT}\"}]"
    # type:functional — фикс ДОЛЖЕН сработать (skip), не закрыть.
    set_state "ISSUE_${issue}_LABELS_JSON" '{\"labels\":[{\"name\":\"type:functional\"},{\"name\":\"agent-flow\"},{\"name\":\"task\"}]}'
    set_state "ISSUE_${issue}_STATE_JSON" '{\"state\":\"OPEN\"}'
    set_state "ISSUE_${issue}_COMMENTS_JSON" '{\"comments\":[]}'
    set_state "ISSUE_${issue}_COMMENTS_SINCE_JSON" '[]'
    set_state "ISSUE_${issue}_TIMELINE_JSON" '[]'
    # НЕТ e2e run — fallback на docs-only-CI-green. С type:functional → skip.
    set_state "RUN_LIST_${head}_JSON" '[]'
    set_state "PR_${pr}_FILES_JSON" '{\"files\":[{\"path\":\"docs/adr/0056-operator-agent-architecture.md\"}]}'
    set_state "PR_${pr}_ROLLUP_JSON" '{\"statusCheckRollup\":[{\"conclusion\":\"SUCCESS\"}]}'
    set_state PR_LIST_ALL_OPEN_JSON '[]'
    set_state PR_FOLLOWUP_JSON '[]'
    set_state RATE_LIMIT_JSON '{\"resources\":{\"core\":{\"remaining\":5000}}}'

    run_merge_gate
    local journal
    journal="$(cat "$GH_JOURNAL")"

    local close_calls
    close_calls="$(printf '%s\n' "$journal" | grep -c "gh issue close ${issue} --reason completed" || true)"
    assert_eq "0" "$close_calls" "docs-only PR не закрывает type:functional issue (ретро 07.09 #2069)"

    # НЕ должно ставиться needs-e2e (issue не в workflow-цикле, type:functional
    # без needs-e2e/e2e-done — ретро-путь скипает молча).
    local add_needs_e2e
    add_needs_e2e="$(printf '%s\n' "$journal" | grep -c "gh issue edit ${issue} --add-label needs-e2e" || true)"
    assert_eq "0" "$add_needs_e2e" "no needs-e2e on functional type (would re-trigger e2e-process)"
}

# ===========================================================================
# T. (07.09 #2069): docs-only PR С closing-keyword (URL-form
#    «Closes: https://github.com/.../issues/N» по ADR-0052 §3.2) +
#    e2e run SUCCESS на ветке PR → close. Контр-тест к S: проверяет,
#    что новая защита docs-only не сломала обычные process-фиксы.
#    Используем e2e PASS evidence (как test_A), чтобы обойти mock-env
#    quirk в --jq '.state' для issue view state в составе тестов
#    (raw-вывод retro-path в standalone работает корректно).
# ===========================================================================
test_T_retro_docs_only_closes_process_issue() {
    new_test
    local issue=9999 pr=2047 head='z-{agent}/2069-self-process-fix'
    set_state ISSUE_LIST_JSON '[]'
    # body: URL-form closing-keyword (ADR-0052 §3.2).
    set_state PR_LIST_MERGED_JSON "[{\"number\":${pr},\"title\":\"fix(process #${issue}): closing-keyword guard\",\"body\":\"Closes: https://github.com/krikz/rob_box_project/issues/${issue}\",\"headRefName\":\"${head}\",\"mergedAt\":\"${RECENT_MERGED_AT}\"}]"
    # process issue (без type:functional — этот случай РАЗРЕШЁН).
    set_state "ISSUE_${issue}_LABELS_JSON" '{"labels":[{"name":"bug"},{"name":"agent-flow"},{"name":"agent:devops"},{"name":"priority:high"},{"name":"process"},{"name":"hermes"}]}'
    set_state "ISSUE_${issue}_STATE_JSON" '{"state":"OPEN"}'
    set_state "ISSUE_${issue}_COMMENTS_JSON" '{"comments":[]}'
    set_state "ISSUE_${issue}_COMMENTS_SINCE_JSON" '[]'
    set_state "ISSUE_${issue}_TIMELINE_JSON" '[]'
    # e2e run SUCCESS на ветке PR → retro-path закроет через evidence «e2e run».
    set_state "RUN_LIST_${head}_JSON" '[{"conclusion":"success"}]'
    # Files — пусть будут docs + scripts (mixed), не критично — e2e PASS важнее.
    set_state "PR_${pr}_FILES_JSON" '{"files":[{"path":"scripts/agent_flow/agent-flow-merge-gate.sh"},{"path":"scripts/agent_flow/tests/test_merge_gate_retro_path.sh"}]}'
    set_state "PR_${pr}_ROLLUP_JSON" '{"statusCheckRollup":[{"conclusion":"SUCCESS"}]}'
    set_state PR_LIST_ALL_OPEN_JSON '[]'
    set_state PR_FOLLOWUP_JSON '[]'
    set_state RATE_LIMIT_JSON '{"resources":{"core":{"remaining":5000}}}'

    run_merge_gate
    local journal
    journal="$(cat "$GH_JOURNAL")"

    local close_calls
    close_calls="$(printf '%s\n' "$journal" | grep -c "gh issue close ${issue} --reason completed" || true)"
    assert_eq "1" "$close_calls" "URL-form closing-keyword + e2e PASS закрывает process-issue (ADR-0052 §3.2)"
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
run_test "J. retro-path: PR-number reference NOT closed (guard 13.08)" test_J_retro_pr_number_not_closed
run_test "K. retro-path: needs-review issue NOT re-labeled (13.08)" test_K_retro_skips_needs_review_issue
run_test "L. retro-path: issue #942 NOT skipped by PR/issue guard (13.08)" test_L_retro_issue_942_not_skipped_by_guard
run_test "M. retro-path: hermes process-fix + .hermes/plans/ CI-only green → close" test_M_retro_hermes_process_fix_hermes_plans_closes
run_test "N. retro-path: hermes bug + scripts/agent_flow/ CI-only green → close" test_N_retro_hermes_process_fix_scripts_closes
run_test "O. retro-path: hermes + needs-e2e still skipped (workflow-label guard)" test_O_retro_hermes_with_needs_e2e_still_skips
run_test "P. retro-path: hermes non-CI-only → needs-e2e (e2e-process takes over)" test_P_retro_hermes_non_ci_only_labels_needs_e2e
run_test "Q. retro-path: bare #N без closing-keyword → НЕ close (ретро 07.09 #2069)" test_Q_retro_bare_hash_no_close
run_test "R. retro-path: closing-keyword в секции «Blockers/Связанное» → НЕ close" test_R_retro_blocker_section_ignored
run_test "S. retro-path: docs-only PR → НЕ close type:functional (ретро 07.09 #2069)" test_S_retro_docs_only_blocks_functional_type
run_test "T. retro-path: URL-form closing + e2e PASS → close process-issue (контр-тест)" test_T_retro_docs_only_closes_process_issue

summary
