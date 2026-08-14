#!/bin/bash
# ============================================================================
# test_merge_gate_e2e_done_review.sh — reconcile e2e-done → needs-review
#
# Ретро 13.08 t_92ec94f3 (#1188, Q22): после e2e-done процесс молчал —
# issue с e2e-done и OPEN PR не получала needs-review (наблюдение 12.08:
# #929/#933 e2e-done без needs-review; kanban: все карточки done, ready=0).
# Причина: post_round_sweep в e2e-process лечил только issue-side
# (e2e-done + remove needs-e2e), PR-side (needs-review) не делал; дальше
# issue «молчит» — e2e-process скипает (e2e-done), merge-gate тоже.
#
# Фикс (merge-gate reconciliation loop, 5m): e2e-done + OPEN PR
# (base=develop) → add-label needs-review на PR + remove needs-e2e с PR.
#
# Scenarios:
#   M1. e2e-done + OPEN PR (base develop) → needs-review поставлен на PR,
#       close НЕ вызывается.
#   M2. повторный тик (PR уже с needs-review) → идемпотентно, close НЕ
#       вызывается, issue остаётся OPEN.
#   M3. e2e-done + MERGED PR → НЕ needs-review; идёт в post-merge close
#       (ADR-0014) — защита от регрессии.
#   M4. DRY_RUN=true → gh pr edit не вызывается.
#
# Run:
#   bash scripts/agent_flow/tests/test_merge_gate_e2e_done_review.sh
# ============================================================================
set -euo pipefail

TEST_LIB_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/mock_env.sh
. "$TEST_LIB_DIR/lib/mock_env.sh"

# Compute branch name the way merge-gate.sh does it.
slugify_branch() {  # $1=issue $2=title
    local issue="$1" title="$2" slug
    slug="$(printf '%s' "$title" | tr '[:upper:]' '[:lower:]' \
        | sed -E 's/[^a-z0-9]+/-/g; s/^-+|-+$//g; s/-{2,}/-/g' \
        | cut -c1-40)"
    printf 'z-{agent}/%s-%s' "$issue" "$slug"
}

# ---------------------------------------------------------------------------
# Common fixture: issue with e2e-done (OPEN), OPEN PR targeting develop.
# $1=issue $2=pr $3=title $4=pr_state $5=pr_labels_csv (optional)
# ---------------------------------------------------------------------------
fixture_e2e_done_open_pr() {
    local issue="$1" pr="$2" title="$3" pr_state="$4" pr_labels_csv="${5:-}"
    local branch
    branch="$(slugify_branch "$issue" "$title")"
    local pr_labels_json="[]"
    if [ -n "$pr_labels_csv" ]; then
        pr_labels_json="[$(printf '%s' "$pr_labels_csv" \
            | tr ',' '\n' \
            | sed 's/.*/{"name":"&"}/' \
            | paste -sd, -)]"
    fi
    set_state ISSUE_LIST_JSON "[{\"number\":${issue},\"title\":\"${title}\",\"labels\":[{\"name\":\"hermes\"},{\"name\":\"e2e-done\"}],\"body\":\"kanban: t_dead${issue}\"}]"
    set_state "ISSUE_${issue}_LABELS_JSON" '{"labels":[{"name":"hermes"},{"name":"e2e-done"}]}'
    set_state "ISSUE_${issue}_STATE_JSON" '{"state":"OPEN"}'
    set_state "ISSUE_${issue}_COMMENTS_JSON" "{\"comments\":[{\"body\":\"kanban: t_dead${issue}\\n\"}]}"
    set_state "ISSUE_${issue}_COMMENTS_SINCE_JSON" '[]'
    set_state "ISSUE_${issue}_TIMELINE_JSON" "[]"
    set_state "PR_HEAD_${branch}_JSON" "[{\"number\":${pr},\"state\":\"${pr_state}\",\"baseRefName\":\"develop\",\"mergeable\":\"MERGEABLE\",\"mergeStateStatus\":\"CLEAN\",\"statusCheckRollup\":[{\"conclusion\":\"SUCCESS\"}],\"title\":\"[robot] ${title}\",\"labels\":${pr_labels_json}}]"
    set_state "PR_${pr}_COMMITS_JSON" '[]'
    set_state PR_LIST_ALL_OPEN_JSON '[]'
    set_state PR_FOLLOWUP_JSON '[]'
    set_state RATE_LIMIT_JSON '{"resources":{"core":{"remaining":5000}}}'
    set_state "KANBAN_LIST_JSON" "[{\"id\":\"t_dead${issue}\",\"status\":\"done\"}]"
    if [ "$pr_state" = "MERGED" ]; then
        set_state "BRANCH_PRESENT_${branch}" 1
    fi
}

# ===========================================================================
# M1. e2e-done + OPEN PR → needs-review поставлен на PR, close НЕ вызывается.
# ===========================================================================
test_M1_e2e_done_open_pr_sets_needs_review() {
    new_test
    local issue=929 pr=1184
    fixture_e2e_done_open_pr "$issue" "$pr" 'voice 929 demo' OPEN

    run_merge_gate
    local journal
    journal="$(cat "$GH_JOURNAL")"

    # 1) needs-review добавлен на PR ровно один раз (journal: pr edit <n> --repo ... --add-label needs-review).
    local add_review
    add_review="$(printf '%s\n' "$journal" | grep -cE "gh pr edit ${pr} .*--add-label needs-review" || true)"
    assert_eq "1" "$add_review" "needs-review added to PR once" || return 1

    # 2) needs-e2e снят с PR (если оставалась от старого цикла).
    local remove_needs_e2e
    remove_needs_e2e="$(printf '%s\n' "$journal" | grep -cE "gh pr edit ${pr} .*--remove-label needs-e2e" || true)"
    assert_eq "1" "$remove_needs_e2e" "needs-e2e removed from PR" || return 1

    # 3) close НЕ вызывается (PR ещё OPEN, юзер не мержил).
    local close_calls
    close_calls="$(printf '%s\n' "$journal" | grep -c 'gh issue close' || true)"
    assert_eq "0" "$close_calls" "no close when PR still OPEN" || return 1

    # 4) destructive cleanup НЕ запускается.
    local del_calls
    del_calls="$(printf '%s\n' "$journal" | grep -c 'gh api -X DELETE' || true)"
    assert_eq "0" "$del_calls" "no destructive branch delete for OPEN PR" || return 1

    # 5) Issue остаётся OPEN.
    local state_now
    state_now="$(grep -E "^ISSUE_${issue}_STATE_JSON=" "$GH_STATE" | sed "s/^ISSUE_${issue}_STATE_JSON=//")"
    assert_contains '"OPEN"' "$state_now" "issue stays OPEN (awaiting review+merge)" || return 1
}

# ===========================================================================
# M2. повторный тик (PR уже с needs-review) → идемпотентно, close НЕ
#     вызывается, issue остаётся OPEN.
# ===========================================================================
test_M2_reconcile_idempotent_on_retick() {
    new_test
    local issue=933 pr=1171
    fixture_e2e_done_open_pr "$issue" "$pr" 'voice 933 demo' OPEN 'needs-review'

    run_merge_gate
    local journal
    journal="$(cat "$GH_JOURNAL")"

    # needs-review add повторяется (no-op), close не происходит.
    local add_review
    add_review="$(printf '%s\n' "$journal" | grep -cE "gh pr edit ${pr} .*--add-label needs-review" || true)"
    assert_eq "1" "$add_review" "idempotent add-label still called" || return 1

    local close_calls
    close_calls="$(printf '%s\n' "$journal" | grep -c 'gh issue close' || true)"
    assert_eq "0" "$close_calls" "no close on retick" || return 1

    local state_now
    state_now="$(grep -E "^ISSUE_${issue}_STATE_JSON=" "$GH_STATE" | sed "s/^ISSUE_${issue}_STATE_JSON=//")"
    assert_contains '"OPEN"' "$state_now" "issue stays OPEN" || return 1
}

# ===========================================================================
# M3. e2e-done + MERGED PR → НЕ needs-review; идёт в post-merge close
#     (ADR-0014). Защита от регрессии: reconcile не перехватывает merged.
# ===========================================================================
test_M3_e2e_done_merged_pr_still_closes() {
    new_test
    local issue=1082 pr=1084
    fixture_e2e_done_open_pr "$issue" "$pr" 'merged demo' MERGED

    run_merge_gate
    local journal
    journal="$(cat "$GH_JOURNAL")"

    # close вызван (ADR-0014 path не сломан).
    local close_calls
    close_calls="$(printf '%s\n' "$journal" | grep -c "gh issue close ${issue} --reason completed" || true)"
    assert_eq "1" "$close_calls" "merged+e2e-done → close still happens" || return 1

    # needs-review НЕ ставится на merged PR (ревью уже не нужно).
    local add_review
    add_review="$(printf '%s\n' "$journal" | grep -cE "gh pr edit ${pr} .*--add-label needs-review" || true)"
    assert_eq "0" "$add_review" "no needs-review for MERGED PR" || return 1
}

# ===========================================================================
# M4. DRY_RUN=true → reconcile не применяется (нет gh pr edit).
# ===========================================================================
test_M4_dry_run_no_pr_edit() {
    new_test
    local issue=940 pr=1190
    fixture_e2e_done_open_pr "$issue" "$pr" 'dry run demo' OPEN

    DRY_RUN=true run_merge_gate
    local journal
    journal="$(cat "$GH_JOURNAL")"

    local add_review
    add_review="$(printf '%s\n' "$journal" | grep -cE "gh pr edit ${pr} .*--add-label needs-review" || true)"
    assert_eq "0" "$add_review" "DRY_RUN: no pr edit side effect" || return 1
}

# ===========================================================================
# M5. e2e-done + OPEN PR, НО PR создан ПОСЛЕ e2e-done → needs-review НЕ
#     ставится; e2e-done снимается, возврат в needs-e2e (ретро 14.08
#     t_28afb585, пункт 4: PR не тестировался — раунд прошёл до создания PR).
# ===========================================================================
test_M5_pr_created_after_e2e_done_back_to_needs_e2e() {
    new_test
    local issue=1238 pr=1238
    fixture_e2e_done_open_pr "$issue" "$pr" 'voice 1238 demo' OPEN
    # e2e-done повешен в 10:35Z, PR создан в 12:00Z (позже).
    set_state "ISSUE_${issue}_TIMELINE_JSON" '[{"event":"labeled","label":{"name":"e2e-done"},"created_at":"2026-08-14T10:35:00Z"}]'
    set_state "PR_${pr}_VIEW_JSON" '{"createdAt":"2026-08-14T12:00:00Z"}'

    run_merge_gate
    local journal
    journal="$(cat "$GH_JOURNAL")"

    # needs-review НЕ ставится на PR.
    local add_review
    add_review="$(printf '%s\n' "$journal" | grep -cE "gh pr edit ${pr} .*--add-label needs-review" || true)"
    assert_eq "0" "$add_review" "PR created after e2e-done → needs-review NOT set" || return 1

    # e2e-done снят с issue, needs-e2e поставлен (возврат в ротацию).
    local remove_done
    remove_done="$(printf '%s\n' "$journal" | grep -c "gh issue edit ${issue} --remove-label e2e-done" || true)"
    assert_eq "1" "$remove_done" "e2e-done removed from issue" || return 1
    local add_e2e
    add_e2e="$(printf '%s\n' "$journal" | grep -c "gh issue edit ${issue} --add-label needs-e2e" || true)"
    assert_eq "1" "$add_e2e" "needs-e2e added back" || return 1

    # Комментарий-объяснение постится.
    local comment
    comment="$(printf '%s\n' "$journal" | grep -c 'e2e-done снят — PR' || true)"
    assert_eq "1" "$comment" "explanation comment posted" || return 1
}

# ===========================================================================
# M6. e2e-done + OPEN PR, PR создан ДО e2e-done → обычный reconcile
#     (needs-review ставится). Регрессия M1 с данными timeline.
# ===========================================================================
test_M6_pr_created_before_e2e_done_still_reconciles() {
    new_test
    local issue=944 pr=1195
    fixture_e2e_done_open_pr "$issue" "$pr" 'voice 944 demo' OPEN
    # e2e-done повешен в 12:00Z, PR создан в 10:00Z (раньше) → нормально.
    set_state "ISSUE_${issue}_TIMELINE_JSON" '[{"event":"labeled","label":{"name":"e2e-done"},"created_at":"2026-08-14T12:00:00Z"}]'
    set_state "PR_${pr}_VIEW_JSON" '{"createdAt":"2026-08-14T10:00:00Z"}'

    run_merge_gate
    local journal
    journal="$(cat "$GH_JOURNAL")"

    local add_review
    add_review="$(printf '%s\n' "$journal" | grep -cE "gh pr edit ${pr} .*--add-label needs-review" || true)"
    assert_eq "1" "$add_review" "PR created before e2e-done → needs-review still set" || return 1

    local remove_done
    remove_done="$(printf '%s\n' "$journal" | grep -c "gh issue edit ${issue} --remove-label e2e-done" || true)"
    assert_eq "0" "$remove_done" "e2e-done NOT removed (PR был протестирован)" || return 1
}

# ===========================================================================
# Run
# ===========================================================================
run_test "M1. e2e-done + OPEN PR → needs-review" test_M1_e2e_done_open_pr_sets_needs_review
run_test "M2. retick idempotent, no close" test_M2_reconcile_idempotent_on_retick
run_test "M3. merged PR still closes (regression)" test_M3_e2e_done_merged_pr_still_closes
run_test "M4. DRY_RUN no side effects" test_M4_dry_run_no_pr_edit
run_test "M5. PR after e2e-done → back to needs-e2e" test_M5_pr_created_after_e2e_done_back_to_needs_e2e
run_test "M6. PR before e2e-done → reconcile" test_M6_pr_created_before_e2e_done_still_reconciles

summary
