#!/bin/bash
# ============================================================================
# test_merge_gate_user_reopen.sh — issue #1391
#
# Bug (ретро 18.08 t_c4f1d5c8 / issue #1391): merge-gate закрывал issue #1363
# через 6 мин после user-reopen'а. Хронология:
#   11:19:44Z  PR #1366 MERGED into develop
#   09:53:29Z  e2e-done label проставлен e2e-process (PASS-proven)
#   11:21:58Z  issue closed (первый раз, ручной close)
#   12:06:15Z  user REOPENED ('свист так и не убран!!!')
#   14:14:37Z  user REOPENED again (после re-close)
#   14:20:48Z  **auto-closed merge-gate'ом** — БАГ
#
# Root cause: merge-gate (post-merge reconcile) видит `state=OPEN + e2e-done +
# PR MERGED into develop` → закрывает issue с reason=completed, **не
# проверяя**, что user сам вручную переоткрыл её после последнего
# проставления e2e-done. Метка e2e-done «протухла» (поставлена до reopen'а).
#
# Fix: перед закрытием проверить timeline issue на наличие события
# `reopened` ПОСЛЕ последней метки `e2e-done`. Если есть → НЕ закрывать,
# снять протухшую e2e-done (чтобы скрипт не зациклился), оставить
# audit-комментарий и вернуть issue в needs-e2e-ротацию (новый e2e-раунд
# на новом MERGE-commit, если юзер смёржит новый фикс; либо ручной
# воркфлоу если issue — false-positive PASS).
#
# Acceptance:
#   U1. MERGED + e2e-done + OPEN + user-reopened ПОСЛЕ e2e-done
#       → NO close call, e2e-done снят, audit comment опубликован.
#   U2. MERGED + e2e-done + OPEN + НЕТ user-reopened после e2e-done
#       → close call выполняется (existing behavior preserved).
#   U3. MERGED + e2e-done + OPEN + user-reopened ДО e2e-done (e.g.
#       тестовая гипотеза про race) → close call всё равно
#       выполняется (мы уважаем свежую e2e-done).
# ============================================================================
set -euo pipefail

TEST_LIB_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/mock_env.sh
. "$TEST_LIB_DIR/lib/mock_env.sh"

slugify_branch() {  # $1=issue $2=title
    local issue="$1" title="$2" slug
    slug="$(printf '%s' "$title" | tr '[:upper:]' '[:lower:]' \
        | sed -E 's/[^a-z0-9]+/-/g; s/^-+|-+$//g; s/-{2,}/-/g' \
        | cut -c1-40)"
    printf 'z-{agent}/%s-%s' "$issue" "$slug"
}

# ============================================================================
# U1. The bug: MERGED + e2e-done + OPEN + user-reopened AFTER e2e-done → no close
# ============================================================================
test_U1_user_reopen_after_e2e_done_blocks_close() {
    new_test
    local issue=1391 branch pr=1395
    branch="$(slugify_branch "$issue" 'autocloser user reopen demo')"
    set_state ISSUE_LIST_JSON "[{\"number\":${issue},\"title\":\"autocloser user reopen demo\",\"labels\":[{\"name\":\"hermes\"},{\"name\":\"e2e-done\"}],\"body\":\"kanban: t_dead${issue}\"}]"
    set_state "ISSUE_${issue}_LABELS_JSON" '{"labels":[{"name":"hermes"},{"name":"e2e-done"}]}'
    set_state "ISSUE_${issue}_STATE_JSON" '{"state":"OPEN"}'
    set_state "ISSUE_${issue}_COMMENTS_JSON" "{\"comments\":[{\"body\":\"kanban: t_dead${issue}\\n\"}]}"
    set_state "ISSUE_${issue}_COMMENTS_SINCE_JSON" '[]'
    # Timeline: e2e-done at T0, then user closed (auto), user reopened at T1
    # (which is AFTER e2e-done). This is the exact pattern from issue #1363.
    set_state "ISSUE_${issue}_TIMELINE_JSON" "[{\"event\":\"labeled\",\"label\":{\"name\":\"e2e-done\"},\"created_at\":\"2026-08-18T09:53:29Z\"},{\"event\":\"closed\",\"actor\":{\"login\":\"krikz\"},\"created_at\":\"2026-08-18T12:10:23Z\",\"state_reason\":\"completed\"},{\"event\":\"reopened\",\"actor\":{\"login\":\"krikz\"},\"created_at\":\"2026-08-18T14:14:37Z\",\"state_reason\":\"reopened\"}]"
    set_state "PR_HEAD_${branch}_JSON" "[{\"number\":${pr},\"state\":\"MERGED\",\"baseRefName\":\"develop\",\"mergedAt\":\"2026-08-18T11:19:44Z\",\"mergeable\":\"MERGEABLE\",\"mergeStateStatus\":\"CLEAN\",\"statusCheckRollup\":[{\"conclusion\":\"SUCCESS\"}],\"title\":\"[robot] fix #${issue}\",\"labels\":[]}]"
    set_state "PR_${pr}_COMMITS_JSON" "[]"
    set_state PR_LIST_ALL_OPEN_JSON '[]'
    set_state PR_FOLLOWUP_JSON '[]'
    set_state RATE_LIMIT_JSON '{"resources":{"core":{"remaining":5000}}}'
    set_state "BRANCH_PRESENT_${branch}" 1
    set_state "KANBAN_LIST_JSON" "[]"

    run_merge_gate

    local journal
    journal="$(cat "$GH_JOURNAL")"

    # THE BUG FIX: NO close call when user reopened after e2e-done.
    local close_calls
    close_calls="$(printf '%s\n' "$journal" | grep -c "gh issue close ${issue} --reason completed" || true)"
    assert_eq "0" "$close_calls" "NO close when user reopened AFTER e2e-done"

    # e2e-done must be removed (stale label, would re-trigger next tick).
    local rm_e2e
    rm_e2e="$(printf '%s\n' "$journal" | grep -c "gh issue edit ${issue} --remove-label e2e-done" || true)"
    assert_eq "1" "$rm_e2e" "stale e2e-done label removed"

    # needs-e2e must be re-added so issue returns to e2e rotation.
    local add_needs_e2e
    add_needs_e2e="$(printf '%s\n' "$journal" | grep -c "gh issue edit ${issue} --add-label needs-e2e" || true)"
    assert_eq "1" "$add_needs_e2e" "needs-e2e re-added"

    # Audit comment must be left explaining why auto-close was blocked.
    local audit
    audit="$(printf '%s\n' "$journal" | grep -c "user-reopened-this\|user-reopen\|reopen после" || true)"
    [ "$audit" -ge 1 ] || { echo "FAIL: audit comment missing (count=$audit)"; exit 1; }

    # No destructive branch delete (we never closed, so no cleanup).
    local del
    del="$(printf '%s\n' "$journal" | grep -c "gh api -X DELETE" || true)"
    assert_eq "0" "$del" "no branch delete when close skipped"
}

# ============================================================================
# U2. No user-reopen after e2e-done → existing behavior preserved (close fires)
# ============================================================================
test_U2_no_user_reopen_still_closes() {
    new_test
    local issue=1392 branch pr=1396
    branch="$(slugify_branch "$issue" 'no user reopen demo')"
    set_state ISSUE_LIST_JSON "[{\"number\":${issue},\"title\":\"no user reopen demo\",\"labels\":[{\"name\":\"hermes\"},{\"name\":\"e2e-done\"}],\"body\":\"kanban: t_dead${issue}\"}]"
    set_state "ISSUE_${issue}_LABELS_JSON" '{"labels":[{"name":"hermes"},{"name":"e2e-done"}]}'
    set_state "ISSUE_${issue}_STATE_JSON" '{"state":"OPEN"}'
    set_state "ISSUE_${issue}_COMMENTS_JSON" "{\"comments\":[{\"body\":\"kanban: t_dead${issue}\\n\"}]}"
    set_state "ISSUE_${issue}_COMMENTS_SINCE_JSON" '[]'
    # Timeline: e2e-done at T0, NOTHING after (no reopen).
    set_state "ISSUE_${issue}_TIMELINE_JSON" "[{\"event\":\"labeled\",\"label\":{\"name\":\"e2e-done\"},\"created_at\":\"2026-08-18T09:53:29Z\"}]"
    set_state "PR_HEAD_${branch}_JSON" "[{\"number\":${pr},\"state\":\"MERGED\",\"baseRefName\":\"develop\",\"mergedAt\":\"2026-08-18T11:19:44Z\",\"mergeable\":\"MERGEABLE\",\"mergeStateStatus\":\"CLEAN\",\"statusCheckRollup\":[{\"conclusion\":\"SUCCESS\"}],\"title\":\"[robot] fix #${issue}\",\"labels\":[]}]"
    set_state "PR_${pr}_COMMITS_JSON" "[]"
    set_state PR_LIST_ALL_OPEN_JSON '[]'
    set_state PR_FOLLOWUP_JSON '[]'
    set_state RATE_LIMIT_JSON '{"resources":{"core":{"remaining":5000}}}'
    set_state "BRANCH_PRESENT_${branch}" 1
    set_state "KANBAN_LIST_JSON" "[{\"id\":\"t_dead${issue}\",\"status\":\"done\"}]"

    run_merge_gate

    local journal
    journal="$(cat "$GH_JOURNAL")"

    # Existing behavior: close fires when no user-reopen.
    local close_calls
    close_calls="$(printf '%s\n' "$journal" | grep -c "gh issue close ${issue} --reason completed" || true)"
    assert_eq "1" "$close_calls" "close fires when no user-reopen (existing behavior)"

    # e2e-done should NOT be removed (it's still valid).
    local rm_e2e
    rm_e2e="$(printf '%s\n' "$journal" | grep -c "gh issue edit ${issue} --remove-label e2e-done" || true)"
    assert_eq "0" "$rm_e2e" "e2e-done NOT removed when no user-reopen"
}

# ============================================================================
# U3. User-reopened BEFORE e2e-done (rare race) → still close (fresh e2e-done)
# ============================================================================
test_U3_reopen_before_e2e_done_still_closes() {
    new_test
    local issue=1393 branch pr=1397
    branch="$(slugify_branch "$issue" 'reopen before e2e done')"
    set_state ISSUE_LIST_JSON "[{\"number\":${issue},\"title\":\"reopen before e2e done\",\"labels\":[{\"name\":\"hermes\"},{\"name\":\"e2e-done\"}],\"body\":\"kanban: t_dead${issue}\"}]"
    set_state "ISSUE_${issue}_LABELS_JSON" '{"labels":[{"name":"hermes"},{"name":"e2e-done"}]}'
    set_state "ISSUE_${issue}_STATE_JSON" '{"state":"OPEN"}'
    set_state "ISSUE_${issue}_COMMENTS_JSON" "{\"comments\":[{\"body\":\"kanban: t_dead${issue}\\n\"}]}"
    set_state "ISSUE_${issue}_COMMENTS_SINCE_JSON" '[]'
    # Timeline: user-reopened at T0, then e2e-done at T1 (later).
    # We respect fresh e2e-done over older user-reopen.
    set_state "ISSUE_${issue}_TIMELINE_JSON" "[{\"event\":\"reopened\",\"actor\":{\"login\":\"krikz\"},\"created_at\":\"2026-08-18T08:00:00Z\",\"state_reason\":\"reopened\"},{\"event\":\"labeled\",\"label\":{\"name\":\"e2e-done\"},\"created_at\":\"2026-08-18T11:00:00Z\"}]"
    set_state "PR_HEAD_${branch}_JSON" "[{\"number\":${pr},\"state\":\"MERGED\",\"baseRefName\":\"develop\",\"mergedAt\":\"2026-08-18T11:30:00Z\",\"mergeable\":\"MERGEABLE\",\"mergeStateStatus\":\"CLEAN\",\"statusCheckRollup\":[{\"conclusion\":\"SUCCESS\"}],\"title\":\"[robot] fix #${issue}\",\"labels\":[]}]"
    set_state "PR_${pr}_COMMITS_JSON" "[]"
    set_state PR_LIST_ALL_OPEN_JSON '[]'
    set_state PR_FOLLOWUP_JSON '[]'
    set_state RATE_LIMIT_JSON '{"resources":{"core":{"remaining":5000}}}'
    set_state "BRANCH_PRESENT_${branch}" 1
    set_state "KANBAN_LIST_JSON" "[{\"id\":\"t_dead${issue}\",\"status\":\"done\"}]"

    run_merge_gate

    local journal
    journal="$(cat "$GH_JOURNAL")"

    local close_calls
    close_calls="$(printf '%s\n' "$journal" | grep -c "gh issue close ${issue} --reason completed" || true)"
    assert_eq "1" "$close_calls" "close fires when reopen happened BEFORE e2e-done (fresh label wins)"
}

# ============================================================================
# U4. Timeline unreadable (gh api failure) → defer destructive cleanup
# (ADR §4 req 4: conservative on uncertainty — don't risk closing
# something user might have just reopened)
# ============================================================================
test_U4_timeline_unreadable_defer_close() {
    new_test
    local issue=1394 branch pr=1398
    branch="$(slugify_branch "$issue" 'timeline unreadable demo')"
    set_state ISSUE_LIST_JSON "[{\"number\":${issue},\"title\":\"timeline unreadable demo\",\"labels\":[{\"name\":\"hermes\"},{\"name\":\"e2e-done\"}],\"body\":\"kanban: t_dead${issue}\"}]"
    set_state "ISSUE_${issue}_LABELS_JSON" '{"labels":[{"name":"hermes"},{"name":"e2e-done"}]}'
    set_state "ISSUE_${issue}_STATE_JSON" '{"state":"OPEN"}'
    set_state "ISSUE_${issue}_COMMENTS_JSON" "{\"comments\":[{\"body\":\"kanban: t_dead${issue}\\n\"}]}"
    set_state "ISSUE_${issue}_COMMENTS_SINCE_JSON" '[]'
    # Empty timeline simulates gh api failure / rate-limit / network drop.
    set_state "ISSUE_${issue}_TIMELINE_JSON" '[]'
    set_state "PR_HEAD_${branch}_JSON" "[{\"number\":${pr},\"state\":\"MERGED\",\"baseRefName\":\"develop\",\"mergedAt\":\"2026-08-18T11:19:44Z\",\"mergeable\":\"MERGEABLE\",\"mergeStateStatus\":\"CLEAN\",\"statusCheckRollup\":[{\"conclusion\":\"SUCCESS\"}],\"title\":\"[robot] fix #${issue}\",\"labels\":[]}]"
    set_state "PR_${pr}_COMMITS_JSON" "[]"
    set_state PR_LIST_ALL_OPEN_JSON '[]'
    set_state PR_FOLLOWUP_JSON '[]'
    set_state RATE_LIMIT_JSON '{"resources":{"core":{"remaining":5000}}}'
    set_state "BRANCH_PRESENT_${branch}" 1
    set_state "KANBAN_LIST_JSON" "[]"

    run_merge_gate

    local journal
    journal="$(cat "$GH_JOURNAL")"

    # Empty timeline = can't prove no user-reopen → DON'T close (safer).
    # This differs from the existing "state unreadable → defer cleanup"
    # branch: there the state itself is unreadable. Here the state IS
    # readable (OPEN) but the timeline lacks the reopen history.
    local close_calls
    close_calls="$(printf '%s\n' "$journal" | grep -c "gh issue close ${issue} --reason completed" || true)"
    assert_eq "0" "$close_calls" "no close when timeline unreadable (can't prove no-reopen)"
}

run_test "U1_user_reopen_after_e2e_done_blocks_close" test_U1_user_reopen_after_e2e_done_blocks_close
run_test "U2_no_user_reopen_still_closes"           test_U2_no_user_reopen_still_closes
run_test "U3_reopen_before_e2e_done_still_closes"   test_U3_reopen_before_e2e_done_still_closes
run_test "U4_timeline_unreadable_defer_close"       test_U4_timeline_unreadable_defer_close

echo
echo "=================================="
echo "Summary: $TESTS_PASSED/$TESTS_TOTAL passed${FAILED_NAMES:+; FAILED: ${FAILED_NAMES[*]}}"
echo "=================================="
exit $(( TESTS_FAILED > 0 ? 1 : 0 ))
