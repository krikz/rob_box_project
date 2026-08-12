#!/bin/bash
# ============================================================================
# test_merge_gate_post_merge.sh — ADR-0014 acceptance suite
#
# Scenarios covered (ADR §7):
#   A. MERGED + base=develop + e2e-done + OPEN → close reason=completed once.
#   B. MERGED + base=develop, no e2e-done     → OPEN, no close call.
#   C. e2e-done + OPEN PR (PR not merged)     → no close call.
#   D. close API failure → no destructive cleanup, next tick retries.
#   E. race merge→label: first tick no close; second tick (label appears) → close.
#   F. already CLOSED issue → close skipped, no duplicate cleanup comment.
#   G. merge-gate does NOT create e2e-done when MERGED without PASS provenance.
#   H. regression: follow-up detection over e2e-done stays green
#                  (regression per ADR §7 item 8: existing follow-up
#                   detection logic remains intact).
#
# Run:
#   bash scripts/agent_flow/tests/test_merge_gate_post_merge.sh
# ============================================================================
set -euo pipefail

TEST_LIB_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/mock_env.sh
. "$TEST_LIB_DIR/lib/mock_env.sh"

# ---------------------------------------------------------------------------
# Helper: build a minimal valid issue+PR fixture for a given issue_number.
# Branch is DERIVED from the issue title (exactly like merge-gate.sh derives
# it: z-{agent}/<issue>-<slugify(title)>) so the PR_HEAD_<branch> state key
# always matches what the script looks up. $3 is the issue title.
# Sets:
#   - ISSUE_LIST_JSON
#   - ISSUE_<n>_LABELS_JSON  (initial labels, pre-tick)
#   - ISSUE_<n>_STATE_JSON   (OPEN / CLOSED)
#   - ISSUE_<n>_COMMENTS_JSON (for kanban marker)
#   - ISSUE_<n>_COMMENTS_SINCE_JSON (for dedup check)
#   - ISSUE_<n>_TIMELINE_JSON (for done-at lookup)
#   - PR_HEAD_<branch>_JSON   (the merged PR)
#   - PR_<n>_COMMITS_JSON
#   - PR_LIST_ALL_OPEN_JSON   ([])
#   - PR_FOLLOWUP_JSON        ([])
#   - RATE_LIMIT_JSON         (non-zero remaining)
#   - BRANCH_PRESENT_<branch> (1)
# Sets env defaults the script needs (HOME, PROFILE_ENV guard, etc.).
# ---------------------------------------------------------------------------
fixture_merged_pass_proven() {  # $1=issue $2=pr $3=title
    local issue="$1" pr="$2" title="$3"
    local branch
    branch="z-{agent}/${issue}-$(printf '%s' "$title" \
        | tr '[:upper:]' '[:lower:]' \
        | sed -E 's/[^a-z0-9]+/-/g; s/^-+|-+$//g; s/-{2,}/-/g' \
        | cut -c1-40)"
    set_state ISSUE_LIST_JSON "[{\"number\":${issue},\"title\":\"${title}\",\"labels\":[{\"name\":\"hermes\"},{\"name\":\"e2e-done\"}],\"body\":\"kanban: t_dead${issue}\"}]"
    set_state "ISSUE_${issue}_LABELS_JSON" "{\"labels\":[{\"name\":\"hermes\"},{\"name\":\"e2e-done\"}]}"
    set_state "ISSUE_${issue}_STATE_JSON" '{"state":"OPEN"}'
    set_state "ISSUE_${issue}_COMMENTS_JSON" "{\"comments\":[{\"body\":\"kanban: t_dead${issue}\\n\"}]}"
    set_state "ISSUE_${issue}_COMMENTS_SINCE_JSON" '[]'
    set_state "ISSUE_${issue}_TIMELINE_JSON" "[{\"event\":\"labeled\",\"label\":{\"name\":\"e2e-done\"},\"created_at\":\"2026-08-11T20:00:00Z\"}]"
    set_state "PR_HEAD_${branch}_JSON" "[{\"number\":${pr},\"state\":\"MERGED\",\"baseRefName\":\"develop\",\"mergedAt\":\"2026-08-11T20:00:00Z\",\"mergeable\":\"MERGEABLE\",\"mergeStateStatus\":\"CLEAN\",\"statusCheckRollup\":[{\"conclusion\":\"SUCCESS\"}],\"title\":\"[robot] fix #${issue}\",\"labels\":[{\"name\":\"agent:devops\"}]}]"
    set_state "PR_${pr}_COMMITS_JSON" '[{"commit":{"committer":{"date":"2026-08-11T20:00:00Z"}}}]'
    set_state PR_LIST_ALL_OPEN_JSON '[]'
    set_state PR_FOLLOWUP_JSON '[]'
    set_state RATE_LIMIT_JSON '{"resources":{"core":{"remaining":5000}}}'
    set_state "BRANCH_PRESENT_${branch}" 1
}

# Compute branch name the way merge-gate.sh does it: z-{agent}/<issue>-<slug>
# where slug = lowercase(title), non-alnum -> -, trim, kebab-case, cap 40.
slugify_branch() {  # $1=issue $2=title
    local issue="$1" title="$2" slug
    slug="$(printf '%s' "$title" | tr '[:upper:]' '[:lower:]' \
        | sed -E 's/[^a-z0-9]+/-/g; s/^-+|-+$//g; s/-{2,}/-/g' \
        | cut -c1-40)"
    printf 'z-{agent}/%s-%s' "$issue" "$slug"
}

# ===========================================================================
# A. MERGED + base=develop + e2e-done + OPEN → close reason=completed once.
# ===========================================================================
test_A_merged_pass_proven_closes_once() {
    new_test
    local branch
    branch="$(slugify_branch 1082 'fix #1082 merged demo')"
    fixture_merged_pass_proven 1082 1084 'fix #1082 merged demo'

    run_merge_gate

    local journal
    journal="$(cat "$GH_JOURNAL")"

    # Acceptance §7 item 1: exactly one close call.
    local close_calls
    close_calls="$(printf '%s\n' "$journal" | grep -c 'gh issue close 1082 --reason completed' || true)"
    assert_eq "1" "$close_calls" "exactly one close call"

    # Acceptance §7 item 7: merge-gate must NOT add e2e-done itself.
    local add_e2e_done
    add_e2e_done="$(printf '%s\n' "$journal" | grep -c 'gh issue edit 1082 --add-label e2e-done' || true)"
    assert_eq "0" "$add_e2e_done" "merge-gate does not add e2e-done"

    # Cleanup comment published with close note.
    local cleanup_comment
    cleanup_comment="$(printf '%s\n' "$journal" | grep -c 'Issue закрыта' || true)"
    assert_eq "1" "$cleanup_comment" "cleanup comment published with close note"

    # State actually flipped to CLOSED via close mock.
    local state_now
    state_now="$(grep -E '^ISSUE_1082_STATE_JSON=' "$GH_STATE" | sed 's/^ISSUE_1082_STATE_JSON=//')"
    assert_contains '"CLOSED"' "$state_now" "issue state flipped to CLOSED"

    # Branch delete attempted exactly once. Branch derived from title
    # 'fix #1082 merged demo' → z-{agent}/1082-fix-1082-merged-demo.
    local del_calls
    del_calls="$(printf '%s\n' "$journal" | grep -c "gh api -X DELETE repos/.*/git/refs/heads/${branch}" || true)"
    assert_eq "1" "$del_calls" "remote branch delete attempted once"
}

# ===========================================================================
# B. MERGED + base=develop, no e2e-done → OPEN, no close call.
# ===========================================================================
test_B_merged_no_e2e_done_no_close() {
    new_test
    local issue=1105 branch
    branch="$(slugify_branch "$issue" 'fix #1105 no pass demo')"
    set_state ISSUE_LIST_JSON "[{\"number\":${issue},\"title\":\"fix #${issue} no pass demo\",\"labels\":[{\"name\":\"hermes\"}],\"body\":\"kanban: t_dead${issue}\"}]"
    set_state "ISSUE_${issue}_LABELS_JSON" '{"labels":[{"name":"hermes"}]}'
    set_state "ISSUE_${issue}_STATE_JSON" '{"state":"OPEN"}'
    set_state "ISSUE_${issue}_COMMENTS_JSON" "{\"comments\":[{\"body\":\"kanban: t_dead${issue}\\n\"}]}"
    set_state "ISSUE_${issue}_COMMENTS_SINCE_JSON" '[]'
    set_state "ISSUE_${issue}_TIMELINE_JSON" "[{\"event\":\"labeled\",\"label\":{\"name\":\"e2e-done\"},\"created_at\":\"2026-08-11T20:00:00Z\"}]"
    set_state "PR_HEAD_${branch}_JSON" "[{\"number\":1106,\"state\":\"MERGED\",\"baseRefName\":\"develop\",\"mergeable\":\"MERGEABLE\",\"mergeStateStatus\":\"CLEAN\",\"statusCheckRollup\":[{\"conclusion\":\"SUCCESS\"}],\"title\":\"[robot] fix #${issue}\",\"labels\":[]}]"
    set_state PR_1106_COMMITS_JSON '[]'
    set_state PR_LIST_ALL_OPEN_JSON '[]'
    set_state PR_FOLLOWUP_JSON '[]'
    set_state RATE_LIMIT_JSON '{"resources":{"core":{"remaining":5000}}}'
    set_state "BRANCH_PRESENT_${branch}" 1

    run_merge_gate

    local journal
    journal="$(cat "$GH_JOURNAL")"

    # No close call.
    local close_calls
    close_calls="$(printf '%s\n' "$journal" | grep -c 'gh issue close' || true)"
    assert_eq "0" "$close_calls" "no close call when e2e-done missing"

    # No branch delete (destructive cleanup deferred).
    local del_calls
    del_calls="$(printf '%s\n' "$journal" | grep -c 'gh api -X DELETE' || true)"
    assert_eq "0" "$del_calls" "no destructive branch delete without PASS provenance"

    # No archive / no comment in destructive branch path.
    local archive_calls
    archive_calls="$(printf '%s\n' "$journal" | grep -c 'hermes kanban' || true)"
    # kanban cli is invoked for free_stale_worktrees and archive, but with
    # no card_id-derived archive (we used kanban-marker comment, so card_id=t_dead<issue>;
    # the mocked hermes is invoked but no destructive branch delete).
    # Here we only assert that no branch delete occurred — already done.

    # Issue stays OPEN in our state (mock didn't flip).
    local state_now
    state_now="$(grep -E "^ISSUE_${issue}_STATE_JSON=" "$GH_STATE" | sed "s/^ISSUE_${issue}_STATE_JSON=//")"
    assert_contains '"OPEN"' "$state_now" "issue still OPEN (state untouched)"
}

# ===========================================================================
# C. e2e-done present, but PR is OPEN (not merged yet) → no close call.
# ===========================================================================
test_C_e2e_done_but_pr_open_no_close() {
    new_test
    local issue=2001 branch
    branch="$(slugify_branch "$issue" 'fix #2001 open demo')"
    set_state ISSUE_LIST_JSON "[{\"number\":${issue},\"title\":\"fix #${issue} open demo\",\"labels\":[{\"name\":\"hermes\"},{\"name\":\"e2e-done\"}],\"body\":\"kanban: t_dead${issue}\"}]"
    set_state "ISSUE_${issue}_LABELS_JSON" '{"labels":[{"name":"hermes"},{"name":"e2e-done"}]}'
    set_state "ISSUE_${issue}_STATE_JSON" '{"state":"OPEN"}'
    set_state "ISSUE_${issue}_COMMENTS_JSON" "{\"comments\":[{\"body\":\"kanban: t_dead${issue}\\n\"}]}"
    set_state "ISSUE_${issue}_COMMENTS_SINCE_JSON" '[]'
    set_state "ISSUE_${issue}_TIMELINE_JSON" "[{\"event\":\"labeled\",\"label\":{\"name\":\"e2e-done\"},\"created_at\":\"2026-08-11T20:00:00Z\"}]"
    set_state "PR_HEAD_${branch}_JSON" "[{\"number\":2002,\"state\":\"OPEN\",\"baseRefName\":\"develop\",\"mergeable\":\"MERGEABLE\",\"mergeStateStatus\":\"CLEAN\",\"statusCheckRollup\":[{\"conclusion\":\"SUCCESS\"}],\"title\":\"[robot] fix #${issue}\",\"labels\":[]}]"
    set_state PR_2002_COMMITS_JSON '[]'
    set_state PR_LIST_ALL_OPEN_JSON '[]'
    set_state PR_FOLLOWUP_JSON '[]'
    set_state RATE_LIMIT_JSON '{"resources":{"core":{"remaining":5000}}}'

    run_merge_gate

    local journal
    journal="$(cat "$GH_JOURNAL")"

    # e2e-done + OPEN PR — but no close. PR is still OPEN, so the script
    # walks into the "all green → needs-e2e/needs-review" branch and either
    # re-sets needs-e2e or detects e2e-done already and skips. Either way,
    # NO close call must happen.
    local close_calls
    close_calls="$(printf '%s\n' "$journal" | grep -c 'gh issue close' || true)"
    assert_eq "0" "$close_calls" "no close when PR still OPEN"
}

# ===========================================================================
# D. close API failure → destructive cleanup deferred, retry tick succeeds.
# ===========================================================================
test_D_close_failure_defers_then_retries() {
    new_test
    local branch
    branch="$(slugify_branch 1082 'fix #1082 retry demo')"
    fixture_merged_pass_proven 1082 1084 'fix #1082 retry demo'

    # First tick: force close failure.
    set_state GH_CLOSE_FAIL_ISSUE_1082 1

    run_merge_gate
    local journal1
    journal1="$(cat "$GH_JOURNAL")"

    # Close was attempted once.
    local close_attempts
    close_attempts="$(printf '%s\n' "$journal1" | grep -c 'gh issue close 1082 --reason completed' || true)"
    assert_eq "1" "$close_attempts" "close attempted once on first tick"

    # Destructive cleanup must NOT run: no branch delete, no archive card.
    local del_calls
    del_calls="$(printf '%s\n' "$journal1" | grep -c 'gh api -X DELETE' || true)"
    assert_eq "0" "$del_calls" "no branch delete on close failure"

    # Issue still OPEN (close failed).
    local state_after
    state_after="$(grep -E '^ISSUE_1082_STATE_JSON=' "$GH_STATE" | sed 's/^ISSUE_1082_STATE_JSON=//')"
    assert_contains '"OPEN"' "$state_after" "issue remains OPEN after close failure"

    # Second tick: clear failure flag, simulate retry.
    set_state GH_CLOSE_FAIL_ISSUE_1082 0

    # Reset journal to capture only the second tick.
    : >"$GH_JOURNAL"
    run_merge_gate
    local journal2
    journal2="$(cat "$GH_JOURNAL")"

    # Second tick calls close and proceeds with destructive cleanup.
    local close_calls2
    close_calls2="$(printf '%s\n' "$journal2" | grep -c 'gh issue close 1082 --reason completed' || true)"
    assert_eq "1" "$close_calls2" "close attempted on second tick"

    local del_calls2
    del_calls2="$(printf '%s\n' "$journal2" | grep -c 'gh api -X DELETE' || true)"
    assert_eq "1" "$del_calls2" "branch deleted on second tick (retry succeeded)"
}

# ===========================================================================
# E. Race merge→label: first tick sees MERGED without e2e-done; later tick
#    finds e2e-done and closes.
# ===========================================================================
test_E_race_merge_then_label() {
    new_test
    local branch issue=1082
    branch="$(slugify_branch "$issue" 'race demo')"
    # Tick 1: MERGED, no e2e-done.
    set_state ISSUE_LIST_JSON "[{\"number\":${issue},\"title\":\"race demo\",\"labels\":[{\"name\":\"hermes\"}],\"body\":\"kanban: t_dead${issue}\"}]"
    set_state "ISSUE_${issue}_LABELS_JSON" '{"labels":[{"name":"hermes"}]}'
    set_state "ISSUE_${issue}_STATE_JSON" '{"state":"OPEN"}'
    set_state "ISSUE_${issue}_COMMENTS_JSON" "{\"comments\":[{\"body\":\"kanban: t_dead${issue}\\n\"}]}"
    set_state "ISSUE_${issue}_COMMENTS_SINCE_JSON" '[]'
    set_state "ISSUE_${issue}_TIMELINE_JSON" "[{\"event\":\"labeled\",\"label\":{\"name\":\"e2e-done\"},\"created_at\":\"2026-08-11T20:00:00Z\"}]"
    set_state "PR_HEAD_${branch}_JSON" "[{\"number\":1083,\"state\":\"MERGED\",\"baseRefName\":\"develop\",\"mergeable\":\"MERGEABLE\",\"mergeStateStatus\":\"CLEAN\",\"statusCheckRollup\":[{\"conclusion\":\"SUCCESS\"}],\"title\":\"[robot] race demo\",\"labels\":[]}]"
    set_state PR_1083_COMMITS_JSON '[]'
    set_state PR_LIST_ALL_OPEN_JSON '[]'
    set_state PR_FOLLOWUP_JSON '[]'
    set_state RATE_LIMIT_JSON '{"resources":{"core":{"remaining":5000}}}'
    set_state "BRANCH_PRESENT_${branch}" 1

    run_merge_gate
    local journal1
    journal1="$(cat "$GH_JOURNAL")"

    local close_calls
    close_calls="$(printf '%s\n' "$journal1" | grep -c 'gh issue close' || true)"
    assert_eq "0" "$close_calls" "tick 1 (no e2e-done yet): no close call"

    local del_calls
    del_calls="$(printf '%s\n' "$journal1" | grep -c 'gh api -X DELETE' || true)"
    assert_eq "0" "$del_calls" "tick 1: no destructive cleanup"

    # Tick 2: e2e-process has now set e2e-done on the issue.
    set_state ISSUE_LIST_JSON "[{\"number\":${issue},\"title\":\"race demo\",\"labels\":[{\"name\":\"hermes\"},{\"name\":\"e2e-done\"}],\"body\":\"kanban: t_dead${issue}\"}]"
    set_state "ISSUE_${issue}_LABELS_JSON" '{"labels":[{"name":"hermes"},{"name":"e2e-done"}]}'

    : >"$GH_JOURNAL"
    run_merge_gate
    local journal2
    journal2="$(cat "$GH_JOURNAL")"

    local close_calls2
    close_calls2="$(printf '%s\n' "$journal2" | grep -c 'gh issue close' || true)"
    assert_eq "1" "$close_calls2" "tick 2 (after e2e-done): close call"

    local del_calls2
    del_calls2="$(printf '%s\n' "$journal2" | grep -c 'gh api -X DELETE' || true)"
    assert_eq "1" "$del_calls2" "tick 2: branch deleted"
}

# ===========================================================================
# F. Issue already CLOSED → close skipped, cleanup comment dedup keeps
#    behavior (no duplicate comment) — we assert no close API call and
#    no double-cleanup-comment for the same PR number.
# ===========================================================================
test_F_already_closed_no_duplicate() {
    new_test
    local issue=1082 branch
    branch="$(slugify_branch "$issue" 'already closed demo')"
    set_state ISSUE_LIST_JSON "[{\"number\":${issue},\"title\":\"already closed demo\",\"labels\":[{\"name\":\"hermes\"},{\"name\":\"e2e-done\"}],\"body\":\"kanban: t_dead${issue}\"}]"
    set_state "ISSUE_${issue}_LABELS_JSON" '{"labels":[{"name":"hermes"},{"name":"e2e-done"}]}'
    set_state "ISSUE_${issue}_STATE_JSON" '{"state":"CLOSED"}'
    # dedup: an existing ✅ comment is present so dedup skips a second one.
    set_state "ISSUE_${issue}_COMMENTS_JSON" "{\"comments\":[{\"body\":\"kanban: t_dead${issue}\\\\n\"},{\"body\":\"✅ PR #1084 смержен в develop. Cleanup: ветка удалена.\\\\n\"}]}"
    set_state "ISSUE_${issue}_COMMENTS_SINCE_JSON" '[{"body":"✅ PR #1084 смержен в develop. Cleanup: ветка удалена."}]'
    set_state "ISSUE_${issue}_TIMELINE_JSON" "[]"
    set_state "PR_HEAD_${branch}_JSON" "[{\"number\":1084,\"state\":\"MERGED\",\"baseRefName\":\"develop\",\"mergeable\":\"MERGEABLE\",\"mergeStateStatus\":\"CLEAN\",\"statusCheckRollup\":[{\"conclusion\":\"SUCCESS\"}],\"title\":\"[robot] already closed demo\",\"labels\":[]}]"
    set_state PR_1084_COMMITS_JSON '[]'
    set_state PR_LIST_ALL_OPEN_JSON '[]'
    set_state PR_FOLLOWUP_JSON '[]'
    set_state RATE_LIMIT_JSON '{"resources":{"core":{"remaining":5000}}}'
    set_state "BRANCH_PRESENT_${branch}" 1

    run_merge_gate
    local journal
    journal="$(cat "$GH_JOURNAL")"

    # Close API call: must NOT happen (state already CLOSED).
    local close_calls
    close_calls="$(printf '%s\n' "$journal" | grep -c 'gh issue close' || true)"
    assert_eq "0" "$close_calls" "no close call on already-CLOSED issue"

    # Comment: dedup must skip (existing ✅ PR #1084 смержен comment).
    local new_cleanup_comments
    new_cleanup_comments="$(printf '%s\n' "$journal" | grep -c 'gh issue comment 1082' || true)"
    # We expect 0 new ✅-prefixed comment because dedup skipped.
    # But the script may still call gh issue comment for dedup logic? No,
    # dedup is checked via gh api (comments list), not issue comment. So
    # the only `gh issue comment` calls would be for the new merged-cleanup
    # comment — which should be skipped due to dedup.
    assert_eq "0" "$new_cleanup_comments" "no duplicate merged-cleanup comment (dedup)"

    # Branch delete still happens (cleanup is destructive and idempotent).
    local del_calls
    del_calls="$(printf '%s\n' "$journal" | grep -c 'gh api -X DELETE' || true)"
    assert_eq "1" "$del_calls" "branch still deleted on already-CLOSED"
}

# ===========================================================================
# G. merge-gate does NOT add e2e-done when MERGED without PASS provenance.
# ===========================================================================
test_G_no_pass_no_e2e_done_added() {
    new_test
    local issue=1500 branch
    branch="$(slugify_branch "$issue" 'no pass demo')"
    set_state ISSUE_LIST_JSON "[{\"number\":${issue},\"title\":\"no pass demo\",\"labels\":[{\"name\":\"hermes\"}],\"body\":\"kanban: t_dead${issue}\"}]"
    set_state "ISSUE_${issue}_LABELS_JSON" '{"labels":[{"name":"hermes"}]}'
    set_state "ISSUE_${issue}_STATE_JSON" '{"state":"OPEN"}'
    set_state "ISSUE_${issue}_COMMENTS_JSON" "{\"comments\":[{\"body\":\"kanban: t_dead${issue}\\n\"}]}"
    set_state "ISSUE_${issue}_COMMENTS_SINCE_JSON" '[]'
    set_state "ISSUE_${issue}_TIMELINE_JSON" "[{\"event\":\"labeled\",\"label\":{\"name\":\"e2e-done\"},\"created_at\":\"2026-08-11T20:00:00Z\"}]"
    set_state "PR_HEAD_${branch}_JSON" "[{\"number\":1501,\"state\":\"MERGED\",\"baseRefName\":\"develop\",\"mergeable\":\"MERGEABLE\",\"mergeStateStatus\":\"CLEAN\",\"statusCheckRollup\":[{\"conclusion\":\"SUCCESS\"}],\"title\":\"[robot] no pass demo\",\"labels\":[]}]"
    set_state PR_1501_COMMITS_JSON '[]'
    set_state PR_LIST_ALL_OPEN_JSON '[]'
    set_state PR_FOLLOWUP_JSON '[]'
    set_state RATE_LIMIT_JSON '{"resources":{"core":{"remaining":5000}}}'
    set_state "BRANCH_PRESENT_${branch}" 1

    run_merge_gate
    local journal
    journal="$(cat "$GH_JOURNAL")"

    # merge-gate must NOT call add-label e2e-done.
    local add_calls
    add_calls="$(printf '%s\n' "$journal" | grep -c 'gh issue edit 1500 --add-label e2e-done' || true)"
    assert_eq "0" "$add_calls" "merge-gate does not add e2e-done (no PASS provenance)"

    # It must also NOT call close (already covered by B, repeated for clarity).
    local close_calls
    close_calls="$(printf '%s\n' "$journal" | grep -c 'gh issue close' || true)"
    assert_eq "0" "$close_calls" "no close when no e2e-done"
}

# ===========================================================================
# H. Regression: follow-up PR detection over e2e-done (pre-existing
#    behavior, must not regress with the new close logic).
# ===========================================================================
test_H_followup_pr_over_e2e_done_still_works() {
    new_test
    local issue=1082 branch
    branch="$(slugify_branch "$issue" 'followup demo')"
    # Issue has e2e-done + a follow-up OPEN PR (different branch, new commits).
    set_state ISSUE_LIST_JSON "[{\"number\":${issue},\"title\":\"followup demo\",\"labels\":[{\"name\":\"hermes\"},{\"name\":\"e2e-done\"}],\"body\":\"kanban: t_dead${issue}\"}]"
    set_state "ISSUE_${issue}_LABELS_JSON" '{"labels":[{"name":"hermes"},{"name":"e2e-done"}]}'
    set_state "ISSUE_${issue}_STATE_JSON" '{"state":"OPEN"}'
    set_state "ISSUE_${issue}_COMMENTS_JSON" "{\"comments\":[{\"body\":\"kanban: t_dead${issue}\\n\"}]}"
    set_state "ISSUE_${issue}_COMMENTS_SINCE_JSON" '[]'
    # timeline has labeled e2e-done at T0; PR commits include some after T0.
    set_state "ISSUE_${issue}_TIMELINE_JSON" "[{\"event\":\"labeled\",\"label\":{\"name\":\"e2e-done\"},\"created_at\":\"2026-08-10T10:00:00Z\"}]"
    # Canonical branch PR: NOT MERGED, still OPEN.
    set_state "PR_HEAD_${branch}_JSON" "[{\"number\":1084,\"state\":\"OPEN\",\"baseRefName\":\"develop\",\"mergeable\":\"MERGEABLE\",\"mergeStateStatus\":\"CLEAN\",\"statusCheckRollup\":[{\"conclusion\":\"SUCCESS\"}],\"title\":\"[robot] followup demo\",\"labels\":[]}]"
    # Follow-up OPEN PR (new commits) found via title search.
    set_state PR_FOLLOWUP_JSON "[{\"number\":1099,\"headRefName\":\"z-{agent}/1082-followup-2\",\"mergeStateStatus\":\"CLEAN\",\"updatedAt\":\"2026-08-11T11:00:00Z\"}]"
    set_state PR_1099_COMMITS_JSON '[{"commit":{"committer":{"date":"2026-08-11T11:00:00Z"}}},{"commit":{"committer":{"date":"2026-08-11T10:30:00Z"}}}]'
    set_state PR_LIST_ALL_OPEN_JSON '[]'
    set_state RATE_LIMIT_JSON '{"resources":{"core":{"remaining":5000}}}'

    run_merge_gate
    local journal
    journal="$(cat "$GH_JOURNAL")"

    # Follow-up logic must have detected the new commits and removed e2e-done.
    local remove_done
    remove_done="$(printf '%s\n' "$journal" | grep -c 'gh issue edit 1082 --remove-label e2e-done' || true)"
    assert_eq "1" "$remove_done" "follow-up detection removes e2e-done"

    # Follow-up logic must have added needs-e2e back.
    local add_needs_e2e
    add_needs_e2e="$(printf '%s\n' "$journal" | grep -c 'gh issue edit 1082 --add-label needs-e2e' || true)"
    assert_eq "1" "$add_needs_e2e" "follow-up detection adds needs-e2e"

    # No close call (we never reach the MERGED branch — PR is still OPEN).
    local close_calls
    close_calls="$(printf '%s\n' "$journal" | grep -c 'gh issue close' || true)"
    assert_eq "0" "$close_calls" "no close when follow-up flips label"
}

# ===========================================================================
# I. Regression (retro t_bea44d70): card archival still works after close.
#    The embedded `python3 -c` parsing `kanban show --json` must yield
#    card_state=done so the archive call fires. A re-indented snippet
#    (IndentationError) silently empties card_state → cards never archived.
# ===========================================================================
test_I_card_archived_after_close() {
    new_test
    local branch
    branch="$(slugify_branch 1082 'fix #1082 merged demo')"
    fixture_merged_pass_proven 1082 1084 'fix #1082 merged demo'

    run_merge_gate
    local journal
    journal="$(cat "$GH_JOURNAL")"

    # Close succeeded (prerequisite for destructive cleanup).
    local close_calls
    close_calls="$(printf '%s\n' "$journal" | grep -c 'gh issue close 1082 --reason completed' || true)"
    assert_eq "1" "$close_calls" "close called before archive"

    # Archive fired for the done card (t_dead1082 from kanban marker).
    local archive_calls
    archive_calls="$(printf '%s\n' "$journal" | grep -c 'hermes kanban --board robbox archive t_dead1082' || true)"
    assert_eq "1" "$archive_calls" "card archived after close (card_state parsed as done)"
}

# ===========================================================================
# Run
# ===========================================================================
run_test "A. MERGED + e2e-done → close once, no e2e-done added" test_A_merged_pass_proven_closes_once
run_test "B. MERGED without e2e-done → no close" test_B_merged_no_e2e_done_no_close
run_test "C. e2e-done + OPEN PR → no close" test_C_e2e_done_but_pr_open_no_close
run_test "D. close failure → defer cleanup, retry succeeds" test_D_close_failure_defers_then_retries
run_test "E. race merge→label: tick 1 no close, tick 2 close" test_E_race_merge_then_label
run_test "F. already-CLOSED → no close, dedup skip comment" test_F_already_closed_no_duplicate
run_test "G. no PASS provenance → merge-gate does NOT add e2e-done" test_G_no_pass_no_e2e_done_added
run_test "H. regression: follow-up over e2e-done still works" test_H_followup_pr_over_e2e_done_still_works
run_test "I. regression: card archived after close (card_state parse)" test_I_card_archived_after_close

summary