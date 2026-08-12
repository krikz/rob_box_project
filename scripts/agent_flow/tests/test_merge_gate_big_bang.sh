#!/bin/bash
# ============================================================================
# test_merge_gate_big_bang.sh — ADR-0013 acceptance suite for big-bang gate
#
# Verifies the rule (docs/adr/0013-incremental-delivery-over-big-bang.md):
#   PR > ${BIG_BANG_MAX_COMMITS} commits ИЛИ > ${BIG_BANG_MAX_LINES} строк
#   запрещён без явной метки `big-bang-override` на issue.
#
# Scenarios covered:
#   A. PR < 50 commits AND < 3000 lines, no override → needs-e2e SET (regression).
#   B. PR > 50 commits, no override → needs-e2e NOT set, comment posted.
#   C. PR > 3000 additions, no override → needs-e2e NOT set, comment posted.
#   D. PR > 50 commits, big-bang-override present → needs-e2e SET (override).
#   E. big-bang comment dedup: 2 ticks, comment posted only once.
#   F. small PR (lint) with no override → needs-review SET (regression for lint).
#
# Run:
#   bash scripts/agent_flow/tests/test_merge_gate_big_bang.sh
# ============================================================================
set -euo pipefail

TEST_LIB_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/mock_env.sh
. "$TEST_LIB_DIR/lib/mock_env.sh"

# ---------------------------------------------------------------------------
# Helper: minimal valid issue+PR fixture for the big-bang gate.
# Sets:
#   - ISSUE_LIST_JSON
#   - ISSUE_<n>_LABELS_JSON  (initial labels, pre-tick)
#   - ISSUE_<n>_STATE_JSON   (OPEN)
#   - ISSUE_<n>_COMMENTS_JSON (for kanban marker)
#   - ISSUE_<n>_COMMENTS_SINCE_JSON (for dedup check on big-bang comment)
#   - PR_HEAD_<branch>_JSON   (the PR with additions+commits)
#   - PR_LIST_ALL_OPEN_JSON   ([])
#   - PR_FOLLOWUP_JSON        ([])
#   - RATE_LIMIT_JSON         (non-zero remaining)
# Sets env defaults the script needs (HOME, PROFILE_ENV guard, etc.).
# ---------------------------------------------------------------------------
fixture_pr() {  # $1=issue $2=pr $3=branch $4=additions $5=commits_count $6=issue_labels_csv $7=title
    local issue="$1" pr="$2" branch="$3" additions="$4" commits="$5" labels_csv="$6" title="$7"
    [ -z "$title" ] && title="fix #${issue} demo"
    # Build a commits array of N empty objects to simulate N commits.
    # printf '{},%.0s' $(seq 1 N) produces "{},{},...,{}," (N+1 elements with
    # trailing comma). We strip the trailing comma and wrap in [ ].
    local commits_json
    if [ "$commits" -eq 0 ]; then
        commits_json='[]'
    else
        commits_json="$(printf '{},%.0s' $(seq 1 "$commits") | sed 's/,$//')"
        commits_json="[${commits_json}]"
    fi
    set_state ISSUE_LIST_JSON "[{\"number\":${issue},\"title\":\"${title}\",\"labels\":[{\"name\":\"hermes\"}$(printf ',{\"name\":\"%s\"}' ${labels_csv//,/ })],\"body\":\"kanban: t_dead${issue}\"}]"
    # ISSUE_LABELS_JSON must have name fields for has_label to work.
    local labels_json="{\"labels\":[{\"name\":\"hermes\"}"
    local l
    IFS=',' read -ra LARR <<< "$labels_csv"
    for l in "${LARR[@]}"; do
        [ -n "$l" ] && labels_json+=",{\"name\":\"$l\"}"
    done
    labels_json+="]}"
    set_state "ISSUE_${issue}_LABELS_JSON" "$labels_json"
    set_state "ISSUE_${issue}_STATE_JSON" '{"state":"OPEN"}'
    set_state "ISSUE_${issue}_COMMENTS_JSON" "{\"comments\":[{\"body\":\"kanban: t_dead${issue}\\n\"}]}"
    set_state "ISSUE_${issue}_COMMENTS_SINCE_JSON" '[]'
    set_state "ISSUE_${issue}_TIMELINE_JSON" '[]'
    set_state "PR_HEAD_${branch}_JSON" "[{\"number\":${pr},\"state\":\"OPEN\",\"baseRefName\":\"develop\",\"mergeable\":\"MERGEABLE\",\"mergeStateStatus\":\"CLEAN\",\"statusCheckRollup\":[{\"conclusion\":\"SUCCESS\"}],\"title\":\"[robot] fix #${issue}\",\"labels\":[{\"name\":\"agent:devops\"}],\"additions\":${additions},\"commits\":${commits_json}}]"
    set_state "PR_${pr}_COMMITS_JSON" '[]'
    set_state PR_LIST_ALL_OPEN_JSON '[]'
    set_state PR_FOLLOWUP_JSON '[]'
    set_state RATE_LIMIT_JSON '{"resources":{"core":{"remaining":5000}}}'
    set_state "BRANCH_PRESENT_${branch}" 1
}

# Compute branch name the way merge-gate.sh does it: z-{agent}/<issue>-<slug>
slugify_branch() {  # $1=issue $2=title
    local issue="$1" title="$2" slug
    slug="$(printf '%s' "$title" | tr '[:upper:]' '[:lower:]' \
        | sed -E 's/[^a-z0-9]+/-/g; s/^-+|-+$//g; s/-{2,}/-/g' \
        | cut -c1-40)"
    printf 'z-{agent}/%s-%s' "$issue" "$slug"
}

# ===========================================================================
# A. PR < 50 commits AND < 3000 lines, no override → needs-e2e SET.
# Regression: small PRs continue to flow through.
# ===========================================================================
test_A_small_pr_sets_needs_e2e() {
    new_test
    local branch
    branch="$(slugify_branch 3001 'fix #3001 small demo')"
    fixture_pr 3001 3002 "$branch" 200 5 "" 'fix #3001 small demo'

    run_merge_gate

    local journal
    journal="$(cat "$GH_JOURNAL")"

    # needs-e2e IS set.
    local needs_e2e_calls
    needs_e2e_calls="$(printf '%s\n' "$journal" | grep -c 'gh issue edit 3001 --add-label needs-e2e' || true)"
    assert_eq "1" "$needs_e2e_calls" "small PR → needs-e2e set once"

    # No big-bang comment posted.
    local bb_comments
    bb_comments="$(printf '%s\n' "$journal" | grep -c 'BIG-BANG' || true)"
    assert_eq "0" "$bb_comments" "no big-bang comment for small PR"
}

# ===========================================================================
# B. PR > 50 commits, no override → needs-e2e NOT set, comment posted.
# The case from PR #1080 (100 commits, 4850 lines).
# ===========================================================================
test_B_too_many_commits_blocks() {
    new_test
    local branch
    branch="$(slugify_branch 3003 'fix #3003 big-bang commits demo')"
    fixture_pr 3003 3004 "$branch" 200 100 "" 'fix #3003 big-bang commits demo'

    run_merge_gate

    local journal
    journal="$(cat "$GH_JOURNAL")"

    # needs-e2e NOT set.
    local needs_e2e_calls
    needs_e2e_calls="$(printf '%s\n' "$journal" | grep -c 'gh issue edit 3003 --add-label needs-e2e' || true)"
    assert_eq "0" "$needs_e2e_calls" "big-bang PR → needs-e2e NOT set"

    # One big-bang comment on issue.
    local issue_bb
    issue_bb="$(printf '%s\n' "$journal" | grep -c 'gh issue comment 3003' | head -1 || true)"
    # Also check the body text starts with the marker
    local issue_bb_text
    issue_bb_text="$(printf '%s\n' "$journal" | grep 'gh issue comment 3003' | grep -c '🚨' || true)"
    assert_eq "1" "$issue_bb_text" "big-bang comment posted on issue"

    # One big-bang comment on PR.
    local pr_bb
    pr_bb="$(printf '%s\n' "$journal" | grep -c 'gh pr comment 3004' | head -1 || true)"
    assert_eq "1" "$pr_bb" "big-bang comment posted on PR"
}

# ===========================================================================
# C. PR > 3000 additions, no override → needs-e2e NOT set, comment posted.
# ===========================================================================
test_C_too_many_lines_blocks() {
    new_test
    local branch
    branch="$(slugify_branch 3005 'fix #3005 big-bang lines demo')"
    fixture_pr 3005 3006 "$branch" 5000 10 "" 'fix #3005 big-bang lines demo'

    run_merge_gate

    local journal
    journal="$(cat "$GH_JOURNAL")"

    local needs_e2e_calls
    needs_e2e_calls="$(printf '%s\n' "$journal" | grep -c 'gh issue edit 3005 --add-label needs-e2e' || true)"
    assert_eq "0" "$needs_e2e_calls" "big-bang lines PR → needs-e2e NOT set"

    local bb_blocked_label
    bb_blocked_label="$(printf '%s\n' "$journal" | grep -c 'agent-flow:big-bang-blocked' || true)"
    assert_eq "1" "$bb_blocked_label" "big-bang-blocked label added to issue"
}

# ===========================================================================
# D. PR > 50 commits + override label → needs-e2e SET (override).
# ===========================================================================
test_D_override_label_allows() {
    new_test
    local branch
    branch="$(slugify_branch 3007 'fix #3007 override demo')"
    fixture_pr 3007 3008 "$branch" 5000 100 "big-bang-override" 'fix #3007 override demo'   # huge but with override

    run_merge_gate

    local journal
    journal="$(cat "$GH_JOURNAL")"

    local needs_e2e_calls
    needs_e2e_calls="$(printf '%s\n' "$journal" | grep -c 'gh issue edit 3007 --add-label needs-e2e' || true)"
    assert_eq "1" "$needs_e2e_calls" "override present → needs-e2e set"

    local bb_comments
    bb_comments="$(printf '%s\n' "$journal" | grep -c 'BIG-BANG' || true)"
    assert_eq "0" "$bb_comments" "override present → no big-bang comment"
}

# ===========================================================================
# E. big-bang comment dedup: 2 ticks, comment posted only once.
# ===========================================================================
test_E_big_bang_comment_dedup() {
    new_test
    local branch
    branch="$(slugify_branch 3009 'fix #3009 dedup demo')"
    fixture_pr 3009 3010 "$branch" 5000 100 "" 'fix #3009 dedup demo'

    # First tick — comment posted.
    run_merge_gate
    local first_journal
    first_journal="$(cat "$GH_JOURNAL")"
    local first_bb_issue
    first_bb_issue="$(printf '%s\n' "$first_journal" | grep 'gh issue comment 3009' | grep -c '🚨' || true)"
    assert_eq "1" "$first_bb_issue" "tick 1: big-bang comment posted on issue"

    # Second tick — mock should already have the comment in PR's COMMENTS_SINCE_JSON.
    # Update fixture to simulate the comment being there (use PR number, not issue,
    # because merge-gate queries comments of the PR, not the issue).
    set_state "ISSUE_3010_COMMENTS_SINCE_JSON" "[{\"body\":\"🚨 **PR #3010 BIG-BANG** — нарушение ADR-0013\"}]"
    # Don't re-init mocks — keep state across ticks (issue with single-tick fixture).
    # Re-run: comment must NOT be posted again.
    run_merge_gate
    local second_journal
    second_journal="$(cat "$GH_JOURNAL")"
    # Count of `gh issue comment 3009` calls total (should be exactly 1, not 2).
    local total_issue_bb
    total_issue_bb="$(printf '%s\n' "$second_journal" | grep -c 'gh issue comment 3009' | head -1 || true)"
    assert_eq "1" "$total_issue_bb" "tick 2: big-bang comment NOT re-posted (dedup)"
}

# ===========================================================================
# F. Small lint PR (no override) → needs-review SET (lint regression).
# ===========================================================================
test_F_small_lint_pr_sets_needs_review() {
    new_test
    local branch
    branch="$(slugify_branch 3011 'fix #3011 lint demo')"
    # Title with [lint] prefix → detect_pr_kind returns "lint".
    # Override fixtures: title has [lint] prefix.
    local issue=3011 pr=3012
    local title="[lint] fix #${issue} small demo"
    local slug
    slug="$(printf '%s' "$title" | tr '[:upper:]' '[:lower:]' \
        | sed -E 's/[^a-z0-9]+/-/g; s/^-+|-+$//g; s/-{2,}/-/g' \
        | cut -c1-40)"
    branch="z-{agent}/${issue}-${slug}"
    set_state ISSUE_LIST_JSON "[{\"number\":${issue},\"title\":\"${title}\",\"labels\":[{\"name\":\"hermes\"}],\"body\":\"kanban: t_dead${issue}\"}]"
    set_state "ISSUE_${issue}_LABELS_JSON" '{"labels":[{"name":"hermes"}]}'
    set_state "ISSUE_${issue}_STATE_JSON" '{"state":"OPEN"}'
    set_state "ISSUE_${issue}_COMMENTS_JSON" "{\"comments\":[{\"body\":\"kanban: t_dead${issue}\\n\"}]}"
    set_state "ISSUE_${issue}_COMMENTS_SINCE_JSON" '[]'
    set_state "ISSUE_${issue}_TIMELINE_JSON" '[]'
    set_state "PR_HEAD_${branch}_JSON" "[{\"number\":${pr},\"state\":\"OPEN\",\"baseRefName\":\"develop\",\"mergeable\":\"MERGEABLE\",\"mergeStateStatus\":\"CLEAN\",\"statusCheckRollup\":[{\"conclusion\":\"SUCCESS\"}],\"title\":\"${title}\",\"labels\":[{\"name\":\"agent:devops\"}],\"additions\":50,\"commits\":[{},{}]}]"
    set_state "PR_${pr}_COMMITS_JSON" '[]'
    set_state PR_LIST_ALL_OPEN_JSON '[]'
    set_state PR_FOLLOWUP_JSON '[]'
    set_state RATE_LIMIT_JSON '{"resources":{"core":{"remaining":5000}}}'
    set_state "BRANCH_PRESENT_${branch}" 1

    run_merge_gate

    local journal
    journal="$(cat "$GH_JOURNAL")"

    local needs_review_calls
    needs_review_calls="$(printf '%s\n' "$journal" | grep -c 'gh pr edit 3012 --repo .* --add-label needs-review' || true)"
    assert_eq "1" "$needs_review_calls" "small lint PR → needs-review set on PR"

    local bb_comments
    bb_comments="$(printf '%s\n' "$journal" | grep -c 'BIG-BANG' || true)"
    assert_eq "0" "$bb_comments" "no big-bang comment for small lint PR"
}

# ===========================================================================
# Run all tests.
# ===========================================================================
run_test "A. small PR → needs-e2e" test_A_small_pr_sets_needs_e2e
run_test "B. too many commits → block" test_B_too_many_commits_blocks
run_test "C. too many lines → block" test_C_too_many_lines_blocks
run_test "D. override label → allow" test_D_override_label_allows
run_test "E. big-bang comment dedup" test_E_big_bang_comment_dedup
run_test "F. small lint PR → needs-review" test_F_small_lint_pr_sets_needs_review

summary
