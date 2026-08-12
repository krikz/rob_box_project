#!/bin/bash
# ============================================================================
# test_merge_gate_stale_branch.sh — ретро 12.08 t_d3aeaa9b acceptance tests
#
# Verifies the stale-branch re-commit guard in agent-flow-merge-gate.sh:
#   Сценарий: ветка УЖЕ влита в develop (есть MERGED PR с тем же head), но
#   воркер коммитил в неё ПОСЛЕ merge (база устарела; re-коммиты = дубли
#   merged-содержимого) и открыл НОВЫЙ PR. Diff такого PR vs develop — это
#   РЕГРЕССИЯ (удаляет влитые voice-фиксы). Guard: коммент в issue + НЕ
#   ставим needs-e2e, чтобы регрессионный дифф не ушёл в e2e-ротацию.
#
# Scenarios covered:
#   A. OPEN PR, head-ветка уже имеет MERGED PR (другой номер) → needs-e2e
#      НЕ ставится, блокирующий комментарий постится (ретро 12.08).
#   B. OPEN PR, head-ветка НЕ имеет merged PR → needs-e2e ставится как раньше
#      (регрессия: обычный флоу не сломан).
#   C. scan-all-prs: ретро-ветка z-devops/t_<id>-... (нет hermes-issue) с
#      MERGED PR на том же head → комментарий на PR постится.
#   D. stale-branch коммент дедуплицируется (2 тика → 1 коммент за 24h).
#
# Run:
#   bash scripts/agent_flow/tests/test_merge_gate_stale_branch.sh
# ============================================================================
set -euo pipefail

TEST_LIB_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/mock_env.sh
. "$TEST_LIB_DIR/lib/mock_env.sh"

# ---------------------------------------------------------------------------
# Helper: minimal valid issue+PR fixture for the stale-branch guard.
# Sets the same keys as fixture_pr in test_merge_gate_big_bang.sh, plus:
#   - PR_MERGED_HEAD_<branch>_JSON (по умолчанию пусто = нет влитого PR)
#   - PR_LIST_ALL_OPEN_JSON (по умолчанию [])
# ---------------------------------------------------------------------------
fixture_stale_pr() {  # $1=issue $2=pr $3=branch $4=merged_pr_number $5=title (default "fix #N demo")
    local issue="$1" pr="$2" branch="$3" merged_pr="$4"
    local title="${5:-fix #${issue} demo}"
    # ВАЖНО: merge-gate генерирует имя ветки из title через slugify():
    #   z-{agent}/<issue>-<slugify(title)>
    # Поэтому ISSUE_LIST title должен давать ровно ту ветку, для которой
    # ниже записан PR_HEAD_<branch>_JSON — иначе PR не найдётся ("no open
    # PR ... yet — skip") и guard не выполнится.
    set_state ISSUE_LIST_JSON "[{\"number\":${issue},\"title\":\"${title}\",\"labels\":[{\"name\":\"hermes\"}],\"body\":\"kanban: t_dead${issue}\"}]"
    set_state "ISSUE_${issue}_LABELS_JSON" '{"labels":[{"name":"hermes"}]}'
    set_state "ISSUE_${issue}_STATE_JSON" '{"state":"OPEN"}'
    set_state "ISSUE_${issue}_COMMENTS_JSON" "{\"comments\":[{\"body\":\"kanban: t_dead${issue}\\n\"}]}"
    set_state "ISSUE_${issue}_COMMENTS_SINCE_JSON" '[]'
    set_state "ISSUE_${issue}_TIMELINE_JSON" '[]'
    set_state "PR_HEAD_${branch}_JSON" "[{\"number\":${pr},\"state\":\"OPEN\",\"baseRefName\":\"develop\",\"mergeable\":\"MERGEABLE\",\"mergeStateStatus\":\"CLEAN\",\"statusCheckRollup\":[{\"conclusion\":\"SUCCESS\"}],\"title\":\"fix #${issue} demo\",\"labels\":[{\"name\":\"agent:devops\"}],\"additions\":50,\"commits\":[{},{}]}]"
    set_state "PR_${pr}_COMMITS_JSON" '[]'
    if [ -n "$merged_pr" ]; then
        set_state "PR_MERGED_HEAD_${branch}_JSON" "[{\"number\":${merged_pr},\"state\":\"MERGED\"}]"
    else
        # Пусто = нет влитого PR; mock-fallback на PR_HEAD_* вернёт текущий
        # open PR, и guard увидит тот же номер → не блокирует.
        set_state "PR_MERGED_HEAD_${branch}_JSON" ''
    fi
    set_state PR_LIST_ALL_OPEN_JSON '[]'
    set_state PR_FOLLOWUP_JSON '[]'
    set_state RATE_LIMIT_JSON '{"resources":{"core":{"remaining":5000}}}'
    set_state "BRANCH_PRESENT_${branch}" 1
}

slugify_branch() {  # $1=issue $2=title
    local issue="$1" title="$2" slug
    slug="$(printf '%s' "$title" | tr '[:upper:]' '[:lower:]' \
        | sed -E 's/[^a-z0-9]+/-/g; s/^-+|-+$//g; s/-{2,}/-/g' \
        | cut -c1-40)"
    printf 'z-{agent}/%s-%s' "$issue" "$slug"
}

# ===========================================================================
# A. OPEN PR, head уже влит (merged PR #3199) → needs-e2e НЕ ставится,
#    блокирующий коммент постится. (Сценарий ретро 12.08 t_d3aeaa9b.)
# ===========================================================================
test_A_stale_branch_blocks_needs_e2e() {
    new_test
    local branch
    branch="$(slugify_branch 3201 'fix #3201 stale demo')"
    fixture_stale_pr 3201 3202 "$branch" 3199 'fix #3201 stale demo'

    run_merge_gate

    local journal
    journal="$(cat "$GH_JOURNAL")"

    # needs-e2e НЕ ставится (ни на issue, ни на PR).
    local needs_e2e_calls
    needs_e2e_calls="$(printf '%s\n' "$journal" | grep -c -- '--add-label needs-e2e' || true)"
    assert_eq "0" "$needs_e2e_calls" "stale branch → needs-e2e NOT set"

    # Блокирующий коммент постится.
    local block_comments
    block_comments="$(printf '%s\n' "$journal" | grep -c 'stale-branch re-commit detected' || true)"
    assert_eq "1" "$block_comments" "stale branch → block comment posted"
}

# ===========================================================================
# B. OPEN PR, head НЕ влит → needs-e2e ставится (обычный флоу не сломан).
# ===========================================================================
test_B_fresh_branch_sets_needs_e2e() {
    new_test
    local branch
    branch="$(slugify_branch 3203 'fix #3203 fresh demo')"
    fixture_stale_pr 3203 3204 "$branch" "" 'fix #3203 fresh demo'

    run_merge_gate

    local journal
    journal="$(cat "$GH_JOURNAL")"

    local needs_e2e_calls
    needs_e2e_calls="$(printf '%s\n' "$journal" | grep -c 'gh issue edit 3203 --add-label needs-e2e' || true)"
    assert_eq "1" "$needs_e2e_calls" "fresh branch → needs-e2e set once"

    local block_comments
    block_comments="$(printf '%s\n' "$journal" | grep -c 'stale-branch re-commit detected' || true)"
    assert_eq "0" "$block_comments" "fresh branch → no stale-branch comment"
}

# ===========================================================================
# C. scan-all-prs: ретро-ветка z-devops/t_<id>-... (нет hermes-issue) с MERGED
#    PR на том же head → блокирующий коммент на PR. (Сценарий из ретро 12.08:
#    z-devops/t_5af222ea-telegram-watchdog, PR #1145 влит, потом re-коммиты.)
# ===========================================================================
test_C_scan_all_prs_stale_retro_branch() {
    new_test
    local head="z-devops/t_5af222ea-telegram-watchdog"
    # Нет ISSUE_LIST — merge-gate основной цикл ничего не найдёт; проверяем
    # scan-all-prs (PR_LIST_ALL_OPEN_JSON).
    set_state ISSUE_LIST_JSON '[]'
    set_state PR_LIST_ALL_OPEN_JSON "[{\"number\":3212,\"headRefName\":\"${head}\",\"title\":\"fix: regression\"}]"
    set_state "PR_MERGED_HEAD_${head}_JSON" '[{"number":1145,"state":"MERGED"}]'
    set_state RATE_LIMIT_JSON '{"resources":{"core":{"remaining":5000}}}'
    set_state "BRANCH_PRESENT_${head}" 1

    run_merge_gate

    local journal
    journal="$(cat "$GH_JOURNAL")"

    # gh pr comment 3212 с stale-branch текстом.
    local pr_comments
    pr_comments="$(printf '%s\n' "$journal" | grep -c 'gh pr comment 3212 .*stale-branch re-commit detected' || true)"
    assert_eq "1" "$pr_comments" "scan-all-prs: stale retro branch → PR comment posted"
}

# ===========================================================================
# D. Дедупликация: 2 тика с одним stale PR → коммент постится один раз за 24h.
# ===========================================================================
test_D_stale_branch_comment_dedup() {
    new_test
    local branch
    branch="$(slugify_branch 3205 'fix #3205 stale dedup demo')"
    fixture_stale_pr 3205 3206 "$branch" 3198 'fix #3205 stale dedup demo'

    run_merge_gate
    local first_journal
    first_journal="$(cat "$GH_JOURNAL")"

    # Второй тик: в ISSUE_3205_COMMENTS_SINCE_JSON уже есть наш коммент
    # (мок возвращает его для dedup-запроса).
    set_state "ISSUE_3205_COMMENTS_SINCE_JSON" '[{"body":"🛑 **stale-branch re-commit detected** (merge-gate, ретро 12.08 t_d3aeaa9b)"}]'
    run_merge_gate
    local second_journal
    second_journal="$(cat "$GH_JOURNAL")"

    local first_comments second_comments
    first_comments="$(printf '%s\n' "$first_journal" | grep -c 'stale-branch re-commit detected' || true)"
    second_comments="$(printf '%s\n' "$second_journal" | grep -c 'stale-branch re-commit detected' || true)"
    assert_eq "1" "$first_comments" "tick 1: stale-branch comment posted once"
    assert_eq "1" "$second_comments" "tick 2: stale-branch comment NOT re-posted (dedup)"
}

# ===========================================================================
# Run all tests.
# ===========================================================================
run_test "A. stale branch → needs-e2e blocked" test_A_stale_branch_blocks_needs_e2e
run_test "B. fresh branch → needs-e2e set" test_B_fresh_branch_sets_needs_e2e
run_test "C. scan-all-prs stale retro branch" test_C_scan_all_prs_stale_retro_branch
run_test "D. stale-branch comment dedup" test_D_stale_branch_comment_dedup

summary
