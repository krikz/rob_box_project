#!/bin/bash
# ============================================================================
# test_merge_gate_timeline_pagination.sh — issue #1977 silent-loop
#
# Bug (ретро 09.09 t_5948c129, retro t_8fba04b9 + t_657c11ba): issue #1977
# зависает в silent-loop: merge-gate 50+ тиков не закрывает issue, хотя
# PR #1979 MERGED в develop + `e2e-done` label стоит на issue. Root cause:
# `_timeline_last_labeled_at` (merge-gate.sh:2277) запрашивал
# `timeline?per_page=100` БЕЗ пагинации. Issue #1977 имеет 457
# комментариев → событие `e2e-done` лежит на странице 3 (events 201-300).
# Merge-gate не видел e2e-done → `_e2e_done_at=""` → conservative guard
# (ADR-0014 §4 req 4) подавлял auto-close → return в needs-e2e-rotation
# → на следующем тике то же самое → infinite silent skip loop.
#
# Fix (merge-gate.sh:3563-3615):
#   (a) `_timeline_last_labeled_at` теперь пагинирует до 3 страниц
#       (300 events — практически все issues в этом окне).
#   (b) В close-path: для определения наличия e2e-done теперь
#       доверяем labels.csv (`_has_e2e_done`), а НЕ timeline API.
#       Timeline нужен только для даты user-reopen (для сравнения
#       «reopen ПОСЛЕ e2e-done»).
#
# Acceptance (этот файл):
#   T1. MERGED + e2e-done в labels + OPEN + e2e-done на page 3 timeline
#       (paginated-out) → close вызывается, labels.csv-trust wins.
#       (PASS-proven regression для issue #1977.)
#   T2. MERGED + e2e-done в labels + OPEN + user-reopen ДО e2e-done
#       (timeline содержит оба события) → close вызывается (свежий
#       e2e-done выигрывает). (Regression user-reopen guard.)
#   T3. MERGED + e2e-done в labels + OPEN + user-reopen ПОСЛЕ e2e-done
#       (timeline page 1 содержит оба) → close подавлен, e2e-done
#       снят, needs-e2e возвращён. (Issue #1391 regression.)
#   T4. MERGED + e2e-done на page 3 + user-reopen на page 1 (редкий
#       случай: много комментариев между reopen и e2e-done) → close
#       подавлен (e2e-done должен быть найден через pagination).
#   T5. MERGED + e2e-done на page 3 + user-reopen ДО e2e-done
#       (страница 3 содержит оба, page 1 — ничего) → close вызывается.
# ============================================================================
set -euo pipefail

TEST_LIB_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/mock_env.sh
. "$TEST_LIB_DIR/lib/mock_env.sh"

slugify_branch() {  # $1=issue $2=title
    local issue="$1" title="$2" slug
    slug="$(printf '%s' "$title" | tr '[:upper:]' '[:lower:]' \
        | sed -E 's/[^a-z0-9]+/-/g; s/^-+|-+$//g; s/-{2,}/-/g' \
        | cut -c1-50)"
    printf 'z-{agent}/%s-%s' "$issue" "$slug"
}

# Build a timeline JSON array of N generic events (so that target events
# land on the desired page of per_page=100 pagination).
# Args: $1=count $2=event_type $3=label_name (optional) $4=created_at (optional)
_make_timeline_events() {
    local count="$1" event_type="$2" label_name="${3:-}" created_at="${4:-}"
    local i label_obj=""
    if [ -n "$label_name" ]; then
        label_obj=",\"label\":{\"name\":\"${label_name}\"}"
    fi
    printf '['
    for ((i = 0; i < count; i++)); do
        if [ "$i" -gt 0 ]; then printf ','; fi
        if [ -n "$created_at" ] && [ "$i" = "$((count-1))" ]; then
            printf '{"event":"%s"%s,"created_at":"%s"}' "$event_type" "$label_obj" "$created_at"
        else
            printf '{"event":"%s"%s,"created_at":"2026-09-0%dT0%d:00:00Z"}' \
                "$event_type" "$label_obj" "$((i % 9 + 1))" "$((i % 24))"
        fi
    done
    printf ']'
}

# ============================================================================
# T1. Page-3 e2e-done + labels.csv has e2e-done + OPEN → close fires
# (issue #1977 regression — silent-loop разрыв)
# ============================================================================
test_T1_page3_e2e_done_close_fires() {
    new_test
    local issue=1977 branch pr=1979
    branch="$(slugify_branch "$issue" 'silent loop pagination exhaust')"
    # labels.csv содержит e2e-done (PASS-proven через e2e-process)
    set_state ISSUE_LIST_JSON "[{\"number\":${issue},\"title\":\"silent loop pagination exhaust\",\"labels\":[{\"name\":\"hermes\"},{\"name\":\"e2e-done\"}],\"body\":\"kanban: t_dead${issue}\"}]"
    set_state "ISSUE_${issue}_LABELS_JSON" '{"labels":[{"name":"hermes"},{"name":"e2e-done"}]}'
    set_state "ISSUE_${issue}_STATE_JSON" '{"state":"OPEN"}'
    set_state "ISSUE_${issue}_COMMENTS_JSON" "{\"comments\":[{\"body\":\"kanban: t_dead${issue}\\n\"}]}"
    set_state "ISSUE_${issue}_COMMENTS_SINCE_JSON" '[]'
    # Timeline page 1 = 100 обычных событий (комментарии "CI красный"),
    # page 2 = ещё 100, page 3 = e2e-done на event 250.
    set_state "ISSUE_${issue}_TIMELINE_JSON_PAGE1" "$(_make_timeline_events 100 commented)"
    set_state "ISSUE_${issue}_TIMELINE_JSON_PAGE2" "$(_make_timeline_events 100 commented)"
    set_state "ISSUE_${issue}_TIMELINE_JSON_PAGE3" "[{\"event\":\"labeled\",\"label\":{\"name\":\"e2e-done\"},\"created_at\":\"2026-09-07T03:42:10Z\"}]"
    # Default page (page 1) для обратной совместимости с моком без pagination
    set_state "ISSUE_${issue}_TIMELINE_JSON" "$(cat <<EOF
$(_make_timeline_events 99 commented),
{"event":"labeled","label":{"name":"e2e-done"},"created_at":"2026-09-07T03:42:10Z"}
EOF
)"
    set_state "PR_HEAD_${branch}_JSON" "[{\"number\":${pr},\"state\":\"MERGED\",\"baseRefName\":\"develop\",\"mergedAt\":\"2026-09-07T11:14:33Z\",\"mergeable\":\"MERGEABLE\",\"mergeStateStatus\":\"CLEAN\",\"statusCheckRollup\":[{\"conclusion\":\"SUCCESS\"}],\"title\":\"[robot] fix #${issue}\",\"labels\":[]}]"
    set_state "PR_${pr}_COMMITS_JSON" '[]'
    set_state PR_LIST_ALL_OPEN_JSON '[]'
    set_state PR_FOLLOWUP_JSON '[]'
    set_state RATE_LIMIT_JSON '{"resources":{"core":{"remaining":5000}}}'
    set_state "BRANCH_PRESENT_${branch}" 1
    set_state "KANBAN_LIST_JSON" "[{\"id\":\"t_dead${issue}\",\"status\":\"done\"}]"

    run_merge_gate
    local journal
    journal="$(cat "$GH_JOURNAL")"

    # PASS-proven: close должен вызваться (timeline page 3 e2e-done
    # находим через pagination, labels.csv-trust — primary).
    local close_calls
    close_calls="$(printf '%s\n' "$journal" | grep -c "gh issue close ${issue} --reason completed" || true)"
    assert_eq "1" "$close_calls" "T1: page-3 e2e-done + labels.csv-trust → close fires"

    # e2e-done НЕ должен быть снят (он валидный).
    local rm_e2e
    rm_e2e="$(printf '%s\n' "$journal" | grep -c "gh issue edit ${issue} --remove-label e2e-done" || true)"
    assert_eq "0" "$rm_e2e" "T1: e2e-done NOT removed (valid)"
}

# ============================================================================
# T2. User-reopen BEFORE e2e-done, оба в timeline page 1 → close fires
# (regression: свежий e2e-done выигрывает над старым reopen)
# ============================================================================
test_T2_reopen_before_e2e_done_close_fires() {
    new_test
    local issue=2401 branch pr=2402
    branch="$(slugify_branch "$issue" 'reopen before e2e done')"
    set_state ISSUE_LIST_JSON "[{\"number\":${issue},\"title\":\"reopen before e2e done\",\"labels\":[{\"name\":\"hermes\"},{\"name\":\"e2e-done\"}],\"body\":\"kanban: t_dead${issue}\"}]"
    set_state "ISSUE_${issue}_LABELS_JSON" '{"labels":[{"name":"hermes"},{"name":"e2e-done"}]}'
    set_state "ISSUE_${issue}_STATE_JSON" '{"state":"OPEN"}'
    set_state "ISSUE_${issue}_COMMENTS_JSON" "{\"comments\":[{\"body\":\"kanban: t_dead${issue}\\n\"}]}"
    set_state "ISSUE_${issue}_COMMENTS_SINCE_JSON" '[]'
    set_state "ISSUE_${issue}_TIMELINE_JSON" "[{\"event\":\"reopened\",\"actor\":{\"login\":\"krikz\"},\"created_at\":\"2026-08-18T08:00:00Z\",\"state_reason\":\"reopened\"},{\"event\":\"labeled\",\"label\":{\"name\":\"e2e-done\"},\"created_at\":\"2026-08-18T11:00:00Z\"}]"
    set_state "PR_HEAD_${branch}_JSON" "[{\"number\":${pr},\"state\":\"MERGED\",\"baseRefName\":\"develop\",\"mergedAt\":\"2026-08-18T11:30:00Z\",\"mergeable\":\"MERGEABLE\",\"mergeStateStatus\":\"CLEAN\",\"statusCheckRollup\":[{\"conclusion\":\"SUCCESS\"}],\"title\":\"[robot] fix #${issue}\",\"labels\":[]}]"
    set_state "PR_${pr}_COMMITS_JSON" '[]'
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
    assert_eq "1" "$close_calls" "T2: reopen before e2e-done (timeline page 1) → close fires (fresh label wins)"
}

# ============================================================================
# T3. User-reopen AFTER e2e-done → close подавлен, e2e-done снят
# (issue #1391 regression — user-reopen guard)
# ============================================================================
test_T3_reopen_after_e2e_done_blocks_close() {
    new_test
    local issue=2403 branch pr=2404
    branch="$(slugify_branch "$issue" 'reopen after e2e done')"
    set_state ISSUE_LIST_JSON "[{\"number\":${issue},\"title\":\"reopen after e2e done\",\"labels\":[{\"name\":\"hermes\"},{\"name\":\"e2e-done\"}],\"body\":\"kanban: t_dead${issue}\"}]"
    set_state "ISSUE_${issue}_LABELS_JSON" '{"labels":[{"name":"hermes"},{"name":"e2e-done"}]}'
    set_state "ISSUE_${issue}_STATE_JSON" '{"state":"OPEN"}'
    set_state "ISSUE_${issue}_COMMENTS_JSON" "{\"comments\":[{\"body\":\"kanban: t_dead${issue}\\n\"}]}"
    set_state "ISSUE_${issue}_COMMENTS_SINCE_JSON" '[]'
    # Timeline: e2e-done at T0, user-reopen at T1 (AFTER). Issue #1363 pattern.
    set_state "ISSUE_${issue}_TIMELINE_JSON" "[{\"event\":\"labeled\",\"label\":{\"name\":\"e2e-done\"},\"created_at\":\"2026-08-18T09:53:29Z\"},{\"event\":\"closed\",\"actor\":{\"login\":\"krikz\"},\"created_at\":\"2026-08-18T12:10:23Z\",\"state_reason\":\"completed\"},{\"event\":\"reopened\",\"actor\":{\"login\":\"krikz\"},\"created_at\":\"2026-08-18T14:14:37Z\",\"state_reason\":\"reopened\"}]"
    set_state "PR_HEAD_${branch}_JSON" "[{\"number\":${pr},\"state\":\"MERGED\",\"baseRefName\":\"develop\",\"mergedAt\":\"2026-08-18T11:19:44Z\",\"mergeable\":\"MERGEABLE\",\"mergeStateStatus\":\"CLEAN\",\"statusCheckRollup\":[{\"conclusion\":\"SUCCESS\"}],\"title\":\"[robot] fix #${issue}\",\"labels\":[]}]"
    set_state "PR_${pr}_COMMITS_JSON" '[]'
    set_state PR_LIST_ALL_OPEN_JSON '[]'
    set_state PR_FOLLOWUP_JSON '[]'
    set_state RATE_LIMIT_JSON '{"resources":{"core":{"remaining":5000}}}'
    set_state "BRANCH_PRESENT_${branch}" 1
    set_state "KANBAN_LIST_JSON" "[]"

    run_merge_gate
    local journal
    journal="$(cat "$GH_JOURNAL")"

    local close_calls
    close_calls="$(printf '%s\n' "$journal" | grep -c "gh issue close ${issue} --reason completed" || true)"
    assert_eq "0" "$close_calls" "T3: reopen AFTER e2e-done → NO close (issue #1391 guard)"

    # e2e-done должен быть снят (протухшая метка).
    local rm_e2e
    rm_e2e="$(printf '%s\n' "$journal" | grep -c "gh issue edit ${issue} --remove-label e2e-done" || true)"
    assert_eq "1" "$rm_e2e" "T3: stale e2e-done removed"

    # needs-e2e возвращён.
    local add_needs_e2e
    add_needs_e2e="$(printf '%s\n' "$journal" | grep -c "gh issue edit ${issue} --add-label needs-e2e" || true)"
    assert_eq "1" "$add_needs_e2e" "T3: needs-e2e re-added"
}

# ============================================================================
# T4. Page-3 e2e-done + page-1 user-reopen (много комментариев между)
# → close подавлен (pagination найдёт e2e-done на page 3, user-reopen
# на page 1 → user-reopen ДО e2e-done? Нет, e2e-done позже → guard НЕ
# срабатывает). Этот тест проверяет что pagination корректно находит
# ОБА события и сравнение даёт правильный вердикт.
# ============================================================================
test_T4_pagination_both_events_compared() {
    new_test
    local issue=2405 branch pr=2406
    branch="$(slugify_branch "$issue" 'pagination both events')"
    set_state ISSUE_LIST_JSON "[{\"number\":${issue},\"title\":\"pagination both events\",\"labels\":[{\"name\":\"hermes\"},{\"name\":\"e2e-done\"}],\"body\":\"kanban: t_dead${issue}\"}]"
    set_state "ISSUE_${issue}_LABELS_JSON" '{"labels":[{"name":"hermes"},{"name":"e2e-done"}]}'
    set_state "ISSUE_${issue}_STATE_JSON" '{"state":"OPEN"}'
    set_state "ISSUE_${issue}_COMMENTS_JSON" "{\"comments\":[{\"body\":\"kanban: t_dead${issue}\\n\"}]}"
    set_state "ISSUE_${issue}_COMMENTS_SINCE_JSON" '[]'
    # Timeline page 1 = 99 комментариев + 1 reopened (T0=08:00)
    # Timeline page 2 = 100 комментариев (без меток)
    # Timeline page 3 = 100 комментариев + e2e-done на event 250 (T1=11:00)
    # user-reopen (T0=08:00) < e2e-done (T1=11:00) → fresh e2e-done
    # выигрывает → close fires.
    set_state "ISSUE_${issue}_TIMELINE_JSON_PAGE1" "$(_make_timeline_events 99 commented),{\"event\":\"reopened\",\"actor\":{\"login\":\"krikz\"},\"created_at\":\"2026-08-18T08:00:00Z\",\"state_reason\":\"reopened\"}"
    set_state "ISSUE_${issue}_TIMELINE_JSON_PAGE2" "$(_make_timeline_events 100 commented)"
    set_state "ISSUE_${issue}_TIMELINE_JSON_PAGE3" "$(_make_timeline_events 99 commented),{\"event\":\"labeled\",\"label\":{\"name\":\"e2e-done\"},\"created_at\":\"2026-08-18T11:00:00Z\"}"
    # Default timeline = combined (для обратной совместимости с mock_env)
    set_state "ISSUE_${issue}_TIMELINE_JSON" "[{\"event\":\"reopened\",\"actor\":{\"login\":\"krikz\"},\"created_at\":\"2026-08-18T08:00:00Z\",\"state_reason\":\"reopened\"},{\"event\":\"labeled\",\"label\":{\"name\":\"e2e-done\"},\"created_at\":\"2026-08-18T11:00:00Z\"}]"
    set_state "PR_HEAD_${branch}_JSON" "[{\"number\":${pr},\"state\":\"MERGED\",\"baseRefName\":\"develop\",\"mergedAt\":\"2026-08-18T11:30:00Z\",\"mergeable\":\"MERGEABLE\",\"mergeStateStatus\":\"CLEAN\",\"statusCheckRollup\":[{\"conclusion\":\"SUCCESS\"}],\"title\":\"[robot] fix #${issue}\",\"labels\":[]}]"
    set_state "PR_${pr}_COMMITS_JSON" '[]'
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
    assert_eq "1" "$close_calls" "T4: pagination finds both events, fresh e2e-done wins → close fires"
}

# ============================================================================
# T5. Page-3 e2e-done + page-3 user-reopen (user-reopen ДО e2e-done,
# оба на одной странице после 200 комментариев) → close fires.
# Это покрывает edge-case: много комментариев, но события оба на page 3.
# ============================================================================
test_T5_pagination_same_page_both_events() {
    new_test
    local issue=2407 branch pr=2408
    branch="$(slugify_branch "$issue" 'pagination same page both')"
    set_state ISSUE_LIST_JSON "[{\"number\":${issue},\"title\":\"pagination same page both\",\"labels\":[{\"name\":\"hermes\"},{\"name\":\"e2e-done\"}],\"body\":\"kanban: t_dead${issue}\"}]"
    set_state "ISSUE_${issue}_LABELS_JSON" '{"labels":[{"name":"hermes"},{"name":"e2e-done"}]}'
    set_state "ISSUE_${issue}_STATE_JSON" '{"state":"OPEN"}'
    set_state "ISSUE_${issue}_COMMENTS_JSON" "{\"comments\":[{\"body\":\"kanban: t_dead${issue}\\n\"}]}"
    set_state "ISSUE_${issue}_COMMENTS_SINCE_JSON" '[]'
    # Page 1 = 100 комментариев, page 2 = 100 комментариев,
    # page 3 = 98 комментариев + reopened (08:00) + e2e-done (11:00)
    set_state "ISSUE_${issue}_TIMELINE_JSON_PAGE1" "$(_make_timeline_events 100 commented)"
    set_state "ISSUE_${issue}_TIMELINE_JSON_PAGE2" "$(_make_timeline_events 100 commented)"
    set_state "ISSUE_${issue}_TIMELINE_JSON_PAGE3" "$(_make_timeline_events 98 commented),{\"event\":\"reopened\",\"actor\":{\"login\":\"krikz\"},\"created_at\":\"2026-08-18T08:00:00Z\",\"state_reason\":\"reopened\"},{\"event\":\"labeled\",\"label\":{\"name\":\"e2e-done\"},\"created_at\":\"2026-08-18T11:00:00Z\"}"
    # Default timeline = simplified
    set_state "ISSUE_${issue}_TIMELINE_JSON" "[{\"event\":\"reopened\",\"actor\":{\"login\":\"krikz\"},\"created_at\":\"2026-08-18T08:00:00Z\",\"state_reason\":\"reopened\"},{\"event\":\"labeled\",\"label\":{\"name\":\"e2e-done\"},\"created_at\":\"2026-08-18T11:00:00Z\"}]"
    set_state "PR_HEAD_${branch}_JSON" "[{\"number\":${pr},\"state\":\"MERGED\",\"baseRefName\":\"develop\",\"mergedAt\":\"2026-08-18T11:30:00Z\",\"mergeable\":\"MERGEABLE\",\"mergeStateStatus\":\"CLEAN\",\"statusCheckRollup\":[{\"conclusion\":\"SUCCESS\"}],\"title\":\"[robot] fix #${issue}\",\"labels\":[]}]"
    set_state "PR_${pr}_COMMITS_JSON" '[]'
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
    assert_eq "1" "$close_calls" "T5: pagination same page (reopened before e2e-done) → close fires"
}

run_test "T1_page3_e2e_done_close_fires"     test_T1_page3_e2e_done_close_fires
run_test "T2_reopen_before_e2e_done"        test_T2_reopen_before_e2e_done_close_fires
run_test "T3_reopen_after_e2e_done_blocks"  test_T3_reopen_after_e2e_done_blocks_close
run_test "T4_pagination_both_events"        test_T4_pagination_both_events_compared
run_test "T5_pagination_same_page"          test_T5_pagination_same_page_both_events
