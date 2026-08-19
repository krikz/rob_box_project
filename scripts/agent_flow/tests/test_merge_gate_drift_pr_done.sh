#!/bin/bash
# ============================================================================
# test_merge_gate_drift_pr_done.sh — PR↔issue e2e-done drift reconcile
#
# Ретро 19.08 t_5cde0bc1 (PR #1398 / issue #1392): issue имеет needs-e2e (в
# ротации), но канонический PR висит с e2e-done от предыдущего раунда. Без
# reconcile e2e-done «прилипает» к PR между ручным возвратом krikz в needs-e2e
# и следующим e2e-раундом → drift в PR-очереди Шифу (видит «готово к ревью»
# хотя по issue ничего не сделано).
#
# Фикс (merge-gate, новый блок после follow-up-PR): если issue:needs-e2e и
# канонический PR имеет e2e-done + в timeline issue последнее e2e-done
# событие — unlabel → снимаем PR-side stale метки (e2e-done + needs-review).
#
# Scenarios:
#   D1. issue:needs-e2e + PR:e2e-done + last issue-e2e-done evt=unlabeled →
#       снимаем PR:e2e-done (и PR:needs-review если есть), комменты в оба
#       места, метрика e2e_drift_minutes в логе.
#   D2. issue:needs-e2e + PR:e2e-done + last issue-e2e-done evt=labeled
#       (e2e-done свежее, e2e-process сам разрулит) → НЕ трогаем PR.
#   D3. issue:needs-e2e + PR без e2e-done → блок не срабатывает.
#   D4. DRY_RUN=true → gh pr edit НЕ вызывается, только лог.
#   D5. issue не в needs-e2e (только e2e-done, или ничего) → блок НЕ
#       срабатывает (это нормальный reconcile в другой ветке).
#   D6. PR c e2e-done, без needs-review → снимаем только e2e-done,
#       needs-review скипаем (его и не было).
#
# Run:
#   bash scripts/agent_flow/tests/test_merge_gate_drift_pr_done.sh
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
# Common fixture: issue с needs-e2e (OPEN), canonical PR с метками.
#   $1=issue $2=pr $3=title $4=issue_labels_csv $5=pr_labels_csv
#   $6=timeline_json
# ---------------------------------------------------------------------------
fixture_drift() {
    local issue="$1" pr="$2" title="$3" issue_labels_csv="$4" pr_labels_csv="$5" timeline_json="$6"
    local branch
    branch="$(slugify_branch "$issue" "$title")"

    # Build JSON arrays for issue / PR labels.
    local issue_labels_json pr_labels_json
    issue_labels_json="$(printf '%s' "$issue_labels_csv" \
        | tr ',' '\n' | sed 's/.*/{"name":"&"}/' | paste -sd, -)"
    issue_labels_json="[$issue_labels_json]"
    pr_labels_json="$(printf '%s' "$pr_labels_csv" \
        | tr ',' '\n' | sed 's/.*/{"name":"&"}/' | paste -sd, -)"
    pr_labels_json="[$pr_labels_json]"

    set_state ISSUE_LIST_JSON "[{\"number\":${issue},\"title\":\"${title}\",\"labels\":${issue_labels_json},\"body\":\"kanban: t_dead${issue}\"}]"
    set_state "ISSUE_${issue}_LABELS_JSON" "{\"labels\":${issue_labels_json}}"
    set_state "ISSUE_${issue}_STATE_JSON" '{"state":"OPEN"}'
    set_state "ISSUE_${issue}_COMMENTS_JSON" "{\"comments\":[{\"body\":\"kanban: t_dead${issue}\\n\"}]}"
    set_state "ISSUE_${issue}_COMMENTS_SINCE_JSON" '[]'
    set_state "ISSUE_${issue}_TIMELINE_JSON" "$timeline_json"

    # Канонический PR — должен попасть под --search "<n> in:title"
    # (mock-окружение: gh pr list --search идёт в PR_FOLLOWUP_JSON).
    set_state "PR_FOLLOWUP_JSON" "[{\"number\":${pr},\"headRefName\":\"${branch}\",\"mergeStateStatus\":\"CLEAN\",\"updatedAt\":\"2026-08-19T01:00:00Z\"}]"
    set_state "PR_HEAD_${branch}_JSON" "[{\"number\":${pr},\"state\":\"OPEN\",\"baseRefName\":\"develop\",\"mergeable\":\"MERGEABLE\",\"mergeStateStatus\":\"CLEAN\",\"statusCheckRollup\":[{\"conclusion\":\"SUCCESS\"}],\"title\":\"[robot] ${title}\",\"labels\":${pr_labels_json}}]"
    # gh pr view N --json labels — отдаём PR-метки
    set_state "PR_${pr}_VIEW_JSON" "{\"labels\":${pr_labels_json}}"
    # gh api pulls/N/commits — последний коммит для метрики
    set_state "PR_${pr}_COMMITS_JSON" "[{\"commit\":{\"committer\":{\"date\":\"2026-08-18T21:28:00Z\"}}}]"
    set_state PR_LIST_ALL_OPEN_JSON '[]'
    set_state RATE_LIMIT_JSON '{"resources":{"core":{"remaining":5000}}}'
    set_state "KANBAN_LIST_JSON" "[{\"id\":\"t_dead${issue}\",\"status\":\"done\"}]"
}

# ===========================================================================
# D1. issue:needs-e2e + PR:e2e-done, last issue-e2e-done evt=unlabeled →
#     снимаем PR:e2e-done (PR:needs-review тоже есть), комменты, метрика.
# ===========================================================================
test_D1_drift_unlabel_pr_done_removed() {
    new_test
    local issue=1392 pr=1398
    # issue: needs-e2e (НЕ e2e-done!) — это ключ дрифта.
    # PR: e2e-done + needs-review.
    # timeline: последнее e2e-done событие — unlabeled (krikz снял руками в 22:50).
    fixture_drift "$issue" "$pr" 'bug voice 1392 music MCP tools fix' \
        'needs-e2e,bug,music,voice,hermes,priority:critical,mcp,agent:backend' \
        'priority:high,music,voice,hermes,e2e-done,mcp,agent:backend,needs-review' \
        '[{"event":"labeled","label":{"name":"e2e-done"},"created_at":"2026-08-18T21:56:14Z"},{"event":"unlabeled","label":{"name":"e2e-done"},"created_at":"2026-08-18T22:50:11Z"}]'

    run_merge_gate
    local journal
    journal="$(cat "$GH_JOURNAL")"

    # 1) PR:e2e-done снят.
    local rm_done
    rm_done="$(printf '%s\n' "$journal" | grep -cE "gh pr edit ${pr} .*--remove-label e2e-done" || true)"
    assert_eq "1" "$rm_done" "PR:e2e-done removed" || return 1

    # 2) PR:needs-review тоже снят (он был).
    local rm_review
    rm_review="$(printf '%s\n' "$journal" | grep -cE "gh pr edit ${pr} .*--remove-label needs-review" || true)"
    assert_eq "1" "$rm_review" "PR:needs-review removed (was set)" || return 1

    # 3) Комменты — на PR и на issue.
    local pr_cmt
    pr_cmt="$(printf '%s\n' "$journal" | grep -c 'gh pr comment 1398' || true)"
    assert_contains "1" "$pr_cmt" "PR comment posted" || return 1
    local issue_cmt
    issue_cmt="$(printf '%s\n' "$journal" | grep -c "gh issue comment ${issue} --body agent-flow: 🧹 e2e-done drift reconcile" || true)"
    assert_contains "1" "$issue_cmt" "issue comment posted" || return 1

    # 4) issue НЕ закрывается (мы только снимаем PR-side метки).
    local close_calls
    close_calls="$(printf '%s\n' "$journal" | grep -c 'gh issue close' || true)"
    assert_eq "0" "$close_calls" "issue NOT closed" || return 1
}

# ===========================================================================
# D2. issue:needs-e2e + PR:e2e-done, last issue-e2e-done evt=labeled (свежий)
#     → НЕ трогаем PR (e2e-process сам разрулит).
# ===========================================================================
test_D2_drift_labeled_evt_does_not_remove() {
    new_test
    local issue=1400 pr=1401
    fixture_drift "$issue" "$pr" 'voice 1400 fresh e2e-done' \
        'needs-e2e,bug,music,voice,hermes,priority:critical,mcp,agent:backend' \
        'priority:high,music,voice,hermes,e2e-done,mcp,agent:backend,needs-review' \
        '[{"event":"labeled","label":{"name":"e2e-done"},"created_at":"2026-08-19T01:30:00Z"}]'

    run_merge_gate
    local journal
    journal="$(cat "$GH_JOURNAL")"

    # 1) PR:e2e-done НЕ снимается.
    local rm_done
    rm_done="$(printf '%s\n' "$journal" | grep -cE "gh pr edit ${pr} .*--remove-label e2e-done" || true)"
    assert_eq "0" "$rm_done" "PR:e2e-done NOT removed (last evt=labeled)" || return 1

    # 2) Лог содержит "PR↔issue e2e-done check ... НЕ снимаю" — stderr.
    local stderr_log
    stderr_log="$(cat "$TEST_TMP/stderr.log" 2>/dev/null || true)"
    assert_contains "PR↔issue e2e-done check" "$stderr_log" "guard log line present in stderr" || return 1
}

# ===========================================================================
# D3. issue:needs-e2e + PR БЕЗ e2e-done → блок не срабатывает.
# ===========================================================================
test_D3_no_pr_done_drift_block_skipped() {
    new_test
    local issue=1410 pr=1411
    fixture_drift "$issue" "$pr" 'voice 1410 fresh needs-e2e PR' \
        'needs-e2e,bug,music,voice,hermes,priority:critical,mcp,agent:backend' \
        'priority:high,music,voice,hermes,mcp,agent:backend,needs-e2e' \
        '[{"event":"labeled","label":{"name":"needs-e2e"},"created_at":"2026-08-19T01:00:00Z"}]'

    run_merge_gate
    local journal
    journal="$(cat "$GH_JOURNAL")"

    # 1) Никаких remove-label e2e-done.
    local rm_done
    rm_done="$(printf '%s\n' "$journal" | grep -cE "gh pr edit ${pr} .*--remove-label e2e-done" || true)"
    assert_eq "0" "$rm_done" "no e2e-done removal" || return 1
    # 2) Никаких PR comment drift.
    local pr_cmt
    pr_cmt="$(printf '%s\n' "$journal" | grep -c 'drift reconcile' || true)"
    assert_eq "0" "$pr_cmt" "no drift reconcile comments" || return 1
}

# ===========================================================================
# D4. DRY_RUN=true → gh pr edit не вызывается, только лог.
# ===========================================================================
test_D4_dry_run_no_pr_edit() {
    new_test
    local issue=1420 pr=1421
    fixture_drift "$issue" "$pr" 'voice 1420 drift dry-run' \
        'needs-e2e,bug,music,voice,hermes,priority:critical,mcp,agent:backend' \
        'priority:high,music,voice,hermes,e2e-done,mcp,agent:backend,needs-review' \
        '[{"event":"labeled","label":{"name":"e2e-done"},"created_at":"2026-08-18T21:56:14Z"},{"event":"unlabeled","label":{"name":"e2e-done"},"created_at":"2026-08-18T22:50:11Z"}]'

    DRY_RUN=true run_merge_gate
    local journal
    journal="$(cat "$GH_JOURNAL")"

    local rm_done
    rm_done="$(printf '%s\n' "$journal" | grep -cE "gh pr edit ${pr} .*--remove-label e2e-done" || true)"
    assert_eq "0" "$rm_done" "DRY_RUN: no pr edit" || return 1
    local rm_review
    rm_review="$(printf '%s\n' "$journal" | grep -cE "gh pr edit ${pr} .*--remove-label needs-review" || true)"
    assert_eq "0" "$rm_review" "DRY_RUN: no pr edit needs-review" || return 1
    local stderr_log
    stderr_log="$(cat "$TEST_TMP/stderr.log" 2>/dev/null || true)"
    assert_contains "PR↔issue e2e-done drift" "$stderr_log" "DRY_RUN: drift log line still emitted to stderr" || return 1
}

# ===========================================================================
# D5. issue:needs-e2e + PR:e2e-done + needs-review, БЕЗ unlabeled в timeline
#     (issue не была в ротации — обычный e2e-done reconcile путь) →
#     наш блок не сработает, но reconcile-блок (e2e-done+OPEN PR) ниже
#     подхватит и поставит needs-review (идемпотентно). Проверяем, что
#     DRIFT-блок НЕ удалил PR:e2e-done.
# ===========================================================================
test_D5_only_labeled_timeline_drift_skipped() {
    new_test
    local issue=1430 pr=1431
    fixture_drift "$issue" "$pr" 'voice 1430 only labeled' \
        'needs-e2e,bug,music,voice,hermes,priority:critical,mcp,agent:backend' \
        'priority:high,music,voice,hermes,e2e-done,mcp,agent:backend,needs-review' \
        '[{"event":"labeled","label":{"name":"e2e-done"},"created_at":"2026-08-19T00:00:00Z"}]'

    run_merge_gate
    local journal
    journal="$(cat "$GH_JOURNAL")"

    local rm_done
    rm_done="$(printf '%s\n' "$journal" | grep -cE "gh pr edit ${pr} .*--remove-label e2e-done" || true)"
    assert_eq "0" "$rm_done" "drift block NOT removing PR:e2e-done when last evt=labeled" || return 1
}

# ===========================================================================
# D6. PR:e2e-done без needs-review → снимаем только e2e-done.
# ===========================================================================
test_D6_pr_done_without_needs_review() {
    new_test
    local issue=1440 pr=1441
    fixture_drift "$issue" "$pr" 'voice 1440 done-no-review' \
        'needs-e2e,bug,music,voice,hermes,priority:critical,mcp,agent:backend' \
        'priority:high,music,voice,hermes,e2e-done,mcp,agent:backend' \
        '[{"event":"labeled","label":{"name":"e2e-done"},"created_at":"2026-08-18T21:56:14Z"},{"event":"unlabeled","label":{"name":"e2e-done"},"created_at":"2026-08-18T22:50:11Z"}]'

    run_merge_gate
    local journal
    journal="$(cat "$GH_JOURNAL")"

    # 1) PR:e2e-done снят.
    local rm_done
    rm_done="$(printf '%s\n' "$journal" | grep -cE "gh pr edit ${pr} .*--remove-label e2e-done" || true)"
    assert_eq "1" "$rm_done" "PR:e2e-done removed" || return 1

    # 2) needs-review НЕ снимается (его и не было).
    local rm_review
    rm_review="$(printf '%s\n' "$journal" | grep -cE "gh pr edit ${pr} .*--remove-label needs-review" || true)"
    assert_eq "0" "$rm_review" "needs-review NOT touched (was absent)" || return 1
}

# ===========================================================================
# Run
# ===========================================================================
run_test "D1. needs-e2e + PR:e2e-done + last evt=unlabeled → reconcile" test_D1_drift_unlabel_pr_done_removed
run_test "D2. needs-e2e + PR:e2e-done + last evt=labeled → skip (guard)" test_D2_drift_labeled_evt_does_not_remove
run_test "D3. needs-e2e + PR без e2e-done → block skipped" test_D3_no_pr_done_drift_block_skipped
run_test "D4. DRY_RUN=true → no side effects" test_D4_dry_run_no_pr_edit
run_test "D5. только labeled в timeline → drift block skip" test_D5_only_labeled_timeline_drift_skipped
run_test "D6. PR:e2e-done без needs-review → снимаем только e2e-done" test_D6_pr_done_without_needs_review

summary
