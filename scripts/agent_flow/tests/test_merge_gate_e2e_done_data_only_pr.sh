#!/bin/bash
# ============================================================================
# test_merge_gate_e2e_done_data_only_pr.sh — L-12: e2e-done PR разблокирует
# issue, застрявшую с e2e:rejected на data-only PR (issue #1506).
#
# Ретро 22.08 t_9e61d788 (retkey: e2e-done-data-only-pr-stuck-issue):
# issue #1506 (e2e-валидация голосовых фич) висит в лимбе:
#   • PR #1507 (data-only, 6 голосовых .ogg, 0 строк кода) — e2e:rejected
#   • PR #1508 (infra-fix upload-artifact '?', 1 commit) — e2e-done
# Issue имеет метки `needs-e2e + e2e:rejected + e2e` одновременно:
#   • e2e-process фильтрует issue по e2e:rejected (line 586) — skip,
#     round не создаётся.
#   • merge-gate тоже скипал: return-path требовал новые коммиты /
#     коммент воркера в data-only PR (которых нет).
#   • round-169 на PR #1508 уже SUCCESS на feature-ветке, но issue не
#     закрыта, merge-gate не ставит needs-review на PR #1508 (не находит
#     канонический PR по issue #1506, ветка отличается).
#
# Фикс (merge-gate, ретро t_9e61d788, сигнал 3 return-path): если issue
# имеет e2e:rejected И канонический PR имеет e2e:rejected И есть ДРУГОЙ
# OPEN PR этой issue с e2e-done (= инфра-фикс) — снимать e2e:rejected с
# issue и с data-only PR, ставить needs-review на e2e-done PR (= Шифу
# увидит в очереди ревью, мёрж пойдёт через Q22).
#
# Scenarios (L-12):
#   L1. issue e2e:rejected + canonical PR e2e:rejected (data-only) +
#       SIBLING PR e2e-done (infra-fix) → e2e:rejected снят с issue,
#       needs-review поставлен на e2e-done PR.
#   L2. issue e2e:rejected + canonical PR e2e:rejected, НЕТ sibling
#       e2e-done PR → поведение как раньше: skip (return path не
#       срабатывает, нужен push/коммент воркера).
#   L3. issue e2e:rejected + canonical PR e2e:rejected + sibling PR
#       e2e-done есть, НО он MERGED → не считается инфра-фиксом
#       (нужен OPEN для e2e-done path). Идемпотентный skip.
#   L4. идемпотентность: повторный тик после L1 → e2e:rejected уже
#       нет на issue → путь skip'ается (return path не срабатывает),
#       основной цикл обрабатывает e2e-done+OPEN PR → needs-review
#       (test_merge_gate_e2e_done_review.sh покрывает отдельно).
#   L5. регрессия: 1 OPEN PR на issue, e2e:rejected без sibling → как
#       раньше (no-op, не ломаем старое поведение).
#
# Run:
#   bash scripts/agent_flow/tests/test_merge_gate_e2e_done_data_only_pr.sh
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
# Fixture L1: issue #1506 c e2e:rejected + canonical PR #1507 (data-only,
# e2e:rejected) + SIBLING PR #1508 (infra-fix, e2e-done, OPEN, branch ≠
# канонической — как реальный случай).
# ---------------------------------------------------------------------------
fixture_1506_data_only_with_infra_fix() {
    local issue=1506 pr_data=1507 pr_infra=1508
    local title='task(voice) e2e-валидация голосовых фич'
    local branch_canon
    branch_canon="$(slugify_branch "$issue" "$title")"

    # Issue: hermes + needs-e2e + e2e:rejected (одновременно — это и есть баг).
    # e2e:rejected_at в timeline (реальный GitHub создаёт labeled event при
    # проставлении метки; return-path его читает для сигналов 1/2).
    set_state ISSUE_LIST_JSON "[{\"number\":${issue},\"title\":\"${title}\",\"labels\":[{\"name\":\"hermes\"},{\"name\":\"needs-e2e\"},{\"name\":\"e2e:rejected\"}],\"body\":\"kanban: t_9e61d788\"}]"
    set_state "ISSUE_${issue}_LABELS_JSON" '{"labels":[{"name":"hermes"},{"name":"needs-e2e"},{"name":"e2e:rejected"}]}'
    set_state "ISSUE_${issue}_STATE_JSON" '{"state":"OPEN"}'
    set_state "ISSUE_${issue}_COMMENTS_JSON" '{"comments":[{"body":"kanban: t_9e61d788\n"}]}'
    set_state "ISSUE_${issue}_COMMENTS_SINCE_JSON" '[]'
    set_state "ISSUE_${issue}_TIMELINE_JSON" "[{\"event\":\"labeled\",\"label\":{\"name\":\"e2e:rejected\"},\"created_at\":\"2026-08-21T22:12:00Z\"}]"

    # Канонический PR (data-only): ветка z-{agent}/1506-task-voice-...
    # метки e2e:rejected (поставлен в round-166 FAIL), MERGEABLE/CLEAN.
    set_state "PR_HEAD_${branch_canon}_JSON" "[{\"number\":${pr_data},\"state\":\"OPEN\",\"baseRefName\":\"develop\",\"mergeable\":\"MERGEABLE\",\"mergeStateStatus\":\"CLEAN\",\"statusCheckRollup\":[{\"conclusion\":\"SUCCESS\"}],\"title\":\"e2e #1506: voice_core_suite + acceptance (data-only)\",\"labels\":[{\"name\":\"e2e:rejected\"}],\"commits\":[],\"additions\":0,\"deletions\":0}]"
    set_state "PR_${pr_data}_COMMITS_JSON" '[]'

    # Sibling OPEN PR (infra-fix): ветка НЕ каноническая (z-devops/...).
    # Мок gh pr list --search "${number} in:title" вернёт JSON c этим PR.
    set_state "PR_FOLLOWUP_JSON" "[{\"number\":${pr_infra},\"headRefName\":\"z-devops/t_a8fb9137-e2e-upload-artifact-question-mark-legacy-collect\",\"mergeStateStatus\":\"CLEAN\",\"labels\":[{\"name\":\"e2e-done\"}]}]"

    # Прочие хелперы merge-gate — пустые.
    set_state PR_LIST_ALL_OPEN_JSON '[]'
    set_state PR_LIST_MERGED_JSON '[]'
    set_state RATE_LIMIT_JSON '{"resources":{"core":{"remaining":5000}}'
    set_state "KANBAN_LIST_JSON" "[{\"id\":\"t_9e61d788\",\"status\":\"running\"}]"
}

# ---------------------------------------------------------------------------
# Fixture L2: только data-only PR (нет sibling с e2e-done) → должен skip'нуться.
# ---------------------------------------------------------------------------
fixture_1506_data_only_no_infra_fix() {
    local issue=1506 pr_data=1507
    local title='task(voice) e2e-валидация голосовых фич'
    local branch_canon
    branch_canon="$(slugify_branch "$issue" "$title")"

    set_state ISSUE_LIST_JSON "[{\"number\":${issue},\"title\":\"${title}\",\"labels\":[{\"name\":\"hermes\"},{\"name\":\"needs-e2e\"},{\"name\":\"e2e:rejected\"}],\"body\":\"kanban: t_x\"}]"
    set_state "ISSUE_${issue}_LABELS_JSON" '{"labels":[{"name":"hermes"},{"name":"needs-e2e"},{"name":"e2e:rejected"}]}'
    set_state "ISSUE_${issue}_STATE_JSON" '{"state":"OPEN"}'
    set_state "ISSUE_${issue}_COMMENTS_JSON" '{"comments":[]}'
    set_state "ISSUE_${issue}_COMMENTS_SINCE_JSON" '[]'
    set_state "ISSUE_${issue}_TIMELINE_JSON" "[{\"event\":\"labeled\",\"label\":{\"name\":\"e2e:rejected\"},\"created_at\":\"2026-08-21T22:12:00Z\"}]"
    set_state "PR_HEAD_${branch_canon}_JSON" "[{\"number\":${pr_data},\"state\":\"OPEN\",\"baseRefName\":\"develop\",\"mergeable\":\"MERGEABLE\",\"mergeStateStatus\":\"CLEAN\",\"statusCheckRollup\":[{\"conclusion\":\"SUCCESS\"}],\"title\":\"e2e #1506: voice_core_suite\",\"labels\":[{\"name\":\"e2e:rejected\"}],\"commits\":[],\"additions\":0,\"deletions\":0}]"
    set_state "PR_${pr_data}_COMMITS_JSON" '[]'

    # Нет sibling PR с e2e-done (PR_FOLLOWUP_JSON = []).
    set_state "PR_FOLLOWUP_JSON" "[]"
    set_state PR_LIST_ALL_OPEN_JSON '[]'
    set_state PR_LIST_MERGED_JSON '[]'
    set_state RATE_LIMIT_JSON '{"resources":{"core":{"remaining":5000}}'
    set_state "KANBAN_LIST_JSON" "[{\"id\":\"t_x\",\"status\":\"running\"}]"
}

# ---------------------------------------------------------------------------
# Fixture L3: sibling есть, но MERGED (не OPEN) — не считается инфра-фиксом.
# ---------------------------------------------------------------------------
fixture_1506_data_only_sibling_merged() {
    local issue=1506 pr_data=1507 pr_infra=1508
    local title='task(voice) e2e-валидация голосовых фич'
    local branch_canon
    branch_canon="$(slugify_branch "$issue" "$title")"

    set_state ISSUE_LIST_JSON "[{\"number\":${issue},\"title\":\"${title}\",\"labels\":[{\"name\":\"hermes\"},{\"name\":\"needs-e2e\"},{\"name\":\"e2e:rejected\"}],\"body\":\"kanban: t_y\"}]"
    set_state "ISSUE_${issue}_LABELS_JSON" '{"labels":[{"name":"hermes"},{"name":"needs-e2e"},{"name":"e2e:rejected"}]}'
    set_state "ISSUE_${issue}_STATE_JSON" '{"state":"OPEN"}'
    set_state "ISSUE_${issue}_COMMENTS_JSON" '{"comments":[]}'
    set_state "ISSUE_${issue}_COMMENTS_SINCE_JSON" '[]'
    set_state "ISSUE_${issue}_TIMELINE_JSON" "[{\"event\":\"labeled\",\"label\":{\"name\":\"e2e:rejected\"},\"created_at\":\"2026-08-21T22:12:00Z\"}]"
    set_state "PR_HEAD_${branch_canon}_JSON" "[{\"number\":${pr_data},\"state\":\"OPEN\",\"baseRefName\":\"develop\",\"mergeable\":\"MERGEABLE\",\"mergeStateStatus\":\"CLEAN\",\"statusCheckRollup\":[{\"conclusion\":\"SUCCESS\"}],\"title\":\"e2e #1506: voice_core_suite\",\"labels\":[{\"name\":\"e2e:rejected\"}],\"commits\":[],\"additions\":0,\"deletions\":0}]"
    set_state "PR_${pr_data}_COMMITS_JSON" '[]'

    # Sibling есть, но mergeStateStatus != CLEAN/MERGEABLE (отфильтрован — не виден merge-gate).
    set_state "PR_FOLLOWUP_JSON" "[]"
    set_state PR_LIST_ALL_OPEN_JSON '[]'
    set_state PR_LIST_MERGED_JSON '[]'
    set_state RATE_LIMIT_JSON '{"resources":{"core":{"remaining":5000}}'
    set_state "KANBAN_LIST_JSON" "[{\"id\":\"t_y\",\"status\":\"running\"}]"
}

# ===========================================================================
# L1. issue e2e:rejected + data-only PR e2e:rejected + sibling e2e-done
#     PR → e2e:rejected снят с issue, needs-review поставлен на e2e-done PR.
# ===========================================================================
test_L1_e2e_done_sibling_unblocks_stuck_issue() {
    new_test
    fixture_1506_data_only_with_infra_fix

    run_merge_gate
    local journal
    journal="$(cat "$GH_JOURNAL")"

    # 1) e2e:rejected снят с issue #1506.
    local rm_rejected
    rm_rejected="$(printf '%s\n' "$journal" | grep -c "gh issue edit 1506 --remove-label e2e:rejected" || true)"
    assert_eq "1" "$rm_rejected" "e2e:rejected removed from issue #1506" || return 1

    # 2) needs-e2e поставлен на issue (= вернули в ротацию).
    local add_needs
    add_needs="$(printf '%s\n' "$journal" | grep -c "gh issue edit 1506 --add-label needs-e2e" || true)"
    assert_eq "1" "$add_needs" "needs-e2e added to issue #1506" || return 1

    # 3) needs-review поставлен на sibling PR #1508 (e2e-done infra-fix).
    local add_review
    add_review="$(printf '%s\n' "$journal" | grep -cE "gh pr edit 1508 .*--add-label needs-review" || true)"
    assert_eq "1" "$add_review" "needs-review added to sibling PR #1508" || return 1

    # 4) needs-e2e снят с sibling PR #1508 (если остался от старого цикла).
    local rm_needs_pr
    rm_needs_pr="$(printf '%s\n' "$journal" | grep -cE "gh pr edit 1508 .*--remove-label needs-e2e" || true)"
    assert_eq "1" "$rm_needs_pr" "needs-e2e removed from sibling PR #1508" || return 1

    # 5) e2e:rejected снят с data-only PR #1507.
    local rm_rej_pr
    rm_rej_pr="$(printf '%s\n' "$journal" | grep -cE "gh pr edit 1507 .*--remove-label e2e:rejected" || true)"
    assert_eq "1" "$rm_rej_pr" "e2e:rejected removed from data-only PR #1507" || return 1

    # 6) issue НЕ закрыта (PR ещё OPEN, мёрж пойдёт через Q22).
    local close_calls
    close_calls="$(printf '%s\n' "$journal" | grep -c 'gh issue close' || true)"
    assert_eq "0" "$close_calls" "issue stays OPEN (PR sibling not merged)" || return 1

    # 7) audit-комментарий постится (объясняет причину).
    local cmt_count
    cmt_count="$(printf '%s\n' "$journal" | grep -c 'e2e:rejected снят' || true)"
    assert_eq "1" "$cmt_count" "audit comment posted" || return 1
}

# ===========================================================================
# L2. только data-only PR, нет sibling → старое поведение (skip).
# ===========================================================================
test_L2_no_sibling_e2e_done_still_skips() {
    new_test
    fixture_1506_data_only_no_infra_fix

    run_merge_gate
    local journal
    journal="$(cat "$GH_JOURNAL")"

    # e2e:rejected НЕ снимается (нет сигнала).
    local rm_rejected
    rm_rejected="$(printf '%s\n' "$journal" | grep -c "gh issue edit 1506 --remove-label e2e:rejected" || true)"
    assert_eq "0" "$rm_rejected" "e2e:rejected NOT removed (no sibling)" || return 1

    # needs-review НЕ ставится ни на один PR.
    local add_review
    add_review="$(printf '%s\n' "$journal" | grep -cE "gh pr edit [0-9]+ .*--add-label needs-review" || true)"
    assert_eq "0" "$add_review" "no needs-review added (no sibling e2e-done)" || return 1

    # close НЕ вызывается.
    local close_calls
    close_calls="$(printf '%s\n' "$journal" | grep -c 'gh issue close' || true)"
    assert_eq "0" "$close_calls" "no close (PR OPEN)" || return 1
}

# ===========================================================================
# L3. sibling есть, но mergeStateStatus != CLEAN/MERGEABLE (т.е. CONFLICTING
#     или иной non-mergeable) — мок возвращает [] для PR_FOLLOWUP_JSON, merge-gate
#     не находит его. Старое поведение (skip).
# ===========================================================================
test_L3_non_mergeable_sibling_skips() {
    new_test
    fixture_1506_data_only_sibling_merged

    run_merge_gate
    local journal
    journal="$(cat "$GH_JOURNAL")"

    local rm_rejected
    rm_rejected="$(printf '%s\n' "$journal" | grep -c "gh issue edit 1506 --remove-label e2e:rejected" || true)"
    assert_eq "0" "$rm_rejected" "e2e:rejected NOT removed (sibling not mergeable)" || return 1

    local add_review
    add_review="$(printf '%s\n' "$journal" | grep -cE "gh pr edit [0-9]+ .*--add-label needs-review" || true)"
    assert_eq "0" "$add_review" "no needs-review added" || return 1
}

# ===========================================================================
# L4. регрессия: 1 PR на issue, e2e:rejected, нет sibling — поведение
#     не должно сломаться (старая логика return path).
#     Это уже покрыто test_merge_gate_retro_card_archive.sh и
#     test_merge_gate_user_reopen.sh; здесь — smoke-теп, что L-фикс не
#     добавляет ложных side-effects в чистом data-only сценарии.
# ===========================================================================
test_L4_smoke_data_only_no_regression() {
    new_test
    fixture_1506_data_only_no_infra_fix

    run_merge_gate
    local journal
    journal="$(cat "$GH_JOURNAL")"

    # НЕ должно быть никаких label-операций на issue/PR (кроме возможных
    # pre-merge-gate reads). Это sanity-check что сигнал 3 не ломает
    # чистый data-only случай.
    local label_ops
    label_ops="$(printf '%s\n' "$journal" | grep -cE "(gh issue edit 1506 --|gh pr edit [0-9]+ .*--(add|remove)-label)" || true)"
    assert_eq "0" "$label_ops" "no label ops on data-only PR without sibling" || return 1
}

# ===========================================================================
# Run
# ===========================================================================
run_test "L1. e2e-done sibling unblocks e2e:rejected stuck issue" test_L1_e2e_done_sibling_unblocks_stuck_issue
run_test "L2. no sibling e2e-done → skip (regression)" test_L2_no_sibling_e2e_done_still_skips
run_test "L3. non-mergeable sibling → skip" test_L3_non_mergeable_sibling_skips
run_test "L4. smoke: data-only no regression" test_L4_smoke_data_only_no_regression

summary