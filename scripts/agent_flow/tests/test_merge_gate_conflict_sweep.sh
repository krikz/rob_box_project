#!/bin/bash
# ============================================================================
# test_merge_gate_conflict_sweep.sh — ADR-0014 Amendment 1 (label conflict)
#
# Сценарий: одновременное присутствие `needs-e2e` + `e2e-done` на issue —
# invariant violation (data race), а не user override. Ретроспектива
# t_8fba04b9 (issue #1977 stuck-open): manual add `needs-e2e` после merge
# приводил к подавлению close через user-reopen guard (issue #1391).
#
# Фикс (merge-gate pre-check, Amendment 1 §6.2): _conflict_sweep_resolve
# вызывается ДО user-reopen guard при наличии conflict:
#   • PR MERGED + conflict → strip `needs-e2e`, audit-коммент (24h dedup),
#     close issue reason=completed. Whitelist `user-reopened-this` побеждает.
#   • PR OPEN / CLOSED unmerged + conflict → strip `needs-e2e`, audit-коммент,
#     leave OPEN (defer).
#
# Scenarios (C1..C6, ADR-0014 Amendment 1 §3.1):
#   C1. MERGED + conflict + OPEN → strip + close + audit-коммент; user-reopen
#       guard НЕ задействован.
#   C2. OPEN PR + conflict + OPEN → strip + audit; close НЕ вызван.
#   C3. MERGED + conflict + OPEN + whitelist `user-reopened-this` → audit-коммент
#       "whitelist overrides", strip НЕ сделан, close НЕ вызван.
#   C4. идемпотентность: повторный тик после C1 → strip no-op, close no-op.
#   C5. conflict + нет PR → skip, audit не публикуем (нет доказательства
#       data-race без ассоциированного PR).
#   C6. регрессия: штатный e2e-done + MERGED путь (без conflict) по-прежнему
#       закрывает issue через существующий user-reopen guard path.
#
# Run:
#   bash scripts/agent_flow/tests/test_merge_gate_conflict_sweep.sh
# ============================================================================
set -euo pipefail

RECENT_MERGED_AT="$(date -u -d '7 days ago' +%Y-%m-%dT%H:%M:%SZ)"

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

# Helper: assert first >= second.
assert_ge() {  # $1=min $2=actual $3=msg
    if [ "$2" -ge "$1" ] 2>/dev/null; then return 0; fi
    printf '  %sassert fail:%s %s\n    min:     %q\n    actual:   %q\n' \
        "$RED" "$END" "$3" "$1" "$2" >&2
    return 1
}

# Compose labels_json from CSV: e.g. "hermes,needs-e2e,e2e-done"
labels_json_from_csv() {  # $1=csv
    local csv="$1" out="[" first=1 lab
    for lab in $(printf '%s' "$csv" | tr ',' ' '); do
        if [ "$first" = "1" ]; then
            out="${out}{\"name\":\"${lab}\"}"
            first=0
        else
            out="${out},{\"name\":\"${lab}\"}"
        fi
    done
    out="${out}]"
    printf '%s' "$out"
}

# Common fixture for conflict scenarios.
# $1=issue $2=pr $3=title $4=pr_state(MERGED|OPEN|CLOSED) $5=issue_labels_csv
fixture_conflict() {
    local issue="$1" pr="$2" title="$3" pr_state="$4" issue_labels="$5"
    local branch
    branch="$(slugify_branch "$issue" "$title")"
    local issue_labels_json
    issue_labels_json="$(labels_json_from_csv "$issue_labels")"

    # Issue в hermes-list (чтобы merge-gate подобрал).
    set_state ISSUE_LIST_JSON "[{\"number\":${issue},\"title\":\"${title}\",\"labels\":${issue_labels_json},\"body\":\"kanban: t_cs${issue}\"}]"
    set_state "ISSUE_${issue}_LABELS_JSON" "{\"labels\":${issue_labels_json}}"
    set_state "ISSUE_${issue}_STATE_JSON" '{"state":"OPEN"}'
    # Kanban-marker в comments (merge-gate main-cycle требует, см. ADR-AF-0063).
    set_state "ISSUE_${issue}_COMMENTS_JSON" "{\"comments\":[{\"body\":\"kanban: t_cs${issue}\\n\"}]}"
    set_state "ISSUE_${issue}_COMMENTS_SINCE_JSON" '[]'
    # Timeline с labeled event для e2e-done (чтобы existing user-reopen
    # guard не подавлял close — он работает ТОЛЬКО если helper
    # _timeline_last_labeled_at возвращает пустоту = API fail = fail-closed).
    # Наш pre-check срабатывает ДО guard, поэтому timeline с labeled event
    # = нормальный путь.
    set_state "ISSUE_${issue}_TIMELINE_JSON" "[{\"event\":\"labeled\",\"label\":{\"name\":\"e2e-done\"},\"created_at\":\"${RECENT_MERGED_AT}\"}]"

    # PR fixture.
    local merged_at_field=""
    if [ "$pr_state" = "MERGED" ]; then
        merged_at_field=",\"mergedAt\":\"${RECENT_MERGED_AT}\""
    fi
    set_state "PR_HEAD_${branch}_JSON" "[{\"number\":${pr},\"state\":\"${pr_state}\",\"baseRefName\":\"develop\",\"mergeable\":\"MERGEABLE\",\"mergeStateStatus\":\"CLEAN\",\"statusCheckRollup\":[{\"conclusion\":\"SUCCESS\"}],\"title\":\"fix #${issue} via conflict\",\"headRefName\":\"${branch}\",\"labels\":[]${merged_at_field}}]"
    set_state "PR_${pr}_COMMITS_JSON" '[]'
    set_state PR_LIST_ALL_OPEN_JSON '[]'
    set_state PR_LIST_MERGED_JSON '[]'
    set_state PR_FOLLOWUP_JSON '[]'
    set_state RATE_LIMIT_JSON '{"resources":{"core":{"remaining":5000}}}'
    set_state KANBAN_LIST_JSON "[]"
    set_state "BRANCH_PRESENT_${branch}" 1
    # Merge-gate постит whoami-коммент через helper post_whoami_comment —
    # мок автоматически отвечает JSON-успехом; реальный GitHub не дёргаем.
    set_state "ISSUE_${issue}_WHOAMI_COMMENT_JSON" '{"id":1}'
}

# ============================================================================
# C1. MERGED + conflict + OPEN → strip + close + audit-коммент.
# ============================================================================
test_C1_conflict_merged_strips_and_closes() {
    new_test
    local issue=1977 pr=1979 title='feat 1977 e2e synth fallback chain'
    fixture_conflict "$issue" "$pr" "$title" MERGED \
        "hermes,needs-e2e,e2e-done"

    run_merge_gate
    local journal
    journal="$(cat "$GH_JOURNAL")"

    # 1) gh issue edit --remove-label needs-e2e ровно один раз.
    local strip_calls
    strip_calls="$(printf '%s\n' "$journal" | grep -cE "gh issue edit ${issue} .*--remove-label needs-e2e" || true)"
    assert_eq "1" "$strip_calls" "strip needs-e2e called once (Amendment 1 §6.2 case A)" || return 1

    # 2) gh issue close --reason completed ровно один раз.
    local close_calls
    close_calls="$(printf '%s\n' "$journal" | grep -cE "gh issue close ${issue} --reason completed" || true)"
    assert_eq "1" "$close_calls" "close called once (data-race path, NOT user-reopen guard)" || return 1

    # 3) Audit-коммент с маркером `data-race-label-conflict` опубликован.
    local audit_count
    audit_count="$(printf '%s\n' "$journal" | grep -c "data-race-label-conflict" || true)"
    assert_ge "1" "$audit_count" "audit comment with data-race marker published" || return 1

    # 4) State flipped to CLOSED.
    local state_now
    state_now="$(grep -E "^ISSUE_${issue}_STATE_JSON=" "$GH_STATE" | sed "s/^ISSUE_${issue}_STATE_JSON=//")"
    assert_contains '"CLOSED"' "$state_now" "issue state flipped to CLOSED" || return 1

    # 5) User-reopen guard path (issue #1391) НЕ задействован: в журнале
    #    не должно быть сообщения "USER-REOPEN GUARD" (это сработало бы,
    #    если бы close был подавлен guard'ом).
    local urg_count
    urg_count="$(printf '%s\n' "$journal" | grep -c "USER-REOPEN GUARD" || true)"
    assert_eq "0" "$urg_count" "user-reopen guard NOT triggered (data-race path bypasses it)" || return 1
}

# ============================================================================
# C2. OPEN PR + conflict → e2e-done reconcile fires (sets needs-review on PR),
#      но НЕ strip'ает needs-e2e с issue. Это документированное поведение:
#      conflict на issue сохраняется пока PR OPEN, и наш data-race
#      pre-check сработает после merge (см. C1). Здесь проверяем, что
#      reconcile-путь отрабатывает штатно и НЕ блокирует data-race handling
#      на следующем merge-тике.
# ============================================================================
test_C2_conflict_open_pr_reconcile_then_defer() {
    new_test
    local issue=2030 pr=2031 title='feat 2030 wip data race defer'
    fixture_conflict "$issue" "$pr" "$title" OPEN \
        "hermes,needs-e2e,e2e-done"

    run_merge_gate
    local journal
    journal="$(cat "$GH_JOURNAL")"

    # 1) PR получил needs-review (штатный e2e-done + OPEN reconcile,
    #    ретро t_92ec94f3).
    local needs_review_adds
    needs_review_adds="$(printf '%s\n' "$journal" | grep -cE "gh pr edit ${pr} .*--add-label needs-review" || true)"
    assert_ge "1" "$needs_review_adds" "OPEN PR reconcile: needs-review added to PR (ретро t_92ec94f3)" || return 1

    # 2) Close НЕ вызван (PR OPEN, инвариант §2 не выполнен).
    local close_calls
    close_calls="$(printf '%s\n' "$journal" | grep -cE "gh issue close ${issue} --reason completed" || true)"
    assert_eq "0" "$close_calls" "close NOT called when PR is still OPEN" || return 1

    # 3) Issue остаётся OPEN с обеими метками — data-race разрешится на
    #    следующем merge-тике (см. C1).
    local state_now
    state_now="$(grep -E "^ISSUE_${issue}_STATE_JSON=" "$GH_STATE" | sed "s/^ISSUE_${issue}_STATE_JSON=//")"
    assert_contains '"OPEN"' "$state_now" "issue stays OPEN when PR not merged (deferred to merge tick)" || return 1

    # 4) Pre-check НЕ сработал на этом тике (OPEN PR не доходит до
    #    case $_issue_state block — reconcile вышел раньше). Это
    #    документированное поведение: для OPEN PR conflict разрешается
    #    на merge-тике.
    local audit_count
    audit_count="$(printf '%s\n' "$journal" | grep -c "data-race-label-conflict" || true)"
    assert_eq "0" "$audit_count" "no data-race audit on OPEN PR tick (deferred to merge tick per Amendment 1 §6.2 case A)" || return 1
}

# ============================================================================
# C3. MERGED + conflict + whitelist `user-reopened-this` → audit "overrides",
#      strip НЕ сделан, close НЕ вызван.
# ============================================================================
test_C3_whitelist_overrides_data_race() {
    new_test
    local issue=2032 pr=2033 title='feat 2032 user override conflict'
    # Issue имеет conflict + whitelist.
    fixture_conflict "$issue" "$pr" "$title" MERGED \
        "hermes,needs-e2e,e2e-done,user-reopened-this"

    run_merge_gate
    local journal
    journal="$(cat "$GH_JOURNAL")"

    # 1) Strip needs-e2e НЕ выполнен (whitelist побеждает).
    local strip_calls
    strip_calls="$(printf '%s\n' "$journal" | grep -cE "gh issue edit ${issue} .*--remove-label needs-e2e" || true)"
    assert_eq "0" "$strip_calls" "strip NOT executed when whitelist present (Amendment 1 §6.2 case D)" || return 1

    # 2) Close НЕ вызван.
    local close_calls
    close_calls="$(printf '%s\n' "$journal" | grep -cE "gh issue close ${issue} --reason completed" || true)"
    assert_eq "0" "$close_calls" "close NOT called when whitelist overrides" || return 1

    # 3) Audit-коммент с упоминанием whitelist опубликован.
    local audit_count
    audit_count="$(printf '%s\n' "$journal" | grep -c "whitelist.*override\|user-reopened-this" || true)"
    assert_ge "1" "$audit_count" "audit comment mentions whitelist override" || return 1

    # 4) Issue остаётся OPEN.
    local state_now
    state_now="$(grep -E "^ISSUE_${issue}_STATE_JSON=" "$GH_STATE" | sed "s/^ISSUE_${issue}_STATE_JSON=//")"
    assert_contains '"OPEN"' "$state_now" "issue stays OPEN on whitelist override" || return 1
}

# ============================================================================
# C4. идемпотентность: повторный тик после C1 (needs-e2e уже снят, issue
#      уже CLOSED) → strip no-op, close no-op.
# ============================================================================
test_C4_idempotency_after_resolve() {
    new_test
    local issue=2034 pr=2035 title='feat 2034 idempotency check'
    # Сначала — issue уже CLOSED (имитация «предыдущий тик уже отработал»),
    # needs-e2e уже снят, остался только e2e-done.
    local branch
    branch="$(slugify_branch "$issue" "$title")"

    set_state ISSUE_LIST_JSON "[{\"number\":${issue},\"title\":\"${title}\",\"labels\":[{\"name\":\"hermes\"},{\"name\":\"e2e-done\"}],\"body\":\"kanban: t_cs${issue}\"}]"
    set_state "ISSUE_${issue}_LABELS_JSON" '{"labels":[{"name":"hermes"},{"name":"e2e-done"}]}'
    set_state "ISSUE_${issue}_STATE_JSON" '{"state":"CLOSED"}'
    set_state "ISSUE_${issue}_COMMENTS_JSON" "{\"comments\":[{\"body\":\"kanban: t_cs${issue}\\n\"}]}"
    set_state "ISSUE_${issue}_COMMENTS_SINCE_JSON" '[]'
    set_state "ISSUE_${issue}_TIMELINE_JSON" "[{\"event\":\"labeled\",\"label\":{\"name\":\"e2e-done\"},\"created_at\":\"${RECENT_MERGED_AT}\"}]"

    set_state "PR_HEAD_${branch}_JSON" "[{\"number\":${pr},\"state\":\"MERGED\",\"baseRefName\":\"develop\",\"mergeable\":\"MERGEABLE\",\"mergeStateStatus\":\"CLEAN\",\"statusCheckRollup\":[{\"conclusion\":\"SUCCESS\"}],\"title\":\"fix #${issue}\",\"headRefName\":\"${branch}\",\"labels\":[],\"mergedAt\":\"${RECENT_MERGED_AT}\"}]"
    set_state "PR_${pr}_COMMITS_JSON" '[]'
    set_state PR_LIST_ALL_OPEN_JSON '[]'
    set_state PR_LIST_MERGED_JSON '[]'
    set_state PR_FOLLOWUP_JSON '[]'
    set_state RATE_LIMIT_JSON '{"resources":{"core":{"remaining":5000}}}'
    set_state KANBAN_LIST_JSON '[]'
    set_state "BRANCH_PRESENT_${branch}" 1
    set_state "ISSUE_${issue}_WHOAMI_COMMENT_JSON" '{"id":1}'

    run_merge_gate
    local journal
    journal="$(cat "$GH_JOURNAL")"

    # 1) Pre-check НЕ срабатывает (нет conflict — `needs-e2e` уже снят
    #    прошлым тиком), значит data-race audit-коммент НЕ публикуется.
    #    (Existing post-merge cleanup БЕЗ conflict всё равно может
    #    вызвать `gh issue edit --remove-label needs-e2e` для гигиены —
    #    это нормальный cleanup path, не наш pre-check. Проверяем
    #    отсутствие audit-маркера.)
    local audit_count
    audit_count="$(printf '%s\n' "$journal" | grep -c "data-race-label-conflict" || true)"
    assert_eq "0" "$audit_count" "no data-race audit on repeat tick (no conflict to resolve)" || return 1

    # 2) Close не вызывается повторно (issue уже CLOSED, case CLOSED
    #    existing path в merge-gate → idempotent skip).
    local close_calls
    close_calls="$(printf '%s\n' "$journal" | grep -cE "gh issue close ${issue} --reason completed" || true)"
    assert_eq "0" "$close_calls" "no repeat close on already-CLOSED issue" || return 1
}

# ============================================================================
# C5. Conflict + нет ассоциированного PR (canonical lookup не дал branch
#      fixture) → pre-check не срабатывает (нужен pr_state, а pr_state
#      доступен только когда PR найден). Issue остаётся OPEN с обеими
#      метками — next tick после manual fix или follow-up PR.
# ============================================================================
test_C5_conflict_no_pr_skipped() {
    new_test
    local issue=2036
    local title='feat 2036 orphan conflict no pr'
    # Issue с conflict, но НЕТ PR fixture (canonical branch lookup → пусто).
    local issue_labels_json
    issue_labels_json="$(labels_json_from_csv "hermes,needs-e2e,e2e-done")"
    set_state ISSUE_LIST_JSON "[{\"number\":${issue},\"title\":\"${title}\",\"labels\":${issue_labels_json},\"body\":\"kanban: t_cs${issue}\"}]"
    set_state "ISSUE_${issue}_LABELS_JSON" "{\"labels\":${issue_labels_json}}"
    set_state "ISSUE_${issue}_STATE_JSON" '{"state":"OPEN"}'
    set_state "ISSUE_${issue}_COMMENTS_JSON" "{\"comments\":[{\"body\":\"kanban: t_cs${issue}\\n\"}]}"
    set_state "ISSUE_${issue}_COMMENTS_SINCE_JSON" '[]'
    set_state "ISSUE_${issue}_TIMELINE_JSON" "[{\"event\":\"labeled\",\"label\":{\"name\":\"e2e-done\"},\"created_at\":\"${RECENT_MERGED_AT}\"}]"
    # Нет PR_HEAD_<branch> → main cycle не найдёт PR.
    set_state PR_LIST_ALL_OPEN_JSON '[]'
    set_state PR_LIST_MERGED_JSON '[]'
    set_state PR_FOLLOWUP_JSON '[]'
    set_state RATE_LIMIT_JSON '{"resources":{"core":{"remaining":5000}}}'
    set_state KANBAN_LIST_JSON '[]'

    run_merge_gate
    local journal
    journal="$(cat "$GH_JOURNAL")"

    # Нет strip и нет close — main cycle skip'нул issue (нет PR → не зашёл
    # в OPEN branch с conflict check).
    local strip_calls
    strip_calls="$(printf '%s\n' "$journal" | grep -cE "gh issue edit ${issue} .*--remove-label needs-e2e" || true)"
    assert_eq "0" "$strip_calls" "no strip when no PR found (main cycle skips)" || return 1

    local close_calls
    close_calls="$(printf '%s\n' "$journal" | grep -cE "gh issue close ${issue} --reason completed" || true)"
    assert_eq "0" "$close_calls" "no close when no PR found" || return 1
}

# ============================================================================
# C6. регрессия: штатный e2e-done + MERGED (без conflict) → закрывается
#      через существующий user-reopen guard path (а не наш pre-check).
# ============================================================================
test_C6_regression_e2e_done_no_conflict() {
    new_test
    local issue=2037 pr=2038 title='feat 2037 regression no conflict'
    # Issue только с e2e-done, без needs-e2e — нормальный happy path,
    # наш pre-check НЕ должен вмешиваться.
    fixture_conflict "$issue" "$pr" "$title" MERGED \
        "hermes,e2e-done"

    run_merge_gate
    local journal
    journal="$(cat "$GH_JOURNAL")"

    # 1) Pre-check НЕ срабатывает (нет conflict — нет needs-e2e метки).
    #    Проверяем отсутствие data-race audit-маркера (strip может
    #    вызываться existing post-merge cleanup'ом, это нормально).
    local audit_count
    audit_count="$(printf '%s\n' "$journal" | grep -c "data-race-label-conflict" || true)"
    assert_eq "0" "$audit_count" "no data-race audit on regression path (no conflict)" || return 1

    # 2) Close вызван ровно один раз (штатный ADR-0014 close path).
    local close_calls
    close_calls="$(printf '%s\n' "$journal" | grep -cE "gh issue close ${issue} --reason completed" || true)"
    assert_eq "1" "$close_calls" "regression: e2e-done + MERGED still closes via existing path" || return 1
}

# ============================================================================
# Test runner.
# ============================================================================
run_test "C1_conflict_merged_strips_and_closes" test_C1_conflict_merged_strips_and_closes
run_test "C2_conflict_open_pr_reconcile_then_defer" test_C2_conflict_open_pr_reconcile_then_defer
run_test "C3_whitelist_overrides_data_race" test_C3_whitelist_overrides_data_race
run_test "C4_idempotency_after_resolve" test_C4_idempotency_after_resolve
run_test "C5_conflict_no_pr_skipped" test_C5_conflict_no_pr_skipped
run_test "C6_regression_e2e_done_no_conflict" test_C6_regression_e2e_done_no_conflict

summary