#!/bin/bash
# ============================================================================
# test_merge_gate_dead_card_recovery.sh — ретро #1356 (10.08 процессный баг)
#
# Сценарий: воркер завершает свою kanban-карточку (статус done/archived),
# но PR остаётся CONFLICTING или UNSTABLE. Старое поведение merge-gate —
# дописать reminder в закрытую карточку: никто не видит, конфликт висит
# бессрочно (см. ретро t_75e787fd-2, issue #1356, кейс PR #1347/issue #1344).
#
# Тесты проверяют ОСНОВНОЙ ЦИКЛ merge-gate (не scan-all-prs), т.к. основной
# цикл — то место, где раньше reminder'ы уходили в мёртвые карточки.
#
# Фикс (18.08):
#   - живая карточка (running/ready/todo) → comment в неё (старое поведение);
#   - мёртвая (done/blocked/archived) → создаём СВЕЖУЮ recovery-карточку с
#     assignee=владелец PR по меткам issue, идемпотентно по PR (см. lines
#     1283-1440 в agent-flow-merge-gate.sh).
#
# Scenarios:
#   M1. CONFLICTING + done-карточка → НЕ пишем comment в done-карточку,
#       создаём СВЕЖУЮ recovery-карточку с assignee=backend (метка agent:backend).
#   M2. UNSTABLE + done-карточка → НЕ пишем comment в done-карточку,
#       создаём СВЕЖУЮ recovery-карточку с assignee=devops.
#   M3. CONFLICTING + ready-карточка → comment в карточку (старое поведение),
#       НЕ создаём recovery-карточку.
#   M4. CONFLICTING + done-карточка, но recovery-карточка уже есть (active) →
#       НЕ создаём дубликат, НЕ пишем в мёртвую (scan-all-prs подхватит).
#   M5. CONFLICTING + done-карточка, PR был закрыт → НЕ создаём recovery
#       (race PR close, ретро t_16325ddd).
#
# Run:
#   bash scripts/agent_flow/tests/test_merge_gate_dead_card_recovery.sh
# ============================================================================
set -euo pipefail

TEST_LIB_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/mock_env.sh
. "$TEST_LIB_DIR/lib/mock_env.sh"

# Имя ветки по соглашению triage (z-{agent}/<issue>-slug). Дублируем из
# test_merge_gate_post_merge.sh — общая версия появится, если будет третий
# пользователь.
slugify_branch() {  # $1=issue $2=title
    local issue="$1" title="$2" slug
    slug="$(printf '%s' "$title" | tr '[:upper:]' '[:lower:]' \
        | sed -E 's/[^a-z0-9]+/-/g; s/^-+|-+$//g; s/-{2,}/-/g' \
        | cut -c1-40)"
    printf 'z-{agent}/%s-%s' "$issue" "$slug"
}

# ---------------------------------------------------------------------------
# Fixture для основного цикла: hermes-issue с CONFLICTING/UNSTABLE PR,
# kanban-маркер `kanban: t_<id>` в issue comments, PR с нужным mergeable/
# mergeStateStatus.
#
# $1=issue $2=pr $3=task_id $4=mergeable $5=merge_state $6=assignee_label
# ---------------------------------------------------------------------------
fixture_main_cycle() {
    local issue="$1" pr="$2" task_id="$3" mergeable="$4" merge_state="$5" assignee_label="$6"
    local branch
    branch="$(slugify_branch "$issue" "fix #${issue} conflict demo")"
    # Issue проходит по основному циклу: hermes + assignee-метка (НЕ e2e-done/needs-e2e — иначе skip).
    set_state ISSUE_LIST_JSON "[{\"number\":${issue},\"title\":\"fix #${issue} conflict demo\",\"labels\":[{\"name\":\"hermes\"},{\"name\":\"agent:${assignee_label}\"}],\"body\":\"kanban: t_${task_id}\"}]"
    set_state "ISSUE_${issue}_LABELS_JSON" "{\"labels\":[{\"name\":\"hermes\"},{\"name\":\"agent:${assignee_label}\"}]}"
    set_state "ISSUE_${issue}_STATE_JSON" '{"state":"OPEN"}'
    set_state "ISSUE_${issue}_COMMENTS_JSON" "{\"comments\":[{\"body\":\"kanban: ${task_id}\\nbranch: ${branch}\\nrole: ${assignee_label}\\n\"}]}"
    set_state "ISSUE_${issue}_COMMENTS_SINCE_JSON" '[]'
    set_state "ISSUE_${issue}_JSON" "{\"number\":${issue},\"labels\":[{\"name\":\"hermes\"},{\"name\":\"agent:${assignee_label}\"}]}"
    # PR для основного цикла: ищется по headRefName = branch (z-{agent}/<id>-slug).
    # statusCheckRollup нужен, чтобы main-cycle не скипнул по «no rollup» (line 1237).
    set_state "PR_HEAD_${branch}_JSON" "[{\"number\":${pr},\"title\":\"fix #${issue} conflict demo\",\"headRefName\":\"${branch}\",\"baseRefName\":\"develop\",\"state\":\"OPEN\",\"mergeable\":\"${mergeable}\",\"mergeStateStatus\":\"${merge_state}\",\"statusCheckRollup\":[{\"conclusion\":\"SUCCESS\"}],\"labels\":[]}]"
    # scan-all-prs: пустой, чтобы наш main-cycle шёл первым и не дублировал работу.
    set_state PR_LIST_ALL_OPEN_JSON '[]'
    set_state PR_LIST_MERGED_JSON '[]'
    set_state PR_FOLLOWUP_JSON '[]'
    set_state RATE_LIMIT_JSON '{"resources":{"core":{"remaining":5000}}}'
    # gh pr view <num> --json headRefName (для CONFLICTING/UNSTABLE-блоков).
    # Mock «(other)» ветка gh pr view берёт PR_<n>_VIEW_JSON.
    set_state "PR_${pr}_VIEW_JSON" "{\"headRefName\":\"${branch}\",\"state\":\"OPEN\"}"
    # gh pr view <num> (для pr_state_now в UNSTABLE recovery-карточке).
    set_state "PR_${pr}_STATE_JSON" '{"state":"OPEN"}'
    # stale-branch guard: нет MERGED PR с этим head.
    set_state "PR_MERGED_HEAD_${branch}_JSON" ''
    set_state "BRANCH_PRESENT_${branch}" 1
    # save context for tests
    FIXTURE_BRANCH="$branch"
}

# ===========================================================================
# M1. CONFLICTING + done-карточка → recovery-карточка, comment НЕ в done
# ===========================================================================
test_M1_conflicting_done_card_creates_recovery_not_comment() {
    new_test
    local issue=4501 pr=4502 task_id=t_d4501
    fixture_main_cycle "$issue" "$pr" "$task_id" CONFLICTING DIRTY backend
    # Карточка УЖЕ done (воркер завершился раньше — кейс ретро #1356).
    set_state KANBAN_LIST_JSON "[{\"id\":\"${task_id}\",\"title\":\"fix #${issue}\",\"status\":\"done\"}]"

    run_merge_gate

    # Главное: comment в done-карточку НЕ пишем.
    local comments
    comments="$(journal_grep "hermes kanban --board robbox comment ${task_id}")"
    assert_not_contains "merge conflict detected" "$comments" \
        "M1: comment с merge-conflict-reminder НЕ должен уходить в done-карточку ${task_id}"
    # Recovery-карточка создаётся с assignee=backend (из метки agent:backend).
    local creates
    creates="$(journal_grep 'hermes kanban --board robbox create')"
    assert_contains "hermes kanban --board robbox create" "$creates" \
        "M1: recovery-карточка создаётся через hermes kanban create"
    assert_contains -- "--assignee backend" "$(cat "$GH_JOURNAL")" \
        "M1: assignee=backend (из метки agent:backend)"
    # Title содержит PR-номер.
    assert_contains "rebase PR #${pr}" "$(cat "$GH_JOURNAL")" \
        "M1: title recovery-карточки содержит PR-номер"
    # body содержит ретро-ссылку #1356 (признак «это рекавери-карточка от ретро #1356»).
    assert_contains "ретро #1356" "$(cat "$GH_JOURNAL")" \
        "M1: body содержит «ретро #1356»"
}

# ===========================================================================
# M2. UNSTABLE + done-карточка → recovery-карточка, comment НЕ в done
# ===========================================================================
test_M2_unstable_done_card_creates_recovery_not_comment() {
    new_test
    local issue=4511 pr=4512 task_id=t_d4511
    fixture_main_cycle "$issue" "$pr" "$task_id" MERGEABLE UNSTABLE devops
    set_state KANBAN_LIST_JSON "[{\"id\":\"${task_id}\",\"title\":\"fix #${issue}\",\"status\":\"done\"}]"

    run_merge_gate

    local comments
    comments="$(journal_grep "hermes kanban --board robbox comment ${task_id}")"
    assert_not_contains "CI UNSTABLE detected" "$comments" \
        "M2: comment с CI-UNSTABLE-reminder НЕ должен уходить в done-карточку ${task_id}"
    local creates
    creates="$(journal_grep 'hermes kanban --board robbox create')"
    assert_contains "hermes kanban --board robbox create" "$creates" \
        "M2: recovery-карточка создаётся"
    assert_contains -- "--assignee devops" "$(cat "$GH_JOURNAL")" \
        "M2: assignee=devops (из метки agent:devops)"
    assert_contains "CI UNSTABLE" "$(cat "$GH_JOURNAL")" \
        "M2: title содержит CI UNSTABLE (это UNSTABLE-рекавери-карточка)"
}

# ===========================================================================
# M3. CONFLICTING + ready-карточка → comment в неё (старое поведение),
#     НЕ создаём recovery-карточку
# ===========================================================================
test_M3_conflicting_ready_card_keeps_comment_no_recovery() {
    new_test
    local issue=4521 pr=4522 task_id=t_d4521
    fixture_main_cycle "$issue" "$pr" "$task_id" CONFLICTING DIRTY backend
    set_state KANBAN_LIST_JSON "[{\"id\":\"${task_id}\",\"title\":\"fix #${issue}\",\"status\":\"ready\"}]"

    run_merge_gate

    local creates
    creates="$(journal_grep 'hermes kanban --board robbox create')"
    # Создание карточки НЕ должно происходить (карточка живая — Шифу прямо:
    # «та же карточка должна знать»).
    assert_not_contains "hermes kanban --board robbox create" "$creates" \
        "M3: при живой ready-карточке recovery-карточка НЕ создаётся (Шифу прямо: «та же карточка»)"
    # Comment пишется в карточку.
    local comments
    comments="$(journal_grep "hermes kanban --board robbox comment ${task_id}")"
    assert_contains "merge conflict detected" "$comments" \
        "M3: comment с merge-conflict-reminder пишется в живую ready-карточку"
}

# ===========================================================================
# M4. CONFLICTING + done-карточка, НО recovery-карточка уже есть (running).
#     НЕ создаём дубликат, НЕ пишем в мёртвую.
# ===========================================================================
test_M4_conflicting_dead_card_existing_active_recovery_skipped() {
    new_test
    local issue=4531 pr=4532 task_id=t_d4531
    fixture_main_cycle "$issue" "$pr" "$task_id" CONFLICTING DIRTY backend
    # Две карточки: мёртвая исходная + активная recovery.
    set_state KANBAN_LIST_JSON "[{\"id\":\"${task_id}\",\"title\":\"fix #${issue}\",\"status\":\"done\"},{\"id\":\"t_recovery_active\",\"title\":\"🔀 rebase PR #${pr}\",\"status\":\"running\"}]"

    run_merge_gate

    # Активная recovery уже подобрана scan-all-prs — main-cycle должен НЕ создавать новую.
    local creates
    creates="$(journal_grep 'hermes kanban --board robbox create')"
    assert_not_contains "hermes kanban --board robbox create" "$creates" \
        "M4: при существующей активной recovery-карточке main-cycle НЕ создаёт дубликат"
    local comments
    comments="$(journal_grep "hermes kanban --board robbox comment ${task_id}")"
    assert_not_contains "merge conflict detected" "$comments" \
        "M4: comment НЕ пишется в done-карточку"
}

# ===========================================================================
# M5. CONFLICTING + done-карточка, НО PR уже CLOSED → recovery НЕ создаём
#     (ретро t_16325ddd: race PR close)
# ===========================================================================
test_M5_conflicting_dead_card_but_pr_closed_no_recovery() {
    new_test
    local issue=4541 pr=4542 task_id=t_d4541
    fixture_main_cycle "$issue" "$pr" "$task_id" CONFLICTING DIRTY backend
    set_state KANBAN_LIST_JSON "[{\"id\":\"${task_id}\",\"title\":\"fix #${issue}\",\"status\":\"done\"}]"
    # PR closed (race: товарищ Шифу «Не делаем это»).
    # gh mock читает PR_${pr}_VIEW_JSON для `--json state` (см. mock_env.sh
    # lines 550-553: всё кроме files/rollup/number → "(other)" branch).
    set_state "PR_${pr}_VIEW_JSON" "{\"headRefName\":\"z-{agent}/4541-fix-4541-conflict-demo\",\"state\":\"CLOSED\"}"

    run_merge_gate

    # Ретро #1356 fix verification: при CLOSED PR recovery-карточка НЕ создаётся.
    # NB: assert_not_contains возвращает 1 при негативном матче, но `if "$fn"`
    # в фреймворке подавляет set -e на bare-вызов, поэтому оборачиваем в `!`
    # чтобы CI упал когда фикс сломается.
    local creates
    creates="$(journal_grep 'hermes kanban --board robbox create')"
    if assert_not_contains "hermes kanban --board robbox create" "$creates" \
        "M5: при closed PR recovery-карточка НЕ создаётся (race PR close, ретро t_16325ddd)"; then
        : # positive match found — assert_not_contains returns 0 when NEEDLE IS NOT in HAYSTACK
    else
        return 1
    fi
    local comments
    comments="$(journal_grep "hermes kanban --board robbox comment ${task_id}")"
    if assert_not_contains "merge conflict detected" "$comments" \
        "M5: comment НЕ пишется в done-карточку при closed PR"; then
        :
    else
        return 1
    fi
}

# ---------------------------------------------------------------------------
# Registry
# ---------------------------------------------------------------------------
run_test "M1_conflicting_done_card_creates_recovery_not_comment" test_M1_conflicting_done_card_creates_recovery_not_comment
run_test "M2_unstable_done_card_creates_recovery_not_comment" test_M2_unstable_done_card_creates_recovery_not_comment
run_test "M3_conflicting_ready_card_keeps_comment_no_recovery" test_M3_conflicting_ready_card_keeps_comment_no_recovery
run_test "M4_conflicting_dead_card_existing_active_recovery_skipped" test_M4_conflicting_dead_card_existing_active_recovery_skipped
run_test "M5_conflicting_dead_card_but_pr_closed_no_recovery" test_M5_conflicting_dead_card_but_pr_closed_no_recovery

summary
