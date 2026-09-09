#!/bin/bash
# ============================================================================
# test_merge_gate_review_handling.sh — review-handling-scan (issue #2063)
#
# Ретро 07.09 t_6127fb86 (PR #2058 / scenario GOODWORKRINKZ): pr-reviewer
# оставил содержательный разбор (3 проблемы, запрос тестов) — НО не сделал
# `gh pr review --request-changes`. Merge-gate не знал что PR «застрял»:
# review был → needs-review поставлен → но никто не создал kanban-карточку
# с разбором → воркер-автор PR не знает что фиксить → PR стоит.
#
# Фикс: новый блок review-handling-scan в merge-gate:
#   - Для каждого OPEN PR (base=develop, non-draft) с COMMENTED review от
#     не-self ревьюера возраста >= REVIEW_HANDLING_MIN_AGE_SECONDS:
#     1) Поставить `needs-followup` на PR
#     2) Снять `needs-review` (review был — переходим в follow-up)
#     3) Создать kanban-карточку с assignee = reviewer→agent mapping
#        (fallback devops если ревьюер — не наш агент)
#     4) Закомментить PR с превью review body + ссылкой на карточку
#     5) Cooldown через STALE_AUTO_BLOCK_STATE_DIR/review-handling-state.json
#   - Idempotency: needs-followup уже стоит → skip. Cooldown активен → skip.
#   - Не триггерим на APPROVED / CHANGES_REQUESTED / DISMISSED / self-review.
#
# Scenarios:
#   R1. COMMENTED review от внешнего ревьюера (GOODWORKRINKZ) возраст 5м →
#       needs-followup поставлен, needs-review снят, kanban-card создан,
#       PR comment опубликован.
#   R2. APPROVED review → scan НЕ реагирует (ничего не создано).
#   R3. CHANGES_REQUESTED review → scan НЕ реагирует (GitHub сам блокирует).
#   R4. COMMENTED review от самого merge-gate (self-login) → scan НЕ реагирует.
#   R5. Идемпотентность: PR с уже стоящим `needs-followup` → scan skip,
#       kanban НЕ создаётся повторно.
#   R6. Cooldown: state-файл содержит недавнюю запись для PR → scan skip,
#       kanban НЕ создаётся.
#   R7. Минимальный возраст: review < MIN_AGE → scan skip.
#   R8. assignee-mapping: reviewer=devops → kanban assignee=devops.
#
# Run:
#   bash scripts/agent_flow/tests/test_merge_gate_review_handling.sh
# ============================================================================
set -euo pipefail

TEST_LIB_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/mock_env.sh
. "$TEST_LIB_DIR/lib/mock_env.sh"

# Helper: build ISO timestamp N seconds ago (merge-gate age filter)
iso_ago() {  # $1=seconds_ago
    local s="$1"
    date -u -d "@$(($(date +%s) - s))" +%Y-%m-%dT%H:%M:%SZ
}

# Fixture: open PR + reviews + labels. Sets state for review-handling scan.
#   $1=pr $2=head $3=title $4=labels_csv $5=reviews_json
fixture_open_pr_with_reviews() {
    local pr="$1" head="$2" title="$3" labels_csv="$4" reviews_json="$5"
    local labels_json="[]"
    if [ -n "$labels_csv" ]; then
        labels_json="[$(printf '%s' "$labels_csv" | tr ',' '\n' | sed 's/.*/{"name":"&"}/' | paste -sd, -)]"
    fi
    set_state PR_LIST_ALL_OPEN_JSON "[{\"number\":${pr},\"headRefName\":\"${head}\",\"title\":\"${title}\",\"isDraft\":false,\"labels\":${labels_json}}]"
    set_state "PR_${pr}_REVIEWS_JSON" "$reviews_json"
    set_state RATE_LIMIT_JSON '{"resources":{"core":{"remaining":5000}}}'
    set_state ISSUE_LIST_JSON '[]'
    # Default merge-gate self-login: krikz-bot. Tests переопределяют через
    # GH_USER_LOGIN_JSON, если хотят симулировать «review от самого себя».
    set_state GH_USER_LOGIN_JSON 'krikz-bot'
    # Cooldown state file: чистый (нет недавних записей).
    mkdir -p "${STALE_AUTO_BLOCK_STATE_DIR:-/tmp/agent-flow-merge-gate-tests.*/stale-auto-block-state}" 2>/dev/null || true
    set_state "PR_${pr}_STATE_JSON" '{"state":"OPEN"}'
}

# ============================================================================
# R1. COMMENTED review от внешнего ревьюера (GOODWORKRINKZ) возраст 5м →
#     needs-followup поставлен, needs-review снят, kanban-card создан.
# ============================================================================
test_R1_external_review_gets_followup() {
    new_test
    local pr=2058
    local submitted; submitted="$(iso_ago 300)"  # 5 минут назад
    local reviews_json
    reviews_json=$(cat <<EOF
[{"id":100,"user":{"login":"GOODWORKRINKZ"},"state":"COMMENTED","submitted_at":"${submitted}","body":"Проблема 1: docker-compose healthcheck не покрывает новый voice-node.\nПроблема 2: тесты на ROS2 mock-state не запускаются в CI.\nПроблема 3: документация ADR-0055 не описывает backward-incompat.\n\nПрошу добавить интеграционный тест на Docker-стенде."}]
EOF
)
    fixture_open_pr_with_reviews "$pr" 'z-backend/2058-voice-step05b' \
        'feat(voice) #2058: ADR-0055 step 05b' 'needs-review' "$reviews_json"
    # Kanban-card id, который вернёт mock:
    set_state KANBAN_CREATE_ID 't_review_2058'

    run_merge_gate

    local journal
    journal="$(cat "$GH_JOURNAL")"

    assert_eq "1" "$(printf '%s\n' "$journal" | grep -c "gh pr edit $pr .*--add-label needs-followup" || true)" \
        "R1: needs-followup поставлен на PR" || return 1
    assert_eq "1" "$(printf '%s\n' "$journal" | grep -c "gh pr edit $pr .*--remove-label needs-review" || true)" \
        "R1: needs-review снят (review был)" || return 1
    assert_eq "1" "$(printf '%s\n' "$journal" | grep -c "kanban .* --assignee devops --idempotency-key pr-review-handling-pr2058" || true)" \
        "R1: kanban-card создан (assignee=devops — fallback для внешнего ревьюера)" || return 1
    # PR comment — проверяем на наличие маркера review-handling-scan (уникальная фраза).
    assert_eq "1" "$(printf '%s\n' "$journal" | grep -c "review-handling-scan (merge-gate, issue #2063" || true)" \
        "R1: PR comment с маркером review-handling-scan опубликован" || return 1
    # Cooldown state-file обновлён — проверяем что python update прошёл
    # (лог в stderr, не в journal — поэтому assert через сам файл).
    local state_dir="${STALE_AUTO_BLOCK_STATE_DIR:-$TEST_TMP/stale-auto-block-state}"
    local state_file="$state_dir/review-handling-state.json"
    assert_eq "1" "$([ -f "$state_file" ] && python3 -c 'import json,sys
d=json.load(open(sys.argv[1]))
print(1 if str(2058) in d or 2058 in d else 0)' "$state_file" 2>/dev/null || echo 0)" \
        "R1: cooldown state-file содержит запись для PR #2058" || return 1
}

# ============================================================================
# R2. APPROVED review → scan НЕ реагирует.
# ============================================================================
test_R2_approved_review_no_followup() {
    new_test
    local pr=2060
    local submitted; submitted="$(iso_ago 600)"
    local reviews_json
    reviews_json=$(cat <<EOF
[{"id":101,"user":{"login":"some-reviewer"},"state":"APPROVED","submitted_at":"${submitted}","body":"LGTM, ship it."}]
EOF
)
    fixture_open_pr_with_reviews "$pr" 'z-backend/2060-approve-demo' \
        'feat: demo' '' "$reviews_json"

    run_merge_gate

    local journal
    journal="$(cat "$GH_JOURNAL")"

    assert_eq "0" "$(printf '%s\n' "$journal" | grep -c -- '--add-label needs-followup' || true)" \
        "R2: APPROVED review → needs-followup NOT set" || return 1
    assert_eq "0" "$(printf '%s\n' "$journal" | grep -c 'kanban .* create' || true)" \
        "R2: APPROVED review → kanban NOT created" || return 1
}

# ============================================================================
# R3. CHANGES_REQUESTED review → scan НЕ реагирует (GitHub сам блокирует).
# ============================================================================
test_R3_changes_requested_review_no_followup() {
    new_test
    local pr=2061
    local submitted; submitted="$(iso_ago 600)"
    local reviews_json
    reviews_json=$(cat <<EOF
[{"id":102,"user":{"login":"strict-reviewer"},"state":"CHANGES_REQUESTED","submitted_at":"${submitted}","body":"Fix this."}]
EOF
)
    fixture_open_pr_with_reviews "$pr" 'z-backend/2061-changes-req' \
        'fix: demo' '' "$reviews_json"

    run_merge_gate

    local journal
    journal="$(cat "$GH_JOURNAL")"

    assert_eq "0" "$(printf '%s\n' "$journal" | grep -c -- '--add-label needs-followup' || true)" \
        "R3: CHANGES_REQUESTED review → needs-followup NOT set" || return 1
    assert_eq "0" "$(printf '%s\n' "$journal" | grep -c 'kanban .* create' || true)" \
        "R3: CHANGES_REQUESTED review → kanban NOT created" || return 1
}

# ============================================================================
# R4. COMMENTED review от самого merge-gate (self-login) → scan НЕ реагирует.
# ============================================================================
test_R4_self_review_no_followup() {
    new_test
    local pr=2062
    local submitted; submitted="$(iso_ago 300)"
    local reviews_json
    reviews_json=$(cat <<EOF
[{"id":103,"user":{"login":"krikz-bot"},"state":"COMMENTED","submitted_at":"${submitted}","body":"whoami-комментарий от самого merge-gate."}]
EOF
)
    fixture_open_pr_with_reviews "$pr" 'z-backend/2062-self-review' \
        'feat: demo' '' "$reviews_json"

    run_merge_gate

    local journal
    journal="$(cat "$GH_JOURNAL")"

    assert_eq "0" "$(printf '%s\n' "$journal" | grep -c -- '--add-label needs-followup' || true)" \
        "R4: self review → needs-followup NOT set" || return 1
    assert_eq "0" "$(printf '%s\n' "$journal" | grep -c 'kanban .* create' || true)" \
        "R4: self review → kanban NOT created" || return 1
}

# ============================================================================
# R5. Идемпотентность: PR с уже стоящим `needs-followup` → scan skip.
# ============================================================================
test_R5_idempotent_when_needs_followup_already_set() {
    new_test
    local pr=2063
    local submitted; submitted="$(iso_ago 300)"
    local reviews_json
    reviews_json=$(cat <<EOF
[{"id":104,"user":{"login":"GOODWORKRINKZ"},"state":"COMMENTED","submitted_at":"${submitted}","body":"Уже разобрали, но Шифу снял метку — снова триггерим."}]
EOF
)
    # Метка needs-followup УЖЕ стоит → skip.
    fixture_open_pr_with_reviews "$pr" 'z-backend/2063-already-followup' \
        'feat: demo' 'needs-followup,needs-review' "$reviews_json"

    run_merge_gate

    local journal
    journal="$(cat "$GH_JOURNAL")"

    assert_eq "0" "$(printf '%s\n' "$journal" | grep -c -- '--add-label needs-followup' || true)" \
        "R5: needs-followup уже стоит → NOT add-label again" || return 1
    assert_eq "0" "$(printf '%s\n' "$journal" | grep -c 'kanban .* create' || true)" \
        "R5: needs-followup уже стоит → kanban NOT created (idempotency)" || return 1
}

# ============================================================================
# R6. Cooldown: state-файл содержит недавнюю запись для PR → scan skip.
# ============================================================================
test_R6_cooldown_blocks_retrigger() {
    new_test
    local pr=2064
    local submitted; submitted="$(iso_ago 300)"
    local reviews_json
    reviews_json=$(cat <<EOF
[{"id":105,"user":{"login":"GOODWORKRINKZ"},"state":"COMMENTED","submitted_at":"${submitted}","body":"Новый review через час после первого."}]
EOF
)
    fixture_open_pr_with_reviews "$pr" 'z-backend/2064-cooldown' \
        'feat: demo' '' "$reviews_json"

    # Cooldown: PR был обработан 30 мин назад (< 6ч cooldown). Запишем в state-файл.
    local state_dir="${STALE_AUTO_BLOCK_STATE_DIR:-$TEST_TMP/stale-auto-block-state}"
    mkdir -p "$state_dir"
    local state_file="$state_dir/review-handling-state.json"
    local now_minus_30min; now_minus_30min="$(($(date +%s) - 1800))"
    printf '{"%s":%s}' "$pr" "$now_minus_30min" > "$state_file"

    run_merge_gate

    local journal
    journal="$(cat "$GH_JOURNAL")"

    assert_eq "0" "$(printf '%s\n' "$journal" | grep -c -- '--add-label needs-followup' || true)" \
        "R6: cooldown active → needs-followup NOT set" || return 1
    assert_eq "0" "$(printf '%s\n' "$journal" | grep -c 'kanban .* create' || true)" \
        "R6: cooldown active → kanban NOT created" || return 1
}

# ============================================================================
# R7. Минимальный возраст: review 10с назад (< 60с MIN_AGE) → scan skip.
# ============================================================================
test_R7_min_age_blocks_recent_review() {
    new_test
    local pr=2065
    local submitted; submitted="$(iso_ago 10)"  # 10 секунд назад
    local reviews_json
    reviews_json=$(cat <<EOF
[{"id":106,"user":{"login":"GOODWORKRINKZ"},"state":"COMMENTED","submitted_at":"${submitted}","body":"Свежий review, ещё не устоялся."}]
EOF
)
    fixture_open_pr_with_reviews "$pr" 'z-backend/2065-recent-review' \
        'feat: demo' '' "$reviews_json"

    run_merge_gate

    local journal
    journal="$(cat "$GH_JOURNAL")"

    assert_eq "0" "$(printf '%s\n' "$journal" | grep -c -- '--add-label needs-followup' || true)" \
        "R7: review < 60s → NOT triggered (MIN_AGE)" || return 1
    assert_eq "0" "$(printf '%s\n' "$journal" | grep -c 'kanban .* create' || true)" \
        "R7: review < 60s → kanban NOT created" || return 1
}

# ============================================================================
# R8. assignee-mapping: reviewer=devops → kanban assignee=devops (наш агент).
# ============================================================================
test_R8_internal_agent_reviewer_mapped_to_agent() {
    new_test
    local pr=2066
    local submitted; submitted="$(iso_ago 600)"
    local reviews_json
    reviews_json=$(cat <<EOF
[{"id":107,"user":{"login":"backend"},"state":"COMMENTED","submitted_at":"${submitted}","body":"От backend-агента — воркер сам себе поревьюил, идём в follow-up."}]
EOF
)
    fixture_open_pr_with_reviews "$pr" 'z-developer/2066-internal-reviewer' \
        'feat: demo' '' "$reviews_json"
    set_state KANBAN_CREATE_ID 't_review_internal'

    run_merge_gate

    local journal
    journal="$(cat "$GH_JOURNAL")"

    assert_eq "1" "$(printf '%s\n' "$journal" | grep -c -- '--add-label needs-followup' || true)" \
        "R8: needs-followup поставлен (внутренний ревьюер)" || return 1
    assert_eq "1" "$(printf '%s\n' "$journal" | grep -c "kanban .* create .* --assignee backend" || true)" \
        "R8: kanban-card assignee=backend (mapped from reviewer)" || return 1
}

# ============================================================================
# Run all tests.
# ============================================================================
run_test "R1. External reviewer COMMENTED → needs-followup + kanban-card" test_R1_external_review_gets_followup
run_test "R2. APPROVED review → no followup" test_R2_approved_review_no_followup
run_test "R3. CHANGES_REQUESTED review → no followup" test_R3_changes_requested_review_no_followup
run_test "R4. Self review → no followup" test_R4_self_review_no_followup
run_test "R5. needs-followup уже стоит → idempotent skip" test_R5_idempotent_when_needs_followup_already_set
run_test "R6. Cooldown active → no retrigger" test_R6_cooldown_blocks_retrigger
run_test "R7. Review < MIN_AGE → skip" test_R7_min_age_blocks_recent_review
run_test "R8. Internal agent reviewer → assignee=agent" test_R8_internal_agent_reviewer_mapped_to_agent

summary
