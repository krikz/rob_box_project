#!/bin/bash
# ============================================================================
# test_pr_contract_drift_bot.sh — ADR-0044 acceptance
# (ретро t_527e1231 → process-fix t_58c69473, блок C)
#
# Тестируем `.github/scripts/pr_contract_drift_bot.sh` через мок `gh` в PATH.
# Проверяем ИНВАРИАНТЫ ADR-0044 §3 (что бот НЕ делает) и полезное поведение.
#
# Сценарии:
#   C1. OPEN + UNSTABLE + failing «Unit Tests» → комментарий с заголовком
#       «CI failing: rebase candidate (…) vs contract-drift inside PR (…)».
#   C2. CONFLICTING/DIRTY → комментария НЕТ (rebase корректен).
#   C3. только Integration Tests FAILURE → комментария НЕТ (rebase_candidate).
#   C4. нет красных чеков → комментария НЕТ (unknown, fail-open).
#   C5. CLOSED PR → комментария НЕТ.
#   C6. дедуп: свой комментарий в окне → PATCH существующего, без POST.
#   C7. бот НИКОГДА не зовёт метки/close/kanban (инварианты §3).
#
# Run:
#   bash scripts/agent_flow/tests/test_pr_contract_drift_bot.sh
# ============================================================================
set -euo pipefail

RED=$'\033[31m'; GRN=$'\033[32m'; BLU=$'\033[34m'; YEL=$'\033[33m'; END=$'\033[0m'
TESTS_TOTAL=0; TESTS_PASSED=0; TESTS_FAILED=0; FAILED_NAMES=()

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
BOT="$REPO_ROOT/.github/scripts/pr_contract_drift_bot.sh"
[ -f "$BOT" ] || { printf 'FAIL: bot script not found at %s\n' "$BOT"; exit 1; }

run_test() {  # $1=name $2=fn
    TESTS_TOTAL=$((TESTS_TOTAL+1))
    printf '%s[ RUN     ]%s %s\n' "$BLU" "$END" "$1"
    if "$2"; then
        TESTS_PASSED=$((TESTS_PASSED+1))
        printf '%s[   PASS  ]%s %s\n' "$GRN" "$END" "$1"
    else
        TESTS_FAILED=$((TESTS_FAILED+1)); FAILED_NAMES+=("$1")
        printf '%s[   FAIL  ]%s %s\n' "$RED" "$END" "$1"
    fi
}
assert_contains() {  # $1=needle $2=haystack $3=msg
    case "$2" in
        *"$1"*) return 0 ;;
        *) printf '  %sassert fail:%s %s\n    needle: %q\n' "$RED" "$END" "$3" "$1" >&2; return 1 ;;
    esac
}
assert_not_contains() {  # $1=needle $2=haystack $3=msg
    case "$2" in
        *"$1"*) printf '  %sassert fail:%s %s\n    unexpected: %q\n' "$RED" "$END" "$3" "$1" >&2; return 1 ;;
    esac
    return 0
}

# --- Мок gh -----------------------------------------------------------------
# Читает фикстуры из $BOT_STATE (key=value), пишет вызовы в $BOT_JOURNAL.
#   PR_VIEW_JSON               — ответ `gh pr view --json ...`
#   COMMENTS_SINCE_JSON        — ответ `gh api .../comments?since=...`
setup_bot_mock() {
    BOT_TMP="$(mktemp -d /tmp/contract-drift-bot-tests.XXXXXX)"
    BOT_STATE="$BOT_TMP/state"; BOT_JOURNAL="$BOT_TMP/journal"
    : >"$BOT_STATE"; : >"$BOT_JOURNAL"
    mkdir -p "$BOT_TMP/bin"
    cat >"$BOT_TMP/bin/gh" <<'GH_EOF'
#!/bin/bash
state="${BOT_STATE:-}"; journal="${BOT_JOURNAL:-/dev/null}"
get_state() { grep -E "^$1=" "$state" 2>/dev/null | head -n1 | sed "s@^$1=@@"; }
printf 'gh %s\n' "$*" >>"$journal"
sub="${1:-}"; shift || true
case "$sub" in
    pr)
        act="${1:-}"; shift || true
        case "$act" in
            view)    get_state PR_VIEW_JSON ;;
            comment) printf 'https://github.com/x/y/pull/1#issuecomment-1\n' ;;
            *)       : ;;
        esac
        ;;
    api)
        # -X PATCH ... → журналируем и выходим; GET comments → фикстура.
        if [ "${1:-}" = "-X" ]; then exit 0; fi
        case "$1" in
            *"/comments?since="*)
                # Бот применяет --jq '... | last | .id // ""'. Реальный gh
                # вернул бы пустую строку для []. Мок эмулирует уже-извлечённый
                # результат: фикстура '[]' (нет комментариев) → пустой вывод,
                # число → id существующего комментария бота.
                _v="$(get_state COMMENTS_SINCE_JSON)"
                [ "$_v" = "[]" ] && _v=""
                printf '%s' "$_v"
                ;;
            *) : ;;
        esac
        ;;
    *) : ;;
esac
exit 0
GH_EOF
    chmod +x "$BOT_TMP/bin/gh"
    # Выкидываем прошлые мок-bin из PATH.
    local real="" p
    IFS=: read -ra _parts <<<"$PATH"
    for p in "${_parts[@]}"; do
        case "$p" in
            */contract-drift-bot-tests.*/bin) ;;
            *) real="${real:+$real:}$p" ;;
        esac
    done
    export PATH="$BOT_TMP/bin:$real"
    export BOT_STATE BOT_JOURNAL
}
set_bot_state() {  # $1=key $2=value
    local tmpf="${BOT_STATE}.tmp.$$"
    grep -v "^$1=" "$BOT_STATE" >"$tmpf" 2>/dev/null || true
    printf '%s=%s\n' "$1" "$2" >>"$tmpf"; mv "$tmpf" "$BOT_STATE"
}
run_bot() {
    GH_REPO=krikz/test-repo PR_NUMBER=1857 DEDUP_HOURS=6 \
        bash "$BOT" >"$BOT_TMP/stdout.log" 2>"$BOT_TMP/stderr.log" || true
}

ROLLUP_DRIFT='{"state":"OPEN","mergeStateStatus":"UNSTABLE","headRefName":"z-architect/t_5d93c7b1-adr","statusCheckRollup":[{"name":"Unit Tests (ROS2 Humble)","conclusion":"FAILURE","detailsUrl":"https://gh/job/99842198694"},{"name":"Shell Scripts","conclusion":"SUCCESS","detailsUrl":""}]}'

# ===========================================================================
test_C1_contract_drift_comments() {
    setup_bot_mock
    set_bot_state PR_VIEW_JSON "$ROLLUP_DRIFT"
    set_bot_state COMMENTS_SINCE_JSON '[]'
    run_bot
    local j; j="$(cat "$BOT_JOURNAL")"
    assert_contains "gh pr comment 1857" "$j" "bot posts a PR comment (C1)" || return 1
    assert_contains "CI failing: rebase candidate" "$j" "comment carries the ADR-0044 headline (C1)" || return 1
    assert_contains "contract-drift inside PR" "$j" "headline mentions contract-drift (C1)" || return 1
    assert_contains "Unit Tests (ROS2 Humble)" "$j" "failing job name is listed (C1)" || return 1
    assert_contains "99842198694" "$j" "job URL is listed (C1)" || return 1
}

test_C2_conflicting_no_comment() {
    setup_bot_mock
    set_bot_state PR_VIEW_JSON '{"state":"OPEN","mergeStateStatus":"DIRTY","headRefName":"z/x","statusCheckRollup":[{"name":"Unit Tests","conclusion":"FAILURE","detailsUrl":""}]}'
    set_bot_state COMMENTS_SINCE_JSON '[]'
    run_bot
    assert_not_contains "gh pr comment" "$(cat "$BOT_JOURNAL")" \
        "DIRTY/CONFLICTING PR gets no comment (C2)"
}

test_C3_rebase_candidate_no_comment() {
    setup_bot_mock
    set_bot_state PR_VIEW_JSON '{"state":"OPEN","mergeStateStatus":"UNSTABLE","headRefName":"z/x","statusCheckRollup":[{"name":"Integration Tests","conclusion":"FAILURE","detailsUrl":""}]}'
    set_bot_state COMMENTS_SINCE_JSON '[]'
    run_bot
    assert_not_contains "gh pr comment" "$(cat "$BOT_JOURNAL")" \
        "integration-only failure gets no comment (C3)"
}

test_C4_no_failures_no_comment() {
    setup_bot_mock
    set_bot_state PR_VIEW_JSON '{"state":"OPEN","mergeStateStatus":"UNSTABLE","headRefName":"z/x","statusCheckRollup":[{"name":"Unit Tests","conclusion":"SUCCESS","detailsUrl":""}]}'
    set_bot_state COMMENTS_SINCE_JSON '[]'
    run_bot
    assert_not_contains "gh pr comment" "$(cat "$BOT_JOURNAL")" \
        "no failing checks → no comment, fail-open (C4)"
}

test_C5_closed_pr_no_comment() {
    setup_bot_mock
    set_bot_state PR_VIEW_JSON '{"state":"CLOSED","mergeStateStatus":"UNSTABLE","headRefName":"z/x","statusCheckRollup":[{"name":"Unit Tests","conclusion":"FAILURE","detailsUrl":""}]}'
    set_bot_state COMMENTS_SINCE_JSON '[]'
    run_bot
    assert_not_contains "gh pr comment" "$(cat "$BOT_JOURNAL")" \
        "CLOSED PR gets no comment (C5)"
}

test_C6_dedup_patches_existing() {
    setup_bot_mock
    set_bot_state PR_VIEW_JSON "$ROLLUP_DRIFT"
    # Свой комментарий уже есть в окне → jq-фильтр вернёт его id (мок отдаёт
    # фикстуру как есть; бот применяет --jq через реальный gh — в моке мы
    # эмулируем уже-извлечённый id).
    set_bot_state COMMENTS_SINCE_JSON '424242'
    run_bot
    local j; j="$(cat "$BOT_JOURNAL")"
    assert_contains "api -X PATCH" "$j" "existing bot comment is PATCHed (C6)" || return 1
    assert_not_contains "gh pr comment" "$j" "no duplicate POST when comment exists (C6)"
}

test_C7_invariants_no_labels_no_close() {
    setup_bot_mock
    set_bot_state PR_VIEW_JSON "$ROLLUP_DRIFT"
    set_bot_state COMMENTS_SINCE_JSON '[]'
    run_bot
    local j; j="$(cat "$BOT_JOURNAL")"
    assert_not_contains "issue close" "$j" "bot never closes issues (C7)" || return 1
    assert_not_contains "--add-label" "$j" "bot never adds labels (C7)" || return 1
    assert_not_contains "--remove-label" "$j" "bot never removes labels (C7)" || return 1
    assert_not_contains "kanban" "$j" "bot never touches kanban (C7)"
}

run_test "C1 contract-drift → comment"          test_C1_contract_drift_comments
run_test "C2 CONFLICTING → no comment"          test_C2_conflicting_no_comment
run_test "C3 rebase-candidate → no comment"     test_C3_rebase_candidate_no_comment
run_test "C4 no failures → no comment"          test_C4_no_failures_no_comment
run_test "C5 CLOSED PR → no comment"            test_C5_closed_pr_no_comment
run_test "C6 dedup → PATCH not POST"            test_C6_dedup_patches_existing
run_test "C7 invariants (no labels/close/kanban)" test_C7_invariants_no_labels_no_close

printf '\n%s==== Summary ====%s\n' "$YEL" "$END"
printf 'total:  %d\n' "$TESTS_TOTAL"
printf '%spassed: %d%s\n' "$GRN" "$TESTS_PASSED" "$END"
if [ "$TESTS_FAILED" -gt 0 ]; then
    printf '%sfailed: %d%s\n' "$RED" "$TESTS_FAILED" "$END"
    for n in "${FAILED_NAMES[@]}"; do printf '  - %s\n' "$n"; done
    exit 1
fi
exit 0
