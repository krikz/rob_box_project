#!/bin/bash
# ============================================================================
# test_e2e_process_fail_streak_sweep.sh — smoke-тест (ретро 28.08 t_4ead2dd4)
#
# Проверяет G2.7: auto-escalation needs-review лейбла при fail-streak ≥ N.
# Сценарии (без реального gh API — моки через AUTO_NEEDS_REVIEW_TEST_MODE):
#   A. streak=5 + PR clean+raw-evidence+no-label → должен пометить (dry-run log)
#   B. streak=4 → sweep no-op (streak < threshold)
#   C. streak=5 + PR с mergeStateStatus=DIRTY → skip (CI не зелёный)
#   D. streak=5 + PR с Raw-evidence + уже needs-review → skip (idempotent)
#   E. streak=5 + PR без Raw-evidence в body → skip (нет evidence)
#   F. streak=5 + PR с e2e-done → skip (уже завершён)
#   G. streak=5 + PR с user-unlabel на needs-review → skip (Q22 respect)
#   H. compute_e2e_fail_streak: 5 failures + 1 success (oldest) → streak=5,
#      last_success=<that run>. 7 failures + 2 successes (older last) → streak=7.
#   I. Sanity: все три функции присутствуют в скрипте.
#
# Run:
#   bash scripts/agent_flow/tests/test_e2e_process_fail_streak_sweep.sh
# ============================================================================
set -euo pipefail

TEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$TEST_DIR/../../.." && pwd)"
E2E_PROCESS="$REPO_ROOT/scripts/agent_flow/agent-flow-e2e-process.sh"

# --- Helpers --------------------------------------------------------------
TESTS_TOTAL=0
TESTS_PASSED=0
TESTS_FAILED=0
FAILED_NAMES=()

if [ -t 1 ]; then
    RED=$'\033[31m'; GRN=$'\033[32m'; YEL=$'\033[33m'; END=$'\033[0m'
else
    RED=''; GRN=''; YEL=''; END=''
fi

pass() {
    local name="$1"
    TESTS_TOTAL=$((TESTS_TOTAL+1))
    TESTS_PASSED=$((TESTS_PASSED+1))
    printf '  %s✓%s %s\n' "$GRN" "$END" "$name"
}
fail() {
    local name="$1" reason="$2"
    TESTS_TOTAL=$((TESTS_TOTAL+1))
    TESTS_FAILED=$((TESTS_FAILED+1))
    FAILED_NAMES+=("$name")
    printf '  %s✗%s %s — %s\n' "$RED" "$END" "$name" "$reason"
}
assert_eq() {
    local name="$1" expected="$2" actual="$3"
    if [ "$expected" = "$actual" ]; then
        pass "$name"
    else
        fail "$name" "expected: '$expected', got: '$actual'"
    fi
}
assert_contains() {
    local name="$1" needle="$2" haystack="$3"
    if printf '%s' "$haystack" | grep -qF -- "$needle"; then
        pass "$name"
    else
        fail "$name" "expected to contain '$needle', got: '$haystack'"
    fi
}
assert_not_contains() {
    local name="$1" needle="$2" haystack="$3"
    if printf '%s' "$haystack" | grep -qF -- "$needle"; then
        fail "$name" "expected NOT to contain '$needle', got: '$haystack'"
    else
        pass "$name"
    fi
}

# --- Sanity-checks --------------------------------------------------------
echo "=== Sanity: G2.7 функции в $E2E_PROCESS ==="
if grep -q '^compute_e2e_fail_streak()' "$E2E_PROCESS"; then
    pass "compute_e2e_fail_streak() определена"
else
    fail "compute_e2e_fail_streak определена" "функция не найдена"
fi
if grep -q '^list_open_prs_for_escalation()' "$E2E_PROCESS"; then
    pass "list_open_prs_for_escalation() определена"
else
    fail "list_open_prs_for_escalation определена" "функция не найдена"
fi
if grep -q '^fail_streak_needs_review_sweep()' "$E2E_PROCESS"; then
    pass "fail_streak_needs_review_sweep() определена"
else
    fail "fail_streak_needs_review_sweep определена" "функция не найдена"
fi
if grep -q 'AUTO_NEEDS_REVIEW_ON_FAIL_STREAK' "$E2E_PROCESS"; then
    pass "AUTO_NEEDS_REVIEW_ON_FAIL_STREAK присутствует (env)"
else
    fail "AUTO_NEEDS_REVIEW_ON_FAIL_STREAK env" "не найдено"
fi
if grep -q '^fail_streak_needs_review_sweep || true$' "$E2E_PROCESS"; then
    pass "fail_streak_needs_review_sweep вызывается в main loop"
else
    fail "вызов в main" "top-level call не найден"
fi

# --- Извлекаем функции в текущий shell ------------------------------------
# Подход: вытащим блок от маркера '# --- G2.7:' до строки 'fail_streak_needs_review_sweep || true'
# включительно (это включает все 3 функции + top-level call).
# Но для теста нам нужны ТОЛЬКО функции + log() — без top-level call.
# Используем awk чтобы вытащить ровно три функции, начинающиеся с известных имён.
extract_block() {
    awk '
        BEGIN { in_fn = 0; depth = 0 }
        # Совпадение начала одной из трёх целевых функций
        /^(compute_e2e_fail_streak|list_open_prs_for_escalation|fail_streak_needs_review_sweep)\(\) \{/ {
            in_fn = 1; depth = 0
        }
        in_fn {
            print
            # Считаем { и } в строке (грубо, но работает: у нас нет
            # вложенных { в одну строку внутри тела функции).
            opens = gsub(/\{/, "{")
            closes = gsub(/\}/, "}")
            depth += opens - closes
            if (depth == 0 && /^\}$/) {
                in_fn = 0
            }
        }
    ' "$E2E_PROCESS"
}

# Также объявим log() — он в основном скрипте, нам нужен для sweep'а
# (без него DRY-RUN лог не появится в stderr — а тест его читает).
# log() — обёртка printf в stderr (как в основном скрипте).
log() { printf '%s %s\n' "$LOG_PREFIX" "$*" >&2; }
export LOG_PREFIX="[test-fail-streak-sweep]"

# Соберём тестовый shell-контекст
_TESTS_SHELL="$(extract_block)
"

# Очистим комментарии-маркеры от вытащенных функций (они только для grep,
# не влияют на функциональность — оставляем как есть).

# shellcheck disable=SC2086
eval "$_TESTS_SHELL"

# Экспортируем функции чтобы они были доступны внутри bash -c.
export -f compute_e2e_fail_streak list_open_prs_for_escalation fail_streak_needs_review_sweep log 2>/dev/null || true

if ! declare -f compute_e2e_fail_streak >/dev/null \
   || ! declare -f list_open_prs_for_escalation >/dev/null \
   || ! declare -f fail_streak_needs_review_sweep >/dev/null; then
    echo ""
    echo "=== Итоги ==="
    echo "Всего: $TESTS_TOTAL  ${GRN}passed: $TESTS_PASSED${END}  ${RED}failed: $TESTS_FAILED${END}"
    echo "FATAL: не удалось загрузить функции"
    exit 1
fi
pass "функции загружены в тестовый shell"

# --- Моки -----------------------------------------------------------------
# User-unlabel guard внутри sweep зовёт user_removed_label_recently +
# user_unlabel_log_skip. Подменяем на заглушки, чтобы не дёргать gh api.
USER_UNLABEL_TEST_MODE=1
export USER_UNLABEL_TEST_MODE
user_removed_label_recently() {  # $1=pr $2=label
    # Берём мок из _USER_UNLABEL_TEST_JSON: "skip:pr1,pr2" = skip user-unlabel.
    local _skip_list="${_MOCK_USER_UNLABEL_SKIP:-}"
    case ",$_skip_list," in
        *",$1,"*) return 0 ;;
        *) return 1 ;;
    esac
}
user_unlabel_log_skip() { printf 'MOCK user_unlabel_log_skip: %s %s\n' "$1" "$2" >&2; }
export -f user_removed_label_recently user_unlabel_log_skip

# --- Конструкторы моков ---------------------------------------------------
# mock_runs_json <count_of_failures> [<last_success_iso>]
#   Возвращает JSON: <count> failure-runs + (если есть last_success_iso) 1 success.
#   newest-first (как gh run list).
mock_runs_json() {  # $1=streak $2=last_success_iso(optional)
    local streak="$1" last_success="${2:-}"
    local first=1
    printf '['
    if [ -n "$last_success" ]; then
        first=0
        printf '{"databaseId":1,"conclusion":"success","createdAt":"%s","headBranch":"main"}' "$last_success"
    fi
    local i
    for i in $(seq 1 "$streak"); do
        if [ "$first" = "0" ]; then printf ','; fi
        first=0
        printf '{"databaseId":%d,"conclusion":"failure","createdAt":"2026-08-28T10:0%d:00Z","headBranch":"z-{e2e}/test-round-99"}' "$((i+10))" "$i"
    done
    printf ']'
}

# mock_pr_json <number> <headRefName> <body> <labels_csv> <mergeStateStatus>
mock_pr_json() {  # $1=n $2=head $3=body $4=labels_csv $5=mss
    local n="$1" head="$2" body="$3" labels_csv="$4" mss="$5"
    # Escape body (replace " with \") — для простых тестов body не содержит кавычек.
    local escaped_body="${body//\"/\\\"}"
    local first=1
    local labels_arr=""
    if [ -n "$labels_csv" ]; then
        labels_arr=""
        local OLD_IFS="$IFS"; IFS=','
        for l in $labels_csv; do
            if [ "$first" = "0" ]; then labels_arr="${labels_arr},"; fi
            first=0
            labels_arr="${labels_arr}{\"name\":\"$l\"}"
        done
        IFS="$OLD_IFS"
    fi
    printf '{"number":%d,"title":"PR #%d","body":"%s","labels":[%s],"headRefName":"%s","baseRefName":"develop","mergeStateStatus":"%s","state":"OPEN"}' \
        "$n" "$n" "$escaped_body" "$labels_arr" "$head" "$mss"
}

# --- Test H: compute_e2e_fail_streak edge cases ---------------------------
echo ""
echo "=== Test H: compute_e2e_fail_streak edge cases ==="

run_streak_test() {  # $1=name $2=runs_json $3=expected_streak
    local name="$1" runs="$2" expected="$3"
    AUTO_NEEDS_REVIEW_TEST_MODE=1 \
    _AUTO_NEEDS_REVIEW_TEST_RUNS_JSON="$runs" \
    _AUTO_NEEDS_REVIEW_TEST_PRS_JSON='[]' \
    bash -c '
        eval '"$(declare -f compute_e2e_fail_streak | sed -e "s/^/        /" -e "s/^        //" | head -n -1)"'
        export -f compute_e2e_fail_streak
        compute_e2e_fail_streak
    ' 2>/dev/null
}

# H1: 5 failures → streak=5, last_success="""
_h_runs="$(mock_runs_json 5)"
_h_out="$(AUTO_NEEDS_REVIEW_TEST_MODE=1 \
    _AUTO_NEEDS_REVIEW_TEST_RUNS_JSON="$_h_runs" \
    _AUTO_NEEDS_REVIEW_TEST_PRS_JSON='[]' \
    bash -c 'eval "$(declare -f compute_e2e_fail_streak)"; export -f compute_e2e_fail_streak; compute_e2e_fail_streak' 2>/dev/null)"
_h_streak="${_h_out%%|*}"
_h_last="${_h_out#*|}"
assert_eq "H1: 5 failures → streak=5" "5" "$_h_streak"
assert_eq "H1: 5 failures → last_success='' " "" "$_h_last"

# H2: 3 failures + 1 success (older) → streak=3, last_success=2026-08-28T08:00:00Z
_h_runs='[{"databaseId":1,"conclusion":"failure","createdAt":"2026-08-28T09:00:00Z","headBranch":"z-{e2e}/test-round-200"},
{"databaseId":2,"conclusion":"failure","createdAt":"2026-08-28T08:30:00Z","headBranch":"z-{e2e}/test-round-200"},
{"databaseId":3,"conclusion":"failure","createdAt":"2026-08-28T08:10:00Z","headBranch":"z-{e2e}/test-round-200"},
{"databaseId":4,"conclusion":"success","createdAt":"2026-08-28T08:00:00Z","headBranch":"main"}]'
_h_out="$(AUTO_NEEDS_REVIEW_TEST_MODE=1 \
    _AUTO_NEEDS_REVIEW_TEST_RUNS_JSON="$_h_runs" \
    _AUTO_NEEDS_REVIEW_TEST_PRS_JSON='[]' \
    bash -c 'eval "$(declare -f compute_e2e_fail_streak)"; export -f compute_e2e_fail_streak; compute_e2e_fail_streak' 2>/dev/null)"
_h_streak="${_h_out%%|*}"
_h_last="${_h_out#*|}"
assert_eq "H2: 3 failures + success → streak=3" "3" "$_h_streak"
assert_eq "H2: 3 failures + success → last_success=2026-08-28T08:00:00Z" "2026-08-28T08:00:00Z" "$_h_last"

# H3: пустой массив → streak=0
_h_out="$(AUTO_NEEDS_REVIEW_TEST_MODE=1 \
    _AUTO_NEEDS_REVIEW_TEST_RUNS_JSON='[]' \
    _AUTO_NEEDS_REVIEW_TEST_PRS_JSON='[]' \
    bash -c 'eval "$(declare -f compute_e2e_fail_streak)"; export -f compute_e2e_fail_streak; compute_e2e_fail_streak' 2>/dev/null)"
_h_streak="${_h_out%%|*}"
assert_eq "H3: empty runs → streak=0" "0" "$_h_streak"

# H4: in_progress runs — НЕ считаются как failure, streak=0
_h_runs='[{"databaseId":1,"conclusion":"","createdAt":"2026-08-28T09:00:00Z","headBranch":"z-{e2e}/test-round-200"}]'
_h_out="$(AUTO_NEEDS_REVIEW_TEST_MODE=1 \
    _AUTO_NEEDS_REVIEW_TEST_RUNS_JSON="$_h_runs" \
    _AUTO_NEEDS_REVIEW_TEST_PRS_JSON='[]' \
    bash -c 'eval "$(declare -f compute_e2e_fail_streak)"; export -f compute_e2e_fail_streak; compute_e2e_fail_streak' 2>/dev/null)"
_h_streak="${_h_out%%|*}"
assert_eq "H4: in_progress → streak=0 (not failure)" "0" "$_h_streak"

# --- Сценарии A-G ----------------------------------------------------------
# Все сценарии используют sweep в DRY-RUN (через DRY_RUN=true), чтобы не звать
# gh pr edit. Нам важно проверить, что sweep ПРАВИЛЬНО фильтрует кандидатов
# (что попадает в log "would: gh pr edit" vs что скипается).

# Helper: запустить sweep с моками и threshold, поймать stdout/stderr.
#   $1=name $2=expected_to_label_csv $3=runs_json $4=prs_json $5=threshold
#   $6=_MOCK_USER_UNLABEL_SKIP (опционально)
run_sweep_scenario() {
    local name="$1" expected="$2" runs="$3" prs="$4" threshold="$5" skip_users="${6:-}"
    # Соберём python-checker в python: какие PR попали бы под фильтр.
    # Используем тот же код, что в sweep, через подмену log() и DRY_RUN=true.
    AUTO_NEEDS_REVIEW_TEST_MODE=1 \
    AUTO_NEEDS_REVIEW_DRY_RUN=true \
    AUTO_NEEDS_REVIEW_ON_FAIL_STREAK="$threshold" \
    _AUTO_NEEDS_REVIEW_TEST_RUNS_JSON="$runs" \
    _AUTO_NEEDS_REVIEW_TEST_PRS_JSON="$prs" \
    _MOCK_USER_UNLABEL_SKIP="$skip_users" \
    DRY_RUN=true \
    bash -c '
        eval "$(declare -f fail_streak_needs_review_sweep compute_e2e_fail_streak list_open_prs_for_escalation log user_removed_label_recently user_unlabel_log_skip)"
        export -f fail_streak_needs_review_sweep compute_e2e_fail_streak list_open_prs_for_escalation log user_removed_label_recently user_unlabel_log_skip
        fail_streak_needs_review_sweep
    ' 2>&1
}

# --- Test A: streak=5, PR clean+raw-evidence+no-label → должен пометить ---
echo ""
echo "=== Test A: streak=5, PR clean+raw-evidence+no-label → label ==="
_a_prs='['"$(mock_pr_json 1720 "z-devops/t_a-fix" "## Raw-evidence\npytest ok" "" "CLEAN")"']'
_a_out="$(run_sweep_scenario "A" "1720" "$(mock_runs_json 5)" "$_a_prs" 5)"
assert_contains "A: PR #1720 попал в would-label (gh pr edit 1720)" "gh pr edit 1720" "$_a_out"

# --- Test B: streak=4 → no-op ---
echo ""
echo "=== Test B: streak=4 (below threshold) → no-op ==="
_b_prs='['"$(mock_pr_json 1721 "z-devops/t_b-fix" "## Raw-evidence" "" "CLEAN")"']'
_b_out="$(run_sweep_scenario "B" "" "$(mock_runs_json 4)" "$_b_prs" 5)"
assert_not_contains "B: PR #1721 НЕ попал в would-label (streak<5)" "PR #1721" "$_b_out"

# --- Test C: streak=5, PR mergeStateStatus=DIRTY → skip ---
echo ""
echo "=== Test C: streak=5, PR mergeStateStatus=DIRTY → skip ==="
_c_prs='['"$(mock_pr_json 1723 "z-devops/t_c-fix" "## Raw-evidence" "" "DIRTY")"']'
_c_out="$(run_sweep_scenario "C" "" "$(mock_runs_json 5)" "$_c_prs" 5)"
assert_not_contains "C: PR #1723 НЕ попал (DIRTY)" "PR #1723" "$_c_out"

# --- Test D: streak=5, PR с уже needs-review → skip (idempotent) ---
echo ""
echo "=== Test D: streak=5, PR already has needs-review → skip ==="
_d_prs='['"$(mock_pr_json 1725 "z-devops/t_d-fix" "## Raw-evidence" "needs-review" "CLEAN")"']'
_d_out="$(run_sweep_scenario "D" "" "$(mock_runs_json 5)" "$_d_prs" 5)"
assert_not_contains "D: PR #1725 НЕ попал (already labeled)" "PR #1725" "$_d_out"

# --- Test E: streak=5, PR без Raw-evidence в body → skip ---
echo ""
echo "=== Test E: streak=5, PR без Raw-evidence в body → skip ==="
_e_prs='['"$(mock_pr_json 1727 "z-devops/t_e-fix" "просто описание PR без evidence-раздела" "" "CLEAN")"']'
_e_out="$(run_sweep_scenario "E" "" "$(mock_runs_json 5)" "$_e_prs" 5)"
assert_not_contains "E: PR #1727 НЕ попал (нет Raw-evidence)" "PR #1727" "$_e_out"

# --- Test F: streak=5, PR с e2e-done → skip ---
echo ""
echo "=== Test F: streak=5, PR с e2e-done → skip ==="
_f_prs='['"$(mock_pr_json 1729 "z-devops/t_f-fix" "## Raw-evidence" "e2e-done" "CLEAN")"']'
_f_out="$(run_sweep_scenario "F" "" "$(mock_runs_json 5)" "$_f_prs" 5)"
assert_not_contains "F: PR #1729 НЕ попал (e2e-done)" "PR #1729" "$_f_out"

# --- Test G: streak=5, PR с user-unlabel guard → skip ---
echo ""
echo "=== Test G: streak=5, PR с user-unlabel guard → skip ==="
_g_prs='['"$(mock_pr_json 1731 "z-devops/t_g-fix" "## Raw-evidence" "" "CLEAN")"']'
_g_out="$(run_sweep_scenario "G" "" "$(mock_runs_json 5)" "$_g_prs" 5 "1731")"
assert_not_contains "G: PR #1731 НЕ попал (user-unlabel)" "PR #1731" "$_g_out"
assert_contains "G: user_unlabel_log_skip залогирован" "user_unlabel_log_skip" "$_g_out"

# --- Test J: смешанный набор — 3 PR, разные исходы ---
echo ""
echo "=== Test J: mixed PRs (3 кандидата: 1 ok + 2 skip) ==="
_j_prs='[
'"$(mock_pr_json 1800 "z-devops/t_j1" "## Raw-evidence" "" "CLEAN")"',
'"$(mock_pr_json 1801 "z-devops/t_j2" "## Raw-evidence" "" "DIRTY")"',
'"$(mock_pr_json 1802 "z-devops/t_j3" "просто PR" "" "CLEAN")"'
]'
_j_out="$(run_sweep_scenario "J" "1800" "$(mock_runs_json 5)" "$_j_prs" 5)"
assert_contains "J: PR #1800 (clean+raw+no-label) попал (gh pr edit 1800)" "gh pr edit 1800" "$_j_out"
assert_not_contains "J: PR #1801 (DIRTY) НЕ попал" "gh pr edit 1801" "$_j_out"
assert_not_contains "J: PR #1802 (no raw) НЕ попал" "gh pr edit 1802" "$_j_out"

# --- Test K: AUTO_NEEDS_REVIEW_ON_FAIL_STREAK=0 → sweep отключён ---
echo ""
echo "=== Test K: AUTO_NEEDS_REVIEW_ON_FAIL_STREAK=0 → sweep disabled ==="
_k_prs='['"$(mock_pr_json 1810 "z-devops/t_k" "## Raw-evidence" "" "CLEAN")"']'
# Если threshold=0 — sweep возвращает 0 сразу, никаких API-вызовов.
# Проверяем что в выводе НЕТ 'needs-review sweep: streak=...' (что было бы
# после compute_e2e_fail_streak).
_k_out="$(AUTO_NEEDS_REVIEW_TEST_MODE=1 \
    AUTO_NEEDS_REVIEW_DRY_RUN=true \
    AUTO_NEEDS_REVIEW_ON_FAIL_STREAK=0 \
    _AUTO_NEEDS_REVIEW_TEST_RUNS_JSON="$(mock_runs_json 5)" \
    _AUTO_NEEDS_REVIEW_TEST_PRS_JSON="$_k_prs" \
    DRY_RUN=true \
    bash -c '
        eval "$(declare -f fail_streak_needs_review_sweep compute_e2e_fail_streak list_open_prs_for_escalation log user_removed_label_recently user_unlabel_log_skip)"
        export -f fail_streak_needs_review_sweep compute_e2e_fail_streak list_open_prs_for_escalation log user_removed_label_recently user_unlabel_log_skip
        fail_streak_needs_review_sweep
    ' 2>&1)"
assert_not_contains "K: threshold=0 → нет log 'streak='" "streak=" "$_k_out"

# --- Итоги ----------------------------------------------------------------
echo ""
echo "=== Итоги ==="
echo "Всего: $TESTS_TOTAL  ${GRN}passed: $TESTS_PASSED${END}  ${RED}failed: $TESTS_FAILED${END}"
if [ "$TESTS_FAILED" -gt 0 ]; then
    echo ""
    echo "Провалы:"
    for n in "${FAILED_NAMES[@]}"; do
        echo "  - $n"
    done
    exit 1
fi
exit 0
