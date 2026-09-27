#!/bin/bash
# ============================================================================
# test_e2e_process_wait_workflow.sh — issue #2303 shipped-parser tests.
#
# Тестирует SHIPPED-функцию wait_workflow() из agent-flow-e2e-process.sh:3367
# (NESTED в main(), 4-пробельный отступ) через extract_func из
# tests/lib/lib_eval_func.sh (поддержка вложенных функций — issue #2303 + #2295).
#
# Тесты НЕ переписывают парсер локально (ретро t_cca7c074 19.08) — ассертят
# именно тот код, что крутится в проде, через stub gh + eval в subshell.
#
# Покрытие (по acceptance criteria issue #2303):
#   1. SUCCESS: conclusion=success, return 0
#   2. FAILURE: conclusion=failure (5 recheck=failure), return 1
#   3. RACE-FIX (ретро 01.09 t_32c28562): initial=failure, recheck#3=success,
#      return 0 + audit-комментарий "race detected"
#   4. GH RUN CANCEL на TIMEOUT (ретро 13.08 t_da3e0bd5): structural —
#      shipped-код содержит gh run cancel + race-safe pre-check
#   5. RUN-ID MULTI-STRING (ретро 13.08 t_e75b74d1): shipped содержит
#      sanitize "grep -oE '[0-9]+' | head -n1"
#
# Стратегия mock:
#   - Создаём $WORK_DIR/bin/gh с переменной $_MODE (success/failure/race-N),
#     управляющей счётчиком conclusion.
#   - eval_helper'ом достаём wait_workflow из shipped e2e-process.sh,
#     eval'им в subshell, подменяем PATH на $WORK_DIR/bin (выше системного).
#
# Run:
#   bash scripts/agent_flow/tests/test_e2e_process_wait_workflow.sh
# ============================================================================
set -o pipefail  # без -u (для stub heredoc с локальными vars); pipefail на test-fail

TEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$TEST_DIR/../../.." && pwd)"
TEST_LIBS_DIR="$TEST_DIR/lib"
E2E_PROCESS="$REPO_ROOT/scripts/agent_flow/agent-flow-e2e-process.sh"

# shellcheck source=lib/lib_eval_func.sh
. "$TEST_LIBS_DIR/lib_eval_func.sh"

TESTS_TOTAL=0
TESTS_PASSED=0
FAILED_NAMES=()

if [ -t 1 ]; then
    RED=$'\033[31m'; GRN=$'\033[32m'; YEL=$'\033[33m'; END=$'\033[0m'
else
    RED=''; GRN=''; YEL=''; END=''
fi

pass() {
    TESTS_TOTAL=$((TESTS_TOTAL + 1))
    TESTS_PASSED=$((TESTS_PASSED + 1))
    printf '  %s✓%s %s\n' "$GRN" "$END" "$1"
}
fail() {
    TESTS_TOTAL=$((TESTS_TOTAL + 1))
    FAILED_NAMES+=("$1")
    printf '  %s✗%s %s — %s\n' "$RED" "$END" "$1" "$2"
}
assert_eq() {
    if [ "$1" = "$2" ]; then pass "$3"
    else fail "$3" "expected: '$1' got: '$2'"
    fi
}
assert_contains() {
    if printf '%s' "$2" | grep -qF "$1"; then pass "$3"
    else fail "$3" "needle '$1' not found"
    fi
}
assert_not_contains() {
    if printf '%s' "$2" | grep -qF "$1"; then fail "$3" "needle '$1' FOUND but should not be"
    else pass "$3"
    fi
}

WORK_DIR="$(mktemp -d)"
mkdir -p "$WORK_DIR/bin"
trap 'rm -rf "$WORK_DIR"' EXIT

# ---------------------------------------------------------------------------
# Генерация stub gh с заданным сценарием conclusion
#   $1=mode      (success | failure | race3 | cancel_test)
#   $2=run_id    (число, что вернёт gh run list)
# Реализация через _STATE_FILE: каждый call пишет текущее состояние в файл,
# gh run view --json conclusion читает режим и возвращает нужное значение по
# счётчику _CALL_COUNT.
# ---------------------------------------------------------------------------
gen_stub_gh() {
    local mode="$1" run_id="$2" state_file="$3"
    cat > "$WORK_DIR/bin/gh" <<EOF
#!/bin/bash
# stub gh — mode=$mode run_id=$run_id
_state="$state_file"
concl_get() {
    # Persistent counter через state-файл (каждый вызов gh = новый процесс,
    # поэтому локальная переменная не работает)
    _n=0
    if [ -f "\$_state.counter" ]; then
        _n="\$(cat "\$_state.counter" 2>/dev/null || echo 0)"
    fi
    _n=\$((_n + 1))
    printf '%s' "\$_n" > "\$_state.counter"
    printf '%s' "\$_n" >> "\$_state.calls"
    case "$mode" in
        success)
            printf 'success'; return 0 ;;
        failure)
            printf 'failure'; return 0 ;;
        race3)
            # initial=failure (calls 1,2), recheck#3+ → success
            if [ "\$_n" -ge 3 ]; then
                printf 'success'
            else
                printf 'failure'
            fi
            return 0 ;;
        *)
            printf 'unknown'; return 1 ;;
    esac
}

# subcommand
_subcmd="\${1:-}"
shift || true

case "\$_subcmd" in
    run)
        _action="\${1:-}"; shift || true
        case "\$_action" in
            list)
                # Возвращаем run_id, всегда
                printf '%s\n' "$run_id"
                return 0 ;;
            view)
                # gh run view <rid> --repo X --json <status|conclusion> --jq .X
                _jq=""
                while [ \$# -gt 0 ]; do
                    case "\$1" in
                        --repo) shift 2 ;;
                        --json) _jq="\$2"; shift 2 ;;
                        --jq) shift 2 ;;
                        *) shift ;;
                    esac
                done
                case "\$_jq" in
                    status)
                        printf 'completed'
                        return 0 ;;
                    conclusion)
                        concl_get
                        return 0 ;;
                    *)
                        printf '%s' ''
                        return 0 ;;
                esac
                ;;
            cancel)
                # gh run cancel <rid> --repo X
                printf 'cancel:%s\n' "\$*" >> "\$_state.calls"
                return 0 ;;
        esac
        ;;
    issue)
        _action="\${1:-}"; shift || true
        case "\$_action" in
            comment)
                printf 'comment:%s\n' "\$*" >> "\$_state.calls"
                return 0 ;;
        esac
        ;;
esac
return 0
EOF
    chmod +x "$WORK_DIR/bin/gh"
}

# ---------------------------------------------------------------------------
# Запуск shipped wait_workflow в подконтролируемой среде
#   $1=mode  $2=run_id
# Возвращает exit code (0/1) и пишет $state_file.calls со всеми stub-вызовами.
# ---------------------------------------------------------------------------
run_wait_workflow() {
    local mode="$1" run_id="$2" state_file="$WORK_DIR/state_${mode}"
    : > "$state_file.calls"
    : > "$state_file.counter"

    # eval_helper ловит _body и eval'ит в subshell с PATH=$WORK_DIR/bin
    (
        set +e
        # Подсовываем stub gh первым в PATH
        export PATH="$WORK_DIR/bin:$PATH"
        export GH_REPO='krikz/rob_box_project'
        export E2E_POLL_INTERVAL='1'
        # log() — shipped wait_workflow пишет через log
        log() { printf 'log:%s\n' "$*" >&2; }
        # number — wait_workflow использует в issue comment
        export number=9999

        # Достаём shipped wait_workflow через extract_func
        _body="$(extract_func "$E2E_PROCESS" wait_workflow)"
        if [ -z "$_body" ]; then
            printf 'FATAL: extract_func вернул пусто для wait_workflow\n' >&2
            exit 99
        fi
        eval "$_body"

        # Вызываем: $1=wf $2=br $3=timeout $4=label $5=min_epoch
        wait_workflow "L-Build" "z-{e2e}/test-round-1" "10" "build" "2026-01-01T00:00:00Z"
    )
}

# ===========================================================================
# Test 1: SUCCESS — conclusion=success → return 0
# ===========================================================================
echo "=== Test 1: SUCCESS — conclusion=success, return 0 ==="
gen_stub_gh "success" "12345" "$WORK_DIR/state_success"
_ec="$(run_wait_workflow success 12345; printf '%d' $?)"
assert_eq "0" "$_ec" "wait_workflow returns 0 on success conclusion"

# ===========================================================================
# Test 2: FAILURE (без race) — conclusion=failure, 5 recheck=failure → return 1
# ===========================================================================
echo ""
echo "=== Test 2: FAILURE — conclusion=failure (5 recheck=failure), return 1 ==="
gen_stub_gh "failure" "12345" "$WORK_DIR/state_failure"
_ec="$(run_wait_workflow failure 12345; printf '%d' $?)"
assert_eq "1" "$_ec" "wait_workflow returns 1 on confirmed failure"
# НЕ должно быть audit-комментария "race detected"
if [ -f "$WORK_DIR/state_failure.calls" ]; then
    assert_not_contains "race detected" "$(cat "$WORK_DIR/state_failure.calls")" "no race-detected comment on confirmed failure"
fi

# ===========================================================================
# Test 3: RACE-FIX (ретро 01.09 t_32c28562)
#   initial=failure, recheck#3=success → return 0 + audit "race detected"
# ===========================================================================
echo ""
echo "=== Test 3: RACE-FIX (ретро 01.09 t_32c28562) — initial=failure, recheck#3=success, return 0 + audit ==="
gen_stub_gh "race3" "99999" "$WORK_DIR/state_race3"
_ec="$(run_wait_workflow race3 99999; printf '%d' $?)"
assert_eq "0" "$_ec" "race-fix: returns 0 (recheck saw success at call#3+)"
# Должен быть audit-комментарий "race detected"
if [ -f "$WORK_DIR/state_race3.calls" ]; then
    _calls="$(cat "$WORK_DIR/state_race3.calls")"
    assert_contains "race detected" "$_calls" "race-detected audit comment posted"
    # issue comment был вызван (через stub gh issue comment)
    assert_contains "comment:" "$_calls" "race-fix: issue comment path was hit"
fi

# ===========================================================================
# Test 4: GH RUN CANCEL на TIMEOUT (ретро 13.08 t_da3e0bd5) — structural
# ===========================================================================
echo ""
echo "=== Test 4: GH RUN CANCEL на TIMEOUT (ретро 13.08 t_da3e0bd5) — structural sanity ==="
_body="$(extract_func "$E2E_PROCESS" wait_workflow)"
assert_contains "gh run cancel" "$_body" "wait_workflow содержит gh run cancel на TIMEOUT"
# Race-safe: проверяет текущий статус перед cancel
assert_contains "_st_now" "$_body" "wait_workflow проверяет текущий статус перед cancel (race-safe)"

# ===========================================================================
# Test 5: RUN-ID MULTI-STRING (ретро 13.08 t_e75b74d1, cobra-краш)
# ===========================================================================
echo ""
echo "=== Test 5: RUN-ID MULTI-STRING (ретро 13.08 t_e75b74d1) — sanitize ==="
_body="$(extract_func "$E2E_PROCESS" wait_workflow)"
# shipped содержит: rid="$(printf '%s' "$rid" | grep -oE '[0-9]+' | head -n1 || true)"
assert_contains "grep -oE '[0-9]+' | head -n1" "$_body" "wait_workflow содержит run_id sanitize (multi-string → first id)"

# ===========================================================================
# Test 6: extract_func correctness для wait_workflow
# ===========================================================================
echo ""
echo "=== Test 6: extract_func correctness для wait_workflow ==="
_body="$(extract_func "$E2E_PROCESS" wait_workflow)"
_lines="$(printf '%s\n' "$_body" | wc -l)"
# 90 строк по факту
if [ "$_lines" -gt 50 ] && [ "$_lines" -lt 200 ]; then
    pass "wait_workflow extract размер разумный ($_lines строк)"
else
    fail "wait_workflow extract размер" "expected 50-200, got $_lines"
fi
# Открывающая строка — с 4-пробельным отступом (nested)
_first="$(printf '%s' "$_body" | head -1)"
case "$_first" in
    *"wait_workflow() {"*)
        pass "wait_workflow extract начинается с сигнатуры (nested)" ;;
    *)
        fail "wait_workflow extract signature" "first: [$_first]" ;;
esac
# Закрывающая — с тем же 4-пробельным отступом
_last="$(printf '%s' "$_body" | tail -1)"
if [ "$_last" = "    }" ]; then
    pass "wait_workflow extract заканчивается на закрывающей } (nested)"
else
    fail "wait_workflow extract close" "last: [$_last]"
fi

# ===========================================================================
# Test 7: smoke — stub gh реально используется (run_id из stub)
# ===========================================================================
echo ""
echo "=== Test 7: stub gh подцеплен по PATH — run_id=42 проходит ==="
gen_stub_gh "success" "42" "$WORK_DIR/state_42"
_ec="$(run_wait_workflow success 42; printf '%d' $?)"
assert_eq "0" "$_ec" "wait_workflow с stub run_id=42 → returns 0"

# ===========================================================================
# Итоги
# ===========================================================================
echo ""
echo "==================================="
printf 'Total: %d / Passed: %d / Failed: %d\n' "$TESTS_TOTAL" "$TESTS_PASSED" "$(($TESTS_TOTAL - $TESTS_PASSED))"
if [ "$TESTS_PASSED" -eq "$TESTS_TOTAL" ]; then
    printf '%sALL TESTS PASSED.%s\n' "$GRN" "$END"
    exit 0
fi
printf '%sFAILED TESTS:%s\n' "$RED" "$END"
for n in "${FAILED_NAMES[@]}"; do
    printf '  - %s\n' "$n"
done
exit 1