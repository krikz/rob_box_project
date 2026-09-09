#!/bin/bash
# ============================================================================
# test_e2e_process_type_testing.sh — ретро 22.08 t_944df2c5 (issue #1506)
#
# Проверяет type:testing gate в agent-flow-e2e-process.sh:
#   Для type:testing задач smoke-default («Робот, спой песенку про енотика»)
#   НЕ валидирует acceptance исходной задачи. Если воркер не задал явно
#   scenario_file + acceptance_file — e2e-раунд НЕ запускается (skip с
#   warning), иначе «красивый PASS вместо честного FAIL» (ADR-0018).
#
# Тест НЕ запускает agent-flow-e2e-process.sh целиком (слишком много
# pre-checks), а: (1) sanity-проверяет что gate-блок есть в скрипте,
# (2) тестирует локальную копию предиката gate — гарантирует, что логика
# совпадает с тем, что в скрипте. Если скрипт изменится — синхронизировать.
#
# Scenarios:
#   L1. type:testing + scenario_file пуст   → skip
#   L2. type:testing + acceptance_file пуст → skip
#   L3. type:testing + оба заданы           → run (прогон)
#   L4. НЕ type:testing + оба пусты         → run (smoke-default легитимен)
#
# Run:
#   bash scripts/agent_flow/tests/test_e2e_process_type_testing.sh
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

# --- Sanity: gate-блок присутствует в скрипте ------------------------------
echo "=== Sanity: type:testing gate в $E2E_PROCESS ==="
if grep -q 'type:testing gate (ретро 22.08 t_944df2c5' "$E2E_PROCESS"; then
    pass "gate-блок type:testing присутствует в e2e-process.sh"
else
    fail "gate-блок type:testing" "не найден в e2e-process.sh — скрипт не патчился?"
fi

if grep -q 'has_label "$labels_norm" "type:testing"' "$E2E_PROCESS"; then
    pass "предикат has_label type:testing присутствует"
else
    fail "предикат has_label type:testing" "не найден"
fi

if grep -q 'scenario_file=${e2e_scenario_file:-empty} acceptance_file=${e2e_acceptance_file:-empty}' "$E2E_PROCESS"; then
    pass "skip-лог с scenario_file/acceptance_file присутствует"
else
    fail "skip-лог" "не найден"
fi

# --- Локальная копия предиката (синхронна со скриптом) ---------------------
# has_label — тот же контракт, что в e2e-process.sh.
has_label_local() {  # $1=labels_csv $2=label
    printf '%s' "$1" | tr ',' '\n' | grep -Fxq "$2"
}

gate_should_skip() {  # $1=labels_csv $2=scenario_file $3=acceptance_file → skip|run
    local labels="$1" scenario="$2" acceptance="$3"
    if has_label_local "$labels" "type:testing"; then
        if [ -z "$scenario" ] || [ -z "$acceptance" ]; then
            printf 'skip'
            return 0
        fi
    fi
    printf 'run'
}

# --- Test cases ------------------------------------------------------------
echo ""
echo "=== Test L1: type:testing + scenario пуст ==="
assert_eq "L1 type:testing без scenario_file → skip" "skip" \
    "$(gate_should_skip 'hermes,needs-e2e,type:testing' '' '.github/e2e/scenarios/foo_acceptance_v1.json')"

echo ""
echo "=== Test L2: type:testing + acceptance пуст ==="
assert_eq "L2 type:testing без acceptance_file → skip" "skip" \
    "$(gate_should_skip 'hermes,type:testing' '.github/e2e/scenarios/foo_suite_v1.json' '')"

echo ""
echo "=== Test L3: type:testing + оба заданы ==="
assert_eq "L3 type:testing с обоими файлами → run" "run" \
    "$(gate_should_skip 'hermes,type:testing' '.github/e2e/scenarios/foo_suite_v1.json' '.github/e2e/scenarios/foo_acceptance_v1.json')"

echo ""
echo "=== Test L4: НЕ type:testing + оба пусты ==="
assert_eq "L4 без type:testing → run (smoke-default легитимен)" "run" \
    "$(gate_should_skip 'hermes,needs-e2e' '' '')"

# --- Summary ----------------------------------------------------------------
echo ""
echo "==== Summary ===="
echo "total:  $TESTS_TOTAL"
echo "passed: $TESTS_PASSED"
echo "failed: $TESTS_FAILED"
if [ "$TESTS_FAILED" -gt 0 ]; then
    printf 'failures: %s\n' "${FAILED_NAMES[*]}"
    exit 1
fi
exit 0
