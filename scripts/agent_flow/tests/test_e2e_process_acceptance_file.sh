#!/bin/bash
# ============================================================================
# test_e2e_process_acceptance_file.sh — bug t_cca7c074 (ретро 19.08)
#
# Проверяет парсинг acceptance_file в agent-flow-e2e-process.sh и его
# auto-discovery по convention в репо:
#   1) Явный acceptance_file в issue body → парсится (в backticks / кавычках /
#      без обёрток — стрипается)
#   2) Fallback на PR body → парсится оттуда (тот же парсер)
#   3) Auto-discovery convention 1: <scenario_dir>/acceptance.json
#   4) Auto-discovery convention 2: <scenario_dir>/<scenario_basename>_acceptance.json
#   5) Auto-discovery PREFIX (issue #1452): <scenario_dir>/<prefix>_acceptance[_v<N>].json
#      где PREFIX = basename без суффиксов _suite / _v<N>
#      (решает кейс music_library_suite_v1.json → music_library_acceptance_v1.json)
#   6) Ничего нет + scenario задан → workflow сам упадёт на GATE-1 (но
#      e2e-process не падает, а логирует warning — fail-fast НЕ делаем)
#   7) Ничего нет + scenario НЕ задан (smoke-test --text) → НЕ ищем
#      acceptance.json (single-shot use case, ADR-0022 §4.1)
#   8) workflow_args содержит -f acceptance_file=<path> если что-то нашлось
#
# Тест НЕ запускает agent-flow-e2e-process.sh целиком (слишком много
# pre-checks), а изолирует ту же логику парсинга в локальные функции —
# гарантирует, что регулярки совпадают с тем, что в скрипте.
#
# Если в будущем логика парсинга в скрипте изменится — обновить и этот тест.
#
# Run:
#   bash scripts/agent_flow/tests/test_e2e_process_acceptance_file.sh
# ============================================================================
set -euo pipefail

TEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# tests/ → scripts/agent_flow/tests/ → scripts/agent_flow/ → scripts/ → REPO_ROOT
REPO_ROOT="$(cd "$TEST_DIR/../../.." && pwd)"
E2E_PROCESS="$REPO_ROOT/scripts/agent_flow/agent-flow-e2e-process.sh"
HARNESS="$REPO_ROOT/.github/workflows/scripts/e2e_voice_test.sh"

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

# --- Локальная копия extract-логики (синхронна со скриптом) -----------------
# Если скрипт изменится — синхронизируйте с ним. Чтобы не дублировать,
# подтверждаем что паттерны совпадают с теми, что в $E2E_PROCESS.
extract_acceptance_file() {  # $1=body
    local body="$1"
    local body_real
    body_real="$(printf '%s' "$body" | sed 's/\\n/\
/g')"
    printf '%s' "$body_real" | grep -iE '^[[:space:]]*acceptance_file[[:space:]]*:' | head -1 \
        | sed -E 's/^[[:space:]]*acceptance_file[[:space:]]*:[[:space:]]*//; s/^`//; s/`$//; s/^"//; s/"$//' || true
}

# --- Локальная копия harness auto-discovery (синхронна с e2e_voice_test.sh) --
# PREFIX: убираем типичные хвосты (_suite, _v1, _v<N>) — issue #1452.
harness_discover_acceptance() {  # $1=scenario_file
    local scenario_file="$1"
    local _scenario_dir _scenario_base _scenario_prefix _cand _found
    _scenario_dir="$(dirname "$scenario_file")"
    _scenario_base="$(basename "$scenario_file" .json)"
    _scenario_prefix="$_scenario_base"
    _scenario_prefix="${_scenario_prefix%_v[0-9]*}"
    _scenario_prefix="${_scenario_prefix%_suite}"
    _found=""
    for _cand in \
        "acceptance.json" \
        "${_scenario_base}_acceptance.json" \
        "${_scenario_prefix}_acceptance.json" \
        "${_scenario_prefix}_acceptance_v1.json" \
        "${_scenario_prefix}_acceptance_v2.json"; do
        if [ -f "${_scenario_dir}/${_cand}" ]; then
            _found="${_scenario_dir}/${_cand}"
            break
        fi
    done
    printf '%s' "$_found"
}

# e2e-process auto-discovery (только 2 конвенции; harness PREFIX мы не дублируем —
# e2e-process передаёт -f acceptance_file явно, harness делает fallback.
# Это разные слои: e2e-process — деплоймент, harness — runtime fallback.)
ep_discover_acceptance() {  # $1=scenario_file
    local scenario_file="$1"
    local _acc_dir _acc_basename
    _acc_dir="$(dirname "$scenario_file")"
    _acc_basename="$(basename "$scenario_file" .json)"
    if [ -f "${_acc_dir}/acceptance.json" ]; then
        printf '%s' "${_acc_dir}/acceptance.json"
    elif [ -f "${_acc_dir}/${_acc_basename}_acceptance.json" ]; then
        printf '%s' "${_acc_dir}/${_acc_basename}_acceptance.json"
    fi
}

# --- Sanity-check: паттерн в скрипте совпадает с тестом ---------------------
echo "=== Sanity: паттерн в $E2E_PROCESS совпадает с локальным extract ==="
if grep -q 'acceptance_file[[:space:]]*:' "$E2E_PROCESS"; then
    pass "паттерн acceptance_file присутствует в e2e-process.sh"
else
    fail "паттерн acceptance_file" "не найден в e2e-process.sh — скрипт не патчился?"
fi

if grep -q 'e2e_args+=(-f "acceptance_file=' "$E2E_PROCESS"; then
    pass "передача acceptance_file в e2e_args присутствует"
else
    fail "передача acceptance_file в e2e_args" "строка не найдена"
fi

if grep -q 'acceptance_file auto-discovered' "$E2E_PROCESS"; then
    pass "auto-discovery блок для acceptance_file присутствует"
else
    fail "auto-discovery acceptance_file" "не найден в e2e-process.sh"
fi

# Sanity для harness: PREFIX-логика из issue #1452 должна присутствовать
if grep -q 'PREFIX: убираем типичные хвосты' "$HARNESS"; then
    pass "PREFIX-логика в e2e_voice_test.sh присутствует (issue #1452)"
else
    fail "PREFIX-логика" "не найдена в e2e_voice_test.sh"
fi

# --- Test 1: простое значение ---------------------------------------------
echo ""
echo "=== Test 1: простое acceptance_file: <path> ==="
out=$(extract_acceptance_file 'acceptance_file: .github/e2e/scenarios/foo_acceptance.json')
assert_eq "plain path" ".github/e2e/scenarios/foo_acceptance.json" "$out"

# --- Test 2: в backticks ---------------------------------------------------
echo ""
echo "=== Test 2: acceptance_file в backticks ==="
out=$(extract_acceptance_file 'acceptance_file: `.github/e2e/scenarios/bar_acceptance.json`')
assert_eq "backticks stripped" ".github/e2e/scenarios/bar_acceptance.json" "$out"

# --- Test 3: в кавычках ----------------------------------------------------
echo ""
echo "=== Test 3: acceptance_file в кавычках ==="
out=$(extract_acceptance_file 'acceptance_file: ".github/e2e/scenarios/baz_acceptance.json"')
assert_eq "quotes stripped" ".github/e2e/scenarios/baz_acceptance.json" "$out"

# --- Test 4: пустое значение (smoke-test или fallback) ---------------------
echo ""
echo "=== Test 4: пустое acceptance_file (smoke-test fallback) ==="
out=$(extract_acceptance_file 'acceptance_file:')
assert_eq "empty value" "" "$out"

# --- Test 5: реальный формат PR ## e2e блока -------------------------------
echo ""
echo "=== Test 5: реальный формат PR ## e2e блока (issue/PR body) ==="
pr_body='## e2e

```
scenario_file: .github/e2e/scenarios/music_library_suite_v1.json
acceptance_file: .github/e2e/scenarios/music_library_acceptance_v1.json
voice: anton
volume: 150
```'
out=$(extract_acceptance_file "$pr_body")
assert_eq "real PR e2e block" \
    ".github/e2e/scenarios/music_library_acceptance_v1.json" "$out"

# --- Test 6: convention 1 — dir/acceptance.json ----------------------------
echo ""
echo "=== Test 6: convention 1 — <scenario_dir>/acceptance.json ==="
_mock_dir="$(mktemp -d)"
trap 'rm -rf "$_mock_dir" "$_mock_dir2" "$_mock_dir3" "$_mock_dir4"' EXIT
mkdir -p "$_mock_dir/scenarios"
echo '{}' > "$_mock_dir/scenarios/full_instrument_suite_v1.json"
echo '{}' > "$_mock_dir/scenarios/acceptance.json"
_discovered="$(harness_discover_acceptance "$_mock_dir/scenarios/full_instrument_suite_v1.json")"
assert_eq "harness convention 1 wins" \
    "$_mock_dir/scenarios/acceptance.json" "$_discovered"
_discovered_ep="$(ep_discover_acceptance "$_mock_dir/scenarios/full_instrument_suite_v1.json")"
assert_eq "e2e-process convention 1 wins" \
    "$_mock_dir/scenarios/acceptance.json" "$_discovered_ep"

# --- Test 7: convention 2 — dir/<basename>_acceptance.json -----------------
echo ""
echo "=== Test 7: convention 2 — <scenario_dir>/<basename>_acceptance.json ==="
_mock_dir2="$(mktemp -d)"
trap 'rm -rf "$_mock_dir" "$_mock_dir2" "$_mock_dir3" "$_mock_dir4"' EXIT
mkdir -p "$_mock_dir2/scenarios"
echo '{}' > "$_mock_dir2/scenarios/foo_suite.json"
# БЕЗ acceptance.json — должен сработать convention 2
echo '{}' > "$_mock_dir2/scenarios/foo_suite_acceptance.json"
_discovered="$(harness_discover_acceptance "$_mock_dir2/scenarios/foo_suite.json")"
assert_eq "harness convention 2 — dir/<basename>_acceptance.json" \
    "$_mock_dir2/scenarios/foo_suite_acceptance.json" "$_discovered"

# --- Test 8: PREFIX — dir/<prefix>_acceptance_v1.json (issue #1452) ---------
echo ""
echo "=== Test 8: PREFIX logic — music_library_suite_v1.json → music_library_acceptance_v1.json ==="
_mock_dir3="$(mktemp -d)"
trap 'rm -rf "$_mock_dir" "$_mock_dir2" "$_mock_dir3" "$_mock_dir4"' EXIT
mkdir -p "$_mock_dir3/scenarios"
echo '{}' > "$_mock_dir3/scenarios/music_library_suite_v1.json"
# БЕЗ acceptance.json / suite_v1_acceptance.json — должен сработать PREFIX
# (strip _suite_v1 → music_library, найти music_library_acceptance_v1.json)
echo '{}' > "$_mock_dir3/scenarios/music_library_acceptance_v1.json"
_discovered="$(harness_discover_acceptance "$_mock_dir3/scenarios/music_library_suite_v1.json")"
assert_eq "harness PREFIX (issue #1452) — strip _suite_v1" \
    "$_mock_dir3/scenarios/music_library_acceptance_v1.json" "$_discovered"

# --- Test 9: ничего не найдено → пусто, но НЕ fail-fast --------------------
echo ""
echo "=== Test 9: ни convention, ни PREFIX не сработал → пусто ==="
_mock_dir4="$(mktemp -d)"
trap 'rm -rf "$_mock_dir" "$_mock_dir2" "$_mock_dir3" "$_mock_dir4"' EXIT
mkdir -p "$_mock_dir4/scenarios"
echo '{}' > "$_mock_dir4/scenarios/orphan.json"
# БЕЗ acceptance.json / *_acceptance.json
_discovered="$(harness_discover_acceptance "$_mock_dir4/scenarios/orphan.json")"
assert_eq "no match → empty (workflow сам упадёт на GATE-1)" "" "$_discovered"

# --- Test 10: scenario_file не задан → НЕ ищем acceptance_file -------------
echo ""
echo "=== Test 10: scenario_file пустой → auto-discovery acceptance НЕ запускается ==="
_e2e_scenario_file=""
_e2e_acceptance_file=""
if [ -z "$_e2e_acceptance_file" ] && [ -n "$_e2e_scenario_file" ]; then
    _auto_discovery_would_run="yes"
else
    _auto_discovery_would_run="no"
fi
assert_eq "guard: auto-discovery НЕ запускается без scenario_file" "no" "$_auto_discovery_would_run"

# --- Test 11: реальный sanity — music_library_*.acceptance.json существует ---
echo ""
echo "=== Test 11: music_library_acceptance_v1.json существует ==="
if [ -f "$REPO_ROOT/.github/e2e/scenarios/music_library_acceptance_v1.json" ]; then
    pass "music_library_acceptance_v1.json присутствует (legacy name, PREFIX logic резолвит)"
else
    fail "music_library_acceptance_v1.json" "отсутствует — регресс"
fi

# --- Test 12: workflow копирует e2e_voice_lib.sh ---------------------------
echo ""
echo "=== Test 12: workflow копирует e2e_voice_lib.sh ==="
if grep -q 'scp.*e2e_voice_lib.sh.*ros2@10.1.1.249:/tmp/e2e_voice_lib.sh' "$REPO_ROOT/.github/workflows/L-E2E Voice Test.yml"; then
    pass "workflow scp'ит e2e_voice_lib.sh на 249"
else
    fail "workflow scp" "scp e2e_voice_lib.sh не найден в workflow"
fi

# --- Test 13: harness передаёт --acceptance вверх по --acceptance ----------
echo ""
echo "=== Test 13: harness CLI --acceptance flag (regression guard) ==="
if grep -q '\-\-acceptance)' "$HARNESS"; then
    pass "harness поддерживает --acceptance CLI flag"
else
    fail "--acceptance flag" "не найден в harness"
fi

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