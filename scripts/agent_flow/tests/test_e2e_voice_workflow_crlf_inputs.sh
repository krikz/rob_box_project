#!/bin/bash
# ============================================================================
# test_e2e_voice_workflow_crlf_inputs.sh — ретро t_4cf2811e
#
# Проверяет что в .github/workflows/L-E2E Voice Test.yml:
#   1) Все bash-context ${{ inputs.x }} заменены на env-vars
#   2) Для каждого env-input есть sanitization через `tr -d '\r\n'` (single-line)
#      или `tr -d '\r'` (multi-line: voice_text, patterns)
#   3) Sanitization сама работает: bash-функция режет CR/LF корректно
#
# Ретро-контекст: round-166 fail на input с trailing \r (workflow_dispatch +
# jq без -r → upstream JSON с CRLF → ${{ inputs.scenario_file }} → scp
# "file.json\r" → "No such file or directory").
#
# Run:
#   bash scripts/agent_flow/tests/test_e2e_voice_workflow_crlf_inputs.sh
# ============================================================================
set -euo pipefail

TEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# tests/ → scripts/agent_flow/tests/ → scripts/agent_flow/ → scripts/ → REPO_ROOT
REPO_ROOT="$(cd "$TEST_DIR/../../.." && pwd)"
WORKFLOW="$REPO_ROOT/.github/workflows/L-E2E Voice Test.yml"

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

# --- Sanity: файл существует --------------------------------------------------
echo "=== Sanity: workflow файл ==="
if [ -f "$WORKFLOW" ]; then
    pass "workflow exists: $WORKFLOW"
else
    fail "workflow file" "не найден $WORKFLOW"
    exit 1
fi

# --- Test 1: НЕТ bare ${{ inputs.x }} в bash-контексте -----------------------
echo ""
echo "=== Test 1: bare \${{ inputs.x }} только в env: / if: / description ==="
# Извлекаем bash-блоки (строки с `run: |` до следующего `      - name:` или
# `        env:` / `        with:` / `        if:`).
# Простой grep по строкам, начинающимся НЕ с '          ' (env) и НЕ с
# `        if:` (condition) и НЕ с `          #` (комментарий внутри env).
# Грубый, но достаточный для regression.

# Найти все строки, где встречается ${{ inputs. }} и которые НЕ внутри env:/if:/description
# Подход: пройти по строкам файла, отслеживая контекст.
in_env_block=0
in_description=0
in_if_block=0
in_run_block=0
violations=()

while IFS= read -r line; do
    # Detect block transitions
    case "$line" in
        *'on:'*) in_env_block=0; in_run_block=0 ;;
        *'env:'*) in_env_block=1; in_run_block=0 ;;
        *'run: |'*) in_run_block=1; in_env_block=0 ;;
        *'if:'*) in_if_block=1; in_env_block=0; in_run_block=0 ;;
        *'description:'*) in_description=1; in_env_block=0; in_run_block=0; in_if_block=0 ;;
        *'- name:'*) in_env_block=0; in_run_block=0; in_if_block=0; in_description=0 ;;
        *'uses:'*) in_env_block=0; in_run_block=0; in_if_block=0; in_description=0 ;;
        *'with:'*) in_env_block=0; in_run_block=0; in_if_block=0; in_description=0 ;;
    esac

    # Проверяем bare ${{ inputs.x }} в bash-контексте
    if [ "$in_run_block" -eq 1 ]; then
        if printf '%s' "$line" | grep -qE '\$\{\{ inputs\.[a-z_]+\s*\}\}'; then
            violations+=("BASH: $line")
        fi
    fi
done < "$WORKFLOW"

if [ "${#violations[@]}" -eq 0 ]; then
    pass "нет bare \${{ inputs.x }} в bash-контексте"
else
    fail "bare inputs in bash" "${#violations[@]} нарушений: ${violations[*]}"
fi

# --- Test 2: env-inputs sanitized (есть CLEAN-переменные) --------------------
echo ""
echo "=== Test 2: env-inputs sanitized (есть *_CLEAN переменные) ==="
for var in SCENARIO_FILE_CLEAN ACCEPTANCE_FILE_CLEAN VOICE_TEXT_CLEAN VOICE_CLEAN PATTERNS_CLEAN RETRIES_CLEAN REACT_WINDOW_CLEAN CHECK_TG_ECHO_CLEAN PR_NUMBER_CLEAN REF_CLEAN; do
    if grep -q "${var}=" "$WORKFLOW"; then
        pass "переменная ${var} присутствует"
    else
        fail "${var}" "не найдена в workflow (sanitization отсутствует?)"
    fi
done

# --- Test 3: использован tr -d для sanitization -----------------------------
echo ""
echo "=== Test 3: sanitization через tr -d ==="
if grep -qE "tr -d '\\\\r\\\\n'" "$WORKFLOW"; then
    pass "tr -d '\\\\r\\\\n' (single-line sanitization)"
else
    fail "tr -d for single-line" "не найдено"
fi
if grep -qE "tr -d '\\\\r'" "$WORKFLOW"; then
    pass "tr -d '\\\\r' (multi-line sanitization)"
else
    fail "tr -d for multi-line" "не найдено"
fi

# --- Test 4: bash-функция sanitization работает корректно --------------------
echo ""
echo "=== Test 4: sanitization runtime (smoke-test) ==="
# Один тест на round-trip CRLF
_input=$'.github/e2e/scenarios/voice_core_suite_v1.json\r'
_expected=".github/e2e/scenarios/voice_core_suite_v1.json"
_actual=$(printf '%s' "$_input" | tr -d '\r\n')
assert_eq "single-line CRLF → clean path" "$_expected" "$_actual"

# Multi-line input (voice_text) — сохраняем LF, режем CR
_multi_input=$'Робот, спой песенку.\r\nпро енотика\r'
_multi_expected=$'Робот, спой песенку.\nпро енотика'
_multi_actual=$(printf '%s' "$_multi_input" | tr -d '\r')
assert_eq "multi-line CRLF+LF → keep LF, strip CR" "$_multi_expected" "$_multi_actual"

# --- Test 5: env-vars declared (single source of truth) ---------------------
echo ""
echo "=== Test 5: env-блоки (one source of truth для inputs) ==="
# Проверяем что inputs объявлены в env в обоих ключевых шагах
_steps_with_env=$(grep -c 'SCENARIO_FILE: \${{ inputs.scenario_file }}' "$WORKFLOW")
if [ "$_steps_with_env" -ge 2 ]; then
    pass "SCENARIO_FILE в env объявлен в $_steps_with_env шагах (ожидалось ≥2)"
else
    fail "SCENARIO_FILE env" "объявлен только в $_steps_with_env шаге (нужно ≥2)"
fi

# --- Test 6: model.json использует jq (JSON-safe) ---------------------------
echo ""
echo "=== Test 6: model.json через jq (CR/LF/quote safe) ==="
if grep -q 'jq -n' "$WORKFLOW"; then
    pass "jq -n используется для model.json"
else
    fail "jq -n для model.json" "не найдено — heredoc ломается на CRLF"
fi

# --- Test 7: Summary step тоже sanitized ------------------------------------
echo ""
echo "=== Test 7: Summary step sanitization ==="
# Берём 50 строк после 'name: Summary' и ищем VOICE_TEXT в env.
if grep -A50 'name: Summary' "$WORKFLOW" | head -50 | grep -q 'VOICE_TEXT:'; then
    pass "Summary имеет VOICE_TEXT в env"
else
    fail "Summary env" "VOICE_TEXT не в env Summary"
fi

# --- Test 8: регрессия — НЕ задето ничего лишнего ----------------------------
echo ""
echo "=== Test 8: регрессия — sanity check структуры ==="
# Workflow должен остаться валидным YAML (хотя бы минимальная проверка)
_yaml_lines=$(grep -c '^      - name:' "$WORKFLOW" || true)
if [ "$_yaml_lines" -ge 20 ]; then
    pass "структура workflow сохранена ($_yaml_lines шагов)"
else
    fail "структура" "слишком мало шагов: $_yaml_lines (ожидалось ≥20)"
fi

# --- Итоги -------------------------------------------------------------------
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
