#!/bin/bash
# ============================================================================
# test_e2e_voice_check_patterns_step_before.sh — ретро t_27c7e7f7 (issue #1771)
#
# Проверяет что в .github/workflows/scripts/e2e_voice_test.sh:
#   1) check_patterns вызывается с $STEP_BEFORE, а не с фиксированным
#      $(date -u -d '-6 minutes') — иначе multi-step сценарии (>6 мин)
#      ложно фейлятся (matcher читает логи РАНЕЕ чем произошёл шаг).
#   2) STEP_BEFORE пересчитывается НА КАЖДОЙ итерации step-loop
#      (внутри `while :` для scenario-loop, до run_step для single).
#   3) STEP_BEFORE вычисляется на host21 (через $ROBOT_SSH) — это wall-clock
#      который docker logs --since корректно интерпретирует.
#
# Ретро-контекст: PASS runs = single/dual-step (1-2 шага), FAIL runs =
# multi-step (7-16 шагов). LLM реально вызывал нужный tool, но matcher
# говорил PATTERN_MISS из-за 6-мин окна.
#
# Run:
#   bash scripts/agent_flow/tests/test_e2e_voice_check_patterns_step_before.sh
# ============================================================================
set -euo pipefail

TEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# tests/ → scripts/agent_flow/tests/ → scripts/agent_flow/ → scripts/ → REPO_ROOT
REPO_ROOT="$(cd "$TEST_DIR/../../.." && pwd)"
SCRIPT="$REPO_ROOT/.github/workflows/scripts/e2e_voice_test.sh"

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

assert_not_present() {
    local name="$1" pattern="$2" file="$3" desc="$4"
    if grep -nE "$pattern" "$file" >/dev/null 2>&1; then
        local matches; matches="$(grep -nE "$pattern" "$file" | head -3)"
        fail "$name" "$desc — найдены совпадения: $matches"
    else
        pass "$name"
    fi
}

assert_present() {
    local name="$1" pattern="$2" file="$3" desc="$4"
    if grep -nE "$pattern" "$file" >/dev/null 2>&1; then
        pass "$name"
    else
        fail "$name" "$desc — pattern '$pattern' не найден"
    fi
}

assert_count() {
    local name="$1" pattern="$2" file="$3" expected="$4" desc="$5"
    local actual; actual="$(grep -cE "$pattern" "$file" 2>/dev/null || echo 0)"
    if [ "$actual" = "$expected" ]; then
        pass "$name (count=$actual)"
    else
        fail "$name" "$desc — ожидалось $expected вхождений, найдено $actual"
    fi
}

# --- Sanity: файл существует --------------------------------------------------
echo "=== Sanity: e2e_voice_test.sh ==="
if [ -f "$SCRIPT" ]; then
    pass "script exists: $SCRIPT"
else
    fail "script exists" "не найден $SCRIPT"
    exit 1
fi

# --- Test 1: НЕТ фиксированного 6-мин окна ----------------------------------
echo ""
echo "=== Test 1: check_patterns использует \$STEP_BEFORE, не -6 minutes ==="
assert_not_present \
    "no_fixed_6min_window" \
    "check_patterns.*date -u -d '-6 minutes'" \
    "$SCRIPT" \
    "фиксированное 6-мин окно в check_patterns пропустит логи ранних шагов для multi-step сценариев"
assert_not_present \
    "no_6min_window_anywhere" \
    "date -u -d '-6 minutes'" \
    "$SCRIPT" \
    "фиксированное 6-мин окно где-либо в скрипте — это footgun для multi-step"

# --- Test 2: check_patterns вызывается с $STEP_BEFORE ------------------------
echo ""
echo "=== Test 2: check_patterns вызывается с \$STEP_BEFORE ==="
# Минимум 2 вхождения: scenario-loop path и single-step path.
assert_count \
    "check_patterns_with_STEP_BEFORE" \
    'check_patterns[[:space:]]+"\$STEP_BEFORE"' \
    "$SCRIPT" \
    "2" \
    "check_patterns должен использовать STEP_BEFORE в scenario-loop (1x) и single-path (1x)"

# --- Test 3: STEP_BEFORE пересчитывается внутри scenario-loop ----------------
echo ""
echo "=== Test 3: STEP_BEFORE пересчитывается на каждой итерации step-loop ==="
# Проверяем что STEP_BEFORE=... появляется внутри `while :; do` блока
# (scenario-loop) И перед run_step в single-path.
#
# Эвристика: STEP_BEFORE=...$ROBOT_SSH... должен быть минимум 2 раза
# (один — внутри scenario-loop, один — single-path), и оба раза
# непосредственно перед вызовом run_step.
step_before_count="$(grep -cE 'STEP_BEFORE="\$?\(\$\{?ROBOT_SSH' "$SCRIPT" 2>/dev/null || echo 0)"
if [ "$step_before_count" -ge "2" ]; then
    pass "STEP_BEFORE calculated on host21 (count=$step_before_count, expected ≥2)"
else
    fail "STEP_BEFORE_recount" \
        "STEP_BEFORE должен пересчитываться минимум в 2 местах (scenario-loop + single-path), найдено: $step_before_count"
fi

# --- Test 4: STEP_BEFORE использует $ROBOT_SSH (host21), не локальный date ---
echo ""
echo "=== Test 4: STEP_BEFORE берётся с host21 через \$ROBOT_SSH ==="
# Если STEP_BEFORE = $(date -u ...) БЕЗ ssh — это clock skew с ROS sim-time,
# docker logs --since интерпретирует timestamp неверно.
assert_not_present \
    "step_before_not_local_date" \
    'STEP_BEFORE="\$\(date -u' \
    "$SCRIPT" \
    "STEP_BEFORE должен вычисляться через \$ROBOT_SSH (host21 wall-clock), не локально"

# --- Test 5: в фиксе нет regression на docker logs команду -------------------
echo ""
echo "=== Test 5: docker logs --since команда внутри check_patterns НЕ изменена ==="
# Параметр `before` уже правильно используется в docker logs, не трогаем.
assert_present \
    "check_patterns_docker_logs_since" \
    'docker logs voice-assistant --since' \
    "$SCRIPT" \
    "docker logs --since команда должна остаться (фикс только в caller, передающем before)"
assert_present \
    "check_patterns_uses_local_before" \
    'local before="\$1"' \
    "$SCRIPT" \
    "check_patterns должен принимать before как \$1 и использовать локально"

# --- Test 6: STEP_BEFORE существует как переменная ----------------------------
echo ""
echo "=== Test 6: STEP_BEFORE объявлен как локальная переменная ==="
# Используется в run_step, parse_transcript, check_patterns, check_acceptance.
# Достаточно проверить что STEP_BEFORE= где-то присваивается.
step_before_assigns="$(grep -cE '^[[:space:]]*STEP_BEFORE=' "$SCRIPT" 2>/dev/null || echo 0)"
if [ "$step_before_assigns" -ge "2" ]; then
    pass "STEP_BEFORE assigned (count=$step_before_assigns, expected ≥2)"
else
    fail "STEP_BEFORE_assigned" \
        "STEP_BEFORE должен присваиваться минимум 2 раза (scenario-loop + single-path), найдено: $step_before_assigns"
fi

# --- Итог --------------------------------------------------------------------
echo ""
echo "=== Итог ==="
echo "Всего тестов: $TESTS_TOTAL"
echo "Прошло:      $TESTS_PASSED"
echo "Упало:       $TESTS_FAILED"
if [ "$TESTS_FAILED" -gt "0" ]; then
    echo ""
    echo "Упавшие тесты:"
    for name in "${FAILED_NAMES[@]}"; do
        echo "  - $name"
    done
    exit 1
fi
echo ""
echo -e "${GRN}PASS${END}: check_patterns использует \$STEP_BEFORE, фиксированное 6-мин окно устранено."
exit 0
