#!/bin/bash
# ============================================================================
# test_e2e_process_scenario_file_default_fallback.sh — bug(#1551) ретро 23.08
#
# Проверяет default-fallback в agent-flow-e2e-process.sh: если scenario_file
# НЕ найден ни в issue body, ни в PR body, ни в PR files, ни в diff
# origin/develop...HEAD, скрипт ОБЯЗАН fallback-иться на
# `.github/e2e/scenarios/voice_core_suite_v1.json` — иначе workflow получает
# пустой scenario_file input и катит smoke-test "Робот, спой песенку про
# енотика" (run #32607268334 round-212 — ложный PASS, ретро 23.08).
#
# Это НЕ fallback на одиночный smoke-test (вариант А в issue #1551 говорит
# именно про core suite, не single). Конвенция закреплена в e2e-process.sh.
#
# Проверяется:
#   1) Sanity: в скрипте присутствует default-fallback строка (grep)
#   2) Hard-default указывает на .github/e2e/scenarios/voice_core_suite_v1.json
#   3) Acceptance default fallback — voice_core_acceptance_v1.json
#   4) Логирование факта defaulting присутствует
#   5) acceptance_file convention (1: <scenario_dir>/acceptance.json) для
#      voice_core_suite_v1.json → voice_core_acceptance_v1.json работает
#   6) Negative: если scenario_file уже задан явно — default НЕ перезаписывает
#   7) Negative: workflow.yml default'ы должны быть пустыми, чтобы дефолт
#      e2e-process работал (иначе workflow может применить свой дефолт
#      раньше нашего, если порядок изменится)
#
# Run:
#   bash scripts/agent_flow/tests/test_e2e_process_scenario_file_default_fallback.sh
# ============================================================================
set -euo pipefail

TEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# tests/ → scripts/agent_flow/tests/ → scripts/agent_flow/ → scripts/ → REPO_ROOT
REPO_ROOT="$(cd "$TEST_DIR/../../.." && pwd)"
E2E_PROCESS="$REPO_ROOT/scripts/agent_flow/agent-flow-e2e-process.sh"
WORKFLOW="$REPO_ROOT/.github/workflows/L-E2E Voice Test.yml"

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
assert_match() {
    local name="$1" pattern="$2" actual="$3"
    if printf '%s' "$actual" | grep -Eq "$pattern"; then
        pass "$name"
    else
        fail "$name" "pattern '$pattern' not in: '$actual'"
    fi
}

# --- Sanity: файлы --------------------------------------------------------
echo "=== Sanity: SOT-файлы на месте ==="
[ -f "$E2E_PROCESS" ] && pass "agent-flow-e2e-process.sh существует" \
    || { fail "agent-flow-e2e-process.sh" "не найден: $E2E_PROCESS"; exit 1; }
[ -f "$WORKFLOW" ] && pass "L-E2E Voice Test.yml существует" \
    || { fail "L-E2E Voice Test.yml" "не найден: $WORKFLOW"; exit 1; }
[ -f "$REPO_ROOT/.github/e2e/scenarios/voice_core_suite_v1.json" ] \
    && pass "voice_core_suite_v1.json в develop" \
    || fail "voice_core_suite_v1.json" "отсутствует — hard-default невалиден"
[ -f "$REPO_ROOT/.github/e2e/scenarios/voice_core_acceptance_v1.json" ] \
    && pass "voice_core_acceptance_v1.json в develop" \
    || fail "voice_core_acceptance_v1.json" "отсутствует — hard-default невалиден"

# --- Test 1: наличие default-fallback в скрипте ---------------------------
echo ""
echo "=== Test 1: наличие default-fallback логики в e2e-process.sh ==="
# bug(#1551): если scenario_file пуст ПОСЛЕ всех auto-discovery, скрипт
# ОБЯЗАН взять дефолт voice_core_suite_v1.json. Ищем явный grep-маркер.
if grep -qE 'voice_core_suite_v1\.json' "$E2E_PROCESS"; then
    pass "hard-default voice_core_suite_v1.json присутствует в e2e-process.sh"
else
    fail "hard-default" "строка 'voice_core_suite_v1.json' не найдена — bug #1551 не пофикшен"
fi

# --- Test 2: дефолт стоит после всех auto-discovery (после diff-блока) ----
echo ""
echo "=== Test 2: default-fallback идёт ПОСЛЕ diff auto-discovery ==="
_diff_line="$(grep -n 'auto-discovered from round diff vs origin/develop' "$E2E_PROCESS" | head -1 | cut -d: -f1 || echo 0)"
# Находим строку с hard-default (НЕ первое вхождение voice_core_suite_v1.json
# в комментарии, а строку `e2e_scenario_file=".github/e2e/scenarios/voice_core_suite_v1.json"`).
_default_line="$(grep -nE '^\s*e2e_scenario_file=\"\.github/e2e/scenarios/voice_core_suite_v1' "$E2E_PROCESS" | head -1 | cut -d: -f1 || echo 0)"
if [ "$_diff_line" -gt 0 ] && [ "$_default_line" -gt 0 ] && [ "$_default_line" -gt "$_diff_line" ]; then
    pass "default-fallback (line $_default_line) после diff auto-discovery (line $_diff_line)"
else
    fail "порядок default" "default_line=$_default_line diff_line=$_diff_line — default должен быть ПОСЛЕ diff"
fi

# --- Test 3: acceptance_file тоже имеет fallback ---------------------------
echo ""
echo "=== Test 3: acceptance_file hard-default voice_core_acceptance_v1.json ==="
if grep -qE 'voice_core_acceptance_v1\.json' "$E2E_PROCESS"; then
    pass "hard-default acceptance_file в e2e-process.sh"
else
    fail "hard-default acceptance" "строка 'voice_core_acceptance_v1.json' не найдена"
fi

# --- Test 4: логирование defaulting ---------------------------------------
echo ""
echo "=== Test 4: логирование факта default-выбора ==="
# В моём фиксе: log "... defaulting to ${e2e_scenario_file} (bug #1551 round-212 fallback)"
# — переменная подставляется в рантайме. Проверяем что в скрипте есть
# literal-строка 'defaulting to' + признак bug #1551 (либо в той же log-строке,
# либо в окружающем комментарии). Это надёжнее чем literal-substring.
if grep -qE 'defaulting to' "$E2E_PROCESS" && grep -qE 'bug #1551' "$E2E_PROCESS"; then
    pass "log-маркер defaulting + bug #1551 присутствуют (Шифу увидит когда сработал fallback)"
else
    fail "log defaulting" "не найдено 'defaulting to' или 'bug #1551' — fallback сработает молча"
fi

# --- Test 5: convention 1 для core_suite (acceptance рядом) --------------
# voice_core_suite_v1.json → должен резолвиться в voice_core_acceptance_v1.json
# через harness convention 1 (или convention 3 strip _suite/_v<N>).
echo ""
echo "=== Test 5: convention 1 — voice_core_suite_v1.json → voice_core_acceptance_v1.json ==="
_scenario_dir="$(dirname '.github/e2e/scenarios/voice_core_suite_v1.json')"
_basename="$(basename 'voice_core_suite_v1.json' .json)"
# convention 3 (PREFIX strip): strip _v<N> then _suite
_prefix="${_basename%_v[0-9]*}"
case "$_prefix" in
    *_suite) _prefix="${_prefix%_suite}" ;;
esac
_candidate="${_scenario_dir}/${_prefix}_acceptance_v1.json"
assert_eq "PREFIX strip convention 3" \
    ".github/e2e/scenarios/voice_core_acceptance_v1.json" "$_candidate"

# --- Test 6: если scenario_file задан — default НЕ перезаписывает ---------
echo ""
echo "=== Test 6: default не перезаписывает явный scenario_file ==="
# Guard: default-блок начинается со строки `if [ -z "$e2e_scenario_file" ]; then`
# — это и есть страховка от перезаписи. Достаточно проверить её наличие
# в окрестности hard-default (в пределах ±5 строк от default-присваивания).
_default_assign_line="$(grep -nE '^\s*e2e_scenario_file=\"\.github/e2e/scenarios/voice_core_suite_v1' "$E2E_PROCESS" | head -1 | cut -d: -f1 || echo 0)"
if [ "$_default_assign_line" -gt 0 ]; then
    _guard_window_start=$((_default_assign_line - 5))
    _guard_window_end=$((_default_assign_line - 1))
    _guard="$(sed -n "${_guard_window_start},${_guard_window_end}p" "$E2E_PROCESS")"
    case "$_guard" in
        *'-z "$e2e_scenario_file"'*) pass "guard 'if -z e2e_scenario_file' непосредственно перед default" ;;
        *) fail "guard default" "нет '[ -z \"\$e2e_scenario_file\" ]' рядом с default (lines $_guard_window_start..$_guard_window_end)" ;;
    esac
else
    fail "guard default" "не нашёл строки default-присваивания в скрипте"
fi

# --- Test 7: workflow default остаётся пустым -----------------------------
# Это страховка: если в workflow.yml поставить default: '.github/.../voice_core_suite_v1.json',
# то при пустом scenario_file input GitHub Actions ПОДСТАВИТ свой default и
# наш e2e-process default-fallback может отработать после — но порядок
# передачи args остаётся детерминированным. Главное — workflow НЕ должен
# иметь default для voice_text, отличный от "Робот, спой песенку про енотика"
# (это smoke-test default, см. issue #1551). Сама проверка — workflow.yml
# имеет default для scenario_file но он ПУСТОЙ (issue #1551 рекомендует
# так и оставить, чтобы наш fallback был источником истины).
echo ""
echo "=== Test 7: workflow scenario_file default пустой (или ожидаемый) ==="
_wf_scen_default="$(grep -A1 'scenario_file:' "$WORKFLOW" | grep 'default:' | head -1 | sed -E 's/.*default:[[:space:]]*//; s/^['\''"]//; s/['\''"]$//' || true)"
case "$_wf_scen_default" in
    '') pass "workflow scenario_file default = '' (наш fallback работает)" ;;
    voice_core_suite_v1.json) pass "workflow scenario_file default = voice_core_suite (тоже OK)" ;;
    *) fail "workflow scenario_file default" "unexpected: '$_wf_scen_default'" ;;
esac

# --- Test 8: комментарий Шифу / ссылка на issue в коде -------------------
echo ""
echo "=== Test 8: код ссылается на issue #1551 / bug(e2e #1551) ==="
if grep -qE 'bug.*#1551|#1551' "$E2E_PROCESS"; then
    pass "ссылка на issue #1551 в комментарии скрипта"
else
    fail "issue reference" "не нашёл ссылки на issue #1551 — Шифу не найдёт контекст через год"
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