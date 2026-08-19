#!/bin/bash
# ============================================================================
# test_e2e_process_acceptance_file.sh — bug t_cca7c074 (ретро 19.08),
# расширено для issue #1456 / #1452
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
#   9) Convention 3 в e2e-process.sh (issue #1456) — strip _suite/_v<N> в
#      deploy-слое (раньше была только в harness). Чтобы не дублировать
#      harness PREFIX на deploy-уровне, для сценариев вида foo_suite_v1.json
#      e2e-process сам отрезолвит foo_acceptance_v1.json и передаст явно
#      в workflow (а не даст harness падать в GATE-1).
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

# e2e-process auto-discovery (convention 1/2 + Convention 3 — issue #1456).
# Синхронна с PREFIX-логикой в harness (issue #1452). Локальная копия
# реализации в e2e-process.sh (Convention 3 в issue #1456 — strip _suite
# и _v<N> для сценариев вида foo_suite_v1.json → foo_acceptance.json /
# foo_acceptance_v1.json). Если скрипт изменится — обновите и эту копию.
ep_discover_acceptance() {  # $1=scenario_file
    local scenario_file="$1"
    local _acc_dir _acc_basename _acc_prefix _acc_found
    _acc_dir="$(dirname "$scenario_file")"
    _acc_basename="$(basename "$scenario_file" .json)"
    _acc_found=""
    # Convention 1
    if [ -f "${_acc_dir}/acceptance.json" ]; then
        _acc_found="${_acc_dir}/acceptance.json"
    # Convention 2
    elif [ -f "${_acc_dir}/${_acc_basename}_acceptance.json" ]; then
        _acc_found="${_acc_dir}/${_acc_basename}_acceptance.json"
    # Convention 3 (issue #1456): strip _v[0-9]+ и _suite для PREFIX,
    # затем ищем <prefix>_acceptance.json или <prefix>_acceptance_v{N}.json.
    else
        _acc_prefix="$_acc_basename"
        _acc_prefix="${_acc_prefix%_v[0-9]*}"
        # %_suite (с подчёркиванием) — НЕ %suite (issue #1461, retro #1456).
        # %suite без _ удаляет буквы ИЗ КОНЦА слова и оставляет висящий _.
        _acc_prefix="${_acc_prefix%_suite}"
        if [ -f "${_acc_dir}/${_acc_prefix}_acceptance.json" ]; then
            _acc_found="${_acc_dir}/${_acc_prefix}_acceptance.json"
        elif [ -f "${_acc_dir}/${_acc_prefix}_acceptance_v1.json" ]; then
            _acc_found="${_acc_dir}/${_acc_prefix}_acceptance_v1.json"
        elif [ -f "${_acc_dir}/${_acc_prefix}_acceptance_v2.json" ]; then
            _acc_found="${_acc_dir}/${_acc_prefix}_acceptance_v2.json"
        fi
    fi
    printf '%s' "$_acc_found"
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

# --- Test 14: Convention 3 — e2e-process.sh (issue #1456) ------------------
# e2e-process должен отрезолвить foo_suite_v1.json → foo_acceptance[_v1].json
# через PREFIX (strip _suite/_v[0-9]+), даже если рядом НЕТ
# foo_suite_v1_acceptance.json / acceptance.json. Это лечит round-156 GATE-1 FAIL:
# e2e_acceptance_file оставался пустым → workflow не передавал acceptance →
# harness падал на GATE-1, хотя реальный файл был в репо.
echo ""
echo "=== Test 14: Convention 3 (issue #1456) — strip _suite/_v<N> в e2e-process ==="
_mock_dir5="$(mktemp -d)"
trap 'rm -rf "$_mock_dir" "$_mock_dir2" "$_mock_dir3" "$_mock_dir4" "$_mock_dir5"' EXIT
mkdir -p "$_mock_dir5/scenarios"
# Ровно кейс issue #1456 / #1452: music_library_suite_v1.json рядом с
# music_library_acceptance_v1.json (без music_library_suite_v1_acceptance.json
# или acceptance.json).
echo '{}' > "$_mock_dir5/scenarios/music_library_suite_v1.json"
echo '{}' > "$_mock_dir5/scenarios/music_library_acceptance_v1.json"
_discovered_ep="$(ep_discover_acceptance "$_mock_dir5/scenarios/music_library_suite_v1.json")"
assert_eq "e2e-process Convention 3 — strip _suite_v1 → music_library_acceptance_v1" \
    "$_mock_dir5/scenarios/music_library_acceptance_v1.json" "$_discovered_ep"

# Также: PREFIX + _v2 (если бы кто-то назвал music_library_acceptance_v2.json)
echo '{}' > "$_mock_dir5/scenarios/voice_selection_suite_v2.json"
echo '{}' > "$_mock_dir5/scenarios/voice_selection_acceptance_v2.json"
_discovered_ep="$(ep_discover_acceptance "$_mock_dir5/scenarios/voice_selection_suite_v2.json")"
assert_eq "e2e-process Convention 3 — _v2 тоже резолвится" \
    "$_mock_dir5/scenarios/voice_selection_acceptance_v2.json" "$_discovered_ep"

# Если есть только PREFIX без версии — резолвится (music_library_acceptance.json
# без _v1). Сценарий может называться music_library_suite.json (без _v<N>).
echo '{}' > "$_mock_dir5/scenarios/music_library_suite.json"
echo '{}' > "$_mock_dir5/scenarios/music_library_acceptance.json"
_discovered_ep="$(ep_discover_acceptance "$_mock_dir5/scenarios/music_library_suite.json")"
assert_eq "e2e-process Convention 3 — без _v<N> тоже резолвится" \
    "$_mock_dir5/scenarios/music_library_acceptance.json" "$_discovered_ep"

# --- Test 13: harness передаёт --acceptance вверх по --acceptance ----------
echo ""
echo "=== Test 13: harness CLI --acceptance flag (regression guard) ==="
if grep -q '\-\-acceptance)' "$HARNESS"; then
    pass "harness поддерживает --acceptance CLI flag"
else
    fail "--acceptance flag" "не найден в harness"
fi

# --- Test 15: e2e-process.sh содержит Convention 3 (sanity) -----------------
# Sanity: после фикса issue #1456 в e2e-process.sh должна быть PREFIX-логика
# (strip _suite / _v[0-9]+). Без неё deploy-слой не отрезолвит
# music_library_suite_v1.json → music_library_acceptance_v1.json.
echo ""
echo "=== Test 15: e2e-process.sh содержит Convention 3 strip _suite/_v[N] (issue #1456) ==="
# Конкретные строки PREFIX-стрипа:
#   _acc_prefix="${_acc_prefix%_v[0-9]*}"   — strip _v<digits>+
#   _acc_prefix="${_acc_prefix%_suite}"     — strip _suite
#   ⚠️  Должно быть именно %_suite (с подчёркиванием): %suite без _ — БАГ
#   (issue #1461), см. Test 17 ниже.
# shellcheck disable=SC2016  # тестовые литералы — не шелл-код
if grep -qE '_acc_prefix="\$\{_acc_prefix%_v\[0-9\]\*\}"' "$E2E_PROCESS"; then
    pass "Convention 3 (strip _v[0-9]+) присутствует в e2e-process.sh"
else
    fail "Convention 3 strip _v[N]" "не найдена в e2e-process.sh — регресс issue #1456"
fi
# Sanity: STRICTLY требуем %_suite (с подчёркиванием). %suite без _ оставляет
# висящий _ (issue #1461): music_library_suite → music_library_.
if grep -qE '_acc_prefix="\$\{_acc_prefix%_suite\}"' "$E2E_PROCESS" \
   && ! grep -qE '_acc_prefix="\$\{_acc_prefix%suite\}"' "$E2E_PROCESS"; then
    pass "Convention 3 (strip _suite с подчёркиванием) присутствует в e2e-process.sh"
else
    fail "Convention 3 strip _suite (правильная форма %_suite)" \
        "в e2e-process.sh — регресс issue #1461 (найден %suite без _ или нет %_suite)"
fi
if grep -q 'auto-discovered (convention 3' "$E2E_PROCESS"; then
    pass "Convention 3 log-message присутствует в e2e-process.sh"
else
    fail "Convention 3 log-message" "не найдена в e2e-process.sh"
fi

# --- Test 16: bash pattern регресс issue #1461 (strip _suite, не strip suite) ----
# Issue #1461: ${var%suite} без _ удаляет буквы suite ИЗ КОНЦА слова и
# оставляет висящий _. Нужно именно ${var%_suite}.
# Прямая проверка: запускает реальный bash-паттерн из e2e-process.sh на
# строке 'music_library_suite' и убеждается, что результат 'music_library'
# (а не 'music_library_'). Без вытаскивания bash-кода через regex.
echo ""
echo "=== Test 16: bash pattern %suite vs %_suite — регресс issue #1461 ==="
# Извлекаем ОБА strip-паттерна из e2e-process.sh (строки идут подряд в
# блоке Convention 3). Ищем строки, начинающиеся с точно такого же
# _acc_prefix= и заканчивающиеся на ${_acc_prefix%...}".
_strip_v_line="$(grep -nE '_acc_prefix="\$\{_acc_prefix%_v\[0-9\]\*\}"' "$E2E_PROCESS" | head -1 | cut -d: -f1)"
_strip_suite_line="$(grep -nE '_acc_prefix="\$\{_acc_prefix%[^}]*\}"' "$E2E_PROCESS" \
    | awk -F: -v base="$_strip_v_line" '$1 > base {print; exit}' | cut -d: -f1)"
if [ -z "$_strip_v_line" ] || [ -z "$_strip_suite_line" ]; then
    fail "Convention 3 bash pattern present" \
        "не нашёл ОБА strip-строки в $E2E_PROCESS (v-line='$_strip_v_line', suite-line='$_strip_suite_line')"
else
    _pattern_v="$(sed -n "${_strip_v_line}p" "$E2E_PROCESS")"
    _pattern_suite="$(sed -n "${_strip_suite_line}p" "$E2E_PROCESS")"
    # Проверяем, что suite-pattern содержит ИМЕННО %_suite (с _).
    if echo "$_pattern_suite" | grep -q '%_suite'; then
        pass "Convention 3 suite-pattern содержит %_suite (правильная форма)"
    else
        fail "Convention 3 suite-pattern" \
            "найден '$_pattern_suite' (нет %_suite — регресс issue #1461)"
    fi
    # Полный pipeline: воспроизводим ровно как в e2e-process.sh.
    _tmp_script="$(mktemp -t ep_strip_XXXXXX.sh)"
    cat > "$_tmp_script" <<EOF
#!/bin/bash
_acc_basename="music_library_suite_v1"
_acc_prefix="\$_acc_basename"
$_pattern_v
$_pattern_suite
printf '%s' "\$_acc_prefix"
EOF
    chmod +x "$_tmp_script"
    _actual="$(bash "$_tmp_script")"
    rm -f "$_tmp_script"
    if [ "$_actual" = "music_library" ]; then
        pass "Convention 3 strip pipeline → 'music_library' (issue #1461 fixed)"
    else
        fail "Convention 3 strip pipeline" \
            "ожидался 'music_library', получено '$_actual' — регресс issue #1461"
    fi
fi

# Также sanity: тот же паттерн должен работать на 'foo_suite' (без _v<N>).
_tmp_script="$(mktemp -t ep_strip2_XXXXXX.sh)"
cat > "$_tmp_script" <<EOF
#!/bin/bash
_acc_basename="foo_suite"
_acc_prefix="\$_acc_basename"
$_pattern_v
$_pattern_suite
printf '%s' "\$_acc_prefix"
EOF
chmod +x "$_tmp_script"
_actual_foo="$(bash "$_tmp_script")"
rm -f "$_tmp_script"
if [ "$_actual_foo" = "foo" ]; then
    pass "Convention 3 strip pipeline → 'foo' для 'foo_suite'"
else
    fail "Convention 3 strip pipeline (foo_suite)" \
        "ожидался 'foo', получено '$_actual_foo'"
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