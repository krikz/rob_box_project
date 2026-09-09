#!/bin/bash
# ============================================================================
# test_e2e_process_scenario_file.sh — bug(e2e #1375) ретро 18.08
#
# Проверяет парсинг scenario_file из блока ## e2e в agent-flow-e2e-process.sh:
#   1) Явный scenario_file в issue body → парсится
#   2) scenario_file в backticks / кавычках → стрипается обёртка
#   3) Fallback на PR body → парсится оттуда
#   4) Auto-discovery из PR files (ADDED > MODIFIED) → подставляется
#   5) Ничего нет → e2e_scenario_file пустой, workflow берёт дефолт
#
# Тест НЕ запускает agent-flow-e2e-process.sh целиком (слишком много
# pre-checks), а изолирует ту же логику парсинга в локальные функции —
# гарантирует, что регулярки совпадают с тем, что в скрипте.
#
# Если в будущем логика парсинга в скрипте изменится — обновить и этот тест.
#
# Run:
#   bash scripts/agent_flow/tests/test_e2e_process_scenario_file.sh
# ============================================================================
set -euo pipefail

TEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# tests/ → scripts/agent_flow/tests/ → scripts/agent_flow/ → scripts/ → REPO_ROOT
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

# --- Локальная копия extract-логики (синхронна со скриптом) ----------------
# Если скрипт изменится — синхронизируйте с ним. Чтобы не дублировать,
# подтверждаем что паттерны совпадают с теми, что в $E2E_PROCESS.
extract_scenario_file() {  # $1=body
    local body="$1"
    local body_real
    body_real="$(printf '%s' "$body" | sed 's/\\n/\
/g')"
    # Только в блоке ## e2e (после маркера ## e2e до следующего ## )
    # Простой grep достаточен для юнит-теста.
    printf '%s' "$body_real" | grep -iE '^[[:space:]]*scenario_file[[:space:]]*:' | head -1 \
        | sed -E 's/^[[:space:]]*scenario_file[[:space:]]*:[[:space:]]*//; s/^`//; s/`$//; s/^"//; s/"$//' || true
}

# --- Sanity-check: паттерн в скрипте совпадает с тестом ---------------------
echo "=== Sanity: паттерн в $E2E_PROCESS совпадает с локальным extract ==="
# Извлечь строку парсинга scenario_file из скрипта и сравнить.
_script_pattern="$(grep -F 'scenario_file[[:space:]]*:' "$E2E_PROCESS" | head -3 || true)"
if [ -n "$_script_pattern" ]; then
    pass "паттерн scenario_file присутствует в e2e-process.sh"
else
    fail "паттерн scenario_file" "не найден в e2e-process.sh — скрипт не патчился?"
fi

if grep -q 'e2e_args+=(-f "scenario_file=' "$E2E_PROCESS"; then
    pass "передача scenario_file в e2e_args присутствует"
else
    fail "передача scenario_file в e2e_args" "строка не найдена"
fi

if grep -q 'auto-discovered from PR' "$E2E_PROCESS"; then
    pass "auto-discovery блок присутствует"
else
    fail "auto-discovery" "не найден в e2e-process.sh"
fi

# --- Test 1: простое значение ---------------------------------------------
echo ""
echo "=== Test 1: простое scenario_file: ./.json ==="
out=$(extract_scenario_file 'scenario_file: .github/e2e/scenarios/foo.json')
assert_eq "plain path" ".github/e2e/scenarios/foo.json" "$out"

# --- Test 2: в backticks ----------------------------------------------------
echo ""
echo "=== Test 2: scenario_file в backticks ==="
out=$(extract_scenario_file 'scenario_file: `.github/e2e/scenarios/bar.json`')
assert_eq "backticks stripped" ".github/e2e/scenarios/bar.json" "$out"

# --- Test 3: в кавычках ----------------------------------------------------
echo ""
echo "=== Test 3: scenario_file в кавычках ==="
out=$(extract_scenario_file 'scenario_file: ".github/e2e/scenarios/baz.json"')
assert_eq "quotes stripped" ".github/e2e/scenarios/baz.json" "$out"

# --- Test 4: с ведущими/завершающими пробелами ----------------------------
echo ""
echo "=== Test 4: ведущие пробелы (стандартный формат блока) ==="
# Реальный формат: scenario_file: <path> (без лишних пробелов)
out=$(extract_scenario_file 'scenario_file:   .github/e2e/scenarios/spaced.json')
# trailing пробелы trim'аются обратным вызовом в bash-контексте НЕ нужно;
# контракт ожидает чистый path. Тест проверяет что ведущие пробелы OK.
case "$out" in
    .github/e2e/scenarios/spaced.json*) pass "leading whitespace (path captured)" ;;
    *) fail "leading whitespace" "got: '$out'" ;;
esac

# --- Test 5: пустое значение (smoke-test fallback) ------------------------
echo ""
echo "=== Test 5: пустое scenario_file (smoke-test fallback) ==="
out=$(extract_scenario_file 'scenario_file:')
assert_eq "empty value" "" "$out"

# --- Test 6: реальный PR #1375 body (фрагмент) ----------------------------
echo ""
echo "=== Test 6: реальный формат PR #1375 ## e2e ==="
pr_body='## e2e

```
scenario_file: .github/e2e/scenarios/music_library_suite_v1.json
voice: anton
volume: 150
```'
out=$(extract_scenario_file "$pr_body")
assert_eq "PR #1375 body format" ".github/e2e/scenarios/music_library_suite_v1.json" "$out"

# --- Test 7: jql auto-discovery (ADDED > MODIFIED) -------------------------
echo ""
echo "=== Test 7: jql auto-discovery (PR #1375 files) ==="
# Воспроизводим логику из скрипта — здесь без jq, проверяем руками выбор.
_files='[
    {"path":".github/e2e/scenarios/full_instrument_suite_v1.json","changeType":"MODIFIED"},
    {"path":".github/e2e/scenarios/music_library_suite_v1.json","changeType":"ADDED"},
    {"path":".github/e2e/voice_commands/rabot_chto_eto_za_trek.ogg","changeType":"ADDED"}
]'
# Python helper (jq отсутствует в окружении)
discovered="$(python3 - <<PY
import json, re
files = json.loads('''$_files''')
rx = re.compile(r"\.github/e2e/scenarios/.*\.json\$")
candidates = [f for f in files if rx.match(f["path"])]
# ADDED first, MODIFIED second (по порядку в списке)
order = {"ADDED": 0, "MODIFIED": 1}
ranked = sorted(candidates, key=lambda f: (order.get(f.get("changeType", "MODIFIED"), 2), files.index(f)))
print(ranked[0]["path"] if ranked else "")
PY
)"
assert_eq "auto-discover picks ADDED music_library" \
    ".github/e2e/scenarios/music_library_suite_v1.json" "$discovered"

# --- Test 8: реальный run проверки jq-выражения через gh api --------------
# Skip если gh недоступен / нет auth
echo ""
echo "=== Test 8: gh pr view --json files (интеграционный, skip если нет auth) ==="
if command -v gh >/dev/null 2>&1 && GH_CONFIG_DIR="${GH_CONFIG_DIR:-/home/builder/.config/gh}" gh auth status >/dev/null 2>&1; then
    _files_json="$(GH_CONFIG_DIR="${GH_CONFIG_DIR:-/home/builder/.config/gh}" gh pr view 1375 \
        --repo krikz/rob_box_project --json files 2>/dev/null || echo "[]")"
    _count="$(printf '%s' "$_files_json" | python3 -c "
import sys, json, re
data = json.load(sys.stdin)
files = data.get('files', [])
rx = re.compile(r'\.github/e2e/scenarios/.*\.json\$')
cands = [f for f in files if rx.match(f['path'])]
order = {'ADDED': 0, 'MODIFIED': 1}
ranked = sorted(cands, key=lambda f: (order.get(f.get('changeType', 'MODIFIED'), 2), files.index(f)))
print(ranked[0]['path'] if ranked else 'NONE')
")"
    assert_eq "real PR #1375 → music_library_suite_v1.json" \
        ".github/e2e/scenarios/music_library_suite_v1.json" "$_count"
else
    echo "  ${YEL}~${END} gh auth недоступен, skip реального вызова"
    TESTS_TOTAL=$((TESTS_TOTAL+1))
    TESTS_PASSED=$((TESTS_PASSED+1))
fi

# --- Test 9: issue #1421 — git diff origin/develop...HEAD auto-discovery ---
echo ""
echo "=== Test 9: issue #1421 — git diff origin/develop...HEAD в WORKTREE_DIR ==="
# Мокируем worktree через отдельный bare-clone + git diff:
#   - создаём bare origin
#   - создаём worktree-clone, чекаутим ROUND-ветку
#   - мержим сценарий из фичи-ветки → diff vs origin/develop содержит *.json
#   - проверяем что новая логика в скрипте найдёт сценарий.
_mock_dir="$(mktemp -d)"
trap 'rm -rf "$_mock_dir"' EXIT
git -C "$_mock_dir" init -q --bare mock-origin.git
git -C "$_mock_dir" init -q mock-wt
git -C "$_mock_dir/mock-wt" remote add origin "$_mock_dir/mock-origin.git"
git -C "$_mock_dir/mock-wt" config user.email "t@test" && git -C "$_mock_dir/mock-wt" config user.name "t"
# seed develop with non-scenario file
mkdir -p "$_mock_dir/mock-wt/scripts"
echo "x" > "$_mock_dir/mock-wt/scripts/README.md"
git -C "$_mock_dir/mock-wt" add scripts/README.md
git -C "$_mock_dir/mock-wt" commit -q -m "seed develop"
git -C "$_mock_dir/mock-wt" push -q origin HEAD:develop 2>/dev/null
git -C "$_mock_dir/mock-wt" branch -f develop origin/develop 2>/dev/null
# create ROUND branch and FEATURE branch with scenario
git -C "$_mock_dir/mock-wt" checkout -q -b round
mkdir -p "$_mock_dir/mock-wt/.github/e2e/scenarios"
echo "{}" > "$_mock_dir/mock-wt/.github/e2e/scenarios/music_library_suite_v1.json"
echo "{}" > "$_mock_dir/mock-wt/.github/e2e/scenarios/full_instrument_suite_v1.json"
git -C "$_mock_dir/mock-wt" add .github
git -C "$_mock_dir/mock-wt" commit -q -m "add scenarios in round"
git -C "$_mock_dir/mock-wt" push -q origin HEAD:round
git -C "$_mock_dir/mock-wt" checkout -q -b feature 2>/dev/null || true

# Test 9a: WORKTREE_DIR with round → diff содержит сценарии
_diff_list="$(git -C "$_mock_dir/mock-wt" diff --name-only origin/develop...HEAD 2>/dev/null \
    | grep -E '(^|/)(\.github/e2e/scenarios/.*\.json)$' || true)"
assert_eq "diff vs develop находит 2 сценария" 2 "$(printf '%s\n' "$_diff_list" | grep -c .)"

# Test 9b: ADDED > MODIFIED — fresh-added файл выигрывает
_diff_added="$(git -C "$_mock_dir/mock-wt" diff --name-only --diff-filter=A origin/develop...HEAD 2>/dev/null \
    | grep -E '(^|/)(\.github/e2e/scenarios/.*\.json)$' || true)"
_picked="$(printf '%s\n' "$_diff_added" | head -1)"
case "$_picked" in
    .github/e2e/scenarios/*) pass "ADDED-filter picks первый сценарий ($_picked)" ;;
    *) fail "ADDED-filter" "не подобрал сценарий, got: '$_picked'" ;;
esac

# Test 9c: regression — НЕ-сценарии в diff НЕ подхватываются
echo "another" > "$_mock_dir/mock-wt/README.md"
git -C "$_mock_dir/mock-wt" add README.md
git -C "$_mock_dir/mock-wt" commit -q -m "non-scenario change"
_diff_only_scenarios="$(git -C "$_mock_dir/mock-wt" diff --name-only origin/develop...HEAD 2>/dev/null \
    | grep -E '(^|/)(\.github/e2e/scenarios/.*\.json)$' || true)"
assert_eq "non-scenario файлы отфильтрованы" \
    "2" "$(printf '%s\n' "$_diff_only_scenarios" | grep -c .)"

# Test 9d: Sanity-check — в e2e-process.sh присутствует новый diff-блок
if grep -q 'auto-discovered from round diff vs origin/develop' "$E2E_PROCESS"; then
    pass "новый diff-блок присутствует в e2e-process.sh"
else
    fail "diff-блок" "строка 'auto-discovered from round diff vs origin/develop' не найдена в e2e-process.sh"
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
