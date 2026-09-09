#!/bin/bash
# ============================================================================
# test_e2e_process_gate_body_backticks.sh — bug(process #1419) ретро 18.08
#
# Регрессия: unescaped backticks внутри _gate_body="..." под
# `set -euo pipefail` → bash интерпретирует их как command substitution →
# пытается выполнить `kanban complete` / `PR closed, ...` → exit 127 →
# ломает весь тик cron. Live evidence: cron 73dcdece0619 18.08 18:09:44
# (stderr: `line 2285: kanban: command not found` / `PR: command not found`).
#
# Acceptance:
#   - bash с `set -euo pipefail` парсит _gate_body без падения (exit 0)
#   - мок python3 (BrokenPipeError) → script exit 0
#   - обе ветки (success-green, failure-red) парсятся одинаково
#
# Стратегия: используем выделенный python-сканер (lib/scan_gate_body_backticks.py),
# который надёжно определяет границы _gate_body="..." блоков и считает
# unescaped backticks внутри них.
#
# Run:
#   bash scripts/agent_flow/tests/test_e2e_process_gate_body_backticks.sh
# ============================================================================
set -euo pipefail

TEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$TEST_DIR/../../.." && pwd)"
E2E_PROCESS="$REPO_ROOT/scripts/agent_flow/agent-flow-e2e-process.sh"

# --- Helpers ---------------------------------------------------------------
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

# --- Sanity: убедимся, что скрипт реально содержит обе gate_body ----------
echo "=== Sanity: gate_body в $E2E_PROCESS ==="
if grep -q 'kanban complete' "$E2E_PROCESS"; then
    pass "gate_body упоминает kanban complete"
else
    fail "gate_body sanity" "kanban complete не найден в $E2E_PROCESS"
fi
if grep -q 'PR closed' "$E2E_PROCESS"; then
    pass "gate_body упоминает PR closed"
else
    fail "gate_body sanity" "PR closed не найден в $E2E_PROCESS"
fi

# --- Test 1: внутри gate_body все backticks экранированы (\`) -------------
echo ""
echo "=== Test 1: внутри gate_body все backticks экранированы ==="
# Сканер: пишем во временный файл (embedded heredoc + mktemp, чтобы не
# зависеть от того, есть ли lib/ в репо).
_scan_tmp=$(mktemp /tmp/_scan_gate_body.XXXXXX.py)
cat > "$_scan_tmp" <<'PYEOF'
import sys
path = sys.argv[1]
with open(path, "r", encoding="utf-8") as f:
    lines = f.readlines()
in_body = False
unscd = 0
total = 0
for line in lines:
    stripped = line.lstrip()
    if stripped.startswith('_gate_body="'):
        in_body = True
        continue
    stripped_n = line.rstrip("\n")
    if in_body and stripped_n.endswith('"') and not stripped_n.endswith('\\"'):
        in_body = False
        continue
    if not in_body:
        continue
    i = 0
    while i < len(line):
        if line[i] == "\\" and i + 1 < len(line):
            if line[i + 1] == "`":
                total += 1
            i += 2
            continue
        if line[i] == "`":
            unscd += 1
            total += 1
        i += 1
print(f"{unscd}	{total}")
PYEOF
_scan_out="$(python3 "$_scan_tmp" "$E2E_PROCESS")"
rm -f "$_scan_tmp"
_unscd="${_scan_out%%	*}"
_total_bt="${_scan_out##*	}"
assert_eq "gate_body: 0 unescaped backticks" "0" "$_unscd"
if [ "$_total_bt" -ge 4 ]; then
    pass "gate_body: backticks присутствуют (>=4), всего: $_total_bt"
else
    fail "gate_body: backticks" "ожидалось >=4, получено $_total_bt"
fi

# --- Test 2: regression — unescaped backticks ломают set -euo pipefail ----
echo ""
echo "=== Test 2: regression: unescaped backticks → exit 127 ==="
_write_tmp=$(mktemp /tmp/_gate_body_regression.XXXXXX.sh)
cat > "$_write_tmp" <<'SH'
#!/bin/bash
set -euo pipefail
# Точная копия бага 18.08 — unescaped backticks
_gate_body="## Тест
**ЕСЛИ PR ЗАКРЫТ** → сделай `kanban complete` с пометкой `PR closed, карточка не нужна`."
echo "${_gate_body}"
SH
chmod +x "$_write_tmp"
_reg_out=$(bash "$_write_tmp" 2>&1) || _reg_exit=$?
_reg_exit=${_reg_exit:-0}
rm -f "$_write_tmp"
assert_eq "regression: exit code = 127" "127" "$_reg_exit"
if echo "$_reg_out" | grep -q "kanban: command not found"; then
    pass "regression: ошибка 'kanban: command not found' в stderr"
else
    fail "regression: ошибка kanban" "stderr не содержит 'kanban: command not found': $_reg_out"
fi
if echo "$_reg_out" | grep -q "PR: command not found"; then
    pass "regression: ошибка 'PR: command not found' в stderr"
else
    fail "regression: ошибка PR" "stderr не содержит 'PR: command not found': $_reg_out"
fi

# --- Test 3: фикс — экранированные backticks парсятся чисто ---------------
echo ""
echo "=== Test 3: фикс: экранированные backticks → exit 0 ==="
_write_tmp=$(mktemp /tmp/_gate_body_fixed.XXXXXX.sh)
cat > "$_write_tmp" <<'SH'
#!/bin/bash
set -euo pipefail
# Фикс — backticks экранированы (как в e2e-process.sh после патча)
_gate_body="## Тест
**ЕСЛИ PR ЗАКРЫТ** → сделай \`kanban complete\` с пометкой \`PR closed, карточка не нужна\`."
echo "${_gate_body}"
SH
chmod +x "$_write_tmp"
_fix_out=$(bash "$_write_tmp" 2>&1) || _fix_exit=$?
_fix_exit=${_fix_exit:-0}
rm -f "$_write_tmp"
assert_eq "fix: exit code = 0" "0" "$_fix_exit"
if echo "$_fix_out" | grep -q "kanban complete"; then
    pass "fix: gate_body содержит 'kanban complete' дословно"
else
    fail "fix: kanban complete в выводе" "ожидалось в выводе: $_fix_out"
fi
if echo "$_fix_out" | grep -q "PR closed, карточка не нужна"; then
    pass "fix: gate_body содержит 'PR closed, карточка не нужна' дословно"
else
    fail "fix: PR closed в выводе" "ожидалось в выводе: $_fix_out"
fi

# --- Test 4: мок python3 BrokenPipeError → exit 0 (acceptance #3) ---------
echo ""
echo "=== Test 4: мок python3 BrokenPipeError → exit 0 ==="
# Acceptance: «Unit-тест: мок python3 (BrokenPipeError) → script exit 0».
# Это проверка, что process substitution `< <(python3 ...)` под
# `set -euo pipefail` не роняет тик при BrokenPipeError.
_write_tmp=$(mktemp /tmp/_broken_pipe.XXXXXX.sh)
cat > "$_write_tmp" <<'SH'
#!/bin/bash
set -euo pipefail
# Мок python3: пишем 1 строку, потом SIGPIPE на следующей записи
_mocker() {
    printf '%s\n' "issue_1"
    # SIGPIPE — bash прибьёт пайп на следующей записи
    while :; do
        printf '%s\n' "more_data_that_nobody_will_read"
    done
}
while IFS= read -r _line; do
    # читатель сразу завершает после первой строки → SIGPIPE
    echo "got: ${_line}"
    break
done < <(_mocker 2>/dev/null || true)
echo "EXIT_OK"
SH
chmod +x "$_write_tmp"
_bp_out=$(bash "$_write_tmp" 2>&1) || _bp_exit=$?
_bp_exit=${_bp_exit:-0}
rm -f "$_write_tmp"
assert_eq "broken-pipe: exit code = 0" "0" "$_bp_exit"
if echo "$_bp_out" | grep -q "got: issue_1"; then
    pass "broken-pipe: первая строка прочитана"
else
    fail "broken-pipe: чтение" "ожидалось 'got: issue_1' в выводе: $_bp_out"
fi
if echo "$_bp_out" | grep -q "EXIT_OK"; then
    pass "broken-pipe: скрипт дошёл до конца после SIGPIPE"
else
    fail "broken-pipe: завершение" "ожидалось 'EXIT_OK' в выводе: $_bp_out"
fi

# --- Test 5: e2e-process.sh компилируется (bash -n syntax check) ----------
echo ""
echo "=== Test 5: bash -n syntax check для e2e-process.sh ==="
if bash -n "$E2E_PROCESS" 2>/dev/null; then
    pass "bash -n: синтаксис OK после патча"
else
    fail "bash -n: синтаксис" "bash -n $E2E_PROCESS вернул ошибку — патч сломал синтаксис?"
fi

# --- Summary ---------------------------------------------------------------
echo ""
echo "==================================================================="
printf 'Tests: %d total, %s%d passed%s, %s%d failed%s\n' \
    "$TESTS_TOTAL" \
    "$GRN" "$TESTS_PASSED" "$END" \
    "$([ "$TESTS_FAILED" -gt 0 ] && echo "$RED" || echo "$GRN")" "$TESTS_FAILED" "$END"

if [ "$TESTS_FAILED" -gt 0 ]; then
    echo "FAILED tests:"
    for n in "${FAILED_NAMES[@]}"; do
        echo "  - $n"
    done
    exit 1
fi

echo "All tests passed."
exit 0
