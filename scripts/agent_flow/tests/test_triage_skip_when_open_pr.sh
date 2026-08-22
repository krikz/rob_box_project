#!/bin/bash
# ============================================================================
# test_triage_skip_when_open_pr.sh — pre-create guard по метке branch:NAME
#                                     (ретро-фикс 22.08 t_8cde8449, issue #1506).
#
# Проверяет, что triage НЕ создаёт kanban-карточку, если на issue стоит явная
# метка `branch:<name>` и на этой ветке уже есть OPEN PR (работа в PR, дубль
# не нужен). Также регрессия: при 1 PR на issue без метки branch: — карточка
# создаётся как обычно (existing guard 584-589 ловит свой случай отдельно).
#
# Acceptance из kanban-карточки t_8cde8449:
#   1. OPEN PR на branch_name → triage не создаёт карточку, логирует skip
#   2. Issue без метки branch: → backward-compat (поведение не меняется)
#   3. Тест: scenario triage_skip_when_open_pr в scripts/agent_flow/tests/
#   4. Регрессия: при 1 PR на issue (норма) → карточка создаётся как обычно
#
# Использование:
#   bash test_triage_skip_when_open_pr.sh
# Env:
#   VERBOSE=1 — печатать подробности
# ============================================================================
set -uo pipefail

TESTS_DIR="$(cd "$(dirname "$0")" && pwd)"
SCRIPT_UNDER_TEST="$TESTS_DIR/../agent-flow-triage.sh"

PASS=0
FAIL=0
FAILED_CASES=()

# --- harness -------------------------------------------------------------
log() { if [ "${VERBOSE:-0}" = "1" ]; then printf '  %s\n' "$*"; fi; }
pass() { PASS=$((PASS+1)); printf '  \033[32m✓\033[0m %s\n' "$1"; }
fail() {
    FAIL=$((FAIL+1)); FAILED_CASES+=("$1")
    printf '  \033[31m✗\033[0m %s\n' "$1"
    if [ -n "${2:-}" ]; then printf '      %s\n' "$2"; fi
}

# Извлекаем функцию branch_label_override из скрипта как bash-исходник
extract_func() {
    local fname="$1"
    awk -v fn="$fname" '
        $0 ~ "^" fn "\\(\\)" { printflag=1; print; next }
        printflag { print; if (/^}$/) { exit} }
    ' "$SCRIPT_UNDER_TEST"
}

# --- T1: branch_label_override парсит метку -------------------------------
echo "=== T1: branch_label_override parses branch:NAME label ==="

FUNC_SRC="$(extract_func branch_label_override)"
if [ -z "$FUNC_SRC" ]; then
    fail "T1.0: branch_label_override function not found in $SCRIPT_UNDER_TEST"
    exit 1
fi

# Подключаем функцию в текущий shell
eval "$FUNC_SRC"

test_label_parse() {
    local labels="$1" expected="$2" desc="$3"
    local actual
    actual="$(branch_label_override "$labels")"
    if [ "$actual" = "$expected" ]; then
        pass "$desc"
    else
        fail "$desc" "labels='$labels' expected='$expected' got='$actual'"
    fi
}

# T1a: метка есть, возвращает имя ветки (включая формат z-{agent}/...)
test_label_parse \
    "agent:devops,branch:z-{agent}/1506-task-voice-e2e-command-gate-new-session-,hermes" \
    "z-{agent}/1506-task-voice-e2e-command-gate-new-session-" \
    "T1a: branch_label_override парсит z-{agent}/... формат"

# T1b: регистронезависимо
test_label_parse "BRANCH:z-devops/t_8cde8449-fix-thing" \
    "z-devops/t_8cde8449-fix-thing" \
    "T1b: branch_label_override case-insensitive (BRANCH → branch:)"

test_label_parse "Branch:z-arch/proposal-adr-123" \
    "z-arch/proposal-adr-123" \
    "T1c: branch_label_override case-insensitive (mixed Branch)"

# T1d: метки нет → пусто (backward-compat!)
test_label_parse "agent:devops,hermes,priority:high" \
    "" \
    "T1d: без метки branch: → пусто (backward-compat acceptance #2)"

# T1e: метка branch: в начале списка — работает
test_label_parse "branch:z-devops/foo,agent:devops" \
    "z-devops/foo" \
    "T1e: метка branch: в начале списка"

# T1f: метка branch: в конце списка — работает
test_label_parse "agent:devops,hermes,branch:z-devops/bar" \
    "z-devops/bar" \
    "T1f: метка branch: в конце списка"

# T1g: ложное срабатывание — substring "branch:" в имени другой метки не ловится
#     (мы ищем только "branch:" как полный label-prefix)
test_label_parse "mybranch:foo,agent:devops" \
    "" \
    "T1g: mybranch:foo (не branch:) → пусто (regex строгий)"

# --- T2: pre-create guard integration в основном скрипте ----------------
echo ""
echo "=== T2: pre-create guard integration в agent-flow-triage.sh ==="

# T2a: код парсит branch_label_override и использует его в guard
if grep -q 'branch_label_override' "$SCRIPT_UNDER_TEST"; then
    pass "T2a: branch_label_override вызывается из основного скрипта"
else
    fail "T2a: branch_label_override NOT called from main script"
fi

# T2b: есть переменная _branch_explicit
if grep -q '_branch_explicit=' "$SCRIPT_UNDER_TEST"; then
    pass "T2b: _branch_explicit variable present"
else
    fail "T2b: _branch_explicit variable missing"
fi

# T2c: guard проверяет OPEN PR через gh pr list --state open
# Команда разбита на 2 строки (multi-line gh call), поэтому grep multiline
if awk '/_branch_explicit=/,/^    fi/' "$SCRIPT_UNDER_TEST" | tr '\n' ' ' | grep -q 'gh pr list .* --state open'; then
    pass "T2c: guard делает gh pr list --state open (acceptance #1)"
else
    fail "T2c: guard не делает gh pr list --state open"
fi

# T2d: при срабатывании guard пишет skip-лог (acceptance #1: «логирует skip»)
if grep -A8 '_branch_explicit=' "$SCRIPT_UNDER_TEST" | grep -q 'pre-create guard'; then
    pass "T2d: guard логирует skip с упоминанием pre-create guard"
else
    fail "T2d: guard не логирует skip явно"
fi

# T2e: при срабатывании guard увеличивает skipped и continue (не падает)
if grep -A10 '_branch_explicit=' "$SCRIPT_UNDER_TEST" | grep -q 'skipped=$((skipped+1)); continue'; then
    pass "T2e: guard skipped++ + continue (не падает, не errored)"
else
    fail "T2e: guard не делает skipped++ + continue"
fi

# T2f: backward-compat — guard работает только при наличии метки (if [ -n ])
if grep -B1 -A1 '_branch_explicit=' "$SCRIPT_UNDER_TEST" | grep -q 'if \[ -n "\$_branch_explicit"'; then
    pass "T2f: guard обёрнут в 'if [ -n ... ]' — backward-compat (acceptance #2)"
else
    fail "T2f: guard НЕ conditional на наличии метки — может сломать backward-compat"
fi

# T2g: при наличии метки branch перезаписывает вычисленный $branch
# Branch label override происходит между строками с _branch_explicit= и fi.
# Используем python re для multiline (re.DOTALL).
if python3 -c '
import re
with open("'"$SCRIPT_UNDER_TEST"'") as f:
    src = f.read()
# Блок от _branch_explicit= до первого "fi" после
m = re.search(r"_branch_explicit=.*?\n\s*fi", src, re.DOTALL)
if m and "branch=\"${_branch_explicit}\"" in m.group(0):
    import sys; sys.exit(0)
sys.exit(1)
' 2>/dev/null; then
    pass "T2g: явная метка override'ит вычисленный branch (Шифу ground truth)"
else
    fail "T2g: branch не перезаписывается на explicit (фикс неполный)"
fi

# T2h: retro-fix comment в коде ссылается на t_8cde8449
if grep -q 't_8cde8449' "$SCRIPT_UNDER_TEST"; then
    pass "T2h: retro-fix comment содержит ссылку на t_8cde8449"
else
    fail "T2h: retro-fix comment без ссылки на t_8cde8449"
fi

# --- T3: regression — backward-compat, существующий OPEN-PR guard ---------
echo ""
echo "=== T3: regression — backward-compat (acceptance #2, #4) ==="

# T3a: существующий OPEN-PR guard (584-589) для вычисленного branch_for()
#      сохранён (мы не сломали regress)
if grep -q 'reopened-loop guard' "$SCRIPT_UNDER_TEST"; then
    pass "T3a: старый OPEN-PR guard (584-589) сохранён (не сломали regress)"
else
    fail "T3a: старый OPEN-PR guard отсутствует — regress!"
fi

# T3b: branch_for() не изменился
ORIG_BRANCH_FOR="$(git show origin/develop:scripts/agent_flow/agent-flow-triage.sh | grep -A4 '^branch_for()')"
NEW_BRANCH_FOR="$(grep -A4 '^branch_for()' "$SCRIPT_UNDER_TEST")"
if [ "$ORIG_BRANCH_FOR" = "$NEW_BRANCH_FOR" ]; then
    pass "T3b: branch_for() не изменился (backward-compat для старого guard)"
else
    fail "T3b: branch_for() изменён — мог сломать существующее поведение"
fi

# T3c: throttle v3.2 (recent_line) сохранён
if grep -q '_recent_line=' "$SCRIPT_UNDER_TEST"; then
    pass "T3c: throttle v3.2 (_recent_line) сохранён"
else
    fail "T3c: throttle v3.2 потерян — regress!"
fi

# --- T4: bash syntax + shellcheck -----------------------------------------
echo ""
echo "=== T4: bash syntax + shellcheck ==="

# Syntax check
if bash -n "$SCRIPT_UNDER_TEST" 2>/tmp/sc_err; then
    pass "T4a: bash -n syntax check passed"
else
    fail "T4a: bash -n syntax check failed" "$(cat /tmp/sc_err)"
fi

# Shellcheck baseline (compare with origin/develop — no NEW warnings)
if ! command -v shellcheck >/dev/null 2>&1; then
    for sc in /home/builder/.hermes/hermes-agent/venv/bin/shellcheck \
             /usr/local/bin/shellcheck /usr/bin/shellcheck; do
        [ -x "$sc" ] && PATH="$(dirname "$sc"):$PATH" && break
    done
fi
if command -v shellcheck >/dev/null 2>&1; then
    _repo_root="$(git -C "$TESTS_DIR/.." rev-parse --show-toplevel 2>/dev/null || echo "$TESTS_DIR/..")"
    if [ -d "$_repo_root/.git" ] || [ -f "$_repo_root/.git" ]; then
        ORIG_SC="$(cd "$_repo_root" && git show "origin/develop:scripts/agent_flow/agent-flow-triage.sh" 2>/dev/null | shellcheck - 2>&1 | wc -l)"
        NEW_SC="$(shellcheck "$SCRIPT_UNDER_TEST" 2>&1 | wc -l)"
        log "  shellcheck: origin/develop=$ORIG_SC, current=$NEW_SC"
        if [ "$NEW_SC" -le "$ORIG_SC" ]; then
            pass "T4b: shellcheck — no NEW warnings (origin=$ORIG_SC, current=$NEW_SC)"
        else
            DIFF="$(shellcheck "$SCRIPT_UNDER_TEST" 2>&1)"
            fail "T4b: shellcheck — $((NEW_SC-ORIG_SC)) new warning(s)" "$(printf '%s\n' "$DIFF" | head -30)"
        fi
    else
        log "T4b: not a git repo — skipping baseline comparison"
    fi
else
    log "T4b: shellcheck not installed — skip"
fi

# --- summary -------------------------------------------------------------
echo ""
echo "============================================================"
echo "PASS: $PASS    FAIL: $FAIL"
echo "============================================================"
if [ "$FAIL" -gt 0 ]; then
    printf '\nFAILED CASES:\n'
    for c in "${FAILED_CASES[@]}"; do printf '  - %s\n' "$c"; done
    exit 1
fi
exit 0
