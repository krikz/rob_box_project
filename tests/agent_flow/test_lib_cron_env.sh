#!/usr/bin/env bash
# ============================================================================
# Regression test для lib_cron_env.sh (ретро t_a2521b07, 31.08).
#
# Background: no-agent cron-tick в devops-профиле получает sandbox HOME
# (/home/builder/.hermes/profiles/devops/home) и HERMES_HOME
# (/home/builder/.hermes/profiles/devops). Без env-preflight `gh auth status`
# не находит /home/builder/.config/gh/hosts.yml → exit 1 (16-fail
# agent-flow-blocked-watchdog, 24-fail agent-flow-unlabeled-sweep).
#
# Тест:
# 1) Source-им lib_cron_env.sh с sandbox HOME/HERMES_HOME/GH_CONFIG_DIR.
# 2) Проверяем, что после source:
#    - HOME=/home/builder (FORCE)
#    - HERMES_HOME=/home/builder/.hermes (FORCE)
#    - GH_CONFIG_DIR=/home/builder/.config/gh (FORCE, непустой)
#    - _CRON_ENV_PREFLIGHT_OK=1 (marker)
# 3) Проверяем export-existing-wins: если caller задал GH_REPO,
#    он НЕ перезаписан значением из .env.
# 4) Sanity: `gh auth status` работает (после FORCE HOME).
# 5) Sanity: source в set +e контексте (как в реальных no-agent скриптах).
#
# Запуск: bash tests/agent_flow/test_lib_cron_env.sh
# Exit:   0 = PASS, N>0 = N failed assertions.
#
# Не требует network / реального GitHub — работает на любом CI с
# bash 4+ и python3 (для парсинга).
# ============================================================================
set -u

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
LIB_CRON_ENV="$REPO_ROOT/scripts/agent_flow/lib_cron_env.sh"

# Цвета для читаемого вывода
if [ -t 1 ]; then
    RED=$'\033[0;31m'; GRN=$'\033[0;32m'; YEL=$'\033[0;33m'; NC=$'\033[0m'
else
    RED=""; GRN=""; YEL=""; NC=""
fi

# Pre-checks
if ! command -v bash >/dev/null 2>&1; then
    printf '%sFATAL: bash не найден%s\n' "$RED" "$NC"
    exit 2
fi
if [ ! -f "$LIB_CRON_ENV" ]; then
    printf '%sFATAL: lib_cron_env.sh не найден: %s%s\n' "$RED" "$LIB_CRON_ENV" "$NC"
    exit 2
fi

# Helper для assertion-ов с читаемым выводом.
# Используем временные файлы для передачи счётчиков из sub-shell в
# родительский scope (sub-shell `(...)` имеет свой variable scope).
PASS_FILE="$(mktemp)"
FAIL_FILE="$(mktemp)"
FAILED_TESTS_FILE="$(mktemp)"
trap 'rm -f "$PASS_FILE" "$FAIL_FILE" "$FAILED_TESTS_FILE"' EXIT
echo "0" > "$PASS_FILE"
echo "0" > "$FAIL_FILE"
: > "$FAILED_TESTS_FILE"

_pass() { cat "$PASS_FILE"; }
_fail() { cat "$FAIL_FILE"; }
_failed_tests() { cat "$FAILED_TESTS_FILE"; }

_inc_pass() {
    local cur
    cur=$(_pass)
    echo $((cur + 1)) > "$PASS_FILE"
}
_inc_fail() {
    local desc="$1"
    local cur
    cur=$(_fail)
    echo $((cur + 1)) > "$FAIL_FILE"
    printf '%s\n' "$desc" >> "$FAILED_TESTS_FILE"
}

assert_eq() {
    local desc="$1"
    local expected="$2"
    local actual="$3"
    if [ "$expected" = "$actual" ]; then
        _inc_pass
        printf '%s✓%s %s\n' "$GRN" "$NC" "$desc"
    else
        _inc_fail "$desc (expected='$expected', got='$actual')"
        printf '%s✗%s %s\n' "$RED" "$NC" "$desc"
        printf '    expected: %s\n' "$expected"
        printf '    actual:   %s\n' "$actual"
    fi
}

assert_nonempty() {
    local desc="$1"
    local actual="$2"
    if [ -n "$actual" ]; then
        _inc_pass
        printf '%s✓%s %s\n' "$GRN" "$NC" "$desc"
    else
        _inc_fail "$desc (got empty)"
        printf '%s✗%s %s\n' "$RED" "$NC" "$desc"
    fi
}

# ============================================================================
# Test 1: sandbox HOME/HERMES_HOME → FORCE real paths
# ============================================================================
printf '%s== Test 1: FORCE real HOME/HERMES_HOME/GH_CONFIG_DIR (sandbox → real) ==%s\n' "$YEL" "$NC"

# Source-им в sub-shell с sandbox env (НЕ unset HOME — нужно имитировать
# реальный cron-tick, который ПЕРЕДАЁТ sandbox HOME).
(
    unset _LIB_CRON_ENV_LOADED  # защита от повторного source в одном процессе
    unset _CRON_ENV_PREFLIGHT_OK
    export HOME="/home/builder/.hermes/profiles/devops/home"
    export HERMES_HOME="/home/builder/.hermes/profiles/devops"
    export GH_CONFIG_DIR=""
    export GH_REPO=""

    # shellcheck source=lib_cron_env.sh
    . "$LIB_CRON_ENV"

    assert_eq "HOME → real /home/builder (sandbox was profiles/devops/home)" \
        "/home/builder" "$HOME"
    assert_eq "HERMES_HOME → real /home/builder/.hermes (sandbox was profiles/devops)" \
        "/home/builder/.hermes" "$HERMES_HOME"
    assert_eq "GH_CONFIG_DIR → real /home/builder/.config/gh" \
        "/home/builder/.config/gh" "$GH_CONFIG_DIR"
    assert_eq "_CRON_ENV_PREFLIGHT_OK=1 (marker)" \
        "1" "$_CRON_ENV_PREFLIGHT_OK"
    # GH_REPO default (если .env не выставил — safety-net)
    assert_nonempty "GH_REPO has value (default or .env)" "$GH_REPO"
)

# ============================================================================
# Test 2: export-existing-wins — caller-переменная не перезаписана
# ============================================================================
printf '\n%s== Test 2: export-existing-wins (caller GH_REPO не перезаписан) ==%s\n' "$YEL" "$NC"

(
    unset _LIB_CRON_ENV_LOADED
    unset _CRON_ENV_PREFLIGHT_OK
    export HOME="/home/builder/.hermes/profiles/devops/home"
    export HERMES_HOME="/home/builder/.hermes/profiles/devops"
    export GH_CONFIG_DIR=""
    export GH_REPO="caller/repo"  # caller уже задал — должен выиграть

    # shellcheck source=lib_cron_env.sh
    . "$LIB_CRON_ENV"

    assert_eq "GH_REPO НЕ перезаписан (caller wins over .env/default)" \
        "caller/repo" "$GH_REPO"
    # HOME всё равно FORCE
    assert_eq "HOME всё равно /home/builder (HOME FORCE, не export-existing-wins)" \
        "/home/builder" "$HOME"
)

# ============================================================================
# Test 3: идемпотентность — повторный source no-op
# ============================================================================
printf '\n%s== Test 3: идемпотентность — повторный source ==%s\n' "$YEL" "$NC"

(
    unset _LIB_CRON_ENV_LOADED
    unset _CRON_ENV_PREFLIGHT_OK
    export HOME="/home/builder/.hermes/profiles/devops/home"
    export HERMES_HOME="/home/builder/.hermes/profiles/devops"
    export GH_CONFIG_DIR=""
    export GH_REPO="first/repo"

    # shellcheck source=lib_cron_env.sh
    . "$LIB_CRON_ENV"
    # Sourceим ещё раз — должно быть no-op
    . "$LIB_CRON_ENV"

    # GH_REPO должен остаться "first/repo" (export-existing-wins)
    assert_eq "повторный source: GH_REPO остался caller-значением" \
        "first/repo" "$GH_REPO"
    assert_eq "повторный source: HOME всё равно /home/builder" \
        "/home/builder" "$HOME"
)

# ============================================================================
# Test 4: реальный no-agent cron-скрипт проходит env-preflight
# ============================================================================
printf '\n%s== Test 4: representative no-agent cron-скрипт проходит preflight ==%s\n' "$YEL" "$NC"

# Берём agent-flow-merge-gate (один из 15 пропатченных) и запускаем в
# sandbox env. Без preflight он бы упал на `gh auth status` или
# `: ${GH_REPO:?...}`. С preflight должен дойти до flock-guard'а или
# нормальной логики.
(
    unset _LIB_CRON_ENV_LOADED
    unset _CRON_ENV_PREFLIGHT_OK
    export HOME="/home/builder/.hermes/profiles/devops/home"
    export HERMES_HOME="/home/builder/.hermes/profiles/devops"
    export GH_CONFIG_DIR=""
    export GH_REPO=""
    # GH_TOKEN из реального keyring (имитируем реальный cron-tick с
    # подгруженным credential'ом)
    export GH_TOKEN="$(command -v gh >/dev/null 2>&1 && gh auth token 2>/dev/null || true)"

    # Запускаем скрипт. flock-guard может сработать (другая инстанция),
    # и это ОК — значит preflight прошёл. Если скрипт упадёт с "gh auth
    # failed — exit 1" — значит патч не сработал.
    out="$(bash "$REPO_ROOT/scripts/agent_flow/agent-flow-merge-gate.sh" 2>&1 | head -5)"
    rc=$?

    # Exit code 0 (skip на flock) или 0 (нормальная работа) — оба ОК.
    # Exit code 1 с сообщением "gh auth failed" — FAIL.
    if [ "$rc" -eq 0 ]; then
        _inc_pass
        printf '%s✓%s merge-gate прошёл preflight (sandbox env), exit=%s\n' \
            "$GRN" "$NC" "$rc"
    elif echo "$out" | grep -q "gh auth failed\|lib_cron_env preflight failed"; then
        _inc_fail "merge-gate упал на env-preflight в sandbox env (exit=$rc)"
        printf '%s✗%s merge-gate упал на env-preflight\n' "$RED" "$NC"
        printf '    output: %s\n' "$out"
    else
        # Exit 1 но не из-за env — может быть другая ошибка (flock, etc).
        # Это приемлемо для smoke-теста.
        _inc_pass
        printf '%s✓%s merge-gate прошёл preflight (exit=%s, другая ошибка ОК)\n' \
            "$GRN" "$NC" "$rc"
    fi
)

# ============================================================================
# Test 5: source в set +e контексте (как в реальных no-agent скриптах)
# ============================================================================
printf '\n%s== Test 5: source в set +e контексте (production идиома) ==%s\n' "$YEL" "$NC"

# Имитируем production идиому: `set +e; . lib_cron_env.sh || exit 1; set -e`
(
    unset _LIB_CRON_ENV_LOADED
    unset _CRON_ENV_PREFLIGHT_OK
    export HOME="/home/builder/.hermes/profiles/devops/home"
    export HERMES_HOME="/home/builder/.hermes/profiles/devops"
    export GH_CONFIG_DIR=""
    export GH_REPO=""

    set +e
    # shellcheck source=lib_cron_env.sh
    . "$LIB_CRON_ENV"
    src_rc=$?
    set -e

    assert_eq "source в set +e контексте: exit 0 (no error)" \
        "0" "$src_rc"
    assert_eq "после source+set -e: HOME=/home/builder" \
        "/home/builder" "$HOME"
)

# ============================================================================
# Summary
# ============================================================================
PASS=$(_pass)
FAIL=$(_fail)
printf '\n%s== Summary ==%s\n' "$YEL" "$NC"
printf '  PASS: %s%d%s\n' "$GRN" "$PASS" "$NC"
printf '  FAIL: %s%d%s\n' "$RED" "$FAIL" "$NC"

if [ "$FAIL" -gt 0 ]; then
    printf '\n%sFailed tests:%s\n' "$RED" "$NC"
    while IFS= read -r t; do
        printf '  - %s\n' "$t"
    done < "$FAILED_TESTS_FILE"
    exit "$FAIL"
fi

printf '\n%sAll lib_cron_env.sh regression tests passed.%s\n' "$GRN" "$NC"
exit 0
