#!/usr/bin/env bash
# ============================================================================
# test_hermes_home_default_convention.sh — регресс-гард issue #2487.
#
# ПРОБЛЕМА (issue #2487): README обещает HERMES_HOME по умолчанию `~/.hermes`
# (т.е. `${HOME}/.hermes`), но ~30 скриптов hardcode-или дефолт как
# `/home/builder/.hermes`. На хосте, где Hermes не лежит в /home/builder
# (CI-раннеры, свежие builder'ы, разработчик с нестандартным путём), это
# тихо ломает пути к state-файлам (cooldown/pause и т.д.).
#
# КОНВЕНЦИЯ (после фикса issue #2487): дефолт HERMES_HOME — ВСЕГДА
# `${HOME}/.hermes` (или его подстановочные варианты), никогда не
# хардкоженный `/home/builder/.hermes`. Проверяем два известных идиома
# регресса:
#   (a) `${HERMES_HOME:-/home/builder/.hermes}` — обычный bash-дефолт;
#   (b) `[ -n "${HERMES_HOME:-}" ] || HERMES_HOME=/home/builder/.hermes`
#       — guard-идиом (см. историю hermes_github.sh).
#
# НЕ проверяем (сознательно, вне scope issue #2487): безусловные
# `HERMES_HOME=/home/builder/.hermes` без `:-`/`||` в
# agent-flow-e2e-process.sh, agent-flow-handoff.sh, agent-flow-merge-gate.sh,
# agent-flow-post-merge-build.sh — там HERMES_HOME намеренно ИГНОРИРУЕТ
# входящий env (а не дефолтит при пустом), потому что cron per-profile
# gateway может передать profile-relative HERMES_HOME/HOME, и PROFILE_ENV
# должен указывать на реальный host install (см. комментарии в этих
# файлах). Это другой контракт — не «дефолт», а «форс» — и правка issue
# #2487 explicитно не должна его трогать («не меняй поведение при явно
# заданном HERMES_HOME — только дефолт»).
#
# Run:
#   bash scripts/agent_flow/tests/test_hermes_home_default_convention.sh
# ============================================================================
set -uo pipefail

TEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "$TEST_DIR/.." && pwd)"

fail_count=0
fail_log=""

check_pattern() {
    local label="$1" pattern="$2"
    local hits
    # Только top-level *.sh скрипты agent_flow (не tests/, не подкаталоги).
    hits="$(grep -nE "$pattern" "$ROOT_DIR"/*.sh 2>/dev/null || true)"
    if [ -n "$hits" ]; then
        fail_count=$((fail_count+1))
        fail_log="${fail_log}FAIL: найден хардкод-регресс ($label):
${hits}

"
    fi
}

# (a) обычный bash-дефолт `${HERMES_HOME:-/home/builder/.hermes}`.
check_pattern "HERMES_HOME:-/home/builder default" 'HERMES_HOME:-/home/builder'

# (b) guard-идиом `... || HERMES_HOME=/home/builder/.hermes`.
check_pattern "guard-idiom HERMES_HOME=/home/builder" '\|\|[[:space:]]*HERMES_HOME=/home/builder'

# Sanity: конвенция реально используется хоть где-то (иначе тест — no-op).
convention_hits="$(grep -lE 'HERMES_HOME:-\$\{HOME\}/\.hermes|HERMES_HOME:-\$HOME/\.hermes' "$ROOT_DIR"/*.sh 2>/dev/null | wc -l)"
if [ "$convention_hits" -lt 1 ]; then
    fail_count=$((fail_count+1))
    fail_log="${fail_log}FAIL: не нашли ни одного скрипта с конвенцией \${HERMES_HOME:-\${HOME}/.hermes} —
подозрение, что сам grep сломан или конвенция исчезла.

"
fi

echo "=== test_hermes_home_default_convention.sh ==="
if [ "$fail_count" -eq 0 ]; then
    echo "PASS: ни один agent_flow-скрипт не хардкодит /home/builder в дефолте HERMES_HOME"
    echo "      (конвенция \${HERMES_HOME:-\${HOME}/.hermes} найдена в $convention_hits файле(ах))"
    exit 0
else
    printf '%s' "$fail_log"
    echo "SUMMARY: $fail_count проверка(и) провалена(ы)"
    exit 1
fi
