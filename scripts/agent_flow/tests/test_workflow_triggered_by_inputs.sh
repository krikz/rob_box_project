#!/bin/bash
# ============================================================================
# test_workflow_triggered_by_inputs.sh — issue #1527
#
# Smoke-проверка контракта: workflow L-Build-All-Services.yml,
# L-Deploy and Verify.yml, L-E2E Voice Test.yml должны принимать
# 4 optional dispatch inputs (triggered_by_script, triggered_by_agent,
# triggered_by_card, triggered_by_reason) и emit-строку
# "🔧 Triggered by: ..." в первом job (через env+tr-d, не bare).
#
# Контракт (из issue body):
#   - defaults работают (ручной gh workflow run без -f — OK)
#   - скрипты пробрасывают 4 поля через -f (в bash через --field)
#   - CI job log показывает "🔧 Triggered by: ..."
#
# Run:
#   bash scripts/agent_flow/tests/test_workflow_triggered_by_inputs.sh
# ============================================================================
set -euo pipefail

TEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$TEST_DIR/../../.." && pwd)"

TESTS_TOTAL=0
TESTS_PASSED=0
TESTS_FAILED=0
FAILED_NAMES=()

if [ -t 1 ]; then
    RED=$'\033[31m'; GRN=$'\033[32m'; END=$'\033[0m'
else
    RED=''; GRN=''; END=''
fi

pass() {
    TESTS_TOTAL=$((TESTS_TOTAL+1))
    TESTS_PASSED=$((TESTS_PASSED+1))
    printf '  %s✓%s %s\n' "$GRN" "$END" "$1"
}
fail() {
    TESTS_TOTAL=$((TESTS_TOTAL+1))
    TESTS_FAILED=$((TESTS_FAILED+1))
    FAILED_NAMES+=("$1")
    printf '  %s✗%s %s — %s\n' "$RED" "$END" "$1" "$2"
}

assert_grep() {
    local name="$1" file="$2" pattern="$3"
    if grep -qE "$pattern" "$file"; then
        pass "$name"
    else
        fail "$name" "pattern not found in $file: $pattern"
    fi
}

# Список трёх целевых workflow
WORKFLOWS=(
    "$REPO_ROOT/.github/workflows/L-Build-All-Services.yml"
    "$REPO_ROOT/.github/workflows/L-Deploy and Verify.yml"
    "$REPO_ROOT/.github/workflows/L-E2E Voice Test.yml"
)

# --- Test 1: каждый workflow имеет 4 inputs в workflow_dispatch.inputs ------
echo "=== Test 1: 4 inputs triggered_by_* в workflow_dispatch ==="
for wf in "${WORKFLOWS[@]}"; do
    wf_name="$(basename "$wf")"
    for inp in triggered_by_script triggered_by_agent triggered_by_card triggered_by_reason; do
        if grep -qE "^\s+${inp}:" "$wf"; then
            pass "$wf_name имеет input $inp"
        else
            fail "$wf_name / $inp" "input отсутствует в workflow_dispatch.inputs"
        fi
    done
done

# --- Test 2: каждый workflow имеет emit-шаг "🔧 Triggered by: ..." ----------
echo ""
echo "=== Test 2: emit-шаг Trigger source в каждом workflow ==="
for wf in "${WORKFLOWS[@]}"; do
    wf_name="$(basename "$wf")"
    if grep -q "Trigger source" "$wf"; then
        pass "$wf_name имеет step 'Trigger source'"
    else
        fail "$wf_name" "step 'Trigger source' отсутствует"
    fi
    # emit-строка идёт через env (CRLF-safety) + tr -d, не bare ${{ inputs.* }}
    # Согласно ретро t_4cf2811e: ${{ inputs.* }} в bash-контексте ЗАПРЕЩЕНО.
    if grep -qE "Triggered by:" "$wf"; then
        # Проверяем, что в этом run-блоке нет bare ${{ inputs.triggered_by_*
        # (используем тот же подход, что и test_e2e_voice_workflow_crlf_inputs.sh)
        if awk '
            /run: \|/ { in_run=1; next }
            /^      - name:/ { in_run=0 }
            in_run && /\${{\s*inputs\.(triggered_by_|env-script|env-agent|env-card|env-reason)/ { print "BAD: " $0; bad=1 }
            END { exit bad }
        ' "$wf"; then
            pass "$wf_name: bare \${{ inputs.* }} НЕ используется в bash-контексте (CRLF-safe)"
        else
            fail "$wf_name: bare inputs" "найден bare \${{ inputs.* }} в run: | — нужен env+tr-d"
        fi
    else
        fail "$wf_name" "строка 'Triggered by:' отсутствует"
    fi
done

# --- Test 3: defaults присутствуют (ручной запуск без -f должен работать) ---
echo ""
echo "=== Test 3: defaults у inputs (ручной gh workflow run без -f) ==="
# Парсим YAML минимальным awk: когда видим строку `      triggered_by_<X>:`
# (наш уровень indent), начинаем искать default внутри блока. Заканчиваем
# когда встречаем строку того же уровня с другим ключом или менее
# индентированную.
for wf in "${WORKFLOWS[@]}"; do
    wf_name="$(basename "$wf")"
    if awk '
        /^[[:space:]]+triggered_by_script:[[:space:]]*$/ {found=1; next}
        found && /default:[[:space:]]*["'\'']?manual["'\'']?[[:space:]]*$/ {ok=1; exit}
        found && /^[[:space:]]+[a-z_]+:[[:space:]]*$/ {exit}
        END {exit !ok}
    ' "$wf"; then
        pass "$wf_name: triggered_by_script default=manual"
    else
        fail "$wf_name" "triggered_by_script default != manual (или отсутствует)"
    fi
    if awk '
        /^[[:space:]]+triggered_by_agent:[[:space:]]*$/ {found=1; next}
        found && /default:[[:space:]]*["'\'']?unknown["'\'']?[[:space:]]*$/ {ok=1; exit}
        found && /^[[:space:]]+[a-z_]+:[[:space:]]*$/ {exit}
        END {exit !ok}
    ' "$wf"; then
        pass "$wf_name: triggered_by_agent default=unknown"
    else
        fail "$wf_name" "triggered_by_agent default != unknown (или отсутствует)"
    fi
done

# --- Test 4: скрипты пробрасывают 4 поля через --field -----------------------
echo ""
echo "=== Test 4: скрипты пробрасывают triggered_by_* в gh workflow run ==="
PMB="$REPO_ROOT/scripts/agent_flow/agent-flow-post-merge-build.sh"
E2EP="$REPO_ROOT/scripts/agent_flow/agent-flow-e2e-process.sh"

# post-merge-build: 4 поля должны быть в вызове gh workflow run
for fld in triggered_by_script triggered_by_agent triggered_by_card triggered_by_reason; do
    if grep -qE -- "--field ${fld}=" "$PMB"; then
        pass "agent-flow-post-merge-build.sh пробрасывает ${fld}"
    else
        fail "agent-flow-post-merge-build.sh / ${fld}" "поле не пробрасывается через --field"
    fi
done

# e2e-process: функция _trigger_workflow_with_retry должна содержать 4 поля.
# В скрипте они пробрасываются через локальные переменные _TBS/_TBA/_TBC/_TBR.
# Проверяем через awk, что внутри функции есть все 4 пары field=variable.
declare -A E2E_VARS=(
    [triggered_by_script]="_TBS"
    [triggered_by_agent]="_TBA"
    [triggered_by_card]="_TBC"
    [triggered_by_reason]="_TBR"
)
for fld in triggered_by_script triggered_by_agent triggered_by_card triggered_by_reason; do
    var="${E2E_VARS[$fld]}"
    if awk -v field="$fld" -v var="$var" '
        BEGIN { found_field=0; found_var=0 }
        /_trigger_workflow_with_retry\(\)/ { in_func=1; next }
        in_func && /^_trigger_workflow_with_retry\(\) \{/ { in_func=1; next }
        in_func && $0 ~ "--field " field "=" { found_field=1 }
        in_func && index($0, "$" var) > 0 { found_var=1 }
        END { exit !(found_field && found_var) }
    ' "$E2EP"; then
        pass "agent-flow-e2e-process.sh пробрасывает ${fld} (через ${var})"
    else
        fail "agent-flow-e2e-process.sh / ${fld}" "нет пары --field ${fld}=...${var} в _trigger_workflow_with_retry"
    fi
done

# --- Test 5: bash -n syntax check -------------------------------------------
echo ""
echo "=== Test 5: bash -n syntax для изменённых скриптов ==="
if bash -n "$PMB"; then
    pass "agent-flow-post-merge-build.sh: синтаксис OK"
else
    fail "agent-flow-post-merge-build.sh: bash -n failed"
fi
if bash -n "$E2EP"; then
    pass "agent-flow-e2e-process.sh: синтаксис OK"
else
    fail "agent-flow-e2e-process.sh: bash -n failed"
fi

# --- Итоги -------------------------------------------------------------------
echo ""
echo "=== Итоги ==="
echo "Всего: $TESTS_TOTAL  passed: $TESTS_PASSED  failed: $TESTS_FAILED"
if [ "$TESTS_FAILED" -gt 0 ]; then
    echo "Провалы:"
    for n in "${FAILED_NAMES[@]}"; do echo "  - $n"; done
    exit 1
fi
