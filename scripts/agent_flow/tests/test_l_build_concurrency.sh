#!/usr/bin/env bash
# ============================================================================
# test_l_build_concurrency.sh — regression для issue #1560 (develop spam-runs)
#                                  + issue #1694 (concurrency removal)
#
# Проверяет, что L-Build-All-Services.yml:
#   1. НЕ имеет concurrency: блок (issue #1694: concurrency вешал builds
#      в очередь без движения — cancel-in-progress=false + нет active run → deadlock)
#   2. Имеет job `validate-trigger` (ЕДИНСТВЕННАЯ защита от spam после #1694)
#   3. validate-trigger.if содержит `github.event_name == 'workflow_dispatch'`
#   4. validate-trigger.if содержит github.ref == develop/main (НЕ на round-ветках)
#   5. validate-trigger.continue-on-error == true (НЕ блокирует build)
#   6. validate-trigger имеет step с проверкой triggered_by_script == 'manual'
#   7. Имеет исходные inputs: build_base_images, push_to_registry
#   8. Имеет triggered_by_script input
#
# Запуск:
#   bash scripts/agent_flow/tests/test_l_build_concurrency.sh
#   или из pytest: tests/test_workflow_concurrency.py (если есть wrapper)
# ============================================================================
set -euo pipefail

REPO_DIR="${REPO_DIR:-/home/builder/rob_box_project}"
WF_FILE="$REPO_DIR/.github/workflows/L-Build-All-Services.yml"

if [ ! -f "$WF_FILE" ]; then
    # Fallback для worktree-режима
    WF_FILE="$REPO_DIR/.worktrees/t_9c5a0b3b/.github/workflows/L-Build-All-Services.yml"
fi

if [ ! -f "$WF_FILE" ]; then
    # Второй fallback: ищем по worktree из текущей директории
    WF_FILE="$(git rev-parse --show-toplevel 2>/dev/null)/.github/workflows/L-Build-All-Services.yml"
fi

if [ ! -f "$WF_FILE" ]; then
    echo "FAIL: $WF_FILE не найден"
    exit 1
fi

fail_count=0
pass_count=0

check() {
    local name="$1"
    local pattern="$2"
    if grep -qE "$pattern" "$WF_FILE"; then
        echo "PASS: $name"
        pass_count=$((pass_count+1))
    else
        echo "FAIL: $name (pattern: $pattern)"
        fail_count=$((fail_count+1))
    fi
}

check_not() {
    local name="$1"
    local pattern="$2"
    if grep -qE "$pattern" "$WF_FILE"; then
        echo "FAIL: $name (pattern должен отсутствовать, но найден: $pattern)"
        fail_count=$((fail_count+1))
    else
        echo "PASS: $name (отсутствует, как и требуется)"
        pass_count=$((pass_count+1))
    fi
}

# Test 1-2: issue #1694 — concurrency block ДОЛЖЕН отсутствовать
check_not "concurrency block отсутствует (issue #1694)" "^concurrency:"
# sanity: комментарии, объясняющие почему concurrency убран — должны быть (issue #1694 marker)
check "есть комментарий issue #1694 про удаление concurrency" "Issue #1694.*concurrency"

# Test 3-8: validate-trigger на месте (ЕДИНСТВЕННАЯ защита после #1694)
check "validate-trigger job exists" "^[[:space:]]+validate-trigger:"
check "validate-trigger checks workflow_dispatch event" "github.event_name[[:space:]]*==[[:space:]]*'workflow_dispatch'"
check "validate-trigger checks develop ref" "github.ref[[:space:]]*==[[:space:]]*'refs/heads/develop'"
check "validate-trigger checks main ref" "github.ref[[:space:]]*==[[:space:]]*'refs/heads/main'"
check "validate-trigger is continue-on-error" "continue-on-error:[[:space:]]+true"
check "validate-trigger detects triggered_by_script == manual" 'TRIGGERED_BY_SCRIPT.*"manual"'
check "validate-trigger checks triggered_by_card empty" '\[[[:space:]]*-z[[:space:]]+"\$\{TRIGGERED_BY_CARD\}"'

# Test 9-11: inputs intact (regression — не сломали существующие inputs)
check "input build_base_images present" "build_base_images:"
check "input push_to_registry present" "push_to_registry:"
check "input triggered_by_script present" "triggered_by_script:"

# Test 12: original jobs (build-base-images etc) still exist
check "build-base-images job intact" "build-base-images:"
check "build-main-services job intact" "build-main-services:"
check "build-vision-services job intact" "build-vision-services:"
check "summary job intact" "^[[:space:]]+summary:"

# Test 13: yaml is valid python yaml
if python3 -c "import yaml; yaml.safe_load(open('$WF_FILE'))" 2>/dev/null; then
    echo "PASS: yaml.safe_load OK"
    pass_count=$((pass_count+1))
else
    echo "FAIL: yaml.safe_load ERROR"
    fail_count=$((fail_count+1))
fi

# Test 14: validate-trigger.if has BOTH workflow_dispatch AND (develop OR main) conditions
# (anti-pattern: forgot the OR, all workflow_dispatch on feature/* would be validated)
_validate_if="$(grep -A 3 'name: ".*Trigger source validation' "$WF_FILE" | grep '^[[:space:]]*if:' || true)"
_validate_if="${_validate_if:-$(grep 'name: ".*Trigger source validation' -A 10 "$WF_FILE" | grep '^[[:space:]]*if:' || true)}"
if echo "$_validate_if" | grep -q "workflow_dispatch" \
   && echo "$_validate_if" | grep -q "refs/heads/develop" \
   && echo "$_validate_if" | grep -q "refs/heads/main"; then
    echo "PASS: validate-trigger.if has workflow_dispatch AND (develop OR main)"
    pass_count=$((pass_count+1))
else
    echo "FAIL: validate-trigger.if missing one of: workflow_dispatch / refs/heads/develop / refs/heads/main"
    echo "      actual: $_validate_if"
    fail_count=$((fail_count+1))
fi

echo ""
echo "============================="
echo "Results: $pass_count passed, $fail_count failed"
echo "============================="

if [ "$fail_count" -gt 0 ]; then
    exit 1
fi
exit 0
