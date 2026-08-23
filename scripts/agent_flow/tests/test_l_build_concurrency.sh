#!/usr/bin/env bash
# ============================================================================
# test_l_build_concurrency.sh — regression для issue #1560 (develop spam-runs)
#
# Проверяет, что L-Build-All-Services.yml:
#   1. Имеет concurrency: блок с group, зависящим от github.ref
#   2. cancel-in-progress == false (НЕ убиваем легитимный build в полёте)
#   3. Имеет job `validate-trigger`
#   4. validate-trigger.if содержит `github.event_name == 'workflow_dispatch'`
#   5. validate-trigger.if содержит github.ref == develop/main (НЕ на round-ветках)
#   6. validate-trigger.continue-on-error == true (НЕ блокирует build)
#   7. validate-trigger имеет step с проверкой triggered_by_script == 'manual'
#   8. Имеет исходные inputs: build_base_images, push_to_registry
#   9. Имеет triggered_by_script input
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

# Test 1-7: presence checks
check "concurrency block exists" "^concurrency:"
check "cancel-in-progress: false" "cancel-in-progress:[[:space:]]+false"
check "concurrency group uses github.workflow + github.ref" "group:[[:space:]]+.*github.workflow.*github.ref"
check "validate-trigger job exists" "^[[:space:]]+validate-trigger:"
check "validate-trigger checks workflow_dispatch event" "github.event_name[[:space:]]*==[[:space:]]*'workflow_dispatch'"
check "validate-trigger checks develop ref" "github.ref[[:space:]]*==[[:space:]]*'refs/heads/develop'"
check "validate-trigger checks main ref" "github.ref[[:space:]]*==[[:space:]]*'refs/heads/main'"
check "validate-trigger is continue-on-error" "continue-on-error:[[:space:]]+true"
check "validate-trigger detects triggered_by_script == manual" 'TRIGGERED_BY_SCRIPT.*"manual"'
check "validate-trigger checks triggered_by_card empty" '\[[[:space:]]*-z[[:space:]]+"\$\{TRIGGERED_BY_CARD\}"'

# Test 8-9: inputs intact (regression — не сломали существующие inputs)
check "input build_base_images present" "build_base_images:"
check "input push_to_registry present" "push_to_registry:"
check "input triggered_by_script present" "triggered_by_script:"

# Test 10: original jobs (build-base-images etc) still exist
check "build-base-images job intact" "build-base-images:"
check "build-main-services job intact" "build-main-services:"
check "build-vision-services job intact" "build-vision-services:"
check "summary job intact" "^[[:space:]]+summary:"

# Test 11: yaml is valid python yaml
if python3 -c "import yaml; yaml.safe_load(open('$WF_FILE'))" 2>/dev/null; then
    echo "PASS: yaml.safe_load OK"
    pass_count=$((pass_count+1))
else
    echo "FAIL: yaml.safe_load ERROR"
    fail_count=$((fail_count+1))
fi

# Test 12: validate-trigger.if has BOTH workflow_dispatch AND (develop OR main) conditions
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
