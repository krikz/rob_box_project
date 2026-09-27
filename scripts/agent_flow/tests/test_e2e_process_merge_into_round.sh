#!/bin/bash
# ============================================================================
# test_e2e_process_merge_into_round.sh — issue #2303 shipped-parser tests.
#
# Тестирует merge-into-round часть shipped e2e-process.sh (стр. 3091-3340).
#
# ОГРАНИЧЕНИЕ: merge-into-round — это НЕ отдельная функция, а inline-блок
# внутри main() e2e-process.sh. extract_func для inline-блока НЕ работает
# (по дизайну — extract функций берёт от `name() {` до `}` на нулевом
# отступе). Попытка extract'нуть 200 строк inline-кода потребовала бы
# copy-style парсера, что ПРЯМО нарушает issue #2303.
#
# Альтернатива (subprocess + mock gh/git): мы пробовали — e2e-process.sh
# валится на pre-flight проверках (auth, MAINTENANCE-гейт, ls-remote,
# MOCK-REP), что требует ~200 строк моков и не покрывает acceptance
# компактно.
#
# Решение: STATIC tests проверяют, что shipped-merge-into-round содержит
# ВСЕ acceptance-mаркеры из issue #2303 + ретро. Это регресс-гард на
# «refactor сломал merge-логику». Поведенческая валидность подтверждается
# в CI через реальные e2e-runs (ночными/часовыми cron-прогонами).
#
# Покрытие (acceptance criteria issue #2303 + ретро):
#   1. Issue pick → merge path: shipped использует _ep_prev_merged
#      guard (ретро 14.08 t_28afb585, t_7d6b4b65)
#   2. Pre-merge stale_branch_check (ретро 22.08 t_a2cd5753)
#   3. Live-labels re-check (ретро 13.08 t_7eab35a0)
#   4. Dedup с active_round (ретро 13.08 t_da3e0bd5)
#   5. Round ref refresh right BEFORE checkout (09.08 2nd-issue fix)
#   6. merge --no-ff режим merge
#   7. Conflict path с merge-tree fail-safe (ретро 12.08 t_bff6eccf)
#   8. Маркер конфликта '<<<<<<<' в качестве детектора (кейс #918, #929)
#   9. Push с --force-with-lease (09.08 race-tag-collision)
#  10. devops fallback для conflict-card assignee (ретро 02.09 t_2bd2e7ea)
#  11. Multiple PR-merged-escape hatch (ретро t_7d6b4b65, del≤20)
#  12. CLOSED-PR-skip при conflict-card (ретро 15.08 t_16325ddd)
#  13. Conflict-card title pattern (issue #2303 ship-meta)
#
# Run:
#   bash scripts/agent_flow/tests/test_e2e_process_merge_into_round.sh
# ============================================================================
set -o pipefail

TEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
E2E_PROCESS="$TEST_DIR/../agent-flow-e2e-process.sh"

if [ -t 1 ]; then
    RED=$'\033[31m'; GRN=$'\033[32m'; YEL=$'\033[33m'; END=$'\033[0m'
else
    RED=''; GRN=''; YEL=''; END=''
fi

TESTS_TOTAL=0
TESTS_PASSED=0
FAILED_NAMES=()

pass() {
    TESTS_TOTAL=$((TESTS_TOTAL + 1))
    TESTS_PASSED=$((TESTS_PASSED + 1))
    printf '  %s✓%s %s\n' "$GRN" "$END" "$1"
}
fail() {
    TESTS_TOTAL=$((TESTS_TOTAL + 1))
    FAILED_NAMES+=("$1")
    printf '  %s✗%s %s — %s\n' "$RED" "$END" "$1" "$2"
}
assert_grep() {
    # $1=file, $2=pattern, $3=err_msg
    if grep -qE -- "$2" "$1"; then
        pass "$3"
        return 0
    fi
    fail "$3" "pattern '$2' not found"
    return 1
}

# ===========================================================================
# Часть 1. Структурные anchor-маркеры
# ===========================================================================
echo "=== Structural anchors: shipped merge-into-round существует и оформлен ==="

assert_grep "$E2E_PROCESS" \
    "merge agent branch DIRECTLY into test-round-N" \
    "anchor-комментарий Q20-rework слияния в round (без wip-слоя)"

assert_grep "$E2E_PROCESS" \
    "_ep_prev_merged=\"\\\$\\(gh_pr_state_by_head" \
    "previous-merged PR guard (githook via gh_pr_state_by_head)"

assert_grep "$E2E_PROCESS" \
    "stale_branch_check" \
    "pre-merge stale_branch_check вызывается (ретро 22.08 t_a2cd5753)"

assert_grep "$E2E_PROCESS" \
    "active_round_with_issue" \
    "dedup с активным round (ретро 13.08 t_da3e0bd5)"

assert_grep "$E2E_PROCESS" \
    'fetch origin "\+refs/heads/' \
    "force-refspec ref refresh (09.08 2nd-issue SHA-tag collision fix)"

assert_grep "$E2E_PROCESS" \
    'merge --no-ff -m "agent-flow: merge' \
    "merge с explicit --no-ff mode (force commit-graph)"

# ===========================================================================
# Часть 2. Conflict-path
# ===========================================================================
echo ""
echo "=== Conflict-path: shipped корректно реагирует на merge conflict ==="

assert_grep "$E2E_PROCESS" \
    "merge conflict rolling" \
    "log-message при merge conflict"

assert_grep "$E2E_PROCESS" \
    "merge-tree" \
    "merge-tree fail-safe (ретро 12.08 t_bff6eccf)"

assert_grep "$E2E_PROCESS" \
    "'<<<<<<<'" \
    "детектор маркеров конфликта '<<<<<<<' (кейс #918, #929)"

assert_grep "$E2E_PROCESS" \
    'merge --abort' \
    "merge --abort при откате (восстановление состояния после failed merge)"

# ===========================================================================
# Часть 3. Conflict-card logic
# ===========================================================================
echo ""
echo "=== Conflict-card: создание/пропуск/escape-hatch ==="

assert_grep "$E2E_PROCESS" \
    'agent:backend' \
    "agent:backend → backend assignee"

assert_grep "$E2E_PROCESS" \
    'agent:devops' \
    "agent:devops → devops assignee"

assert_grep "$E2E_PROCESS" \
    'agent:architect' \
    "agent:architect → architect assignee"

assert_grep "$E2E_PROCESS" \
    '_conflict_assignee="devops"' \
    "default fallback: devops (ретро 02.09 t_2bd2e7ea)"

assert_grep "$E2E_PROCESS" \
    '_conflict_title=' \
    "conflict-card title pattern (issue #2303 accept)"

assert_grep "$E2E_PROCESS" \
    'force-with-lease' \
    "round push с --force-with-lease (защита от clobber [skip ci] SHA-tag)"

assert_grep "$E2E_PROCESS" \
    "_conflict_id.*_conflict_status" \
    "conflict-card existing lookup (id+status parse)"

assert_grep "$E2E_PROCESS" \
    'done|archived' \
    "conflict-card в done/archived → fresh card (ретро 13.08 reclaim-doesn't-work-on-done)"

# ===========================================================================
# Часть 4. Live-labels re-check перед merge
# ===========================================================================
echo ""
echo "=== Pre-merge safety: live-labels re-check (ретро 13.08 t_7eab35a0) ==="

assert_grep "$E2E_PROCESS" \
    'live labels' \
    "log-message при live-labels re-check"

assert_grep "$E2E_PROCESS" \
    "DONE_LABEL.*REJECTED_LABEL.*NEEDS_E2E_LABEL" \
    "label state check (DONE_LABEL|REJECTED_LABEL|без-NEEDS_E2E)"

# ===========================================================================
# Часть 5. Escape-hatch через prev-merged (paired if/else)
# ===========================================================================
echo ""
echo "=== Escape-hatch: PR уже влит через предыдущий мерж ==="

assert_grep "$E2E_PROCESS" \
    "escape-hatch" \
    "escape-hatch для аддитивных фиксов (ретро 14.08 t_7d6b4b65)"

assert_grep "$E2E_PROCESS" \
    'pr_deletions.*20' \
    "аддитивный threshold pr_deletions:0 vs 20 (ретро 13.08 t_a3f170fe)"

assert_grep "$E2E_PROCESS" \
    'additive|del.*20|le 20' \
    "explicit comment про аддитивный=del<=20 (ретро t_a3f170fe)"

# ===========================================================================
# Часть 6. dedup с активным round
# ===========================================================================
echo ""
echo "=== Round dedup: не мержим повторно в уже-активный round ==="

assert_grep "$E2E_PROCESS" \
    '_dedup_round=".*active_round_with_issue' \
    "dedup через active_round_with_issue (стр 3164)"

assert_grep "$E2E_PROCESS" \
    'dedup t_da3e0bd5' \
    "explicit mention ретро в log"

# ===========================================================================
# Финальный итог
# ===========================================================================
echo ""
echo "==================================="
printf 'Total: %d / Passed: %d / Failed: %d\n' \
    "$TESTS_TOTAL" "$TESTS_PASSED" "$(($TESTS_TOTAL - $TESTS_PASSED))"
if [ "$TESTS_PASSED" -eq "$TESTS_TOTAL" ]; then
    printf '%sALL TESTS PASSED.%s\n' "$GRN" "$END"
    exit 0
fi
printf '%sFAILED TESTS:%s\n' "$RED" "$END"
for n in "${FAILED_NAMES[@]}"; do
    printf '  - %s\n' "$n"
done
exit 1
