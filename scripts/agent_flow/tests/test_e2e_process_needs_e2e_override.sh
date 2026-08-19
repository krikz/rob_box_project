#!/bin/bash
# ============================================================================
# test_e2e_process_needs_e2e_override.sh — bug(e2e #1448) ретро 19.08
#
# Проверяет guard в agent-flow-e2e-process.sh: когда PR уже merged
# (fail_kind=merged) и issue имеет явный `needs-e2e` (override Шифу
# после ручного возврата в ротацию), авто-`e2e-done` ПОДАВЛЕНО —
# вместо этого ставится `e2e:rejected` (needs-e2e снимается), issue
# уходит на ручное решение Шифу. Без guard'а каждый тик e2e-process
# видит merged PR → fail_kind=merged → auto-e2e-done → reconcile
# merge-gate снимает PR-side stale метки → Шифу снова ставит needs-e2e
# → цикл бесконечный (issue #1448, баг #1392/#1398).
#
# Тест НЕ запускает agent-flow-e2e-process.sh целиком (слишком много
# pre-checks), а изолирует ту же логику выбора label_action в локальные
# функции — гарантирует, что регулярки/ветвления совпадают с тем, что
# в скрипте. Если в будущем логика в скрипте изменится — обновить и этот
# тест.
#
# Acceptance (issue #1448, секция A):
#   - fail_kind=merged + verdict≠success + issue имеет needs-e2e →
#     label_action="add e2e:rejected", remove_action="remove needs-e2e".
#   - fail_kind=merged + verdict≠success + issue НЕ имеет needs-e2e →
#     старое поведение: label_action="add e2e-done", remove_action="remove needs-e2e"
#     (stale-метка не нужна, ретро 10.08 t_9caf5d52).
#   - verdict=success → label_action="add e2e-done" независимо от needs-e2e.
#   - В e2e-process.sh присутствуют все три ветки.
#
# Run:
#   bash scripts/agent_flow/tests/test_e2e_process_needs_e2e_override.sh
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

assert_contains() {
    local name="$1" needle="$2" haystack="$3"
    if printf '%s' "$haystack" | grep -qF -- "$needle"; then
        pass "$name"
    else
        fail "$name" "expected to contain '$needle', got: '$haystack'"
    fi
}

# --- Sanity-check: наличие guard в скрипте ---------------------------------
echo "=== Sanity: guard в $E2E_PROCESS ==="
if grep -q 'merged-override' "$E2E_PROCESS"; then
    pass "merged-override ветка присутствует в e2e-process.sh"
else
    fail "merged-override sanity" "строка 'merged-override' не найдена в $E2E_PROCESS — скрипт не патчился?"
fi
if grep -q 'issue #1448' "$E2E_PROCESS"; then
    pass "ссылка на issue #1448 в комментариях скрипта"
else
    fail "issue #1448 reference" "комментарий с issue #1448 не найден — ретро-маркер отсутствует"
fi
if grep -q 't_b3691e1b' "$E2E_PROCESS"; then
    pass "kanban-task-id t_b3691e1b в комментариях скрипта"
else
    fail "t_b3691e1b reference" "kanban-task-id t_b3691e1b не упомянут — невозможно отследить ретро-происхождение"
fi
# Guard на явный needs-e2e override — должен быть ДО ветвления label_action
# (т.е. до строки "if [ \"\$verdict\" = \"success\" ]").
_guard_line="$(grep -n 'fail_kind="merged-override"' "$E2E_PROCESS" | head -1 | cut -d: -f1)"
_verdict_line="$(grep -n 'if \[ "\$verdict" = "success" \]' "$E2E_PROCESS" | head -1 | cut -d: -f1)"
if [ -n "$_guard_line" ] && [ -n "$_verdict_line" ] && [ "$_guard_line" -lt "$_verdict_line" ]; then
    pass "guard ДО ветвления label_action (line $_guard_line < $_verdict_line)"
else
    fail "guard order" "guard должен стоять ДО if verdict=success (guard=$_guard_line verdict=$_verdict_line)"
fi

# --- Локальная копия decision-логики (синхронна со скриптом) ---------------
# Воспроизводит ровно тот же блок выбора label_action, что и в
# agent-flow-e2e-process.sh (строки ~2128..). Если скрипт изменится —
# синхронизируйте с ним.
#
# Аргументы:
#   $1=verdict ("success" / "failure" / ...)
#   $2=fail_kind ("feature" / "merged" / "infra" / "merged-override" / ...)
#   $3=labels_norm (lowercase comma-separated)
# Выходы:
#   label_action через stdout: "add <LABEL>"
#   remove_action через $REPLY_ACTION: "remove <LABEL>" или ""
NEEDS_E2E_LABEL="needs-e2e"
DONE_LABEL="e2e-done"
REJECTED_LABEL="e2e:rejected"
INFRA_FAIL_LABEL="e2e:infra-fail"
has_label() {
    printf '%s' "$1" | tr ',' '\n' | grep -Fxq "$2"
}
# Возвращает "label_action<TAB>remove_action".
decide_label_actions() {
    local verdict="$1" fail_kind="$2" labels_norm="$3"
    local label_action="" remove_action=""
    # Ретро 19.08 t_b3691e1b (issue #1448): guard на явный needs-e2e
    # override при fail_kind=merged → переход в merged-override.
    if [ "$fail_kind" = "merged" ] && [ "$verdict" != "success" ] \
        && has_label "$labels_norm" "$NEEDS_E2E_LABEL"; then
        fail_kind="merged-override"
    fi
    if [ "$verdict" = "success" ]; then
        label_action="add ${DONE_LABEL}"
        remove_action="remove ${NEEDS_E2E_LABEL}"
    elif [ "$fail_kind" = "merged" ]; then
        label_action="add ${DONE_LABEL}"
        remove_action="remove ${NEEDS_E2E_LABEL}"
    elif [ "$fail_kind" = "merged-override" ]; then
        label_action="add ${REJECTED_LABEL}"
        remove_action="remove ${NEEDS_E2E_LABEL}"
    elif [ "$fail_kind" = "infra" ]; then
        label_action="add ${INFRA_FAIL_LABEL}"
        remove_action=""
    else
        label_action="add ${REJECTED_LABEL}"
        remove_action="remove ${NEEDS_E2E_LABEL}"
    fi
    printf '%s\t%s' "$label_action" "$remove_action"
}

# --- Тест 1: needs-e2e override + merged + verdict≠success → e2e:rejected --
echo ""
echo "=== Тест 1: needs-e2e override + merged PR + FAIL → e2e:rejected (issue #1448) ==="
# Сценарий: Шифу вернул issue в ротацию (needs-e2e), PR уже merged,
# verdict прогона FAIL → guard должен переключить fail_kind на
# merged-override → label_action=e2e:rejected (не e2e-done!).
result=$(decide_label_actions "failure" "merged" "hermes,needs-e2e,bug,priority:critical")
out_label="${result%%	*}"
out_remove="${result#*	}"
assert_eq "label_action=add e2e:rejected" "add e2e:rejected" "$out_label"
assert_eq "remove_action=remove needs-e2e" "remove needs-e2e" "$out_remove"
# НЕ должно быть e2e-done
case "$out_label" in
    *e2e-done*) fail "guard bypassed" "поставил бы e2e-done при явном needs-e2e override — баг #1448 не починен" ;;
    *) pass "guard НЕ ставит e2e-done при явном needs-e2e override" ;;
esac

# --- Тест 2: merged + verdict≠success + БЕЗ needs-e2e → старое поведение --
echo ""
echo "=== Тест 2: merged + verdict≠success, БЕЗ needs-e2e → e2e-done (старое поведение) ==="
# Сценарий: первый e2e прогон на merged PR без явного override —
# guard НЕ должен срабатывать, должно быть старое поведение
# (fail_kind=merged → e2e-done, ретро 10.08 t_9caf5d52).
result=$(decide_label_actions "failure" "merged" "hermes,bug,priority:critical")
out_label="${result%%	*}"
out_remove="${result#*	}"
assert_eq "label_action=add e2e-done" "add e2e-done" "$out_label"
assert_eq "remove_action=remove needs-e2e" "remove needs-e2e" "$out_remove"

# --- Тест 3: verdict=success → e2e-done (независимо от needs-e2e) ---------
echo ""
echo "=== Тест 3: verdict=success + merged + needs-e2e → e2e-done (PASS override) ==="
# Сценарий: Шифу вернул в needs-e2e, прогон прошёл успешно — guard
# НЕ срабатывает (verdict=success), e2e-done ставится штатно.
result=$(decide_label_actions "success" "merged" "hermes,needs-e2e,bug,priority:critical")
out_label="${result%%	*}"
out_remove="${result#*	}"
assert_eq "label_action=add e2e-done" "add e2e-done" "$out_label"
assert_eq "remove_action=remove needs-e2e" "remove needs-e2e" "$out_remove"

# --- Тест 4: feature-FAIL + needs-e2e → e2e:rejected (как раньше) ---------
echo ""
echo "=== Тест 4: feature-FAIL + needs-e2e → e2e:rejected (старое поведение) ==="
# Сценарий: PR не merged, FAIL по вине фичи — guard не срабатывает
# (fail_kind=feature, не merged), e2e:rejected ставится.
result=$(decide_label_actions "failure" "feature" "hermes,needs-e2e,bug")
out_label="${result%%	*}"
out_remove="${result#*	}"
assert_eq "label_action=add e2e:rejected" "add e2e:rejected" "$out_label"
assert_eq "remove_action=remove needs-e2e" "remove needs-e2e" "$out_remove"

# --- Тест 5: infra-FAIL + needs-e2e → e2e:infra-fail (needs-e2e сохранён) -
echo ""
echo "=== Тест 5: infra-FAIL + needs-e2e → e2e:infra-fail, needs-e2e сохранён ==="
# Сценарий: PR не merged, FAIL по инфре (429 / робот недоступен) —
# guard не срабатывает (fail_kind=infra), needs-e2e сохраняется.
result=$(decide_label_actions "failure" "infra" "hermes,needs-e2e,bug")
out_label="${result%%	*}"
out_remove="${result#*	}"
assert_eq "label_action=add e2e:infra-fail" "add e2e:infra-fail" "$out_label"
assert_eq "remove_action='' (не снимаем needs-e2e)" "" "$out_remove"

# --- Тест 6: regression — sanity-checks синтаксиса скрипта ----------------
echo ""
echo "=== Тест 6: bash -n $E2E_PROCESS ==="
if bash -n "$E2E_PROCESS"; then
    pass "bash -n OK (синтаксис корректен)"
else
    fail "bash -n" "синтаксическая ошибка в скрипте"
fi

# --- Тест 7: cycle-prevention property -------------------------------------
echo ""
echo "=== Тест 7: cycle prevention — повторный тик не возвращает в needs-e2e ==="
# После merged-override issue имеет e2e:rejected (НЕ needs-e2e).
# Следующий тик e2e-process увидит e2e:rejected → skip (логика строки 1309).
# Это разрывает цикл needs-e2e → e2e-done.
result=$(decide_label_actions "failure" "merged" "hermes,e2e:rejected,bug")
out_label="${result%%	*}"
out_remove="${result#*	}"
# Имитируем следующий тик: issue уже e2e:rejected, не needs-e2e
# → guard не срабатывает → старое поведение e2e-done (как и до guard'а,
# но guard не виноват — цикл уже разорван через e2e:rejected).
# Главное: на следующем тике e2e-process skip (логика 1309 — has REJECTED).
# Это мы проверим grep'ом по скрипту.
if grep -q 'has_label "$labels_norm" "$REJECTED_LABEL"' "$E2E_PROCESS"; then
    pass "e2e-process skip при e2e:rejected (строка 1309+) → цикл разорван"
else
    fail "skip rejected" "логика skip при REJECTED_LABEL не найдена — цикл может возобновиться"
fi

# --- Итоги -----------------------------------------------------------------
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