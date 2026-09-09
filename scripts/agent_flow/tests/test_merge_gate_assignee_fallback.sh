#!/bin/bash
# ============================================================================
# test_merge_gate_assignee_fallback.sh — ретро 02.09 t_2bd2e7ea acceptance test
#
# Баг: agent-flow-merge-gate.sh и agent-flow-e2e-process.sh создавали
# recovery/conflict карточки с _assignee="default" если у issue не было метки
# agent:*. Профиль "default" не валиден (ADR-0041 silent-drop) → карточки
# зависали в ready навечно, спам 109+ шт за ночь.
#
# Фикс (02.09): default → devops fallback. Если метка agent:* найдена —
# используется она, иначе — devops (он же воркер, который умеет
# force-with-lease push).
#
# Рефактор (09.09, issue #2292): вся логика вынесена в
# af_role_for(labels) → profile в lib_agent_flow_common.sh. Здесь тесты
# идут НАПРЯМУЮ через общий модуль — тест-реплика `extract_assignee_for_labels`
# удалена. Дополнительно — статические проверки в merge-gate / e2e-process,
# что они вызывают af_role_for (а не свою копию case-таблицы).
#
# Сценарии:
#   F1. af_role_for с пустым labels_csv → fallback=devops
#   F2. af_role_for с меткой agent:backend → backend
#   F3. af_role_for с двумя метками (agent:architect первая) → architect
#   F4. af_role_for с метками без agent:* (priority:high, bug) → fallback=devops
#   F5. af_role_for с явной меткой agent:devops → devops
#   F6. af_role_for с agent:tester → tester (восстановление, было сломано
#       в e2e-process до #2292)
#   F7. af_role_for с неизвестной меткой agent:triager → fallback=devops +
#       warn (caller сам делает hard-gate)
#   F8. agent-flow-merge-gate.sh: 0 вхождений локальной case-таблицы
#       (должно быть af_role_for)
#   F9. agent-flow-e2e-process.sh: 0 вхождений локальной case-таблицы
#   F10. bash -n на lib_agent_flow_common.sh + оба скрипта
#
# Run:
#   bash scripts/agent_flow/tests/test_merge_gate_assignee_fallback.sh
# ============================================================================
set -uo pipefail

TEST_LIB_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
_REAL_TESTS_DIR="$TEST_LIB_DIR"  # Сохраняем ДО source — mock_env перезапишет TEST_LIB_DIR.
# shellcheck source=lib/mock_env.sh
. "$TEST_LIB_DIR/lib/mock_env.sh"

# Реальные пути к скриптам.
_REAL_REPO_ROOT="$(cd "$_REAL_TESTS_DIR/../../.." && pwd)"
MERGE_GATE="$_REAL_REPO_ROOT/scripts/agent_flow/agent-flow-merge-gate.sh"
E2E_PROCESS="$_REAL_REPO_ROOT/scripts/agent_flow/agent-flow-e2e-process.sh"
LIB_COMMON="$_REAL_REPO_ROOT/scripts/agent_flow/lib_agent_flow_common.sh"

PASS=0
FAIL=0
FAILED_CASES=()
pass() { PASS=$((PASS+1)); printf '  \033[32m✓\033[0m %s\n' "$1"; }
fail() {
    FAIL=$((FAIL+1)); FAILED_CASES+=("$1")
    printf '  \033[31m✗\033[0m %s\n' "$1"
}
log() { if [ "${VERBOSE:-0}" = "1" ]; then printf '  %s\n' "$*"; fi; }

# ============================================================================
# F1-F7: общий модуль af_role_for (lib_agent_flow_common.sh)
# ============================================================================
# Идея (issue #2292): тест идёт через ОБЩИЙ модуль, а не через реплику case-
# таблицы. Раньше `extract_assignee_for_labels` копировал case-логику — если
# бы она расходилась с реальной таблицей, тест прошёл бы зря. Теперь источник
# истины один: lib_agent_flow_common.sh::af_role_for.
#
# HERMES_BIN не задан → warn-блок af_role_for пропускается, но case-таблица
# всё равно отрабатывает (warn+fail-open на отсутствующий профиль не меняет
# возвращаемое значение).

# Загружаем только af_role_for + зависимости (_af_log). Делаем это через
# sub-shell source, чтобы не загрязнять глобальный scope тестов.
#
# ВАЖНО: НЕ глушим stderr при source — внешний вызов перенаправляет stderr
# в файл и потом грепает warn. Если бы мы поставили 2>&1 здесь, warn от
# _af_log в lib_agent_flow_common.sh ушёл бы в dev/null внутри sub-shell.
_af_role_for_wrapper() {
    # shellcheck disable=SC1090
    # unset -f log: иначе _af_log (внутри af_role_for) делегирует в `log` родителя,
    # а `log` теста пишет в stdout (строка 59) → command substitution $()
    # поглощает warn вместе со stdout, и grep по warn-файлу ничего не найдёт.
    ( unset -f log; source "$LIB_COMMON"; af_role_for "$@" )
}

# Wrapper для af_role_found_for — exit-code-only проверка.
_af_role_found_for_wrapper() {
    # shellcheck disable=SC1090
    ( source "$LIB_COMMON" >/dev/null 2>&1; af_role_found_for "$@" )
}

# F1: пустые метки → fallback
test_F1_no_label_devops_fallback() {
    new_test
    local got
    got="$(_af_role_for_wrapper "" devops)"
    if [ "$got" = "devops" ]; then
        pass "F1: пустые метки + fallback=devops → devops"
    else
        fail "F1: пустые метки → assignee='$got' (expected devops)"
    fi
}

# F2: agent:backend → backend
test_F2_backend_label() {
    new_test
    local got
    got="$(_af_role_for_wrapper "agent:backend" devops)"
    if [ "$got" = "backend" ]; then
        pass "F2: agent:backend → backend"
    else
        fail "F2: agent:backend → assignee='$got' (expected backend)"
    fi
}

# F3: первый agent:* в списке выигрывает (architect перед devops)
test_F3_first_agent_label_wins() {
    new_test
    local got
    got="$(_af_role_for_wrapper "agent:architect,agent:devops" devops)"
    if [ "$got" = "architect" ]; then
        pass "F3: agent:architect (первая) → architect"
    else
        fail "F3: agent:architect первая → assignee='$got' (expected architect)"
    fi
}

# F4: метки без agent:* → fallback=devops
test_F4_unrelated_label_devops_fallback() {
    new_test
    local got
    got="$(_af_role_for_wrapper "priority:high,bug" devops)"
    if [ "$got" = "devops" ]; then
        pass "F4: priority:high,bug (нет agent:*) → fallback=devops"
    else
        fail "F4: unrelated labels → assignee='$got' (expected devops)"
    fi
}

# F5: явная agent:devops → devops
test_F5_explicit_devops_label() {
    new_test
    local got
    got="$(_af_role_for_wrapper "agent:devops" devops)"
    if [ "$got" = "devops" ]; then
        pass "F5: agent:devops → devops (явная)"
    else
        fail "F5: agent:devops → assignee='$got' (expected devops)"
    fi
}

# F6: agent:tester → tester (регрессия — раньше e2e-process терял эту ветку)
test_F6_tester_label_restored() {
    new_test
    local got
    got="$(_af_role_for_wrapper "agent:tester,bug" devops)"
    if [ "$got" = "tester" ]; then
        pass "F6: agent:tester → tester (восстановлено, регрессия e2e-process)"
    else
        fail "F6: agent:tester → assignee='$got' (expected tester)"
    fi
}

# F7: неизвестная agent:triager → fallback=devops + warn
test_F7_unknown_agent_label_fallback() {
    new_test
    local got warn_seen
    # Проверяем stdout через wrapper; warn уходит в stderr через _af_log
    # (после `unset -f log` в wrapper'е — без делегирования).
    got="$(_af_role_for_wrapper "agent:triager,bug" devops 2>/tmp/.af_role_warn.$$)"
    warn_seen="$(grep -c 'unknown agent:label' /tmp/.af_role_warn.$$ 2>/dev/null)"
    warn_seen="${warn_seen:-0}"
    rm -f /tmp/.af_role_warn.$$
    if [ "$got" = "devops" ] && [ "$warn_seen" -ge 1 ]; then
        pass "F7: agent:triager → fallback=devops + warn ('unknown agent:label')"
    else
        fail "F7: agent:triager → got='$got' warn_count='$warn_seen' (expected devops + warn)"
    fi
}

# ============================================================================
# F8-F9: статическая проверка — никакой локальной case-таблицы не осталось
# ============================================================================

# F8: merge-gate больше не содержит локальную case-таблицу с agent:*
#     (заменена на af_role_for).
test_F8_merge_gate_no_local_case_table() {
    new_test
    local hits
    # Старый паттерн: цикл for + case "agent:backend|developer|tester|devops|architect".
    # Новый: af_role_for. Считаем старый паттерн.
    hits="$(grep -cE 'case "\$lbl" in[[:space:]]*$' "$MERGE_GATE" || true)"
    if [ "$hits" = "0" ]; then
        pass "F8: agent-flow-merge-gate.sh: 0 локальных case \$lbl (всё через af_role_for)"
    else
        fail "F8: agent-flow-merge-gate.sh: найдено $hits локальных case-таблиц (ожидалось 0)"
    fi
    # Дополнительно: af_role_for должен быть вызван хотя бы 1 раз.
    local calls
    calls="$(grep -c 'af_role_for' "$MERGE_GATE" || true)"
    if [ "$calls" -ge 1 ]; then
        pass "F8b: agent-flow-merge-gate.sh: $calls вызовов af_role_for (≥1 OK)"
    else
        fail "F8b: agent-flow-merge-gate.sh: $calls вызовов af_role_for (ожидалось ≥1)"
    fi
}

# F9: e2e-process больше не содержит локальную case-таблицу
test_F9_e2e_process_no_local_case_table() {
    new_test
    local hits calls
    hits="$(grep -cE 'case "\$lbl" in[[:space:]]*$' "$E2E_PROCESS" || true)"
    if [ "$hits" = "0" ]; then
        pass "F9: agent-flow-e2e-process.sh: 0 локальных case \$lbl (всё через af_role_for)"
    else
        fail "F9: agent-flow-e2e-process.sh: найдено $hits локальных case-таблиц (ожидалось 0)"
    fi
    calls="$(grep -c 'af_role_for' "$E2E_PROCESS" || true)"
    if [ "$calls" -ge 2 ]; then
        pass "F9b: agent-flow-e2e-process.sh: $calls вызовов af_role_for (≥2 — conflict+worker)"
    else
        fail "F9b: agent-flow-e2e-process.sh: $calls вызовов af_role_for (ожидалось ≥2)"
    fi
}

# F10: bash syntax check на lib_common + оба скрипта
test_F10_bash_syntax() {
    new_test
    local lib_ok mg_ok ep_ok
    if bash -n "$LIB_COMMON" 2>/dev/null; then lib_ok=1; else lib_ok=0; fi
    if bash -n "$MERGE_GATE" 2>/dev/null; then mg_ok=1; else mg_ok=0; fi
    if bash -n "$E2E_PROCESS" 2>/dev/null; then ep_ok=1; else ep_ok=0; fi
    if [ "$lib_ok" = "1" ] && [ "$mg_ok" = "1" ] && [ "$ep_ok" = "1" ]; then
        pass "F10: bash -n lib_common + merge-gate + e2e-process OK"
    else
        fail "F10: bash -n lib=$lib_ok mg=$mg_ok ep=$ep_ok"
    fi
}

# ============================================================================
# F11-F14: af_role_found_for (companion) — scan-all-prs contract_drift.
# ============================================================================
# Зачем: scan-all-prs в merge-gate использует _assignee_explicit=1 только
# когда метка agent:* НАЙДЕНА (явная). Иначе contract_drift перезаписывает
# assignee на backend. Companion функция `af_role_found_for` экспортирует
# этот предикат из общего модуля, чтобы не дублировать парсинг.

# F11: пустые метки → нет явной
test_F11_found_for_empty() {
    new_test
    if _af_role_found_for_wrapper ""; then
        fail "F11: пустые метки → af_role_found_for вернул 0 (ожидался 1)"
    else
        pass "F11: пустые метки → af_role_found_for=1 (нет явной)"
    fi
}

# F12: валидная agent:devops → найдена
test_F12_found_for_valid_devops() {
    new_test
    if _af_role_found_for_wrapper "agent:devops,bug"; then
        pass "F12: agent:devops → af_role_found_for=0 (явная)"
    else
        fail "F12: agent:devops → af_role_found_for=1 (ожидался 0)"
    fi
}

# F13: неизвестная agent:triager → НЕ считаем явной (caller уйдёт в fallback)
test_F13_found_for_unknown_token() {
    new_test
    if _af_role_found_for_wrapper "agent:triager,bug"; then
        fail "F13: agent:triager → af_role_found_for=0 (НЕ должен считать явной)"
    else
        pass "F13: agent:triager → af_role_found_for=1 (невалидный → fallback)"
    fi
}

# F14: метки без agent:* → нет явной
test_F14_found_for_unrelated() {
    new_test
    if _af_role_found_for_wrapper "priority:high,bug"; then
        fail "F14: priority:high,bug → af_role_found_for=0 (ожидался 1)"
    else
        pass "F14: priority:high,bug → af_role_found_for=1 (нет agent:*)"
    fi
}

# ============================================================================
# Run
# ============================================================================
run_test() {
    printf '\n[ RUN     ] %s\n' "$1"
    "$1" || true
}

run_test test_F1_no_label_devops_fallback
run_test test_F2_backend_label
run_test test_F3_first_agent_label_wins
run_test test_F4_unrelated_label_devops_fallback
run_test test_F5_explicit_devops_label
run_test test_F6_tester_label_restored
run_test test_F7_unknown_agent_label_fallback
run_test test_F8_merge_gate_no_local_case_table
run_test test_F9_e2e_process_no_local_case_table
run_test test_F10_bash_syntax
run_test test_F11_found_for_empty
run_test test_F12_found_for_valid_devops
run_test test_F13_found_for_unknown_token
run_test test_F14_found_for_unrelated

printf '\n==== Summary ====\n'
printf 'total:  %d\npassed: %d\nfailed: %d\n' "$((PASS+FAIL))" "$PASS" "$FAIL"
if [ "$FAIL" -gt 0 ]; then
    printf '\nFailures:\n'
    for c in "${FAILED_CASES[@]}"; do printf '  - %s\n' "$c"; done
    exit 1
fi
exit 0
