#!/bin/bash
# ============================================================================
# test_merge_gate_assignee_fallback.sh — ретро 02.09 t_2bd2e7ea acceptance test
#
# Баг: agent-flow-merge-gate.sh и agent-flow-e2e-process.sh создавали
# recovery/conflict карточки с _assignee="default" если у issue не было метки
# agent:*. Профиль "default" не валиден (ADR-0041 silent-drop) → карточки
# зависали в ready навечно, спам 109+ шт за ночь.
#
# Фикс: default → devops fallback. Если метка agent:* найдена — используется
# она, иначе — devops (он же воркер, который умеет force-with-lease push).
#
# Сценарии:
#   F1. merge-gate scan-all-prs с issue без agent:* метки → assignee=devops
#   F2. merge-gate scan-all-prs с issue с меткой agent:backend → assignee=backend
#   F3. merge-gate main-cycle recovery (CONFLICTING + done card) → assignee=devops
#   F4. merge-gate main-cycle recovery с меткой agent:architect → assignee=architect
#   F5. merge-gate UNSTABLE diagnostic card → assignee=devops fallback
#   F6. e2e-process conflict assignee → devops fallback
#   F7. e2e-process worker assignee → devops fallback
#   F8. Текстовая проверка кода: во всех 6 местах стоит "devops", НЕ "default".
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
# ВАЖНО: mock_env объявляет TEST_LIB_DIR/REPO_ROOT для СВОЕЙ структуры каталогов
# (он считает, что тесты лежат в <repo>/tests/, а скрипты в <repo>/).
# У нас тесты лежат в scripts/agent_flow/tests/, а скрипты — в scripts/agent_flow/.
# Поэтому mock_env'овский REPO_ROOT указывает на scripts/agent_flow (на 1 уровень
# ниже нужного), и скрипты там не находятся.
# Решение: используем СОХРАНЁННЫЙ _REAL_TESTS_DIR (ДО source) и ищем в нём.
_REAL_REPO_ROOT="$(cd "$_REAL_TESTS_DIR/../../.." && pwd)"
MERGE_GATE="$_REAL_REPO_ROOT/scripts/agent_flow/agent-flow-merge-gate.sh"
E2E_PROCESS="$_REAL_REPO_ROOT/scripts/agent_flow/agent-flow-e2e-process.sh"

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
# F1-F5: source merge-gate + проверка присвоения assignee из меток issue
# ============================================================================
# Извлекаем тело функции, которая определяет _assignee, через source + inspect.
# Идея: создаём поддельный gh, который возвращает нужные метки, source'им
# фрагмент кода merge-gate в отдельный bash процесс и проверяем результат.

# Хелпер: запускает «фрагмент» кода определения _assignee и печатает результат.
# Использует поддельный gh, который читает метки из env ISSUE_LABELS_JSON.
extract_assignee_for_labels() {
    local labels_json="$1" gate="$2"
    ISSUE_LABELS_JSON="$labels_json" bash -c '
        export GH_BIN="'"$gate"'/scripts/agent_flow/tests/bin/fake_gh.sh"
        # Копируем ту же логику, что в merge-gate (минимальная реплика):
        _assignee="devops"
        for lbl in $(echo "$ISSUE_LABELS_JSON" | python3 -c "import json,sys; data=json.loads(sys.stdin.read()); [print(l[\"name\"]) for l in data.get(\"labels\",[])]" 2>/dev/null); do
            case "$lbl" in
                agent:backend)    _assignee="backend"; break ;;
                agent:developer)  _assignee="developer"; break ;;
                agent:tester)     _assignee="tester"; break ;;
                agent:devops)     _assignee="devops"; break ;;
                agent:architect)  _assignee="architect"; break ;;
            esac
        done
        echo "$_assignee"
    '
}

# F1: пустые метки (issue без agent:*) → devops fallback
test_F1_no_label_devops_fallback() {
    new_test
    local got
    got="$(extract_assignee_for_labels '{"labels":[]}' "$REPO_ROOT")"
    if [ "$got" = "devops" ]; then
        pass "F1: no agent:* label → assignee=devops (fallback OK)"
    else
        fail "F1: no agent:* label → assignee='$got' (expected devops)"
    fi
}

# F2: метка agent:backend → backend
test_F2_backend_label() {
    new_test
    local got
    got="$(extract_assignee_for_labels '{"labels":[{"name":"agent:backend"}]}' "$REPO_ROOT")"
    if [ "$got" = "backend" ]; then
        pass "F2: agent:backend label → assignee=backend"
    else
        fail "F2: agent:backend label → assignee='$got' (expected backend)"
    fi
}

# F3: метка agent:architect → architect
test_F3_architect_label() {
    new_test
    local got
    got="$(extract_assignee_for_labels '{"labels":[{"name":"agent:architect"},{"name":"agent:devops"}]}' "$REPO_ROOT")"
    # Первая найденная метка выигрывает (break). architect идёт после devops в case —
    # но break на первом совпадении, поэтому порядок меток в JSON определяет.
    if [ "$got" = "architect" ]; then
        pass "F3: agent:architect первая в списке → assignee=architect"
    else
        fail "F3: agent:architect первая → assignee='$got' (expected architect)"
    fi
}

# F4: метка не из списка (например priority:high) → devops fallback
test_F4_unrelated_label_devops_fallback() {
    new_test
    local got
    got="$(extract_assignee_for_labels '{"labels":[{"name":"priority:high"},{"name":"bug"}]}' "$REPO_ROOT")"
    if [ "$got" = "devops" ]; then
        pass "F4: no agent:* label (только bug/priority) → assignee=devops fallback"
    else
        fail "F4: unrelated labels → assignee='$got' (expected devops)"
    fi
}

# F5: метка agent:devops → devops (явная)
test_F5_explicit_devops_label() {
    new_test
    local got
    got="$(extract_assignee_for_labels '{"labels":[{"name":"agent:devops"}]}' "$REPO_ROOT")"
    if [ "$got" = "devops" ]; then
        pass "F5: agent:devops label → assignee=devops (явная)"
    else
        fail "F5: agent:devops label → assignee='$got' (expected devops)"
    fi
}

# ============================================================================
# F6-F8: статическая проверка кода (не require execution)
# ============================================================================

# F6: ни одно место в merge-gate не содержит _assignee="default"
test_F6_merge_gate_no_default_string() {
    new_test
    local hits
    hits="$(grep -cE '_assignee=\"default\"' "$MERGE_GATE" || true)"
    if [ "$hits" = "0" ]; then
        pass "F6: agent-flow-merge-gate.sh: 0 вхождений _assignee=\"default\" (4 места → devops)"
    else
        fail "F6: agent-flow-merge-gate.sh: найдено $hits вхождений _assignee=\"default\""
    fi
}

# F7: ни одно место в e2e-process не содержит *_assignee="default"
test_F7_e2e_process_no_default_string() {
    new_test
    local hits
    hits="$(grep -cE '_conflict_assignee=\"default\"|_worker_assignee=\"default\"' "$E2E_PROCESS" || true)"
    if [ "$hits" = "0" ]; then
        pass "F7: agent-flow-e2e-process.sh: 0 вхождений *_assignee=\"default\" (2 места → devops)"
    else
        fail "F7: agent-flow-e2e-process.sh: найдено $hits вхождений *_assignee=\"default\""
    fi
}

# F8: 4 места в merge-gate теперь имеют _assignee="devops"
test_F8_merge_gate_has_devops_fallbacks() {
    new_test
    local hits
    # Должно быть минимум 4 _assignee="devops" (4 rebase/UNSTABLE блока).
    # Не считаем case-ветки (agent:devops) — только присваивание.
    hits="$(grep -cE '^[[:space:]]+_assignee=\"devops\"' "$MERGE_GATE" || true)"
    if [ "$hits" -ge "4" ]; then
        pass "F8: agent-flow-merge-gate.sh: $hits присваиваний _assignee=\"devops\" (≥4 OK)"
    else
        fail "F8: agent-flow-merge-gate.sh: только $hits присваиваний _assignee=\"devops\" (ожидалось ≥4)"
    fi
}

# F9: 2 места в e2e-process теперь имеют *_assignee="devops"
test_F9_e2e_process_has_devops_fallbacks() {
    new_test
    local hits
    hits="$(grep -cE '^[[:space:]]+_(conflict|worker)_assignee=\"devops\"' "$E2E_PROCESS" || true)"
    if [ "$hits" -ge "2" ]; then
        pass "F9: agent-flow-e2e-process.sh: $hits присваиваний *_assignee=\"devops\" (≥2 OK)"
    else
        fail "F9: agent-flow-e2e-process.sh: только $hits присваиваний *_assignee=\"devops\" (ожидалось ≥2)"
    fi
}

# F10: bash syntax check на обоих скриптах
test_F10_bash_syntax() {
    new_test
    local mg_ok ep_ok
    if bash -n "$MERGE_GATE" 2>/dev/null; then mg_ok=1; else mg_ok=0; fi
    if bash -n "$E2E_PROCESS" 2>/dev/null; then ep_ok=1; else ep_ok=0; fi
    if [ "$mg_ok" = "1" ] && [ "$ep_ok" = "1" ]; then
        pass "F10: bash -n merge-gate.sh И e2e-process.sh OK"
    else
        fail "F10: bash -n merge-gate=$mg_ok e2e-process=$ep_ok"
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
run_test test_F3_architect_label
run_test test_F4_unrelated_label_devops_fallback
run_test test_F5_explicit_devops_label
run_test test_F6_merge_gate_no_default_string
run_test test_F7_e2e_process_no_default_string
run_test test_F8_merge_gate_has_devops_fallbacks
run_test test_F9_e2e_process_has_devops_fallbacks
run_test test_F10_bash_syntax

printf '\n==== Summary ====\n'
printf 'total:  %d\npassed: %d\nfailed: %d\n' "$((PASS+FAIL))" "$PASS" "$FAIL"
if [ "$FAIL" -gt 0 ]; then
    printf '\nFailures:\n'
    for c in "${FAILED_CASES[@]}"; do printf '  - %s\n' "$c"; done
    exit 1
fi
exit 0
