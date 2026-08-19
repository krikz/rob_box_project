#!/bin/bash
# ============================================================================
# test_e2e_process_post_round_sweep_override.sh — bug(process #1450) ретро 19.08 t_723a539d
#
# Проверяет фикс post-round sweep в agent-flow-e2e-process.sh:
#   если Шифу ВРУЧНУЮ вернул needs-e2e на issue после того как round
#   завершился SUCCESS — sweep должен SKIP (НЕ ставить e2e-done).
#
# Сигнал: latest LabeledEvent{label=needs-e2e, actor≠bot} timestamp >
# workflow run createdAt. Это явный re-test request от Шифу (ретро 18.08
# t_de6bea69, та же философия что у user_removed_label_recently).
#
# Сценарии:
#   A. needs-e2e labeled РАНЬШЕ run (нормальный SUCCESS)         → guard = 0
#   B. needs-e2e labeled ПОЗЖЕ run (Шифу override)              → guard = 1
#   C. needs-e2e labeled bot'ом (auto-cleanup) ПОЗЖЕ run         → guard = 0
#   D. timeline API сдох → fail-OPEN (guard = 0)
#   E. run createdAt недоступен → fail-OPEN (guard = 0)
#   F. событий нет → guard = 0
#   G. needs-e2e labeled ПОЗЖЕ, потом unlabeled ПОЗЖЕ           → guard = 0 (нет labeled после run)
#   H. needs-e2e labeled bot'ом РАНЬШЕ, юзером ПОЗЖЕ             → guard = 1
#   I. Sanity: функция issue_needs_e2e_re_added_after_run присутствует
#      в скрипте и используется в post_round_sweep
#
# Run:
#   bash scripts/agent_flow/tests/test_e2e_process_post_round_sweep_override.sh
# ============================================================================
set -euo pipefail

TEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# tests/ → scripts/agent_flow/tests/ → scripts/agent_flow/ → scripts/ → REPO_ROOT
REPO_ROOT="$(cd "$TEST_DIR/../../.." && pwd)"
E2E_PROCESS="$REPO_ROOT/scripts/agent_flow/agent-flow-e2e-process.sh"

# --- Helpers --------------------------------------------------------------
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

# --- Source скрипт в test-mode (выдёргиваем функцию без запуска main loop) --
# Скрипт начинается с shebang + комментариев, потом `set -euo pipefail`,
# потом function-definitions. Нам нужна ТОЛЬКО функция
# issue_needs_e2e_re_added_after_run, плюс переменные POST_ROUND_SWEEP_TEST_MODE.
# Подход: source'им через `bash -c`, выставив mode ДО source.
extract_function() {
    # awk: от маркера '# issue_needs_e2e_re_added_after_run' до строки
    # содержащей `^}` (закрытие функции).
    awk '
        /^# issue_needs_e2e_re_added_after_run / { in_fn=1 }
        in_fn { print }
        in_fn && /^}$/ { in_fn=0 }
    ' "$E2E_PROCESS"
}

# --- Sanity-check: функция присутствует в скрипте --------------------------
echo "=== Sanity: функция issue_needs_e2e_re_added_after_run в $E2E_PROCESS ==="
if grep -q 'issue_needs_e2e_re_added_after_run() {' "$E2E_PROCESS"; then
    pass "функция issue_needs_e2e_re_added_after_run определена в скрипте"
else
    fail "функция определена" "issue_needs_e2e_re_added_after_run() не найдена"
fi

if grep -q 'issue_needs_e2e_re_added_after_run "\$_sn" "\$_sweep_run_id"' "$E2E_PROCESS"; then
    pass "функция вызывается в post_round_sweep"
else
    fail "вызов в sweep" "строка вызова не найдена"
fi

if grep -q 'имеет явный needs-e2e override ПОСЛЕ run' "$E2E_PROCESS"; then
    pass "лог-сообщение 'имеет явный needs-e2e override' присутствует"
else
    fail "лог-сообщение" "не найдено в скрипте"
fi

if grep -q 'POST_ROUND_SWEEP_TEST_MODE' "$E2E_PROCESS"; then
    pass "test-mode хук (POST_ROUND_SWEEP_TEST_MODE) присутствует"
else
    fail "test-mode" "POST_ROUND_SWEEP_TEST_MODE не найден"
fi

# --- Загружаем функцию в текущий shell ------------------------------------
_FN_SRC="$(extract_function)"
if [ -z "$_FN_SRC" ]; then
    fail "extract_function" "не удалось извлечь тело функции из $E2E_PROCESS"
    echo ""
    echo "=== Итоги ==="
    echo "Всего: $TESTS_TOTAL  ${GRN}passed: $TESTS_PASSED${END}  ${RED}failed: $TESTS_FAILED${END}"
    exit 1
fi

# Source функцию в текущий шелл (eval нужен т.к. awk даёт многострочный код).
# shellcheck disable=SC2086
eval "$_FN_SRC"

# Экспортируем функцию чтобы она была доступна внутри `bash -c` ниже.
export -f issue_needs_e2e_re_added_after_run

# Проверяем что функция доступна
if ! declare -f issue_needs_e2e_re_added_after_run >/dev/null; then
    fail "issue_needs_e2e_re_added_after_run не загрузилась" "declare -f пусто"
    exit 1
fi
pass "функция загружена в тестовый shell"

# --- Helpers для тестов ----------------------------------------------------
# run_epoch: epoch для run
# label_epoch: epoch для labeled event
# actor: 'shifu' / 'github-actions[bot]' / 'dependabot[bot]'
make_labeled_event() {  # $1=epoch(iso) $2=label $3=actor
    local at="$1" lbl="$2" actor="$3"
    printf '{"event":"labeled","label":{"name":"%s"},"actor":{"login":"%s"},"created_at":"%s"}' "$lbl" "$actor" "$at"
}

# epoch → iso8601 UTC. Без python-dateutil: через GNU date.
epoch_to_iso() {  # $1=epoch
    date -u -d "@$1" '+%Y-%m-%dT%H:%M:%SZ' 2>/dev/null || echo ""
}

# Конструктор timeline JSON: принимает N пар "epoch actor label" в виде args.
make_timeline_json() {
    local first=1
    printf '['
    while [ $# -gt 0 ]; do
        local epoch="$1" actor="$2" lbl="$3"
        shift 3 || break
        local iso
        iso="$(epoch_to_iso "$epoch")"
        [ "$first" = "0" ] && printf ','
        first=0
        printf '{"event":"labeled","label":{"name":"%s"},"actor":{"login":"%s"},"created_at":"%s"}' "$lbl" "$actor" "$iso"
    done
    printf ']'
}

# Wrapper: вызывает функцию в test-mode с заданным timeline и run_epoch.
# Печатает "1" если guard сработал (skip), "0" если не сработал.
call_guard() {  # $1=timeline_json $2=run_epoch
    POST_ROUND_SWEEP_TEST_MODE=1 \
    _POST_ROUND_SWEEP_TEST_TIMELINE_JSON="$1" \
    _POST_ROUND_SWEEP_TEST_RUN_EPOCH="$2" \
    bash -c 'if issue_needs_e2e_re_added_after_run 1392 12345; then echo 1; else echo 0; fi'
}

# --- Test A: needs-e2e labeled РАНЬШЕ run (нормальный SUCCESS) -------------
echo ""
echo "=== Test A: labeled РАНЬШЕ run → guard = 0 (sweep лечит) ==="
_RUN_EPOCH=1700000000  # round-154 SUCCESS created at
# needs-e2e labeled ДО run (например, в начале тика)
_TIMELINE="$(make_timeline_json \
    "$((_RUN_EPOCH - 3600))" "shifu" "needs-e2e" \
)"
assert_eq "labeled before run → no skip" "0" "$(call_guard "$_TIMELINE" "$_RUN_EPOCH")"

# --- Test B: needs-e2e labeled ПОЗЖЕ run (Шифу override) ------------------
echo ""
echo "=== Test B: labeled ПОЗЖЕ run → guard = 1 (sweep skip) ==="
# needs-e2e labeled ПОСЛЕ run (Шифу вручную вернул для re-test)
_TIMELINE="$(make_timeline_json \
    "$((_RUN_EPOCH + 7200))" "shifu" "needs-e2e" \
)"
assert_eq "labeled after run by user → skip" "1" "$(call_guard "$_TIMELINE" "$_RUN_EPOCH")"

# --- Test C: needs-e2e labeled bot'ом ПОЗЖЕ run (не user-decisional) -------
echo ""
echo "=== Test C: labeled bot'ом ПОЗЖЕ run → guard = 0 (не user) ==="
_TIMELINE="$(make_timeline_json \
    "$((_RUN_EPOCH + 7200))" "github-actions[bot]" "needs-e2e" \
)"
assert_eq "bot label after run → no skip" "0" "$(call_guard "$_TIMELINE" "$_RUN_EPOCH")"

# --- Test D: timeline API сдох → fail-OPEN ---------------------------------
echo ""
echo "=== Test D: timeline API пустой → fail-OPEN (guard = 0) ==="
assert_eq "empty timeline → fail-OPEN" "0" "$(call_guard "[]" "$_RUN_EPOCH")"

# --- Test E: run createdAt недоступен → fail-OPEN -------------------------
echo ""
echo "=== Test E: run_epoch=0 (недоступен) → fail-OPEN ==="
_TIMELINE="$(make_timeline_json \
    "$((_RUN_EPOCH + 7200))" "shifu" "needs-e2e" \
)"
assert_eq "run_epoch=0 → fail-OPEN" "0" "$(call_guard "$_TIMELINE" "0")"

# --- Test F: событий нет (нулевая активность) → guard = 0 -----------------
echo ""
echo "=== Test F: timeline без labeled events → guard = 0 ==="
# timeline с другими событиями (но не labeled/needs-e2e)
_NOOP='[{"event":"commented","actor":{"login":"shifu"},"created_at":"2024-01-01T00:00:00Z"}]'
assert_eq "no labeled events → no skip" "0" "$(call_guard "$_NOOP" "$_RUN_EPOCH")"

# --- Test G: labeled+unlabeled cycle: latest labeled РАНЬШЕ run ------------
echo ""
echo "=== Test G: labeled, потом unlabeled — последнее действие unlabeled → guard = 0 ==="
# Шифу labeled (ДО run), потом unlabeled (ПОСЛЕ run) → последнее событие — unlabeled
# Timeline API показывает ОБА события (в т.ч. unlabeled), но наш guard смотрит
# только на labeled. Последний labeled был ДО run → нет override сигнала.
_LABEL_ISO=$(date -u -d "@$((_RUN_EPOCH - 3600))" '+%Y-%m-%dT%H:%M:%SZ')
_UNLAB_ISO=$(date -u -d "@$((_RUN_EPOCH + 7200))" '+%Y-%m-%dT%H:%M:%SZ')
export _LABEL_ISO _UNLAB_ISO
_TIMELINE=$(cat <<EOF
[
{"event":"labeled","label":{"name":"needs-e2e"},"actor":{"login":"shifu"},"created_at":"${_LABEL_ISO}"},
{"event":"unlabeled","label":{"name":"needs-e2e"},"actor":{"login":"shifu"},"created_at":"${_UNLAB_ISO}"}
]
EOF
)
unset _LABEL_ISO _UNLAB_ISO
assert_eq "labeled before, unlabeled after → no skip" "0" "$(call_guard "$_TIMELINE" "$_RUN_EPOCH")"

# --- Test H: bot labeled раньше, user labeled позже → guard = 1 ------------
echo ""
echo "=== Test H: bot labeled РАНЬШЕ, user labeled ПОЗЖЕ → guard = 1 ==="
# bot создал needs-e2e ДО run (нормальная auto-установка), потом sweep снял
# (unlabeled event между), потом Шифу вернул labeled event ПОСЛЕ run.
# Последний user-labeled ПОСЛЕ run → override.
_BOT_ISO=$(date -u -d "@$((_RUN_EPOCH - 7200))" '+%Y-%m-%dT%H:%M:%SZ')
_UNLAB_ISO=$(date -u -d "@$((_RUN_EPOCH + 3600))" '+%Y-%m-%dT%H:%M:%SZ')
_USER_ISO=$(date -u -d "@$((_RUN_EPOCH + 10800))" '+%Y-%m-%dT%H:%M:%SZ')
export _BOT_ISO _UNLAB_ISO _USER_ISO
_TIMELINE=$(cat <<EOF
[
{"event":"labeled","label":{"name":"needs-e2e"},"actor":{"login":"github-actions[bot]"},"created_at":"${_BOT_ISO}"},
{"event":"unlabeled","label":{"name":"needs-e2e"},"actor":{"login":"github-actions[bot]"},"created_at":"${_UNLAB_ISO}"},
{"event":"labeled","label":{"name":"needs-e2e"},"actor":{"login":"shifu"},"created_at":"${_USER_ISO}"}
]
EOF
)
unset _BOT_ISO _UNLAB_ISO _USER_ISO
assert_eq "bot+user labeled (latest user after run) → skip" "1" "$(call_guard "$_TIMELINE" "$_RUN_EPOCH")"

# --- Test I: dependabot actor → bot ----------------------------------------
echo ""
echo "=== Test I: dependabot[bot] labeled → не считается user ==="
_TIMELINE="$(make_timeline_json \
    "$((_RUN_EPOCH + 7200))" "dependabot[bot]" "needs-e2e" \
)"
assert_eq "dependabot[bot] after run → no skip" "0" "$(call_guard "$_TIMELINE" "$_RUN_EPOCH")"

# --- Test J: другой label labeled ПОЗЖЕ → не триггерит --------------------
echo ""
echo "=== Test J: другой label (не needs-e2e) labeled ПОЗЖЕ → guard = 0 ==="
_TIMELINE="$(make_timeline_json \
    "$((_RUN_EPOCH + 7200))" "shifu" "e2e-done" \
)"
assert_eq "e2e-done label after run → no skip" "0" "$(call_guard "$_TIMELINE" "$_RUN_EPOCH")"

# --- Test K: смешанная хронология (точная репродукция #1392) --------------
echo ""
echo "=== Test K: точная хронология #1392 — bot labels, sweep unlabels, user relabels ==="
# Воспроизводим наблюдение 19.08 (issue #1392):
# 1) Bot: needs-e2e (worker PR opened)
# 2) Bot: needs-e2e remove (e2e-done поставлен sweep'ом round-154 SUCCESS)
# 3) Шифу: needs-e2e add (override → "re-test с music_library_suite_v1")
# 4) Sweep следующего тика должен SKIP потому что step 3 ПОСЛЕ round-154 run
# Только labeled-events нужны для guard; шаг 2 (unlabeled) тут не релевантен.
_USER_ISO=$(date -u -d "@$((_RUN_EPOCH + 14400))" '+%Y-%m-%dT%H:%M:%SZ')
_BOT_ISO=$(date -u -d "@$((_RUN_EPOCH - 3600))" '+%Y-%m-%dT%H:%M:%SZ')
export _USER_ISO _BOT_ISO
_TIMELINE=$(cat <<EOF
[
{"event":"labeled","label":{"name":"needs-e2e"},"actor":{"login":"github-actions[bot]"},"created_at":"${_BOT_ISO}"},
{"event":"labeled","label":{"name":"needs-e2e"},"actor":{"login":"shifu"},"created_at":"${_USER_ISO}"}
]
EOF
)
unset _USER_ISO _BOT_ISO
assert_eq "mixed chron #1392 → skip (Шифу override)" "1" "$(call_guard "$_TIMELINE" "$_RUN_EPOCH")"

# --- Итоги ----------------------------------------------------------------
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
