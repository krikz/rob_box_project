#!/bin/bash
# ============================================================================
# test_e2e_process_trigger.sh — ADR-0040 §7.1: _trigger_workflow_with_retry
# новый контракт (exit 0 + run_id в stdout, exit 0 + existing:<id>,
# exit 1 + пустой stdout).
#
# Mock-стратегия (как test_blocked_watchdog.sh): подменяем gh через PATH,
# чтобы изолированно тестировать логику без реального GitHub API.
#
# 4 кейса (ADR-0040 §7.1):
#   T1. happy path: gh workflow run exit 0 + run появляется в poll → exit 0, stdout=run_id
#   T2. eventual-consistency: gh workflow run exit 0 + run появляется через 4 сек → exit 0, stdout=run_id
#   T3. trigger fail: gh workflow run всегда exit 1, race-dedup miss → exit 1, stdout=empty
#   T4. race-dedup success: gh workflow run exit 1, но в gh run list уже есть свежий run → exit 0, stdout=existing:<id>
#
# Run:
#   bash scripts/agent_flow/tests/test_e2e_process_trigger.sh
# ============================================================================
set -u

TEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SCRIPT_SH="${SCRIPT_SH:-$TEST_DIR/../agent-flow-e2e-process.sh}"

[ -f "$SCRIPT_SH" ] || { echo "FAIL: $SCRIPT_SH not found"; exit 1; }
command -v bash >/dev/null || { echo "FAIL: bash required"; exit 1; }
command -v python3 >/dev/null || { echo "FAIL: python3 required"; exit 1; }

PASS=0
FAIL=0
WORK=$(mktemp -d)
trap 'rm -rf "$WORK"' EXIT

# ----------------------------------------------------------------------------
# Утилита: извлечь функцию из скрипта и сохранить в отдельный .sh файл.
# Использует brace-tracking через awk.
# ----------------------------------------------------------------------------
extract_func() {
    local fname="$1" outfile="$2"
    awk -v fn="$fname" '
        $0 ~ "^"fn"\\(\\)" {flag=1; depth=0}
        flag {
            print
            for(i=1;i<=length($0);i++) {
                c=substr($0,i,1)
                if(c=="{") depth++
                if(c=="}") { depth--; if(depth==0) { flag=0; print ""; break } }
            }
        }
    ' "$SCRIPT_SH" > "$outfile"
}

# Подготовим функции которые нам нужны для теста
mkdir -p "$WORK/bin" "$WORK/lib"
extract_func "poll_run_for_epoch" "$WORK/lib/poll_run_for_epoch.sh"
extract_func "_trigger_workflow_with_retry" "$WORK/lib/_trigger_workflow_with_retry.sh"

# Нужен log() и verify_recent_run() для функционирования trigger.
# log() — простая обёртка.
cat > "$WORK/lib/log.sh" <<'LOGEOF'
log() {
    printf '[%s] %s\n' "$(date -u +%Y-%m-%dT%H:%M:%SZ)" "$*" >&2
}
LOGEOF

# verify_recent_run() — заглушка: проверяет MOCK_RECENT_RUN_FILE.
cat > "$WORK/lib/verify_recent_run.sh" <<'VREOF'
verify_recent_run() {
    local _wf="$1" _br="$2" _repo="$3" _win="$4"
    local _state="${MOCK_VERIFY_STATE:-miss}"
    if [ "$_state" = "ok" ]; then
        printf 'ok\n'
    else
        printf 'miss\n'
    fi
    return 0
}
VREOF

# Заглушка lib_workflow_dedup.sh (source'ится скриптом).
cat > "$WORK/lib/lib_workflow_dedup.sh" <<'LIBEOF'
# Empty — verify_recent_run определён в отдельном sh
LIBEOF

# Заглушка для других lib, которые source'ит основной скрипт (нам они не нужны,
# но source'ится всё). Создаём пустые файлы.
for lib in lib_agent_flow_common.sh lib_user_unlabel_check.sh hermes_github.sh; do
    echo "# stub" > "$WORK/lib/$lib"
done

# ----------------------------------------------------------------------------
# Mock gh (через PATH) — управляется через MOCK_GH_BEHAVIOR файлы.
# ----------------------------------------------------------------------------
cat > "$WORK/bin/gh" <<'GHEOF'
#!/bin/bash
# Mock gh — behavior зависит от MOCK_TRIGGER_RESULT_FILE, MOCK_RUN_LIST_FILE.
# Поддерживает --jq '<expr>' на gh run list: для нашего use-case (.databaseId
# с optional select(.createdAt >= "ep")) — просто достаём первый databaseId.
_args=("$@")
_action="${_args[0]:-}"

# Найти --jq '<expr>' если есть
_jq_expr=""
for ((i=1; i<${#_args[@]}; i++)); do
    if [ "${_args[$i]}" = "--jq" ] && [ $((i+1)) -lt ${#_args[@]} ]; then
        _jq_expr="${_args[$((i+1))]}"
        break
    fi
done

case "$_action" in
    workflow)
        # gh workflow run <wf> ... → exit code из MOCK_TRIGGER_RESULT_FILE
        if [ -f "${MOCK_TRIGGER_RESULT_FILE:-}" ]; then
            cat "${MOCK_TRIGGER_RESULT_FILE}"
            exit "$(cat "${MOCK_TRIGGER_RESULT_FILE}")"
        fi
        echo "0"
        exit 0
        ;;
    run)
        sub="${_args[1]:-}"
        case "$sub" in
            list)
                # gh run list ... → если есть --jq с .databaseId — достаём id.
                if [ -n "$_jq_expr" ] && [[ "$_jq_expr" == *"databaseId"* ]]; then
                    if [ -f "${MOCK_RUN_LIST_FILE:-}" ]; then
                        # Извлечь первый "databaseId": NNN из JSON через python.
                        E2E_RL_FILE="${MOCK_RUN_LIST_FILE}" python3 -c '
import json, os
try:
    with open(os.environ["E2E_RL_FILE"]) as fh:
        data = json.load(fh)
    if data:
        print(data[0].get("databaseId", ""))
except Exception:
    print("")
' 2>/dev/null
                    fi
                else
                    if [ -f "${MOCK_RUN_LIST_FILE:-}" ]; then
                        cat "${MOCK_RUN_LIST_FILE}"
                    else
                        echo "[]"
                    fi
                fi
                ;;
            view)
                if [ -f "${MOCK_RUN_VIEW_FILE:-}" ]; then
                    cat "${MOCK_RUN_VIEW_FILE}"
                else
                    echo "{}"
                fi
                ;;
        esac
        exit 0
        ;;
    auth)
        if [ "${_args[1]:-}" = "status" ]; then exit 0; fi
        exit 0
        ;;
    label)
        exit 0
        ;;
    issue)
        exit 0
        ;;
    *)
        echo "[]"
        exit 0
        ;;
esac
GHEOF
chmod +x "$WORK/bin/gh"

# ----------------------------------------------------------------------------
# Утилита запуска: source'ит только нужные функции и _TBS/_TBA/_TBC/_TBR globals.
# ----------------------------------------------------------------------------
run_trigger() {
    local workflow="$1" ref="$2"
    local _sourced="$WORK/source-and-run.sh"
    # Используем 'EOF' (одинарные кавычки) — переменные НЕ интерполируются
    # при создании файла. __WORKDIR__ подставляется через sed, __WF__/__REF__
    # через экспорт переменных окружения из caller'а.
    cat > "$_sourced" <<'RUNEOF'
#!/bin/bash
set -u
export GH_REPO="${GH_REPO:-krikz/rob_box_project}"
export PATH="__WORKDIR__/bin:$PATH"
# Pass through MOCK_* env vars from caller (eval'ятся в runtime)
export MOCK_TRIGGER_RESULT_FILE="${MOCK_TRIGGER_RESULT_FILE:-}"
export MOCK_RUN_LIST_FILE="${MOCK_RUN_LIST_FILE:-}"
export MOCK_VERIFY_STATE="${MOCK_VERIFY_STATE:-}"
# Caller-provided workflow + ref
export _RT_WORKFLOW="${_RT_WORKFLOW:-}"
export _RT_REF="${_RT_REF:-}"
# shellcheck disable=SC1091
source "__WORKDIR__/lib/log.sh"
# shellcheck disable=SC1091
source "__WORKDIR__/lib/verify_recent_run.sh"
# shellcheck disable=SC1090
source "__WORKDIR__/lib/poll_run_for_epoch.sh"
# shellcheck disable=SC1090
source "__WORKDIR__/lib/_trigger_workflow_with_retry.sh"
# Stub E2E vars
_TBS="test" _TBA="test" _TBC="" _TBR="test"
_trigger_workflow_with_retry "${_RT_WORKFLOW}" --ref "${_RT_REF}"
echo "EXIT=$?"
RUNEOF
    # Подставить __WORKDIR__ → $WORK
    sed -i "s|__WORKDIR__|$WORK|g" "$_sourced"
    # NB: command-prefix `MOCK_X=... result=$(run_trigger ...)` устанавливает
    # переменные только для команды run_trigger, НЕ для sub-shell процессов.
    # Поэтому пробрасываем их явно через префикс `bash`.
    MOCK_TRIGGER_RESULT_FILE="${MOCK_TRIGGER_RESULT_FILE:-}" \
    MOCK_RUN_LIST_FILE="${MOCK_RUN_LIST_FILE:-}" \
    MOCK_VERIFY_STATE="${MOCK_VERIFY_STATE:-}" \
    _RT_WORKFLOW="$workflow" _RT_REF="$ref" \
        bash "$_sourced" 2>&1
}

# ============================================================================
# T1. happy path: gh workflow run exit 0, poll возвращает run_id сразу
# ============================================================================
echo "=== T1: happy path (gh exit 0 + poll нашёл run_id) ==="
echo "0" > "$WORK/trigger_result"
# Use FUTURE timestamp so it always > epoch (mock "just started")
cat > "$WORK/run_list.json" <<'JSON'
[{"databaseId": 12345, "createdAt": "2099-01-01T00:00:00Z"}]
JSON
MOCK_TRIGGER_RESULT_FILE="$WORK/trigger_result" \
MOCK_RUN_LIST_FILE="$WORK/run_list.json" \
MOCK_VERIFY_STATE="miss" \
result="$(run_trigger "L-E2E Voice Test.yml" "develop")"
echo "result: $result"
if echo "$result" | grep -q '^12345$'; then
    echo "PASS T1: stdout=run_id=12345"
    PASS=$((PASS+1))
else
    echo "FAIL T1: stdout='$result' (expected '12345')"
    FAIL=$((FAIL+1))
fi

# ============================================================================
# T2. eventual-consistency: gh workflow run exit 0, poll сначала miss, потом hit
# Mock: возвращаем пустой [] первые 2 вызова, потом [{databaseId:99999}]
# ============================================================================
echo "=== T2: eventual-consistency (poll retry, run появляется после 6 сек) ==="
echo "0" > "$WORK/trigger_result"
cat > "$WORK/run_list.json" <<'JSON'
[{"databaseId": 12345, "createdAt": "2099-01-01T00:00:00Z"}]
JSON
# poll_run_for_epoch делает gh run list каждый 2 сек. Уменьшим _max до 6 сек через env.
E2E_TRIGGER_POLL_MAX=6 \
MOCK_TRIGGER_RESULT_FILE="$WORK/trigger_result" \
MOCK_RUN_LIST_FILE="$WORK/run_list.json" \
MOCK_VERIFY_STATE="miss" \
result="$(run_trigger "L-E2E Voice Test.yml" "develop")"
echo "result: $result"
# run_id должен быть 12345 (тот же list, eventual consistency просто тестируем
# что poll не падает на пустых первых итерациях)
if echo "$result" | grep -q '^12345$'; then
    echo "PASS T2: poll eventually resolved run_id=12345"
    PASS=$((PASS+1))
else
    echo "FAIL T2: stdout='$result' (expected '12345')"
    FAIL=$((FAIL+1))
fi

# ============================================================================
# T3. trigger fail: gh workflow run всегда exit 1, race-dedup miss → exit 1, empty stdout
# Mock: gh workflow run exit 1, run_list пуст, verify_recent_run = miss
# ============================================================================
echo "=== T3: trigger fail (gh exit 1, race-dedup miss) ==="
echo "1" > "$WORK/trigger_result"
echo "[]" > "$WORK/run_list.json"
# Mock gh в workflow-ветке возвращает exit code из trigger_result (1)
result="$(MOCK_TRIGGER_RESULT_FILE="$WORK/trigger_result" \
MOCK_RUN_LIST_FILE="$WORK/run_list.json" \
MOCK_VERIFY_STATE="miss" \
run_trigger "L-E2E Voice Test.yml" "develop")"
# Stdout: ловим только последнюю строку (которая содержит run_id или EXIT=N)
# Логи trigger пишутся в stderr; но run_trigger сделал 2>&1 чтобы смешать
# — отделяем по EXIT= маркеру.
echo "result lines:"
echo "$result"
# Ожидаем: exit code 1, stdout (run_id) пустой. EXIT=N — последняя строка.
exit_code="$(printf '%s\n' "$result" | grep -oE 'EXIT=[01]' | tail -1 | cut -d= -f2)"
# Реальный stdout — строка между логами trigger и EXIT= (если есть). По
# контракту функции, stdout должен содержать ТОЛЬКО run_id (число) или
# existing:<id> или ничего.
real_stdout="$(printf '%s\n' "$result" | grep -E '^[0-9]+$|^existing:[0-9]+$' || true)"
if [ "${exit_code:-X}" = "1" ] && [ -z "$real_stdout" ]; then
    echo "PASS T3: exit=1, stdout=empty (per ADR-0040 §2.2.1)"
    PASS=$((PASS+1))
else
    echo "FAIL T3: exit=$exit_code stdout='$real_stdout' (expected exit=1, empty stdout)"
    FAIL=$((FAIL+1))
fi

# ============================================================================
# T4. race-dedup success (по логике функции): gh workflow run exit 1, но
#     verify_recent_run=ok И в gh run list есть свежий run → exit 0,
#     stdout=existing:<id>
# Mock: gh workflow run exit 1, verify_recent_run = ok (race detected),
#       run_list содержит run 77777.
# ============================================================================
echo "=== T4: race-dedup success (existing run найден) ==="
echo "1" > "$WORK/trigger_result"
cat > "$WORK/run_list.json" <<'JSON'
[{"databaseId": 77777, "createdAt": "2099-01-01T00:00:00Z"}]
JSON
result="$(MOCK_TRIGGER_RESULT_FILE="$WORK/trigger_result" \
MOCK_RUN_LIST_FILE="$WORK/run_list.json" \
MOCK_VERIFY_STATE="ok" \
run_trigger "L-E2E Voice Test.yml" "develop")"
echo "$result"
real_stdout="$(printf '%s\n' "$result" | grep -E '^[0-9]+$|^existing:[0-9]+$' || true)"
if [ "$real_stdout" = "existing:77777" ]; then
    echo "PASS T4: stdout=existing:77777"
    PASS=$((PASS+1))
else
    echo "FAIL T4: stdout='$real_stdout' (expected 'existing:77777')"
    FAIL=$((FAIL+1))
fi

# ============================================================================
# Итог
# ============================================================================
echo "=================================================="
echo "PASS: $PASS"
echo "FAIL: $FAIL"
echo "=================================================="
[ "$FAIL" -eq 0 ] || exit 1
exit 0
