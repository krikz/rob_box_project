#!/usr/bin/env bash
# ============================================================================
# test_wait_run.sh — контрактный тест lib_agent_flow_common.sh::wait_run
# (issue #2302, ретро 09.09.2026: один модуль poll+retry+race-fix для workflow
# polling). До этого фикса одна и та же логика копипастой жила в двух местах
# e2e-process.sh (wait_workflow + inline verdict-цикл), а race-fix
# `in_progress→success` (ретро 01.09 t_32c28562) приходилось зеркалить
# руками. Здесь тестируем изолированно через mock gh (PATH-shim), без сети.
#
# 6 кейсов покрывают весь контракт wait_run:
#   T1. happy path: run_id=12345, status=completed сразу, conclusion=success
#   T2. conclusion retry: первый poll даёт пустой conclusion, второй — success
#       (ретро 09.08 #4: race пустого/null conclusion)
#   T3. race-fix `in_progress→success`: initial=failure, 1-й recheck=success
#       → итог success, WR_RUN_RACE_DETECTED=1 (ретро 01.09 t_32c28562)
#   T4. timeout+cancel: status всегда in_progress до бюджета → rc=1,
#       cancel вызван, итог = timed_out
#   T5. invalid run_id → rc=2, cancel НЕ вызван (программная ошибка)
#   T6. cancel_on_timeout=0 → timeout без cancel (e2e-фаза)
#
# Run:
#   bash scripts/agent_flow/tests/test_wait_run.sh
# ============================================================================
set -u

TEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SCRIPT_LIB="${SCRIPT_LIB:-$TEST_DIR/../lib_agent_flow_common.sh}"

[ -f "$SCRIPT_LIB" ] || { echo "FAIL: $SCRIPT_LIB not found"; exit 1; }
command -v bash >/dev/null || { echo "FAIL: bash required"; exit 1; }
command -v python3 >/dev/null || { echo "FAIL: python3 required"; exit 1; }

PASS=0
FAIL=0
WORK=$(mktemp -d)
trap 'rm -rf "$WORK"' EXIT

# ----------------------------------------------------------------------------
# Извлечь wait_run + _af_log из lib (brace-tracking через awk). Берём ТОЛЬКО
# эти две функции, остальное не нужно.
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
    ' "$SCRIPT_LIB" > "$outfile"
}

mkdir -p "$WORK/bin"
extract_func "_af_log" "$WORK/lib_af_log.sh"
extract_func "wait_run" "$WORK/lib_wait_run.sh"

# log() — _af_log делегирует в log, если тот определён. Подсунем заглушку.
cat > "$WORK/lib_log.sh" <<'LOGEOF'
log() {
    printf '[%s] %s\n' "$(date -u +%Y-%m-%dT%H:%M:%SZ)" "$*" >&2
}
LOGEOF

# ----------------------------------------------------------------------------
# Mock gh (через PATH). Отдельные счётчики для status / conclusion / cancel:
# реальный wait_run делает status-poll → conclusion-poll, потом retry-loop
# conclusion-poll'ов, потом race-fix-loop (ещё conclusion-poll'ы). Один
# счётчик для всех типов делает тесты нечитаемыми; разделение по типу даёт
# каждому тесту явный, причинно-следственный сценарий.
#
# Контракт mock:
#   MOCK_STATUS_FILE     — JSON [{"i": <call_idx>, "status": "<...>"}]
#   MOCK_CONCLUSION_FILE — JSON [{"i": <call_idx>, "conclusion": "<...>"}]
#   MOCK_CANCEL_LOG      — файл, куда пишем "cancel <run_id>" при cancel
# ----------------------------------------------------------------------------
cat > "$WORK/bin/gh" <<'GHEOF'
#!/bin/bash
{ echo "MOCK_GH args=$* PATH=$PATH" >&2; } >>/tmp/_mock_gh.log
_args=("$@")
_action="${_args[0]:-}"
_sub="${_args[1]:-}"

# Найти --jq '<expr>' (для status / conclusion)
_jq_field=""
for ((i=1; i<${#_args[@]}; i++)); do
    if [ "${_args[$i]}" = "--jq" ] && [ $((i+1)) -lt ${#_args[@]} ]; then
        _jq_field="${_args[$((i+1))]}"
    fi
done

# Type-based counter (отдельный на status/conclusion)
case "$_jq_field" in
    *".status"*)
        _kind="status"
        ;;
    *".conclusion"*)
        _kind="conclusion"
        ;;
    *)
        _kind=""
        ;;
esac

if [ -n "$_kind" ] && [ -n "${MOCK_COUNTER_DIR:-}" ]; then
    _idx_file="$MOCK_COUNTER_DIR/.$_kind"
    _idx="$(cat "$_idx_file" 2>/dev/null || echo 0)"
    _idx=$((_idx+1))
    echo "$_idx" > "$_idx_file"
else
    _idx=0
fi

_journal_pick() {
            MOCK_JF="$1" MOCK_IDX="$2" MOCK_FLD="$3" python3 -c "
import json, os
try:
    with open(os.environ['MOCK_JF']) as fh:
        data = json.load(fh)
    idx = int(os.environ['MOCK_IDX'])
    for r in data:
        if r.get('i', 0) >= idx:
            print(r.get(os.environ['MOCK_FLD'], ''))
            raise SystemExit
    if data:
        print(data[-1].get(os.environ['MOCK_FLD'], ''))
except Exception:
    print('')
" 2>/dev/null
        }

case "$_action" in
    run)
        case "$_sub" in
            view)
                echo "DEBUG_VIEW kind=$_kind idx=$_idx MS=${MOCK_STATUS_FILE:-EMPTY} MC=${MOCK_CONCLUSION_FILE:-EMPTY}" >&2
                if [ "$_kind" = "status" ] && [ -n "${MOCK_STATUS_FILE:-}" ]; then
                    _journal_pick "$MOCK_STATUS_FILE" "$_idx" "status"
                elif [ "$_kind" = "conclusion" ] && [ -n "${MOCK_CONCLUSION_FILE:-}" ]; then
                    _journal_pick "$MOCK_CONCLUSION_FILE" "$_idx" "conclusion"
                else
                    echo ""
                fi
                ;;
            cancel)
                echo "MOCK cancel called, args=$* MOCK_CANCEL_LOG=${MOCK_CANCEL_LOG:-EMPTY}" >&2
                if [ -n "${MOCK_CANCEL_LOG:-}" ]; then
                    # _args from "$@" (without $0): [run, cancel, <run_id>, --repo, <repo>]
                    echo "cancel ${_args[2]}" >> "$MOCK_CANCEL_LOG"
                fi
                ;;
        esac
        ;;
esac
exit 0
GHEOF
chmod +x "$WORK/bin/gh"

# ----------------------------------------------------------------------------
# Helper: запустить wait_run в isolated env.
# ----------------------------------------------------------------------------
run_wait_run() {
    # args: rid timeout lbl cancel_on_timeout
    # Запускаем wait_run в subshell bash с доступом к mock gh через PATH и
    # к журналам через env от родителя. subshell нужен потому что wait_run
    # сам делает ~sleeps 5-10с между poll'ами; держим тест в одном
    # процессе, не плодим детей.
    local rid="$1" tmo="$2" lbl="$3" cancel="${4:-1}"
    # MOCK_STATUS_FILE / MOCK_CONCLUSION_FILE — глобально выставлены
    # caller-тестом через `export` (см. ниже). Без export subshell их
    # не видит, и mock gh возвращает пусто (ловим TIMEOUT вместо happy
    # path).
    PATH="$WORK/bin:$PATH" \
        GH_REPO="${GH_REPO:-krikz/rob_box_project}" \
        E2E_POLL_INTERVAL="${E2E_POLL_INTERVAL:-1}" \
        MOCK_COUNTER_DIR="$WORK" \
        MOCK_STATUS_FILE="${MOCK_STATUS_FILE:-}" \
        MOCK_CONCLUSION_FILE="${MOCK_CONCLUSION_FILE:-}" \
        MOCK_CANCEL_LOG="${MOCK_CANCEL_LOG:-}" \
        bash -c "
set -u
source '$WORK/lib_log.sh'
source '$WORK/lib_af_log.sh'
source '$WORK/lib_wait_run.sh'
wait_run '$rid' '$tmo' '$lbl' '${GH_REPO:-krikz/rob_box_project}' '${E2E_POLL_INTERVAL:-1}' '$cancel'
echo \"RC=\$?\"
echo \"RACE=\${WR_RUN_RACE_DETECTED:-0}\"
"
}

reset_counters() {
    rm -f "$WORK/.status" "$WORK/.conclusion"
}

# ============================================================================
# T1. happy path
# ============================================================================
echo "=== T1: happy path (status=completed, conclusion=success) ==="
cat > "$WORK/status.json" <<'JSON'
[{"i": 1, "status": "in_progress"}, {"i": 2, "status": "completed"}]
JSON
cat > "$WORK/conclusion.json" <<'JSON'
[{"i": 1, "conclusion": "success"}]
JSON
reset_counters
MOCK_STATUS_FILE="$WORK/status.json" \
MOCK_CONCLUSION_FILE="$WORK/conclusion.json" \
E2E_POLL_INTERVAL=1 \
result="$(run_wait_run "12345" 10 "build" 1)"
echo "$result"
if echo "$result" | grep -q '^success$' && echo "$result" | grep -q '^RC=0$' && echo "$result" | grep -q '^RACE=0$'; then
    echo "PASS T1"
    PASS=$((PASS+1))
else
    echo "FAIL T1"
    FAIL=$((FAIL+1))
fi

# ============================================================================
# T2. conclusion retry (ретро 09.08 #4): conclusion пустой/null первые 2 раза,
#     3-й раз = success. wait_run должен перечитывать до 3 раз.
# ============================================================================
echo "=== T2: conclusion retry (3× retry, 3rd=success) ==="
cat > "$WORK/status.json" <<'JSON'
[{"i": 1, "status": "completed"}]
JSON
cat > "$WORK/conclusion.json" <<'JSON'
[{"i": 1, "conclusion": ""}, {"i": 2, "conclusion": "null"}, {"i": 3, "conclusion": "success"}]
JSON
reset_counters
MOCK_STATUS_FILE="$WORK/status.json" \
MOCK_CONCLUSION_FILE="$WORK/conclusion.json" \
E2E_POLL_INTERVAL=1 \
result="$(run_wait_run "22222" 10 "deploy" 1)"
echo "$result"
if echo "$result" | grep -q '^success$' && echo "$result" | grep -q '^RC=0$'; then
    echo "PASS T2"
    PASS=$((PASS+1))
else
    echo "FAIL T2"
    FAIL=$((FAIL+1))
fi

# ============================================================================
# T3. race-fix `in_progress→success` (ретро 01.09 t_32c28562): initial=failure,
#     recheck#1=success → итог=success, RACE=1.
# ============================================================================
echo "=== T3: race-fix (initial=failure, recheck#1=success) ==="
cat > "$WORK/status.json" <<'JSON'
[{"i": 1, "status": "completed"}]
JSON
cat > "$WORK/conclusion.json" <<'JSON'
[{"i": 1, "conclusion": "failure"}, {"i": 2, "conclusion": "success"}]
JSON
reset_counters
MOCK_STATUS_FILE="$WORK/status.json" \
MOCK_CONCLUSION_FILE="$WORK/conclusion.json" \
E2E_POLL_INTERVAL=1 \
result="$(run_wait_run "33333" 30 "e2e" 0)"
echo "$result"
if echo "$result" | grep -q '^success$' && echo "$result" | grep -q '^RC=0$' && echo "$result" | grep -q '^RACE=1$'; then
    echo "PASS T3"
    PASS=$((PASS+1))
else
    echo "FAIL T3"
    FAIL=$((FAIL+1))
fi

# ============================================================================
# T4. timeout+cancel (ретро 13.08 t_da3e0bd5): status всегда in_progress,
#     timeout → rc=1, cancel вызван, итог=timed_out.
# ============================================================================
echo "=== T4: timeout + cancel ==="
cat > "$WORK/status.json" <<'JSON'
[{"i": 1, "status": "in_progress"}]
JSON
reset_counters
rm -f "$WORK/cancel.log"
export MOCK_STATUS_FILE="$WORK/status.json"
export MOCK_CANCEL_LOG="$WORK/cancel.log"
export E2E_POLL_INTERVAL=1
result="$(run_wait_run "44444" 2 "build" 1)"
echo "$result"
if echo "$result" | grep -q '^timed_out$' && echo "$result" | grep -q '^RC=1$' && [ -s "$WORK/cancel.log" ] && grep -q "cancel 44444" "$WORK/cancel.log"; then
    echo "PASS T4"
    PASS=$((PASS+1))
else
    echo "FAIL T4: cancel_log='$(cat "$WORK/cancel.log" 2>/dev/null)'"
    FAIL=$((FAIL+1))
fi
unset MOCK_STATUS_FILE MOCK_CANCEL_LOG

# ============================================================================
# T5. invalid run_id → rc=2, cancel НЕ вызван (программная ошибка, не сеть).
# ============================================================================
echo "=== T5: invalid run_id (rc=2, no cancel) ==="
rm -f "$WORK/cancel.log"
result="$(run_wait_run "not-a-number" 5 "build" 1)"
echo "$result"
if echo "$result" | grep -q '^RC=2$' && [ ! -s "$WORK/cancel.log" ]; then
    echo "PASS T5"
    PASS=$((PASS+1))
else
    echo "FAIL T5: cancel='$(cat "$WORK/cancel.log" 2>/dev/null)'"
    FAIL=$((FAIL+1))
fi

# ============================================================================
# T6. e2e cancel_on_timeout=0: timeout НЕ cancel'ит run (в отличие от build).
# ============================================================================
echo "=== T6: e2e cancel_on_timeout=0 → no cancel on timeout ==="
cat > "$WORK/status.json" <<'JSON'
[{"i": 1, "status": "in_progress"}]
JSON
reset_counters
rm -f "$WORK/cancel.log"
export MOCK_STATUS_FILE="$WORK/status.json"
export MOCK_CANCEL_LOG="$WORK/cancel.log"
export E2E_POLL_INTERVAL=1
result="$(run_wait_run "55555" 2 "e2e" 0)"
echo "$result"
if echo "$result" | grep -q '^timed_out$' && echo "$result" | grep -q '^RC=1$' && [ ! -s "$WORK/cancel.log" ]; then
    echo "PASS T6"
    PASS=$((PASS+1))
else
    echo "FAIL T6: cancel='$(cat "$WORK/cancel.log" 2>/dev/null)'"
    FAIL=$((FAIL+1))
fi
unset MOCK_STATUS_FILE MOCK_CANCEL_LOG

echo ""
echo "===================="
echo "PASS: $PASS / 6"
echo "FAIL: $FAIL / 6"
echo "===================="
[ "$FAIL" = "0" ] && exit 0 || exit 1