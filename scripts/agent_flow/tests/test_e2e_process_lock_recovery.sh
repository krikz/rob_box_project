#!/bin/bash
# ============================================================================
# test_e2e_process_lock_recovery.sh — ADR-0040 §7.3 / §2.2.4:
# stale lock-file recovery. Если lock содержит PID и этот PID мёртв
# (kill -0 возвращает non-zero) — process логирует «lock stale, recovered»
# и продолжает работу (не выходит с «already running»).
#
# Тестируем чистую логику flock + recovery. Поскольку полный скрипт делает
# много лишнего (gh API, MAINTENANCE gate, rate-limit), выделяем секцию G6
# (lock init) и тестируем её в изоляции через source.
#
# Scenarios:
#   L1. empty lock file: пустой /tmp/... → flock захватывает, recovery logged
#   L2. fresh lock (другой PID жив): flock уже занят → exit 0 без run
#   L3. stale lock (PID в файле мёртв): recovery → flock захватывает, run
#   L4. lock PID == свой PID: race-rare case, не ломаем (защита — flock)
#   L5. recovery writes PID:EPOCH (а не 0 bytes)
#
# Run:
#   bash scripts/agent_flow/tests/test_e2e_process_lock_recovery.sh
# ============================================================================
set -u

TEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SCRIPT_SH="${SCRIPT_SH:-$TEST_DIR/../agent-flow-e2e-process.sh}"

[ -f "$SCRIPT_SH" ] || { echo "FAIL: $SCRIPT_SH not found"; exit 1; }
command -v bash >/dev/null || { echo "FAIL: bash required"; exit 1; }

PASS=0
FAIL=0
WORK=$(mktemp -d)
trap 'rm -rf "$WORK"' EXIT

# Извлекаем секцию G6 (lock init) из скрипта. Это ПОСЛЕ всех .env/lib source,
# но ДО остального кода. Структура: heredoc + flock + recover block + write.
# Мы выделим только ключевую логику.
extract_lock_section() {
    local outfile="$1"
    # Секция идёт от "G6: flock sentinel" до "G6.5: ADR-0040 run-state init" (исключительно).
    # Добавляем log() stub ПЕРЕД секцией, потому что код вызывает log().
    {
        echo '# log() stub for isolated testing'
        echo 'log() {'
        echo '    printf "[%s] %s\n" "$(date -u +%Y-%m-%dT%H:%M:%SZ)" "$*" >&2'
        echo '}'
        echo ''
        awk '
            /^# --- G6: flock sentinel \+ ADR-0040 lock-recovery/ {flag=1; depth=0}
            flag {
                print
                if(/^# --- G6\.5: ADR-0040 run-state init/) { flag=0; exit }
            }
        ' "$SCRIPT_SH"
    } > "$outfile"
}

# ----------------------------------------------------------------------------
# L1. empty lock: пустой файл. Никакого recovery не нужно (recovery срабатывает
# только если в файле НЕпустой PID, который мёртв). flock захватывает, OK.
# ----------------------------------------------------------------------------
echo "=== L1: empty lock (zero bytes) — flock захватывает без recovery ==="
LOCK_FILE="$WORK/lock1.lock"
: > "$LOCK_FILE"
extract_lock_section "$WORK/g6.sh"
out="$(bash -c "
set -u
export LOCK_FILE='$LOCK_FILE'
export HERMES_HOME='$WORK'
source '$WORK/g6.sh'
echo RECOVERED=\${RECOVERED:-no}
echo LOCK_CONTENT=\$(cat \$LOCK_FILE 2>/dev/null)
" 2>&1)"
echo "$out"
lock_content="$(echo "$out" | grep -oE 'LOCK_CONTENT=[^\n]*' | head -1 | sed 's/^LOCK_CONTENT=//')"
recovered="$(echo "$out" | grep -oE 'RECOVERED=[^\n]*' | head -1 | sed 's/^RECOVERED=//')"
# Empty lock → нет recovery, но flock захвачен и lock записан с PID:EPOCH
if [[ "$lock_content" =~ ^[0-9]+:[0-9]{4}-[0-9]{2}-[0-9]{2}T ]]; then
    echo "PASS L1: lock записан с PID:EPOCH (empty → fresh)"
    PASS=$((PASS+1))
else
    echo "FAIL L1: lock content='$lock_content' (expected PID:EPOCH)"
    FAIL=$((FAIL+1))
fi

# ----------------------------------------------------------------------------
# L2. fresh lock (другой PID жив): нужно создать «фейкового владельца» —
# дочерний процесс который держит lock. Затем проверить, что новый процесс
# НЕ может захватить lock сразу (flock -n non-zero).
# Это проверяет контракт: stale recovery НЕ срабатывает на живой PID.
# ----------------------------------------------------------------------------
echo "=== L2: fresh lock (PID жив) — НЕ должно быть recovery, flock -n fail ==="
LOCK_FILE="$WORK/lock2.lock"
# Создаём долгоживущий процесс который держит flock
exec 9>"$LOCK_FILE"
flock 9
echo "$$:$(date -u +%Y-%m-%dT%H:%M:%SZ)" > "$LOCK_FILE"
fake_pid=$$
out="$(bash -c "
set -u
export LOCK_FILE='$LOCK_FILE'
export HERMES_HOME='$WORK'
_run_now_triggered=0
source '$WORK/g6.sh' 2>&1
echo NEVER_REACHED
" 2>&1)"
exec 9>&-
echo "$out"
recovery_log="$(echo "$out" | grep -c 'lock stale, recovered')"
never_reached="$(echo "$out" | grep -c 'NEVER_REACHED')"
if [ "${recovery_log:-0}" = "0" ] && [ "${never_reached:-1}" = "0" ]; then
    echo "PASS L2: lock holder жив → recovery НЕ сработал, subshell вышел через exit 0 (skip)"
    PASS=$((PASS+1))
else
    echo "FAIL L2: recovery_log=$recovery_log never_reached=$never_reached"
    FAIL=$((FAIL+1))
fi

# ----------------------------------------------------------------------------
# L3. stale lock (PID мёртв): recovery сработает, flock захватит.
# Имитируем мёртвый PID: пишем в lock высокий несуществующий PID.
# ----------------------------------------------------------------------------
echo "=== L3: stale lock (PID мёртв) — recovery сработает ==="
LOCK_FILE="$WORK/lock3.lock"
: > "$LOCK_FILE"
# Используем PID который гарантированно мёртв (999999 вряд ли занят; но
# safety: проверим kill -0 сам).
_dead_pid=999999
if kill -0 "$_dead_pid" 2>/dev/null; then
    # Если вдруг жив — пропускаем тест (нельзя симулировать stale)
    echo "SKIP L3: $_dead_pid unexpectedly alive"
else
    echo "${_dead_pid}:2025-01-01T00:00:00Z" > "$LOCK_FILE"
    out="$(bash -c "
set -u
export LOCK_FILE='$LOCK_FILE'
export HERMES_HOME='$WORK'
_run_now_triggered=0
source '$WORK/g6.sh' 2>&1
echo EXIT=\$?
echo LOCK_CONTENT=\$(cat \$LOCK_FILE 2>/dev/null)
" 2>&1)"
    echo "$out"
    recovery_log="$(echo "$out" | grep -c 'lock stale')"
    lock_content="$(echo "$out" | grep -oE 'LOCK_CONTENT=[^\n]*' | head -1 | sed 's/^LOCK_CONTENT=//')"
    if [ "${recovery_log:-0}" -ge 1 ] && [[ "$lock_content" =~ ^[0-9]+:[0-9]{4}-[0-9]{2}-[0-9]{2}T ]]; then
        # Lock now has NEW PID (fresh), different from _dead_pid
        if [[ ! "$lock_content" =~ ^${_dead_pid}: ]]; then
            echo "PASS L3: stale PID detected → recovery → fresh lock with new PID"
            PASS=$((PASS+1))
        else
            echo "FAIL L3: lock content still has dead PID"
            FAIL=$((FAIL+1))
        fi
    else
        echo "FAIL L3: recovery_log=$recovery_log content='$lock_content'"
        FAIL=$((FAIL+1))
    fi
fi

# ----------------------------------------------------------------------------
# L4. lock PID == свой PID (legitimate continuation, rare but possible if
#     скрипт рестартует и пишет свой PID в lock сразу). На этот случай
#     НЕ должно быть recovery (lock holder == self, kill -0 = 0, мы НЕ
#     входим в stale ветку).
# ----------------------------------------------------------------------------
echo "=== L4: lock PID == свой PID (own PID) — НЕ recovery ==="
LOCK_FILE="$WORK/lock4.lock"
out="$(bash -c "
set -u
export LOCK_FILE='$LOCK_FILE'
export HERMES_HOME='$WORK'
_run_now_triggered=0
# Записать свой PID в lock
echo \"\$\$:2025-01-01T00:00:00Z\" > '$LOCK_FILE'
source '$WORK/g6.sh' 2>&1
echo EXIT=\$?
" 2>&1)"
echo "$out"
recovery_log="$(echo "$out" | grep -c 'lock stale, recovered')"
# Self PID всегда жив (kill -0 == 0), поэтому recovery НЕ сработает.
if [ "${recovery_log:-0}" = "0" ]; then
    echo "PASS L4: own PID detected → no recovery (правильно)"
    PASS=$((PASS+1))
else
    echo "FAIL L4: recovery triggered for own PID (false positive)"
    FAIL=$((FAIL+1))
fi

# ----------------------------------------------------------------------------
# L5. recovery writes PID:EPOCH (НЕ 0 bytes) — это ADR §2.2.4 ключевая гарантия
# ----------------------------------------------------------------------------
echo "=== L5: recovery writes PID:EPOCH format ==="
LOCK_FILE="$WORK/lock5.lock"
: > "$LOCK_FILE"
echo "999999:2025-01-01T00:00:00Z" > "$LOCK_FILE"
out="$(bash -c "
set -u
export LOCK_FILE='$LOCK_FILE'
export HERMES_HOME='$WORK'
_run_now_triggered=0
source '$WORK/g6.sh'
cat \$LOCK_FILE
" 2>&1)"
# Извлечь только последнюю строку (это вывод cat $LOCK_FILE после recovery)
lock_after="$(printf '%s\n' "$out" | tail -1)"
echo "lock after: $lock_after"
if [[ "$lock_after" =~ ^[0-9]+:[0-9]{4}-[0-9]{2}-[0-9]{2}T[0-9]{2}:[0-9]{2}:[0-9]{2}Z$ ]]; then
    echo "PASS L5: lock after recovery = PID:EPOCH (full ISO 8601)"
    PASS=$((PASS+1))
else
    echo "FAIL L5: lock='$lock_after' (expected PID:YYYY-MM-DDTHH:MM:SSZ)"
    FAIL=$((FAIL+1))
fi

# ----------------------------------------------------------------------------
echo "=================================================="
echo "PASS: $PASS"
echo "FAIL: $FAIL"
echo "=================================================="
[ "$FAIL" -eq 0 ] || exit 1
exit 0
