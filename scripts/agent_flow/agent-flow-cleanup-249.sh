#!/bin/bash
# ============================================================================
# agent-flow-cleanup-249.sh — безопасный cleanup /tmp на build-хосте 10.1.1.249
# (ретро 11.08 t_26a6d362: cleanup убил артефакты активного e2e-прогона round-49)
#
# ПРОБЛЕМА (11.08): ручной cleanup devops (t_0a5d65af) удалял /tmp/e2e_v2_*
# ВО ВРЕМЯ активного e2e-прогона (round-49, run 31544057593) → paplay open():
# No such file → ложный FAIL. Cleanup не проверял, чьи это артефакты.
#
# ПРАВИЛА:
#   1. НЕ удалять файлы моложе CLEANUP_MIN_AGE_MIN (default 30 мин) — активный
#      e2e-прогон пишет свежие /tmp/e2e_v2_* и /tmp/e2e_atomic_out.log.
#   2. Если e2e-process активен (flock /tmp/agent-flow-e2e-process.lock на
#      хосте ротации) — cleanup пропускается целиком.
#   3. Удаляются только: yandex_key_*, build_*.log, dialog_e2e_*.wav,
#      e2e_v2_*, voice_e2e_*.log старше MIN_AGE (root-owned мусор прошлых
#      ранов, который копится на 249 и мешает scp/paplay).
#   4. e2e_voice_test.sh (актуальный харнесс) НЕ удаляется никогда.
#
# Usage:
#   agent-flow-cleanup-249.sh [--min-age 30] [--dry-run]
# Env: BUILD_HOST=10.1.1.249, BUILD_USER=ros2, SSHPASS (или -p open),
#      LOCK_FILE=/tmp/agent-flow-e2e-process.lock (локальный flock e2e-process)
# ============================================================================
set -euo pipefail

BUILD_HOST="${BUILD_HOST:-10.1.1.249}"
BUILD_USER="${BUILD_USER:-ros2}"
SSHPASS_VAL="${SSHPASS:-open}"
LOCK_FILE="${LOCK_FILE:-/tmp/agent-flow-e2e-process.lock}"
MIN_AGE_MIN="${MIN_AGE_MIN:-30}"
DRY_RUN=0
LOG_PREFIX="[cleanup-249]"

log() { printf '%s %s %s\n' "$LOG_PREFIX" "$(date -Iseconds)" "$*" >&2; }

while [ $# -gt 0 ]; do
    case "$1" in
        --min-age) MIN_AGE_MIN="$2"; shift 2 ;;
        --dry-run) DRY_RUN=1; shift ;;
        *) log "unknown arg: $1"; exit 2 ;;
    esac
done

# --- 1. e2e-process активен? (локальный flock на хосте ротации) -------------
if [ -f "$LOCK_FILE" ]; then
    if exec 9>"$LOCK_FILE" && flock -n 9 2>/dev/null; then
        : # lock свободен — можно чистить
    else
        log "🛑 e2e-process активен (flock $LOCK_FILE занят) — cleanup SKIP (ретро 11.08: не трогаем артефакты активного round)"
        exit 0
    fi
else
    log "lock $LOCK_FILE не найден — считаем ротацию неактивной (cleanup можно)"
fi

# --- 2. Проверка: свежий e2e-прогон на 249? (mtime guard на самом хосте) ----
# Даже если локальный flock свободен, e2e-прогон МОЖЕТ идти с другого хоста
# (workflow L-E2E шлёт харнесс на 249 напрямую с GitHub runner). Двойная
# защита: не удалять ничего моложе MIN_AGE_MIN.
log "min-age guard: ${MIN_AGE_MIN} мин (файлы моложе не трогаем)"

# --- 3. Собираем список мусора на 249 ---------------------------------------
# Паттерны: старые ключи, build-логи, wav-записи прошлых ранов, e2e_v2_*.
# Найти файлы старше MIN_AGE_MIN и НЕ трогать e2e_voice_test.sh.
find_cmd="find /tmp -maxdepth 1 -type f \( \
    -name 'yandex_key_*' -o -name 'build_*.log' -o -name 'dialog_e2e_*.wav' \
    -o -name 'e2e_v2_*' -o -name 'voice_e2e_*.log' -o -name 'e2e_atomic_out.log' \
\) -mmin +${MIN_AGE_MIN} ! -name 'e2e_voice_test.sh' -print 2>/dev/null"

if [ "$DRY_RUN" = "1" ]; then
    log "DRY-RUN: список файлов для удаления на ${BUILD_HOST} (старше ${MIN_AGE_MIN} мин):"
    sshpass -p "$SSHPASS_VAL" ssh -o StrictHostKeyChecking=no -o ConnectTimeout=10 \
        "${BUILD_USER}@${BUILD_HOST}" "$find_cmd" 2>/dev/null | sed 's/^/  /'
    log "DRY-RUN: ничего не удалено"
    exit 0
fi

# Удаляем по одному (rm -f не падает на отсутствующих; -- не даёт съесть флаги).
sshpass -p "$SSHPASS_VAL" ssh -o StrictHostKeyChecking=no -o ConnectTimeout=10 \
    "${BUILD_USER}@${BUILD_HOST}" \
    "list=\$($find_cmd); if [ -n \"\$list\" ]; then printf '%s\n' \"\$list\" | xargs -r rm -f -- && echo \"CLEANUP_OK removed: \$(printf '%s\n' \"\$list\" | wc -l) files\"; else echo 'CLEANUP_OK nothing to remove'; fi" 2>&1 | sed 's/^/  /'

log "cleanup done (${BUILD_HOST}, min-age ${MIN_AGE_MIN} мин)"
