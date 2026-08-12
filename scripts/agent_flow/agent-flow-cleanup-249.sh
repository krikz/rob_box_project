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
#      LOCK_FILE=/tmp/agent-flow-e2e-process.lock (локальный flock e2e-process),
#      GH_REPO (owner/repo; для round-branch cleanup), ROUND_STALE_HOURS (default 48)
# ============================================================================
set -euo pipefail

# gh auth на этом хосте: HOME=/home/builder (иначе gh ищет конфиг в
# $HOME/.config/gh профильной оболочки и падает «not logged in»).
export HOME=/home/builder

# source profile .env (GH_REPO и пр.) — как round_ensure.sh
PROFILE_ENV="/home/builder/.hermes/profiles/agent-flow/.env"
if [ -f "$PROFILE_ENV" ]; then
    while IFS='=' read -r key val; do
        case "$key" in ''|'#'*) continue ;; esac
        val="${val%\"}"; val="${val#\"}"
        val="${val%\'}"; val="${val#\'}"
        if [ -z "${!key:-}" ]; then
            export "$key=$val"
        fi
    done < "$PROFILE_ENV"
fi

BUILD_HOST="${BUILD_HOST:-10.1.1.249}"
BUILD_USER="${BUILD_USER:-ros2}"
SSHPASS_VAL="${SSHPASS:-open}"
LOCK_FILE="${LOCK_FILE:-/tmp/agent-flow-e2e-process.lock}"
MIN_AGE_MIN="${MIN_AGE_MIN:-30}"
ROUND_STALE_HOURS="${ROUND_STALE_HOURS:-48}"
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

# --- 4. stale round-branch cleanup на remote (ретро 12.08 t_d3aeaa9b) ------
# ПРАВИЛО: round-ветка без e2e-активности > ROUND_STALE_HOURS (48ч) → delete.
# e2e-активность = последний коммит в round-ветке: e2e-process мержит
# agent-ветку в round и пушит ПЕРЕД запуском L-E2E, поэтому свежий коммит =
# свежий прогон. Старые round-ветки (напр. round-59, создан из develop ДО
# фиксов валидатора #1143 и ротации #1141) несут устаревшую базу: при reuse
# round_ensure берёт max-N со старой базой → e2e гоняется на регрессе.
# Guard: flock e2e-process уже проверен в секции 1 (активный round не тронем).
if [ -n "${GH_REPO:-}" ] && command -v gh >/dev/null 2>&1 && gh auth status >/dev/null 2>&1; then
    log "round-cleanup: ищу stale round-ветки (>${ROUND_STALE_HOURS}ч без e2e-активности) на ${GH_REPO}"
    now_epoch="$(date +%s)"
    while read -r round_branch round_sha; do
        [ -z "$round_branch" ] && continue
        last_date="$(gh api "repos/${GH_REPO}/commits/${round_sha}" --jq '.commit.committer.date' 2>/dev/null || echo '')"
        if [ -z "$last_date" ]; then
            log "  WARN: не удалось получить дату для ${round_branch} (${round_sha}) — пропуск"
            continue
        fi
        last_epoch="$(date -d "$last_date" +%s 2>/dev/null || echo 0)"
        age_h=$(( (now_epoch - last_epoch) / 3600 ))
        if [ "$age_h" -gt "$ROUND_STALE_HOURS" ]; then
            if [ "$DRY_RUN" = "1" ]; then
                log "  DRY-RUN: удалил бы ${round_branch} (последний коммит ${age_h}ч назад)"
            else
                # URL-encode { } в имени ветки (GitHub API требует %7B/%7D)
                enc="${round_branch//\{/%7B}"; enc="${enc//\}/%7D}"
                if gh api -X DELETE "repos/${GH_REPO}/git/refs/heads/${enc}" >/dev/null 2>&1; then
                    log "  DELETED stale round ${round_branch} (последний коммит ${age_h}ч назад)"
                else
                    log "  WARN: не удалось удалить ${round_branch}"
                fi
            fi
        else
            log "  keep ${round_branch} (последний коммит ${age_h}ч назад)"
        fi
    done < <(gh api "repos/${GH_REPO}/branches?per_page=100" \
        --jq '.[] | select(.name | startswith("z-{e2e}/test-round-")) | "\(.name)\t\(.commit.sha)"' 2>/dev/null || true)
else
    log "round-cleanup: GH_REPO/gh недоступны — пропуск (только /tmp cleanup)"
fi

log "cleanup done (${BUILD_HOST}, min-age ${MIN_AGE_MIN} мин)"
