#!/bin/bash
# ============================================================================
# round_ensure.sh — ручной валидационный e2e-раунд для devops (ретро 11.08 t_26a6d362)
#
# ПРОБЛЕМА (11.08, round-49/50 race): ручной раунд devops (t_0a5d65af, round-50)
# шёл ПАРАЛЛЕЛЬНО с автоматической ротацией e2e-process (round-49 для #1077).
# Cleanup на 249 удалил /tmp/e2e_v2_* активного прогона → paplay open(): No
# such file → ложный FAIL-вердикт. Причина: ручные раунды обходили flock
# /tmp/agent-flow-e2e-process.lock и не знали про активный round.
#
# ПРАВИЛО (процессное):
#   Ручные валидационные раунды — ТОЛЬКО через этот скрипт (или ROUND_ONLY=1
#   режим agent-flow-e2e-process.sh). Скрипт берёт ТОТ ЖЕ flock, что и
#   автоматическая ротация: если e2e-process активен — выход с ошибкой,
#   повторяй позже (дождись idle ротации). Никогда не создавай round вручную
#   мимо этого скрипта.
#
# Usage:
#   round_ensure.sh            # печатает z-{e2e}/test-round-N (создаёт если нет)
#   round_ensure.sh --wait N   # ждать до N секунд освобождения flock
#
# Env (как у agent-flow-e2e-process.sh): GH_REPO, REPO_DIR, KANBAN_BOARD,
# HERMES_HOME, LOCK_FILE (default /tmp/agent-flow-e2e-process.lock).
# ============================================================================
set -euo pipefail

HERMES_HOME="${HERMES_HOME:-/home/builder/.hermes}"
export HOME=/home/builder
GH_REPO="${GH_REPO:-}"
REPO_DIR="${REPO_DIR:-}"
KANBAN_BOARD="${KANBAN_BOARD:-robbox}"
LOCK_FILE="${LOCK_FILE:-/tmp/agent-flow-e2e-process.lock}"
LOG_PREFIX="[round_ensure]"
DRY_RUN="${DRY_RUN:-false}"

# source profile .env если есть (GH_REPO, REPO_DIR, ...)
PROFILE_ENV="${HERMES_HOME}/profiles/agent-flow/.env"
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

log() { printf '%s %s %s\n' "$LOG_PREFIX" "$(date -Iseconds)" "$*" >&2; }

[ -n "${GH_REPO:-}" ] || { log "GH_REPO must be set"; exit 1; }
[ -d "${REPO_DIR:-}" ] || { log "REPO_DIR must exist: ${REPO_DIR:-}"; exit 1; }

# --- flock: тот же lock, что у e2e-process ----------------------------------
exec 9>"$LOCK_FILE" || { log "cannot open lock $LOCK_FILE"; exit 1; }
if ! flock -n 9; then
    wait_s="${1:-}"
    if [ "$wait_s" = "--wait" ] && [ -n "${2:-}" ]; then
        t="${2:-0}"
        log "e2e-process активен (flock занят) — жду до ${t}s..."
        waited=0
        while [ "$waited" -lt "$t" ]; do
            if flock -n 9 2>/dev/null; then
                log "flock получен через ${waited}s"
                break
            fi
            sleep 10
            waited=$((waited + 10))
        done
        if ! flock -n 9 2>/dev/null; then
            log "❌ e2e-process всё ещё активен после ${t}s — НЕ создаю round. Дождись idle ротации и повтори."
            exit 1
        fi
    else
        log "❌ e2e-process активен (flock занят) — ручной round НЕ создаю (ретро 11.08: параллельный round жжёт артефакты). Повтори позже или round_ensure.sh --wait N"
        exit 1
    fi
fi

# --- round number: max(N) на remote + 1 (персистентный счётчик) -------------
# Ретро 12.08 (t_bff6eccf): cleanup удаляет stale round-ветки → max по remote
# сбрасывается на 1. Счётчик храним в файле состояния — нумерация переживает
# cleanup (тот же файл, что у agent-flow-e2e-process.sh).
TEST_ROUND_PREFIX='z-{e2e}/test-round-'
ROUND_COUNTER_FILE="${ROUND_COUNTER_FILE:-${HERMES_HOME}/state/agent-flow-e2e-round-counter}"
list="$(git -C "$REPO_DIR" ls-remote --heads origin "${TEST_ROUND_PREFIX}*" 2>/dev/null \
    | awk '{print $2}' | sed "s#refs/heads/${TEST_ROUND_PREFIX}##" || true)"
if [ -z "$list" ]; then
    max_n=0
else
    max_n="$(printf '%s\n' "$list" | sort -n | tail -n1)"
fi
counter_n=0
if [ -f "$ROUND_COUNTER_FILE" ]; then
    counter_n="$(tr -dc '0-9' < "$ROUND_COUNTER_FILE" 2>/dev/null || echo 0)"
    counter_n="${counter_n:-0}"
fi
if [ "$counter_n" -gt "$max_n" ]; then
    log "round counter: file=${counter_n} > remote-max=${max_n} (cleanup сбросил ветки?) — берём max из файла"
    max_n="$counter_n"
fi
n=$((max_n + 1))
ROUND_BRANCH="${TEST_ROUND_PREFIX}${n}"
log "round number: max=${max_n} -> next=${n}"

if ! git -C "$REPO_DIR" ls-remote --heads origin "$ROUND_BRANCH" 2>/dev/null | grep -q .; then
    if [ "$DRY_RUN" = "true" ]; then
        log "DRY-RUN would: create ${ROUND_BRANCH} from origin/develop"
    else
        log "creating ${ROUND_BRANCH} from origin/develop (fresh fetch)"
        git -C "$REPO_DIR" fetch origin develop 2>&1 | sed 's/^/  /' || true
        if ! git -C "$REPO_DIR" push origin "origin/develop:refs/heads/${ROUND_BRANCH}" 2>&1 | sed 's/^/  /'; then
            log "failed to create ${ROUND_BRANCH}"; exit 1
        fi
    fi
else
    # Ретро 12.08 t_d3aeaa9b: НЕ переиспользуем stale round (база устарела).
    # round-59 был создан из develop ДО фиксов валидатора #1143 и ротации
    # #1141 — reuse вернул бы e2e на регрессе. Проверка: round-ветка должна
    # содержать актуальный origin/develop; если нет — удаляем и создаём заново.
    log "checking ${ROUND_BRANCH} base freshness (must contain origin/develop)"
    if [ "$DRY_RUN" = "true" ]; then
        log "DRY-RUN would: check ancestry origin/develop..${ROUND_BRANCH}"
    else
        git -C "$REPO_DIR" fetch origin develop 2>&1 | sed 's/^/  /' || true
        if git -C "$REPO_DIR" merge-base --is-ancestor "origin/develop" "origin/${ROUND_BRANCH}" 2>/dev/null; then
            log "reusing ${ROUND_BRANCH} (база актуальна: содержит origin/develop)"
        else
            log "🛑 ${ROUND_BRANCH} база УСТАРЕЛА (не содержит origin/develop) — удаляю и создам заново (ретро 12.08 t_d3aeaa9b)"
            git -C "$REPO_DIR" push origin --delete "$ROUND_BRANCH" 2>&1 | sed 's/^/  /' || true
            git -C "$REPO_DIR" fetch origin develop 2>&1 | sed 's/^/  /' || true
            if ! git -C "$REPO_DIR" push origin "origin/develop:refs/heads/${ROUND_BRANCH}" 2>&1 | sed 's/^/  /'; then
                log "failed to recreate ${ROUND_BRANCH}"; exit 1
            fi
            log "recreated ${ROUND_BRANCH} from fresh origin/develop"
        fi
    fi
fi

# Сохраняем счётчик (только после успешного создания/reuse).
if [ "$n" -gt "$counter_n" ]; then
    printf '%s\n' "$n" > "$ROUND_COUNTER_FILE" 2>/dev/null \
        && log "round counter saved: ${n} -> ${ROUND_COUNTER_FILE}" \
        || log "WARNING: cannot write round counter ${ROUND_COUNTER_FILE}"
fi

printf '%s\n' "$ROUND_BRANCH"
log "OK: ручной round = ${ROUND_BRANCH}. Дальше — merge фикса в эту ветку и запуск e2e (workflow L-E2E) НЕ параллельно с e2e-process."
