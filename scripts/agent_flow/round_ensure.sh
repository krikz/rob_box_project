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
# Использование:
#   round_ensure.sh            # печатает z-{e2e}/test-round-N (создаёт если нет)
#   round_ensure.sh --wait N   # ждать до N секунд освобождения flock
#
# Ретро 09.09 (issue #2299): round_ensure.sh — ТОНКАЯ ОБЁРТКА над
# round_formation.sh (общий модуль формирования round-ветки, см. ADR-0075
# с defer-семантикой). Скрипт владеет:
#   - flock + --wait N (как у agent-flow-e2e-process.sh)
#   - env-загрузчиком profile .env
#   - печатью ROUND_BRANCH
# Counter записывается через rf_persist_counter_if_real_round / ghost-маркер
# через rf_ghost_round_log_and_metric — тем же каноном, что и автоматика,
# чтобы счётчик не убегал на ручных прогонах.
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

# --- Делегируем в round_formation.sh (issue #2299) -------------------------
# SOT лежит рядом со скриптом. install.sh раскладывает оба файла в одинаковые
# каталоги профилей (см. EXPECTED[] в install.sh).
_LIB_DIR_HERE="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
# shellcheck source=round_formation.sh
. "${_LIB_DIR_HERE}/round_formation.sh"

# Используем git_push_with_cred_fallback если доступен (защита от регрессии
# credential policy, ретро 23.08 t_b977cb4b), иначе raw git push.
_pf() {
    if declare -F git_push_with_cred_fallback >/dev/null 2>&1; then
        git_push_with_cred_fallback "$@"
    else
        # Fallback: прямой git push (для unit-тестов, где моки gh).
        # round_formation.sh принимает имя функции первым аргументом;
        # если cred_fallback нет, передаём "git" — но это не сработает
        # для push (round_formation ожидает $push_fn <dir> <remote> <refspec>).
        # Поэтому для standalone-сценариев без cred_fallback делая raw push.
        local dir="$1" remote="$2"; shift 2
        git -C "$dir" push "$remote" "$@" 2>&1 || return 1
    fi
}

if ! round_formation _pf; then
    log "❌ round_formation failed"; exit 1
fi

# --- Persist counter — DEFERRED-семантика (issue #2299) ---------------------
# Канон (09.09.2026): ручной прогон использует ту же deferred-семантику, что
# и автоматика. round_ensure.sh НЕ персистит счётчик немедленно — этим владеет
# ТОЛЬКО round_formation → post-tick cleanup (через rf_persist_counter_if_real_
# round). Это устраняет возвратную ghost-дрейф, описанную в issue #2299:
# раньше ручной round на пустом раунде (оператор умер до build) оставлял
# счётчик на +1 при том, что ветка удалялась.
#
# Операторский workflow после --wait exit 0:
#   1. merge фикса в ROUND_BRANCH
#   2. gh workflow run "L-E2E Voice Test" --ref $ROUND_BRANCH
#   3. если прогон упал/не запустился — следующий e2e-process tick увидит
#      round-ветку и либо REUSE (если ещё живая), либо RECREATE.
#
# Если оператор хочет ГАРАНТИРОВАТЬ счётчик для своего прогона (например,
# ручной прогон фактически сделан и больше не повторится), он может вызвать
# `round_formation_persist_now` явно из shell после успешного e2e run. Это
# та же rf_persist_counter_if_real_round — экспортирована ниже как алиас
# для operator convenience.

printf '%s\n' "$ROUND_BRANCH"
log "OK: ручной round = ${ROUND_BRANCH}. Counter НЕ персистится (deferred). Дальше — merge фикса в эту ветку и запуск e2e (workflow L-E2E) НЕ параллельно с e2e-process."