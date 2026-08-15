#!/bin/bash
# ============================================================================
# SOT (source-of-truth): <repo>/scripts/agent_flow/agents_sleep.sh
# Каноническая версия живёт в репо. На хост раскладывается через
# `bash <repo>/scripts/agent_flow/install.sh`, который создаёт hardlink'и в:
#   - ~/.hermes/profiles/agent-flow/scripts/agents_sleep.sh
#   - ~/.hermes/profiles/architect/scripts/agents_sleep.sh
#   - ~/.hermes/profiles/devops/scripts/agents_sleep.sh
#   - ~/.hermes/scripts/agents_sleep.sh
# Правка: редактируем <repo>/scripts/agent_flow/agents_sleep.sh, commit, merge.
# ============================================================================
# agents_sleep.sh — авто-сон агентов по расписанию DeepSeek peak/off-peak.
#
# DeepSeek ввёл peak/off-peak биллинг (с 16.08.2026):
#   PEAK    (100% цены): 01:00–04:00 + 06:00–10:00 UTC
#                      = 04:00–07:00 + 09:00–13:00 MSK
#   OFF-PEAK (50% цены): все остальные часы.
#
# Механизм: MAINTENANCE-файл в origin/develop. Все agent-flow скрипты и
# промпты проверяют его в начале тика: «MAINTENANCE есть = все спят».
#
# Логика (идемпотентная, pure bash, no LLM):
#   PEAK     + MAINTENANCE отсутствует → touch MAINTENANCE + commit + push
#   OFF-PEAK + MAINTENANCE есть (auto)  → git rm MAINTENANCE + commit + push
#   состояние уже правильное            → exit 0, ничего не делает
#
# ВАЖНО: скрипт снимает ТОЛЬКО MAINTENANCE, созданный им самим (маркер
# `auto-sleep:` в содержимом файла). MAINTENANCE, поставленный человеком
# (live-отладка робота, hot-fix сессия), НЕ трогается — человеческое окно
# обслуживания имеет приоритет (правило Шифу: «ничего руками не делай,
# всё по процессу»; автоматика не должна ломать ручной maintenance).
#
# Расписание правится через PR в scripts/agent_flow/agents_sleep_schedule.conf.
#
# Run (cron no_agent, тик 5–15 мин):
#   bash agents_sleep.sh                          # нормальный тик
#   NOW_MSK=06:30 bash agents_sleep.sh            # принудительное время (тесты/демо)
#   DRY_RUN=true  bash agents_sleep.sh            # показать решение, без git-ops
#
# Собственный clone для git-операций (НЕ трогаем общий REPO_DIR — там живут
# worktree'ы других агентов, и он часто сидит на фича-ветке с незакоммиченным).
set -euo pipefail

# --- defaults (env-overridable) ---------------------------------------------
export HOME=/home/builder
HERMES_HOME="${HERMES_HOME:-/home/builder/.hermes}"
GH_REPO="${GH_REPO:-krikz/rob_box_project}"
DEVELOP_BRANCH="${DEVELOP_BRANCH:-develop}"
MAINTENANCE_FILE="${MAINTENANCE_FILE:-MAINTENANCE}"
AGENTS_SLEEP_REPO="${AGENTS_SLEEP_REPO:-$HERMES_HOME/agents-sleep-repo}"
AGENTS_SLEEP_REMOTE="${AGENTS_SLEEP_REMOTE:-https://github.com/${GH_REPO}.git}"
AUTO_MAINTENANCE_MARKER="${AUTO_MAINTENANCE_MARKER:-auto-sleep:}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SCHEDULE_CONF="${AGENTS_SLEEP_CONF:-$SCRIPT_DIR/agents_sleep_schedule.conf}"
PEAK_HOURS="${PEAK_HOURS:-04:00-07:00,09:00-13:00}"   # fallback, если conf нет
DRY_RUN="${DRY_RUN:-false}"
NOW_MSK="${NOW_MSK:-}"
LOCK_FILE="${LOCK_FILE:-/tmp/agents-sleep.lock}"
LOG_PREFIX="${LOG_PREFIX:-[agents-sleep]}"

log() { printf '%s %s %s\n' "$LOG_PREFIX" "$(date -Iseconds)" "$*" >&2; }

# --- load schedule (SOT: agents_sleep_schedule.conf) -------------------------
if [ -f "$SCHEDULE_CONF" ]; then
    # shellcheck source=/dev/null
    . "$SCHEDULE_CONF"
fi

# is_peak <HH:MM> — 0 если время в пике, 1 если нет.
# PEAK_HOURS="04:00-07:00,09:00-13:00" — полуинтервалы [start, end):
# в start ровно MAINTENANCE появляется, в end ровно снимается.
is_peak() {
    local now="$1" range start end
    local -a ranges
    IFS=',' read -ra ranges <<< "$PEAK_HOURS"
    for range in "${ranges[@]}"; do
        start="${range%%-*}"
        end="${range##*-}"
        # zero-padded HH:MM сравнивается лексикографически корректно
        if { [ "$now" \> "$start" ] || [ "$now" = "$start" ]; } && [ "$now" \< "$end" ]; then
            return 0
        fi
    done
    return 1
}

# --- dedicated clone ----------------------------------------------------------
ensure_repo() {
    if [ ! -d "$AGENTS_SLEEP_REPO/.git" ]; then
        log "clone $AGENTS_SLEEP_REMOTE → $AGENTS_SLEEP_REPO"
        git clone --quiet --depth 1 --branch "$DEVELOP_BRANCH" --single-branch \
            "$AGENTS_SLEEP_REMOTE" "$AGENTS_SLEEP_REPO"
    fi
    git -C "$AGENTS_SLEEP_REPO" fetch --quiet origin "$DEVELOP_BRANCH" \
        || { log "fetch origin/$DEVELOP_BRANCH failed (network?) — skip tick"; exit 0; }
    # встаём на origin/develop (свой служебный бранч agents-sleep, develop локально не трогаем)
    git -C "$AGENTS_SLEEP_REPO" checkout -q -B agents-sleep "origin/$DEVELOP_BRANCH" 2>/dev/null \
        || git -C "$AGENTS_SLEEP_REPO" checkout -q --detach "origin/$DEVELOP_BRANCH"
    git -C "$AGENTS_SLEEP_REPO" reset --hard -q "origin/$DEVELOP_BRANCH"
}

maintenance_exists() {
    git -C "$AGENTS_SLEEP_REPO" ls-tree "origin/$DEVELOP_BRANCH" --name-only 2>/dev/null \
        | grep -qx "$MAINTENANCE_FILE"
}

maintenance_is_auto() {
    git -C "$AGENTS_SLEEP_REPO" show "origin/$DEVELOP_BRANCH:$MAINTENANCE_FILE" 2>/dev/null \
        | grep -q "$AUTO_MAINTENANCE_MARKER"
}

# apply_and_push <create|remove> — применяет изменение на свежий origin/develop,
# коммитит от служебного автора и пушит. При reject (конкурентный push в develop)
# повторяет: refetch → re-apply → push, до 3 попыток.
apply_and_push() {
    local action="$1" tries=0
    while :; do
        tries=$((tries+1))
        git -C "$AGENTS_SLEEP_REPO" checkout -q -B agents-sleep "origin/$DEVELOP_BRANCH" 2>/dev/null \
            || git -C "$AGENTS_SLEEP_REPO" checkout -q --detach "origin/$DEVELOP_BRANCH"
        git -C "$AGENTS_SLEEP_REPO" reset --hard -q "origin/$DEVELOP_BRANCH"

        if [ "$action" = "create" ]; then
            printf 'auto-sleep: DeepSeek peak %s MSK (agents_sleep.sh)\n' "$now_msk" \
                > "$AGENTS_SLEEP_REPO/$MAINTENANCE_FILE"
            git -C "$AGENTS_SLEEP_REPO" add "$MAINTENANCE_FILE"
            git -C "$AGENTS_SLEEP_REPO" -c user.name="agents-sleep" \
                -c user.email="agents-sleep@localhost" commit -q \
                -m "maintenance: auto-sleep agents (DeepSeek peak $now_msk MSK)"
        else
            git -C "$AGENTS_SLEEP_REPO" rm -q "$MAINTENANCE_FILE"
            git -C "$AGENTS_SLEEP_REPO" -c user.name="agents-sleep" \
                -c user.email="agents-sleep@localhost" commit -q \
                -m "ci: resume agents (DeepSeek off-peak $now_msk MSK)"
        fi

        if git -C "$AGENTS_SLEEP_REPO" push --quiet origin agents-sleep:"$DEVELOP_BRANCH" 2>/dev/null; then
            return 0
        fi
        [ "$tries" -ge 3 ] && { log "push failed after 3 tries (concurrent develop updates)"; return 1; }
        log "push rejected — refetch, re-apply, retry ($tries/3)"
        sleep 3
        git -C "$AGENTS_SLEEP_REPO" fetch --quiet origin "$DEVELOP_BRANCH" || true
    done
}

# --- main ----------------------------------------------------------------------
main() {
    # flock: параллельные тики не нужны
    exec 9>"$LOCK_FILE" || { log "cannot open lock $LOCK_FILE"; return 1; }
    if ! flock -n 9; then
        log "another instance holds $LOCK_FILE — skip"; return 0
    fi

    # текущее время MSK
    if [ -n "$NOW_MSK" ]; then
        now_msk="$NOW_MSK"
    else
        now_msk="$(TZ='Europe/Moscow' date +%H:%M)"
    fi

    # репозиторий
    ensure_repo

    # решение
    if is_peak "$now_msk"; then
        if maintenance_exists; then
            log "PEAK $now_msk MSK — MAINTENANCE уже есть, спим (идемпотентно)"
            return 0
        fi
        log "PEAK $now_msk MSK — MAINTENANCE нет → создаю (все спят)"
        if [ "$DRY_RUN" = "true" ]; then
            echo "DRY-RUN: would create MAINTENANCE (peak $now_msk MSK)"
            return 0
        fi
        if apply_and_push create; then
            echo "agents_sleep: MAINTENANCE создан (DeepSeek peak $now_msk MSK)"
        else
            log "ERROR: failed to create MAINTENANCE"; return 1
        fi
    else
        if ! maintenance_exists; then
            log "OFF-PEAK $now_msk MSK — MAINTENANCE нет, работаем (идемпотентно)"
            return 0
        fi
        if ! maintenance_is_auto; then
            log "OFF-PEAK $now_msk MSK — MAINTENANCE ручной (без маркера «$AUTO_MAINTENANCE_MARKER») — НЕ трогаю (человеческое окно)"
            return 0
        fi
        log "OFF-PEAK $now_msk MSK — MAINTENANCE auto → снимаю (проснулись)"
        if [ "$DRY_RUN" = "true" ]; then
            echo "DRY-RUN: would remove MAINTENANCE (off-peak $now_msk MSK)"
            return 0
        fi
        if apply_and_push remove; then
            echo "agents_sleep: MAINTENANCE снят (DeepSeek off-peak $now_msk MSK)"
        else
            log "ERROR: failed to remove MAINTENANCE"; return 1
        fi
    fi
}

# Run main only when executed directly (не при source — для тестов).
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
    main "$@"
fi
