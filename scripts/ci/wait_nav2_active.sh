#!/usr/bin/env bash
# ============================================================================
# wait_nav2_active.sh — nav2 не считается живым, пока не active (issue #5114)
#
# Ретро run 45019034598: L: Deploy and Verify закончился success, пока
# controller_server уже умер `[ros2run]: Illegal instruction`. Оба замера
# статуса показали `health=starting` внутри start_period 90s, а healthcheck
# контейнера (docker/main/scripts/nav2/healthcheck.sh: lifecycle трёх узлов
# == active) воркфлоу не спрашивал.
#
# Этот скрипт — только про nav2 и только на Main Pi. Остальные контейнеры
# он не трогает: их решает check_container_status.sh.
#
#   1. Если check_container_status.sh вернул pending_start с nav2 — ждём
#      конца его start_period (плюс запас на одну пробу healthcheck) и
#      снимаем docker inspect ещё раз. Ждать раньше вердикта нельзя:
#      внутри окна Docker ещё не обязан вынести его.
#   2. Вне окна читаем последний Health.Log[].ExitCode. Ненулевой — провал:
#      healthcheck nav2 возвращает rc=1, когда хоть один из
#      /controller_server /planner_server /bt_navigator не active.
#      Контейнера нет или healthcheck ещё ни разу не отработал вне окна —
#      тоже провал: «не знаем» не равно «жив».
#
# stdout — одна JSON-строка {ok: bool, reason: str}. exit 0 всегда:
# вызывающий валит шаг по .ok, чтобы диагностика успела напечататься.
# docker/sshpass берутся из PATH — тесты подменяют их mock-скриптом.
# ============================================================================
set -euo pipefail

: "${PI_HOST:?PI_HOST must be set}"
: "${PI_USER:?PI_USER must be set}"
: "${PI_PASSWORD:?PI_PASSWORD must be set}"
: "${PI_COMPOSE_DIR:?PI_COMPOSE_DIR must be set}"
: "${PI_SSH_OPTS:?PI_SSH_OPTS must be set}"

# Запас после конца start_period: healthcheck nav2 — interval 15s, timeout 10s
# (docker/main/docker-compose.yaml). Меньше — и замер попадёт между концом окна
# и первой пробой, которая уже имеет право красить контейнер.
PROBE_SLACK_S="${PROBE_SLACK_S:-25}"
# Потолок ожидания. start_period nav2 — 90s; больше этого ждать нечего.
MAX_WAIT_S="${MAX_WAIT_S:-180}"

log() { printf '[wait_nav2_active] %s\n' "$*" >&2; }

# JSON одного контейнера nav2: StartedAt, StartPeriod, последний ExitCode
# healthcheck (-1, если проб ещё не было). Пусто — контейнера нет.
inspect_nav2() {
    sshpass -p "$PI_PASSWORD" ssh $PI_SSH_OPTS "${PI_USER}@${PI_HOST}" \
        "cd ${PI_COMPOSE_DIR} && cid=\$(docker compose ps -aq nav2) && [ -n \"\$cid\" ] && docker inspect --format '{{json .}}' \"\$cid\"" \
        2>/dev/null || true
}

project_nav2() {  # stdin: сырой docker inspect JSON
    jq -c '{
        started_at: (.State.StartedAt // ""),
        start_period_ns: (.Config.Healthcheck.StartPeriod // 0),
        exit_code: (.State.Health.Log[-1].ExitCode // -1),
        health: (.State.Health.Status // "none")
    }' 2>/dev/null || true
}

emit() {  # $1=ok(true|false) $2=reason
    jq -nc --argjson ok "$1" --arg reason "$2" '{ok:$ok, reason:$reason}'
    exit 0
}

# --- main -------------------------------------------------------------------
STATUS_JSON="${STATUS_JSON:-}"
pending="false"
if [ -n "$STATUS_JSON" ] && printf '%s' "$STATUS_JSON" | jq -e '.pending_start == true' >/dev/null 2>&1; then
    if printf '%s' "$STATUS_JSON" | jq -e '.pending[]? | select(.name=="nav2")' >/dev/null 2>&1; then
        pending="true"
    fi
fi

if [ "$pending" = "true" ]; then
    raw="$(inspect_nav2)"
    nav="$(printf '%s' "$raw" | project_nav2)"
    started_at="$(printf '%s' "$nav" | jq -r '.started_at // empty')"
    period_ns="$(printf '%s' "$nav" | jq -r '.start_period_ns // 0')"
    if [ -n "$started_at" ] && [ "$period_ns" -gt 0 ] 2>/dev/null; then
        started_epoch="$(date -d "$started_at" +%s 2>/dev/null || echo 0)"
        period_s=$((period_ns / 1000000000))
        deadline=$((started_epoch + period_s + PROBE_SLACK_S))
        now="$(date +%s)"
        wait_s=$((deadline - now))
        if [ "$wait_s" -gt "$MAX_WAIT_S" ]; then
            wait_s="$MAX_WAIT_S"
        fi
        if [ "$wait_s" -gt 0 ]; then
            log "nav2 still inside start_period; waiting ${wait_s}s for the window to close"
            sleep "$wait_s"
        fi
    fi
fi

raw="$(inspect_nav2)"
if [ -z "$raw" ]; then
    log "nav2 container not found in 'docker compose ps -aq nav2'"
    emit false "nav2 container absent"
fi
nav="$(printf '%s' "$raw" | project_nav2)"
if [ -z "$nav" ]; then
    emit false "nav2 inspect produced no JSON"
fi

started_at="$(printf '%s' "$nav" | jq -r '.started_at // empty')"
period_ns="$(printf '%s' "$nav" | jq -r '.start_period_ns // 0')"
exit_code="$(printf '%s' "$nav" | jq -r '.exit_code')"
health="$(printf '%s' "$nav" | jq -r '.health')"

# Вне окна решаем по коду healthcheck. Внутри окна (повторный замер всё ещё
# не дождался конца) вердикта нет — это не «жив».
if [ -n "$started_at" ] && [ "$period_ns" -gt 0 ] 2>/dev/null; then
    started_epoch="$(date -d "$started_at" +%s 2>/dev/null || echo 0)"
    age_s=$(( $(date +%s) - started_epoch ))
    period_s=$((period_ns / 1000000000))
    if [ "$age_s" -lt "$period_s" ]; then
        log "nav2 still inside start_period after the wait (age=${age_s}s period=${period_s}s)"
        emit false "nav2 still inside start_period after wait"
    fi
fi

log "nav2 health=${health} last healthcheck ExitCode=${exit_code}"
if [ "$exit_code" != "0" ]; then
    emit false "nav2 healthcheck ExitCode=${exit_code} (want 0: controller_server, planner_server, bt_navigator active)"
fi
emit true "nav2 healthcheck ExitCode=0"
