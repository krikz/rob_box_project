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
#   3. Лечение #5039 (ADR-0148, детерминированно) — РУЧНОЙ рубильник, по
#      умолчанию выключен (решение товарища Шифу 23.11, #5149). По умолчанию
#      гейт не перезапускает контейнер: ложный зелёный после самолечения
#      скрывает причину, а класс #5039 так и не воспроизведён — рестарт
#      маскировал диагноз. NAV2_RESTART_ON_FAIL=1 (вход L: Deploy and Verify)
#      включает ровно один `docker restart nav2`, и только если провал
#      подтверждён второй свежей пробой, zenoh-router уже healthy, а замер
#      вне окна. Таймаут (ExitCode=-1) рестарт не лечит. Второго перезапуска
#      не бывает, роутер не перезапускается никогда. После рестарта вердикт
#      — только по пробе, начатой не раньше StartedAt нового запуска: старая
#      запись Health.Log не принимается.
#
# stdout — одна JSON-строка {ok: bool, reason: str}. exit 0 всегда:
# вызывающий валит шаг по .ok, чтобы диагностика успела напечататься.
# docker/sshpass берутся из PATH — тесты подменяют их mock-скриптом.
# ============================================================================
set -euo pipefail

# Запас после конца start_period: healthcheck nav2 — interval 15s, timeout 10s
# (docker/main/docker-compose.yaml). Меньше — и замер попадёт между концом окна
# и первой пробой, которая уже имеет право красить контейнер.
PROBE_SLACK_S="${PROBE_SLACK_S:-25}"
# Потолок ожидания. start_period nav2 — 90s; больше этого ждать нечего.
MAX_WAIT_S="${MAX_WAIT_S:-180}"
# Лечение #5039 — ручной рубильник (решение Шифу 23.11, #5149).
# По умолчанию 0: гейт честно краснеет с причиной, контейнер не трогает.
# 1 — оператор/дежурный явно просит один рестарт после подтверждённого провала.
# Самолечение по умолчанию давало ложный зелёный и прятало диагноз #5039.
NAV2_RESTART_ON_FAIL="${NAV2_RESTART_ON_FAIL:-0}"
# Пауза перед повторным замером, когда одна красная проба ещё не подтверждена
# или после рестарта свежей пробы ещё нет. healthcheck nav2 — interval 15s.
CONFIRM_WAIT_S="${CONFIRM_WAIT_S:-15}"

log() { printf '[wait_nav2_active] %s\n' "$*" >&2; }

# JSON одного контейнера nav2: StartedAt, StartPeriod, последний ExitCode
# healthcheck (-1, если проб ещё не было). Пусто — контейнера нет.
inspect_nav2() {
    # shellcheck disable=SC2086  # PI_SSH_OPTS — список опций ssh, сплит нужен
    sshpass -p "$PI_PASSWORD" ssh $PI_SSH_OPTS "${PI_USER}@${PI_HOST}" \
        "cd ${PI_COMPOSE_DIR} && cid=\$(docker compose ps -aq nav2) && [ -n \"\$cid\" ] && docker inspect --format '{{json .}}' \"\$cid\"" \
        2>/dev/null || true
}

# Состояние healthcheck zenoh-router на том же Pi: healthy / starting /
# unhealthy / пусто (контейнера нет или healthcheck не описан).
router_health() {
    # shellcheck disable=SC2086  # PI_SSH_OPTS — список опций ssh, сплит нужен
    sshpass -p "$PI_PASSWORD" ssh $PI_SSH_OPTS "${PI_USER}@${PI_HOST}" \
        "cd ${PI_COMPOSE_DIR} && rid=\$(docker compose ps -aq zenoh-router) && [ -n \"\$rid\" ] && docker inspect --format '{{if .State.Health}}{{.State.Health.Status}}{{end}}' \"\$rid\"" \
        2>/dev/null || true
}

# Один перезапуск nav2 на Pi. Роутер не трогаем.
restart_nav2() {
    log "docker restart nav2 (${PI_USER}@${PI_HOST})"
    # shellcheck disable=SC2086  # PI_SSH_OPTS — список опций ssh, сплит нужен
    sshpass -p "$PI_PASSWORD" ssh $PI_SSH_OPTS "${PI_USER}@${PI_HOST}" \
        "cd ${PI_COMPOSE_DIR} && docker restart nav2" >/dev/null 2>&1
}

project_nav2() {  # stdin: сырой docker inspect JSON
    jq -c '{
        started_at: (.State.StartedAt // ""),
        start_period_ns: (.Config.Healthcheck.StartPeriod // 0),
        exit_code: (.State.Health.Log[-1].ExitCode // -1),
        probe_start: (.State.Health.Log[-1].Start // ""),
        health: (.State.Health.Status // "none")
    }' 2>/dev/null || true
}

# true, если последняя проба началась не раньше StartedAt текущего запуска.
# Пустой Start или пустой StartedAt — проба не свежая: старую запись Health.Log
# (особенно при StartPeriod=0) нельзя принимать за вердикт нового контейнера.
probe_is_fresh() {  # $1=probe_start $2=started_at
    local probe_start="$1" started_at="$2"
    [ -n "$probe_start" ] && [ -n "$started_at" ] || return 1
    local probe_epoch started_epoch
    probe_epoch="$(date -d "$probe_start" +%s 2>/dev/null || echo 0)"
    started_epoch="$(date -d "$started_at" +%s 2>/dev/null || echo 0)"
    [ "$probe_epoch" -gt 0 ] && [ "$started_epoch" -gt 0 ] \
        && [ "$probe_epoch" -ge "$started_epoch" ]
}

# Чистое решение по одному замеру: без ssh и без робота. Тесты вызывают её
# напрямую (source этого файла) — решение проверяется таблицей, а не догадкой.
#   $1 exit_code последнего healthcheck (-1 — проб ещё не было)
#   $2 inside_window (true|false): замер внутри start_period
#   $3 router (healthy|…|unknown): состояние zenoh-router
#   $4 restarted (true|false): перезапуск уже был в этом прогоне
#   $5 confirmed (true|false): провал уже подтверждён второй свежей пробой
# stdout: ok | wait | restart | fail | confirm
nav2_action() {
    local exit_code="$1" inside_window="$2" router="$3" restarted="$4" confirmed="${5:-false}"
    if [ "$exit_code" = "0" ]; then
        printf 'ok\n'
        return 0
    fi
    if [ "$inside_window" = "true" ]; then
        printf 'wait\n'
        return 0
    fi
    # -1 = healthcheck ещё не отработал вне окна: перезапуск это не лечит,
    # причина другая (healthcheck не описан/не запускается) — честный fail.
    # Одна красная проба — ещё не диагноз: сначала confirm, рестарт только
    # после второй. Рубильник по умолчанию выключен — тогда сразу fail.
    if [ "$exit_code" != "-1" ] && [ "$restarted" != "true" ] \
        && [ "$router" = "healthy" ] && [ "$NAV2_RESTART_ON_FAIL" = "1" ]; then
        if [ "$confirmed" = "true" ]; then
            printf 'restart\n'
        else
            printf 'confirm\n'
        fi
        return 0
    fi
    printf 'fail\n'
}

emit() {  # $1=ok(true|false) $2=reason
    jq -nc --argjson ok "$1" --arg reason "$2" '{ok:$ok, reason:$reason}'
    exit 0
}

main() {
    : "${PI_HOST:?PI_HOST must be set}"
    : "${PI_USER:?PI_USER must be set}"
    : "${PI_PASSWORD:?PI_PASSWORD must be set}"
    : "${PI_COMPOSE_DIR:?PI_COMPOSE_DIR must be set}"
    : "${PI_SSH_OPTS:?PI_SSH_OPTS must be set}"

    # Общий бюджет ожидания на весь прогон. Перезапуск выдаёт свежее окно
    # (start_period начинается заново) и сбрасывает бюджет один раз — иначе
    # ожидание старого окна съедало бы весь MAX_WAIT_S и «лечение» стало бы
    # ложным провалом по таймауту.
    local deadline
    deadline=$(( $(date +%s) + MAX_WAIT_S ))

    local status_json="${STATUS_JSON:-}"
    local pending="false"
    if [ -n "$status_json" ] && printf '%s' "$status_json" | jq -e '.pending_start == true' >/dev/null 2>&1; then
        if printf '%s' "$status_json" | jq -e '.pending[]? | select(.name=="nav2")' >/dev/null 2>&1; then
            pending="true"
        fi
    fi

    local raw nav started_at period_ns exit_code health probe_start restarted router action
    local confirmed fresh
    restarted="false"
    confirmed="false"

    raw="$(inspect_nav2)"
    if [ "$pending" = "true" ]; then
        nav="$(printf '%s' "$raw" | project_nav2)"
        started_at="$(printf '%s' "$nav" | jq -r '.started_at // empty')"
        period_ns="$(printf '%s' "$nav" | jq -r '.start_period_ns // 0')"
        if [ -n "$started_at" ] && [ "$period_ns" -gt 0 ] 2>/dev/null; then
            local started_epoch period_s wait_s
            started_epoch="$(date -d "$started_at" +%s 2>/dev/null || echo 0)"
            period_s=$((period_ns / 1000000000))
            wait_s=$((started_epoch + period_s + PROBE_SLACK_S - $(date +%s)))
            if [ "$wait_s" -gt "$MAX_WAIT_S" ]; then
                wait_s="$MAX_WAIT_S"
            fi
            if [ "$wait_s" -gt 0 ]; then
                log "nav2 still inside start_period; waiting ${wait_s}s for the window to close"
                sleep "$wait_s"
            fi
        fi
        raw="$(inspect_nav2)"
    fi

    while :; do
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
        probe_start="$(printf '%s' "$nav" | jq -r '.probe_start // empty')"
        health="$(printf '%s' "$nav" | jq -r '.health')"

        # После рестарта Health.Log[-1] может быть пробой прошлого запуска
        # (StartPeriod=0/нет — Docker её не прячет). Вердикт только по пробе,
        # начатой не раньше StartedAt текущего контейнера.
        fresh="true"
        if [ "$restarted" = "true" ]; then
            if probe_is_fresh "$probe_start" "$started_at"; then
                fresh="true"
            else
                fresh="false"
            fi
        fi

        # Внутри окна вердикта нет: ждём его конца в пределах общего бюджета.
        if [ -n "$started_at" ] && [ "$period_ns" -gt 0 ] 2>/dev/null; then
            local started_epoch age_s period_s wait_s budget
            started_epoch="$(date -d "$started_at" +%s 2>/dev/null || echo 0)"
            age_s=$(( $(date +%s) - started_epoch ))
            period_s=$((period_ns / 1000000000))
            if [ "$age_s" -lt "$period_s" ]; then
                wait_s=$(( period_s - age_s + PROBE_SLACK_S ))
                budget=$(( deadline - $(date +%s) ))
                if [ "$wait_s" -gt "$budget" ]; then
                    wait_s="$budget"
                fi
                if [ "$wait_s" -le 0 ]; then
                    log "nav2 still inside start_period after the wait (age=${age_s}s period=${period_s}s)"
                    emit false "nav2 still inside start_period after wait"
                fi
                log "nav2 inside start_period (age=${age_s}s period=${period_s}s); waiting ${wait_s}s"
                sleep "$wait_s"
                raw="$(inspect_nav2)"
                continue
            fi
        fi

        # Свежей пробы текущего запуска ещё нет — ждём в бюджете, старую
        # запись за вердикт не принимаем.
        if [ "$fresh" != "true" ]; then
            budget=$(( deadline - $(date +%s) ))
            # Пауза — interval healthcheck, но не дольше капа и не дольше
            # бюджета. Ноль не крутим: иначе цикл не двигает часы и вердикт
            # не выносится. Если и минимальная пауза уже не влезает — свежей
            # пробы в этом бюджете не будет, честный fail.
            wait_s="$CONFIRM_WAIT_S"
            cap="${CONFIRM_PROBE_CAP_S:-15}"
            if [ "$wait_s" -gt "$cap" ]; then
                wait_s="$cap"
            fi
            if [ "$wait_s" -lt 1 ]; then
                wait_s=1
            fi
            if [ "$wait_s" -gt "$budget" ]; then
                log "no fresh healthcheck probe after restart (budget exhausted)"
                emit false "no fresh healthcheck probe after restart (last ExitCode=${exit_code} started before ${started_at})"
            fi
            log "probe Start=${probe_start:-<none>} is older than StartedAt=${started_at}; waiting ${wait_s}s for a fresh probe"
            sleep "$wait_s"
            raw="$(inspect_nav2)"
            continue
        fi

        # Вне окна решаем по коду healthcheck. Состояние роутера спрашиваем
        # только когда healthcheck упал — это единственный случай, где оно
        # влияет на решение.
        router="unknown"
        if [ "$exit_code" != "0" ]; then
            router="$(router_health)"
            [ -n "$router" ] || router="unknown"
        fi

        action="$(nav2_action "$exit_code" "false" "$router" "$restarted" "$confirmed")"
        log "nav2 health=${health} last healthcheck ExitCode=${exit_code} zenoh-router=${router} (action=${action})"
        case "$action" in
            ok)
                emit true "nav2 healthcheck ExitCode=0"
                ;;
            confirm)
                # Одна красная проба — не повод рестартовать healthy контейнер
                # (таймаут пробы, одиночный сбой). Ждём interval и мерим снова.
                confirmed="true"
                budget=$(( deadline - $(date +%s) ))
                if [ "$budget" -le 0 ]; then
                    log "nav2 healthcheck ExitCode=${exit_code} not confirmed (budget exhausted)"
                    emit false "nav2 healthcheck ExitCode=${exit_code} (want 0: controller_server, planner_server, bt_navigator active; zenoh-router=${router}; failure not confirmed)"
                fi
                wait_s="$CONFIRM_WAIT_S"
                if [ "$wait_s" -gt "$budget" ]; then
                    wait_s="$budget"
                fi
                log "nav2 healthcheck ExitCode=${exit_code} — ждём ${wait_s}s и мерим снова, рестарта ещё нет"
                sleep "$wait_s"
                raw="$(inspect_nav2)"
                ;;
            restart)
                # Провал подтверждён второй пробой, роутер healthy, рубильник
                # включён: ровно один рестарт (#5039). Второго не бывает.
                log "nav2 healthcheck ExitCode=${exit_code} — однократный restart nav2"
                if ! restart_nav2; then
                    emit false "nav2 healthcheck ExitCode=${exit_code}; one-shot 'docker restart nav2' failed"
                fi
                restarted="true"
                confirmed="false"
                # Свежее окно start_period начинается заново. Бюджет продлеваем
                # один раз, но не выше RESTART_WAIT_S: иначе «лечение» съест
                # весь MAX_WAIT_S, а нулевой потолок (тест) не даст ждать
                # пробу, которой нет. Второго перезапуска всё равно не будет.
                restart_budget="${RESTART_WAIT_S:-$MAX_WAIT_S}"
                if [ "$restart_budget" -gt "$MAX_WAIT_S" ]; then
                    restart_budget="$MAX_WAIT_S"
                fi
                deadline=$(( $(date +%s) + restart_budget ))
                raw="$(inspect_nav2)"
                ;;
            fail)
                emit false "nav2 healthcheck ExitCode=${exit_code} (want 0: controller_server, planner_server, bt_navigator active; zenoh-router=${router})"
                ;;
            *)
                emit false "unexpected nav2 action '${action}'"
                ;;
        esac
    done
}

# При source (тесты решения) main не запускается: PI_* и робот не нужны.
if [ "${BASH_SOURCE[0]}" = "$0" ]; then
    main "$@"
fi
