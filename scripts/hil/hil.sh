#!/usr/bin/env bash
# HIL: стек настоящего робота на данных Gazebo с песочницы (эпик #4371, шаг 1 — навигация).
# План и порядок запуска: docs/research/2026-11-04-hil-robot-on-gazebo-data.md §8.
#
#   scripts/hil/hil.sh status   что сейчас: замок, контейнеры, sim time, /controller_manager
#   scripts/hil/hil.sh on       включить HIL (робот на кирпичах!)
#   scripts/hil/hil.sh off      вернуть робота как было (docker compose up -d)
#   scripts/hil/hil.sh restart <сервис>...  пересоздать сервисы контура в работающем HIL (рестарт стека, #4468)
#
# on:
#   1. проверки: ssh до четырёх машин, настройки, пути карт и реестра (не боевые), образ сима;
#      read-only проба e2e_mode робота (e2e_probe) до любых записей на Pi — продолжает ТОЛЬКО
#      по достоверному false: рекреация voice-assistant сбросила бы e2e_mode марафона (#4686);
#   2. замок HIL_LOCK (/tmp/hil.lock) на Main Pi — деплой его видит и не идёт;
#   3. Main: docker stop ros2-control lslidar — моторы (CAN → VESC) получают команды
#      только от ros2-control; Vision: docker stop oak-d ceiling-camera;
#   3b. Sound: снапшот боевого `SELECT * FROM maps` (только чтение) в HIL_MAPS_SNAPSHOT;
#      voice-assistant пересоздаётся с docker/sound/compose.hil.yaml — реестр карт и точек
#      (maps/waypoints) в отдельном файле HIL_WAYPOINT_DB, остальная память боевая (#4392);
#   4. Main: robot-state-publisher, twist-mux, proximity-guard, followpath-watchdog, rtabmap, imu-filter, nav2 пересоздаются
#      с USE_SIM_TIME=true и картой HIL_MAPS_DIR (docker/main/compose.hil.yaml, --no-deps);
#   5. песочница: zenoh-router + gazebo из docker/sim/compose.hil.yaml под ID робота.
#      В контейнере gazebo сторож (docker/sim/hil_run.sh): при живом /controller_manager
#      робота Gazebo не стартует, второй контроллер позже — Gazebo гасится;
#   6. ждём: в графе ровно один /controller_manager, контейнер ros2-control не работает —
#      значит, единственный контроллер — Gazebo; у /diff_drive_controller/cmd_vel_unstamped
#      ровно один подписчик (§5.1). Иначе сим гасится, робот остаётся без моторов.
# off: сим гасится первым, затем на Main — Created-контейнеры MAIN_SIM_TIME (в т.ч. с
#   hash-префиксом вроде abe6c83d1f0a_imu-filter) docker rm, иначе следующий recreate
#   Conflict; дальше Vision и Sound `docker compose up -d --no-deps` тех же сервисов с
#   теми же образами (закреплены по работающим контейнерам), замок снимается;
#   восстановление пересоздаёт voice-assistant — активный e2e_mode сбросится: громкое
#   предупреждение в логе, сам off не блокируем (restore возвращает моторы, #4686);
#   последним — сверка боевого `SELECT * FROM maps` со снапшотом on: изменился — выход 3.
#   Состояние «до»: сразу после замка on пишет running|restarting|exited|absent каждого затрагиваемого
#   сервиса в HIL_PRE_STATE (Main Pi). off сверяет с ним: сервис был нерабочим до on и таким остался —
#   предупреждение, не провал; провал только у сервиса, который до on работал, а после off нет.
#   Сверка реестра идёт всегда; замок снимается, если не провалился ни один бывший рабочий сервис.
#
# Настройки — scripts/hil/hil.conf (переопределяются переменными окружения или HIL_CONF).
# Пароли не используются: ssh -o BatchMode=yes, нужен ключ в authorized_keys.
set -euo pipefail
export MSYS_NO_PATHCONV=1

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$HERE/../.." && pwd)"
if [ -n "${HIL_CONF:-}" ]; then
    # shellcheck source=/dev/null
    . "$HIL_CONF"
fi
# shellcheck source=scripts/hil/hil.conf
. "$HERE/hil.conf"

HIL_SSH="${HIL_SSH:-ssh}"
SSH_OPTS=(-o BatchMode=yes -o ConnectTimeout=10 -o ServerAliveInterval=5 -o ServerAliveCountMax=3)

MAIN_STOP=(ros2-control lslidar)
VISION_STOP=(oak-d ceiling-camera)
MAIN_SIM_TIME=(robot-state-publisher twist-mux proximity-guard followpath-watchdog rtabmap imu-filter nav2)
SIM_SERVICES=(zenoh-router gazebo)
# Сервис и контейнер Sound Pi с mcp_server — владельцем реестра карт и точек (WaypointStore).
SOUND_SWITCH=(voice-assistant)

log() { printf '[hil %s] %s\n' "$(date -u +%H:%M:%SZ)" "$*"; }
# die после взятого замка сам возвращает робота в бой (#5138): подсказка «hil.sh off»
# строкой не гасит USE_SIM_TIME и не поднимает моторы. До замка — только печать.
# HIL_ABORTING не даёт die внутри cmd_off зайти в возврат второй раз.
die() {
    printf '❌ %s\n' "$*" >&2
    if [ "${HIL_LOCK_HELD:-}" = 1 ] && [ "${HIL_ABORTING:-}" != 1 ]; then
        HIL_ABORTING=1
        hil_abort_restore || true
    fi
    exit 1
}
# Вернуть робота штатным off и не оставить контейнер, который compose down не снял.
# Created-но-не-запущенный gazebo (compose завис на Creating) down не удаляет —
# cmd_off тогда отказывает «робота не возвращаю» и бросает робота в полу-HIL.
hil_abort_restore() {
    log "гейт отказал: возвращаю робота в бой (штатный off)"
    sim_down
    local name
    for name in "$HIL_SIM_PREFIX-gazebo" "$HIL_SIM_PREFIX-zenoh-router"; do
        rsh "$HIL_SANDBOX_HOST" "docker rm -f $(q "$name")" >/dev/null 2>&1 || true
    done
    cmd_off
}

rsh() {  # rsh <host> <команда>
    local host="$1"; shift
    "$HIL_SSH" "${SSH_OPTS[@]}" "$host" "$*" </dev/null
}
rput() {  # rput <host> <локальный файл> <удалённый путь>
    "$HIL_SSH" "${SSH_OPTS[@]}" "$1" "mkdir -p $(q "$(dirname "$3")") && cat > $(q "$3")" <"$2"
}
# Кавычки для удалённой оболочки: 'значение' (одинарная кавычка внутри — '\'').
q() {
    local s="$1" sq="'\\''"
    printf "'%s'" "${s//"'"/$sq}"
}

REMOTE_PI="$HIL_REMOTE_DIR/remote_pi.sh"
rpi() {  # rpi <host> <подкоманда> <аргументы>...
    local host="$1"; shift
    local args=() a
    for a in "$@"; do args+=("$(q "$a")"); done
    rsh "$host" "bash $REMOTE_PI ${args[*]}"
}

# e2e_probe: read-only проба e2e_mode /speaker_id_node внутри ${SOUND_SWITCH[0]}.
# stdout — «true|false» только по подтверждённому ответу узла; rc≠0 — пробы не было
# (нет контейнера/ros, пустой или битый вывод). Ничего на Pi не пишет и от Pi ничего
# не требует (в отличие от remote_pi.sh) — зовётся до push_helpers, чтобы отказ не
# оставил следов на Pi (#4686).
e2e_probe() {
    # Одинарные кавычки в q('...') намеренны: этот скрипт раскрывается удалённым
    # bash -c на Sound Pi, локального раскрытия $( ) и $out здесь быть не должно.
    # shellcheck disable=SC2016
    rsh "$HIL_SOUND_HOST" "docker exec $(q "${SOUND_SWITCH[0]}") bash -c $(q 'source /opt/ros/humble/setup.bash; source /ws/install/setup.bash; out=$(ros2 param get /speaker_id_node e2e_mode --no-daemon 2>/dev/null | grep -aoE "Boolean value is: (True|False)" | tail -1); [ -n "$out" ] || exit 9; case "$out" in *True*) printf true ;; *) printf false ;; esac')"
}

# e2e_guard: on/off пересоздают ${SOUND_SWITCH[@]} (docker compose up -d), а рекреация
# контейнера поднимает speaker_id_node с e2e_mode=false по умолчанию — марафон/e2e-прогон
# теряет режим посреди актов (#4686). Fail-closed: продолжать можно только по достоверному
# false; true, пустой/битый вывод и ненулевой probe — отказ/предупреждение.
e2e_guard() {  # $1 = abort|warn
    local st rc=0
    st=$(e2e_probe 2>/dev/null) || rc=$?
    if [ "$rc" -ne 0 ] || [ "$st" != false ]; then
        if [ "$1" = abort ]; then
            die "на Sound Pi не подтверждён выключенный e2e-режим (${SOUND_SWITCH[0]}: probe rc=$rc, ответ='${st:-пусто}'): пересоздание контейнера сбросит e2e_mode и сломает e2e-прогон; дождись его конца или сними режим вручную (issue #4686)"
        fi
        log "⚠️ e2e-режим ${SOUND_SWITCH[0]} не подтверждён выключенным (probe rc=$rc, ответ='${st:-пусто}'): restore всё равно сбросит e2e_mode (#4686) — чужой e2e-прогон придётся повторить"
    fi
}

# ── Состояние сервисов «до on» (решение «сломан до on → предупреждение» принимает код) ──

# Снимок «имя статус» для всех сервисов, которые HIL останавливает или пересоздаёт.
pre_state_snapshot() {
    local line n st
    {
        rpi "$HIL_MAIN_HOST" state "${MAIN_STOP[@]}" "${MAIN_SIM_TIME[@]}"
        rpi "$HIL_VISION_HOST" state "${VISION_STOP[@]}"
        rpi "$HIL_SOUND_HOST" state "${SOUND_SWITCH[@]}"
    } | while read -r line; do
        [ -n "$line" ] || continue
        # «имя статус USE_SIM_TIME=…» → «имя статус»
        read -r n st _ <<<"$line"
        printf '%s %s
' "$n" "$st"
    done
}

# PRE_STATE (текст «имя статус») → статус сервиса до on; нет записи — running (старое поведение: ждём работающим).
pre_status_of() {
    local name="$1" st
    st=$(printf '%s
' "${PRE_STATE:-}" | awk -v n="$name" '$1 == n { print $2; exit }')
    printf '%s' "${st:-running}"
}

# svc_verdict <имя> <статус после off>: ok — работает; warn — не работает, но не работал и до on; fail — иначе.
svc_verdict() {
    local name="$1" after="$2" before
    if [ "$after" = running ]; then echo ok; return 0; fi
    before=$(pre_status_of "$name")
    if [ "$before" = running ]; then echo fail; else echo warn; fi
}

# ── Проверки настроек (до любого действия) ────────────────────────────────

is_ipv4() { [[ "$1" =~ ^([0-9]{1,3}\.){3}[0-9]{1,3}$ ]]; }

# Путь карты HIL не должен совпадать с боевыми данными: ./maps Main Pi, /maps, /data.
check_hil_paths() {
    local p="$HIL_MAPS_DIR"
    case "$p" in
        /*) ;;
        *) die "HIL_MAPS_DIR='$p' — нужен абсолютный путь на Main Pi" ;;
    esac
    case "/$p/" in
        */../* | */./*) die "HIL_MAPS_DIR='$p' — без . и .. в пути" ;;
    esac
    p="${p%/}"
    case "$p" in
        "" | /maps | /maps/* | /data | /data/*)
            die "HIL_MAPS_DIR='$HIL_MAPS_DIR' — боевой путь (/maps, /data), HIL туда не пишет" ;;
        "${HIL_MAIN_DIR%/}/maps" | "${HIL_MAIN_DIR%/}/maps/"*)
            die "HIL_MAPS_DIR='$HIL_MAPS_DIR' — это боевой ./maps Main Pi (карта мастерской)" ;;
    esac
    case "$HIL_RTABMAP_MODE" in
        mapping | localization) ;;
        *) die "HIL_RTABMAP_MODE='$HIL_RTABMAP_MODE' — mapping или localization" ;;
    esac
    case "$HIL_GUARD_SOUND" in
        on | off) ;;
        *) die "HIL_GUARD_SOUND='$HIL_GUARD_SOUND' — on или off" ;;
    esac
    check_hil_registry
}

# Реестр карт и точек HIL — файл внутри voice-assistant только под /data/hil/ (не боевой
# /data/voice_memory.db и не соседние БД памяти), имя *.db, без . и .. (#4392).
check_hil_registry() {
    local p="$HIL_WAYPOINT_DB"
    case "/$p/" in
        */../* | */./*) die "HIL_WAYPOINT_DB='$p' — без . и .. в пути" ;;
    esac
    case "$p" in
        /data/hil/?*.db) ;;
        *) die "HIL_WAYPOINT_DB='$p' — нужен файл *.db под /data/hil/ (внутри voice-assistant), не боевая БД" ;;
    esac
    [ "$p" != "$HIL_PROD_TOOLS_DB" ] || die "HIL_WAYPOINT_DB совпадает с боевой БД инструментов"
    case "$HIL_PROD_TOOLS_DB" in
        /*) ;;
        *) die "HIL_PROD_TOOLS_DB='$HIL_PROD_TOOLS_DB' — нужен абсолютный путь внутри voice-assistant" ;;
    esac
    case "$HIL_MAPS_SNAPSHOT" in
        /*) ;;
        *) die "HIL_MAPS_SNAPSHOT='$HIL_MAPS_SNAPSHOT' — нужен абсолютный путь на Sound Pi" ;;
    esac
}

check_net() {
    is_ipv4 "$HIL_SANDBOX_IP" || die "HIL_SANDBOX_IP='$HIL_SANDBOX_IP' — нужен IPv4"
    local ip
    for ip in $HIL_ROBOT_ROUTERS; do
        is_ipv4 "$ip" || die "HIL_ROBOT_ROUTERS: '$ip' — нужен IPv4"
    done
    [ -n "$HIL_ROBOT_ROUTERS" ] || die "HIL_ROBOT_ROUTERS пуст"
    [[ "$HIL_SANDBOX_IFACE" =~ ^[A-Za-z0-9._-]*$ ]] || die "HIL_SANDBOX_IFACE='$HIL_SANDBOX_IFACE'"
}

# tcp/10.1.1.10:7447#iface=enp2s0,tcp/10.1.1.11:7447#iface=enp2s0,...
zenoh_connect() {
    local out="" ip ep
    for ip in $HIL_ROBOT_ROUTERS; do
        ep="tcp/$ip:7447"
        [ -n "$HIL_SANDBOX_IFACE" ] && ep="$ep#iface=$HIL_SANDBOX_IFACE"
        out="${out:+$out,}$ep"
    done
    printf '%s' "$out"
}

resolve_robot_id() {
    if [ -z "$HIL_ROBOT_ID" ]; then
        HIL_ROBOT_ID=$(rsh "$HIL_MAIN_HOST" "sed -n 's/^ROBOT_ID=//p' $(q "$HIL_MAIN_DIR/.env") | tail -n 1" | tr -d '\r"')
    fi
    [[ "$HIL_ROBOT_ID" =~ ^[A-Za-z0-9_-]+$ ]] || die "ROBOT_ID робота не определён ('$HIL_ROBOT_ID'): задай HIL_ROBOT_ID"
    [ "$HIL_ROBOT_ID" != sim ] || die "HIL_ROBOT_ID=sim — это ID изолированного сима, а не робота"
}

push_helpers() {
    local host
    for host in "$HIL_MAIN_HOST" "$HIL_VISION_HOST" "$HIL_SOUND_HOST"; do
        rput "$host" "$HERE/remote_pi.sh" "$REMOTE_PI"
    done
    rput "$HIL_MAIN_HOST" "$REPO_ROOT/docker/main/compose.hil.yaml" "$HIL_REMOTE_DIR/main.hil.yaml"
    rput "$HIL_SOUND_HOST" "$REPO_ROOT/docker/sound/compose.hil.yaml" "$HIL_REMOTE_DIR/sound.hil.yaml"
}

# ── Sound Pi: реестр карт и точек (#4392) ─────────────────────────────────

# Снапшот боевого maps: сначала во временный файл, потом mv — оборванный ssh не оставит полснапшота.
sound_snapshot_maps() {
    rsh "$HIL_SOUND_HOST" "mkdir -p $(q "$(dirname "$HIL_MAPS_SNAPSHOT")") && bash $REMOTE_PI maps-dump $(q "${SOUND_SWITCH[0]}") $(q "$HIL_PROD_TOOLS_DB") > $(q "$HIL_MAPS_SNAPSHOT.part") && mv -f $(q "$HIL_MAPS_SNAPSHOT.part") $(q "$HIL_MAPS_SNAPSHOT")"
}

# off, последний шаг: боевой maps сейчас против снапшота on. 0 — совпало, 3 — изменился, 1 — не сверить.
sound_verify_maps() {
    local now before
    if ! rsh "$HIL_SOUND_HOST" "test -f $(q "$HIL_MAPS_SNAPSHOT")"; then
        log "⚠️  боевой реестр карт: снапшота $HIL_SOUND_HOST:$HIL_MAPS_SNAPSHOT нет (on не доходил до Sound?) — сверять не с чем"
        return 1
    fi
    before=$(rsh "$HIL_SOUND_HOST" "cat $(q "$HIL_MAPS_SNAPSHOT")") || { log "⚠️  снапшот не прочитан"; return 1; }
    now=$(rpi "$HIL_SOUND_HOST" maps-dump "${SOUND_SWITCH[0]}" "$HIL_PROD_TOOLS_DB") \
        || { log "⚠️  боевой реестр карт не прочитан ($HIL_PROD_TOOLS_DB) — сверка не сделана"; return 1; }
    log "боевой реестр карт до on ($(grep -c . <<<"$before") строк):"
    printf '%s\n' "$before" | sed 's/^/    до    /'
    log "боевой реестр карт после off ($(grep -c . <<<"$now") строк):"
    printf '%s\n' "$now" | sed 's/^/    после /'
    if [ "$before" = "$now" ]; then
        log "✅ боевой реестр карт (SELECT * FROM maps) за HIL не изменился"
        rsh "$HIL_SOUND_HOST" "rm -f -- $(q "$HIL_MAPS_SNAPSHOT")" || true
        return 0
    fi
    log "❌ боевой реестр карт за HIL ИЗМЕНИЛСЯ (diff до → после):"
    diff <(printf '%s\n' "$before") <(printf '%s\n' "$now") | sed 's/^/    /' || true
    log "снапшот оставлен для разбора: $HIL_SOUND_HOST:$HIL_MAPS_SNAPSHOT"
    return 3
}

# ── Песочница ─────────────────────────────────────────────────────────────

sim_env() {
    printf 'HIL_ROBOT_ID=%s HIL_SANDBOX_IP=%s HIL_ZENOH_CONNECT=%s SIM_IMAGE=%s SIM_PREFIX=%s SIM_WORLD=%s SIM_DRI_CARD=%s SIM_DRI_RENDER=%s' \
        "$(q "$HIL_ROBOT_ID")" "$(q "$HIL_SANDBOX_IP")" "$(q "$(zenoh_connect)")" "$(q "$HIL_SIM_IMAGE")" \
        "$(q "$HIL_SIM_PREFIX")" "$(q "$HIL_SIM_WORLD")" "$(q "$HIL_SIM_DRI_CARD")" "$(q "$HIL_SIM_DRI_RENDER")"
}
sim_compose() {  # sim_compose <args compose>...
    rsh "$HIL_SANDBOX_HOST" "cd $(q "$HIL_SANDBOX_REPO") && $(sim_env) docker compose -p $(q "$HIL_SIM_PROJECT") -f docker/sim/compose.yaml -f docker/sim/compose.hil.yaml $*"
}
sim_gazebo_state() {
    rsh "$HIL_SANDBOX_HOST" "docker inspect -f '{{.State.Status}} {{.State.ExitCode}}' $(q "$HIL_SIM_PREFIX-gazebo") 2>/dev/null | grep . || echo 'absent -'"
}
# Топики сима, по которым гейт судит мир (#5138). Голого /odom в симе нет:
# его публикует rtabmap icp_odometry на роботе, в HIL-симе этого нет.
# Модель/плагин подняты = мост Gazebo дал /sim/ground_truth/odom или идёт /clock.
# Контроллер поднят = есть /diff_drive_controller/odom.
SIM_TRUTH_ODOM="/sim/ground_truth/odom"
SIM_DRIVE_ODOM="/diff_drive_controller/odom"
# Сырьё гейта контроллера (#5138): снимается ДО гашения сима, пока контейнер ещё есть.
# Строка inspect + хвост логов + три топика сима. Пустой ответ поля — «?», не выдумка.
sim_gate_evidence() {
    local gz="$HIL_SIM_PREFIX-gazebo" line topics
    line=$(rsh "$HIL_SANDBOX_HOST" "docker inspect -f 'Status={{.State.Status}} RestartCount={{.RestartCount}} StartedAt={{.State.StartedAt}} ExitCode={{.State.ExitCode}}' $(q "$gz")" 2>/dev/null | grep . || true)
    printf 'сырьё gazebo inspect: %s\n' "${line:-inspect не ответил}" >&2
    printf 'сырьё gazebo logs (tail 30):\n' >&2
    rsh "$HIL_SANDBOX_HOST" "docker logs --tail 30 $(q "$gz")" >&2 2>&1 || printf 'docker logs не ответил\n' >&2
    topics=$(sim_exec_bounded "$gz" "/entrypoint.sh ros2 topic list 2>/dev/null" || true)
    if [ -z "$topics" ]; then
        printf 'сырьё sim: ros2 topic list пуст или не ответил (/clock нет, %s нет, %s нет, топиков 0); /odom здесь не проверяем: его в симе нет\n' \
            "$SIM_TRUTH_ODOM" "$SIM_DRIVE_ODOM" >&2
        return 0
    fi
    local n clock=нет truth=нет drive=нет
    n=$(printf '%s\n' "$topics" | grep -c .)
    if printf '%s\n' "$topics" | grep -qx '/clock'; then clock=есть; fi
    if printf '%s\n' "$topics" | grep -qx "$SIM_TRUTH_ODOM"; then truth=есть; fi
    if printf '%s\n' "$topics" | grep -qx "$SIM_DRIVE_ODOM"; then drive=есть; fi
    printf 'сырьё sim: /clock %s, %s %s, %s %s, топиков %s; /odom здесь не проверяем: его в симе нет\n' \
        "$clock" "$SIM_TRUTH_ODOM" "$truth" "$SIM_DRIVE_ODOM" "$drive" "$n" >&2
}
# Причина таймаута гейта — из сырья, не общая фраза. Код печатает одну строку.
# (а) контейнер перезапускался или вышел с кодом;
# (б) жив, но нет ни /sim/ground_truth/odom, ни /clock — модель/плагин не поднялись;
# (в) модель есть (/clock или ground_truth), а /diff_drive_controller/odom нет;
# иначе — прежнее сообщение. Голый /odom не смотрим: в симе его нет.
sim_cm_timeout_cause() {  # sim_cm_timeout_cause <inspect-строка> <topic-list>
    local inspect="$1" topics="$2" status rc restart
    status=$(printf '%s\n' "$inspect" | sed -n 's/.*Status=\([^ ]*\).*/\1/p' | head -n 1)
    rc=$(printf '%s\n' "$inspect" | sed -n 's/.*ExitCode=\([^ ]*\).*/\1/p' | head -n 1)
    restart=$(printf '%s\n' "$inspect" | sed -n 's/.*RestartCount=\([^ ]*\).*/\1/p' | head -n 1)
    case "$restart" in
        ''|*[!0-9]*) restart=0 ;;
    esac
    if [ "$restart" -gt 0 ] || { [ -n "$rc" ] && [ "$rc" != 0 ]; }; then
        printf 'Gazebo перезапускался (RestartCount=%s, ExitCode=%s, Status=%s): смотри сырьё выше; hil.sh off' \
            "${restart:-?}" "${rc:-?}" "${status:-?}"
        return 0
    fi
    local clock=0 truth=0 drive=0
    if printf '%s\n' "$topics" | grep -qx '/clock'; then clock=1; fi
    if printf '%s\n' "$topics" | grep -qx "$SIM_TRUTH_ODOM"; then truth=1; fi
    if printf '%s\n' "$topics" | grep -qx "$SIM_DRIVE_ODOM"; then drive=1; fi
    if [ "$status" = running ] && [ "$truth" = 0 ] && [ "$clock" = 0 ]; then
        printf 'Gazebo жив, но кадр odom не публикуется: модель/плагин не поднялись (нет %s и /clock); hil.sh off' \
            "$SIM_TRUTH_ODOM"
        return 0
    fi
    if { [ "$clock" = 1 ] || [ "$truth" = 1 ]; } && [ "$drive" = 0 ]; then
        printf 'мир идёт, ros2_control не поднял контроллеры (нет %s); hil.sh off' "$SIM_DRIVE_ODOM"
        return 0
    fi
    printf 'за %s с контроллер Gazebo не появился; hil.sh off' "$HIL_WAIT_CM_S"
}
# docker exec внутри гейта ограничен по времени (#5138): живой заход 22.11 держал
# `hil count --settle 3` 234 с, и цикл из 5 с за это время не проверял ничего.
# «timeout» — проба не ответила; пусто — exec упал. Код вызывает сам решает.
sim_exec_bounded() {  # sim_exec_bounded <контейнер> <команда внутри>
    rsh "$HIL_SANDBOX_HOST" "timeout $(q "$HIL_PROBE_TIMEOUT_S") docker exec $(q "$1") $2"
}
# Число /controller_manager в графе — глазами сторожа внутри gazebo.
# «-» — gazebo не работает или ответ не число; «timeout» — проба не ответила за срок.
sim_cm_count() {
    local out rc=0
    out=$(sim_exec_bounded "$HIL_SIM_PREFIX-gazebo" \
        "/entrypoint.sh python3 -m rob_box_sim.hil count --settle 3 2>/dev/null") || rc=$?
    if [ "$rc" -eq 124 ]; then
        echo timeout
        return 0
    fi
    printf '%s\n' "$out" | grep -E '^[0-9]+$' | tail -n 1 | grep . || echo -
}
# Подписчики /diff_drive_controller/cmd_vel_unstamped (§5.1): норма HIL — 1 (только Gazebo).
# «-» — gazebo не работает или topic info не ответил.
CMD_VEL_UNSTAMPED="/diff_drive_controller/cmd_vel_unstamped"
sim_cmd_vel_subs() {
    local out rc=0
    out=$(sim_exec_bounded "$HIL_SIM_PREFIX-gazebo" \
        "/entrypoint.sh ros2 topic info $(q "$CMD_VEL_UNSTAMPED") 2>/dev/null") || rc=$?
    if [ "$rc" -eq 124 ]; then
        echo timeout
        return 0
    fi
    printf '%s\n' "$out" | sed -n 's/^Subscription count: *//p' | tail -n 1 | grep -E '^[0-9]+$' || echo -
}
sim_down() {
    log "песочница: гашу zenoh-router и gazebo ($HIL_SIM_PROJECT)"
    sim_compose down --timeout 20 || log "⚠️  compose down на песочнице вернул ошибку"
}
# Мир до первой цели Depot (#4694): PASS только по трём фактам (args/mount/SDF).
# Обязательность решает код (world_check.py --gate-required): HIL_EXPECT_WORLD=depot
# или точный world:=/sim_video/world.sdf. Обычный polygon без переменной — молчит.
# world.sdf.old гейт не включает.
sim_world_gate_required() {  # sim_world_gate_required <expect> <args>
    local ans
    ans=$(python3 -X utf8 "$HERE/checks/world_check.py" --gate-required "$1" --args "$2" 2>/dev/null || true)
    [ "$ans" = required ]
}
# Копия Compose на песочнице — штатный Depot? (полный маршрут, M1, R7).
# Код печатает required|skip. Пустой ответ — не «skip»: файл не прочитан.
sim_on_must_check_depot() {  # sim_on_must_check_depot <expect> <локальный файл compose>
    local ans
    ans=$(python3 -X utf8 "$HERE/checks/world_check.py" --on-must-check "$1" --compose-file "$2" 2>/dev/null || true)
    [ "$ans" = required ]
}
sim_check_expect_world() {
    local expect="$1" gz="$HIL_SIM_PREFIX-gazebo" args mounts wpath sdf image line rc=0 tmp
    args=$(rsh "$HIL_SANDBOX_HOST" "docker inspect -f '{{join .Args \" \"}}' $(q "$gz")" 2>/dev/null || true)
    mounts=$(rsh "$HIL_SANDBOX_HOST" "docker inspect -f '{{range .Mounts}}{{println .Destination}}{{end}}' $(q "$gz")" 2>/dev/null || true)
    image=$(rsh "$HIL_SANDBOX_HOST" "docker inspect -f '{{.Image}}' $(q "$gz")" 2>/dev/null || true)
    wpath=""
    case "$args" in
        *world:=*) wpath="${args#*world:=}"; wpath="${wpath%% *}" ;;
    esac
    sdf=""
    if [ -n "$wpath" ]; then
        sdf=$(rsh "$HIL_SANDBOX_HOST" "docker exec $(q "$gz") cat $(q "$wpath") 2>/dev/null" || true)
    fi
    if [ -n "${HIL_WORLD_CHECK_DIR:-}" ]; then
        mkdir -p "$HIL_WORLD_CHECK_DIR"
        # Бриф 15:44 UTC: каталог Compose, команда Gazebo, SHA SDF, ID образа.
        printf '%s\n' "$HIL_SANDBOX_REPO" >"$HIL_WORLD_CHECK_DIR/compose_dir.txt"
        printf '%s\n' "$args" >"$HIL_WORLD_CHECK_DIR/gazebo_args.txt"
        printf '%s\n' "$args" >"$HIL_WORLD_CHECK_DIR/gazebo_command.txt"
        printf '%s\n' "$mounts" >"$HIL_WORLD_CHECK_DIR/gazebo_mounts.txt"
        printf '%s\n' "$image" >"$HIL_WORLD_CHECK_DIR/gazebo_image_id.txt"
        if [ -n "$sdf" ]; then
            printf '%s' "$sdf" | sha256sum | awk '{print $1}' >"$HIL_WORLD_CHECK_DIR/sdf.sha256"
        else
            printf 'missing\n' >"$HIL_WORLD_CHECK_DIR/sdf.sha256"
        fi
    fi
    tmp=$(mktemp -d)
    printf '%s\n' "$mounts" >"$tmp/mounts"
    printf '%s' "$sdf" >"$tmp/sdf"
    line=$(python3 -X utf8 "$HERE/checks/world_check.py" "$expect" \
        --args "$args" --mounts-file "$tmp/mounts" --sdf-file "$tmp/sdf") || rc=$?
    rm -rf "$tmp"
    if [ -n "${HIL_WORLD_CHECK_DIR:-}" ]; then
        printf '%s\n' "$line" >"$HIL_WORLD_CHECK_DIR/verdict.txt"
    fi
    if [ "$rc" -ne 0 ]; then
        sim_down
        die "$line; hil.sh off"
    fi
    log "мир: $line"
}

# ── Main Pi: контур навигации в HIL ───────────────────────────────────────

# created_containers_to_rm: stdin «ID NAMES STATUS…» (как docker ps --format),
# аргументы — имена сервисов. stdout — ID со статусом Created, чьё имя равно
# сервису или оканчивается на _сервис / -сервис (hash-префикс compose v1,
# project-svc[-N] v2). Up/Exited/Paused не трогаем — только Created.
created_containers_to_rm() {
    local id name status cand tail svc
    while read -r id name status _; do
        [ -n "${id:-}" ] && [ -n "${name:-}" ] || continue
        case "$status" in
            Created | created) ;;
            *) continue ;;
        esac
        cand="$name"
        tail="${name##*-}"
        case "$tail" in
            "" | *[!0-9]*) ;;
            *) cand="${name%-*}" ;;
        esac
        for svc in "$@"; do
            case "$name" in
                "$svc" | *_"$svc" | *-"$svc") printf '%s\n' "$id"; continue 2 ;;
            esac
            if [ "$cand" != "$name" ]; then
                case "$cand" in
                    "$svc" | *_"$svc" | *-"$svc") printf '%s\n' "$id"; continue 2 ;;
                esac
            fi
        done
    done
}

# Убрать Created-контейнеры MAIN_SIM_TIME на Main до recreate: осиротевший
# abe6c83d1f0a_imu-filter (и аналоги) иначе даёт Conflict на следующем up.
main_rm_created_sim() {
    local list ids line
    list=$(rsh "$HIL_MAIN_HOST" "docker ps -a --format '{{.ID}} {{.Names}} {{.Status}}'") || true
    ids=$(printf '%s\n' "$list" | created_containers_to_rm "${MAIN_SIM_TIME[@]}")
    [ -n "$ids" ] || return 0
    line=$(printf '%s\n' "$ids" | tr '\n' ' ')
    line=${line% }
    log "Main: docker rm Created контура HIL: $line"
    # shellcheck disable=SC2086
    rsh "$HIL_MAIN_HOST" "docker rm $line" || log "⚠️  docker rm Created не полностью ($line)"
}

# compose up сервисов контура (on и restart): sim time, карта HIL_MAPS_DIR, режим rtabmap HIL_RTABMAP_MODE.
# HIL_NAV2TUNE_COMPOSE — необязательный четвёртый -f (повтор M1, #4694). Пусто — три файла, как раньше.
# HIL_GOAL_PROBE_COMPOSE — необязательный пятый -f (проба goal-checker, #4945), только вместе с
# M1_GOAL_PROBE_MOUNT: compose пробы требует переменную (${M1_GOAL_PROBE_MOUNT:?}).
# off эти файлы не читает: боевой nav2 поднимается без оверлея и без пробы.
main_sim_up() {  # main_sim_up [флаги compose up] <сервисы>...
    main_rm_created_sim
    local extra=""
    local probe_env=""
    if [ -n "${HIL_NAV2TUNE_COMPOSE:-}" ]; then
        extra="-f $(q "$HIL_NAV2TUNE_COMPOSE")"
    fi
    if [ -n "${HIL_GOAL_PROBE_COMPOSE:-}" ]; then
        extra="$extra -f $(q "$HIL_GOAL_PROBE_COMPOSE")"
        probe_env="M1_GOAL_PROBE_MOUNT=$(q "${M1_GOAL_PROBE_MOUNT:-}")"
    fi
    # shellcheck disable=SC2086
    rsh "$HIL_MAIN_HOST" "cd $(q "$HIL_MAIN_DIR") && HIL_MAPS_DIR=$(q "$HIL_MAPS_DIR") HIL_RTABMAP_MODE=$(q "$HIL_RTABMAP_MODE") HIL_GUARD_SOUND=$(q "$HIL_GUARD_SOUND") NAV2TUNE_OVERLAYS=$(q "${NAV2TUNE_OVERLAYS:-}") $probe_env docker compose -f docker-compose.yaml -f $(q "$HIL_REMOTE_DIR/main.images.yaml") -f $(q "$HIL_REMOTE_DIR/main.hil.yaml") $extra up -d --no-deps --pull never $*"
}

# ── Команды ───────────────────────────────────────────────────────────────

preflight() {
    check_hil_paths
    check_net
    local host
    for host in "$HIL_MAIN_HOST" "$HIL_VISION_HOST" "$HIL_SOUND_HOST" "$HIL_SANDBOX_HOST"; do
        rsh "$host" true || die "нет ssh до $host (нужен ключ, пароли не используются)"
    done
    resolve_robot_id
    rsh "$HIL_SANDBOX_HOST" "test -f $(q "$HIL_SANDBOX_REPO/docker/sim/compose.hil.yaml")" \
        || die "на песочнице нет $HIL_SANDBOX_REPO с docker/sim/compose.hil.yaml (см. план §8)"
    rsh "$HIL_SANDBOX_HOST" "docker image inspect $(q "$HIL_SIM_IMAGE") >/dev/null" \
        || die "на песочнице нет образа $HIL_SIM_IMAGE (docker build -f docker/sim/Dockerfile -t $HIL_SIM_IMAGE .)"
    log "ROBOT_ID=$HIL_ROBOT_ID, песочница $HIL_SANDBOX_IP (iface=${HIL_SANDBOX_IFACE:-—}), роутеры робота: $HIL_ROBOT_ROUTERS"
}

validate_hil_run_token() {
    if [ -n "${HIL_RUN_TOKEN:-}" ] && ! [[ "$HIL_RUN_TOKEN" =~ ^[0-9a-f]{32}$ ]]; then
        die "HIL_RUN_TOKEN: ожидались 32 строчных шестнадцатеричных цифры"
    fi
}

check_hil_off_owner() {
    validate_hil_run_token
    [ -n "${HIL_RUN_TOKEN:-}" ] || return 0
    local lock
    lock=$(rsh "$HIL_MAIN_HOST" "cat $(q "$HIL_LOCK") 2>/dev/null") || die "off: замок HIL не прочитан"
    [[ "$lock" == "HIL on since "*" token=$HIL_RUN_TOKEN maps=$HIL_MAPS_DIR" ]] \
        || die "off: замок HIL принадлежит другому запуску"
}

cmd_on() {
    validate_hil_run_token
    preflight
    e2e_guard abort
    push_helpers
    local who
    who="HIL on since $(date -u +%FT%TZ) by ${USER:-${USERNAME:-?}}@$(hostname) sandbox=$HIL_SANDBOX_IP"
    if [ -n "${HIL_RUN_TOKEN:-}" ]; then
        who+=" token=$HIL_RUN_TOKEN"
    fi
    who+=" maps=$HIL_MAPS_DIR"
    rpi "$HIL_MAIN_HOST" lock-take "$HIL_LOCK" "$who" || die "HIL уже включён или замок остался: scripts/hil/hil.sh status / off"
    # Замок взят: дальше любой die возвращает робота штатным off, а не подсказкой (#5138).
    HIL_LOCK_HELD=1

    log "состояние сервисов до on → $HIL_MAIN_HOST:$HIL_PRE_STATE"
    local pre
    pre=$(pre_state_snapshot) || die "не снято состояние сервисов до on; hil.sh off"
    printf '%s
' "$pre" | sed 's/^/    до on  /'
    printf '%s
' "$pre" | rput "$HIL_MAIN_HOST" /dev/stdin "$HIL_PRE_STATE" || die "не записано $HIL_PRE_STATE; hil.sh off"

    log "Main: останавливаю ${MAIN_STOP[*]} (моторы и лидар)"
    rpi "$HIL_MAIN_HOST" stop "${MAIN_STOP[@]}" || die "Main: не остановились ${MAIN_STOP[*]} — HIL не включён, замок стоит; разобраться и hil.sh off"
    log "Vision: останавливаю ${VISION_STOP[*]}"
    rpi "$HIL_VISION_HOST" stop "${VISION_STOP[@]}" || die "Vision: не остановились ${VISION_STOP[*]}; hil.sh off"

    log "Sound: снапшот боевого реестра карт ($HIL_PROD_TOOLS_DB, только чтение) → $HIL_MAPS_SNAPSHOT"
    sound_snapshot_maps || die "Sound: снапшот боевого maps не снят — реестр не переключаю; hil.sh off"
    log "Sound: ${SOUND_SWITCH[*]} → реестр карт и точек $HIL_WAYPOINT_DB (остальная память боевая)"
    rpi "$HIL_SOUND_HOST" pin "$HIL_SOUND_DIR" "$HIL_REMOTE_DIR/sound.images.yaml" \
        || die "Sound: образы не закреплены; hil.sh off"
    rsh "$HIL_SOUND_HOST" "cd $(q "$HIL_SOUND_DIR") && HIL_WAYPOINT_DB=$(q "$HIL_WAYPOINT_DB") docker compose -f docker-compose.yaml -f $(q "$HIL_REMOTE_DIR/sound.images.yaml") -f $(q "$HIL_REMOTE_DIR/sound.hil.yaml") up -d --no-deps --pull never ${SOUND_SWITCH[*]}" \
        || die "Sound: пересоздание ${SOUND_SWITCH[*]} с HIL-реестром не прошло; hil.sh off"
    local wpdb
    wpdb=$(rpi "$HIL_SOUND_HOST" env "${SOUND_SWITCH[0]}" WAYPOINT_DB_PATH)
    [ "$wpdb" = "$HIL_WAYPOINT_DB" ] \
        || die "Sound: у ${SOUND_SWITCH[0]} WAYPOINT_DB_PATH='$wpdb', ждали '$HIL_WAYPOINT_DB' — реестр боевой; hil.sh off"
    case "$(rpi "$HIL_SOUND_HOST" state "${SOUND_SWITCH[@]}")" in
        *" running "*) ;;
        *) die "Sound: ${SOUND_SWITCH[*]} не работает после пересоздания; hil.sh off" ;;
    esac

    log "Main: ${MAIN_SIM_TIME[*]} → USE_SIM_TIME=true, карта $HIL_MAPS_DIR ($HIL_RTABMAP_MODE)"
    rsh "$HIL_MAIN_HOST" "mkdir -p $(q "$HIL_MAPS_DIR")"
    rpi "$HIL_MAIN_HOST" pin "$HIL_MAIN_DIR" "$HIL_REMOTE_DIR/main.images.yaml"
    main_sim_up "${MAIN_SIM_TIME[@]}" || die "Main: пересоздание с sim time не прошло; hil.sh off"

    # Перед стартом Gazebo: контроллер робота точно не работает.
    local st
    st=$(rpi "$HIL_MAIN_HOST" state ros2-control)
    case "$st" in
        *" running "*) die "ros2-control снова работает ($st) — Gazebo не поднимаю; hil.sh off" ;;
    esac

    # До compose up: копия — штатный Depot (runvideo, M1, R7) или expect=depot.
    # Гейт обязателен и при потере overlay (#4694). Polygon без env — молчит.
    _depot_compose=$(mktemp)
    if ! rsh "$HIL_SANDBOX_HOST" "cat $(q "$HIL_SANDBOX_REPO/docker/sim/compose.hil.yaml")" >"$_depot_compose"; then
        rm -f "$_depot_compose"
        die "WORLD_MISMATCH: compose.hil.yaml не прочитан, Gazebo не поднимаю"
    fi
    if sim_on_must_check_depot "${HIL_EXPECT_WORLD:-}" "$_depot_compose"; then
        export HIL_EXPECT_WORLD=depot
        log "мир: копия Compose — Depot, проверка обязательна до первой цели"
    elif [ -n "${HIL_EXPECT_WORLD:-}" ] && [ "${HIL_EXPECT_WORLD}" != depot ]; then
        rm -f "$_depot_compose"
        die "WORLD_MISMATCH: HIL_EXPECT_WORLD=${HIL_EXPECT_WORLD} (только depot или пусто)"
    fi
    if [ -n "${HIL_WORLD_CHECK_DIR:-}" ]; then
        mkdir -p "$HIL_WORLD_CHECK_DIR"
        cp "$_depot_compose" "$HIL_WORLD_CHECK_DIR/compose.hil.yaml"
        printf '%s\n' "$HIL_SANDBOX_REPO" >"$HIL_WORLD_CHECK_DIR/compose_dir.txt"
    fi
    rm -f "$_depot_compose"

    log "песочница: поднимаю ${SIM_SERVICES[*]} (мир $HIL_SIM_WORLD)"
    if ! sim_compose up -d --no-build "${SIM_SERVICES[@]}"; then
        # #5138, второй заход: compose завис на Creating — контейнер создан и не стартовал.
        # Это не «compose up не прошёл»: причина своя, контейнер down не снимает.
        _up_gz=$(sim_gazebo_state)
        case "$_up_gz" in
            created* | Created*)
                die "песочница: gazebo создан и не стартовал (compose завис на Creating, состояние $_up_gz); робот без моторов" ;;
            *)
                die "песочница: compose up не прошёл (gazebo: $_up_gz); робот без моторов" ;;
        esac
    fi

    log "жду ровно один /controller_manager и одного подписчика $CMD_VEL_UNSTAMPED (§5.1), до $HIL_WAIT_CM_S с"
    local deadline=$(( $(date +%s) + HIL_WAIT_CM_S )) cm gz subs
    while :; do
        gz=$(sim_gazebo_state)
        case "$gz" in
            running*) ;;
            created* | Created*)
                die "контейнер gazebo не стартовал ($gz): пробы контроллера не было" ;;
            *)
                rsh "$HIL_SANDBOX_HOST" "docker logs --tail 30 $(q "$HIL_SIM_PREFIX-gazebo")" || true
                die "gazebo не работает ($gz): сторож отказал или Gazebo упал — см. лог выше" ;;
        esac
        cm=$(sim_cm_count)
        if [ "$cm" = timeout ]; then
            # #5138: контейнер жив, единственная проба не ответила. Цикл не ждёт её до конца.
            sim_gate_evidence
            die "контейнер gazebo жив, проба контроллера не ответила за ${HIL_PROBE_TIMEOUT_S} с"
        fi
        st=$(rpi "$HIL_MAIN_HOST" state ros2-control)
        case "$st" in
            *" running "*) sim_down; die "ros2-control робота работает ($st) — Gazebo погашен; hil.sh off" ;;
        esac
        if [ "$cm" = 1 ]; then
            # §5.1: один подписчик cmd_vel_unstamped (Gazebo); два — ещё слушает ros2-control.
            subs=$(sim_cmd_vel_subs)
            if [ "$subs" = 1 ]; then
                log "✅ /controller_manager ровно один, $CMD_VEL_UNSTAMPED подписчиков: 1, ros2-control робота: ${st#ros2-control }"
                break
            fi
            if [ "$subs" = timeout ]; then
                sim_gate_evidence
                die "контейнер gazebo жив, проба подписчиков $CMD_VEL_UNSTAMPED не ответила за ${HIL_PROBE_TIMEOUT_S} с"
            fi
            if [ "$subs" != 0 ] && [ "$subs" != - ]; then
                die "$CMD_VEL_UNSTAMPED подписчиков: $subs (ждали 1)"
            fi
        elif [ "$cm" != 0 ] && [ "$cm" != - ]; then
            die "/controller_manager в графе: $cm (ждали 1)"
        fi
        if [ "$(date +%s)" -ge "$deadline" ]; then
            if [ "$cm" = 1 ]; then
                sim_gate_evidence
                die "за $HIL_WAIT_CM_S с у $CMD_VEL_UNSTAMPED не ровно 1 подписчик (сейчас ${subs:--})"
            fi
            # #5138: сырьё и причина — пока контейнер жив. sim_down после, иначе логи уже не снять.
            _cm_inspect=$(rsh "$HIL_SANDBOX_HOST" "docker inspect -f 'Status={{.State.Status}} RestartCount={{.RestartCount}} StartedAt={{.State.StartedAt}} ExitCode={{.State.ExitCode}}' $(q "$HIL_SIM_PREFIX-gazebo")" 2>/dev/null | grep . || true)
            _cm_topics=$(sim_exec_bounded "$HIL_SIM_PREFIX-gazebo" "/entrypoint.sh ros2 topic list 2>/dev/null" || true)
            sim_gate_evidence
            die "$(sim_cm_timeout_cause "${_cm_inspect:-}" "${_cm_topics:-}")"
        fi
        sleep 5
    done
    # Гейт до «HIL включён» / первой цели. Обязательность решает код, не case в shell:
    # HIL_EXPECT_WORLD=depot (штатный сценарий сам его выставляет) или точный
    # world:=/sim_video/world.sdf. Обычный polygon без переменной и world.sdf.old — молчит (#4694).
    _depot_args=$(rsh "$HIL_SANDBOX_HOST" "docker inspect -f '{{join .Args \" \"}}' $(q "$HIL_SIM_PREFIX-gazebo")" 2>/dev/null || true)
    if sim_world_gate_required "${HIL_EXPECT_WORLD:-}" "$_depot_args"; then
        sim_check_expect_world depot
    fi
    log "HIL включён. Сторож в gazebo следит дальше (docker logs $HIL_SIM_PREFIX-gazebo | grep hil-guard)."
    cmd_status
}

# restart: пересоздать сервисы контура в работающем HIL — «рестарт стека» прогона здания (#4468). Карта — из
# замка (maps=), не из окружения: другой каталог открыл бы чужую базу. Режим rtabmap — HIL_RTABMAP_MODE
# (localization: mapping стирает базу на старте, --delete_db_on_start). Сначала docker stop --time
# HIL_RESTART_STOP_S: rtabmap дописывает базу на остановке, обрыв через 10 с по умолчанию оставил бы её неполной.
cmd_restart() {
    [ "$#" -gt 0 ] || die "restart: какие сервисы? из ${MAIN_SIM_TIME[*]}"
    local s lock
    for s in "$@"; do
        case " ${MAIN_SIM_TIME[*]} " in
            *" $s "*) ;;
            *) die "restart: '$s' не из контура HIL (${MAIN_SIM_TIME[*]})" ;;
        esac
    done
    case " $* " in
        *" rtabmap "*)
            [ "$HIL_RTABMAP_MODE" != mapping ] \
                || die "restart rtabmap в mapping сотрёт карту HIL (--delete_db_on_start); рестарт стека — HIL_RTABMAP_MODE=localization" ;;
    esac
    check_net
    lock=$(rsh "$HIL_MAIN_HOST" "cat $(q "$HIL_LOCK") 2>/dev/null" || true)
    case "$lock" in
        "HIL on since "*) ;;
        *) die "restart: HIL не включён (на $HIL_MAIN_HOST нет замка $HIL_LOCK) — сначала hil.sh on" ;;
    esac
    HIL_MAPS_DIR=$(printf '%s\n' "$lock" | sed -n 's/.* maps=\([^ ]*\).*/\1/p' | head -n 1)
    [ -n "$HIL_MAPS_DIR" ] || die "restart: в замке нет maps= — каталог карты HIL не известен"
    check_hil_paths
    rsh "$HIL_MAIN_HOST" "test -f $(q "$HIL_REMOTE_DIR/main.images.yaml")" \
        || die "restart: нет $HIL_REMOTE_DIR/main.images.yaml (образы закрепляет on) — hil.sh off и on"
    push_helpers
    log "Main: restart $* — stop (до $HIL_RESTART_STOP_S с), up: карта $HIL_MAPS_DIR, rtabmap $HIL_RTABMAP_MODE"
    rsh "$HIL_MAIN_HOST" "docker stop --time $(q "$HIL_RESTART_STOP_S") $*" >/dev/null || die "Main: docker stop $* не прошёл"
    main_sim_up --force-recreate "$@" || die "Main: пересоздание $* не прошло; hil.sh status"
    rpi "$HIL_MAIN_HOST" state "$@"
}

cmd_off() {
    check_hil_off_owner
    check_net
    check_hil_registry
    local host
    for host in "$HIL_MAIN_HOST" "$HIL_VISION_HOST" "$HIL_SOUND_HOST" "$HIL_SANDBOX_HOST"; do
        rsh "$host" true || die "нет ssh до $host"
    done
    resolve_robot_id
    e2e_guard warn
    push_helpers
    # Сначала сим: иначе вернувшийся ros2-control встретит второй /controller_manager.
    sim_down
    local gz
    gz=$(sim_gazebo_state)
    case "$gz" in
        absent*) ;;
        *) die "gazebo на песочнице не погас ($gz) — робота не возвращаю" ;;
    esac

    log "Main: возвращаю ${MAIN_SIM_TIME[*]} ${MAIN_STOP[*]} (боевое окружение, те же образы)"
    rpi "$HIL_MAIN_HOST" pin "$HIL_MAIN_DIR" "$HIL_REMOTE_DIR/main.images.yaml"
    main_rm_created_sim
    rpi "$HIL_MAIN_HOST" compose "$HIL_MAIN_DIR" -f docker-compose.yaml -f "$HIL_REMOTE_DIR/main.images.yaml" \
        up -d --no-deps --pull never "${MAIN_SIM_TIME[@]}" "${MAIN_STOP[@]}"
    log "Vision: возвращаю ${VISION_STOP[*]}"
    rpi "$HIL_VISION_HOST" pin "$HIL_VISION_DIR" "$HIL_REMOTE_DIR/vision.images.yaml"
    rpi "$HIL_VISION_HOST" compose "$HIL_VISION_DIR" -f docker-compose.yaml -f "$HIL_REMOTE_DIR/vision.images.yaml" \
        up -d --no-deps --pull never "${VISION_STOP[@]}"
    log "Sound: возвращаю ${SOUND_SWITCH[*]} на боевой реестр карт и точек"
    rpi "$HIL_SOUND_HOST" pin "$HIL_SOUND_DIR" "$HIL_REMOTE_DIR/sound.images.yaml"
    rpi "$HIL_SOUND_HOST" compose "$HIL_SOUND_DIR" -f docker-compose.yaml -f "$HIL_REMOTE_DIR/sound.images.yaml" \
        up -d --no-deps --pull never "${SOUND_SWITCH[@]}"

    # Каждый сервис: работает и без sim time. Строк должно прийти ровно столько, сколько сервисов.
    # Нерабочий сервис, который не работал и до on (PRE_STATE), — предупреждение, не провал.
    local bad=0 warn=0 line n=0 main_state vision_state sound_state wpdb name st verdict
    PRE_STATE=$(rsh "$HIL_MAIN_HOST" "cat $(q "$HIL_PRE_STATE") 2>/dev/null" || true)
    if [ -n "$PRE_STATE" ]; then
        log "состояние до on (из $HIL_MAIN_HOST:$HIL_PRE_STATE):"
        printf '%s
' "$PRE_STATE" | sed 's/^/    до on  /'
    else
        log "⚠️  $HIL_MAIN_HOST:$HIL_PRE_STATE нет (on не доходил до записи?) — все сервисы ждём работающими"
    fi
    main_state=$(rpi "$HIL_MAIN_HOST" state "${MAIN_SIM_TIME[@]}" "${MAIN_STOP[@]}") || bad=1
    vision_state=$(rpi "$HIL_VISION_HOST" state "${VISION_STOP[@]}") || bad=1
    sound_state=$(rpi "$HIL_SOUND_HOST" state "${SOUND_SWITCH[@]}") || bad=1
    wpdb=$(rpi "$HIL_SOUND_HOST" env "${SOUND_SWITCH[0]}" WAYPOINT_DB_PATH) || bad=1
    # check_line <метка> <строка state> <running-проверка: sim|wpdb|none>
    check_line() {
        local label="$1" l="$2" kind="$3"
        read -r name st _ <<<"$l"
        n=$((n + 1))
        verdict=$(svc_verdict "$name" "$st")
        case "$verdict" in
            ok)
                echo "  $label $l"
                case "$kind" in
                    sim) case "$l" in *" running USE_SIM_TIME=unset" | *" running USE_SIM_TIME=false") ;; *) bad=1 ;; esac ;;
                    wpdb) [ "$wpdb" = unset ] || bad=1 ;;
                esac ;;
            warn)
                warn=$((warn + 1))
                echo "  $label $l  ⚠️  до on было: $(pre_status_of "$name"), не работает и сейчас — не провал off" ;;
            *)
                bad=1
                echo "  $label $l  ❌ до on работал (running), после off — $st" ;;
        esac
    }
    while read -r line; do
        [ -n "$line" ] || continue
        check_line "Sound " "$line WAYPOINT_DB_PATH=$wpdb" wpdb
    done <<<"$sound_state"
    while read -r line; do
        [ -n "$line" ] || continue
        check_line "Main  " "$line" sim
    done <<<"$main_state"
    while read -r line; do
        [ -n "$line" ] || continue
        check_line "Vision" "$line" none
    done <<<"$vision_state"
    [ "$n" -eq $(( ${#MAIN_SIM_TIME[@]} + ${#MAIN_STOP[@]} + ${#VISION_STOP[@]} + ${#SOUND_SWITCH[@]} )) ] || bad=1

    # Сверка реестра — всегда, даже если сервис не вернулся: боевой maps читаем только на чтение.
    local reg=0
    sound_verify_maps || reg=$?
    if [ "$bad" != 0 ]; then
        die "не всё вернулось (см. выше: ❌) — замок $HIL_LOCK оставлен"
    fi
    check_hil_off_owner
    rsh "$HIL_MAIN_HOST" "rm -f -- $(q "$HIL_PRE_STATE")" || true
    rpi "$HIL_MAIN_HOST" lock-release "$HIL_LOCK"
    local tail_note=""
    [ "$warn" = 0 ] || tail_note="; ⚠️  сервисов нерабочих и до on: $warn (см. ⚠️ выше, чинить отдельно)"
    case "$reg" in
        0) log "✅ HIL выключен, робот в боевом режиме$tail_note" ;;
        3) printf '❌ %s
' "HIL выключен, робот в боевом режиме$tail_note, НО боевой реестр карт изменился за HIL (diff выше)" >&2
           exit 3 ;;
        *) log "✅ HIL выключен, робот в боевом режиме$tail_note; ⚠️  боевой реестр карт НЕ сверен (см. выше)" ;;
    esac
}

# Статус Pi: сначала отличаем «нет ssh» от «нет helper'а», helper доставляем сами (как on/off).
status_pi() {  # status_pi <host> <контейнеры>...
    local host="$1"; shift
    if ! rsh "$host" true 2>/dev/null; then
        echo "  нет ssh до $host"
        return 0
    fi
    if ! rsh "$host" "test -f $(q "$REMOTE_PI")" 2>/dev/null; then
        rput "$host" "$HERE/remote_pi.sh" "$REMOTE_PI" || { echo "  helper не доставлен на $host (ssh есть, rput не удался)"; return 0; }
    fi
    rpi "$host" state "$@" || echo "  helper на $host вернул ошибку (ssh есть)"
}

cmd_status() {
    check_net
    echo "── замок ($HIL_MAIN_HOST:$HIL_LOCK)"
    rsh "$HIL_MAIN_HOST" "cat $(q "$HIL_LOCK") 2>/dev/null || echo нет" || echo "  нет ssh"
    echo "── Main Pi"
    status_pi "$HIL_MAIN_HOST" "${MAIN_STOP[@]}" "${MAIN_SIM_TIME[@]}"
    echo "── Vision Pi"
    status_pi "$HIL_VISION_HOST" "${VISION_STOP[@]}" vision-face
    echo "── Sound Pi (реестр карт и точек: WAYPOINT_DB_PATH, unset = боевой)"
    status_pi "$HIL_SOUND_HOST" "${SOUND_SWITCH[@]}"
    if rsh "$HIL_SOUND_HOST" "test -f $(q "$REMOTE_PI")" 2>/dev/null; then
        echo "  WAYPOINT_DB_PATH=$(rpi "$HIL_SOUND_HOST" env "${SOUND_SWITCH[0]}" WAYPOINT_DB_PATH || echo '?')"
    fi
    echo "── песочница ($HIL_SANDBOX_HOST, проект $HIL_SIM_PROJECT)"
    rsh "$HIL_SANDBOX_HOST" "docker ps -a --filter label=com.docker.compose.project=$(q "$HIL_SIM_PROJECT") --format '{{.Names}} {{.Status}}'" || echo "  нет ssh"
    local gz
    gz=$(sim_gazebo_state)
    case "$gz" in
        running*)
            echo "── /controller_manager в графе (сторож в gazebo): $(sim_cm_count) (норма HIL — 1)"
            echo "── $CMD_VEL_UNSTAMPED подписчиков: $(sim_cmd_vel_subs) (норма HIL — 1)" ;;
        *) echo "── gazebo: $gz" ;;
    esac
}

case "${1:-}" in
    on) cmd_on ;;
    off) cmd_off ;;
    status) cmd_status ;;
    restart) shift; cmd_restart "$@" ;;
    *) sed -n '2,8p' "$0"; exit 2 ;;
esac
