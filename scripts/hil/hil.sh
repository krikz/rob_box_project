#!/usr/bin/env bash
# HIL: стек настоящего робота на данных Gazebo с песочницы (эпик #4371, шаг 1 — навигация).
# План и порядок запуска: docs/research/2026-11-04-hil-robot-on-gazebo-data.md §8.
#
#   scripts/hil/hil.sh status   что сейчас: замок, контейнеры, sim time, /controller_manager
#   scripts/hil/hil.sh on       включить HIL (робот на кирпичах!)
#   scripts/hil/hil.sh off      вернуть робота как было (docker compose up -d)
#
# on:
#   1. проверки: ssh до четырёх машин, настройки, пути карт и реестра (не боевые), образ сима;
#   2. замок HIL_LOCK (/tmp/hil.lock) на Main Pi — деплой его видит и не идёт;
#   3. Main: docker stop ros2-control lslidar — моторы (CAN → VESC) получают команды
#      только от ros2-control; Vision: docker stop oak-d ceiling-camera;
#   3b. Sound: снапшот боевого `SELECT * FROM maps` (только чтение) в HIL_MAPS_SNAPSHOT;
#      voice-assistant пересоздаётся с docker/sound/compose.hil.yaml — реестр карт и точек
#      (maps/waypoints) в отдельном файле HIL_WAYPOINT_DB, остальная память боевая (#4392);
#   4. Main: robot-state-publisher, twist-mux, proximity-guard, rtabmap, imu-filter, nav2 пересоздаются
#      с USE_SIM_TIME=true и картой HIL_MAPS_DIR (docker/main/compose.hil.yaml, --no-deps);
#   5. песочница: zenoh-router + gazebo из docker/sim/compose.hil.yaml под ID робота.
#      В контейнере gazebo сторож (docker/sim/hil_run.sh): при живом /controller_manager
#      робота Gazebo не стартует, второй контроллер позже — Gazebo гасится;
#   6. ждём: в графе ровно один /controller_manager, контейнер ros2-control не работает —
#      значит, единственный контроллер — Gazebo. Иначе сим гасится, робот остаётся без моторов.
# off: сим гасится первым, затем на Main, Vision и Sound `docker compose up -d --no-deps` тех же
#   сервисов с теми же образами (закреплены по работающим контейнерам), замок снимается;
#   последним — сверка боевого `SELECT * FROM maps` со снапшотом on: изменился — выход 3.
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
MAIN_SIM_TIME=(robot-state-publisher twist-mux proximity-guard rtabmap imu-filter nav2)
SIM_SERVICES=(zenoh-router gazebo)
# Сервис и контейнер Sound Pi с mcp_server — владельцем реестра карт и точек (WaypointStore).
SOUND_SWITCH=(voice-assistant)

log() { printf '[hil %s] %s\n' "$(date -u +%H:%M:%SZ)" "$*"; }
die() { printf '❌ %s\n' "$*" >&2; exit 1; }

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
# Число /controller_manager в графе — глазами сторожа внутри gazebo; «-» — gazebo не работает.
sim_cm_count() {
    rsh "$HIL_SANDBOX_HOST" "docker exec $(q "$HIL_SIM_PREFIX-gazebo") /entrypoint.sh python3 -m rob_box_sim.hil count --settle 3 2>/dev/null | grep -E '^[0-9]+$' | tail -n 1 | grep . || echo -" || echo -
}
sim_down() {
    log "песочница: гашу zenoh-router и gazebo ($HIL_SIM_PROJECT)"
    sim_compose down --timeout 20 || log "⚠️  compose down на песочнице вернул ошибку"
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

cmd_on() {
    preflight
    push_helpers
    local who
    who="HIL on since $(date -u +%FT%TZ) by ${USER:-${USERNAME:-?}}@$(hostname) sandbox=$HIL_SANDBOX_IP maps=$HIL_MAPS_DIR"
    rpi "$HIL_MAIN_HOST" lock-take "$HIL_LOCK" "$who" || die "HIL уже включён или замок остался: scripts/hil/hil.sh status / off"

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
    rsh "$HIL_MAIN_HOST" "cd $(q "$HIL_MAIN_DIR") && HIL_MAPS_DIR=$(q "$HIL_MAPS_DIR") HIL_RTABMAP_MODE=$(q "$HIL_RTABMAP_MODE") HIL_GUARD_SOUND=$(q "$HIL_GUARD_SOUND") docker compose -f docker-compose.yaml -f $(q "$HIL_REMOTE_DIR/main.images.yaml") -f $(q "$HIL_REMOTE_DIR/main.hil.yaml") up -d --no-deps --pull never ${MAIN_SIM_TIME[*]}" \
        || die "Main: пересоздание с sim time не прошло; hil.sh off"

    # Перед стартом Gazebo: контроллер робота точно не работает.
    local st
    st=$(rpi "$HIL_MAIN_HOST" state ros2-control)
    case "$st" in
        *" running "*) die "ros2-control снова работает ($st) — Gazebo не поднимаю; hil.sh off" ;;
    esac

    log "песочница: поднимаю ${SIM_SERVICES[*]} (мир $HIL_SIM_WORLD)"
    sim_compose up -d --no-build "${SIM_SERVICES[@]}" || { sim_down; die "песочница: compose up не прошёл; робот без моторов, hil.sh off"; }

    log "жду ровно один /controller_manager (Gazebo), до $HIL_WAIT_CM_S с"
    local deadline=$(( $(date +%s) + HIL_WAIT_CM_S )) cm gz
    while :; do
        gz=$(sim_gazebo_state)
        case "$gz" in
            running*) ;;
            *)
                rsh "$HIL_SANDBOX_HOST" "docker logs --tail 30 $(q "$HIL_SIM_PREFIX-gazebo")" || true
                sim_down
                die "gazebo не работает ($gz): сторож отказал или Gazebo упал — см. лог выше; hil.sh off" ;;
        esac
        cm=$(sim_cm_count)
        st=$(rpi "$HIL_MAIN_HOST" state ros2-control)
        case "$st" in
            *" running "*) sim_down; die "ros2-control робота работает ($st) — Gazebo погашен; hil.sh off" ;;
        esac
        if [ "$cm" = 1 ]; then
            log "✅ /controller_manager ровно один, ros2-control робота: ${st#ros2-control }"
            break
        fi
        if [ "$cm" != 0 ] && [ "$cm" != - ]; then
            sim_down
            die "/controller_manager в графе: $cm (ждали 1) — Gazebo погашен; hil.sh off"
        fi
        [ "$(date +%s)" -lt "$deadline" ] || { sim_down; die "за $HIL_WAIT_CM_S с контроллер Gazebo не появился; hil.sh off"; }
        sleep 5
    done
    log "HIL включён. Сторож в gazebo следит дальше (docker logs $HIL_SIM_PREFIX-gazebo | grep hil-guard)."
    cmd_status
}

cmd_off() {
    check_net
    check_hil_registry
    local host
    for host in "$HIL_MAIN_HOST" "$HIL_VISION_HOST" "$HIL_SOUND_HOST" "$HIL_SANDBOX_HOST"; do
        rsh "$host" true || die "нет ssh до $host"
    done
    resolve_robot_id
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
    local bad=0 line n=0 main_state vision_state sound_state wpdb
    main_state=$(rpi "$HIL_MAIN_HOST" state "${MAIN_SIM_TIME[@]}" "${MAIN_STOP[@]}") || bad=1
    vision_state=$(rpi "$HIL_VISION_HOST" state "${VISION_STOP[@]}") || bad=1
    sound_state=$(rpi "$HIL_SOUND_HOST" state "${SOUND_SWITCH[@]}") || bad=1
    wpdb=$(rpi "$HIL_SOUND_HOST" env "${SOUND_SWITCH[0]}" WAYPOINT_DB_PATH) || bad=1
    while read -r line; do
        [ -n "$line" ] || continue
        n=$((n + 1))
        echo "  Sound  $line WAYPOINT_DB_PATH=$wpdb"
        case "$line" in *" running "*) ;; *) bad=1 ;; esac
        [ "$wpdb" = unset ] || bad=1
    done <<<"$sound_state"
    while read -r line; do
        [ -n "$line" ] || continue
        n=$((n + 1))
        echo "  Main   $line"
        case "$line" in
            *" running USE_SIM_TIME=unset" | *" running USE_SIM_TIME=false") ;;
            *) bad=1 ;;
        esac
    done <<<"$main_state"
    while read -r line; do
        [ -n "$line" ] || continue
        n=$((n + 1))
        echo "  Vision $line"
        case "$line" in *" running "*) ;; *) bad=1 ;; esac
    done <<<"$vision_state"
    [ "$n" -eq $(( ${#MAIN_SIM_TIME[@]} + ${#MAIN_STOP[@]} + ${#VISION_STOP[@]} + ${#SOUND_SWITCH[@]} )) ] || bad=1
    [ "$bad" = 0 ] || die "не всё вернулось (см. выше) — замок $HIL_LOCK оставлен"

    # Сверка до снятия замка: результат в лог, но робот уже вернулся — замок снимается в любом случае.
    local reg=0
    sound_verify_maps || reg=$?
    rpi "$HIL_MAIN_HOST" lock-release "$HIL_LOCK"
    case "$reg" in
        0) log "✅ HIL выключен, робот в боевом режиме" ;;
        3) printf '❌ %s\n' "HIL выключен, робот в боевом режиме, НО боевой реестр карт изменился за HIL (diff выше)" >&2
           exit 3 ;;
        *) log "✅ HIL выключен, робот в боевом режиме; ⚠️  боевой реестр карт НЕ сверен (см. выше)" ;;
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
        running*) echo "── /controller_manager в графе (сторож в gazebo): $(sim_cm_count) (норма HIL — 1)" ;;
        *) echo "── gazebo: $gz" ;;
    esac
}

case "${1:-}" in
    on) cmd_on ;;
    off) cmd_off ;;
    status) cmd_status ;;
    *) sed -n '2,7p' "$0"; exit 2 ;;
esac
