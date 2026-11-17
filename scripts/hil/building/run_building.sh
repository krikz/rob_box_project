#!/usr/bin/env bash
# Прогон «большая карта здания» в HIL (#4468, шаг 2): картография этажа, маршрут по кабинетам по сохранённой карте,
# локализация (рестарт стека, случайные старты, похищения), по желанию — голосом. Runbook: docs/runbooks/hil-building.md.
#
#   BUILDING_OWNER=<первое слово stand.lock> scripts/hil/building/run_building.sh full    prep + hil.sh on + all + off
#   scripts/hil/building/run_building.sh plan          порядок объезда, старты, похищения, оценка времени (без стенда)
#   scripts/hil/building/run_building.sh prep          копия частей репо на песочнице (без Depot), CRLF
#   BUILDING_OWNER=… scripts/hil/building/run_building.sh all    map route loc [voice] report — HIL уже включён
#   BUILDING_OWNER=… scripts/hil/building/run_building.sh map|route|loc|voice   один блок
#   scripts/hil/building/run_building.sh stop          остановить рекордер поз и скачать raw (после map|route|loc)
#   scripts/hil/building/run_building.sh report [OUT]  вердикты и анимации по скачанному каталогу
#
# Что решает код (rob_box_sim.building_run): порядок целей, старты, похищения и вердикты «доехал / нашёл себя /
# ложный скачок». Скрипт только исполняет шаги и собирает raw: шаги в контейнере gazebo песочницы
# (scripts/hil/video/building_step.py через building_remote.sh), рестарт стека — hil.sh restart на Main Pi,
# карта и точки в HIL-реестре — тулы start_mapping / finish_mapping / save_waypoint напрямую, мимо LLM
# (scripts/music/live_check_mcp_call.py, sender harness) на Sound Pi.
#
# Предусловия проверяет сам (выход 3): stand.lock начинается с BUILDING_OWNER, HIL включён и /controller_manager
# один (scripts/hil/checks/verdicts.py preflight), мир gazebo — building, у map — rtabmap в mapping.
# Пароли не используются: ssh -o BatchMode=yes, ключ в authorized_keys.
set -euo pipefail
export MSYS_NO_PATHCONV=1

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$HERE/../../.." && pwd)"
if [ -n "${HIL_CONF:-}" ]; then
    # shellcheck source=/dev/null
    . "$HIL_CONF"
fi
# shellcheck source=scripts/hil/hil.conf
. "$REPO_ROOT/scripts/hil/hil.conf"

: "${BUILDING_OWNER:=}"
: "${STAND_LOCK:=C:/Users/krikz/AppData/Local/Temp/claude/a31bd7f7b8b42df0/3f238318-a4ed-402f-88a9-a5003be4590b/scratchpad/stand.lock}"
: "${BUILDING_OUT:=${TMPDIR:-/tmp}/hil_building/$(date -u +%Y%m%dT%H%M%SZ)}"
: "${BUILDING_SANDBOX_DIR:=/home/ros2/rob_box_hilbuilding}"
: "${BUILDING_STARTS:=6}"
: "${BUILDING_SEED:=4468}"
: "${BUILDING_REGISTRY:=1}"            # 1 — карта и точки-кабинеты в HIL-реестре тулами; 0 — без реестра
: "${BUILDING_MAP_NAME:=здание}"
: "${BUILDING_RESTART:=rtabmap}"       # что перезапускать как «рестарт стека» (сервисы контура HIL)
: "${BUILDING_VOICE:=0}"               # 1 — голосовой сценарий (нужен BUILDING_REGISTRY=1 на route)
: "${BUILDING_VOICE_TARGETS:=dean=Робот, поезжай в деканат.|canteen=Робот, поезжай в столовую.|room_205=Робот, поезжай в аудиторию двести пять.}"
: "${BUILDING_STEP_TIMEOUT_S:=7200}"   # потолок ожидания одного шага, с стены
: "${PY:=python}"

OUT="$BUILDING_OUT"
D="$BUILDING_SANDBOX_DIR"
GZ="$HIL_SIM_PREFIX-gazebo"
SSH_OPTS=(-o BatchMode=yes -o ConnectTimeout=10 -o ServerAliveInterval=5 -o ServerAliveCountMax=3)
LABEL_N=0

log() { printf '[building %s] %s\n' "$(date -u +%H:%M:%SZ)" "$*" | tee -a "$OUT/run.log" >&2; }
die() { printf '❌ %s\n' "$*" | tee -a "$OUT/run.log" >&2; exit 3; }
rsh() {  # rsh <host> <команда>
    local host="$1"; shift
    ssh "${SSH_OPTS[@]}" "$host" "$*" </dev/null
}
pypath() { if command -v cygpath >/dev/null 2>&1; then cygpath -m "$1"; else printf '%s' "$1"; fi; }
PYSEP=":"
if command -v cygpath >/dev/null 2>&1; then PYSEP=";"; fi  # Windows-python: разделитель PYTHONPATH — «;»
br() {  # br <аргументы rob_box_sim.building_run>: план и вердикты считает код
    PYTHONPATH="$(pypath "$REPO_ROOT/src/rob_box_sim")${PYTHONPATH:+$PYSEP$PYTHONPATH}" "$PY" -X utf8 \
        -m rob_box_sim.building_run "$@"
}
remote() { rsh "$HIL_SANDBOX_HOST" "BUILDING_DIR=$D G=$GZ bash $D/building_remote.sh $*"; }

# step <метка> <аргументы building_step.py>: шаг в фоне на песочнице (обрыв ssh его не убивает), ждём .rc
step() {
    local label t0=$SECONDS t_end rc
    LABEL_N=$((LABEL_N + 1))
    label=$(printf '%03d_%s' "$LABEL_N" "$1")
    shift
    remote start "$label" "$@" >>"$OUT/run.log" 2>&1 || die "шаг $label не запустился на песочнице"
    t_end=$((SECONDS + BUILDING_STEP_TIMEOUT_S))
    while [ "$SECONDS" -lt "$t_end" ]; do
        rc=$(rsh "$HIL_SANDBOX_HOST" "cat $D/out/building/steps/$label.rc 2>/dev/null" || true)
        if [ -n "$rc" ]; then
            [ "$rc" = 0 ] || log "⚠️  шаг $label: код $rc (лог out/building/steps/$label.log)"
            return 0
        fi
        sleep $((SECONDS - t0 < 60 ? 2 : 10))  # метки и телепорт — секунды, объезд — десятки минут
    done
    log "⚠️  шаг $label: нет итога за $BUILDING_STEP_TIMEOUT_S с — гашу building_step.py"
    rsh "$HIL_SANDBOX_HOST" "docker exec $GZ pkill -INT -f /building_step.py" || true
}
mark() { step mark mark "$@"; }
seg_start() { mark SEG_START "name=$1" "kind=$2" "${@:3}"; }
seg_end() { mark SEG_END "name=$1"; }

# tool <имя> <json параметров>: тул MCP мимо LLM (подпись harness), ответ одной строкой JSON → tools.log
tool() {
    local b64
    b64=$(printf '%s' "$2" | base64 | tr -d '\n')
    {
        printf '%s TOOL %s %s\n' "$(date -u +%FT%TZ)" "$1" "$2"
        ssh "${SSH_OPTS[@]}" "$HIL_SOUND_HOST" "docker exec -i voice-assistant bash -lc 'source /opt/ros/humble/setup.bash; source /ws/install/setup.bash; python3 - $1 $b64 90'" \
            <"$REPO_ROOT/scripts/music/live_check_mcp_call.py" 2>&1 || echo "TOOL_FAILED $1 rc=$?"
    } >>"$OUT/tools.log"
}

restart_stack() {  # restart_stack <метка>: рестарт стека в режиме локализации, метки по sim time
    mark restart_begin "seg=$1"
    HIL_RTABMAP_MODE=localization bash "$REPO_ROOT/scripts/hil/hil.sh" restart $BUILDING_RESTART >>"$OUT/restart.log" 2>&1 \
        || log "⚠️  hil.sh restart $BUILDING_RESTART: ошибка (restart.log)"
    mark restart_end "seg=$1"
}

fetch() {
    mkdir -p "$OUT"
    rsh "$HIL_SANDBOX_HOST" "BUILDING_DIR=$D G=$GZ bash $D/building_remote.sh pack" | tar xf - -C "$OUT" \
        || log "⚠️  не скачан out/building с песочницы"
    if [ -d "$OUT/container" ]; then
        cp -r "$OUT/container/." "$OUT/" && rm -rf "$OUT/container"
    fi
}

# ── предусловия ─────────────────────────────────────────────────────────────────────────────

preflight() {  # preflight <ждём режим rtabmap: mapping|localization|any>
    local d="$OUT/preflight" fmt line world mode
    need_loc_metrics
    mkdir -p "$d"
    if [ -f "$STAND_LOCK" ]; then cat "$STAND_LOCK" >"$d/stand_lock.txt"; else : >"$d/stand_lock.txt"; fi
    bash "$REPO_ROOT/scripts/hil/hil.sh" status >"$d/hil_status.txt" 2>&1 || true
    fmt='{{.Name}} {{.State.Status}} {{if .State.Health}}{{.State.Health.Status}}{{else}}none{{end}} {{.RestartCount}} {{.State.StartedAt}}'
    {
        rsh "$HIL_MAIN_HOST" "docker inspect -f '$fmt' nav2 twist-mux proximity-guard rtabmap 2>&1"
        rsh "$HIL_SOUND_HOST" "docker inspect -f '$fmt' voice-assistant 2>&1"
        rsh "$HIL_SANDBOX_HOST" "docker inspect -f '$fmt' $GZ 2>&1"
    } >"$d/inspect.txt" || true
    {
        echo "local $(date +%s.%N)"
        for host in "$HIL_MAIN_HOST" "$HIL_SOUND_HOST" "$HIL_SANDBOX_HOST"; do
            echo "$host $(rsh "$host" 'date +%s.%N' || echo nan)"
        done
    } >"$d/clock.txt"
    line=$("$PY" -X utf8 "$(pypath "$REPO_ROOT/scripts/hil/checks/verdicts.py")" preflight "$(pypath "$d")" \
        --owner "$BUILDING_OWNER" --need "nav2 twist-mux proximity-guard rtabmap voice-assistant $GZ" || true)
    echo "$line" | tee -a "$OUT/run.log"
    case "$line" in *" preflight PASS:"*) ;; *) die "предусловия не выполнены — raw в $d" ;; esac
    world=$(rsh "$HIL_SANDBOX_HOST" "docker inspect -f '{{join .Args \" \"}}' $GZ" || true)
    echo "gazebo args: $world" >"$d/world.txt"
    case "$world" in *"/building.sdf"*) ;; *) die "в gazebo не мир building ($world): HIL_SIM_WORLD=building hil.sh on" ;; esac
    mode=$(rsh "$HIL_MAIN_HOST" "docker inspect -f '{{range .Config.Env}}{{println .}}{{end}}' rtabmap" \
        | sed -n 's/^RTABMAP_START_MODE=//p' | tail -n 1 || true)
    echo "rtabmap: RTABMAP_START_MODE=$mode" >>"$d/world.txt"
    if [ "$1" != any ] && [ "$mode" != "$1" ]; then
        die "rtabmap в режиме '$mode', блоку нужен '$1' (картография — со свежего HIL_MAPS_DIR и HIL_RTABMAP_MODE=mapping)"
    fi
    remote push >>"$OUT/run.log" 2>&1 || die "не доставлены шаги в $GZ (сначала run_building.sh prep)"
}

# ── блоки ───────────────────────────────────────────────────────────────────────────────────

block_map() {
    local tour
    tour=$(br tour)
    log "картография: объезд ($(wc -w <<<"$tour") целей): $tour"
    remote maprec_start maprec_map 1
    if [ "$BUILDING_REGISTRY" = 1 ]; then tool start_mapping "{\"map_name\": \"$BUILDING_MAP_NAME\"}"; fi
    seg_start map map
    # shellcheck disable=SC2086
    step explore goto $tour
    seg_end map
    if [ "$BUILDING_REGISTRY" = 1 ]; then tool finish_mapping "{\"map_name\": \"$BUILDING_MAP_NAME\"}"; fi
    remote maprec_stop
    # переход в локализацию — первый «рестарт стека»: база пишется на остановке, rtabmap встаёт по сохранённой карте
    remote maprec_start maprec_loc 0
    seg_start restart_after_map restart
    restart_stack restart_after_map
    step probe probe
    seg_end restart_after_map
    rsh "$HIL_MAIN_HOST" "ls -l --time-style=+%FT%T \$(sed -n 's/.* maps=\\([^ ]*\\).*/\\1/p' $HIL_LOCK)" >"$OUT/map_files.txt" 2>&1 || true
    fetch
}

block_route() {
    local tour name label
    tour=$(br tour)
    log "маршрут по кабинетам по сохранённой карте (локализация)"
    remote maprec_start maprec_route 0
    seg_start route route
    for name in $tour; do
        step "goto_$name" goto "$name"
        if [ "$BUILDING_REGISTRY" = 1 ]; then
            label=$(br label "$name")
            tool save_waypoint "{\"name\": \"$label\"}"
        fi
    done
    seg_end route
    if [ "$BUILDING_REGISTRY" = 1 ]; then tool list_waypoints "{}"; fi
    fetch
}

block_loc() {
    local i x y yaw k
    log "локализация: рестарт на месте, $BUILDING_STARTS стартов из случайных точек, похищения"
    remote maprec_start maprec_locblock 0
    seg_start restart_here restart
    restart_stack restart_here
    step probe probe
    seg_end restart_here
    i=0
    while read -r x y yaw; do
        i=$((i + 1))
        seg_start "start_$i" start "x=$x" "y=$y" "yaw=$yaw"
        step teleport teleport "$x" "$y" "$yaw"
        restart_stack "start_$i"
        step probe probe
        step "goto_after_start_$i" goto "$(br nearest "$x" "$y")"
        seg_end "start_$i"
    done < <(br starts --starts "$BUILDING_STARTS" --seed "$BUILDING_SEED")
    for k in kidnap_similar kidnap_far; do
        seg_start "$k" kidnap
        step "$k" kidnap "$k"
        seg_end "$k"
    done
    fetch
}

block_voice() {
    local spec target phrase i=0
    [ "$BUILDING_VOICE" = 1 ] || { log "голосовой сценарий выключен (BUILDING_VOICE=0)"; return 0; }
    log "голос: реплики в диалог (inject.sh), доезд по правде Gazebo"
    IFS='|' read -r -a specs <<<"$BUILDING_VOICE_TARGETS"
    for spec in "${specs[@]}"; do
        i=$((i + 1))
        target="${spec%%=*}"
        phrase="${spec#*=}"
        seg_start "voice_$i" voice "target=$target"
        ROBOT="$HIL_SOUND_HOST" bash "$REPO_ROOT/scripts/robot_dev/inject.sh" "$phrase" >>"$OUT/inject.txt" 2>&1 || true
        step "arrive_$target" arrive "$target" --timeout 300
        seg_end "voice_$i"
    done
    rsh "$HIL_SOUND_HOST" "docker logs --since 2h voice-assistant 2>&1 | grep -E 'tts_node|navigate_to_waypoint|tool' | tail -200" >"$OUT/voice.log" || true
    fetch
}

block_report() {
    local dir="${1:-$OUT}"
    "$PY" -X utf8 "$(pypath "$HERE/building_report.py")" "$(pypath "$dir")" | tee -a "$dir/run.log"
}

cmd_prep() {
    local SSH=(ssh "${SSH_OPTS[@]}")
    cd "$REPO_ROOT"
    "${SSH[@]}" "$HIL_SANDBOX_HOST" "mkdir -p '$D/out/building' '$D/pylib'" </dev/null
    # части репо для docker/sim/compose.yaml (extends ../main/docker-compose.yaml, тома конфигов и миров) — как
    # sandbox_prep.sh шаг 1, но без Depot и без patch_compose.py: мир берётся из HIL_SIM_WORLD=building
    tar cf - docker/sim docker/main/config docker/main/docker-compose.yaml docker/common src/rob_box_sim/worlds \
        | "${SSH[@]}" "$HIL_SANDBOX_HOST" "tar xf - -C '$D'"
    tar cf - -C scripts/hil/video route.py building_step.py fix_crlf.sh | "${SSH[@]}" "$HIL_SANDBOX_HOST" "tar xf - -C '$D'"
    tar cf - -C scripts/hil/building building_remote.sh | "${SSH[@]}" "$HIL_SANDBOX_HOST" "tar xf - -C '$D'"
    tar cf - -C src/rob_box_sim --exclude=__pycache__ rob_box_sim | "${SSH[@]}" "$HIL_SANDBOX_HOST" "tar xf - -C '$D/pylib'"
    "${SSH[@]}" "$HIL_SANDBOX_HOST" "tr -d '\r' < '$D/fix_crlf.sh' | bash -s -- '$D'" </dev/null
    echo "готово: $D; дальше HIL_SANDBOX_REPO=$D HIL_SIM_WORLD=building HIL_RTABMAP_MODE=mapping HIL_MAPS_DIR=<свежий> hil.sh on"
}

need_loc_metrics() {
    [ -f "$REPO_ROOT/src/rob_box_sim/rob_box_sim/loc_metrics.py" ] \
        || die "нет rob_box_sim/loc_metrics.py (PR #4568, #4346): без него «нашёл себя» и скачки не посчитать"
}

cmd_all() {
    preflight mapping
    block_map
    preflight localization
    block_route
    block_loc
    block_voice
    remote maprec_stop
    fetch
    block_report
}

cmd_full() {
    need_loc_metrics
    cmd_prep
    export HIL_SANDBOX_REPO="$D" HIL_SIM_WORLD=building HIL_RTABMAP_MODE=mapping
    export HIL_MAPS_DIR="${HIL_MAPS_DIR_BUILDING:-/home/ros2/rob_box_hil/maps_building_$(date -u +%m%d_%H%M)}"
    log "hil.sh on: мир building, карта $HIL_MAPS_DIR (свежая), rtabmap mapping"
    bash "$REPO_ROOT/scripts/hil/hil.sh" on >"$OUT/hil_on.log" 2>&1 || die "hil.sh on не прошёл (hil_on.log) — hil.sh off"
    trap 'log "hil.sh off"; bash "$REPO_ROOT/scripts/hil/hil.sh" off >"$OUT/hil_off.log" 2>&1 || log "❌ hil.sh off: см. hil_off.log"' EXIT
    cmd_all
}

mkdir -p "$OUT"
case "${1:-}" in
    plan) br plan --starts "$BUILDING_STARTS" --seed "$BUILDING_SEED" --voice "$([ "$BUILDING_VOICE" = 1 ] && echo 3 || echo 0)" ;;
    prep) cmd_prep ;;
    all) cmd_all ;;
    full) cmd_full ;;
    map) preflight mapping; block_map ;;
    route) preflight localization; block_route ;;
    loc) preflight localization; block_loc ;;
    voice) preflight localization; BUILDING_VOICE=1 block_voice ;;
    stop) remote maprec_stop; fetch ;;
    report) block_report "${2:-$OUT}" ;;
    *) sed -n '2,12p' "$0"; exit 2 ;;
esac
log "каталог прогона: $OUT"
