#!/usr/bin/env bash
# Повторный одиночный прогон точки маршрута с расширенной записью (#4694).
# M1_REPEAT_GOAL — имя точки route.py (M1..M6, R1..R7), по умолчанию M1.
#
# Условия сопоставимости с прогоном 18.11 16:26Z (/tmp/hil_m1_stall):
#   Depot (HIL_EXPECT_WORLD=depot, гейт scripts/hil/hil.sh как есть),
#   спавн (0, 0, 0), цель M1 (-3.7, 4, 0°) по умолчанию, max_vel_x 0.15,
#   PathAlign.forward_point_distance 0.20 (боевой nav2_params.yaml, оверлей скорости не кладём).
#   Оверлей repeat только включает publish_evaluation/publish_trajectories/publish_local_plan:
#   DWB считает score до публикации, выбор vx/wz от этого не меняется. Боевой yaml не правится.
#
# До отправки цели на хост: sha SDF, ID образов, фактические параметры, старт map+GT,
# вердикт мира. Bag (в т.ч. /cmd_vel_yaw, global costmap, /cmd_vel_nav) и route.log
# копируются на хост ДО hil off. Живой прогон этим файлом не запускался в PR.
#
#   M1_REPEAT_OUT=/tmp/hil_m1_repeat bash scripts/hil/repeat/run_m1_repeat.sh
#   M1_REPEAT_OUT=/tmp/hil_r7 M1_REPEAT_GOAL=R7 bash scripts/hil/repeat/run_m1_repeat.sh
#
# Не берёт /tmp/hil.lock сам: замок берёт hil.sh on. Если замок уже есть — выход 20, стенд не трогаем.
set -uo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HIL_CMD="${HIL_CMD:-$ROOT/scripts/hil/hil.sh}"
WORLD_CHECK="${WORLD_CHECK:-$ROOT/scripts/hil/checks/world_check.py}"
TOPICS_PY="${TOPICS_PY:-$HERE/m1_topics.py}"
HIL_LOCK=$(
    if [ -n "${HIL_CONF:-}" ]; then
        # shellcheck source=/dev/null
        . "$HIL_CONF" >/dev/null || exit 2
    fi
    # shellcheck source=scripts/hil/hil.conf
    . "$ROOT/scripts/hil/hil.conf" || exit 2
    printf '%s' "$HIL_LOCK"
) || exit 2
# Одинаковые run_id или пути карты не дают второму запуску права на чужой замок.
HIL_RUN_TOKEN=$(python3 -c 'import uuid; print(uuid.uuid4().hex)') || exit 2
export HIL_RUN_TOKEN

: "${M1_REPEAT_OUT:?M1_REPEAT_OUT — каталог артефактов на хосте}"
MAIN="${HIL_MAIN_HOST:-ros2@10.1.1.10}"
SANDBOX="${HIL_SANDBOX_HOST:-ros2@10.1.1.250}"
SANDBOX_REPO="${HIL_SANDBOX_REPO:-/home/ros2/rob_box_hilvideo}"
G="${HIL_SIM_PREFIX:-rob-box-hil}-gazebo"
NAV2_CONTAINER="${M1_NAV2_CONTAINER:-nav2}"
RUN_ID="${M1_REPEAT_RUN_ID:-m1repeat_$(date -u +%Y%m%dT%H%M%SZ)}"
MAPS_DIR="${HIL_MAPS_DIR:-/home/ros2/rob_box_hil/maps_${RUN_ID}}"
CONTAINER_OUT="/sim_video/${RUN_ID}"
REMOTE_OUT="$SANDBOX_REPO/out/${RUN_ID}"
NAV_READY_TIMEOUT="${M1_NAV_READY_TIMEOUT:-180}"
BAG_CLOSE_WAIT="${M1_BAG_CLOSE_WAIT:-40}"
GOAL_PROBE_DIR="${M1_GOAL_PROBE_DIR:-}"
GOAL_PROBE_TARGET=/opt/ros/humble/lib/libcontroller_server_core.so
GOAL_PROBE_MOUNT="/tmp/hil/goal_check_${RUN_ID}.so"
# Цель прогона: имя точки route.py. То же имя уходит в runvideo.sh и в grep GOAL_END —
# вердикт чужой цели не пройдёт. Полный список имён проверяет route.py на песочнице.
GOAL="${M1_REPEAT_GOAL:-M1}"
[[ "$GOAL" =~ ^[MR][0-9]{1,2}$ ]] || {
    printf 'M1_REPEAT_GOAL=%s: ждали имя точки route.py (M1..M6, R1..R7)\n' "$GOAL" >&2
    exit 2
}

SSHO=(-o BatchMode=yes -o ConnectTimeout=10 -o ServerAliveInterval=5 -o ServerAliveCountMax=6)
ART="$M1_REPEAT_OUT"
mkdir -p "$ART"
SUMMARY="$ART/verdict.log"
: >"$SUMMARY"
LOCK_OWNED=0
CLEANED=1

# shellcheck disable=SC2029
main() { ssh "${SSHO[@]}" "$MAIN" "$*" </dev/null; }
# shellcheck disable=SC2029
sandbox() { ssh "${SSHO[@]}" "$SANDBOX" "$*" </dev/null; }
log() { printf '%s %s\n' "$(date -u +%FT%TZ)" "$*" | tee -a "$SUMMARY"; }

# ros2 CLI без timeout висит на zenoh scout (run2, 30 мин, lifecycle get /bt_navigator).
# 37 — CLI не вернулся за отведённые секунды. Не ждём NAV_READY_TIMEOUT по кругу.
ROS2_CLI_TIMEOUT="${M1_ROS2_CLI_TIMEOUT:-15}"

ros2_in_nav2() {
    local inner="$1"
    timeout "$ROS2_CLI_TIMEOUT" ssh "${SSHO[@]}" "$MAIN" \
        "docker exec $(printf '%q' "$NAV2_CONTAINER") /ros_scripts/ros_with_namespace.sh bash -c 'source /opt/ros/humble/setup.bash; timeout $ROS2_CLI_TIMEOUT $inner'" \
        </dev/null
}

lock_is_ours() {
    local lock
    lock=$(main "cat $(printf '%q' "$HIL_LOCK") 2>/dev/null") || return 1
    [[ "$lock" == "HIL on since "*" token=$HIL_RUN_TOKEN maps=$MAPS_DIR" ]]
}

validate_goal_probe() {
    local name unused
    [ "$YAW_MODE" = detach ] || { log "GOAL_PROBE_REQUIRES_DETACH"; return 1; }
    GOAL_PROBE_IMAGE=$(cat "$GOAL_PROBE_DIR/nav2_image_id") || return 1
    read -r GOAL_PROBE_SHA name unused <"$GOAL_PROBE_DIR/probe.sha256" || return 1
    [[ "$GOAL_PROBE_IMAGE" =~ ^sha256:[0-9a-f]{64}$ ]] || return 1
    [[ "$GOAL_PROBE_SHA" =~ ^[0-9a-f]{64}$ && "$name" = libcontroller_server_core.so && -z "$unused" ]] || return 1
    (cd "$GOAL_PROBE_DIR" && sha256sum --check --status probe.sha256)
}

verify_goal_probe_loaded() {
    local nav_image container
    container=$(printf '%q' "$NAV2_CONTAINER")
    nav_image=$(main "docker inspect -f '{{.Image}}' $container") || return 1
    [ "$nav_image" = "$GOAL_PROBE_IMAGE" ] || { log "GOAL_PROBE_IMAGE_MISMATCH $nav_image"; return 1; }
    main "docker exec $container sha256sum '$GOAL_PROBE_TARGET'" >"$ART/probe_loaded.sha256" || return 1
    grep -Fxq "$GOAL_PROBE_SHA  $GOAL_PROBE_TARGET" "$ART/probe_loaded.sha256" || return 1
    main "docker logs $container 2>&1" | grep -F 'HIL_GOAL_PROBE_READY v1' >"$ART/probe_ready.log"
}

# Один ros2 param get. Баннер ros_with_namespace.sh без «value is:» — пусто, зовём ещё раз.
param_get_once() {
    local pname="$1" raw value rc=0
    # timeout 15: run2 (0.005b) висел 30 мин, ros2 param get не вернулся. Нет числа — missing, не ждём.
    raw=$(ros2_in_nav2 "ros2 param get /controller_server $(printf '%q' "$pname")" 2>/dev/null) || rc=$?
    if [ "$rc" -eq 124 ]; then
        log "PARAM_GET_TIMEOUT $pname"
        printf '%s' ""
        return 0
    fi
    printf 'PARAM %s\n%s\n' "$pname" "$raw" >>"$ART/params_raw.txt"
    value=$(python3 -X utf8 -c 'import sys; sys.path.insert(0, sys.argv[1]); from m1_topics import parse_param_value; v = parse_param_value(sys.stdin.read()); print("" if v is None else v)' "$HERE" <<<"$raw")
    printf '%s' "$value"
}

# bag/ без trailing «/.»: OpenSSH 9 scp (SFTP) отвергает filename «.» (прогон 18.11, exit 50).
# Песочница = эта машина — cp, не scp. Удалённый хост: scp -r host:bag каталог, не bag/.
copy_bag_to_host() {
    local err="${ART}/artifact_scp.err"
    mkdir -p "$ART/bag"
    if pull_host_is_local "$SANDBOX"; then
        cp "$REMOTE_OUT/route.log" "$ART/route.log" 2>>"$err" || return 1
        cp -a "$REMOTE_OUT/bag/." "$ART/bag/" 2>>"$err" || return 1
        return 0
    fi
    scp -q -o BatchMode=yes "$SANDBOX:$REMOTE_OUT/route.log" "$ART/route.log" 2>>"$err" || return 1
    scp -q -o BatchMode=yes -r "$SANDBOX:$REMOTE_OUT/bag" "$ART/" 2>>"$err" || return 1
}

# Песочница на этой же машине: scp user@10.1.1.250 сам себе ловит тот же SFTP-отказ на «.».
pull_host_is_local() {
    case "$1" in
        "" | localhost | 127.0.0.1 | "::1") return 0 ;;
    esac
    local host="${1#*@}"
    host="${host%%:*}"
    [ "$host" = "localhost" ] || [ "$host" = "127.0.0.1" ] && return 0
    hostname -I 2>/dev/null | tr ' ' '\n' | grep -qx "$host"
}

topics_line() {
    python3 -X utf8 -c 'import sys; sys.path.insert(0, sys.argv[1]); from m1_topics import repeat_bag_topics; print(" ".join(repeat_bag_topics()))' "$HERE"
}

stop_route() {
    sandbox "docker exec $(printf '%q' "$G") pkill -INT -f 'python3 -u /route.py' 2>/dev/null || true" >/dev/null 2>&1 || true
}

# Закрыть bag и забрать артефакты на хост. Вызывается до hil off.
pull_before_off() {
    local rc=0 nav_log_rc=0
    sandbox "docker exec $(printf '%q' "$G") bash -c 'if test -f $(printf '%q' "$CONTAINER_OUT")/recorder.pid; then kill -INT \$(cat $(printf '%q' "$CONTAINER_OUT")/recorder.pid) 2>/dev/null || true; fi'" \
        >>"$ART/bag_stop.log" 2>&1 || true
    sandbox "for i in \$(seq 1 $BAG_CLOSE_WAIT); do test -f '$REMOTE_OUT/bag/metadata.yaml' && break; sleep 1; done; docker exec $(printf '%q' "$G") /entrypoint.sh ros2 bag info '$CONTAINER_OUT/bag'" \
        >"$ART/bag_info.txt" 2>&1 || rc=1
    # Только route этого run_id. Общий out/route.log — прошлый прогон
    # (yaw_std утащил GOAL 07:28 от clean_003). Нет своего файла — не копируем чужой.
    :
    copy_bag_to_host || rc=1
    # Лог nav2 до off: «Failed to make progress» и какой recovery (clear/spin/wait/back_up).
    main "docker logs $(printf '%q' "$NAV2_CONTAINER") 2>&1" >"$ART/nav2.log" 2>&1 || nav_log_rc=$?
    if [ -n "$GOAL_PROBE_DIR" ]; then
        if [ "$nav_log_rc" -ne 0 ]; then
            log "GOAL_CHECK_LOGS_UNREADABLE rc=$nav_log_rc"
            return 1
        fi
        if grep -F 'HIL_GOAL_CHECK_REACHED ' "$ART/nav2.log" >"$ART/goal_check_reached.log"; then
            log "GOAL_CHECK_TRACE_CAPTURED"
        elif grep -Fq "Reached the goal!" "$ART/nav2.log" || \
            grep -Eq "GOAL_END $GOAL result=(STATUS_SUCCEEDED|OUTSIDE_TOLERANCE)([[:space:]]|\$)" "$ART/route.log"; then
            log "GOAL_CHECK_TRACE_MISSING"
            return 1
        else
            log "GOAL_CHECK_TRACE_NO_REACH"
        fi
    fi
    python3 -X utf8 -c '
import pathlib, sys
sys.path.insert(0, sys.argv[1])
from m1_topics import recovery_hits
text = pathlib.Path(sys.argv[2]).read_text(encoding="utf-8", errors="replace")
hits = recovery_hits(text)
pathlib.Path(sys.argv[3]).write_text(
    ("hits=" + " ".join(hits) if hits else "hits=none") + "\n", encoding="utf-8")
' "$HERE" "$ART/nav2.log" "$ART/recovery.txt" || true
    if [ -s "$ART/route.log" ] && grep -q "GOAL_START" "$ART/route.log" && [ -s "$ART/bag_info.txt" ] && [ -f "$ART/bag/metadata.yaml" ] && [ -s "$ART/nav2.log" ]; then
        log "ARTIFACTS_ON_HOST route.log bag_info.txt bag/ nav2.log recovery.txt"
        return 0
    fi
    log "ARTIFACTS_INCOMPLETE rc=$rc"
    return 1
}

cleanup() {
    [ "$CLEANED" = 0 ] || return 0
    CLEANED=1
    local art_rc=0 off_rc=0
    if ! lock_is_ours; then
        log "OFF_SKIPPED_LOCK_NOT_OWNED"
        [ "$LOCK_OWNED" = 0 ] && return 0
        return 51
    fi
    stop_route
    stop_resource_samplers
    pull_before_off || art_rc=1
    if lock_is_ours; then
        # hil off пересоздаёт nav2 без временных оверлеев.
        main "rm -f /tmp/hil/nav2tune/dwb_publish.yaml /tmp/hil/nav2tune.compose.yaml /tmp/hil/twist_mux_hil.yaml /tmp/hil/twist_mux.hil.yaml" || true
        if [ -n "${GOAL_PROBE_DIR:-}" ]; then
            main "rm -f -- $(printf '%q' "$GOAL_PROBE_MOUNT") /tmp/hil/goal_check_probe.compose.yaml" || true
        fi
        bash "$HIL_CMD" off >"$ART/hil_off.log" 2>&1 || off_rc=$?
        if [ "$off_rc" = 0 ]; then
            main "test ! -e $(printf '%q' "$HIL_LOCK")" || off_rc=1
        fi
    else
        off_rc=1
    fi
    # off — безопасность стенда. Неполный bag её не маскирует и наоборот.
    if [ "$off_rc" -eq 0 ]; then
        log "OFF_OK"
    else
        log "OFF_FAILED rc=$off_rc"
    fi
    if [ "$art_rc" -eq 0 ]; then
        log "ARTIFACTS_OK"
    else
        log "ARTIFACTS_INCOMPLETE"
    fi
    if [ "$off_rc" -ne 0 ]; then
        return 51
    fi
    if [ "$art_rc" -ne 0 ]; then
        return 52
    fi
    log "CLEANUP_OK"
    return 0
}

on_exit() {
    local rc=$? cleanup_rc=0
    trap - EXIT INT TERM
    cleanup || cleanup_rc=$?
    # Прогон дошёл, off прошёл, не доехали только артефакты — не 50 (это был маскарад).
    if [ "$rc" -eq 0 ] && [ "$cleanup_rc" -ne 0 ]; then rc=$cleanup_rc; fi
    exit "$rc"
}
trap on_exit EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

# 1 Гц, лёгкий разбор /proc. Нет intel_gpu_top и iostat — поле missing, не падаем.
# cadvisor не читаем (даёт 0). nvidia-smi не зовём.
start_resource_samplers() {
    local gpu="" card
    for card in /sys/class/drm/card*/gt_cur_freq_mhz /sys/class/drm/card*/gt_busy_percent; do
        [ -r "$card" ] && { gpu="$card"; break; }
    done
    # Песочница — эта машина. Нет sysfs GPU — поле missing, не падаем.
    python3 -X utf8 "$HERE/resource_sample.py" --loop --host sandbox --out "$ART/sandbox_resources.log" \
        --disk sda ${gpu:+--gpu "$gpu"} >>"$ART/sandbox_sampler.err" 2>&1 &
    echo $! >"$ART/sandbox_sampler.pid"
    # Main: тот же разбор /proc, без docker stats (cadvisor врёт нулями, stats грузит демон).
    main "mkdir -p /tmp/hil" || true
    scp -q -o BatchMode=yes "$HERE/resource_sample.py" "$MAIN:/tmp/hil/resource_sample.py" || {
        log "MAIN_SAMPLER_SCP_FAIL"
        return 0
    }
    main "nohup python3 -X utf8 /tmp/hil/resource_sample.py --loop --host main --out '/tmp/hil/$RUN_ID.main_resources.log' --disk mmcblk0 >'/tmp/hil/$RUN_ID.main_sampler.err' 2>&1 & echo \$! > '/tmp/hil/$RUN_ID.main_sampler.pid'" \
        || log "MAIN_SAMPLER_START_FAIL"
    log "SAMPLERS_STARTED"
}

stop_resource_samplers() {
    if [ -f "$ART/sandbox_sampler.pid" ]; then
        kill "$(cat "$ART/sandbox_sampler.pid")" 2>/dev/null || true
    fi
    main "if test -f '/tmp/hil/$RUN_ID.main_sampler.pid'; then kill \$(cat '/tmp/hil/$RUN_ID.main_sampler.pid') 2>/dev/null || true; fi; cat '/tmp/hil/$RUN_ID.main_resources.log'" \
        >"$ART/main_resources.log" 2>/dev/null || true
    log "SAMPLERS_STOPPED"
}

# После active controller_server. Нет числа после двух попыток — name=missing, не баннер.
capture_params_after_ready() {
    local pname value attempt overlays
    local -a goal_probe_params=()
    if [ -n "$GOAL_PROBE_DIR" ]; then
        goal_probe_params=(general_goal_checker.yaw_goal_tolerance general_goal_checker.stateful)
    fi
    : >"$ART/params_raw.txt"
    : >"$ART/params_values.txt"
    for pname in \
        FollowPath.max_vel_x \
        FollowPath.PathAlign.forward_point_distance \
        progress_checker.movement_time_allowance \
        FollowPath.min_speed_xy \
        FollowPath.trans_stopped_velocity \
        general_goal_checker.xy_goal_tolerance \
        FollowPath.xy_goal_tolerance \
        FollowPath.RotateToGoal.scale \
        FollowPath.RotateToGoal.lookahead_time \
        FollowPath.Oscillation.scale \
        FollowPath.Oscillation.oscillation_reset_dist \
        FollowPath.Oscillation.oscillation_reset_angle \
        FollowPath.Oscillation.oscillation_reset_time \
        FollowPath.publish_evaluation \
        FollowPath.publish_trajectories \
        FollowPath.publish_local_plan \
        FollowPath.short_circuit_trajectory_evaluation "${goal_probe_params[@]}"
    do
        value=""
        for attempt in 1 2; do
            value=$(param_get_once "$pname")
            [ -n "$value" ] && break
            sleep 2
        done
        if [ -n "$value" ]; then
            printf '%s=%s\n' "$pname" "$value" >>"$ART/params_values.txt"
        else
            printf '%s=missing\n' "$pname" >>"$ART/params_values.txt"
            log "PARAM_MISSING $pname"
        fi
    done
    overlays=$(cat "$ART/params_overlays.txt" 2>/dev/null || true)
    {
        cat "$ART/params_values.txt"
        printf '%s\n' "$overlays"
        printf 'goal=(-3.7,4.0,0)\n'
        printf 'spawn=(0.0,0.0,0.0)\n'
        printf 'expect_world=depot\n'
        printf 'run_id=%s\n' "$RUN_ID"
    } >"$ART/params.txt"
    if grep -q '=missing$' "$ART/params_values.txt"; then
        return 1
    fi
    # Оверлей читается на configure. Если просили false, а узел ответил не false —
    # yaml лёг мимо (пара run1: True при файле false). Цель не шлём.
    if [ "${M1_REPEAT_SHORT_CIRCUIT:-true}" = false ]; then
        if ! grep -qx 'FollowPath.short_circuit_trajectory_evaluation=[Ff]alse' "$ART/params_values.txt"; then
            log "SHORT_CIRCUIT_NOT_APPLIED $(grep short_circuit "$ART/params_values.txt" || true)"
            return 1
        fi
        log "SHORT_CIRCUIT_APPLIED false"
    fi
    # Только FollowPath.xy_goal_tolerance. general_goal_checker не сверяем и не меняем.
    if [ -n "${M1_REPEAT_XY_GOAL_TOL:-}" ]; then
        got=$(grep -x 'FollowPath.xy_goal_tolerance=.*' "$ART/params_values.txt" | head -1 | cut -d= -f2-)
        if ! python3 -X utf8 -c 'import sys; raise SystemExit(0 if abs(float(sys.argv[1]) - float(sys.argv[2])) < 1e-6 else 1)' "$got" "$M1_REPEAT_XY_GOAL_TOL"; then
            log "XY_GOAL_TOL_NOT_APPLIED got=${got:-missing} want=$M1_REPEAT_XY_GOAL_TOL"
            return 1
        fi
        log "XY_GOAL_TOL_APPLIED $got"
    fi
    log "PARAMS_NUMERIC $(wc -l <"$ART/params_values.txt") values"
}

# ── фиксация до цели (код, не текст промпта) ──
capture_before_goal() {
    local args mounts sdf image_gz image_nav2 overlays max_vel pathalign line rc=0
    args=$(sandbox "docker inspect -f '{{join .Args \" \"}}' $(printf '%q' "$G")" 2>/dev/null || true)
    mounts=$(sandbox "docker inspect -f '{{range .Mounts}}{{println .Destination}}{{end}}' $(printf '%q' "$G")" 2>/dev/null || true)
    image_gz=$(sandbox "docker inspect -f '{{.Image}}' $(printf '%q' "$G")" 2>/dev/null || true)
    image_nav2=$(main "docker inspect -f '{{.Image}}' $(printf '%q' "$NAV2_CONTAINER")" 2>/dev/null || true)
    printf '%s\n' "$args" >"$ART/gazebo_args.txt"
    printf '%s\n' "$mounts" >"$ART/gazebo_mounts.txt"
    {
        printf 'gazebo %s\n' "$image_gz"
        printf 'nav2 %s\n' "$image_nav2"
        main "docker inspect -f '{{.Config.Image}} {{.Image}}' robot-state-publisher twist-mux rtabmap followpath-watchdog 2>/dev/null" || true
    } >"$ART/images.txt"

    local wpath=""
    case "$args" in
        *world:=*) wpath="${args#*world:=}"; wpath="${wpath%% *}" ;;
    esac
    sdf=""
    if [ -n "$wpath" ]; then
        sdf=$(sandbox "docker exec $(printf '%q' "$G") cat $(printf '%q' "$wpath") 2>/dev/null" || true)
    fi
    if [ -n "$sdf" ]; then
        printf '%s' "$sdf" >"$ART/world.sdf"
        printf '%s\n' "$(printf '%s' "$sdf" | sha256sum | awk '{print $1}')" >"$ART/sdf.sha256"
    else
        printf 'missing\n' >"$ART/sdf.sha256"
        : >"$ART/world.sdf"
    fi

    overlays=$(main "docker inspect $(printf '%q' "$NAV2_CONTAINER") --format '{{range .Config.Env}}{{println .}}{{end}}' | sed -n 's/^NAV2_PARAMS_OVERLAYS=//p'" || true)
    # Параметры — после active controller_server (capture_params_after_ready), не здесь:
    # до готовности ros2 param get отдаёт баннер ros_with_namespace.sh без числа.
    printf 'overlays=%s\n' "${overlays:-none}" >"$ART/params_overlays.txt"
    printf 'pending\n' >"$ART/params.txt"

    # Старт map←base_footprint и GT. Нет кадра — none, ноль не подставляем (m1_topics).
    main "docker exec $(printf '%q' "$NAV2_CONTAINER") /ros_scripts/ros_with_namespace.sh bash -c 'source /opt/ros/humble/setup.bash; timeout 15 ros2 run tf2_ros tf2_echo map base_footprint'" \
        >"$ART/start_map_raw.txt" 2>&1 || true
    sandbox "docker exec $(printf '%q' "$G") /entrypoint.sh bash -c 'source /opt/ros/humble/setup.bash; timeout 15 ros2 topic echo /sim/ground_truth/odom --once'" \
        >"$ART/start_gt_raw.txt" 2>&1 || true
    python3 -X utf8 -c 'import pathlib, sys; sys.path.insert(0, sys.argv[1]); from m1_topics import write_start_map_gt; art = pathlib.Path(sys.argv[2]); write_start_map_gt(art, (art / "start_map_raw.txt").read_text(encoding="utf-8", errors="replace"), (art / "start_gt_raw.txt").read_text(encoding="utf-8", errors="replace"))' \
        "$HERE" "$ART"

    printf '%s\n' "$mounts" >"$ART/mounts.txt"
    line=$(python3 -X utf8 "$WORLD_CHECK" depot --args "$args" --mounts-file "$ART/mounts.txt" --sdf-file "$ART/world.sdf") || rc=$?
    printf '%s\n' "$line" >"$ART/world_check.txt"
    if [ "$rc" -ne 0 ]; then
        log "WORLD_CHECK_FAIL $line"
        return 1
    fi
    log "WORLD_CHECK_PASS $line"
    if ! grep -q . "$ART/sdf.sha256" || grep -qx missing "$ART/sdf.sha256"; then
        log "SDF_SHA_MISSING"
        return 1
    fi
    python3 -X utf8 -c 'import sys; sys.path.insert(0, sys.argv[1]); from m1_topics import assert_ready; assert_ready(sys.argv[2], "before_goal")' "$HERE" "$ART" \
        || { log "BEFORE_GOAL_INCOMPLETE"; return 1; }
    log "BEFORE_GOAL_OK sha=$(cat "$ART/sdf.sha256")"
}

# ── ход ──
export HIL_SANDBOX_REPO="$SANDBOX_REPO"
export HIL_RTABMAP_MODE="${HIL_RTABMAP_MODE:-mapping}"
export HIL_MAPS_DIR="$MAPS_DIR"
export HIL_EXPECT_WORLD=depot
export HIL_WORLD_CHECK_DIR="$ART/hil_on_world"

log "START run_id=$RUN_ID out=$ART"
if ! main "test ! -e $(printf '%q' "$HIL_LOCK")"; then
    log "PREEXISTING_HIL_LOCK"
    exit 20
fi
if ! main "test ! -e '$MAPS_DIR'"; then
    log "MAP_DIR_NOT_FRESH path=$MAPS_DIR"
    exit 21
fi

bash "$HIL_CMD" status >"$ART/hil_status_before.log" 2>&1 || { log "STATUS_BEFORE_FAIL"; exit 23; }

YAW_MODE=$(python3 -X utf8 -c 'import os,sys; sys.path.insert(0, sys.argv[1]); from m1_topics import yaw_isolate_from_env; print(yaw_isolate_from_env(os.environ.get("M1_YAWATGOAL_ISOLATE")))' "$HERE") \
    || { log "YAW_ISOLATE_ENV_BAD value=${M1_YAWATGOAL_ISOLATE:-}"; exit 2; }
if [ -n "$GOAL_PROBE_DIR" ]; then
    validate_goal_probe || { log "GOAL_PROBE_INPUT_INVALID"; exit 2; }
    current_nav_image=$(main "docker inspect -f '{{.Image}}' $(printf '%q' "$NAV2_CONTAINER")") \
        || { log "GOAL_PROBE_IMAGE_UNREADABLE"; exit 2; }
    [ "$current_nav_image" = "$GOAL_PROBE_IMAGE" ] || { log "GOAL_PROBE_IMAGE_MISMATCH $current_nav_image"; exit 2; }
fi

# Оверлей публикации DWB и проба goal-checker кладём ДО on. hil.sh on читает
# HIL_NAV2TUNE_COMPOSE / HIL_GOAL_PROBE_COMPOSE и поднимает nav2 уже с --params-file
# и томом пробы. Recreate после on — второй старт: пара 19.11 видела файл в контейнере
# и боевой short_circuit True (configure прошёл без оверлея).
# Скорости нет. M1_REPEAT_SHORT_CIRCUIT=false добавляет только
# short_circuit_trajectory_evaluation: false (полный /evaluation, выбор траектории тот же).
# default true — строку не пишем, остаётся боевой true.
export M1_REPEAT_SHORT_CIRCUIT="${M1_REPEAT_SHORT_CIRCUIT:-true}"
# Пусто — боевой trans_stopped_velocity 0.03, строку не пишем. Число > 0 — в тот же оверлей.
python3 -X utf8 -c 'import os, sys; sys.path.insert(0, sys.argv[1]); from m1_topics import dwb_publish_overlay_yaml, short_circuit_from_env, trans_stopped_from_env, xy_goal_tol_from_env; open(sys.argv[2], "w", encoding="utf-8").write(dwb_publish_overlay_yaml(short_circuit_from_env(os.environ.get("M1_REPEAT_SHORT_CIRCUIT")), trans_stopped_from_env(os.environ.get("M1_REPEAT_TRANS_STOPPED")), xy_goal_tol_from_env(os.environ.get("M1_REPEAT_XY_GOAL_TOL"))))' \
    "$HERE" "$ART/dwb_publish.yaml" || { log "OVERLAY_ENV_BAD short_circuit=${M1_REPEAT_SHORT_CIRCUIT} trans_stopped=${M1_REPEAT_TRANS_STOPPED:-}"; exit 2; }
log "DWB_OVERLAY short_circuit=${M1_REPEAT_SHORT_CIRCUIT} trans_stopped=${M1_REPEAT_TRANS_STOPPED:-combat} xy_goal_tol=${M1_REPEAT_XY_GOAL_TOL:-combat}"
main "mkdir -p /tmp/hil/nav2tune" || { log "NAV2TUNE_DIR_FAIL"; exit 30; }
# Не через main(): она делает ssh </dev/null и выбрасывает переданный файл.
# Чистая пара m1repeat_20261119T084941Z: compose на Main 0 байт, overlays=none,
# param get True при yaml false. Шифу, 19.11.
tr -d '\r' <"$ROOT/scripts/hil/speed/nav2tune.compose.yaml" >"$ART/nav2tune.compose.yaml"
scp -q -o BatchMode=yes "$ART/dwb_publish.yaml" "$MAIN:/tmp/hil/nav2tune/dwb_publish.yaml" || { log "OVERLAY_SCP_FAIL"; exit 30; }
scp -q -o BatchMode=yes "$ART/nav2tune.compose.yaml" "$MAIN:/tmp/hil/nav2tune.compose.yaml" || { log "OVERLAY_SCP_FAIL"; exit 30; }
main "test -s /tmp/hil/nav2tune/dwb_publish.yaml && test -s /tmp/hil/nav2tune.compose.yaml && wc -c /tmp/hil/nav2tune/dwb_publish.yaml /tmp/hil/nav2tune.compose.yaml" \
    >"$ART/overlay_sizes.txt" || { log "OVERLAY_EMPTY_ON_MAIN"; exit 30; }
# on читает эти переменные: main_sim_up добавляет -f до первого старта controller_server.
# off их не видит и поднимает боевой nav2 без оверлея.
export HIL_NAV2TUNE_COMPOSE=/tmp/hil/nav2tune.compose.yaml
export NAV2TUNE_OVERLAYS=/nav2tune/dwb_publish.yaml
# Проба goal-checker (#4945) — тот же первый старт, не recreate после configure.
if [ -n "$GOAL_PROBE_DIR" ]; then
    scp -q -o BatchMode=yes "$GOAL_PROBE_DIR/libcontroller_server_core.so" "$MAIN:$GOAL_PROBE_MOUNT" \
        || { log "GOAL_PROBE_SCP_FAIL"; exit 30; }
    scp -q -o BatchMode=yes "$ROOT/scripts/hil/diagnostics/nav2_goal_check.compose.yaml" \
        "$MAIN:/tmp/hil/goal_check_probe.compose.yaml" || { log "GOAL_PROBE_COMPOSE_SCP_FAIL"; exit 30; }
    export HIL_GOAL_PROBE_COMPOSE=/tmp/hil/goal_check_probe.compose.yaml
    export M1_GOAL_PROBE_MOUNT="$GOAL_PROBE_MOUNT"
    log "GOAL_PROBE_BEFORE_ON compose=$HIL_GOAL_PROBE_COMPOSE mount=$M1_GOAL_PROBE_MOUNT"
fi
log "OVERLAY_BEFORE_ON compose=$HIL_NAV2TUNE_COMPOSE overlays=$NAV2TUNE_OVERLAYS"
ON_ATTEMPTED=1
LOCK_OWNED=0
CLEANED=0
# Один retry только если ЭТОТ запуск взял замок и отказ — «не видно RSP»,
# и hil off этого запуска завершился 0. Отказ до замка: ни retry, ни off.
bash "$HIL_CMD" on >"$ART/hil_on_attempt1.log" 2>&1
on_rc=$?
cp "$ART/hil_on_attempt1.log" "$ART/hil_on.log"
if [ "$on_rc" != 0 ]; then
    log "HIL_ON_TRY_1 FAIL rc=$on_rc"
    if ! lock_is_ours; then
        log "HIL_ON_FAIL before_lock"
        exit 24
    fi
    LOCK_OWNED=1
    if ! grep -q 'robot_state_publisher' "$ART/hil_on_attempt1.log"; then
        log "HIL_ON_FAIL not_rsp"
        exit 24
    fi
    bash "$HIL_CMD" off >"$ART/hil_on_retry_off.log" 2>&1
    off_retry=$?
    if [ "$off_retry" != 0 ]; then
        log "HIL_ON_FAIL retry_off_rc=$off_retry"
        exit 24
    fi
    LOCK_OWNED=0
    sleep 20
    bash "$HIL_CMD" on >"$ART/hil_on_attempt2.log" 2>&1
    on_rc=$?
    cat "$ART/hil_on_attempt2.log" >>"$ART/hil_on.log"
    if [ "$on_rc" != 0 ]; then
        log "HIL_ON_FAIL attempt2 rc=$on_rc"
        exit 24
    fi
fi
lock_is_ours || { log "HIL_ON_LOCK_NOT_OWNED"; exit 24; }
LOCK_OWNED=1
log "HIL_ON_OK"
# Второй старт не делаем: он опоздал бы к configure. Env должен уже быть в контейнере,
# который поднял on. Нет NAV2_PARAMS_OVERLAYS — start_nav2_direct.sh не передаст --params-file.
# Проба goal-checker едет тем же первым up (HIL_GOAL_PROBE_COMPOSE), не recreate.
overlays_env=$(main "docker inspect $(printf '%q' "$NAV2_CONTAINER") --format '{{range .Config.Env}}{{println .}}{{end}}'" | sed -n 's/^NAV2_PARAMS_OVERLAYS=//p' || true)
printf '%s\n' "${overlays_env:-none}" >"$ART/overlays_env.txt"
if [ "$overlays_env" != "/nav2tune/dwb_publish.yaml" ]; then
    log "OVERLAY_ENV_MISSING got=${overlays_env:-none}"
    exit 30
fi
# Файл внутри контейнера. Строку short_circuit требуем только когда просили false:
# боевой true её не пишет, и grep по ней ложно валил default-прогон.
overlay_in=$(main "docker exec $(printf '%q' "$NAV2_CONTAINER") cat /nav2tune/dwb_publish.yaml" || true)
printf '%s\n' "$overlay_in" >"$ART/overlay_in_container.yaml"
if ! grep -q 'publish_evaluation: true' "$ART/overlay_in_container.yaml"; then
    log "OVERLAY_NOT_IN_CONTAINER"
    exit 30
fi
if [ "${M1_REPEAT_SHORT_CIRCUIT:-true}" = false ]; then
    if ! grep -q 'short_circuit_trajectory_evaluation: false' "$ART/overlay_in_container.yaml"; then
        log "OVERLAY_NOT_IN_CONTAINER short_circuit"
        exit 30
    fi
fi
log "DWB_PUBLISH_OVERLAY before configure (env set, file in container)"
# Изоляция YawAtGoal: HIL-оверлей mux, боевой twist_mux.yaml не трогаем.
# Замки остаются. Применяется до цели, файл удаляется до off.
if [ "$YAW_MODE" != off ]; then
    # Полный боевой yaml с одним изменением. Второй --params-file не умеет
    # удалить вход, поэтому mux читает ЭТОТ файл вместо боевого (TWIST_MUX_CONFIG).
    # Боевой twist_mux.yaml на диске не правится. YawAtGoal публикует как раньше.
    python3 -X utf8 -c 'import sys; sys.path.insert(0, sys.argv[1]); from m1_topics import twist_mux_config_for_isolate; open(sys.argv[3], "w", encoding="utf-8").write(twist_mux_config_for_isolate(open(sys.argv[2], encoding="utf-8").read(), sys.argv[4]))' \
        "$HERE" "$ROOT/docker/main/config/twist_mux/twist_mux.yaml" "$ART/twist_mux_hil.yaml" "$YAW_MODE" \
        || { log "YAW_ISOLATE_YAML_FAIL"; exit 2; }
    scp -q -o BatchMode=yes "$ART/twist_mux_hil.yaml" "$MAIN:/tmp/hil/twist_mux_hil.yaml" || { log "YAW_ISOLATE_SCP_FAIL"; exit 30; }
    main "test -s /tmp/hil/twist_mux_hil.yaml" || { log "YAW_ISOLATE_NOT_ON_MAIN"; exit 30; }
    main "mkdir -p /tmp/hil && cat > /tmp/hil/twist_mux.hil.yaml << 'EOF'
services:
  twist-mux:
    environment:
      - TWIST_MUX_CONFIG=/config/twist_mux/twist_mux.yaml
    volumes:
      - /tmp/hil/twist_mux_hil.yaml:/config/twist_mux/twist_mux.yaml:ro
EOF
cd /home/ros2/rob_box_project/docker/main && HIL_MAPS_DIR='$MAPS_DIR' docker compose -f docker-compose.yaml -f /tmp/hil/main.images.yaml -f /tmp/hil/main.hil.yaml -f /tmp/hil/twist_mux.hil.yaml up -d --no-deps --pull never --force-recreate twist-mux" \
        >"$ART/yaw_isolate_up.log" 2>&1 || { log "YAW_ISOLATE_RECREATE_FAIL"; exit 30; }
    log "YAW_ISOLATE_RECREATED mode=$YAW_MODE config=/tmp/hil/twist_mux_hil.yaml"
fi

# Боевые 0.15 / PathAlign 0.20: оверлей скорости не кладём. Проверяем факт параметра до цели.
capture_before_goal || { log "CAPTURE_BEFORE_GOAL_FAIL"; exit 32; }

TOPICS="$(topics_line)"
log "BAG_TOPICS $TOPICS"
sandbox "mkdir -p '$REMOTE_OUT'" || { log "REMOTE_OUT_FAIL"; exit 31; }
# shellcheck disable=SC2086
python3 -X utf8 -c 'import sys; sys.path.insert(0, sys.argv[1]); from m1_topics import bag_qos_overrides_yaml; open(sys.argv[2], "w", encoding="utf-8").write(bag_qos_overrides_yaml())' \
    "$HERE" "$ART/bag_qos.yaml"
scp -q -o BatchMode=yes "$ART/bag_qos.yaml" "$SANDBOX:$REMOTE_OUT/bag_qos.yaml" || { log "BAG_QOS_SCP_FAIL"; exit 31; }
sandbox "docker exec -d $(printf '%q' "$G") /entrypoint.sh bash -c 'echo \$\$ >$(printf '%q' "$CONTAINER_OUT")/recorder.pid; exec ros2 bag record --qos-profile-overrides-path $(printf '%q' "$CONTAINER_OUT")/bag_qos.yaml -o $(printf '%q' "$CONTAINER_OUT")/bag $TOPICS >$(printf '%q' "$CONTAINER_OUT")/bag.log 2>&1'" \
    || { log "BAG_RECORD_FAIL"; exit 31; }
sleep 5
sandbox "test -d '$REMOTE_OUT/bag' && echo BAG_RECORDING_DIRECTORY_PRESENT" >"$ART/bag_start.log" 2>&1 \
    || { log "BAG_DIR_ABSENT"; exit 31; }

# Сэмплеры до ожидания ready: сбой TF должен оставить метрики обоих хостов.
start_resource_samplers || log "SAMPLERS_START_FAILED"
# Готовность навигатора — как route.py: не шлём цель, пока bt_navigator не active.
ready=0
for _ in $(seq 1 $((NAV_READY_TIMEOUT / 5))); do
    state=""
    cli_rc=0
    state=$(ros2_in_nav2 "ros2 lifecycle get /bt_navigator" 2>/dev/null) || cli_rc=$?
    printf '%s rc=%s %s\n' "$(date -u +%FT%TZ)" "$cli_rc" "$state" >>"$ART/nav_ready.log"
    if [ "$cli_rc" -eq 124 ]; then
        log "NAV_CLI_TIMEOUT ros2 lifecycle get /bt_navigator"
        exit 37
    fi
    if grep -q 'active \[3\]' <<<"$state"; then ready=1; break; fi
    sleep 5
done
if [ "$ready" != 1 ]; then
    # Half1 100054Z: bag /tf = только map→icp_odom, /odom пуст. local_costmap
    # ждала odom→base_footprint. Первая дыра цепочки — нет кадра odom.
    # Пишем, кто должен его дать, и выходим. Таймаут не растягиваем.
    main "docker logs --tail 40 $(printf '%q' "$NAV2_CONTAINER") 2>&1" >"$ART/nav_not_ready_tail.log" 2>&1 || true
    {
        echo "FIRST_MISSING odom->base_footprint"
        echo "publisher diff_drive_controller in gazebo (controller_manager.yaml odom_frame_id=odom base_frame_id=base_footprint)"
        echo "bag_half1 /tf had map->icp_odom only; /odom message_count 0"
        echo "icp_odom->odom publisher rtabmap icp_odometry; needs guess frame odom"
    } >"$ART/tf_break.txt"
    if grep -q 'frame does not exist' "$ART/nav_not_ready_tail.log"; then
        log "NAV_NOT_READY tf_odom_missing M1_NOT_SENT"
    else
        log "NAV_NOT_READY M1_NOT_SENT"
    fi
    exit 33
fi
# controller_server active — иначе param get снова баннер, как в ночном прогоне 19.11.
cs_ready=0
for _ in $(seq 1 12); do
    cs=""
    cli_rc=0
    cs=$(ros2_in_nav2 "ros2 lifecycle get /controller_server" 2>/dev/null) || cli_rc=$?
    printf '%s rc=%s %s\n' "$(date -u +%FT%TZ)" "$cli_rc" "$cs" >>"$ART/nav_ready.log"
    if [ "$cli_rc" -eq 124 ]; then
        log "NAV_CLI_TIMEOUT ros2 lifecycle get /controller_server"
        exit 37
    fi
    if grep -q 'active \[3\]' <<<"$cs"; then cs_ready=1; break; fi
    sleep 5
done
if [ "$cs_ready" != 1 ]; then
    log "CONTROLLER_NOT_READY M1_NOT_SENT"
    exit 34
fi
# Граф до цели. detach: twist_mux не в подписчиках /cmd_vel_yaw. Иначе — есть.
if [ "${YAW_MODE:-off}" = detach ]; then
    main "docker exec $(printf '%q' "$NAV2_CONTAINER") /ros_scripts/ros_with_namespace.sh bash -c 'source /opt/ros/humble/setup.bash; timeout 20 ros2 topic info /cmd_vel_yaw --verbose --no-daemon'" \
        >"$ART/cmd_vel_yaw_info.txt" 2>&1 || { log "YAW_GRAPH_UNREADABLE M1_NOT_SENT"; exit 36; }
    if grep -qx 'Node name: twist_mux' "$ART/cmd_vel_yaw_info.txt"; then
        log "YAW_MUX_STILL_SUBSCRIBED M1_NOT_SENT"
        exit 36
    fi
    awk '/^Publisher count:/ {publisher=1}
         /^Subscription count:/ {publisher=0; subscriptions=1}
         publisher && /^Node name: followpath_watchdog$/ {yaw=1}
         END {exit !(yaw && subscriptions)}' "$ART/cmd_vel_yaw_info.txt" \
        || { log "YAW_PUBLISHER_MISSING M1_NOT_SENT"; exit 36; }
    main "docker exec $(printf '%q' "$NAV2_CONTAINER") /ros_scripts/ros_with_namespace.sh bash -c 'source /opt/ros/humble/setup.bash; timeout 20 ros2 node list --no-daemon'" \
        >"$ART/mux_nodes.txt" 2>&1 || { log "YAW_GRAPH_UNREADABLE M1_NOT_SENT"; exit 36; }
    grep -Eq '(^|/)twist_mux$' "$ART/mux_nodes.txt" || { log "YAW_MUX_MISSING M1_NOT_SENT"; exit 36; }
    log "YAW_MUX_DETACHED publisher=followpath_watchdog"
fi
capture_params_after_ready || { log "PARAMS_INCOMPLETE M1_NOT_SENT"; exit 35; }
if [ -n "$GOAL_PROBE_DIR" ]; then
    verify_goal_probe_loaded || { log "GOAL_PROBE_NOT_LOADED M1_NOT_SENT"; exit 37; }
    log "GOAL_PROBE_LOADED sha=$GOAL_PROBE_SHA"
fi
log "GOAL_SEND run_id=$RUN_ID goal=$GOAL after BEFORE_GOAL_OK and PARAMS_NUMERIC"
# route.py читает ROUTE_LOG. Путь — файл этого run_id, не общий out/route.log.
# Общий out/route.log трогаем только после runvideo и только если он не старше старта:
# runvideo.sh игнорирует ROUTE_LOG и пишет $D/out/route.log (raw m1repeat_20261119T185434Z:
# cp: cannot stat '.../out/m1repeat_20261119T185434Z/route.log' → ARTIFACTS_INCOMPLETE rc=1).
sandbox "mkdir -p '$REMOTE_OUT'; rm -f '$REMOTE_OUT/route.log'"
SANDBOX_T0=$(sandbox "date +%s" 2>/dev/null) || SANDBOX_T0=0
sandbox "cd '$SANDBOX_REPO' && CAMERAS=0 ROUTE_LOG='$REMOTE_OUT/route.log' ./runvideo.sh '$GOAL'" >"$ART/runvideo.log" 2>&1 || true
# Старый общий лог (прошлый заезд, yaw_std утащил GOAL 07:28 от clean_003) не копируем:
# mtime не раньше старта runvideo — значит, файл перезаписан этим прогоном.
ROUTE_COPIED=0
if [ "$SANDBOX_T0" -gt 0 ] && \
    sandbox "test -f '$SANDBOX_REPO/out/route.log' && [ \$(stat -c %Y '$SANDBOX_REPO/out/route.log') -ge $SANDBOX_T0 ]"; then
    sandbox "cp -f '$SANDBOX_REPO/out/route.log' '$REMOTE_OUT/route.log'" && ROUTE_COPIED=1
fi
if [ "$ROUTE_COPIED" = 1 ]; then
    log "ROUTE_LOG_COPIED goal=$GOAL"
else
    log "ROUTE_LOG_NOT_COPIED goal=$GOAL sandbox_t0=$SANDBOX_T0"
fi
log "ROUTE_DONE"
# pull_before_off — в cleanup, до hil off.
exit 0
