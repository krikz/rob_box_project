#!/usr/bin/env bash
# Повторный одиночный M1 с расширенной записью (#4694).
#
# Условия сопоставимости с прогоном 18.11 16:26Z (/tmp/hil_m1_stall):
#   Depot (HIL_EXPECT_WORLD=depot, гейт scripts/hil/hil.sh как есть),
#   спавн (0, 0, 0), цель M1 (-3.7, 4, 0°), max_vel_x 0.15,
#   PathAlign.forward_point_distance 0.20 (боевой nav2_params.yaml, оверлей скорости не кладём).
#   Оверлей repeat только включает publish_evaluation/publish_trajectories/publish_local_plan:
#   DWB считает score до публикации, выбор vx/wz от этого не меняется. Боевой yaml не правится.
#
# До отправки цели на хост: sha SDF, ID образов, фактические параметры, старт map+GT,
# вердикт мира. Bag (в т.ч. /cmd_vel_yaw, global costmap, /cmd_vel_nav) и route.log
# копируются на хост ДО hil off. Живой прогон этим файлом не запускался в PR.
#
#   M1_REPEAT_OUT=/tmp/hil_m1_repeat bash scripts/hil/repeat/run_m1_repeat.sh
#
# Не берёт /tmp/hil.lock сам: замок берёт hil.sh on. Если замок уже есть — выход 20, стенд не трогаем.
set -uo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HIL_CMD="${HIL_CMD:-$ROOT/scripts/hil/hil.sh}"
WORLD_CHECK="${WORLD_CHECK:-$ROOT/scripts/hil/checks/world_check.py}"
TOPICS_PY="${TOPICS_PY:-$HERE/m1_topics.py}"

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

SSHO=(-o BatchMode=yes -o ConnectTimeout=10 -o ServerAliveInterval=5 -o ServerAliveCountMax=6)
ART="$M1_REPEAT_OUT"
mkdir -p "$ART"
SUMMARY="$ART/verdict.log"
: >"$SUMMARY"
ON_ATTEMPTED=0
CLEANED=1

# shellcheck disable=SC2029
main() { ssh "${SSHO[@]}" "$MAIN" "$*" </dev/null; }
# shellcheck disable=SC2029
sandbox() { ssh "${SSHO[@]}" "$SANDBOX" "$*" </dev/null; }
log() { printf '%s %s\n' "$(date -u +%FT%TZ)" "$*" | tee -a "$SUMMARY"; }

# Один ros2 param get. Баннер ros_with_namespace.sh без «value is:» — пусто, зовём ещё раз.
param_get_once() {
    local pname="$1" raw value
    # timeout 15: run2 (0.005b) висел 30 мин, ros2 param get не вернулся. Нет числа — missing, не ждём.
    raw=$(main "docker exec $(printf '%q' "$NAV2_CONTAINER") /ros_scripts/ros_with_namespace.sh bash -c 'source /opt/ros/humble/setup.bash; timeout 15 ros2 param get /controller_server $(printf '%q' "$pname")'" 2>/dev/null || true)
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
    local rc=0
    sandbox "docker exec $(printf '%q' "$G") bash -c 'if test -f $(printf '%q' "$CONTAINER_OUT")/recorder.pid; then kill -INT \$(cat $(printf '%q' "$CONTAINER_OUT")/recorder.pid) 2>/dev/null || true; fi'" \
        >>"$ART/bag_stop.log" 2>&1 || true
    sandbox "for i in \$(seq 1 $BAG_CLOSE_WAIT); do test -f '$REMOTE_OUT/bag/metadata.yaml' && break; sleep 1; done; docker exec $(printf '%q' "$G") /entrypoint.sh ros2 bag info '$CONTAINER_OUT/bag'" \
        >"$ART/bag_info.txt" 2>&1 || rc=1
    sandbox "mkdir -p '$REMOTE_OUT'; cp '$SANDBOX_REPO/out/route.log' '$REMOTE_OUT/route.log' 2>/dev/null || true" || true
    copy_bag_to_host || rc=1
    # Лог nav2 до off: «Failed to make progress» и какой recovery (clear/spin/wait/back_up).
    main "docker logs $(printf '%q' "$NAV2_CONTAINER") 2>&1" >"$ART/nav2.log" 2>&1 || true
    python3 -X utf8 -c '
import pathlib, sys
sys.path.insert(0, sys.argv[1])
from m1_topics import recovery_hits
text = pathlib.Path(sys.argv[2]).read_text(encoding="utf-8", errors="replace")
hits = recovery_hits(text)
pathlib.Path(sys.argv[3]).write_text(
    ("hits=" + " ".join(hits) if hits else "hits=none") + "\n", encoding="utf-8")
' "$HERE" "$ART/nav2.log" "$ART/recovery.txt" || true
    if [ -s "$ART/route.log" ] && [ -s "$ART/bag_info.txt" ] && [ -f "$ART/bag/metadata.yaml" ] && [ -s "$ART/nav2.log" ]; then
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
    stop_route
    stop_resource_samplers
    if [ "$ON_ATTEMPTED" = 1 ]; then
        pull_before_off || art_rc=1
        # hil off пересоздаёт nav2 без nav2tune.compose.yaml — оверлей публикации уходит
        # вместе с HIL. Файл на Main удаляем, чтобы следующий on его не подхватил.
        main "rm -f /tmp/hil/nav2tune/dwb_publish.yaml /tmp/hil/nav2tune.compose.yaml" || true
        bash "$HIL_CMD" off >"$ART/hil_off.log" 2>&1 || off_rc=$?
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
    main "nohup python3 -X utf8 /tmp/hil/resource_sample.py --loop --host main --out /tmp/hil/main_resources.log --disk mmcblk0 >/tmp/hil/main_sampler.err 2>&1 & echo \$! > /tmp/hil/main_sampler.pid" \
        || log "MAIN_SAMPLER_START_FAIL"
    log "SAMPLERS_STARTED"
}

stop_resource_samplers() {
    if [ -f "$ART/sandbox_sampler.pid" ]; then
        kill "$(cat "$ART/sandbox_sampler.pid")" 2>/dev/null || true
    fi
    main "if test -f /tmp/hil/main_sampler.pid; then kill \$(cat /tmp/hil/main_sampler.pid) 2>/dev/null || true; fi; cat /tmp/hil/main_resources.log" \
        >"$ART/main_resources.log" 2>/dev/null || true
    log "SAMPLERS_STOPPED"
}

# После active controller_server. Нет числа после двух попыток — name=missing, не баннер.
capture_params_after_ready() {
    local pname value attempt overlays
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
        FollowPath.short_circuit_trajectory_evaluation
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
if main "test -e /tmp/hil.lock"; then
    log "PREEXISTING_HIL_LOCK"
    exit 20
fi
if ! main "test ! -e '$MAPS_DIR'"; then
    log "MAP_DIR_NOT_FRESH path=$MAPS_DIR"
    exit 21
fi

bash "$HIL_CMD" status >"$ART/hil_status_before.log" 2>&1 || { log "STATUS_BEFORE_FAIL"; exit 23; }

# Оверлей публикации DWB кладём ДО on: hil.sh on поднимает nav2, и контроллер читает
# параметры на старте. Скорости нет. M1_REPEAT_SHORT_CIRCUIT=false добавляет только
# short_circuit_trajectory_evaluation: false (полный /evaluation, выбор траектории тот же).
# default true — строку не пишем, остаётся боевой true.
export M1_REPEAT_SHORT_CIRCUIT="${M1_REPEAT_SHORT_CIRCUIT:-true}"
# Пусто — боевой trans_stopped_velocity 0.03, строку не пишем. Число > 0 — в тот же оверлей.
python3 -X utf8 -c 'import os, sys; sys.path.insert(0, sys.argv[1]); from m1_topics import dwb_publish_overlay_yaml, short_circuit_from_env, trans_stopped_from_env; open(sys.argv[2], "w", encoding="utf-8").write(dwb_publish_overlay_yaml(short_circuit_from_env(os.environ.get("M1_REPEAT_SHORT_CIRCUIT")), trans_stopped_from_env(os.environ.get("M1_REPEAT_TRANS_STOPPED"))))' \
    "$HERE" "$ART/dwb_publish.yaml" || { log "OVERLAY_ENV_BAD short_circuit=${M1_REPEAT_SHORT_CIRCUIT} trans_stopped=${M1_REPEAT_TRANS_STOPPED:-}"; exit 2; }
log "DWB_OVERLAY short_circuit=${M1_REPEAT_SHORT_CIRCUIT} trans_stopped=${M1_REPEAT_TRANS_STOPPED:-combat}"
main "mkdir -p /tmp/hil/nav2tune" || { log "NAV2TUNE_DIR_FAIL"; exit 30; }
scp -q -o BatchMode=yes "$ART/dwb_publish.yaml" "$MAIN:/tmp/hil/nav2tune/dwb_publish.yaml" || { log "OVERLAY_SCP_FAIL"; exit 30; }
main "tr -d '\r' > /tmp/hil/nav2tune.compose.yaml" <"$ROOT/scripts/hil/speed/nav2tune.compose.yaml" || { log "COMPOSE_SCP_FAIL"; exit 30; }
# После on hil.sh пересоздаёт nav2 без этого файла — оверлей не останется в бою.
# Здесь on ещё не взят: кладём compose-оверлей, on его не знает. Поэтому после on
# пересоздаём только nav2 с тем же HIL_MAPS_DIR, что возьмёт замок.
ON_ATTEMPTED=1
CLEANED=0
bash "$HIL_CMD" on >"$ART/hil_on.log" 2>&1 || { log "HIL_ON_FAIL"; exit 24; }
main "cd /home/ros2/rob_box_project/docker/main && HIL_MAPS_DIR='$MAPS_DIR' HIL_RTABMAP_MODE='${HIL_RTABMAP_MODE:-mapping}' NAV2TUNE_OVERLAYS='/nav2tune/dwb_publish.yaml' docker compose -f docker-compose.yaml -f /tmp/hil/main.images.yaml -f /tmp/hil/main.hil.yaml -f /tmp/hil/nav2tune.compose.yaml up -d --no-deps --pull never --force-recreate nav2" \
    >"$ART/dwb_overlay_up.log" 2>&1 || { log "DWB_OVERLAY_UP_FAIL"; exit 30; }
log "DWB_PUBLISH_OVERLAY on (publish_evaluation/trajectories/local_plan only)"

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

# Готовность навигатора — как route.py: не шлём цель, пока bt_navigator не active.
ready=0
for _ in $(seq 1 $((NAV_READY_TIMEOUT / 5))); do
    state=$(main "docker exec $(printf '%q' "$NAV2_CONTAINER") /ros_scripts/ros_with_namespace.sh bash -c 'source /opt/ros/humble/setup.bash; ros2 lifecycle get /bt_navigator'" 2>/dev/null || true)
    printf '%s %s\n' "$(date -u +%FT%TZ)" "$state" >>"$ART/nav_ready.log"
    if grep -q 'active \[3\]' <<<"$state"; then ready=1; break; fi
    sleep 5
done
if [ "$ready" != 1 ]; then
    log "NAV_NOT_READY M1_NOT_SENT"
    exit 33
fi
# controller_server active — иначе param get снова баннер, как в ночном прогоне 19.11.
cs_ready=0
for _ in $(seq 1 12); do
    cs=$(main "docker exec $(printf '%q' "$NAV2_CONTAINER") /ros_scripts/ros_with_namespace.sh bash -c 'source /opt/ros/humble/setup.bash; ros2 lifecycle get /controller_server'" 2>/dev/null || true)
    printf '%s %s\n' "$(date -u +%FT%TZ)" "$cs" >>"$ART/nav_ready.log"
    if grep -q 'active \[3\]' <<<"$cs"; then cs_ready=1; break; fi
    sleep 5
done
if [ "$cs_ready" != 1 ]; then
    log "CONTROLLER_NOT_READY M1_NOT_SENT"
    exit 34
fi
capture_params_after_ready || { log "PARAMS_INCOMPLETE M1_NOT_SENT"; exit 35; }

start_resource_samplers
log "GOAL_SEND M1 target=(-3.7,4.0,0) after BEFORE_GOAL_OK and PARAMS_NUMERIC"
sandbox "cd '$SANDBOX_REPO' && CAMERAS=0 ./runvideo.sh M1" >"$ART/runvideo.log" 2>&1 || true
log "ROUTE_DONE"
# pull_before_off — в cleanup, до hil off.
exit 0
