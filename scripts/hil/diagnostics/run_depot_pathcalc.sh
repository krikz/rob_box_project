#!/usr/bin/env bash
# #4682: до трёх свежих M1 с diagnostic NavFn; стоп на первой ошибке PathCalc.
# Не меняет маршрут, его таймаут, скорости, зоны или critics. Только HIL, не production.
set -uo pipefail
export MSYS_NO_PATHCONV=1

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HIL_CMD="${HIL_CMD:-$ROOT/scripts/hil/hil.sh}"
: "${PATHCALC_OUT:?PATHCALC_OUT — локальный каталог артефактов}"
RUN_ID="${PATHCALC_RUN_ID:-depot4682_pathcalc_$(date -u +%Y%m%dT%H%M%SZ)}"
MAIN="${HIL_MAIN_HOST:-ros2@10.1.1.10}"
SANDBOX="${HIL_SANDBOX_HOST:-ros2@10.1.1.250}"
SANDBOX_REPO="${HIL_SANDBOX_REPO:-/home/ros2/rob_box_hilvideo}"
DIAG_LIB="${PATHCALC_DIAG_LIB:-/home/ros2/rob_box_hil/nav2_diag_4682_snapshot/install/nav2_navfn_planner/lib/libnav2_navfn_planner.so}"
SNAPSHOT_PATH=/tmp/nav2_pathcalc_snapshot.bin
MONITOR_TIMEOUT="${PATHCALC_MONITOR_TIMEOUT:-150}"
LIFECYCLE_ATTEMPTS="${PATHCALC_LIFECYCLE_ATTEMPTS:-60}"
SSHO=(-o BatchMode=yes -o ConnectTimeout=10 -o ServerAliveInterval=5 -o ServerAliveCountMax=6)
SUMMARY="$PATHCALC_OUT/verdict.log"
ART="$PATHCALC_OUT"
ON_ATTEMPTED=0
CLEANED=1
ATTEMPT=0

mkdir -p "$PATHCALC_OUT"
: >"$SUMMARY"
# $* намеренно собирается на клиенте в одну команду для удалённого bash.
# shellcheck disable=SC2029
main() { ssh "${SSHO[@]}" "$MAIN" "$*" </dev/null; }
# shellcheck disable=SC2029
sandbox() { ssh "${SSHO[@]}" "$SANDBOX" "$*" </dev/null; }
stop_route() {
    sandbox "docker exec rob-box-hil-gazebo pkill -INT -f 'python3 -u /route.py' 2>/dev/null || true" >/dev/null 2>&1
}

copy_route_artifacts() {
    local remote_out="$SANDBOX_REPO/out/${RUN_ID}_r$ATTEMPT"
    sandbox "sleep 5; cp '$SANDBOX_REPO/out/route.log' '$remote_out/route.log'; cp '$SANDBOX_REPO/out/run_start.txt' '$SANDBOX_REPO/out/run_end.txt' '$remote_out/' 2>/dev/null || true; mkdir -p '$remote_out/maprec'; cp '$SANDBOX_REPO/out/maprec/poses.csv' '$SANDBOX_REPO/out/maprec/meta.json' '$remote_out/maprec/' 2>/dev/null || true" >"$ART/goals_raw.log" 2>&1
    scp -q -o BatchMode=yes "$SANDBOX:$remote_out/route.log" "$SANDBOX:$remote_out/runvideo.log" "$ART/" 2>"$ART/artifact_scp.err" || true
    scp -q -o BatchMode=yes "$SANDBOX:$remote_out/maprec/poses.csv" "$SANDBOX:$remote_out/maprec/meta.json" "$ART/" 2>>"$ART/artifact_scp.err" || true
}

capture_pathcalc_snapshot() {
    main "docker exec nav2 cat $SNAPSHOT_PATH" >"$ART/pathcalc_snapshot.bin"
    [ -s "$ART/pathcalc_snapshot.bin" ]
}

capture_pathcalc() {
    main "docker logs nav2 2>&1" >"$ART/nav2_error.log" 2>&1
    grep -E 'calcPath|best_pose|potarr|stc|reason|Failed to create a plan from potential' "$ART/nav2_error.log" >"$ART/calcPath_error.log" || true
    main "timeout 60 docker exec nav2 /ros_scripts/ros_with_namespace.sh bash -c \"source /opt/ros/humble/setup.bash; echo REQUEST_UTC=\\\$(date -u +%FT%TZ); echo REQUEST_ROS; ros2 topic echo /clock --once; ros2 service call /global_costmap/get_costmap nav2_msgs/srv/GetCostmap '{}'; rc=\\\$?; echo RESPONSE_UTC=\\\$(date -u +%FT%TZ); echo RESPONSE_ROS; ros2 topic echo /clock --once; exit \\\$rc\"" >"$ART/get_costmap_error.log" 2>&1
    local costmap_rc=$?
    echo "GET_COSTMAP_RC=$costmap_rc $(date -u +%FT%TZ)" >>"$ART/monitor_verdict.log"
    [ "$costmap_rc" -eq 0 ] || return 1
    local marker
    for marker in REQUEST_UTC REQUEST_ROS RESPONSE_UTC RESPONSE_ROS; do
        grep -q "$marker" "$ART/get_costmap_error.log" || return 1
    done
    [ "$(grep -c '^sec:' "$ART/get_costmap_error.log")" -ge 2 ] && grep -q '^response:' "$ART/get_costmap_error.log"
}

cleanup_attempt() {
    [ "$CLEANED" = 0 ] || return 0
    CLEANED=1
    local bad=0 off_rc=0 gz prod_env
    if [ "$ON_ATTEMPTED" != 1 ]; then
        echo "CLEANUP_NOT_NEEDED attempt=$ATTEMPT/3" | tee -a "$SUMMARY"
        return 0
    fi
    stop_route
    if sandbox "docker inspect -f '{{.State.Running}}' rob-box-hil-gazebo 2>/dev/null | grep -qx true"; then
        local container_out="/sim_video/${RUN_ID}_r$ATTEMPT"
        local remote_out="$SANDBOX_REPO/out/${RUN_ID}_r$ATTEMPT"
        sandbox "docker exec rob-box-hil-gazebo bash -c 'if test -f $container_out/recorder.pid; then kill -INT \$(cat $container_out/recorder.pid) 2>/dev/null || true; fi'; for i in {1..30}; do test -f '$remote_out/bag/metadata.yaml' && break; sleep 1; done; docker exec rob-box-hil-gazebo /entrypoint.sh ros2 bag info '$container_out/bag'" >"$ART/bag_info.log" 2>&1
    fi
    main "docker logs nav2 2>&1" >"$ART/nav2_debug.log" 2>&1
    if [ "$ON_ATTEMPTED" = 1 ]; then
        bash "$HIL_CMD" off >"$ART/hil_off.log" 2>&1
        off_rc=$?
        [ "$off_rc" -eq 0 ] || bad=1
        grep -q 'боевой реестр карт (SELECT \* FROM maps) за HIL не изменился' "$ART/hil_off.log" || bad=1
    fi
    main "rm -f /tmp/hil/nav2_pathcalc.yaml /tmp/hil/start_nav2_pathcalc.sh" >/dev/null 2>&1 || true
    main "test ! -e /tmp/hil/nav2_pathcalc.yaml && test ! -e /tmp/hil/start_nav2_pathcalc.sh" || bad=1
    bash "$HIL_CMD" status >"$ART/hil_status_final.log" 2>&1 || bad=1
    if [ "$ON_ATTEMPTED" = 1 ]; then
        main "test ! -e /tmp/hil.lock" || bad=1
        gz=$(sandbox "docker inspect -f '{{.State.Status}}' rob-box-hil-gazebo 2>/dev/null || echo absent")
        case "$gz" in absent* | exited*) ;; *) bad=1 ;; esac
        prod_env=$(main "docker inspect nav2 --format '{{range .Config.Env}}{{println .}}{{end}}'")
        printf '%s\n' "$prod_env" >"$ART/production_nav2_env.log"
        if grep -q '^NAV2_PARAMS_OVERLAYS=' <<<"$prod_env" || grep -q '^USE_SIM_TIME=true$' <<<"$prod_env"; then
            bad=1
        fi
    fi
    if [ "$bad" -eq 0 ]; then
        echo "CLEANUP_OK attempt=$ATTEMPT/3 off_rc=$off_rc" | tee -a "$SUMMARY"
    else
        echo "CLEANUP_FAILED attempt=$ATTEMPT/3 off_rc=$off_rc" | tee -a "$SUMMARY"
    fi
    return "$bad"
}

# Вызывается косвенно через trap EXIT.
# shellcheck disable=SC2329
on_exit() {
    local rc=$? cleanup_rc=0
    trap - EXIT INT TERM
    cleanup_attempt || cleanup_rc=$?
    if [ "$rc" -eq 0 ] && [ "$cleanup_rc" -ne 0 ]; then rc=50; fi
    exit "$rc"
}
trap on_exit EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

check_lifecycle() {
    local node state rc not_ready=0
    : >"$ART/lifecycle_active.log"
    for ((i = 1; i <= LIFECYCLE_ATTEMPTS; i++)); do
        not_ready=0
        for node in bt_navigator planner_server controller_server smoother_server; do
            state=$(main "docker exec nav2 /ros_scripts/ros_with_namespace.sh bash -c \"source /opt/ros/humble/setup.bash; ros2 lifecycle get /$node\"" 2>&1)
            rc=$?
            printf 'attempt=%s node=%s rc=%s\n%s\n' "$i" "$node" "$rc" "$state" >>"$ART/lifecycle_active.log"
            if [ "$rc" -ne 0 ] || ! grep -q '^active \[3\]$' <<<"$state"; then not_ready=1; fi
        done
        [ "$not_ready" -eq 0 ] && return 0
        sleep 5
    done
    main "docker exec nav2 /ros_scripts/ros_with_namespace.sh bash -c \"source /opt/ros/humble/setup.bash; ros2 service list | grep -E 'bt_navigator|planner_server|controller_server|smoother_server'\"" >"$ART/lifecycle_services.log" 2>&1 || true
    return 1
}

export HIL_SANDBOX_REPO="$SANDBOX_REPO"
export HIL_RTABMAP_MODE=mapping
for ATTEMPT in 1 2 3; do
    ART="$PATHCALC_OUT/${RUN_ID}_r$ATTEMPT"
    REMOTE_OUT="$SANDBOX_REPO/out/${RUN_ID}_r$ATTEMPT"
    CONTAINER_OUT="/sim_video/${RUN_ID}_r$ATTEMPT"
    export HIL_MAPS_DIR="/home/ros2/rob_box_hil/maps_${RUN_ID}_r$ATTEMPT"
    mkdir -p "$ART"
    ON_ATTEMPTED=0
    CLEANED=0
    echo "ATTEMPT=$ATTEMPT/3 RUN_ID=${RUN_ID}_r$ATTEMPT START=$(date -u +%FT%TZ)" | tee -a "$SUMMARY"
    if main "test -e /tmp/hil.lock"; then
        echo "PREEXISTING_HIL_LOCK attempt=$ATTEMPT/3" | tee -a "$SUMMARY"
        exit 20
    fi
    if ! main "test ! -e '$HIL_MAPS_DIR'"; then
        echo "MAP_DIR_NOT_FRESH attempt=$ATTEMPT/3" | tee -a "$SUMMARY"
        exit 21
    fi
    if ! main "test -f '$DIAG_LIB'"; then
        echo "DIAGNOSTIC_LIB_MISSING attempt=$ATTEMPT/3 path=$DIAG_LIB" | tee -a "$SUMMARY"
        exit 22
    fi
    bash "$HIL_CMD" status >"$ART/hil_status_before.log" 2>&1 || exit 23
    ON_ATTEMPTED=1
    bash "$HIL_CMD" on >"$ART/hil_on.log" 2>&1 || exit 24
    scp -q -o BatchMode=yes "$HERE/nav2_pathcalc.compose.yaml" "$MAIN:/tmp/hil/nav2_pathcalc.yaml" || exit 30
    main "sed 's/--ros-args/--ros-args --log-level DEBUG/g' /home/ros2/rob_box_project/docker/main/scripts/nav2/start_nav2_direct.sh > /tmp/hil/start_nav2_pathcalc.sh && chmod +x /tmp/hil/start_nav2_pathcalc.sh" || exit 30
    main "cd /home/ros2/rob_box_project/docker/main && HIL_MAPS_DIR='$HIL_MAPS_DIR' HIL_RTABMAP_MODE='$HIL_RTABMAP_MODE' PATHCALC_DIAG_LIB='$DIAG_LIB' docker compose -f docker-compose.yaml -f /tmp/hil/main.images.yaml -f /tmp/hil/main.hil.yaml -f /tmp/hil/nav2_pathcalc.yaml up -d --no-deps --pull never nav2" >"$ART/debug_enable.log" 2>&1 || exit 30
    sleep 10
    if ! check_lifecycle; then
        echo "LIFECYCLE_NOT_ACTIVE attempt=$ATTEMPT/3; M1_NOT_SENT" | tee -a "$SUMMARY"
        exit 33
    fi
    main "docker exec nav2 rm -f $SNAPSHOT_PATH $SNAPSHOT_PATH.tmp" || exit 34
    sandbox "mkdir -p '$REMOTE_OUT'; docker exec -d rob-box-hil-gazebo /entrypoint.sh bash -c 'echo \$\$ >$CONTAINER_OUT/recorder.pid; exec ros2 bag record -o $CONTAINER_OUT/bag /rosout /evaluation /scan /odom /diff_drive_controller/odom /tf /tf_static /rtabmap/info /rtabmap/odom_info /rtabmap/map /sim/ground_truth/odom /plan /cmd_vel /cmd_vel_emergency /global_costmap/costmap_raw /local_costmap/costmap_raw >$CONTAINER_OUT/bag.log 2>&1'" || exit 31
    sleep 8
    sandbox "test -d '$REMOTE_OUT/bag' && echo BAG_RECORDING_DIRECTORY_PRESENT; tail -20 '$REMOTE_OUT/bag.log'" >"$ART/bag_start.log" 2>&1 || exit 31
    sandbox "cd '$SANDBOX_REPO' && CAMERAS=0 ./runvideo.sh M1" >"$ART/runvideo.log" 2>&1 &
    ROUTE_SSH_PID=$!
    main "timeout $MONITOR_TIMEOUT docker logs -f nav2 2>&1 | grep -m1 'Failed to create a plan from potential'" >"$ART/first_pathcalc_trigger.log" 2>&1
    MONITOR_RC=$?
    echo "MONITOR_RC=$MONITOR_RC $(date -u +%FT%TZ)" >"$ART/monitor_verdict.log"
    stop_route
    wait "$ROUTE_SSH_PID" 2>/dev/null || true
    if [ "$MONITOR_RC" -eq 0 ]; then
        capture_pathcalc_snapshot || {
            echo "PATHCALC_SNAPSHOT_CAPTURE_FAILED attempt=$ATTEMPT/3" | tee -a "$SUMMARY"
            exit 41
        }
        capture_pathcalc || {
            echo "COSTMAP_CAPTURE_FAILED attempt=$ATTEMPT/3" | tee -a "$SUMMARY"
            exit 40
        }
        copy_route_artifacts
        echo "EVIDENCE_CAPTURED attempt=$ATTEMPT/3; M1_STOPPED; ROUTE_NOT_EVALUATED" | tee "$ART/verdict.log" | tee -a "$SUMMARY"
        exit 0
    fi
    copy_route_artifacts
    echo "NO_PATHCALC_FAILURE attempt=$ATTEMPT/3" | tee "$ART/verdict.log" | tee -a "$SUMMARY"
    cleanup_attempt || exit 50
done

echo "EVIDENCE_INSUFFICIENT recurrences=0/3; DEBUG_OR_TIMING_OR_LIFECYCLE_UNRESOLVED" | tee -a "$SUMMARY"
exit 4
