#!/usr/bin/env bash
# Offline-тест diagnostic-сценария #4682: ssh/scp/hil.sh подменены, робот не вызывается.
# check вычисляет выражение через eval: одинарные кавычки вокруг $VAR намеренные.
# shellcheck disable=SC2016
set -uo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
DIAG="$REPO_ROOT/scripts/hil/diagnostics/run_depot_pathcalc.sh"
WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT
pass=0
failures=0
ok() { echo "PASS: $1"; pass=$((pass + 1)); }
ko() { echo "FAIL: $1"; failures=$((failures + 1)); }
check() { if eval "$2"; then ok "$1"; else ko "$1"; fi; }

if [ ! -f "$DIAG" ]; then
    ko "diagnostic script exists: $DIAG"
    echo "итог: $pass PASS, $failures FAIL"
    exit 1
fi

mkdir -p "$WORK/bin"
cat >"$WORK/bin/ssh" <<'EOF'
#!/usr/bin/env bash
while [ "$#" -gt 0 ]; do
    case "$1" in -o) shift 2 ;; *) break ;; esac
done
host="$1"; shift
cmd="$*"
echo "$host|$cmd" >>"$FAKE_SSH_LOG"
case "$cmd" in
    "test -e /tmp/hil.lock") [ "${FAKE_PREEXISTING_LOCK:-0}" = 1 ] && exit 0 || exit 1 ;;
    "test ! -e /tmp/hil.lock") [ "${FAKE_LOCK_LEFT:-0}" = 1 ] && exit 1 || exit 0 ;;
    "test ! -e /tmp/hil/nav2_pathcalc.yaml"*) [ "${FAKE_TEMP_OVERLAY_LEFT:-0}" = 1 ] && exit 1 || exit 0 ;;
    "test ! -e "*maps_depot4682*) exit 0 ;;
    "test -f "*libnav2_navfn_planner.so) exit 0 ;;
    *"ros2 lifecycle get /smoother_server"*)
        [ "${FAKE_LIFECYCLE:-active}" = inactive ] && echo 'inactive [2]' || echo 'active [3]' ;;
    *"ros2 lifecycle get /"*) echo 'active [3]' ;;
    *"ros2 service list"*) echo '/global_costmap/get_costmap'; echo '/planner_server/set_logger_levels'; echo '/controller_server/set_logger_levels' ;;
    *"runvideo.sh M1"*)
        n=$(cat "$FAKE_STATE/routes" 2>/dev/null || echo 0); echo $((n + 1)) >"$FAKE_STATE/routes" ;;
    *"docker logs -f nav2"*)
        n=$(cat "$FAKE_STATE/monitors" 2>/dev/null || echo 0); n=$((n + 1)); echo "$n" >"$FAKE_STATE/monitors"
        if [ "$n" = "${FAKE_FAIL_AT:-never}" ]; then
            echo '[ERROR] [planner_server]: Failed to create a plan from potential when a legal potential was found'
            exit 0
        fi
        exit 1 ;;
    *"ros2 service call /global_costmap/get_costmap"*)
        echo 'REQUEST_UTC=2026-11-12T00:00:00Z'; echo 'REQUEST_ROS'; echo 'sec: 120'
        echo 'response: map_load_time={sec: 100} update_time={sec: 120}'
        echo 'RESPONSE_UTC=2026-11-12T00:00:01Z'; echo 'RESPONSE_ROS'
        [ "${FAKE_MISSING_TIMESTAMP:-0}" = 1 ] || echo 'sec: 121' ;;
    *"docker logs nav2"*)
        echo 'calcPath begin stc=579379 start=(379,579) goal=(499,499)'
        echo 'best_pose=(-6,4) potarr=23935.889 reason=potential'
        echo 'Failed to create a plan from potential when a legal potential was found' ;;
    *"Config.Env"*)
        echo 'PATH=/x'
        [ "${FAKE_OVERLAY_LEFT:-0}" = 1 ] && echo 'NAV2_PARAMS_OVERLAYS=/nav2tune/debug.yaml'
        [ "${FAKE_SIM_TIME_LEFT:-0}" = 1 ] && echo 'USE_SIM_TIME=true' ;;
    *"State.Status}}' rob-box-hil-gazebo"*) echo absent ;;
    *"State.Running}}' rob-box-hil-gazebo"*) echo true ;;
    *"ros2 bag info"*) echo 'Files: bag.db3'; echo 'Topics: /evaluation /rosout /plan /cmd_vel' ;;
    *) ;;
esac
exit 0
EOF
cat >"$WORK/bin/scp" <<'EOF'
#!/usr/bin/env bash
echo "scp $*" >>"$FAKE_SSH_LOG"
exit 0
EOF
cat >"$WORK/bin/sleep" <<'EOF'
#!/usr/bin/env bash
exit 0
EOF
cat >"$WORK/bin/hil" <<'EOF'
#!/usr/bin/env bash
echo "hil $*" >>"$FAKE_HIL_LOG"
case "${1:-}" in
    off)
        if [ "${FAKE_REGISTRY_BAD:-0}" = 1 ]; then
            echo 'боевой реестр карт НЕ сверен'
        else
            echo 'боевой реестр карт (SELECT * FROM maps) за HIL не изменился'
        fi
        exit "${FAKE_OFF_RC:-0}" ;;
    *) exit 0 ;;
esac
EOF
chmod +x "$WORK/bin/ssh" "$WORK/bin/scp" "$WORK/bin/sleep" "$WORK/bin/hil"

run_diag() {  # run_diag <case-name> [VAR=value ...]
    local name="$1"; shift
    rm -rf "$WORK/state" "$WORK/out-$name"
    mkdir -p "$WORK/state"
    : >"$WORK/ssh-$name.log"; : >"$WORK/hil-$name.log"
    env PATH="$WORK/bin:$PATH" FAKE_SSH_LOG="$WORK/ssh-$name.log" FAKE_HIL_LOG="$WORK/hil-$name.log" \
        FAKE_STATE="$WORK/state" HIL_CMD="$WORK/bin/hil" PATHCALC_OUT="$WORK/out-$name" \
        PATHCALC_RUN_ID="depot4682_test_$name" PATHCALC_MONITOR_TIMEOUT=1 PATHCALC_LIFECYCLE_ATTEMPTS=1 \
        "$@" bash "$DIAG" >"$WORK/stdout-$name.log" 2>&1
}

# Чужой HIL: никаких действий, даже остановки route.py; чужой off не выполняется.
run_diag locked FAKE_PREEXISTING_LOCK=1; rc=$?
check "preexisting lock: выход 20 (rc=$rc)" '[ "$rc" -eq 20 ]'
check "preexisting lock: ни M1, ни pkill, ни off" \
    '! grep -Eq "runvideo.sh M1|pkill -INT" "$WORK/ssh-locked.log" && ! grep -q "hil off" "$WORK/hil-locked.log"'

# inactive: цель и bag не запускаются; off всё равно обязателен после on.
run_diag inactive FAKE_LIFECYCLE=inactive; rc=$?
check "inactive: выход 33 (rc=$rc)" '[ "$rc" -eq 33 ]'
check "inactive: все 4 lifecycle-ноды проверены" \
    'for n in bt_navigator planner_server controller_server smoother_server; do grep -q "ros2 lifecycle get /$n" "$WORK/ssh-inactive.log" || exit 1; done'
check "inactive: M1 и bag не отправлены" \
    '! grep -q "runvideo.sh M1" "$WORK/ssh-inactive.log" && ! grep -q "ros2 bag record" "$WORK/ssh-inactive.log"'
check "inactive: off и честный lifecycle verdict" \
    'grep -q "hil off" "$WORK/hil-inactive.log" && grep -q "LIFECYCLE_NOT_ACTIVE" "$WORK/out-inactive/verdict.log"'

# Ни одной ошибки: ровно 3 свежие попытки, каждая выключена; никакого PASS.
run_diag none; rc=$?
check "0/3: выход 4 EVIDENCE_INSUFFICIENT (rc=$rc)" \
    '[ "$rc" -eq 4 ] && grep -q "EVIDENCE_INSUFFICIENT recurrences=0/3" "$WORK/out-none/verdict.log"'
check "0/3: ровно 3 M1 и 3 on/off" \
    '[ "$(grep -c "runvideo.sh M1" "$WORK/ssh-none.log")" -eq 3 ] && [ "$(grep -c "hil on" "$WORK/hil-none.log")" -eq 3 ] && [ "$(grep -c "hil off" "$WORK/hil-none.log")" -eq 3 ]'
check "0/3: нет ложного route PASS и R5 не запускается" \
    '! grep -Eq "ROUTE_PASS|VERDICT PASS|runvideo.sh.*R5" "$WORK/out-none/verdict.log" "$WORK/ssh-none.log"'

# Ошибка на второй попытке: стоп до costmap, два timestamp, серия прекращается.
run_diag captured FAKE_FAIL_AT=2; rc=$?
check "recurrence 2: evidence captured, выход 0 (rc=$rc)" \
    '[ "$rc" -eq 0 ] && grep -q "EVIDENCE_CAPTURED attempt=2/3" "$WORK/out-captured/verdict.log"'
check "recurrence 2: ровно 2 M1, третьей попытки нет" '[ "$(grep -c "runvideo.sh M1" "$WORK/ssh-captured.log")" -eq 2 ]'
ART="$WORK/out-captured/depot4682_test_captured_r2"
check "recurrence 2: stop_route предшествует get_costmap" \
    'stop=$(grep -n -m1 "pkill -INT -f.*route.py" "$WORK/ssh-captured.log" | cut -d: -f1); cost=$(grep -n -m1 "ros2 service call /global_costmap/get_costmap" "$WORK/ssh-captured.log" | cut -d: -f1); [ -n "$stop" ] && [ -n "$cost" ] && [ "$stop" -lt "$cost" ]'
check "recurrence 2: request/response UTC и ROS timestamps сохранены" \
    'for marker in REQUEST_UTC REQUEST_ROS RESPONSE_UTC RESPONSE_ROS; do grep -q "$marker" "$ART/get_costmap_error.log" || exit 1; done'
check "recurrence 2: calcPath reason и bag info сохранены" \
    'grep -q "best_pose=.*potarr=.*reason=" "$ART/calcPath_error.log" && grep -q "/evaluation" "$ART/bag_info.log"'
run_diag incomplete FAKE_FAIL_AT=1 FAKE_MISSING_TIMESTAMP=1; rc=$?
check "incomplete timestamps: нет EVIDENCE_CAPTURED, выход 40 (rc=$rc)" \
    '[ "$rc" -eq 40 ] && grep -q "COSTMAP_CAPTURE_FAILED" "$WORK/out-incomplete/verdict.log" && ! grep -q "EVIDENCE_CAPTURED" "$WORK/out-incomplete/verdict.log"'
check "recurrence 2: обязательные lock/overlay/registry/sim-time проверки выполнены" \
    'grep -q "test ! -e /tmp/hil.lock" "$WORK/ssh-captured.log" && grep -q "Config.Env" "$WORK/ssh-captured.log" && grep -q "реестр карт.*не изменился" "$ART/hil_off.log" && grep -q "CLEANUP_OK" "$WORK/out-captured/verdict.log"'

# Даже пойманная ошибка не маскирует неубранный production overlay.
run_diag dirty FAKE_FAIL_AT=1 FAKE_OVERLAY_LEFT=1; rc=$?
check "dirty overlay: diagnostic evidence не даёт PASS, выход 50 (rc=$rc)" \
    '[ "$rc" -eq 50 ] && grep -q "CLEANUP_FAILED" "$WORK/out-dirty/verdict.log"'
run_diag dirtytemp FAKE_FAIL_AT=1 FAKE_TEMP_OVERLAY_LEFT=1; rc=$?
check "dirty temp overlay: остаток /tmp/hil даёт выход 50 (rc=$rc)" \
    '[ "$rc" -eq 50 ] && grep -q "CLEANUP_FAILED" "$WORK/out-dirtytemp/verdict.log"'

bash -n "$DIAG"
check "bash -n diagnostic script" '[ "$?" -eq 0 ]'

echo
echo "итог: $pass PASS, $failures FAIL"
[ "$failures" -eq 0 ]
