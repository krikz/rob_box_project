#!/usr/bin/env bash
# Регресс предохранителей «цель за стеной» (#5003).
# Робота и docker нет: ssh подменён фейком в PATH.
#   bash scripts/hil/tests/test_goal_behind_wall.sh
set -uo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
SCENE="$REPO_ROOT/scripts/hil/goal_behind_wall/goal_behind_wall.sh"
WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT

mkdir -p "$WORK/bin"
cat >"$WORK/bin/ssh" <<'ENDSSH'
#!/usr/bin/env bash
while [ "$#" -gt 0 ]; do
    case "$1" in
        -o) shift 2 ;;
        *) break ;;
    esac
done
host="$1"
shift
printf '%s|%s\n' "$host" "$*" >>"${FAKE_SSH_LOG:?}"
case "$*" in
    *"ros2-control"*)
        if [ -n "${FAKE_ROS2_CONTROL+x}" ]; then
            printf '%s\n' "$FAKE_ROS2_CONTROL"
        else
            printf '%s\n' exited
        fi
        ;;
    *"State.Status"*)
        if [ -n "${FAKE_GAZEBO+x}" ]; then
            printf '%s\n' "$FAKE_GAZEBO"
        else
            printf '%s\n' running
        fi
        ;;
    *"ros2 topic info"*)
        printf '%s\n' "${FAKE_GRAPH}"
        ;;
    *"candump"*)
        if [ -n "${FAKE_CAN+x}" ]; then
            printf '%s\n' "$FAKE_CAN"
        fi
        ;;
    *)
        printf 'unexpected ssh: %s\n' "$*" >&2
        exit 9
        ;;
esac
ENDSSH
chmod +x "$WORK/bin/ssh"

export PATH="$WORK/bin:$PATH"
export FAKE_SSH_LOG="$WORK/ssh.log"
export HIL_CONF="$REPO_ROOT/scripts/hil/hil.conf"
export HIL_SIM_WORLD=dead_end
export HIL_GBW_CAN_SECS=1
export FAKE_ROS2_CONTROL=exited
export FAKE_GAZEBO=running
export FAKE_GRAPH=$'Subscription count: 1\nCM_MARK\n/controller_manager'
export FAKE_CAN='(1.0) can0 904#00000000'

pass=0
fail=0
ok() { printf 'PASS: %s\n' "$1"; pass=$((pass + 1)); }
ko() { printf 'FAIL: %s\n' "$1"; fail=$((fail + 1)); }

run_scene() {
    : >"$FAKE_SSH_LOG"
    bash "$SCENE" >"$WORK/out" 2>"$WORK/err"
    rc=$?
    return 0
}

run_scene
if [ "$rc" -eq 0 ] && grep -q 'изоляция есть' "$WORK/out"; then
    ok "изоляция есть: выход 0"
else
    ko "изоляция есть: rc=$rc out=$(cat "$WORK/out") err=$(cat "$WORK/err")"
fi
if grep -q 'hil.sh on' "$FAKE_SSH_LOG"; then ko "сценарий сам вызвал hil.sh"; else ok "hil.sh не вызывается"; fi

FAKE_ROS2_CONTROL=running run_scene
if [ "$rc" -eq 2 ] && grep -q 'ros2-control работает' "$WORK/err"; then ok "ros2-control running: выход 2"; else ko "ros2-control running: rc=$rc err=$(cat "$WORK/err")"; fi
FAKE_ROS2_CONTROL=exited

FAKE_ROS2_CONTROL="" run_scene
if [ "$rc" -eq 2 ] && grep -q 'не доказано' "$WORK/err"; then ok "ros2-control не доказан: выход 2"; else ko "не доказан: rc=$rc err=$(cat "$WORK/err")"; fi
FAKE_ROS2_CONTROL=exited

FAKE_GAZEBO=exited run_scene
if [ "$rc" -eq 2 ] && grep -q 'gazebo не работает' "$WORK/err"; then ok "gazebo погашен: выход 2"; else ko "gazebo: rc=$rc err=$(cat "$WORK/err")"; fi
FAKE_GAZEBO=running

FAKE_GRAPH=$'Subscription count: 1\nCM_MARK\n/controller_manager\n/controller_manager' run_scene
if [ "$rc" -eq 2 ] && grep -q 'в графе: 2' "$WORK/err"; then ok "два controller_manager: выход 2"; else ko "два cm: rc=$rc err=$(cat "$WORK/err")"; fi

FAKE_GRAPH=$'Subscription count: 2\nCM_MARK\n/controller_manager' run_scene
if [ "$rc" -eq 2 ] && grep -q 'подписчиков: 2' "$WORK/err"; then ok "два подписчика: выход 2"; else ko "два подписчика: rc=$rc err=$(cat "$WORK/err")"; fi
FAKE_GRAPH=$'Subscription count: 1\nCM_MARK\n/controller_manager'

FAKE_CAN='(1.0) can0 100#00000000' run_scene
if [ "$rc" -eq 2 ] && grep -q 'командных кадров VESC' "$WORK/err"; then ok "кадр VESC: выход 2"; else ko "кадр VESC: rc=$rc err=$(cat "$WORK/err")"; fi

FAKE_CAN='(1.0) can0 904#00000000' run_scene
if [ "$rc" -eq 0 ]; then ok "статус VESC не команда"; else ko "статус VESC: rc=$rc err=$(cat "$WORK/err")"; fi

FAKE_CAN="" run_scene
if [ "$rc" -eq 2 ] && grep -q 'can0 пуст' "$WORK/err"; then ok "пустой can0: выход 2"; else ko "пустой can0: rc=$rc err=$(cat "$WORK/err")"; fi
FAKE_CAN='(1.0) can0 904#00000000'

HIL_SIM_WORLD=polygon run_scene
if [ "$rc" -eq 2 ] && grep -q 'ждали dead_end' "$WORK/err" && [ ! -s "$FAKE_SSH_LOG" ]; then
    ok "мир polygon: выход 2 до ssh"
else
    ko "мир polygon: rc=$rc err=$(cat "$WORK/err")"
fi

if bash -n "$SCENE"; then ok "bash -n сценария"; else ko "bash -n сценария"; fi

printf 'итог: %s pass, %s fail\n' "$pass" "$fail"
[ "$fail" -eq 0 ]
