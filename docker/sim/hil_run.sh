#!/bin/bash
# Gazebo в HIL-режиме под сторожем графа (эпик #4371, план
# docs/research/2026-11-04-hil-robot-on-gazebo-data.md §5.1).
#
# Команда сервиса gazebo в docker/sim/compose.hil.yaml (после ros_with_namespace.sh):
#   1. precheck — в графе робота нет ни одного /controller_manager, а RSP Main Pi
#      виден (линк Zenoh и ROBOT_ID совпадают). Иначе выход 2, Gazebo не стартует;
#   2. ros2 launch rob_box_sim sim.launch.py hil:=true <аргументы скрипта>;
#   3. watch — каждые HIL_GUARD_INTERVAL с; если /controller_manager стало два
#      (поднялся ros2-control робота), launch гасится и контейнер выходит с кодом 3.
#      restart: "no" — сам Gazebo обратно не поднимется.
#
# Переменные:
#   HIL_GUARD_INTERVAL   период watch, с (5)
#   HIL_GUARD_NO_LINK    1 — precheck без проверки линка с роботом. Только для
#                        проверки сим-части на песочнице без робота.
set -uo pipefail

interval="${HIL_GUARD_INTERVAL:-5}"
case "$interval" in
    "" | *[!0-9.]*) echo "❌ HIL_GUARD_INTERVAL='$interval' — нужно число секунд" >&2; exit 2 ;;
esac
precheck_args=()
case "${HIL_GUARD_NO_LINK:-0}" in
    0) ;;
    1) precheck_args=(--no-link-check)
       echo "⚠️  HIL_GUARD_NO_LINK=1: линк с роботом не проверяется (песочница без робота)" ;;
    *) echo "❌ HIL_GUARD_NO_LINK='${HIL_GUARD_NO_LINK}' — допустимо 0 или 1" >&2; exit 2 ;;
esac

echo "🛡️  HIL precheck: ROBOT_ID=${ROBOT_ID:-?}"
python3 -m rob_box_sim.hil precheck "${precheck_args[@]}" || exit $?

ros2 launch rob_box_sim sim.launch.py hil:=true "$@" &
launch_pid=$!
python3 -m rob_box_sim.hil watch --interval "$interval" &
guard_pid=$!

# Фоновые задачи неинтерактивного bash игнорируют SIGINT, поэтому launch гасим
# SIGTERM. Не вышел за 20 с — выходим сами: скрипт — PID 1 контейнера, с его
# выходом docker гасит и ign gazebo, и спаунеры.
stop_launch() {
    kill -TERM "$launch_pid" 2>/dev/null
    for _ in $(seq 1 40); do
        kill -0 "$launch_pid" 2>/dev/null || { wait "$launch_pid" 2>/dev/null; return 0; }
        sleep 0.5
    done
    echo "⚠️  launch не вышел за 20 с после SIGTERM — выхожу, docker погасит остальное" >&2
}
trap 'kill "$guard_pid" 2>/dev/null; stop_launch; exit 143' TERM INT

wait -n "$launch_pid" "$guard_pid"
rc=$?
if ! kill -0 "$guard_pid" 2>/dev/null; then
    echo "🛑 HIL guard: нарушение (код $rc) — гашу Gazebo" >&2
    stop_launch
    exit 3
fi
echo "Gazebo launch завершился (код $rc), сторож останавливается" >&2
kill "$guard_pid" 2>/dev/null
wait "$guard_pid" 2>/dev/null
exit "$rc"
