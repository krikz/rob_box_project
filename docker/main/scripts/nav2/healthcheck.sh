#!/bin/bash
# Healthcheck контейнера nav2 (#5043).
#
# pgrep -f bt_navigator зеленел, когда controller_server уже умер (SIGILL, #5039)
# или завис на configure: bt_navigator запускается отдельной командой и переживает
# контроллер. Здесь спрашиваем lifecycle трёх узлов, без которых навигации нет.
# Любой не-active, нет ответа или зависший вызов — ненулевой код, docker уходит
# в unhealthy и L: Deploy and Verify не красится зелёным.
#
# Каждый ros2 lifecycle get обёрнут своим timeout: зависший discovery не должен
# съесть весь бюджет healthcheck и оставить docker без кода выхода.

set -u

NODES=(/controller_server /planner_server /bt_navigator)
PER_NODE_TIMEOUT_S=8

# ROS_SETUP переопределяется только тестом. На роботе пусто → humble.
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
if [ ! -f "$ROS_SETUP" ]; then
    echo "nav2 healthcheck: ${ROS_SETUP} not found" >&2
    exit 1
fi

# shellcheck disable=SC1090
source "$ROS_SETUP"

fail=0
for node in "${NODES[@]}"; do
    out=$(timeout "${PER_NODE_TIMEOUT_S}" ros2 lifecycle get "${node}" 2>&1)
    rc=$?
    if [ "$rc" -eq 124 ]; then
        echo "nav2 healthcheck: ${node} no answer within ${PER_NODE_TIMEOUT_S}s" >&2
        fail=1
        continue
    fi
    if [ "$rc" -ne 0 ]; then
        echo "nav2 healthcheck: ${node} ros2 lifecycle get exited ${rc}: ${out}" >&2
        fail=1
        continue
    fi
    # ros2 lifecycle get печатает строку вида "active [3]". Берём первое слово.
    state=$(printf '%s\n' "$out" | head -n 1 | awk '{print $1}')
    if [ "$state" != "active" ]; then
        echo "nav2 healthcheck: ${node} is '${state:-<empty>}', want active" >&2
        fail=1
    fi
done

exit "$fail"
