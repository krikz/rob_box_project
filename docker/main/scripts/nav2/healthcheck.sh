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

# set -u стоит выше. /opt/ros/humble/setup.bash (ament) читает
# $AMENT_TRACE_SETUP_FILES без проверки на заданность; под set -u неинтерактивный
# bash умирает на этой строке, и healthcheck вечно красный. Снимаем nounset
# только на время source.
# `exit N` внутри sourced-файла убивает эту оболочку целиком — ERR-ловушка его
# не отменяет (проверено на bash 5.1). source в подоболочке, чтобы поймать rc;
# переменные, которые выставил setup, забираем diff'ом окружения.
set +u
src_env=$(mktemp)
# shellcheck disable=SC1090
(
    source "$ROS_SETUP"
    src_rc=$?
    export -p >"$src_env"
    exit "$src_rc"
)
src_rc=$?
set -u
if [ "$src_rc" -ne 0 ]; then
    rm -f "$src_env"
    echo "nav2 healthcheck: source ${ROS_SETUP} failed rc=${src_rc}" >&2
    exit 1
fi
# shellcheck disable=SC1090
source "$src_env"
rm -f "$src_env"
# nounset мог прийти из setup; нам он нужен для проверок ниже.
set -u

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
