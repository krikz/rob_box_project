#!/bin/bash
# Healthcheck контейнера nav2 (#5043, #5150).
#
# pgrep -f bt_navigator зеленел, когда controller_server уже умер (SIGILL, #5039)
# или завис на configure: bt_navigator запускается отдельной командой и переживает
# контроллер. Спрашиваем lifecycle трёх узлов, без которых навигации нет.
#
# #5150: три последовательных `ros2 lifecycle get` на Main Pi стоят 10.5–16.8 с
# (замер 23.11: 16.76 с, замер Шифу 16.025 с) при docker Timeout 10 с — живые
# active-узлы красились unhealthy. Измерено: такие CLI жгут ядра (100 % и
# 72.7 % CPU на двух процессах сразу). Поэтому здесь ОДИН процесс rclpy
# спрашивает три сервиса /get_state под общим дедлайном короче bond-ориентира
# 4 с (числа — healthcheck_budget.py). Решение Шифу 23.11: Timeout не поднимать,
# проба обязана уложиться в текущие 10 с. Любой не-active, нет ответа или
# зависший вызов — ненулевой код и причина в stderr.

set -u

# ROS_SETUP переопределяется только тестом. На роботе пусто → humble.
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
if [ ! -f "$ROS_SETUP" ]; then
    echo "nav2 healthcheck: ${ROS_SETUP} not found" >&2
    exit 1
fi

# Каталог этого скрипта: probe и бюджет лежат рядом и монтируются тем же томом.
SCRIPT_DIR=$(CDPATH='' cd -- "$(dirname -- "$0")" && pwd)

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

# PYTHONPATH теста указывает на подменный rclpy. Каталог скрипта добавляем
# в конец, чтобы не заслонить подмену, но probe всё равно импортировался.
export PYTHONPATH="${PYTHONPATH:+${PYTHONPATH}:}${SCRIPT_DIR}"

exec python3 -m lifecycle_probe
