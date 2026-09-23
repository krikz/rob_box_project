#!/bin/bash
# Repro для issue #2805 — изолируем robot_ros() локально, ДОСЛОВНО из харнесса.
# Определение ВЗЯТО из .github/workflows/scripts/e2e_voice_test.sh:353-355 (без правок).
set +e

ROBOT_SSH='cat <<EOF'

robot_ros() {
    ${ROBOT_SSH} "docker exec voice-assistant bash -lc 'source /opt/ros/humble/setup.bash; source /ws/install/setup.bash; $*'"
}

# Раздельный вызов без run_case-shift — вызываем robot_ros напрямую, как в харнессе.
echo "============================================================"
echo "CASE 1: текущий контракт — один строковый аргумент без shell-метасимволов"
echo "  call: robot_ros \"ros2 param get /speaker_id_node e2e_mode --no-daemon\""
echo "  ---- итоговая команда, которая уехала бы на робота ----"
robot_ros "ros2 param get /speaker_id_node e2e_mode --no-daemon"
echo "============================================================"

echo "CASE 2: один аргумент С апострофом — синтаксическая ошибка bash"
echo "  call: robot_ros \"echo it's broken\""
echo "  ---- итоговая команда ----"
robot_ros "echo it's broken"
echo "============================================================"

echo "CASE 3: аргумент с апострофом + пробелы (должно работать, но НЕ работает)"
echo "  call: robot_ros \"echo hello it's me\""
echo "  ---- итоговая команда ----"
robot_ros "echo hello it's me"
echo "============================================================"

echo "CASE 4: путь с апострофом (баг проявится при любой будущей правке)"
echo "  call: robot_ros \"ls /tmp/foo's bar/\""
echo "  ---- итоговая команда ----"
robot_ros "ls /tmp/foo's bar/"
echo "============================================================"

echo "CASE 5: shell-инъекция через \$(...) — выполнится на удалённой стороне"
echo "  call: robot_ros 'echo \$(whoami) secret'"
echo "  ---- итоговая команда ----"
robot_ros 'echo $(whoami) secret'
echo "============================================================"
