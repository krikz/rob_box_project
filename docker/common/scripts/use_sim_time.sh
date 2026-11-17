# shellcheck shell=bash disable=SC2034  # SIM_TIME_*_ARGS читают скрипты, которые подключают файл
# Переключатель времени симуляции для боевых стартовых скриптов.
#
# Подключается через `source /ros_scripts/use_sim_time.sh` из скриптов
# docker/main/scripts/{robot_state_publisher,twist_mux,rtabmap,nav2}/.
# Профиль docker/sim (Gazebo) ставит контейнерам USE_SIM_TIME=true, и узлы
# берут время из /clock. На роботе переменная не задана → false, команда
# узла та же, что была до сима (tests/unit/docker/test_main_scripts_use_sim_time.py).
#
# Допустимы только «true» и «false»: опечатка — громкое падение, а не тихое
# «настенное» время в симе или «симовое» на роботе.

USE_SIM_TIME="${USE_SIM_TIME:-false}"
case "$USE_SIM_TIME" in
    true | false) ;;
    *)
        echo "❌ ОШИБКА: USE_SIM_TIME='$USE_SIM_TIME' — допустимо только true или false" >&2
        exit 1
        ;;
esac
export USE_SIM_TIME

# Аргументы ros2 run/launch для режима сима. На роботе массивы пустые.
SIM_TIME_ROS_ARGS=()      # для `ros2 run ... --ros-args`
SIM_TIME_LAUNCH_ARGS=()   # для `ros2 launch ...`
if [ "$USE_SIM_TIME" = "true" ]; then
    SIM_TIME_ROS_ARGS=(-p use_sim_time:=true)
    SIM_TIME_LAUNCH_ARGS=(use_sim_time:=true)
    echo "⏱️  USE_SIM_TIME=true — время из /clock (симуляция)"
fi
