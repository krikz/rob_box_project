#!/bin/bash
# Nav2 Direct Launch - запуск нодов напрямую без navigation_launch.py

set -e

echo "Starting Nav2 nodes directly..."

# Source ROS2
source /opt/ros/humble/setup.bash

# USE_SIM_TIME=true только в профиле docker/sim (Gazebo): тогда каждому узлу
# добавляется -p use_sim_time:=true поверх nav2_params.yaml. На роботе пусто.
source /ros_scripts/use_sim_time.sh

PARAMS="/config/nav2/nav2_params.yaml"

# Только профиль docker/sim (шаг 6 плана Gazebo). На роботе обе переменные пусты,
# и команды узлов те же (tests/unit/docker/test_main_scripts_use_sim_time.py).
#   NAV2_PARAMS_OVERLAYS — yaml через пробел, кладутся --params-file ПОСЛЕ
#     nav2_params.yaml: одноимённые параметры оверлея сильнее. Копии
#     nav2_params.yaml под сим нет — оверлей меняет только то, что сравнивают.
#   NAV2_SMOOTH_CMD_VEL=true — цепочка как в nav2_bringup navigation_launch.py:
#     controller_server → cmd_vel_nav → velocity_smoother → cmd_vel (twist_mux).
#     Без неё smoother слушает тот же cmd_vel и пишет в никуда.
PARAMS_ARGS=(--params-file "$PARAMS")
read -r -a OVERLAYS <<< "${NAV2_PARAMS_OVERLAYS:-}"
for overlay in "${OVERLAYS[@]}"; do
    if [ ! -f "$overlay" ]; then
        echo "❌ ОШИБКА: оверлей параметров Nav2 '$overlay' не найден" >&2
        exit 1
    fi
    PARAMS_ARGS+=(--params-file "$overlay")
done
# Зоны скорости (решение Шифу 10.11): штатный SpeedFilter Nav2 — оверлей
# /config/nav2/speed_zones.yaml плюс публикаторы маски (ниже). Только когда задан
# NAV2_SPEED_MASK_YAML — путь к yaml маски внутри контейнера. Генератор маски:
# python3 -m rob_box_sim.speed_zones (маска пересчитывается после смены карты).
# Маски нет, а переменная задана — не стартуем: ехать на базовой скорости без
# ограничений зон небезопасно, и это должно быть видно, а не молчаливо.
SPEED_ZONES_ON=false
case "${NAV2_SPEED_MASK_YAML:-}" in
    "") ;;
    *)
        if [ ! -f "$NAV2_SPEED_MASK_YAML" ]; then
            echo "❌ ОШИБКА: NAV2_SPEED_MASK_YAML='$NAV2_SPEED_MASK_YAML' не найден в контейнере" >&2
            exit 1
        fi
        SPEED_ZONES_ON=true
        PARAMS_ARGS+=(--params-file /config/nav2/speed_zones.yaml)
        echo "Зоны скорости: маска '$NAV2_SPEED_MASK_YAML' + оверлей /config/nav2/speed_zones.yaml"
        ;;
esac
CONTROLLER_REMAP=()
SMOOTHER_REMAP=()
case "${NAV2_SMOOTH_CMD_VEL:-false}" in
    false) ;;
    true)
        CONTROLLER_REMAP=(-r cmd_vel:=cmd_vel_nav)
        SMOOTHER_REMAP=(-r cmd_vel:=cmd_vel_nav -r cmd_vel_smoothed:=cmd_vel)
        echo "velocity_smoother в цепочке: cmd_vel_nav → cmd_vel"
        ;;
    *)
        echo "❌ ОШИБКА: NAV2_SMOOTH_CMD_VEL='$NAV2_SMOOTH_CMD_VEL' — только true или false" >&2
        exit 1
        ;;
esac

# Ожидание готовности одометрии (топик /odom публикуется diff_drive_controller
# одновременно с TF odom → base_footprint). Без этого local_costmap при
# активации логирует "Invalid frame ID \"odom\" ... frame does not exist" —
# известный startup race, который деплой-воркфлоу ловит как critical_log.
# На таймауте продолжаем с warning (Nav2 всё равно стартует), чтобы не
# уходить в restart-loop: отсутствие odometry останется видимым в логах.
echo "Waiting for /odom topic (odometry readiness)..."
TIMEOUT=30
ELAPSED=0
while ! ros2 topic list | grep -q "^/odom$"; do
    if [ $ELAPSED -ge $TIMEOUT ]; then
        echo "WARNING: /odom topic not found after ${TIMEOUT}s — starting Nav2 anyway"
        break
    fi
    sleep 1
    ELAPSED=$((ELAPSED + 1))
done
if ros2 topic list | grep -q "^/odom$"; then
    echo "✓ /odom topic found"
fi

# Запуск lifecycle manager в фоне
ros2 run nav2_lifecycle_manager lifecycle_manager \
    --ros-args \
    "${PARAMS_ARGS[@]}" \
    "${SIM_TIME_ROS_ARGS[@]}" \
    -r __node:=lifecycle_manager_navigation &

sleep 2

# Запуск остальных нод
ros2 run nav2_controller controller_server \
    --ros-args \
    "${PARAMS_ARGS[@]}" \
    "${SIM_TIME_ROS_ARGS[@]}" \
    "${CONTROLLER_REMAP[@]}" \
    -r /tf:=tf \
    -r /tf_static:=tf_static &

ros2 run nav2_planner planner_server \
    --ros-args \
    "${PARAMS_ARGS[@]}" \
    "${SIM_TIME_ROS_ARGS[@]}" \
    -r __node:=planner_server \
    -r /tf:=tf \
    -r /tf_static:=tf_static &

ros2 run nav2_bt_navigator bt_navigator \
    --ros-args \
    "${PARAMS_ARGS[@]}" \
    "${SIM_TIME_ROS_ARGS[@]}" \
    -r __node:=bt_navigator \
    -r /tf:=tf \
    -r /tf_static:=tf_static &

ros2 run nav2_behaviors behavior_server \
    --ros-args \
    "${PARAMS_ARGS[@]}" \
    "${SIM_TIME_ROS_ARGS[@]}" \
    -r __node:=behavior_server \
    -r /tf:=tf \
    -r /tf_static:=tf_static &

ros2 run nav2_waypoint_follower waypoint_follower \
    --ros-args \
    "${PARAMS_ARGS[@]}" \
    "${SIM_TIME_ROS_ARGS[@]}" \
    -r __node:=waypoint_follower \
    -r /tf:=tf \
    -r /tf_static:=tf_static &

ros2 run nav2_velocity_smoother velocity_smoother \
    --ros-args \
    "${PARAMS_ARGS[@]}" \
    "${SIM_TIME_ROS_ARGS[@]}" \
    "${SMOOTHER_REMAP[@]}" \
    -r __node:=velocity_smoother \
    -r /tf:=tf \
    -r /tf_static:=tf_static &

ros2 run nav2_smoother smoother_server \
    --ros-args \
    "${PARAMS_ARGS[@]}" \
    "${SIM_TIME_ROS_ARGS[@]}" \
    -r __node:=smoother_server \
    -r /tf:=tf \
    -r /tf_static:=tf_static &

# Публикаторы маски зон скорости: map_server (маска как «карта», mode scale в yaml)
# и costmap_filter_info_server (type=SPEED_FILTER_ABSOLUTE, base 0.0, multiplier 0.01).
# Оба узла lifecycle — поднимает свой lifecycle_manager (autostart).
if [ "$SPEED_ZONES_ON" = true ]; then
    ros2 run nav2_map_server map_server \
        --ros-args \
        "${SIM_TIME_ROS_ARGS[@]}" \
        -r __node:=speed_mask_server \
        -r map:=speed_filter_mask \
        -r /tf:=tf \
        -r /tf_static:=tf_static \
        -p yaml_filename:="$NAV2_SPEED_MASK_YAML" &
    ros2 run nav2_map_server costmap_filter_info_server \
        --ros-args \
        "${SIM_TIME_ROS_ARGS[@]}" \
        -r __node:=costmap_filter_info_server \
        -p type:=2 \
        -p base:=0.0 \
        -p multiplier:=0.01 \
        -p mask_topic:=/speed_filter_mask \
        -p filter_info_topic:=/costmap_filter_info &
    ros2 run nav2_lifecycle_manager lifecycle_manager \
        --ros-args \
        "${SIM_TIME_ROS_ARGS[@]}" \
        -r __node:=lifecycle_manager_speed_mask \
        -p autostart:=true \
        -p node_names:=[speed_mask_server,costmap_filter_info_server] &
fi

# Держим скрипт живым
wait
