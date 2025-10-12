#!/bin/bash
# Быстрый запуск rtabmap локально БЕЗ Docker
# Для тестирования разных конфигураций

# Настройка ROS 2
source /opt/ros/humble/setup.bash

# Выбор middleware
echo "Выберите middleware:"
echo "1. Zenoh (как на Raspberry Pi)"
echo "2. CycloneDDS (по умолчанию)"
echo ""
read -p "Выбор (1/2): " middleware

if [ "$middleware" == "1" ]; then
    export RMW_IMPLEMENTATION=rmw_zenoh_cpp
    export ZENOH_CONFIG="$(pwd)/local_test/zenoh_local_config.json5"
    export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
    echo "✅ Используем Zenoh (peer mode)"
else
    export RMW_IMPLEMENTATION=""
    unset ZENOH_CONFIG
    unset ROS_AUTOMATIC_DISCOVERY_RANGE
    echo "✅ Используем CycloneDDS"
fi

echo "========================================="
echo " RTAB-Map Local Test (NO DOCKER)"
echo "========================================="
echo ""
echo "Режимы:"
echo "1. Тест с симулятором лидара"
echo "2. Тест с реальным лидаром"
echo "3. Только rtabmap.launch.py"
echo ""
read -p "Выбор (1/2/3): " mode

# TF: base_link -> laser
ros2 run tf2_ros static_transform_publisher 0 0 0.2 0 0 0 base_link laser &
TF_PID=$!

if [ "$mode" == "1" ]; then
    echo "Запуск симулятора лидара..."
    python3 local_test/fake_lidar_publisher.py &
    LIDAR_PID=$!
    sleep 2
fi

echo ""
echo "Запуск rtabmap.launch.py с 2D лидаром..."
echo ""

ros2 launch rtabmap_launch rtabmap.launch.py \
    args:="--delete_db_on_start" \
    use_sim_time:=false \
    frame_id:=base_link \
    odom_frame_id:=odom \
    rgb_topic:=/null/rgb \
    depth_topic:=/null/depth \
    camera_info_topic:=/null/camera_info \
    scan_topic:=/scan \
    qos:=0 \
    approx_sync:=false \
    rtabmap_viz:=true \
    rviz:=false \
    visual_odometry:=false \
    icp_odometry:=true \
    odom_topic:=odom \
    database_path:=/tmp/rtabmap_test.db \
    wait_for_transform:=0.5

# Cleanup
kill $TF_PID 2>/dev/null || true
kill $LIDAR_PID 2>/dev/null || true
