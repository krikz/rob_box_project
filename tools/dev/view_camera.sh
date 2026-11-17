#!/bin/bash
# Просмотр изображения с камеры через zenoh middleware

echo "🎥 === Подключение к камере через Zenoh ==="
echo ""

# Source ROS2
source /opt/ros/lyrical/setup.bash

# Настройка Zenoh middleware
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ROS_DOMAIN_ID=0
export RUST_LOG=zenoh=info
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET

# Zenoh config
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
export ZENOH_SESSION_CONFIG_URI="$SCRIPT_DIR/zenoh_client_config.json5"

echo "✓ ROS2 Humble sourced"
echo "✓ RMW: rmw_zenoh_cpp"
echo "✓ Zenoh config: $ZENOH_SESSION_CONFIG_URI"
echo "✓ Connecting to: 10.1.1.21:7447, 10.1.1.10:7447"
echo ""
echo "Проверка топиков..."
ros2 topic list | grep -i image | head -10
echo ""
echo "Запуск rqt_image_view..."
echo ""

# Запуск rqt_image_view
ros2 run rqt_image_view rqt_image_view
