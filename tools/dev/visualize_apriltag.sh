#!/bin/bash
# Визуализация AprilTag детекций с наложением на видео

echo "🎨 === AprilTag Визуализация ==="
echo ""

# Source ROS2
source /opt/ros/lyrical/setup.bash

# Настройка Zenoh middleware
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ROS_DOMAIN_ID=0
export RUST_LOG=zenoh=warn
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET

# Zenoh config
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
export ZENOH_SESSION_CONFIG_URI="$SCRIPT_DIR/zenoh_client_config.json5"

echo "✓ ROS2 Humble sourced"
echo "✓ RMW: rmw_zenoh_cpp"
echo "✓ Подключение к Vision Pi"
echo ""
echo "Запуск визуализации..."
echo "Нажмите 'q' в окне изображения для выхода"
echo ""

# Запуск Python скрипта
python3 "$SCRIPT_DIR/visualize_apriltag.py"
