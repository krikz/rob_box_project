#!/bin/bash
# Запуск RViz2 на локальной машине с Zenoh middleware

echo "🎨 === RViz2 с Zenoh Middleware ==="
echo ""

# Source ROS2
source /opt/ros/humble/setup.bash

# Установка Zenoh middleware
export RMW_IMPLEMENTATION=rmw_zenoh_cpp

# Путь к конфигурации Zenoh (подключение к роботу)
export ZENOH_SESSION_CONFIG_URI="$(dirname "$0")/zenoh_client_config.json5"

echo "✓ ROS2 Humble sourced"
echo "✓ RMW: $RMW_IMPLEMENTATION"
echo "✓ Zenoh config: $ZENOH_SESSION_CONFIG_URI"
echo ""
echo "Подключение к Main Pi (10.1.1.10) и Vision Pi (10.1.1.21)..."
echo ""

# Запуск RViz2
echo "Запуск RViz2..."
echo "Доступные топики и TF будут от робота через Zenoh"
echo ""

rviz2
