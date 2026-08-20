#!/bin/bash
# Мониторинг AprilTag детекций через zenoh

echo "🏷️  === Мониторинг AprilTag детекций ==="
echo ""

# Source ROS2
source /opt/ros/humble/setup.bash

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
echo "Мониторинг детекций..."
echo "─────────────────────────────────────────────"

# Простой вывод - показываем ID и координаты
ros2 topic echo /detections 2>/dev/null | grep -E "(id:|decision_margin:|translation:|  x:|  y:|  z:)" | \
awk '
  /id:/ { printf "\n🎯 TAG "; id=$2; printf "%s | ", id }
  /decision_margin:/ { margin=$2; printf "Quality: %.1f | ", margin }
  /translation:/ { getline; x=$2; getline; y=$2; getline; z=$2; printf "Dist: %.2fm (%.2f, %.2f, %.2f)", sqrt(x*x+y*y+z*z), x, y, z }
'
