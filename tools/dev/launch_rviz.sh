#!/bin/bash
# Запуск RViz2 на локальной машине с Zenoh middleware

echo "🎨 === RViz2 с Zenoh Middleware ==="
echo ""

# Source ROS2
source /opt/ros/lyrical/setup.bash

# Установка Zenoh middleware
export RMW_IMPLEMENTATION=rmw_zenoh_cpp

# Путь к конфигурации Zenoh (подключение через локальный роутер)
export ZENOH_SESSION_CONFIG_URI="$(dirname "$0")/zenoh_local_session.json5"
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

# Фикс конфликта snap/GTK
unset GTK_PATH GTK_EXE_PREFIX GIO_MODULE_DIR

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

RVIZ_CONFIG="$(dirname "$0")/rob_box.rviz"
if [ -f "$RVIZ_CONFIG" ]; then
  rviz2 -d "$RVIZ_CONFIG"
else
  rviz2
fi
