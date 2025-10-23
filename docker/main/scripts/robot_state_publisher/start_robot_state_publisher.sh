#!/bin/bash
set -e

# ВАЖНО: Сначала настраиваем Zenoh конфиг через wrapper
echo "Setting up Zenoh session config with namespace..."
if [ -f "/ros_scripts/ros_with_namespace.sh" ]; then
    # Создаём zenoh config с namespace через wrapper 
    /ros_scripts/ros_with_namespace.sh echo "Zenoh config set up"
    echo "ZENOH_SESSION_CONFIG_URI=$ZENOH_SESSION_CONFIG_URI"
else
    echo "WARNING: ros_with_namespace.sh not found, using default config"
    export ZENOH_SESSION_CONFIG_URI=/config/zenoh_session_config.json5
fi

# Source ROS 2
source /opt/ros/humble/setup.bash

# Генерируем URDF из xacro
echo "Generating URDF from xacro..."
xacro /workspace/src/rob_box_description/urdf/rob_box.xacro > /tmp/rob_box.urdf

# Проверяем, что URDF создан
if [ ! -s /tmp/rob_box.urdf ]; then
    echo "ERROR: Failed to generate URDF!"
    exit 1
fi

echo "URDF generated successfully ($(wc -l < /tmp/rob_box.urdf) lines)"

# Создаем yaml файл с параметрами для robot_state_publisher
cat > /tmp/robot_state_publisher_params.yaml << EOF
robot_state_publisher:
  ros__parameters:
    robot_description: |
$(cat /tmp/rob_box.urdf | sed 's/^/      /')
    use_sim_time: false
    publish_frequency: 30.0
EOF

# Запускаем robot_state_publisher с параметрами из файла
echo "Starting robot_state_publisher with params from YAML file..."
exec ros2 run robot_state_publisher robot_state_publisher \
    --ros-args --params-file /tmp/robot_state_publisher_params.yaml
