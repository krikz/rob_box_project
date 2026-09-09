#!/bin/bash
set -e

# Zenoh config уже настроен через ros_with_namespace.sh wrapper в docker-compose.yaml
echo "ZENOH_SESSION_CONFIG_URI=$ZENOH_SESSION_CONFIG_URI"

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
