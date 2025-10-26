# Dockerfile для robot_state_publisher - публикует TF дерево робота из URDF
# Использует базовый образ ros2-zenoh из GitHub Container Registry
# Локальная сборка: docker build --build-arg BASE_IMAGE=rob_box_base:ros2-zenoh
ARG BASE_IMAGE=ghcr.io/krikz/rob_box_base:ros2-zenoh
FROM ${BASE_IMAGE}

# Установка специфичных пакетов для robot_state_publisher
RUN apt-get update && apt-get install -y \
    ros-humble-robot-state-publisher \
    ros-humble-xacro \
    && rm -rf /var/lib/apt/lists/*

# Рабочая директория
WORKDIR /workspace

# ВАЖНО: URDF файлы и скрипты НЕ копируются в образ!
# Они монтируются через volumes в docker-compose.yaml:
# - ./config:/config:ro (конфиги Zenoh)
# - ./scripts:/ros_scripts:ro (ros_with_namespace.sh wrapper)
# - ./scripts/robot_state_publisher:/scripts:ro (start_robot_state_publisher.sh)
# - ../../src/rob_box_description:/workspace/src/rob_box_description:ro (URDF)

# Command задается в docker-compose.yaml через ros_with_namespace.sh wrapper
# для генерации Zenoh session config с правильным namespace

