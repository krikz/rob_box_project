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
# - ./config:/config/shared:ro (общие конфиги: zenoh, cyclonedds)
# - ../../src/rob_box_description:/workspace/src/rob_box_description:ro (URDF)
# - ./robot_state_publisher/scripts:/scripts:ro (скрипт запуска)

# ENTRYPOINT монтируется из volume: /scripts/start_robot_state_publisher.sh
ENTRYPOINT ["/scripts/start_robot_state_publisher.sh"]

