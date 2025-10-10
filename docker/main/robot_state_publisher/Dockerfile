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

# Создаем рабочую директорию

# Рабочая директория
WORKDIR /workspace

COPY docker/main/scripts/start_robot_state_publisher.sh /start_robot_state_publisher.sh
RUN chmod +x /start_robot_state_publisher.sh
ENTRYPOINT ["/start_robot_state_publisher.sh"]

