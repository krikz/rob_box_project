# Dockerfile для robot_state_publisher - публикует TF дерево робота из URDF
# Использует базовый образ ros2-zenoh
ARG BASE_IMAGE=rob_box_base:ros2-zenoh
FROM ${BASE_IMAGE}

# Установка специфичных пакетов для robot_state_publisher
RUN apt-get update && apt-get install -y \
    ros-humble-robot-state-publisher \
    ros-humble-xacro \
    && rm -rf /var/lib/apt/lists/*

# Создаем рабочую директорию
WORKDIR /workspace

# Точка входа
COPY entrypoint.sh /
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
