# Dockerfile для robot_state_publisher - публикует TF дерево робота из URDF
FROM ros:humble-ros-base

# Установка необходимых пакетов
RUN apt-get update && apt-get install -y \
    ros-humble-robot-state-publisher \
    ros-humble-xacro \
    ros-humble-rmw-zenoh-cpp \
    && rm -rf /var/lib/apt/lists/*

# Создаем рабочую директорию
WORKDIR /workspace

# Точка входа
COPY entrypoint.sh /
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
