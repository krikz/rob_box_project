# Internal Dialogue Agent - Dockerfile для Vision Pi
# Использует базовый образ ros2-zenoh

ARG BASE_IMAGE=ghcr.io/krikz/rob_box_base:ros2-zenoh
FROM ${BASE_IMAGE}

# Установка Python зависимостей для perception
RUN apt-get update && apt-get install -y \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Копируем requirements и устанавливаем
COPY docker/vision/perception/requirements.txt /tmp/perception_requirements.txt
RUN pip3 install --no-cache-dir -r /tmp/perception_requirements.txt

# Рабочая директория для ROS2 workspace
WORKDIR /ws
RUN mkdir -p src

# Копируем пакет rob_box_perception
COPY src/rob_box_perception /ws/src/rob_box_perception

# Сборка пакета
ARG ROS_DISTRO=humble
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build \
    --packages-select rob_box_perception \
    --symlink-install \
    --cmake-args \
    -DCMAKE_BUILD_TYPE=Release

# Source workspace в bashrc
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    echo "source /ws/install/setup.bash" >> /root/.bashrc

# Версия пакета
RUN echo "0.1.0-alpha" > /version.txt

# Entry point от базового образа
# ros_entrypoint.sh уже настроен
