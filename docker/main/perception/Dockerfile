# Internal Dialogue Agent - Dockerfile для Main Pi
# Использует базовый образ ros2-zenoh

ARG BASE_IMAGE=ghcr.io/krikz/rob_box_base:ros2-zenoh
FROM ${BASE_IMAGE}

# Установка Python зависимостей для perception
RUN apt-get update && apt-get install -y \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Рабочая директория для ROS2 workspace
WORKDIR /ws
RUN mkdir -p src

# ============================================================
# ОПТИМИЗАЦИЯ КЭШИРОВАНИЯ: Копируем файлы от наименее к наиболее изменяемым
# ============================================================

# 1. Копируем package.xml (метаданные зависимостей) - изменяются редко
COPY src/rob_box_perception_msgs/package.xml /ws/src/rob_box_perception_msgs/
COPY src/rob_box_perception/package.xml /ws/src/rob_box_perception/

# 2. Устанавливаем ROS зависимости через rosdep (кэшируется если package.xml не изменился)
ARG ROS_DISTRO=humble
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y || true

# 3. Копируем setup.py и requirements.txt - изменяются реже чем код
COPY src/rob_box_perception/setup.py /ws/src/rob_box_perception/
COPY docker/main/perception/requirements.txt /tmp/perception_requirements.txt

# 4. Устанавливаем Python зависимости (кэшируется отдельно от кода)
RUN --mount=type=cache,target=/root/.cache/pip,sharing=locked \
    pip3 install --no-cache-dir -r /tmp/perception_requirements.txt && \
    rm /tmp/perception_requirements.txt

# 5. Копируем ресурсы (prompts) - изменяются иногда
COPY src/rob_box_perception/prompts /ws/src/rob_box_perception/prompts

# 6. Только ТЕПЕРЬ копируем весь исходный код (изменяется часто)
COPY src/rob_box_perception_msgs /ws/src/rob_box_perception_msgs
COPY src/rob_box_perception /ws/src/rob_box_perception

# 7. Сборка пакетов (сначала msgs, потом nodes)
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build \
    --packages-select rob_box_perception_msgs rob_box_perception \
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
