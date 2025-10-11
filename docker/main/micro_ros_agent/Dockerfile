# micro-ROS Agent для связи с ESP32 Sensor Hub
# Транслирует сообщения между ROS2 (DDS/Zenoh) и micro-ROS (XRCE-DDS)
# Работает по UART (serial) с ESP32

ARG ROS_DISTRO=humble
ARG IMAGE_TAG=latest
ARG BASE_IMAGE_PREFIX=ghcr.io/krikz/rob_box_base

FROM ${BASE_IMAGE_PREFIX}:ros2-zenoh-${ROS_DISTRO}-${IMAGE_TAG}

LABEL org.opencontainers.image.title="micro-ROS Agent for ESP32"
LABEL org.opencontainers.image.description="XRCE-DDS Agent for sensor hub communication"
LABEL org.opencontainers.image.source="https://github.com/krikz/rob_box_project"

# Установка micro-ROS agent через snap
RUN apt-get update && apt-get install -y \
    # Snap package manager
    snapd \
    # Утилиты для serial debugging
    minicom \
    screen \
    && rm -rf /var/lib/apt/lists/*

# Установка micro-ROS agent snap
# https://snapcraft.io/micro-ros-agent
RUN snap install micro-ros-agent

# Создание рабочего пространства для custom messages
WORKDIR /ws

# Копирование пакета custom messages (относительно Docker build context)
# Build context должен быть корень репозитория
COPY src/robot_sensor_hub_msg /ws/src/robot_sensor_hub_msg

# Сборка custom messages
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build --packages-select robot_sensor_hub_msg && \
    rm -rf build log

# Источник custom messages в bashrc для всех сессий
RUN echo "source /ws/install/setup.bash" >> /root/.bashrc

WORKDIR /workspace

# Healthcheck - проверка что agent запущен
# Примечание: micro_ros_agent не создает ROS2 ноду, это standalone процесс
HEALTHCHECK --interval=15s --timeout=5s --start-period=10s --retries=3 \
    CMD pgrep -f "micro.ros.agent" || exit 1

# Entry point - source custom messages and ROS
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/${ROS_DISTRO}/setup.bash && source /ws/install/setup.bash && exec \"$@\"", "--"]

CMD ["bash"]
