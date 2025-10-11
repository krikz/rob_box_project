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

# Установка micro-ROS agent
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-micro-ros-agent \
    # Утилиты для serial debugging
    minicom \
    screen \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace

# Healthcheck - проверка что agent запущен
# Примечание: micro_ros_agent не создает ROS2 ноду, это standalone процесс
HEALTHCHECK --interval=15s --timeout=5s --start-period=10s --retries=3 \
    CMD pgrep -f "micro_ros_agent" || exit 1

# Entry point
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/${ROS_DISTRO}/setup.bash && exec \"$@\"", "--"]

CMD ["bash"]
