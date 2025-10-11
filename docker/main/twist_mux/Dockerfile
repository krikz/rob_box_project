# twist_mux для мультиплексирования команд скорости
# Приоритизирует команды от разных источников (joystick > nav2)

ARG ROS_DISTRO=humble
ARG IMAGE_TAG=latest
ARG BASE_IMAGE_PREFIX=ghcr.io/krikz/rob_box_base

FROM ${BASE_IMAGE_PREFIX}:ros2-zenoh-${ROS_DISTRO}-${IMAGE_TAG}

LABEL org.opencontainers.image.title="twist_mux for rob_box"
LABEL org.opencontainers.image.description="Multiplexes velocity commands with prioritization"
LABEL org.opencontainers.image.source="https://github.com/krikz/rob_box_project"

# Установка twist_mux
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-twist-mux \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace

# Healthcheck - проверка что нода работает
HEALTHCHECK --interval=10s --timeout=5s --start-period=10s --retries=3 \
    CMD ros2 node list | grep -q "twist_mux" || exit 1

# Entry point
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/${ROS_DISTRO}/setup.bash && exec \"$@\"", "--"]

CMD ["bash"]
