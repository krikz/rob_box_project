# Dockerfile для ROS2 Control Manager с VESC Hardware Interface
# Этот контейнер запускает controller_manager который загружает VescSystemHardwareInterface
# 
# VESC Nexus работает как hardware_interface плагин внутри controller_manager,
# а НЕ как отдельная нода!

ARG ROS_DISTRO=humble
ARG IMAGE_TAG=latest
ARG BASE_IMAGE_PREFIX=ghcr.io/krikz/rob_box_base

FROM ${BASE_IMAGE_PREFIX}:ros2-zenoh-${ROS_DISTRO}-${IMAGE_TAG}

# Метаданные
LABEL org.opencontainers.image.title="ROS2 Control Manager with VESC"
LABEL org.opencontainers.image.description="Controller Manager with VescSystemHardwareInterface for 4-wheel robot"
LABEL org.opencontainers.image.source="https://github.com/krikz/rob_box_project"

# Установка ROS2 Control и зависимостей
RUN apt-get update && apt-get install -y \
    # CAN утилиты
    can-utils \
    iproute2 \
    net-tools \
    # ROS2 Control
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-controller-manager \
    ros-${ROS_DISTRO}-hardware-interface \
    ros-${ROS_DISTRO}-pluginlib \
    # rosidl для компиляции кастомных сообщений (ПОЛНЫЙ набор)
    ros-${ROS_DISTRO}-rosidl-default-runtime \
    ros-${ROS_DISTRO}-rosidl-default-generators \
    ros-${ROS_DISTRO}-rosidl-generator-c \
    ros-${ROS_DISTRO}-rosidl-generator-cpp \
    ros-${ROS_DISTRO}-rosidl-typesupport-c \
    ros-${ROS_DISTRO}-rosidl-typesupport-cpp \
    ros-${ROS_DISTRO}-rosidl-typesupport-fastrtps-c \
    ros-${ROS_DISTRO}-rosidl-typesupport-fastrtps-cpp \
    # Контроллеры
    ros-${ROS_DISTRO}-diff-drive-controller \
    ros-${ROS_DISTRO}-joint-state-broadcaster \
    ros-${ROS_DISTRO}-joint-trajectory-controller \
    # Утилиты
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-ros2controlcli \
    && rm -rf /var/lib/apt/lists/*

# Рабочая директория
WORKDIR /ws
RUN mkdir -p src

# Копируем vesc_nexus пакеты (vesc_msgs и vesc_nexus) из их src/
# vesc_nexus - это отдельный репозиторий, пакеты находятся в vesc_nexus/src/
COPY src/vesc_nexus/src/vesc_msgs /ws/src/vesc_msgs
COPY src/vesc_nexus/src/vesc_nexus /ws/src/vesc_nexus

# Копируем rob_box_description (URDF с ros2_control блоком)
COPY src/rob_box_description /ws/src/rob_box_description

# Установка зависимостей через rosdep
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y || true

# Сборка пакетов
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build \
    --packages-select vesc_msgs vesc_nexus rob_box_description \
    --symlink-install \
    --cmake-args \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

# Source workspace
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    echo "source /ws/install/setup.bash" >> /root/.bashrc

# ROS2 и Zenoh environment variables
ENV ROS_DOMAIN_ID=0
ENV RMW_IMPLEMENTATION=rmw_zenoh_cpp
ENV RUST_LOG=zenoh=info
ENV ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ENV ZENOH_CONFIG=/config/shared/zenoh_session_config.json5

# Healthcheck - проверяем что controller_manager запущен
HEALTHCHECK --interval=10s --timeout=5s --start-period=30s --retries=3 \
    CMD ros2 control list_controllers | grep -q diff_drive_controller || exit 1

# Команда запуска будет переопределена в docker-compose через startup script
CMD ["bash", "-c", "echo 'Use docker-compose to start this container'"]
