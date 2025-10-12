# Dockerfile для ROS2 Control Manager с VESC Hardware Interface
# Этот контейнер запускает controller_manager который загружает VescSystemHardwareInterface
# 
# VESC Nexus работает как hardware_interface плагин внутри controller_manager,
# а НЕ как отдельная нода!

# ВАЖНО: Используем rtabmap базовый образ вместо минимального ros2-zenoh
# rtabmap базируется на полном ROS2 с всеми необходимыми библиотеками для компиляции
ARG BASE_IMAGE=ghcr.io/krikz/rob_box_base:rtabmap
# Версия сборки для инвалидации кеша при изменении зависимостей
ARG BUILD_VERSION=2025-10-12-rtabmap

FROM ${BASE_IMAGE}

# Метаданные
LABEL org.opencontainers.image.title="ROS2 Control Manager with VESC"
LABEL org.opencontainers.image.description="Controller Manager with VescSystemHardwareInterface for 4-wheel robot"
LABEL org.opencontainers.image.source="https://github.com/krikz/rob_box_project"

# Принудительная инвалидация кеша для слоев установки
ARG BUILD_VERSION
RUN echo "Build version: ${BUILD_VERSION}"

# Установка ROS2 Control и зависимостей
# Базовый образ rtabmap уже содержит полный ROS2, нужны только Control пакеты
RUN apt-get update && apt-get install -y \
    # CAN утилиты
    can-utils \
    iproute2 \
    net-tools \
    # ROS2 Control (ros-base уже в базовом образе)
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-controller-manager \
    ros-humble-hardware-interface \
    ros-humble-pluginlib \
    # Контроллеры
    ros-humble-diff-drive-controller \
    ros-humble-joint-state-broadcaster \
    ros-humble-joint-trajectory-controller \
    # Утилиты
    ros-humble-xacro \
    ros-humble-ros2controlcli \
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

# Установка зависимостей через rosdep и сборка пакетов
RUN . /opt/ros/humble/setup.sh && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y \
        --skip-keys "rviz2 joint_state_publisher joint_state_publisher_gui serial_driver" && \
    colcon build \
    --packages-select vesc_msgs vesc_nexus rob_box_description \
    --symlink-install \
    --cmake-args \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

# Source workspace
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
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
