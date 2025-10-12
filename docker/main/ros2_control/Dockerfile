# Dockerfile для ROS2 Control Manager с VESC Hardware Interface
# Этот контейнер запускает controller_manager который загружает VescSystemHardwareInterface
# 
# VESC Nexus работает как hardware_interface плагин внутри controller_manager,
# а НЕ как отдельная нода!

ARG ROS_DISTRO=humble
ARG IMAGE_TAG=latest
ARG BASE_IMAGE_PREFIX=ghcr.io/krikz/rob_box_base
# Версия сборки для инвалидации кеша при изменении зависимостей
ARG BUILD_VERSION=2025-10-12

FROM ${BASE_IMAGE_PREFIX}:ros2-zenoh-${ROS_DISTRO}-${IMAGE_TAG}

# Метаданные
LABEL org.opencontainers.image.title="ROS2 Control Manager with VESC"
LABEL org.opencontainers.image.description="Controller Manager with VescSystemHardwareInterface for 4-wheel robot"
LABEL org.opencontainers.image.source="https://github.com/krikz/rob_box_project"

# Принудительная инвалидация кеша для слоев установки
ARG BUILD_VERSION
RUN echo "Build version: ${BUILD_VERSION}"

# Установка ROS2 Control и зависимостей
# КРИТИЧНО: Базовый образ слишком минимальный, устанавливаем ros-base для всех библиотек
# Очищаем apt кеш и устанавливаем все за один RUN для согласованности слоев
RUN apt-get clean && \
    rm -rf /var/lib/apt/lists/* /var/cache/apt/* && \
    apt-get update && apt-get install -y \
    # ROS2 Base - содержит ВСЕ необходимые библиотеки для компиляции
    ros-${ROS_DISTRO}-ros-base \
    # CAN утилиты
    can-utils \
    iproute2 \
    net-tools \
    # ROS2 базовая инфраструктура
    ros-${ROS_DISTRO}-ament-cmake \
    ros-${ROS_DISTRO}-ament-cmake-ros \
    # ROS2 Control
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-controller-manager \
    ros-${ROS_DISTRO}-hardware-interface \
    ros-${ROS_DISTRO}-pluginlib \
    # Контроллеры
    ros-${ROS_DISTRO}-diff-drive-controller \
    ros-${ROS_DISTRO}-joint-state-broadcaster \
    ros-${ROS_DISTRO}-joint-trajectory-controller \
    # Утилиты
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-ros2controlcli \
    && rm -rf /var/lib/apt/lists/* /var/cache/apt/*

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
# ВАЖНО: Объединяем rosdep update и сборку в один слой для избежания кеш-проблем
# Пропускаем GUI пакеты (rviz2, joint-state-publisher-gui) и serial-driver (для vesc_nexus они опциональны)
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    rm -rf /root/.ros/rosdep && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y \
        --skip-keys "rviz2 joint_state_publisher joint_state_publisher_gui serial_driver" && \
    # ДИАГНОСТИКА: Проверяем пути библиотек для ARM64
    echo "=== Architecture and library paths ===" && \
    dpkg --print-architecture && \
    echo "=== Searching for builtin_interfaces libraries ===" && \
    find /opt/ros/${ROS_DISTRO} -name "*builtin_interfaces*.so" 2>/dev/null | head -20 && \
    find /usr/lib -name "*builtin_interfaces*.so" 2>/dev/null | head -20 && \
    echo "=== LD_LIBRARY_PATH ===" && \
    echo $LD_LIBRARY_PATH && \
    echo "=== CMAKE_PREFIX_PATH ===" && \
    echo $CMAKE_PREFIX_PATH && \
    # Пробуем установить недостающие dev пакеты явно
    apt-get update && apt-get install -y \
        ros-${ROS_DISTRO}-builtin-interfaces \
        ros-${ROS_DISTRO}-rosidl-default-runtime \
        ros-${ROS_DISTRO}-rosidl-default-generators \
    && rm -rf /var/lib/apt/lists/* && \
    # Проверяем после установки
    echo "=== After explicit install ===" && \
    find /opt/ros/${ROS_DISTRO} -name "*builtin_interfaces*rosidl_generator_c.so" 2>/dev/null && \
    # Собираем с явными путями
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
