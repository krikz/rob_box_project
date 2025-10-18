# Dockerfile для VESC Nexus - ROS2 драйвер для управления VESC через CAN
# Базируется на ros2-zenoh образе для интеграции с Zenoh middleware

ARG ROS_DISTRO=humble
ARG IMAGE_TAG=latest
ARG BASE_IMAGE_PREFIX=ghcr.io/krikz/rob_box_base

FROM ${BASE_IMAGE_PREFIX}:ros2-zenoh-${ROS_DISTRO}-${IMAGE_TAG}

# Метаданные
LABEL org.opencontainers.image.title="VESC Nexus Driver"
LABEL org.opencontainers.image.description="Multi-CAN VESC driver with ros2_control support"
LABEL org.opencontainers.image.source="https://github.com/krikz/vesc_nexus"
LABEL org.opencontainers.image.version="1.0.0"

# Установка зависимостей для CAN и ros2_control
RUN apt-get update && apt-get install -y \
    can-utils \
    iproute2 \
    net-tools \
    # ROS 2 development tools
    ros-dev-tools \
    # ROS 2 message generation dependencies
    ros-${ROS_DISTRO}-rosidl-default-generators \
    ros-${ROS_DISTRO}-rosidl-default-runtime \
    # ros2_control stack
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-controller-manager \
    ros-${ROS_DISTRO}-hardware-interface \
    ros-${ROS_DISTRO}-diff-drive-controller \
    ros-${ROS_DISTRO}-joint-state-broadcaster \
    ros-${ROS_DISTRO}-pluginlib \
    ros-${ROS_DISTRO}-xacro \
    && rm -rf /var/lib/apt/lists/*

# Рабочая директория
WORKDIR /ws
RUN mkdir -p src

# ============================================================
# ОПТИМИЗАЦИЯ КЭШИРОВАНИЯ: Копируем файлы от наименее к наиболее изменяемым
# ============================================================

# 1. Копируем package.xml (метаданные зависимостей) - изменяются редко
COPY src/vesc_nexus/src/vesc_msgs/package.xml /ws/src/vesc_msgs/
COPY src/vesc_nexus/src/vesc_nexus/package.xml /ws/src/vesc_nexus/

# 2. Установка ROS зависимостей через rosdep (кэшируется если package.xml не изменился)
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y \
    --skip-keys "serial_driver joint_state_publisher joint_state_publisher_gui rviz2" || true

# 3. Копируем CMakeLists.txt - изменяются реже чем код
COPY src/vesc_nexus/src/vesc_msgs/CMakeLists.txt /ws/src/vesc_msgs/
COPY src/vesc_nexus/src/vesc_nexus/CMakeLists.txt /ws/src/vesc_nexus/

# 4. Копируем msg/srv определения (изменяются редко)
COPY src/vesc_nexus/src/vesc_msgs/msg /ws/src/vesc_msgs/msg

# 5. Только ТЕПЕРЬ копируем весь исходный код (изменяется часто)
COPY src/vesc_nexus/src/vesc_msgs /ws/src/vesc_msgs
COPY src/vesc_nexus/src/vesc_nexus /ws/src/vesc_nexus

# 6. Сборка пакетов
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build \
    --packages-select vesc_msgs vesc_nexus \
    --symlink-install \
    --cmake-args \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

# Добавляем source в bashrc для интерактивного использования
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    echo "source /ws/install/setup.bash" >> /root/.bashrc

# Переменные окружения для ROS 2 и Zenoh
ENV ROS_DOMAIN_ID=0
ENV RMW_IMPLEMENTATION=rmw_zenoh_cpp
ENV RUST_LOG=zenoh=info
ENV ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ENV ZENOH_CONFIG=/config/shared/zenoh_session_config.json5

# Healthcheck для проверки работоспособности ноды
HEALTHCHECK --interval=10s --timeout=5s --start-period=30s --retries=3 \
    CMD ros2 node list | grep -q vesc_can_driver || exit 1

# По умолчанию запускаем через скрипт (будет переопределено в docker-compose)
CMD ["bash", "-c", "source /ws/install/setup.bash && ros2 launch vesc_nexus vesc_nexus_node.launch.py"]
