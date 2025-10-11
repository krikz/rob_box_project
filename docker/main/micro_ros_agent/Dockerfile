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

# Установка зависимостей для сборки micro-ROS agent
RUN apt-get update && apt-get install -y \
    # Build tools
    python3-pip \
    git \
    # Threading support для spdlog/fastrtps
    libpthread-stubs0-dev \
    # Утилиты для serial debugging
    minicom \
    screen \
    && rm -rf /var/lib/apt/lists/*

# Создание рабочего пространства для micro-ROS agent
RUN mkdir -p /uros_ws/src && \
    cd /uros_ws && \
    git clone -b ${ROS_DISTRO} https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup

# Установка зависимостей micro_ros_setup
RUN cd /uros_ws && \
    . /opt/ros/${ROS_DISTRO}/setup.sh && \
    apt-get update && \
    rosdep install --from-paths src --ignore-src -y && \
    rm -rf /var/lib/apt/lists/*

# Сборка micro_ros_setup (без параллелизма для экономии памяти)
RUN cd /uros_ws && \
    . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build --parallel-workers 1

# Создание workspace для agent
RUN cd /uros_ws && \
    . /opt/ros/${ROS_DISTRO}/setup.sh && \
    . install/local_setup.sh && \
    ros2 run micro_ros_setup create_agent_ws.sh

# Сборка micro-ROS agent (долгая операция, без параллелизма для экономии памяти)
RUN cd /uros_ws && \
    . /opt/ros/${ROS_DISTRO}/setup.sh && \
    . install/local_setup.sh && \
    MAKEFLAGS="-j1" ros2 run micro_ros_setup build_agent.sh

# Очистка ненужных файлов
RUN cd /uros_ws && \
    rm -rf build log src/micro_ros_setup

# Создание рабочего пространства для custom messages
WORKDIR /ws_custom

# Копирование пакета custom messages (относительно Docker build context)
# Build context должен быть корень репозитория
COPY src/robot_sensor_hub_msg /ws_custom/src/robot_sensor_hub_msg

# Сборка custom messages
# Ограничиваем параллелизм для экономии памяти при эмуляции ARM64
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    . /uros_ws/install/local_setup.sh && \
    colcon build --packages-select robot_sensor_hub_msg --parallel-workers 1 --executor sequential && \
    rm -rf build log

# Источник всех workspace в bashrc
RUN echo "source /uros_ws/install/local_setup.bash" >> /root/.bashrc && \
    echo "source /ws_custom/install/setup.bash" >> /root/.bashrc

WORKDIR /workspace

# Healthcheck - проверка что agent запущен
# Примечание: micro_ros_agent не создает ROS2 ноду, это standalone процесс
HEALTHCHECK --interval=15s --timeout=5s --start-period=10s --retries=3 \
    CMD pgrep -f "micro.ros.agent" || exit 1

# Entry point - source all workspaces
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/${ROS_DISTRO}/setup.bash && source /uros_ws/install/local_setup.bash && source /ws_custom/install/setup.bash && exec \"$@\"", "--"]

CMD ["bash"]
