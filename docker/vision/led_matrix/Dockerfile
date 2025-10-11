# Dockerfile для LED Matrix Driver и Compositor
# Управление NeoPixel LED матрицами через SPI на Vision Pi (Raspberry Pi 5)
#
# Архитектура:
#   - led_matrix_driver: низкоуровневый драйвер (SPI → LED через pi5neo)
#   - led_matrix_compositor: композитор панелей (sensor_msgs/Image → driver)
#
# Физическая конфигурация:
#   - 4× панели 8×8 (фары передние/задние)
#   - 5× панели 5×5 (main_display 5×25)
#   - Общая цепочка: 381 LED (256 + 125)

ARG ROS_DISTRO=humble
ARG IMAGE_TAG=latest
ARG BASE_IMAGE_PREFIX=ghcr.io/krikz/rob_box_base

FROM ${BASE_IMAGE_PREFIX}:ros2-zenoh-${ROS_DISTRO}-${IMAGE_TAG}

# Метаданные
LABEL org.opencontainers.image.title="LED Matrix System for Vision Pi"
LABEL org.opencontainers.image.description="ROS2 LED matrix driver and compositor with pi5neo"
LABEL org.opencontainers.image.source="https://github.com/krikz/ros2leds"

# Установка системных зависимостей для SPI и LED
RUN apt-get update && apt-get install -y \
    # SPI и GPIO инструменты
    python3-pip \
    python3-dev \
    python3-spidev \
    build-essential \
    libffi-dev \
    # ROS2 зависимости для Image messages
    ros-${ROS_DISTRO}-sensor-msgs \
    ros-${ROS_DISTRO}-std-msgs \
    && rm -rf /var/lib/apt/lists/*

# Установка Python библиотек для LED
# pi5neo - библиотека для управления WS2812B через SPI на Raspberry Pi 5
# Обновляем pip перед установкой для поддержки современных флагов
RUN pip3 install --no-cache-dir --upgrade pip && \
    pip3 install --no-cache-dir \
    pi5neo \
    spidev \
    numpy

# Рабочая директория
WORKDIR /ws
RUN mkdir -p src

# Копируем пакеты ros2leds (driver и compositor)
COPY src/ros2leds/led_matrix_driver /ws/src/led_matrix_driver
COPY src/ros2leds/led_matrix_compositor /ws/src/led_matrix_compositor

# Установка ROS зависимостей через rosdep
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y || true

# Сборка пакетов
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build \
    --packages-select led_matrix_driver led_matrix_compositor \
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

# Проверка доступности SPI устройства
HEALTHCHECK --interval=15s --timeout=5s --start-period=10s --retries=3 \
    CMD ros2 node list | grep -q led_matrix || exit 1

# Точка входа - source ROS и запуск через startup script
ENTRYPOINT ["/bin/bash", "-c", "source /ws/install/setup.bash && exec \"$@\"", "--"]

# По умолчанию запускаем оба компонента через launch file
CMD ["bash", "-c", "source /ws/install/setup.bash && ros2 launch led_matrix_compositor led_matrix_compositor_launch.py"]
