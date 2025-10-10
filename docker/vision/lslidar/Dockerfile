# LSLIDAR N10 Driver для ROS 2 Humble
# Использует базовый образ PCL с предустановленными зависимостями для облаков точек
# Локальная сборка: docker build --build-arg BASE_IMAGE=rob_box_base:pcl
ARG BASE_IMAGE=ghcr.io/krikz/rob_box_base:pcl
FROM ${BASE_IMAGE}

# Все необходимые пакеты (rmw-zenoh-cpp, PCL, diagnostic-updater, libpcap, boost)
# уже установлены в базовом образе

# Устанавливаем дополнительные зависимости для сборки lslidar драйвера
RUN apt-get update && apt-get install -y \
    ros-humble-rosidl-default-generators \
    ros-humble-rosidl-default-runtime \
    ros-humble-builtin-interfaces \
    ros-humble-std-msgs \
    ros-humble-sensor-msgs \
    ros-humble-ament-cmake \
    && rm -rf /var/lib/apt/lists/*

# Создаём рабочее пространство
WORKDIR /ws/src

# Клонируем LSLIDAR N10 драйвер (ветка M10P/N10P для N10)
RUN git clone --depth 1 -b M10P/N10P https://github.com/Lslidar/lslidar_ros2_driver.git

# Собираем все пакеты из репозитория
# ВАЖНО: конфиги и скрипты НЕ копируются в образ!
# Они монтируются через volumes в docker-compose.yaml:
# - ./config:/config/shared:ro (общие конфиги: zenoh, cyclonedds)
# - ./lslidar/config:/config/lslidar:ro (конфиги lslidar)
# - ./lslidar/scripts:/scripts:ro (скрипт запуска)
WORKDIR /ws
SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install

# Entrypoint монтируется из volume: /scripts/start_lslidar.sh
CMD ["/scripts/start_lslidar.sh"]
