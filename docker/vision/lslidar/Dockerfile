# LSLIDAR N10 Driver для ROS 2 Humble
# Использует базовый образ PCL с предустановленными зависимостями для облаков точек
# Локальная сборка: docker build --build-arg BASE_IMAGE=rob_box_base:pcl
ARG BASE_IMAGE=ghcr.io/krikz/rob_box_base:pcl
FROM ${BASE_IMAGE}

# Все необходимые пакеты (rmw-zenoh-cpp, PCL, diagnostic-updater, libpcap, boost)
# уже установлены в базовом образе

# Создаём рабочее пространство
WORKDIR /ws/src

# Клонируем LSLIDAR N10 драйвер (ветка M10P/N10P для N10)
RUN git clone --depth 1 -b M10P/N10P https://github.com/Lslidar/lslidar_ros2_driver.git

# Копируем наш headless лаунч-файл (без RViz2)
COPY config/lslidar_headless_launch.py /ws/src/lslidar_ros2_driver/lslidar_driver/launch/lslidar_launch.py

# Копируем кастомный конфиг с правильным serial портом
COPY lslidar/config/lsx10_custom.yaml /config/lsx10_custom.yaml

# Собираем все пакеты из репозитория
WORKDIR /ws
SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install

# Копируем entrypoint скрипт
COPY lslidar/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
