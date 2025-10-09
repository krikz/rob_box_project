# LSLIDAR N10 Driver для ROS 2 Humble
FROM ros:humble-ros-base

# Установка необходимых пакетов
RUN apt-get update && \
    apt-get install -y \
        ros-humble-rmw-zenoh-cpp \
        git \
        python3-pip \
        libboost-thread-dev \
        libboost-chrono-dev \
        libpcl-dev \
        ros-humble-pcl-conversions \
        ros-humble-pcl-ros \
        ros-humble-diagnostic-updater \
        && rm -rf /var/lib/apt/lists/*

# Создаём рабочее пространство
WORKDIR /ws/src

# Клонируем LSLIDAR N10 драйвер (ветка M10P/N10P для N10)
RUN git clone --depth 1 -b M10P/N10P https://github.com/Lslidar/lslidar_ros2_driver.git

# Собираем все пакеты из репозитория
WORKDIR /ws
SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install

# Копируем entrypoint скрипт
COPY lslidar/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
