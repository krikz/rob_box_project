# LSLIDAR N10 Driver для ROS 2 Humble
FROM ros:humble-ros-base

# Установка необходимых пакетов
RUN apt-get update && \
    apt-get install -y \
        ros-humble-rmw-zenoh-cpp \
        git \
        python3-pip \
        && rm -rf /var/lib/apt/lists/*

# Создаём рабочее пространство
WORKDIR /ws/src

# Клонируем LSLIDAR ROS 2 драйвер
RUN git clone --depth 1 https://github.com/Lslidar/lslidar_ros2_driver.git

# Собираем пакет
WORKDIR /ws
SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/humble/setup.bash && \
    colcon build --packages-select lslidar_driver lslidar_msgs --symlink-install

# Копируем entrypoint скрипт
COPY lslidar/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
