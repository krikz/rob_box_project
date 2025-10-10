# AprilTag Detection - использует базовый образ ros2-zenoh
# Локальная сборка: docker build --build-arg BASE_IMAGE=rob_box_base:ros2-zenoh
ARG BASE_IMAGE=ghcr.io/krikz/rob_box_base:ros2-zenoh
FROM ${BASE_IMAGE}

# Установка AprilTag пакетов
RUN apt-get update && apt-get install -y \
    ros-humble-apriltag-ros \
    ros-humble-image-transport-plugins \
    ros-humble-cv-bridge \
    && rm -rf /var/lib/apt/lists/*

RUN echo $(dpkg -s ros-humble-apriltag-ros | grep 'Version' | sed -r 's/Version:\s([0-9]+.[0-9]+.[0-9]+).*/\1/g') > /version.txt 

# Устанавливаем права на выполнение скрипта запуска
COPY apriltag/scripts/start_apriltag.sh /start_apriltag.sh
RUN chmod +x /start_apriltag.sh

CMD ["/start_apriltag.sh"]
