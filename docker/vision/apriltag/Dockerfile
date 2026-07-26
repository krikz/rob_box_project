# AprilTag Detection - использует базовый образ ros2-zenoh
# Локальная сборка: docker build --build-arg BASE_IMAGE=rob_box_base:ros2-zenoh
ARG BASE_IMAGE=ghcr.io/krikz/rob_box_base:ros2-zenoh
FROM ${BASE_IMAGE}

# Установка AprilTag пакетов
RUN rm -f /etc/apt/apt.conf.d/02proxy && apt-get update && apt-get install -y \
    ros-kilted-apriltag-ros \
    ros-kilted-image-transport-plugins \
    ros-kilted-cv-bridge \
    && rm -rf /var/lib/apt/lists/*

RUN echo $(dpkg -s ros-kilted-apriltag-ros | grep 'Version' | sed -r 's/Version:\s([0-9]+.[0-9]+.[0-9]+).*/\1/g') > /version.txt 

# ВАЖНО: Скрипты и конфиги НЕ копируются в образ!
# Они монтируются через volumes в docker-compose.yaml:
# - ./config:/config:ro (общие конфиги: zenoh, cyclonedds)
# - ./config/apriltag:/config/apriltag:ro (конфиг apriltag)
# - ./scripts/apriltag:/scripts:ro (скрипт запуска)

# CMD монтируется из volume: /scripts/start_apriltag.sh
CMD ["/scripts/start_apriltag.sh"]
