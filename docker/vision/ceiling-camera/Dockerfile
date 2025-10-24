# Ceiling Camera - использует базовый образ ros2-zenoh
# Локальная сборка: docker build --build-arg BASE_IMAGE=rob_box_base:ros2-zenoh
ARG BASE_IMAGE=ghcr.io/krikz/rob_box_base:ros2-zenoh
FROM ${BASE_IMAGE}

# Установка USB Camera пакета и image transport для сжатия
RUN apt-get update && apt-get install -y \
    ros-humble-usb-cam \
    ros-humble-image-transport-plugins \
    ros-humble-cv-bridge \
    && rm -rf /var/lib/apt/lists/*

# Версия пакета для отслеживания
RUN echo $(dpkg -s ros-humble-usb-cam | grep 'Version' | sed -r 's/Version:\s([0-9]+.[0-9]+.[0-9]+).*/\1/g') > /version.txt 

# ВАЖНО: Скрипты и конфиги НЕ копируются в образ!
# Они монтируются через volumes в docker-compose.yaml:
# - ./config:/config:ro (общие конфиги: zenoh, cyclonedds)
# - ./config/ceiling-camera:/config/ceiling-camera:ro (конфиг камеры)
# - ./scripts/ceiling-camera:/scripts:ro (скрипт запуска)

# CMD монтируется из volume: /scripts/start_ceiling_camera.sh
CMD ["/scripts/start_ceiling_camera.sh"]
