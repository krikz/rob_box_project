# Internal Dialogue Agent - Dockerfile для Vision Pi
# Использует базовый образ ros2-zenoh

ARG BASE_IMAGE=ghcr.io/krikz/rob_box_base:ros2-zenoh
FROM ${BASE_IMAGE}

# Установка Python зависимостей для perception
RUN apt-get update && apt-get install -y \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Копируем requirements и устанавливаем
COPY docker/vision/perception/requirements.txt /tmp/perception_requirements.txt
RUN pip3 install --no-cache-dir -r /tmp/perception_requirements.txt

# Версия пакета
RUN echo "0.1.0-alpha" > /version.txt

# ВАЖНО: Код и конфиги НЕ копируются в образ!
# Они монтируются через volumes в docker-compose.yaml

# Entry point от базового образа
# ros_entrypoint.sh уже настроен
