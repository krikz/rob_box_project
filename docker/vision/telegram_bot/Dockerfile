# Dockerfile для Telegram Bot — операторский интерфейс через Telegram
#
# Легковесный контейнер: python-telegram-bot + rclpy + aiohttp
# Деплоится на Vision Pi рядом с voice-assistant
#
# ВАЖНО: конфиги и скрипты монтируются через volumes, НЕ COPY!
ARG BASE_IMAGE=ghcr.io/krikz/rob_box_base:ros2-zenoh

FROM ${BASE_IMAGE}

ARG ROS_DISTRO=humble

LABEL org.opencontainers.image.title="Telegram Bot for Rob Box"
LABEL org.opencontainers.image.description="Operator interface via Telegram for Rob Box autonomous rover"
LABEL org.opencontainers.image.source="https://github.com/krikz/rob_box_project"

# Системные зависимости (минимум)
RUN apt-get update && apt-get install -y \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Python зависимости
RUN pip3 install --no-cache-dir \
    python-telegram-bot[job-queue]==21.* \
    aiohttp>=3.9 \
    && pip3 cache purge

# Копируем ROS 2 пакет
WORKDIR /ws/src
COPY src/rob_box_telegram /ws/src/rob_box_telegram

# Собираем ROS 2 пакет
WORKDIR /ws
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build --packages-select rob_box_telegram && \
    rm -rf /ws/log

WORKDIR /ws
