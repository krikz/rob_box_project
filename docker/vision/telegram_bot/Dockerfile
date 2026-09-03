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
# ffmpeg — AV-23 (issue #1915): voice_transcode.ogg_to_pcm16k гоняет
# через него OGG/Opus из Telegram в PCM 16k. Базовый образ
# rob_box_base:ros2-zenoh идёт от ros:humble-ros-base и ffmpeg не
# содержит; ffmpeg стоит только в voice_base, от которого этот
# контейнер НЕ наследуется. Без этой строки _find_ffmpeg() бросает
# VoiceTranscodeError на первом же голосовом с робота.
RUN apt-get update && apt-get install -y \
    python3-pip \
    ffmpeg \
    wget \
    unzip \
    && rm -rf /var/lib/apt/lists/*

# Python зависимости
# vosk — локальный офлайн-фолбек STT (issue найден 2026-09-03: у Yandex
# закончился баланс, голосовые в Telegram молчали вместо деградации).
# Тот же контракт, что у stt_node.py в rob_box_voice: Yandex — primary,
# Vosk — fallback без сети/квоты.
RUN pip3 install --no-cache-dir \
    python-telegram-bot[job-queue]==21.* \
    aiohttp>=3.9 \
    Pillow>=10.0 \
    numpy>=1.24 \
    prometheus_client>=0.20.0 \
    vosk>=0.3.45 \
    && pip3 cache purge

# Vosk-модель (45 МБ, ru) — тот же артефакт, что voice_base/Dockerfile
# распаковывает для stt_node.py, путь по умолчанию совпадает
# (/models/vosk-model-small-ru-0.22), так что VOSK_MODEL_PATH менять не
# нужно ни там, ни тут.
RUN mkdir -p /models && \
    wget -q -O /tmp/vosk-model.zip \
        https://alphacephei.com/vosk/models/vosk-model-small-ru-0.22.zip && \
    unzip -q -o /tmp/vosk-model.zip -d /models/ && \
    test -f /models/vosk-model-small-ru-0.22/README && \
    rm /tmp/vosk-model.zip

# Копируем ROS 2 пакеты
# rob_box_core — AV-13/AV-28 (avatar_command): telegram_node импортирует
# rob_box_core.avatar_command, без сборки пакета в образе падает
# ModuleNotFoundError на старте контейнера.
WORKDIR /ws/src
COPY src/rob_box_core /ws/src/rob_box_core
COPY src/rob_box_telegram /ws/src/rob_box_telegram

# Собираем ROS 2 пакеты
WORKDIR /ws
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build --packages-select rob_box_core rob_box_telegram && \
    rm -rf /ws/log

WORKDIR /ws
