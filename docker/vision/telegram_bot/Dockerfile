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
# wget/unzip отсюда убраны вместе со ступенью скачивания Vosk (см. ниже):
# больше их в этом образе не использовал никто — ни start_telegram_bot.sh,
# ни healthcheck, ни код ноды.
RUN apt-get update && apt-get install -y \
    python3-pip \
    ffmpeg \
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

# ============================================================================
# Vosk-модель — БОЛЬШЕ НЕ ЗДЕСЬ (Ресурсный пак, ADR-0125)
# ============================================================================
# Здесь была ступень скачивания vosk-model-small-ru-0.22 (архив 44 МиБ,
# 88 МиБ распакованный) — ровно тот же артефакт, который до 21.09.2026 качал
# и voice_base/Dockerfile. Две сборки ходили за одной моделью, два образа
# хранили её копии.
#
# Почему удалена: сборка telegram-bot ходила в сеть и падала ЦЕЛИКОМ при
# недоступном alphacephei.com (цепочка `wget && unzip && test -f` без
# `|| true`), а версия модели в этой строке могла молча разъехаться с
# манифестом Ресурсного пака. Это был пятый путь доставки, не описанный
# нигде: реестр docs/plans/2026-09-15-resource-pack.md §1.1 перечислял
# четыре (ADR-0125 §8 п.3, план §16.5).
#
# Куда переехало: docker/vision/scripts/resource_pack/manifest.yaml, запись
# `vosk-ru-small` (required: hard). Файл кладёт на ХОСТ шаг деплоя
# «[Vision Pi] Ensure STT/TTS models», контейнер получает его bind-mount'ом
# `/opt/rob_box/models:/models:ro` (docker-compose.yaml, telegram-bot — тот
# же том, что уже был у voice-assistant).
#
# Путь ВНУТРИ контейнера не изменился: voice_processor.py:40 читает
# `/models/vosk-model-small-ru-0.22` (дефолт VOSK_MODEL_PATH) — ни одна
# строка Python не правилась.
#
# `mkdir -p /models` тоже не нужен: каталог создаёт bind-mount.
#
# NB: запуск контейнера БЕЗ этого тома — не молчаливая поломка:
# _load_vosk_model() пишет в лог «Vosk model not found at ...» и возвращает
# None (voice_processor.py:133-140), STT остаётся на одном Yandex.

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
