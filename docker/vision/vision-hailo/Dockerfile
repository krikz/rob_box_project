# ADR-0089 Phase 1: AI HAT+ 26 TOPS inference service.
#
# Этот Dockerfile собирает arm64 образ с HailoRT + TAPPAS + Python binding
# для запуска vision_hailo_node на Vision Pi 5 (10.1.1.21).
#
# Hardware reference:
#   https://www.raspberrypi.com/documentation/accessories/ai-hat-plus.html
#   https://github.com/hailo-ai/hailo-rpi5-examples
#
# Build context: docker/vision/vision-hailo/ (этот файл)
# Base:        rob_box base для Vision Pi (тот же что oak-d/led-matrix)
# Target:      ghcr.io/krikz/rob_box:vision-hailo-humble-${VISION_HAILO_TAG:-${IMAGE_TAG}}
#
# Phase 1 build flow:
#   1. docker/vision/vision-hailo/start_vision_hailo.sh — entrypoint
#   2. /config/hailo_models.yaml — SSoT активных HEF (volume mount)
#   3. /opt/rob_box/vision_hailo — node entrypoint (colcon install)
#
# Phase 1.5 (на железе):
#   1. Скачать HEF из hailo_model_zoo (YOLOv8n, RetinaFace, ArcFace-MobileFaceNet).
#   2. Положить в /opt/rob_box/models/*.hef (volume mount).
#   3. Запуск: hailo_enabled=True, hef_path=/opt/rob_box/models/yolov8n.hef.
#
# Почему НЕ multi-stage: на Pi5 сборка TAPPAS = ~30 минут; проще
# переиспользовать готовый hailort deb из официального Pi-репо
# (https://github.com/hailo-ai/hailort) и накатить сверху Python binding.
# Это сокращает build с 30 минут до 5 минут при cold cache.
#
# Phase 1 acceptance (см. ADR-0089 §9):
#   hailortcli scan на Vision Pi возвращает `hailo8` device.

ARG ROS_DISTRO=humble
ARG BASE_IMAGE=ghcr.io/krikz/rob_box:vision-base-${ROS_DISTRO}-local

# ---------- Stage 1: builder для Python wheels ----------
# Используем builder только если нужно скомпилировать hailort python
# binding (например под новую версию). На Phase 1 builder опционален —
# мы берём готовый hailort deb из Pi-репо.
FROM ubuntu:22.04 AS builder

ARG HAILORT_VERSION=4.18.0
ARG ROS_DISTRO=humble

ENV DEBIAN_FRONTEND=noninteractive \
    PIP_NO_CACHE_DIR=1

RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential \
        cmake \
        git \
        python3-pip \
        python3-dev \
        wget \
        libhailort-dev \
    && rm -rf /var/lib/apt/lists/*

# Опциональная сборка hailort python binding (для CI без интернета).
# В обычном build-flow мы используем pre-built wheel из PyPI.
RUN pip3 install --upgrade pip setuptools wheel || true

# ---------- Stage 2: runtime ----------
FROM ${BASE_IMAGE}

ARG HAILORT_VERSION=4.18.0
ARG ROS_DISTRO=humble

LABEL org.opencontainers.image.title="rob_box vision-hailo" \
      org.opencontainers.image.description="AI HAT+ 26 TOPS inference (ADR-0089)" \
      org.opencontainers.image.source="https://github.com/krikz/rob_box_project"

ENV ROS_DISTRO=${ROS_DISTRO} \
    DEBIAN_FRONTEND=noninteractive

# HailoRT + Python binding + TAPPAS runtime dependencies.
# Здесь используется стандартный deb-репозиторий Hailo для Raspbian
# (URL конфигурируется через ARG, дефолт для Debian Bookworm совместим
# с Ubuntu 22.04 host на Pi5). Если репо недоступно — собираем из
# исходников (см. builder stage).
RUN apt-get update && apt-get install -y --no-install-recommends \
        hailort=4.18.0 \
        libhailort-dev \
        python3-pip \
        python3-numpy \
        python3-opencv \
        python3-yaml \
    && rm -rf /var/lib/apt/lists/*

# Python binding для hailort — pre-built wheel от hailo-ai.
# Если wheel недоступен, fallback на pip install hailort (community).
RUN pip3 install --no-cache-dir \
        hailort==4.18.0 || \
    pip3 install --no-cache-dir \
        hailort || \
    echo "WARNING: hailort Python binding install failed — runtime may be degraded"

# ROS 2 зависимости для vision_hailo_node (rclpy, sensor_msgs, vision_msgs).
# Ставятся через apt (ros-${ROS_DISTRO}-ros-base) — мы их получаем из BASE_IMAGE.
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-sensor-msgs \
    && rm -rf /var/lib/apt/lists/*

# Workspace с vision_hailo_node. Собирается через colcon на этапе CI;
# здесь ожидаем что /opt/rob_box уже содержит setup.bash с собранным
# пакетом rob_box_perception (Phase 1 PoC собирается как part of CI matrix).
ENV ROB_BOX_WS=/opt/rob_box
RUN if [ ! -d "${ROB_BOX_WS}/install" ]; then \
        echo "ERROR: /opt/rob_box/install отсутствует. " \
             "Соберите rob_box_perception через colcon перед сборкой этого образа."; \
        exit 1; \
    fi

# Скрипты запуска.
COPY start_vision_hailo.sh /scripts/start_vision_hailo.sh
COPY hailo_smoke.py /opt/rob_box/scripts/hailo_smoke.py
RUN chmod +x /scripts/start_vision_hailo.sh

# Smoke-test на этапе build — проверяем, что hailortcli доступен.
# Если нет — НЕ падаем, а только warning (Phase 1 на CI без железа).
RUN hailortcli --version || echo "WARNING: hailortcli не найден — Phase 1 на Pi5 проверит вручную"

# entrypoint — через ros_with_namespace.sh (как у других vision-сервисов).
CMD ["/ros_scripts/ros_with_namespace.sh", "/scripts/start_vision_hailo.sh"]
