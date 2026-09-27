# Dockerfile для Avatar Supervisor (AV-9, ADR-0028)
#
# Phase 1 (monitor-only): publishes /avatar/state, не вмешивается в
# twist_mux / dialogue_node. Phase 2 (mode=active) — отдельной задачей.
#
# Наследует voice-assistant image, который УЖЕ содержит согласованный
# colcon-workspace (rob_box_core, rob_box_supervisor_msgs + остальные
# пакеты — issue #2279). Здесь только свой пакет (rob_box_supervisor):
# COPY → colcon build --packages-select rob_box_supervisor. Это
# исключает version skew между базой и потребителем и экономит CI
# (pytorch-кеш ~10 GB переиспользуется; supervisor пересобирается
# за минуты, не часы). Также убран дублирующий rosdep install,
# apt-зависимости уже разрешены в voice-base/voice-assistant.
#
# Источники истины:
#   - docs/adr/0028-avatar-supervisor.md §4.6 (deploy-слой)
#   - docs/development/DOCKER_STANDARDS.md
#   - docs/architecture/adr-reviews/2026-09-09-docker-build-deploy.md (issue #2279)
#   - docker/vision/voice_assistant/Dockerfile (база, эталон структуры)
#   - docker/vision/docker-compose.yaml (zenoh-router как depends_on)

ARG BASE_IMAGE=ghcr.io/krikz/rob_box:voice-assistant-humble-latest
ARG BUILD_VERSION=2026-08-25-supervisor-app-layer

FROM ${BASE_IMAGE}

ARG ROS_DISTRO=humble

LABEL org.opencontainers.image.title="Avatar Supervisor (Vision Pi, Phase 1 monitor)"
LABEL org.opencontainers.image.description="ADR-0028. Phase 1 monitor mode publishes /avatar/state."
LABEL org.opencontainers.image.source="https://github.com/krikz/rob_box_project"

# Рабочая директория — общий /ws из voice-assistant (где уже лежит
# /ws/install с собранными пакетами). Добавляем ТОЛЬКО rob_box_supervisor
# и не пересобираем весь стек (pytorch/renardo остаются из base).

# ============================================================
# Шаг 1: rob_box_supervisor (ament_python). 
#
# AV-12: rosidl-интерфейсы для acquire/release/mode приходят из
# rob_box_supervisor_msgs, который УЖЕ собран в базовом
# voice-assistant image (issue #2279, root-cause для регрессий
# #2081 / #2089 — version skew между базой и потребителями).
# Поэтому IDL НЕ копируем сюда повторно.
# supervisor зависит от IDL — если пакет не собран в base,
# _try_load_supervisor_msgs() даст fallback на std_srvs/Trigger +
# JSON (W3-2 техдолг), нода останется в monitor, что безопасно,
# но полноценный active-режим не включится.
# ============================================================
COPY src/rob_box_supervisor/package.xml /ws/src/rob_box_supervisor/
COPY src/rob_box_supervisor/setup.py   /ws/src/rob_box_supervisor/
COPY src/rob_box_supervisor/setup.cfg  /ws/src/rob_box_supervisor/
COPY src/rob_box_supervisor/resource   /ws/src/rob_box_supervisor/resource

# ============================================================
# Шаг 2: rosdep — БОЛЬШЕ НЕ НУЖЕН (issue #2279). Раньше здесь
# стоял `rosdep update && rosdep install --from-paths src/...
# src/rob_box_supervisor_msgs`, что приводило к 2× network rosdep
# на каждый прогон L-Build All Services. Все apt-зависимости уже
# разрешены в voice-base/voice-assistant (std_msgs, builtin_interfaces
# и т.п. нужны rob_box_supervisor_msgs, который собирается в
# voice-assistant). Если в будущем supervisor получит новые
# apt-зависимости — добавлять ИХ на слой voice-base, а не сюда.
# ============================================================

# ============================================================
# Шаг 3: исходный код Python (самый горячий слой, последний)
# ============================================================
COPY src/rob_box_supervisor/rob_box_supervisor /ws/src/rob_box_supervisor/rob_box_supervisor

# ============================================================
# Шаг 3а: промпты оператора (issue #1988) — setup.py включает их в
# data_files (share/.../prompts{,/skills}). Без этой копии colcon build
# падает: "can't copy 'prompts/operator_system_prompt.txt': doesn't exist".
# Как в perception/voice_assistant Dockerfile (промпты копируются в образ).
# ============================================================
COPY src/rob_box_supervisor/prompts /ws/src/rob_box_supervisor/prompts

# ============================================================
# Шаг 4: scripts (start_supervisor.sh)
# ============================================================
COPY docker/vision/scripts/supervisor/start_supervisor.sh /scripts/start_supervisor.sh
RUN chmod +x /scripts/start_supervisor.sh

# ============================================================
# Шаг 5: пересборка ТОЛЬКО rob_box_supervisor (pytorch, rosidl
# и т.п. уже из base image). rob_box_core и rob_box_supervisor_msgs
# приходят согласованно из voice-assistant (issue #2279).
# ============================================================
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    . /ws/install/setup.sh && \
    colcon build \
        --packages-select rob_box_supervisor \
        --cmake-args -DCMAKE_BUILD_TYPE=Release

# ============================================================
# Метаданные + ENV
# ============================================================
ARG SOURCE_HASH=unknown
RUN echo "Source hash: ${SOURCE_HASH}" > /tmp/.source_hash

ENV ROS_DOMAIN_ID=0
ENV RMW_IMPLEMENTATION=rmw_zenoh_cpp
ENV ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
# AV-9 монитор-режим НЕ вмешивается в twist_mux / dialogue_node
# (см. ADR-0028 §4.5). mode переключается параметром при старте.
ENV AVATAR_SUPERVISOR_MODE=monitor
