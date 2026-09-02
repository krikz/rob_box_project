# Dockerfile для Avatar Supervisor (AV-9, ADR-0028)
#
# Phase 1 (monitor-only): publishes /avatar/state, не вмешивается в
# twist_mux / dialogue_node. Phase 2 (mode=active) — отдельной задачей.
#
# Наследует voice-assistant image, чтобы переиспользовать уже собранные
# aiohttp/prometheus_client/msgpack + rosdep + colcon-workspace. Это
# экономит CI (pytorch-кеш ~10 GB переиспользуется) и держит blast
# radius маленьким: пересборка supervisor занимает минуты, а не часы.
#
# Источники истины:
#   - docs/adr/0028-avatar-supervisor.md §4.6 (deploy-слой)
#   - docs/development/DOCKER_STANDARDS.md
#   - docker/vision/voice_assistant/Dockerfile (образец структуры)
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
# Шаг 1: IDL-пакет (rob_box_supervisor_msgs) + supervisor package.xml +
# setup.py + setup.cfg (структура пакета, кешируется).
#
# AV-12: rosidl-интерфейсы для acquire/release/mode вынесены в отдельный
# ament_cmake-пакет rob_box_supervisor_msgs (rob_box_supervisor собран как
# ament_python и сам IDL не генерирует). supervisor зависит от IDL — если
# пакет не скопирован/не собран, _try_load_supervisor_msgs() даст fallback
# на std_srvs/Trigger + JSON (W3-2 техдолг), нода останется в monitor,
# что безопасно, но полноценный active-режим не включится.
# ============================================================
COPY src/rob_box_supervisor_msgs/package.xml /ws/src/rob_box_supervisor_msgs/
COPY src/rob_box_supervisor_msgs/CMakeLists.txt /ws/src/rob_box_supervisor_msgs/
COPY src/rob_box_supervisor_msgs/srv /ws/src/rob_box_supervisor_msgs/srv
COPY src/rob_box_supervisor_msgs/msg /ws/src/rob_box_supervisor_msgs/msg
COPY src/rob_box_supervisor/package.xml /ws/src/rob_box_supervisor/
COPY src/rob_box_supervisor/setup.py   /ws/src/rob_box_supervisor/
COPY src/rob_box_supervisor/setup.cfg  /ws/src/rob_box_supervisor/
COPY src/rob_box_supervisor/resource   /ws/src/rob_box_supervisor/resource

# ============================================================
# Шаг 2: rosdep (если supervisor-у нужны новые apt-зависимости)
# ============================================================
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    . /ws/install/setup.sh && \
    rosdep update && \
    rosdep install --from-paths src/rob_box_supervisor src/rob_box_supervisor_msgs --ignore-src -r -y || true

# ============================================================
# Шаг 3: исходный код Python (самый горячий слой, последний)
# ============================================================
COPY src/rob_box_supervisor/rob_box_supervisor /ws/src/rob_box_supervisor/rob_box_supervisor

# ============================================================
# Шаг 4: scripts (start_supervisor.sh)
# ============================================================
COPY docker/vision/scripts/supervisor/start_supervisor.sh /scripts/start_supervisor.sh
RUN chmod +x /scripts/start_supervisor.sh

# ============================================================
# Шаг 5: пересборка rob_box_supervisor_msgs (IDL) + rob_box_supervisor
# (pytorch и т.п. из base image не трогаем).
# ============================================================
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    . /ws/install/setup.sh && \
    colcon build \
        --packages-select rob_box_supervisor_msgs rob_box_supervisor \
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
