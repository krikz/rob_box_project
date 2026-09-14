# ADR-0089 Phase 1: AI HAT+ 26 TOPS inference service (vision_hailo_node).
#
# Лёгкий runtime поверх rob_box_base:ros2-zenoh (как quest): НЕ тащим
# pytorch/renardo/vosk из voice-assistant. Нужны только rclpy + rmw_zenoh
# + rob_box_perception_msgs + rob_box_perception (vision_hailo_node).
#
# Что собирает Phase 1 (stub-режим, без железа):
#   - rob_box_perception_msgs (VisionEvent.msg + PerceptionEvent.msg) — IDL.
#   - rob_box_perception (console_scripts vision_hailo) — ament_python.
# HailoRT / HEF / real inference — Phase 1.5 (на железе Vision Pi):
#   - hailortcli scan smoke (start_vision_hailo.sh, HAILO_ENABLED=true).
#   - HEF из hailo_model_zoo в /opt/rob_box/models/*.hef.
#
# Почему НЕ ставим hailort здесь: (1) Hailo apt-repo недоступен на
# build-машине (Katana) через apt-proxy; (2) Phase 1 acceptance =
# stub-pipeline, без железа. Ставить hailort "наугад" в Phase 1 =
# выдавать желаемое за действительное (ADR-0018). HailoRT-слой
# добавляется отдельным PR в Phase 1.5.
#
# Build context: корень репо (COPY src/...), как quest/supervisor.
# Скрипты запуска НЕ COPY'им — монтируются volume'ом из
# docker/vision/scripts/vision-hailo (DOCKER_STANDARDS: нет COPY scripts/).
#
# Источники истины:
#   - docs/adr/0089-ai-hat-plus-deployment.md §3
#   - docker/vision/quest/Dockerfile (паттерн ros2-zenoh + colcon)
#   - docker/vision/supervisor/Dockerfile (паттерн COPY-этапов)
#   - docs/development/DOCKER_STANDARDS.md (нет COPY config/, нет COPY scripts/)

ARG BASE_IMAGE=ghcr.io/krikz/rob_box_base:ros2-zenoh-${ROS_DISTRO:-humble}

FROM ${BASE_IMAGE}

# ARG обязательно переобъявить в новом FROM — иначе ${ROS_DISTRO} пустой.
ARG ROS_DISTRO=humble

LABEL org.opencontainers.image.title="rob_box vision-hailo" \
      org.opencontainers.image.description="AI HAT+ 26 TOPS inference (ADR-0089, Phase 1 stub)" \
      org.opencontainers.image.source="https://github.com/krikz/rob_box_project"

# Katana (build-машина): apt через локальный apt-cacher-ng
# (APT_PROXY=http://host.docker.internal:3142) — тот же паттерн, что в
# docker/base/Dockerfile.ros2-zenoh и quest. 02proxy удаляем после apt —
# не тащим прокси-конфиг в рантайм.
ARG APT_PROXY=""
RUN if [ -n "$APT_PROXY" ]; then \
        PROXY_HOST=$(echo "$APT_PROXY" | sed 's|.*://\([^:/]*\).*|\1|'); \
        PROXY_PORT=$(echo "$APT_PROXY" | sed 's|.*://[^:]*:\([0-9]*\).*|\1|'); \
        if timeout 5 bash -c "cat < /dev/null > /dev/tcp/$PROXY_HOST/$PROXY_PORT" 2>/dev/null; then \
            echo "Acquire::http::Proxy \"$APT_PROXY\";" > /etc/apt/apt.conf.d/02proxy; \
            echo "Acquire::http::Timeout \"10\";" >> /etc/apt/apt.conf.d/02proxy; \
            echo "Acquire::Retries \"3\";" >> /etc/apt/apt.conf.d/02proxy; \
        fi \
    fi

# ROS 2 зависимости: IDL-генерация (rob_box_perception_msgs — ament_cmake) +
# exec-зависимости rob_box_perception (package.xml: std/sensor/geometry/nav/
# control msgs, builtin/rcl_interfaces). python3-yaml нужен start-скрипту для
# парсинга /config/hailo_models.yaml.
RUN apt-get update -qq \
 && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
      python3-yaml \
      ros-${ROS_DISTRO}-ament-cmake \
      ros-${ROS_DISTRO}-rosidl-default-generators \
      ros-${ROS_DISTRO}-rosidl-default-runtime \
      ros-${ROS_DISTRO}-builtin-interfaces \
      ros-${ROS_DISTRO}-std-msgs \
      ros-${ROS_DISTRO}-sensor-msgs \
      ros-${ROS_DISTRO}-geometry-msgs \
      ros-${ROS_DISTRO}-nav-msgs \
      ros-${ROS_DISTRO}-control-msgs \
      ros-${ROS_DISTRO}-rcl-interfaces \
 && rm -rf /var/lib/apt/lists/* \
 && rm -f /etc/apt/apt.conf.d/02proxy

# Python runtime-зависимости rob_box_perception (setup.py install_requires:
# setuptools, pytz) + pyyaml для парсинга hailo_models.yaml в start-скрипте.
# ros2-zenoh база — Ubuntu 22.04 (jammy), системный pip 22.0.2 без PEP 668.
RUN pip3 install --no-cache-dir --retries 5 --timeout 300 \
      pytz \
      pyyaml

# Workspace (ros2-zenoh база — ros:humble-ros-base без colcon-workspace;
# создаём /ws/src заранее, иначе COPY упадёт "no such file or directory").
WORKDIR /ws
RUN mkdir -p src/rob_box_perception_msgs src/rob_box_perception

# ============================================================
# Шаг 1: IDL-пакет (ament_cmake) — структура, кешируется отдельно.
# ============================================================
COPY src/rob_box_perception_msgs/package.xml  /ws/src/rob_box_perception_msgs/
COPY src/rob_box_perception_msgs/CMakeLists.txt /ws/src/rob_box_perception_msgs/
COPY src/rob_box_perception_msgs/msg /ws/src/rob_box_perception_msgs/msg

# ============================================================
# Шаг 2: rob_box_perception (ament_python) — структура.
# ============================================================
COPY src/rob_box_perception/package.xml /ws/src/rob_box_perception/
COPY src/rob_box_perception/setup.py   /ws/src/rob_box_perception/
COPY src/rob_box_perception/setup.cfg  /ws/src/rob_box_perception/
COPY src/rob_box_perception/resource /ws/src/rob_box_perception/resource

# setup.py data_files: prompts/*.txt + launch/*.launch.py — без этих копий
# colcon build падает "can't copy ... doesn't exist" (тот же класс багов,
# что issue #1988 для supervisor prompts).
COPY src/rob_box_perception/prompts /ws/src/rob_box_perception/prompts
COPY src/rob_box_perception/launch /ws/src/rob_box_perception/launch

# ============================================================
# Шаг 3: исходный код (самый горячий слой, последний).
# ============================================================
COPY src/rob_box_perception/rob_box_perception /ws/src/rob_box_perception/rob_box_perception

# ============================================================
# Шаг 4: пересборка rob_box_perception_msgs (IDL) + rob_box_perception.
# Порядок в --packages-select важен: msgs собирается первым, чтобы его
# сгенерированные Python-типы были доступны при сборке perception.
# ============================================================
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
 && colcon build \
        --packages-select rob_box_perception_msgs rob_box_perception \
        --cmake-args -DCMAKE_BUILD_TYPE=Release

# Cache invalidation: хеш исходников (build-arg из workflow). Меняется —
# инвалидирует финальный слой (страховка как в supervisor/quest).
ARG SOURCE_HASH=unknown
RUN echo "Source hash: ${SOURCE_HASH}" > /tmp/.source_hash

# Zenoh-роутинг (как у других vision-сервисов).
ENV ROS_DOMAIN_ID=0
ENV RMW_IMPLEMENTATION=rmw_zenoh_cpp
ENV ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

# ENTRYPOINT/CMD НЕ задаём: compose даёт
#   command: ["/ros_scripts/ros_with_namespace.sh", "/scripts/start_vision_hailo.sh"]
# (тот же паттерн, что oak-d).
