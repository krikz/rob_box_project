# ADR-0089 Phase 1: AI HAT+ 26 TOPS inference service (Vision Pi).
#
# Base: ros2-zenoh (та же база, что oak-d / led-matrix).
# Workspace: rob_box_perception_msgs + rob_box_perception собираются
# здесь через colcon в /ws/install (паттерн docker/main/perception).
# Скрипты запуска НЕ копируются в образ — монтируются volume'ами:
#   ./scripts/vision-hailo:/scripts:ro   (start_vision_hailo.sh)
#   ./scripts:/ros_scripts:ro            (ros_with_namespace.sh)
#
# Phase 1 (default): HAILO_ENABLED=false → stub-режим, железо/HailoRT
# не нужны. Реальный inference (hailo_enabled=true) — Phase 1.5,
# отдельной задачей после установки HEF на Vision Pi.
#
# Phase 1 acceptance (см. ADR-0089 §9):
#   hailortcli scan на Vision Pi возвращает `hailo8` device.

ARG ROS_DISTRO=humble
ARG BASE_IMAGE=ghcr.io/krikz/rob_box_base:ros2-zenoh-${ROS_DISTRO}

FROM ${BASE_IMAGE}

ARG ROS_DISTRO=humble

LABEL org.opencontainers.image.title="rob_box vision-hailo" \
      org.opencontainers.image.description="AI HAT+ 26 TOPS inference (ADR-0089)" \
      org.opencontainers.image.source="https://github.com/krikz/rob_box_project"

ENV ROS_DISTRO=${ROS_DISTRO} \
    DEBIAN_FRONTEND=noninteractive

# ROS 2 build-tools для rosidl msg-пакетов + runtime-зависимости
# rob_box_perception (nav_msgs/control_msgs НЕ входят в ros-base).
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-pip \
        python3-numpy \
        python3-opencv \
        python3-yaml \
        ros-${ROS_DISTRO}-rosidl-default-generators \
        ros-${ROS_DISTRO}-rosidl-default-runtime \
        ros-${ROS_DISTRO}-ament-cmake \
        ros-${ROS_DISTRO}-ament-cmake-python \
        ros-${ROS_DISTRO}-nav-msgs \
        ros-${ROS_DISTRO}-control-msgs \
    && rm -rf /var/lib/apt/lists/*

# HailoRT + python binding — BEST-EFFORT. На CI (arm64-эмуляция, без
# официального Hailo apt-repo) ставиться не может — НЕ падаем: Phase 1
# stub-режим железо не требует. RealHEFLoader импортирует hailo_platform
# лениво, real inference — Phase 1.5 (на Vision Pi вручную).
RUN apt-get update && \
    (apt-get install -y --no-install-recommends libhailort-dev || true) && \
    (pip3 install --no-cache-dir hailort==4.18.0 2>/dev/null || \
     pip3 install --no-cache-dir hailort 2>/dev/null || \
     echo "WARNING: hailort python binding install skipped — Phase 1 stub mode OK") && \
    rm -rf /var/lib/apt/lists/*

# Smoke-test: hailortcli доступен? Не падаем (см. выше).
RUN hailortcli --version 2>/dev/null || \
    echo "WARNING: hailortcli не найден — Phase 1 stub mode OK"

# ---------- workspace (паттерн docker/main/perception) ----------
WORKDIR /ws
RUN mkdir -p src

# 1. package.xml (метаданные) — кэшируются отдельно
COPY src/rob_box_perception_msgs/package.xml /ws/src/rob_box_perception_msgs/
COPY src/rob_box_perception/package.xml /ws/src/rob_box_perception/

# 2. rosdep (best-effort — apt-зависимости явно установлены выше)
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    rosdep update || true; \
    rosdep install --from-paths src --ignore-src -r -y || true

# 3. setup.py / setup.cfg / requirements — реже, чем код
COPY src/rob_box_perception/setup.py /ws/src/rob_box_perception/
COPY src/rob_box_perception/setup.cfg /ws/src/rob_box_perception/
COPY src/rob_box_perception/requirements.txt /tmp/perception_requirements.txt
RUN pip3 install --retries 5 --timeout 120 --no-cache-dir -r /tmp/perception_requirements.txt && \
    rm /tmp/perception_requirements.txt

# 4. data_files (resource / prompts / launch из setup.py) — изменяются иногда
COPY src/rob_box_perception/resource /ws/src/rob_box_perception/resource
COPY src/rob_box_perception/prompts /ws/src/rob_box_perception/prompts
COPY src/rob_box_perception/launch /ws/src/rob_box_perception/launch

# 5. весь исходный код (горячий слой). SOURCE_HASH инвалидирует кэш
# при изменении src/rob_box_perception* (issue #2314 pattern).
ARG SOURCE_HASH=unknown
RUN echo "Source hash: ${SOURCE_HASH}" > /tmp/.source_hash
COPY src/rob_box_perception_msgs /ws/src/rob_box_perception_msgs
COPY src/rob_box_perception /ws/src/rob_box_perception

# 6. сборка (сначала msgs, потом nodes)
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build \
    --packages-select rob_box_perception_msgs rob_box_perception \
    --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source workspace
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    echo "source /ws/install/setup.bash" >> /root/.bashrc

# entrypoint — ros_with_namespace.sh (volume) + start script (volume).
CMD ["/ros_scripts/ros_with_namespace.sh", "/scripts/start_vision_hailo.sh"]
