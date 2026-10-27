# syntax=docker/dockerfile:1
# Локальная сборка: docker build --build-arg BASE_IMAGE=localhost:5000/krikz/rob_box_base:ros2-zenoh \
#                               --build-arg MSGS_IMAGE=<образ docker/msgs/Dockerfile>
ARG BASE_IMAGE=ghcr.io/krikz/rob_box_base:ros2-zenoh-humble-latest

# Общий образ ROS-сообщений (docker/msgs/Dockerfile): src/rob_box_*_msgs
# компилируются ОДИН раз за прогон, а не в каждом образе (rosidl под qemu —
# 4-14 минут на пакет). Тег по хэшу пакетов вычисляет gen_build_matrix.py
# (docker/build-manifest.yaml: teleop -> msgs_image: true) и передаёт
# build-arg'ом; без него сборка падает сразу на имени образа.
# Нужен rob_box_dialog_msgs: джойстик говорит через speech_client.say ->
# SpeechRequest (ADR-0150 §6.1, PR-15 часть 15-0d, решение D8).
ARG MSGS_IMAGE=MSGS_IMAGE-build-arg-is-required
FROM ${MSGS_IMAGE} AS msgs

FROM ${BASE_IMAGE}

# APT через apt-cacher-ng на katana (APT_PROXY передаёт gen_build_matrix.py).
# Базовый образ удаляет свой 02proxy, поэтому без этого блока apt ходил
# напрямую, а прямой путь с katana ненадёжен: GPG error (run 34387765562),
# Hash Sum mismatch у .deb того же размера (run 36421581380, nav2/voice-base).
# Шаблон и причина Pipeline-Depth 0 — docker/base/Dockerfile.ros2-zenoh.
ARG APT_PROXY=""
RUN if [ -n "$APT_PROXY" ]; then \
        PROXY_HOST=$(echo "$APT_PROXY" | sed 's|.*://\([^:/]*\).*|\1|'); \
        PROXY_PORT=$(echo "$APT_PROXY" | sed 's|.*://[^:]*:\([0-9]*\).*|\1|'); \
        if timeout 5 bash -c "cat < /dev/null > /dev/tcp/$PROXY_HOST/$PROXY_PORT" 2>/dev/null; then \
            printf '%s\n' "Acquire::http::Proxy \"$APT_PROXY\";" 'Acquire::http::Timeout "10";' \
                'Acquire::Retries "3";' 'Acquire::http::Pipeline-Depth "0";' > /etc/apt/apt.conf.d/02proxy; \
            echo "✅ Using APT proxy: $APT_PROXY"; \
        else \
            echo "⚠️  APT proxy not accessible, will use direct connection"; \
        fi \
    fi

# Install joy node from ROS 2 packages and Serial dependencies
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-joy \
    ros-${ROS_DISTRO}-joy-linux \
    joystick \
    jstest-gtk \
    python3-serial \
    && rm -rf /var/lib/apt/lists/*
RUN rm -f /etc/apt/apt.conf.d/02proxy

# Рабочее пространство - /ws, как в остальных образах с msgs (quest, perception,
# telegram-bot): rob_box_dialog_msgs приходит готовым /ws/install из stage `msgs`.
# start_teleop.sh (его монтирует compose из репозитория) делает
# `source /workspace/install/setup.bash`, поэтому /workspace - ссылка на /ws:
# скрипт не правится и не зависит от порядка, в котором обновятся образ и репозиторий.
WORKDIR /ws
COPY --from=msgs /ws/install /ws/install
RUN ln -s /ws /workspace

# rob_box_core (speech_client, Utterance) - ВЕСЬ пакет, а не отдельные файлы:
# setup.py тащит data_files (config/, resource/), и новый файл там ломает сборку
# выборочного COPY (#2250 в quest). Образец - docker/sound/telegram_bot/Dockerfile.
COPY src/rob_box_core /ws/src/rob_box_core
# Copy workspace source (пакет rob_box_teleop)
COPY --chown=ros2:ros2 src/rob_box_teleop /ws/src/rob_box_teleop

# Build workspace
# НЕ используем --symlink-install (ретро 14.08 t_7d6b4b65): при symlink-install
# install(PROGRAMS ...) создаёт symlink на ИСХОДНИК из git, а joystick_control_node.py
# в репо имеет mode 100644 (не executable, коммит 9ec4f554 24.07) → ros2 run
# не находит executable (get_executable_paths требует os.access X_OK) →
# crash-loop 'No executable found'. Без --symlink-install install(PROGRAMS)
# копирует файл и ставит +x (как voice_assistant в production image).
# `. /ws/install/setup.sh` - IDL-пакеты из stage `msgs`: colcon видит зависимость
# rob_box_dialog_msgs, но не собирает её (нет в --packages-select).
# Последняя проверка - ровно тот путь, который source'ит start_teleop.sh: если
# раскладка рабочего пространства изменится, сборка упадёт здесь, а не на роботе.
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    . /ws/install/setup.sh && \
    cd /ws && \
    colcon build \
        --packages-select rob_box_core rob_box_teleop \
        --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    test -f /workspace/install/setup.bash

WORKDIR /ws

# Default command will be overridden by docker-compose
CMD ["bash"]
