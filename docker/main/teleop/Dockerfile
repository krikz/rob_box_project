# syntax=docker/dockerfile:1
# Локальная сборка: docker build --build-arg BASE_IMAGE=localhost:5000/krikz/rob_box_base:ros2-zenoh
ARG BASE_IMAGE=ghcr.io/krikz/rob_box_base:ros2-zenoh-humble-latest
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

# Copy workspace source (пакет rob_box_teleop)
COPY --chown=ros2:ros2 src/rob_box_teleop /workspace/src/rob_box_teleop

# Build workspace
# НЕ используем --symlink-install (ретро 14.08 t_7d6b4b65): при symlink-install
# install(PROGRAMS ...) создаёт symlink на ИСХОДНИК из git, а joystick_control_node.py
# в репо имеет mode 100644 (не executable, коммит 9ec4f554 24.07) → ros2 run
# не находит executable (get_executable_paths требует os.access X_OK) →
# crash-loop 'No executable found'. Без --symlink-install install(PROGRAMS)
# копирует файл и ставит +x (как voice_assistant в production image).
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    cd /workspace && \
    colcon build \
        --packages-select rob_box_teleop \
        --cmake-args -DCMAKE_BUILD_TYPE=Release

WORKDIR /workspace

# Default command will be overridden by docker-compose
CMD ["bash"]
