# twist_mux для мультиплексирования команд скорости
# Приоритизирует команды от разных источников (joystick > nav2)

ARG ROS_DISTRO=humble
ARG BASE_IMAGE=ghcr.io/krikz/rob_box_base:ros2-zenoh

FROM ${BASE_IMAGE}

LABEL org.opencontainers.image.title="twist_mux for rob_box"
LABEL org.opencontainers.image.description="Multiplexes velocity commands with prioritization"
LABEL org.opencontainers.image.source="https://github.com/krikz/rob_box_project"

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

# Установка twist_mux
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-twist-mux \
    && rm -rf /var/lib/apt/lists/*
RUN rm -f /etc/apt/apt.conf.d/02proxy

WORKDIR /workspace

# Healthcheck - проверка что нода работает
HEALTHCHECK --interval=10s --timeout=5s --start-period=10s --retries=3 \
    CMD ros2 node list | grep -q "twist_mux" || exit 1

# Entry point
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/${ROS_DISTRO}/setup.bash && exec \"$@\"", "--"]

CMD ["bash"]
