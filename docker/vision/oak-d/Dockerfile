# OAK-D Camera - использует специализированный базовый образ depthai
# Базовый образ luxonis/depthai-ros уже содержит все необходимое для OAK-D
# Локальная сборка: docker build --build-arg BASE_IMAGE=rob_box_base:depthai
ARG BASE_IMAGE=ghcr.io/krikz/rob_box_base:depthai
FROM ${BASE_IMAGE}

# Все необходимые зависимости (rmw-zenoh, image-transport, opencv, depthai) 
# уже установлены в базовом образе

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

# Установка AprilTag пакетов для интеграции детекции в OAK-D контейнер
# Это позволяет избежать отдельного контейнера и снизить нагрузку на Vision Pi
RUN apt-get update && apt-get install -y \
    ros-humble-apriltag-ros \
    ros-humble-image-transport-plugins \
    ros-humble-cv-bridge \
    && rm -rf /var/lib/apt/lists/* 
RUN rm -f /etc/apt/apt.conf.d/02proxy

# ВАЖНО: Launch файлы и скрипты НЕ копируются в образ!
# Они монтируются через volumes в docker-compose.yaml:
# - ./config:/config:ro (общие конфиги: zenoh, cyclonedds)
# - ./config/oak-d:/config/oak-d:ro (конфиги и launch файлы oak-d)
# - ./scripts/oak-d:/scripts:ro (скрипт запуска)

# CMD монтируется из volume: /scripts/start_oak_d.sh
CMD ["/scripts/start_oak_d.sh"]
