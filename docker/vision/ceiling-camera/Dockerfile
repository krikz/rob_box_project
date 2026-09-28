# Ceiling Camera - использует базовый образ ros2-zenoh
# Локальная сборка: docker build --build-arg BASE_IMAGE=rob_box_base:ros2-zenoh
ARG BASE_IMAGE=ghcr.io/krikz/rob_box_base:ros2-zenoh
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

# Установка USB Camera пакета и image transport для сжатия
RUN apt-get update && apt-get install -y \
    ros-humble-usb-cam \
    ros-humble-image-transport-plugins \
    ros-humble-cv-bridge \
    && rm -rf /var/lib/apt/lists/*
RUN rm -f /etc/apt/apt.conf.d/02proxy

# Версия пакета для отслеживания
RUN echo $(dpkg -s ros-humble-usb-cam | grep 'Version' | sed -r 's/Version:\s([0-9]+.[0-9]+.[0-9]+).*/\1/g') > /version.txt 

# Создаем директорию для camera_info файлов (требуется для usb_cam)
# Фактический файл калибровки будет монтирован через volume
RUN mkdir -p /root/.ros/camera_info

# ВАЖНО: Скрипты и конфиги НЕ копируются в образ!
# Они монтируются через volumes в docker-compose.yaml:
# - ./config:/config:ro (общие конфиги: zenoh, cyclonedds)
# - ./config/ceiling-camera:/config/ceiling-camera:ro (конфиг камеры)
# - ./config/ceiling-camera/ceiling_camera.yaml:/root/.ros/camera_info/ceiling_camera.yaml:ro (калибровка)
# - ./scripts/ceiling-camera:/scripts:ro (скрипт запуска)

# CMD монтируется из volume: /scripts/start_ceiling_camera.sh
CMD ["/scripts/start_ceiling_camera.sh"]
