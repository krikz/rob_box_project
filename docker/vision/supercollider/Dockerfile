# Dockerfile для SuperCollider synthesis server
# Запускает scsynth в headless-режиме для Renardo/FoxDot интеграции
#
# Компоненты:
#   - scsynth: SuperCollider synthesis server (UDP OSC на порту 57110)
#
# Использование:
#   - Renardo/FoxDot подключается к scsynth для синтеза звука
#   - Общается через OSC (UDP, порт 57110)
#   - Использует ALSA dmix для совместной работы с TTS/sound effects
FROM ubuntu:26.04

ENV DEBIAN_FRONTEND=noninteractive

LABEL org.opencontainers.image.title="SuperCollider Server for Vision Pi"
LABEL org.opencontainers.image.description="scsynth audio synthesis engine for Renardo/FoxDot real-time music"
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

RUN apt-get update && apt-get install -y \
    supercollider-server \
    sc3-plugins-server \
    jackd2 \
    alsa-utils \
    procps \
    python3 \
    && rm -rf /var/lib/apt/lists/*
RUN rm -f /etc/apt/apt.conf.d/02proxy

WORKDIR /sc_data

ENTRYPOINT ["/bin/bash"]
CMD ["/scripts/start_supercollider.sh"]
