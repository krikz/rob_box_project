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
FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

LABEL org.opencontainers.image.title="SuperCollider Server for Vision Pi"
LABEL org.opencontainers.image.description="scsynth audio synthesis engine for Renardo/FoxDot real-time music"
LABEL org.opencontainers.image.source="https://github.com/krikz/rob_box_project"

RUN apt-get update && apt-get install -y \
    supercollider-server \
    alsa-utils \
    procps \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /sc_data

ENTRYPOINT ["/bin/bash"]
CMD ["/scripts/start_supercollider.sh"]
