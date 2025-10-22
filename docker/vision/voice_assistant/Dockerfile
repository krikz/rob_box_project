# Dockerfile для Voice Assistant System
# AI голосовой ассистент с интеграцией LED анимаций
#
# Компоненты:
#   - rob_box_voice: голосовой ассистент (AudioNode, STTNode, TTSNode, DialogueNode, LEDNode, SoundNode)
#   - rob_box_animations: LED матричные анимации для эмоций и состояний
#
# Hardware:
#   - ReSpeaker Mic Array v2.0 (USB, 4 микрофона, 12× RGB LED)
#   - Sound card WM8960 (3.5mm jack для TTS output)
#   - NeoPixel LED матрицы (через led_matrix_compositor)

# БАЗОВЫЙ ОБРАЗ: voice-base содержит все зависимости, драйверы, модели
ARG BASE_IMAGE=ghcr.io/krikz/rob_box_base:voice-base-humble-latest
FROM ${BASE_IMAGE}

ARG ROS_DISTRO=humble

# Метаданные
LABEL org.opencontainers.image.title="Voice Assistant System for Vision Pi"
LABEL org.opencontainers.image.description="AI voice assistant with ReSpeaker and LED animations"
LABEL org.opencontainers.image.source="https://github.com/krikz/rob_box_project"

# ============================================================
# СБОРКА НАШИХ ROS2 ПАКЕТОВ (rob_box_voice + rob_box_animations)
# ============================================================

# Рабочая директория
WORKDIR /ws
RUN mkdir -p src/rob_box_voice src/rob_box_animations

# Шаг 1: Копируем package.xml для rosdep (зависимости меняются редко)
COPY src/rob_box_voice/package.xml /ws/src/rob_box_voice/
COPY src/rob_box_animations/package.xml /ws/src/rob_box_animations/

# Шаг 2: Установка ROS зависимостей через rosdep
# Этот слой кешируется пока package.xml не изменяется
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    . /ws_base/install/setup.sh && \
    rosdep update && \
    rosdep install --from-paths src/rob_box_voice src/rob_box_animations --ignore-src -r -y || true

# Шаг 3: Копируем setup.py и CMakeLists.txt (структура пакета меняется редко)

COPY src/rob_box_voice/setup.py /ws/src/rob_box_voice/
COPY src/rob_box_voice/setup.cfg /ws/src/rob_box_voice/
COPY src/rob_box_animations/CMakeLists.txt /ws/src/rob_box_animations/

# Шаг 5: Копируем конфигурационные файлы и ресурсы
COPY src/rob_box_voice/resource /ws/src/rob_box_voice/resource
COPY sound_pack /ws/sound_pack

# Шаг 6: Копируем launch файлы и конфигурации
COPY src/rob_box_voice/launch /ws/src/rob_box_voice/launch
COPY src/rob_box_voice/config /ws/src/rob_box_voice/config
COPY src/rob_box_voice/prompts /ws/src/rob_box_voice/prompts
COPY src/rob_box_animations/launch /ws/src/rob_box_animations/launch

# Шаг 8: Копируем animations data
COPY src/rob_box_animations/animations /ws/src/rob_box_animations/animations

# Шаг 7: Копируем исходный код Python (меняется чаще всего - последний слой!)
COPY src/rob_box_voice/rob_box_voice /ws/src/rob_box_voice/rob_box_voice
COPY src/rob_box_animations/rob_box_animations /ws/src/rob_box_animations/rob_box_animations

# Шаг 8: Копируем scripts
COPY src/rob_box_voice/scripts /ws/src/rob_box_voice/scripts
COPY src/rob_box_animations/scripts /ws/src/rob_box_animations/scripts

# ============================================
# 🚀 СБОРКА ROS2 ПАКЕТОВ
# ============================================

# Собираем наши пакеты (audio_common_msgs уже собран в базовом образе)
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    . /ws_base/install/setup.sh && \
    colcon build \
    --packages-select rob_box_voice rob_box_animations \
    --symlink-install \
    --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

# Source workspace
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    echo "source /ws_base/install/setup.bash" >> /root/.bashrc && \
    echo "source /ws/install/setup.bash" >> /root/.bashrc

# ============================================================
# ENVIRONMENT VARIABLES
# ============================================================

# ROS2 и Zenoh environment variables
ENV ROS_DOMAIN_ID=0
ENV RMW_IMPLEMENTATION=rmw_zenoh_cpp
ENV RUST_LOG=zenoh=info
ENV ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ENV ZENOH_CONFIG=/config/shared/zenoh_session_config.json5

# Аудио environment variables
ENV AUDIODEV=hw:ArrayUAC10,0
ENV ALSA_CARD=ArrayUAC10

# Пути к данным
ENV SOUND_PACK_DIR=/ws/sound_pack
ENV TTS_CACHE_DIR=/config/voice/tts_cache

# ============================================================
# HEALTHCHECK & ENTRYPOINT
# ============================================================

# Проверка доступности нод
HEALTHCHECK --interval=30s --timeout=10s --start-period=30s --retries=3 \
    CMD ros2 node list | grep -q "audio_node\|animation_player" || exit 1

# Точка входа
ENTRYPOINT ["/bin/bash", "-c", "source /ws_base/install/setup.bash && source /ws/install/setup.bash && exec \"$@\"", "--"]

# По умолчанию запускаем через startup script
CMD ["/scripts/start_voice_assistant.sh"]
