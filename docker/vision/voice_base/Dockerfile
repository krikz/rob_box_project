# Base Dockerfile для Voice Assistant dependencies
# Содержит все Python библиотеки и системные зависимости
# Используется как база для rob_box_voice
#
# Цель: кеширование тяжелых зависимостей (PyTorch, Whisper, etc)
# для быстрой пересборки при изменениях в rob_box_voice коде

ARG BASE_IMAGE=ghcr.io/krikz/rob_box_base:rtabmap
ARG BUILD_VERSION=2026-03-08-v2

FROM ${BASE_IMAGE}

ARG ROS_DISTRO=humble

# Метаданные
LABEL org.opencontainers.image.title="Voice Base Image"
LABEL org.opencontainers.image.description="Base image with Python audio/ML dependencies for Voice Assistant"
LABEL org.opencontainers.image.source="https://github.com/krikz/rob_box_project"

# ============================================================================
# Системные зависимости (аудио, USB, ROS2)
# ============================================================================
# ВАЖНО: DEBIAN_FRONTEND=noninteractive обязателен — jackd2 (зависимость
# supercollider-language) в postinst задаёт вопросы через debconf и может
# подвесить сборку без TTY.
RUN apt-get update && apt-get install -y debconf-utils \
    && echo "jackd2 jackd/tweak_rt_limits boolean false" | debconf-set-selections \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y \
    python3-pip \
    python3-dev \
    libasound2-dev \
    portaudio19-dev \
    libportaudio2 \
    libportaudiocpp0 \
    ffmpeg \
    alsa-utils \
    pulseaudio \
    libusb-1.0-0 \
    libusb-1.0-0-dev \
    udev \
    git \
    wget \
    curl \
    unzip \
    usbutils \
    supercollider-language \
    sc3-plugins-language \
    ros-dev-tools \
    ros-${ROS_DISTRO}-std-msgs \
    ros-${ROS_DISTRO}-std-srvs \
    ros-${ROS_DISTRO}-sensor-msgs \
    ros-${ROS_DISTRO}-geometry-msgs \
    ros-${ROS_DISTRO}-nav-msgs \
    ros-${ROS_DISTRO}-nav2-msgs \
    ros-${ROS_DISTRO}-rtabmap-msgs \
    ros-${ROS_DISTRO}-ament-cmake \
    ros-${ROS_DISTRO}-rosidl-default-generators \
    ros-${ROS_DISTRO}-rosidl-default-runtime \
    && apt-get install -y --reinstall \
    ros-${ROS_DISTRO}-builtin-interfaces \
    ros-${ROS_DISTRO}-rcutils \
    ros-${ROS_DISTRO}-rosidl-runtime-c \
    ros-${ROS_DISTRO}-rosidl-runtime-cpp \
    ros-${ROS_DISTRO}-rosidl-typesupport-c \
    ros-${ROS_DISTRO}-rosidl-typesupport-cpp \
    ros-${ROS_DISTRO}-rosidl-typesupport-fastrtps-c \
    ros-${ROS_DISTRO}-rosidl-typesupport-fastrtps-cpp \
    && rm -rf /var/lib/apt/lists/*

# ============================================================================
# Python зависимости
# ============================================================================
COPY requirements.txt /tmp/requirements.txt

RUN --mount=type=cache,target=/root/.cache,sharing=locked \
    pip3 install --no-cache-dir --upgrade pip "setuptools>=59.6.0,<68" wheel && \
    pip3 install -r /tmp/requirements.txt && \
    rm /tmp/requirements.txt

# ============================================================================
# ReSpeaker драйверы (usb_4_mic_array и pixel_ring)
# ============================================================================
RUN cd /tmp && \
    git clone --depth 1 https://github.com/respeaker/usb_4_mic_array.git && \
    cd usb_4_mic_array && \
    pip3 install --no-cache-dir -r requirements.txt && \
    cp tuning.py /usr/local/bin/ && \
    cp dfu.py /usr/local/bin/ && \
    chmod +x /usr/local/bin/tuning.py /usr/local/bin/dfu.py && \
    cd /tmp && \
    git clone --depth 1 https://github.com/respeaker/pixel_ring.git && \
    cd pixel_ring && \
    python3 setup.py install && \
    cd / && rm -rf /tmp/usb_4_mic_array /tmp/pixel_ring

# ============================================================================
# udev правила для ReSpeaker USB device
# ============================================================================
RUN echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="2886", ATTR{idProduct}=="0018", MODE="0666", GROUP="audio"' \
    > /etc/udev/rules.d/60-respeaker.rules

# ============================================================================
# Скачивание STT/TTS моделей (offline inference)
# ============================================================================
RUN --mount=type=cache,target=/root/.cache,sharing=locked \
    mkdir -p /models /root/.cache/models /models/silero && \
    VOSK_MODEL_FILE="vosk-model-small-ru-0.22.zip" && \
    VOSK_MODEL_URL="https://alphacephei.com/vosk/models/vosk-model-small-ru-0.22.zip" && \
    SILERO_MODEL_FILE="v5_ru.pt" && \
    SILERO_MODEL_URL="https://models.silero.ai/models/tts/ru/v5_ru.pt" && \
    if [ -f "/root/.cache/models/${VOSK_MODEL_FILE}" ]; then \
        echo "✅ Using cached Vosk model"; \
        cp "/root/.cache/models/${VOSK_MODEL_FILE}" /tmp/vosk-model.zip; \
    else \
        echo "⬇️ Downloading Vosk model (45 MB)..."; \
        wget -q -O /tmp/vosk-model.zip "${VOSK_MODEL_URL}" && \
        cp /tmp/vosk-model.zip "/root/.cache/models/${VOSK_MODEL_FILE}"; \
    fi && \
    unzip -q /tmp/vosk-model.zip -d /models/ && \
    mv /models/vosk-model-small-ru-0.22 /models/vosk-model-ru && \
    rm /tmp/vosk-model.zip && \
    if [ -f "/root/.cache/models/${SILERO_MODEL_FILE}" ]; then \
        echo "✅ Using cached Silero model"; \
        cp "/root/.cache/models/${SILERO_MODEL_FILE}" /models/silero/v5_ru.pt; \
    else \
        echo "⬇️ Downloading Silero v5 model (68 MB)..."; \
        wget -q -O /models/silero/v5_ru.pt "${SILERO_MODEL_URL}" && \
        cp /models/silero/v5_ru.pt "/root/.cache/models/${SILERO_MODEL_FILE}"; \
    fi && \
    echo "⬇️ Pre-downloading Silero repository for torch.hub..." && \
    python3 -c "import torch; torch.hub.load(repo_or_dir='snakers4/silero-models', model='silero_tts', language='ru', speaker='v5_ru', source='github')" || true && \
    echo "✅ STT/TTS models ready: Vosk (45 MB) + Silero v5 (68 MB)"

# ============================================================================
# Клонирование и сборка audio_common_msgs
# ============================================================================
RUN mkdir -p /ws/src && \
    cd /ws/src && \
    git clone -b ros2 --depth 1 https://github.com/ros-drivers/audio_common.git && \
    cd audio_common && \
    find . -maxdepth 1 -type d ! -name '.' ! -name '.git' ! -name 'audio_common_msgs' -exec rm -rf {} + && \
    echo "✅ audio_common_msgs prepared for building"

RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    rosdep update && \
    rosdep install --from-paths /ws/src/audio_common/audio_common_msgs --ignore-src -r -y || true

RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    cd /ws && \
    colcon build \
    --packages-select audio_common_msgs \
    --cmake-args \
      -DCMAKE_BUILD_TYPE=Release

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    echo "source /ws/install/setup.bash" >> /root/.bashrc

ENV AUDIODEV=hw:ArrayUAC10,0
ENV ALSA_CARD=ArrayUAC10
ENV SOUND_PACK_DIR=/ws/sound_pack
ENV TTS_CACHE_DIR=/config/voice_assistant/tts_cache

WORKDIR /ws

RUN echo "Voice Base Image ${BUILD_VERSION}" > /voice_base_version.txt
